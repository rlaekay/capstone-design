"""aiohttp web server: live WebSocket stream, trial list/downloads, controls."""
from __future__ import annotations

import asyncio
import io
import json
import logging
import socket
import subprocess
import zipfile
from pathlib import Path

from aiohttp import WSMsgType, web

from .recorder import TRIAL_RE, Hub
from . import alignid
from .kin import JointMap, kin_from_calib

log = logging.getLogger(__name__)
STATIC = Path(__file__).parent / "static"
RAW_FILES = ["arm.csv", "imu.csv", "att.csv", "merged.csv", "meta.json"]


def lan_addresses() -> list[str]:
    ips: set[str] = set()
    for iface in ("en0", "en1", "en2"):
        try:
            ip = subprocess.run(["ipconfig", "getifaddr", iface], capture_output=True, text=True,
                                timeout=1).stdout.strip()
            if ip:
                ips.add(ip)
        except Exception:  # noqa: BLE001
            pass
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            ips.add(s.getsockname()[0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


def mdns_name() -> str | None:
    try:
        name = subprocess.run(["scutil", "--get", "LocalHostName"], capture_output=True, text=True,
                              timeout=1).stdout.strip()
        return f"{name}.local" if name else None
    except Exception:  # noqa: BLE001
        return None


def urls(port: int) -> list[str]:
    out = [f"http://{ip}:{port}" for ip in lan_addresses()]
    name = mdns_name()
    if name:
        out.append(f"http://{name}:{port}")
    return out


def make_app(hub: Hub) -> web.Application:
    cfg = hub.cfg
    app = web.Application()
    app["hub"] = hub
    app["clients"] = set()
    app["urls"] = urls(cfg.port)
    local_ips = {"127.0.0.1", "::1", *lan_addresses()}

    # ---- model, live kinematics, alignid
    def current_calib():
        return alignid.load_current(Path(cfg.data_dir))

    def kin_factory(nominal: bool = False):
        c = current_calib()
        if nominal:                      # current joint map, model's nominal Crazyflie mount
            k = kin_from_calib(None)
            if c and c.get("joint_map"):
                k.jmap = JointMap(**c["joint_map"])
            return k
        return kin_from_calib(c)

    app["calib"] = current_calib()
    runner = alignid.AlignRunner(hub, kin_factory)
    app["runner"] = runner
    from . import replay
    replayer = replay.ReplayRunner(hub, kin_factory)
    app["replayer"] = replayer

    def asset_version() -> int:
        return max(int(f.stat().st_mtime_ns // 1_000_000) for f in STATIC.glob("*.*"))

    app["asset_version"] = asset_version()

    def calib_summary():
        c = app["calib"] = current_calib()     # re-read: offline re-solves show up immediately
        return alignid._summary(c) if c else None

    def control_ok(request: web.Request) -> bool:
        return cfg.allow_remote_control or request.remote in local_ips

    def trial_dir(request: web.Request) -> Path:
        tid = request.match_info["tid"]
        if not TRIAL_RE.match(tid):
            raise web.HTTPNotFound()
        d = Path(cfg.data_dir) / tid
        if not d.is_dir():
            raise web.HTTPNotFound()
        return d

    def attachment(name: str) -> dict:
        return {"Content-Disposition": f'attachment; filename="{name}"'}

    # ------------------------------------------------------------ pages
    async def index(_):
        # version static assets by mtime so browsers never run a stale app.js after an update
        html = (STATIC / "index.html").read_text()
        for name in ("app.js", "style.css"):
            v = int((STATIC / name).stat().st_mtime)
            html = html.replace(f"/static/{name}", f"/static/{name}?v={v}")
        return web.Response(text=html, content_type="text/html", headers={"Cache-Control": "no-cache"})

    async def ws_handler(request: web.Request):
        ws = web.WebSocketResponse(heartbeat=15)
        await ws.prepare(request)
        app["clients"].add(ws)
        try:
            await ws.send_json({"type": "hello", "joints": cfg.joint_names,
                                "asset_version": app["asset_version"],
                                "payload_mass_g": cfg.payload_mass_g,
                                "control_allowed": control_ok(request), "urls": app["urls"],
                                "live_window_s": cfg.live_window_s, "sim": hub.arm_src.kind == "sim",
                                "imu_latency_ms": cfg.imu_latency_ms})
            await ws.send_json(full_status())
            await ws.send_json({"type": "trials", "trials": hub.trials.list()})
            await ws.send_str(json.dumps(hub.snapshot(), separators=(",", ":")))
            async for msg in ws:
                if msg.type == WSMsgType.ERROR:
                    break
        finally:
            app["clients"].discard(ws)
        return ws

    # ------------------------------------------------------------ API
    def full_status():
        st = hub.status()
        st["align"] = runner.status()
        st["replay"] = replayer.status()
        st["calib"] = calib_summary()
        return st

    async def api_status(_):
        return web.json_response(full_status())

    async def replay_action(request: web.Request):
        action = request.match_info["action"]
        if action == "list":
            return web.json_response({"motions": replay.list_motions(), **replayer.status()})
        if action == "status":
            return web.json_response(replayer.status())
        if not control_ok(request):
            raise web.HTTPForbidden(text="로봇 제어는 로거가 실행 중인 Mac에서만 가능합니다")
        if action == "start":
            body = await request.json()
            return web.json_response(replayer.start(list(body.get("motions", []))))
        if action == "abort":
            replayer.abort()
            return web.json_response({"ok": True})
        raise web.HTTPNotFound()

    async def align_action(request: web.Request):
        action = request.match_info["action"]
        precise = request.query.get("precise") == "1"
        if action == "check":
            return web.json_response(await asyncio.to_thread(runner.check, precise))
        if not control_ok(request):
            raise web.HTTPForbidden(text="로봇 제어는 로거가 실행 중인 Mac에서만 가능합니다")
        if action == "start":
            return web.json_response(await asyncio.to_thread(runner.start, precise))
        if action == "abort":
            runner.abort()
            return web.json_response({"ok": True})
        raise web.HTTPNotFound()

    async def api_payload(request: web.Request):
        body = await request.json()
        m = body.get("mass_g")
        cfg.payload_mass_g = float(m) if m not in (None, "") and float(m) > 0 else None
        for ws in list(app["clients"]):
            try:
                await ws.send_json({"type": "payload", "mass_g": cfg.payload_mass_g})
            except (ConnectionResetError, RuntimeError):
                pass
        return web.json_response({"mass_g": cfg.payload_mass_g})

    async def align_current(_):
        return web.json_response(current_calib() or {})

    async def api_trials(_):
        return web.json_response(hub.trials.list())

    async def api_record(request: web.Request):
        action = request.match_info["action"]
        if action == "start":
            return web.json_response({"id": hub.start_trial()})
        if action == "stop":
            return web.json_response({"id": hub.stop_trial()})
        raise web.HTTPNotFound()

    async def api_auto(request: web.Request):
        body = await request.json()
        with hub.lock:
            hub.trials.set_auto(bool(body.get("on")))
        return web.json_response({"auto": hub.trials.auto})

    async def api_teleop(request: web.Request):
        if not control_ok(request):
            raise web.HTTPForbidden(text="로봇 제어는 로거가 실행 중인 Mac에서만 가능합니다")
        action = request.match_info["action"]
        if action not in ("start", "hold", "torque_off"):
            raise web.HTTPNotFound()
        hub.arm_src.command(action)
        return web.json_response({"ok": True})

    async def api_label(request: web.Request):
        tid = trial_dir(request).name
        body = await request.json()
        label = str(body.get("label", ""))[:120]
        with hub.lock:
            ok = hub.trials.set_label(tid, label)
        return web.json_response({"ok": ok})

    async def dl_merged(request: web.Request):
        d = trial_dir(request)
        f = d / "merged.csv"
        if not f.exists():
            raise web.HTTPConflict(text="아직 처리 중입니다")
        return web.FileResponse(f, headers=attachment(f"NA_{d.name}.csv"))

    async def dl_preview(request: web.Request):
        f = trial_dir(request) / "preview.json"
        if not f.exists():
            raise web.HTTPConflict(text="아직 처리 중입니다")
        return web.FileResponse(f, headers={"Cache-Control": "no-cache"})

    def zip_bytes(entries: list[tuple[Path, str]]) -> bytes:
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for path, arc in entries:
                z.write(path, arc)
        return buf.getvalue()

    async def dl_raw(request: web.Request):
        d = trial_dir(request)
        entries = [(d / n, f"{d.name}/{n}") for n in RAW_FILES if (d / n).exists()]
        data = await asyncio.to_thread(zip_bytes, entries)
        return web.Response(body=data, content_type="application/zip",
                            headers=attachment(f"NA_{d.name}_raw.zip"))

    async def dl_all(_):
        root = Path(cfg.data_dir)
        entries = [(p, f"NA_{p.parent.name}.csv") for p in sorted(root.glob("T*/merged.csv"))
                   if TRIAL_RE.match(p.parent.name)]
        data = await asyncio.to_thread(zip_bytes, entries)
        return web.Response(body=data, content_type="application/zip",
                            headers=attachment("NA_all_trials.zip"))

    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/api/status", api_status)
    app.router.add_get("/api/trials", api_trials)
    app.router.add_post("/api/record/{action}", api_record)
    app.router.add_post("/api/auto", api_auto)
    app.router.add_post("/api/teleop/{action}", api_teleop)
    app.router.add_post("/api/trials/{tid}/label", api_label)
    app.router.add_get("/api/trials/{tid}/merged.csv", dl_merged)
    app.router.add_get("/api/trials/{tid}/raw.zip", dl_raw)
    app.router.add_get("/api/trials/{tid}/preview.json", dl_preview)
    app.router.add_get("/api/all.zip", dl_all)
    app.router.add_post("/api/align/{action}", align_action)
    app.router.add_route("*", "/api/replay/{action}", replay_action)
    app.router.add_get("/api/align/current.json", align_current)
    app.router.add_post("/api/payload", api_payload)
    app.router.add_static("/static", STATIC)

    # ------------------------------------------------------------ broadcaster
    async def broadcaster(app: web.Application):
        n = 0
        while True:
            await asyncio.sleep(0.05)
            n += 1
            hub.tick()
            data, events, trials_changed = hub.drain()
            msgs = []
            if len(data) > 1:
                msgs.append(json.dumps(data, separators=(",", ":")))
            for ev in events:
                if ev.get("type") == "align_done":
                    app["calib"] = current_calib()
                msgs.append(json.dumps(ev))
            if trials_changed:
                msgs.append(json.dumps({"type": "trials", "trials": hub.trials.list()}))
            if n % 5 == 0 or events:
                msgs.append(json.dumps(full_status(), separators=(",", ":")))
            if n % 20 == 0:                       # GUI files edited -> open pages reload
                v = asset_version()
                if v != app["asset_version"]:
                    app["asset_version"] = v
                    msgs.append(json.dumps({"type": "reload", "asset_version": v}))
            for ws in list(app["clients"]):
                try:
                    for m in msgs:
                        await ws.send_str(m)
                except (ConnectionResetError, RuntimeError):
                    app["clients"].discard(ws)

    async def start_bg(app: web.Application):
        app["bcast"] = asyncio.create_task(broadcaster(app))

    async def stop_bg(app: web.Application):
        app["bcast"].cancel()
        for ws in list(app["clients"]):
            await ws.close()

    app.on_startup.append(start_bg)
    app.on_shutdown.append(stop_bg)
    return app
