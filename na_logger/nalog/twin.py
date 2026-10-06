"""MuJoCo digital twin: the user's model (model/torque_scene.xml) driven live by the logger.

  * robot  : real follower joint angles (joint map from alignid)
  * drone  : Crazyflie placed at the alignid mount (nominal model mount if no calibration yet)
  * arrows : at the IMU  red = real acceleration (from the Crazyflie), blue = model (MuJoCo, from
             joint motion); at the TCP green = real acceleration converted to the gripper point.
             All gravity-removed, base frame, 2 cm per m/s^2.
  * text   : joint angles, real vs model IMU (specific force, gyro), model joint torques
  * plots  : specific force x/y/z and gyro x/y/z, real (solid) vs model (light)

Run (macOS needs mjpython):  ./twin.sh  [--url http://<logger-ip>:8080]
Headless check:              .venv/bin/python -m nalog.twin --snapshot twin.png
"""
from __future__ import annotations

import argparse
import asyncio
import json
import threading
import time
from collections import deque

import mujoco
import numpy as np

from .accel import G, G_BASE, imu_to_point
from .kin import MODEL_DIR, POINTS, JointMap, Kin

SCENE = MODEL_DIR / "torque_scene.xml"
ARROW_SCALE = 0.02          # m per m/s^2
WIN = 0.1                   # half window [s] for joint velocity/acceleration fits


class Feed:
    """Background WebSocket client: latest arm/IMU history from the logger."""

    def __init__(self, url: str):
        self.url = url.rstrip("/")
        self.lock = threading.Lock()
        self.arm: deque = deque(maxlen=400)      # (t, pos_deg[6])
        self.imu: deque = deque(maxlen=400)      # (t, acc_g[3], gyro_dps[3])
        self.calib: dict | None = None
        self.calib_rev = 0
        self.connected = False
        self.status: dict = {}
        threading.Thread(target=lambda: asyncio.run(self._run()), daemon=True).start()

    async def _fetch_calib(self, session):
        try:
            async with session.get(f"{self.url}/api/align/current.json") as r:
                c = await r.json()
            with self.lock:
                self.calib = c or None
                self.calib_rev += 1
        except Exception:  # noqa: BLE001
            pass

    async def _run(self):
        import aiohttp
        while True:
            try:
                async with aiohttp.ClientSession() as s:
                    await self._fetch_calib(s)
                    async with s.ws_connect(f"{self.url}/ws", heartbeat=15) as ws:
                        self.connected = True
                        poll = asyncio.create_task(self._poll_calib(s))
                        try:
                            async for msg in ws:
                                if msg.type != aiohttp.WSMsgType.TEXT:
                                    continue
                                d = json.loads(msg.data)
                                t = d.get("type")
                                if t == "data":
                                    self._on_data(d)
                                elif t == "status":
                                    self.status = d
                                elif t == "align_done":
                                    await self._fetch_calib(s)
                        finally:
                            poll.cancel()
            except Exception:  # noqa: BLE001
                pass
            self.connected = False
            await asyncio.sleep(1.0)

    async def _poll_calib(self, session):
        """Pick up calibrations written outside an alignid run (e.g. an offline re-solve)."""
        while True:
            await asyncio.sleep(3.0)
            try:
                async with session.get(f"{self.url}/api/align/current.json") as r:
                    c = await r.json()
                with self.lock:
                    if (c or {}).get("id") != (self.calib or {}).get("id"):
                        self.calib = c or None
                        self.calib_rev += 1
            except Exception:  # noqa: BLE001
                return

    def _on_data(self, d: dict):
        with self.lock:
            if d.get("reset"):
                self.arm.clear()
                self.imu.clear()
            a = d.get("arm")
            if a:
                for t, p in zip(a["t"], a["pos"]):
                    if all(v is not None for v in p):
                        self.arm.append((t, p))
            m = d.get("imu")
            if m:
                for t, acc, gyr in zip(m["t"], m["acc"], m["gyro"]):
                    if all(v is not None for v in acc + gyr):
                        self.imu.append((t, acc, gyr))

    def snapshot(self):
        with self.lock:
            return list(self.arm), list(self.imu), self.calib, self.calib_rev


def _fit(t, y, tc):
    """Quadratic least squares around tc -> value, first and second derivative."""
    x = t - tc
    A = np.c_[np.ones_like(x), x, x * x]
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return c[0], c[1], 2 * c[2]


class Twin:
    def __init__(self, feed: Feed):
        self.feed = feed
        self.kin = Kin(path=SCENE)
        self.m = self.kin.m
        # kinematic replay: no contacts, and no joint-limit forces when the real arm rests past
        # the model's limits (otherwise inverse dynamics reports huge constraint torques)
        self.m.opt.disableflags |= (mujoco.mjtDisableBit.mjDSBL_CONTACT
                                    | mujoco.mjtDisableBit.mjDSBL_LIMIT)
        self.d = mujoco.MjData(self.m)
        self.calib = None
        self.calib_rev = -1
        self.point_site = POINTS["payload"]
        self.pt_site = self.m.site(self.point_site).id
        self.imu_site = self.m.site("imu").id
        # show only the IMU and TCP frames (group 4), small
        for name in ("crazyflie_mount", "crazyflie_frame", "drone_com"):
            self.m.site_group[self.m.site(name).id] = 5
        self.m.vis.scale.framelength *= 0.35
        self.m.vis.scale.framewidth *= 0.35
        self.hist = {k: deque(maxlen=300) for k in ("f_real", "f_model", "w_real", "w_model")}
        self.info: dict = {}

    def _apply_calib(self, calib):
        self.calib = calib
        if calib:
            self.kin.jmap = JointMap(**calib["joint_map"])
            self.kin.set_imu_mount(calib["R_gi"], calib["p_gi"])
        else:
            self.kin.jmap = JointMap()
            R, p = self.kin.nominal_imu()
            self.kin.set_imu_mount(R, p)

    def step(self) -> bool:
        """Update self.d (pose for display) and the comparison values. False if no data."""
        arm, imu, calib, rev = self.feed.snapshot()
        if rev != self.calib_rev:
            self._apply_calib(calib)
            self.calib_rev = rev
        if len(arm) < 25:
            return False
        t = np.array([a[0] for a in arm])
        q = np.array([a[1] for a in arm], float)
        tc = t[-1] - WIN                          # centre of the fit window (physical time)
        w = (t >= tc - WIN) & (t <= t[-1])
        if w.sum() < 8:
            return False
        q0, q1, q2 = _fit(t[w], q[w], tc)
        jm = self.kin.jmap
        sgn = np.radians(np.asarray(jm.sign))
        d = self.d
        # model sensors at tc from joint motion (inverse dynamics also gives joint torques)
        d.qpos[:] = jm.qpos(q0)
        d.qvel[:] = sgn * q1
        d.qacc[:] = sgn * q2
        mujoco.mj_inverse(self.m, d)
        f_model = d.sensor("imu_accelerometer").data.copy()
        w_model = d.sensor("imu_gyro").data.copy()
        torque = d.qfrc_inverse.copy()
        R_bi = d.site_xmat[self.imu_site].reshape(3, 3).copy()
        R_bg = d.xmat[self.kin.gripper_id].reshape(3, 3).copy()
        imu_pos = d.site_xpos[self.imu_site].copy()
        pt_pos = d.site_xpos[self.pt_site].copy()
        a_model = R_bi @ f_model + G_BASE

        f_real = w_real = a_real = a_pt = None
        if len(imu) > 30:
            ti = np.array([x[0] for x in imu])
            acc = np.array([x[1] for x in imu])
            gyr = np.array([x[2] for x in imu])
            c = self.calib or {}
            lag = c.get("lag_ms", 0.0) / 1000.0
            alag = lag + c.get("acc_extra_lag_ms", 0.0) / 1000.0
            if ti[0] <= tc + lag and tc + alag <= ti[-1]:
                ba = np.asarray(c.get("acc_bias_g", [0, 0, 0]))
                bg = np.asarray(c.get("gyro_bias_dps", [0, 0, 0]))
                f_real = np.array([np.interp(tc + alag, ti, acc[:, k]) for k in range(3)])
                f_real = (f_real - ba) / np.asarray(c.get("acc_scale", [1, 1, 1])) * G
                w_real = np.radians([np.interp(tc + lag, ti, gyr[:, k]) for k in range(3)] - bg)
                a_real = R_bi @ f_real + G_BASE
                if c:
                    r = self.kin.site_in_gripper(self.point_site) - np.asarray(c["p_gi"])
                    tg, _, _, fp = imu_to_point(ti, gyr, acc, c, r)
                    if len(tg):
                        k = int(np.clip(np.searchsorted(tg, tc), 0, len(tg) - 1))
                        a_pt = R_bg @ fp[k] + G_BASE
        for key, v in (("f_real", f_real), ("f_model", f_model), ("w_real", w_real),
                       ("w_model", w_model)):
            self.hist[key].append(v if v is not None else np.full(3, np.nan))
        self.info = dict(q=q0, f_model=f_model, w_model=w_model, f_real=f_real, w_real=w_real,
                         a_model=a_model, a_real=a_real, a_pt=a_pt, torque=torque,
                         imu_pos=imu_pos, pt_pos=pt_pos, n_imu=len(imu))
        # display the latest pose (not the fit centre) so the twin lags as little as possible
        d.qpos[:] = jm.qpos(q[-1])
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)
        return True

    # ------------------------------------------------------------ drawing
    def draw_arrows(self, scn):
        scn.ngeom = 0
        i = self.info
        if not i:
            return
        for start, vec, rgba in ((i["imu_pos"], i["a_real"], (0.9, 0.15, 0.15, 1)),
                                 (i["imu_pos"], i["a_model"], (0.2, 0.4, 1.0, 0.8)),
                                 (i["pt_pos"], i["a_pt"], (0.1, 0.8, 0.3, 1))):
            if vec is None or scn.ngeom >= scn.maxgeom:
                continue
            end = start + ARROW_SCALE * vec
            if np.linalg.norm(end - start) < 1e-4:
                continue
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                                np.zeros(9), np.asarray(rgba, np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.004, start, end)
            scn.ngeom += 1

    def texts(self) -> list[tuple]:
        """Overlay text blocks. MuJoCo's overlay font is ASCII-only, so no Korean here."""
        i, c = self.info, self.calib
        st = self.feed.status or {}
        arm_hz = st.get("arm", {}).get("rate_hz", 0)
        imu_hz = st.get("cf", {}).get("rate_hz", 0)
        status = [
            ("logger", "connected" if self.feed.connected else "DISCONNECTED - start the logger"),
            ("rate", f"arm {arm_hz:.0f} Hz   imu {imu_hz:.0f} Hz"),
            ("align", (f"{c['id']}  ({c.get('quality')}, {c.get('angle_from_nominal_deg', 0):.1f} deg from model)"
                       if c else "none - model nominal mount")),
            ("arrows", "red imu real | blue imu model | green payload (real)"),
        ]
        blocks = [(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                   "\n".join(a for a, _ in status), "\n".join(b for _, b in status))]
        if i:
            v3 = lambda v: "   -" if v is None else " ".join(f"{x:7.2f}" for x in v)  # noqa: E731
            rows = [
                ("joint [deg]", " ".join(f"{x:7.1f}" for x in i["q"])),
                ("torque [Nm]", " ".join(f"{x:7.2f}" for x in i["torque"])),
                ("f real [m/s2]", v3(i["f_real"])),
                ("f model [m/s2]", v3(i["f_model"])),
                ("gyro real [rad/s]", v3(i["w_real"])),
                ("gyro model [rad/s]", v3(i["w_model"])),
                ("a payload [m/s2]", v3(i["a_pt"])),
            ]
            blocks.append((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                           "\n".join(a for a, _ in rows), "\n".join(b for _, b in rows)))
        return blocks

    def figures(self, viewport):
        figs = []
        W, H = viewport.width, viewport.height
        w, h = int(W * 0.32), int(H * 0.28)
        colors = [(0.9, 0.3, 0.3), (0.3, 0.8, 0.4), (0.35, 0.55, 1.0)]
        for n, (title, real, model) in enumerate((("specific force [m/s^2]", "f_real", "f_model"),
                                                  ("gyro [rad/s]", "w_real", "w_model"))):
            fig = mujoco.MjvFigure()
            mujoco.mjv_defaultFigure(fig)
            fig.title = title
            fig.flg_extend = 1
            fig.figurergba[3] = 0.6
            fig.linename[0] = "x real"
            r = np.array(self.hist[real]) if self.hist[real] else np.zeros((0, 3))
            mo = np.array(self.hist[model]) if self.hist[model] else np.zeros((0, 3))
            npt = len(r)
            for k in range(3):
                for line, arr, alpha in ((k, r, 1.0), (k + 3, mo, 0.45)):
                    fig.linergb[line] = [c * alpha + (1 - alpha) for c in colors[k]]
                    fig.linepnt[line] = npt
                    for p in range(npt):
                        v = arr[p, k] if np.isfinite(arr[p, k]) else 0.0
                        fig.linedata[line][2 * p] = p - npt
                        fig.linedata[line][2 * p + 1] = v
            rect = mujoco.MjrRect(W - w - 10, 10 + n * (h + 10), w, h)
            figs.append((rect, fig))
        return figs


def run_viewer(url: str, seconds: float | None):
    import mujoco.viewer
    feed = Feed(url)
    twin = Twin(feed)
    with mujoco.viewer.launch_passive(twin.m, twin.d, show_left_ui=False,
                                      show_right_ui=False) as v:
        v.opt.sitegroup[4] = 1
        v.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        v.cam.lookat[:] = [0.15, 0.0, 0.15]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 0.75, 140, -22
        t0 = time.monotonic()
        while v.is_running() and (seconds is None or time.monotonic() - t0 < seconds):
            with v.lock():
                twin.step()
                twin.draw_arrows(v.user_scn)
            v.set_texts(twin.texts())
            v.set_figures(twin.figures(v.viewport))
            v.sync()
            time.sleep(1 / 60)


def snapshot(url: str, path: str, wait: float = 4.0, size=(1280, 800)):
    """Headless: render one frame (scene + overlay text + figures) to a PNG."""
    feed = Feed(url)
    twin = Twin(feed)
    t = time.monotonic()
    ok = False
    while time.monotonic() - t < wait + 6:
        ok = twin.step()
        if ok and time.monotonic() - t > wait:
            break
        time.sleep(0.05)
    W, H = size
    gl = mujoco.GLContext(W, H)
    gl.make_current()
    m = twin.m
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    con = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_100)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, con)
    scn = mujoco.MjvScene(m, maxgeom=2000)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.15, 0.0, 0.15]
    cam.distance, cam.azimuth, cam.elevation = 0.75, 140, -22
    opt = mujoco.MjvOption()
    opt.sitegroup[4] = 1
    opt.frame = mujoco.mjtFrame.mjFRAME_SITE
    mujoco.mjv_updateScene(m, twin.d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    extra = mujoco.MjvScene(m, maxgeom=10)
    twin.draw_arrows(extra)
    for k in range(extra.ngeom):
        g = extra.geoms[k]
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], g.type, g.size, g.pos, g.mat.reshape(9), g.rgba)
        scn.ngeom += 1
    vp = mujoco.MjrRect(0, 0, W, H)
    mujoco.mjr_render(vp, scn, con)
    for font, grid, a, b in twin.texts():
        mujoco.mjr_overlay(font, grid, vp, a, b, con)
    for rect, fig in twin.figures(vp):
        mujoco.mjr_figure(rect, fig, con)
    rgb = np.zeros((H, W, 3), np.uint8)
    mujoco.mjr_readPixels(rgb, None, vp, con)
    _save_png(path, np.flipud(rgb))
    print(f"data ok={ok}, arrows={extra.ngeom}")
    for font, grid, a, b in twin.texts():
        for x, y in zip(a.split("\n"), b.split("\n")):
            print(f"  {x:20s} {y}")


def _save_png(path: str, img: np.ndarray) -> None:
    import struct
    import zlib
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) \
        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def main():
    ap = argparse.ArgumentParser(prog="nalog.twin")
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--seconds", type=float, help="close the viewer after this long")
    ap.add_argument("--snapshot", help="render one frame to this PNG instead of opening a window")
    a = ap.parse_args()
    if a.snapshot:
        snapshot(a.url, a.snapshot)
    else:
        run_viewer(a.url, a.seconds)


if __name__ == "__main__":
    main()
