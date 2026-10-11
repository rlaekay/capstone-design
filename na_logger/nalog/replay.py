"""Replay pre-planned joint trajectories (Artemis/1 motion bank) on the follower and record them.

Each motion folder holds reference.csv (home -> 12 s training clip -> home, model qpos in rad,
100 Hz) and config.json (training_window_s = the clip's crop in reference.csv time).
A run = one trial: slow approach from the present pose to the first reference sample, a short
settle, the full reference, a short hold. replay.json in the trial dir tells where each part is.
"""
from __future__ import annotations

import csv
import json
import threading
import time
from pathlib import Path

import numpy as np

from .alignid import real_limits
from .clock import now_ns

LIBRARY = Path(__file__).resolve().parents[2] / "1"
SETS = ["excitation_v2_set_1", "excitation_v2_set_2", "excitation_v2_set_3", "paper_timewarp_10"]
APPROACH_DPS = 25.0
SETTLE_S = 1.5
HOLD_S = 1.0


def list_motions() -> list[str]:
    out = []
    for s in SETS:
        for m in sorted((LIBRARY / s).glob("motion_*")):
            if (m / "reference.csv").exists():
                out.append(f"{s}/{m.name}")
    return out


def load_reference(motion: str):
    d = LIBRARY / motion
    with open(d / "reference.csv") as f:
        rows = list(csv.reader(f))
    a = np.array(rows[1:], float)
    cfg = json.loads((d / "config.json").read_text()) if (d / "config.json").exists() else {}
    return a[:, 0], a[:, 1:7], cfg.get("training_window_s")


class Replay:
    """Goal function in logger degrees: approach -> settle -> reference -> hold."""

    def __init__(self, q_now_deg, t_ref, q_ref_deg, grip_deg: float | None = None):
        self.q_now = np.asarray(q_now_deg, float)
        self.t_ref, self.q_ref = t_ref - t_ref[0], np.array(q_ref_deg, float)
        # gripper held where it was at the start, or squeezed a little further (grip_deg)
        self.grip = float(self.q_now[5] if grip_deg is None else grip_deg)
        self.q_ref[:, 5] = self.grip
        dist = np.abs(self.q_ref[0] - self.q_now).max()
        self.t_app = max(2.0, dist / APPROACH_DPS)
        self.t_ref0 = self.t_app + SETTLE_S
        self.duration = self.t_ref0 + self.t_ref[-1] + HOLD_S

    def goal(self, t: float) -> list[float]:
        if t < self.t_app:
            s = t / self.t_app
            s = s * s * (3 - 2 * s)
            return list(self.q_now + s * (self.q_ref[0] - self.q_now))
        tau = t - self.t_ref0
        if tau <= 0:
            return list(self.q_ref[0])
        return [float(np.interp(tau, self.t_ref, self.q_ref[:, j])) for j in range(6)]


def check(motion: str, kin, q_now_deg=None, grip_deg: float | None = None) -> list[str]:
    t, q, _ = load_reference(motion)
    deg = np.array([kin.jmap.deg(r) for r in q])
    if q_now_deg is not None:
        deg[:, 5] = q_now_deg[5] if grip_deg is None else grip_deg
    problems = []
    lim = real_limits(kin, margin_deg=1.0)
    bad = (deg < lim[:, 0]) | (deg > lim[:, 1])
    bad[:, 5] = False                                # gripper held fixed, its range is EEPROM's
    if bad.any():
        j = int(np.argwhere(bad)[0][1])
        problems.append(f"{motion}: 관절 {j + 1} 한계 초과 ({bad.sum()} 샘플)")
    v = np.abs(np.diff(deg, axis=0)) / np.diff(t)[:, None]
    if v.max() > 200:
        problems.append(f"{motion}: 속도 {v.max():.0f}°/s")
    rows, hits = deg, []
    if q_now_deg is not None:
        rp = Replay(q_now_deg, t, deg, grip_deg)
        # the approach leaves the present pose: contacts the model already sees there (e.g. the
        # arm resting folded) are the real arm's state, not a planned collision
        start = {frozenset((h["geom1"], h["geom2"])) for h in kin.collisions([rp.goal(0.0)])}
        for x in np.arange(0, rp.t_app + SETTLE_S, 0.04):
            hits += [h for h in kin.collisions([rp.goal(x)])
                     if frozenset((h["geom1"], h["geom2"])) not in start]
            if hits:
                break
        rows = deg
    hits = hits or kin.collisions(rows, stride=2)
    if hits:
        h = hits[0]
        problems.append(f"{motion}: 충돌 예상 {h['geom1']} ↔ {h['geom2']} ({h['dist_mm']} mm)")
    return problems


class ReplayRunner:
    def __init__(self, hub, kin_factory):
        self.hub, self.kin_factory = hub, kin_factory
        self.state, self.motion, self.message, self.progress = "idle", None, "", 0.0
        self.queue: list[str] = []
        self.done: list[dict] = []
        self._abort = threading.Event()
        self._thread = None

    def status(self) -> dict:
        return {"state": self.state, "motion": self.motion, "message": self.message,
                "progress": self.progress, "queue": list(self.queue), "done": list(self.done)}

    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, motions: list[str], tag: str = "", payload_g: float | None = None,
              grip_squeeze_deg: float = 0.0) -> dict:
        if self.busy():
            return {"ok": False, "problems": ["이미 실행 중"]}
        known = set(list_motions())
        unknown = [m for m in motions if m not in known]
        if unknown or not motions:
            return {"ok": False, "problems": [f"모르는 모션: {unknown}"]}
        self._abort.clear()
        self.tag = tag
        self.grip_squeeze = float(grip_squeeze_deg)   # degrees past the start value, closing
        self.payload_g = payload_g        # not None -> each trial is also exported to data/loaded/
        self.queue = list(motions)
        self._thread = threading.Thread(target=self._run, name="replay", daemon=True)
        self._thread.start()
        return {"ok": True, "n": len(motions)}

    def abort(self):
        self._abort.set()
        self.queue = []
        if self.hub.arm_src:
            self.hub.arm_src.command("hold")

    def _run(self):
        while self.queue and not self._abort.is_set():
            m = self.queue.pop(0)
            self.motion = m
            try:
                tid = self._one(m)
                entry = {"motion": m, "trial": tid, "ok": True}
                self.done.append(entry)
                if self.payload_g is not None:
                    threading.Thread(target=self._export, args=(tid, self.payload_g, entry),
                                     name="loaded-export", daemon=True).start()
            except Exception as e:
                self.state, self.message = "error", f"{m}: {e}"
                self.done.append({"motion": m, "ok": False, "error": str(e)})
                self.queue = []
                if self.hub.arm_src:
                    self.hub.arm_src.command("hold")
                self.hub.stop_trial()
                return
        self.state = "aborted" if self._abort.is_set() else "idle"
        self.message = "중단됨" if self._abort.is_set() else f"완료 {len(self.done)}개"

    def _export(self, tid: str, mass_g: float, entry: dict):
        from .loaded import export
        try:
            out = export(Path(self.hub.cfg.data_dir) / tid, Path(self.hub.cfg.data_dir), mass_g)
            entry["export"] = str(out)
        except Exception as e:  # noqa: BLE001
            entry["export_error"] = str(e)

    def _one(self, motion: str) -> str:
        hub, arm = self.hub, self.hub.arm_src
        self.state, self.progress, self.message = "checking", 0.0, motion
        st = hub.status()
        if not st["arm"].get("follower", {}).get("connected"):
            raise RuntimeError("팔로워 미연결")
        if st["cf"].get("state") != "connected":
            raise RuntimeError("Crazyflie 미연결")
        s = hub.last_arm
        if s is None or any(v is None for v in s["pos_deg"][:6]):
            raise RuntimeError("팔로워 관절값 없음")
        q_now = np.array(s["pos_deg"], float)
        kin = self.kin_factory()
        # gripper closes toward smaller angles; squeeze = hold a bit past where it stopped
        grip = float(q_now[5] - getattr(self, "grip_squeeze", 0.0))
        problems = check(motion, kin, q_now, grip)
        if problems:
            raise RuntimeError("; ".join(problems))
        t, q, win = load_reference(motion)
        deg = np.array([kin.jmap.deg(r) for r in q])
        rp = Replay(q_now, t, deg, grip)

        self.state = "running"
        hub.stop_trial()
        tag = getattr(self, "tag", "")
        tid = hub.start_trial(label=f"replay {motion}" + (f" [{tag}]" if tag else ""))
        arm.start_script(rp.goal, rp.duration)
        t_start = time.monotonic()
        script_t0 = None
        hot_since = None
        while True:
            if self._abort.is_set():
                raise RuntimeError("사용자 중단")
            ast = arm.status()
            script_t0 = script_t0 or ast.get("script_t0_ns")
            el = time.monotonic() - t_start
            self.progress = min(1.0, el / rp.duration)
            if ast.get("teleop") != "script" and el > 1.0:
                if el < rp.duration - 0.5:
                    raise RuntimeError(f"팔 동작 중단: {ast.get('message', '')}")
                break
            if not ast.get("follower", {}).get("connected"):
                raise RuntimeError("팔로워 연결 끊김")
            # single corrupted temperature reads (seen on ID 5) are ignored: real heating is slow,
            # so only a reading that stays >= 69 C for 8 s stops the run
            temps = [x for x in (hub.status().get("joints") or {}).get("temp", []) if x is not None]
            if temps and max(temps) >= 69:
                hot_since = hot_since or time.monotonic()
                if time.monotonic() - hot_since > 8.0:
                    raise RuntimeError(f"서보 온도 {max(temps)} °C (8초 이상)")
            else:
                hot_since = None
            time.sleep(0.05)
        time.sleep(0.3)
        hub.stop_trial()
        info = {"motion": motion, "source": str(LIBRARY / motion / "reference.csv"),
                "script_t0_ns": script_t0, "approach_s": [0.0, rp.t_app],
                "reference_start_s": rp.t_ref0, "reference_end_s": rp.t_ref0 + float(rp.t_ref[-1]),
                "training_window_s": ([rp.t_ref0 + win[0], rp.t_ref0 + win[1]] if win else None),
                "note": "times are seconds after script_t0_ns (Mac monotonic clock)",
                "q_start_deg": q_now.tolist(), "gripper_fixed_deg": rp.grip, "gripper_start_deg": float(q_now[5]),
                "grip_squeeze_deg": float(q_now[5]) - rp.grip,
                "scene": "table behind base, open in front (model/scene.xml)"}
        (Path(hub.cfg.data_dir) / tid / "replay.json").write_text(json.dumps(info, indent=1))
        return tid
