"""Hub (live state shared by sources and the web server) and trial recording.

Sources push samples from their own threads; everything below runs under Hub.lock and only
does cheap work. File writing happens on a writer thread, post-processing on a worker.
"""
from __future__ import annotations

import csv
import json
import logging
import queue
import re
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from .clock import T0_NS, UNIX0_NS, now_ns, rel_s, unix_s
from .config import Config
from .finalize import ATT_COLS, IMU_COLS, TELEOP_CODE, arm_header, finalize, ARM_FIELDS
from .sync import MotionDetector, OnlineClock, RateMeter

log = logging.getLogger(__name__)
TRIAL_RE = re.compile(r"^T(\d{4})_\d{8}-\d{6}$")


def _r(v, nd=2):
    return None if v is None or v != v else round(v, nd)


class Trial:
    def __init__(self, tid: str, path: Path, t0_ns: int, auto: bool):
        self.id, self.path, self.t0_ns, self.auto = tid, path, t0_ns, auto
        self.last_motion_ns = t0_ns
        self.n_arm = self.n_imu = self.n_att = 0


class TrialManager:
    def __init__(self, cfg: Config, hub: "Hub"):
        self.cfg = cfg
        self.hub = hub
        self.root = Path(cfg.data_dir)
        self.root.mkdir(parents=True, exist_ok=True)
        self.auto = cfg.auto_trial
        self.cur: Trial | None = None
        self.index: dict[str, dict] = {}
        self.changed = True
        self._quiet_needed = False        # after a manual stop, wait for stillness before auto
        self._meta_lock = threading.Lock()
        self._q: queue.Queue = queue.Queue()
        self._fin = ThreadPoolExecutor(max_workers=1, thread_name_prefix="finalize")
        self._writer = threading.Thread(target=self._write_loop, name="trial-writer", daemon=True)
        self._writer.start()
        self._load_existing()

    # ------------------------------------------------------------ index
    def _load_existing(self):
        for d in sorted(self.root.iterdir()):
            m = d / "meta.json"
            if not (d.is_dir() and TRIAL_RE.match(d.name) and m.exists()):
                continue
            try:
                meta = json.loads(m.read_text())
            except Exception:  # noqa: BLE001
                continue
            if meta.get("state") in ("recording", "processing"):
                meta["state"] = "processing"      # interrupted last time: rebuild merged.csv
                self._fin.submit(self._finalize, d)
            self.index[d.name] = self._summary(meta)

    @staticmethod
    def _summary(meta: dict) -> dict:
        lag = meta.get("imu_lag") or {}
        clock = meta.get("clock_fit") or {}
        return {k: meta.get(k) for k in ("id", "num", "label", "state", "auto", "started_unix",
                                          "payload_mass_g",
                                          "duration_s", "n_arm", "n_imu", "arm_rate_hz",
                                          "imu_rate_hz", "imu_coverage", "max_temp_C", "error")} | {
            "lag_ms": lag.get("lag_ms"), "lag_corr": lag.get("corr"),
            "lag_reliable": lag.get("reliable"),
            "clock_jitter_ms": clock.get("envelope_jitter_ms"), "drift_ppm": clock.get("drift_ppm")}

    def list(self) -> list[dict]:
        return sorted(self.index.values(), key=lambda m: m.get("num") or 0, reverse=True)

    def _new_id(self) -> tuple[str, int]:
        nums = [int(TRIAL_RE.match(k).group(1)) for k in self.index if TRIAL_RE.match(k)]
        num = max(nums, default=0) + 1
        return f"T{num:04d}_{time.strftime('%Y%m%d-%H%M%S')}", num

    # ------------------------------------------------------------ control (hub.lock held)
    def start(self, now: int, reason: str, auto: bool, label: str = "") -> str:
        if self.cur:
            if not auto:
                self.cur.auto = False             # manual start during an auto trial: keep it open
            return self.cur.id
        tid, num = self._new_id()
        path = self.root / tid
        path.mkdir()
        t0 = now - int(self.cfg.pre_roll_s * 1e9)
        hub = self.hub
        meta = {
            "id": tid, "num": num, "label": label, "state": "recording", "auto": auto,
            "reason": reason, "started_unix": unix_s(t0), "t0_ns": t0,
            "clock_anchor": {"unix_ns": UNIX0_NS, "mono_ns": T0_NS},
            "host_clock": "time.monotonic_ns (macOS)",
            "joint_names": self.cfg.joint_names, "servo_ids": self.cfg.servo_ids,
            "calib": hub.arm_src.calib_info() if hub.arm_src else {},
            "arm_status": hub.arm_src.status() if hub.arm_src else None,
            "cf_status": hub.imu_src.status() if hub.imu_src else None,
            "sim": getattr(hub.arm_src, "kind", "") == "sim",
            "config": self.cfg.public(),
            "align_calib": _current_calib(self.root),
            "payload_mass_g": self.cfg.payload_mass_g,
        }
        tr = Trial(tid, path, t0, auto)
        self.cur = tr
        self.index[tid] = self._summary(meta)
        self._q.put(("open", tr, meta))
        for kind, hist, key in (("arm", hub.arm_hist, "t"), ("imu", hub.imu_hist, "rx"),
                                ("att", hub.att_hist, "rx")):
            for s in hist:
                if s[key] >= t0:
                    self._push(kind, tr, s)
        self.changed = True
        hub.event({"type": "trial_started", "id": tid, "auto": auto, "reason": reason})
        log.info("trial %s started (%s)", tid, reason)
        return tid

    def stop(self, now: int, reason: str, manual: bool = False) -> str | None:
        tr = self.cur
        if not tr:
            return None
        self.cur = None
        if manual:
            self._quiet_needed = True
        self.index[tr.id]["state"] = "processing"
        self._q.put(("close", tr, {"stopped_unix": unix_s(now), "stop_reason": reason,
                                   "end_ns": now}))
        self.changed = True
        self.hub.event({"type": "trial_stopped", "id": tr.id, "reason": reason})
        log.info("trial %s stopped (%s)", tr.id, reason)
        return tr.id

    def set_auto(self, on: bool):
        self.auto = on
        self.changed = True

    def _push(self, kind: str, tr: Trial, s: dict):
        self._q.put((kind, tr, s))
        if kind == "arm":
            tr.n_arm += 1
        elif kind == "imu":
            tr.n_imu += 1
        else:
            tr.n_att += 1

    def on_arm(self, s: dict, moving: bool):
        t = s["t"]
        if self._quiet_needed and not moving and self.hub.last_motion_ns is not None \
                and t - self.hub.last_motion_ns > self.cfg.idle_stop_s * 1e9:
            self._quiet_needed = False
        tr = self.cur
        if tr:
            self._push("arm", tr, s)
            if moving:
                tr.last_motion_ns = t
            elif tr.auto and t - tr.last_motion_ns > self.cfg.idle_stop_s * 1e9:
                self.stop(t, "idle")
        elif self.auto and moving and not self._quiet_needed:
            self.start(t, "motion", auto=True)

    def on_imu(self, kind: str, s: dict):
        if self.cur:
            self._push(kind, self.cur, s)

    def tick(self, now: int):
        tr = self.cur
        if tr and tr.auto and now - tr.last_motion_ns > (self.cfg.idle_stop_s + 1.0) * 1e9:
            self.stop(now, "idle")            # also covers the arm disappearing mid-trial

    def update_meta(self, tid: str, extra: dict) -> None:
        path = self.root / tid / "meta.json"
        with self._meta_lock:
            meta = json.loads(path.read_text())
            meta.update(extra)
            path.write_text(json.dumps(meta, indent=2, ensure_ascii=False))

    def set_label(self, tid: str, label: str) -> bool:
        path = self.root / tid / "meta.json"
        if tid not in self.index or not path.exists():
            return False
        with self._meta_lock:
            meta = json.loads(path.read_text())
            meta["label"] = label
            path.write_text(json.dumps(meta, indent=2, ensure_ascii=False))
        self.index[tid]["label"] = label
        self.changed = True
        return True

    def shutdown(self):
        self._q.put(("quit", None, None))
        self._writer.join(timeout=10)
        self._fin.shutdown(wait=True)

    # ------------------------------------------------------------ writer thread
    def _write_loop(self):
        files: dict[str, tuple] = {}
        J = len(self.cfg.joint_names)
        while True:
            kind, tr, payload = self._q.get()
            if kind == "quit":
                break
            try:
                if kind == "open":
                    fa = open(tr.path / "arm.csv", "w", newline="")
                    fi = open(tr.path / "imu.csv", "w", newline="")
                    ft = open(tr.path / "att.csv", "w", newline="")
                    wa, wi, wt = csv.writer(fa), csv.writer(fi), csv.writer(ft)
                    wa.writerow(arm_header(self.cfg.joint_names))
                    wi.writerow(IMU_COLS)
                    wt.writerow(ATT_COLS)
                    files[tr.id] = (fa, fi, ft, wa, wi, wt)
                    with self._meta_lock:
                        (tr.path / "meta.json").write_text(
                            json.dumps(payload, indent=2, ensure_ascii=False))
                elif kind == "arm":
                    s = payload
                    row = [s["t"], s["t0"], s["t1"], s["seq"], s["leader_t"],
                           TELEOP_CODE.get(s["teleop"], 0)]
                    for j in range(J):
                        for f in ARM_FIELDS:
                            v = s[f][j]
                            row.append(int(v) if f == "ok" else v)
                    files[tr.id][3].writerow(row)
                elif kind == "imu":
                    s = payload
                    files[tr.id][4].writerow([s["rx"], s["cf_ms"], *s["acc"], *s["gyro"]])
                elif kind == "att":
                    s = payload
                    files[tr.id][5].writerow([s["rx"], s["cf_ms"], *s["rpy"]])
                elif kind == "close":
                    for fh in files.pop(tr.id)[:3]:
                        fh.close()
                    with self._meta_lock:
                        mp = tr.path / "meta.json"
                        meta = json.loads(mp.read_text())
                        meta.update(payload, state="processing")
                        mp.write_text(json.dumps(meta, indent=2, ensure_ascii=False))
                    self._fin.submit(self._finalize, tr.path)
            except Exception:  # noqa: BLE001
                log.exception("trial writer error (%s)", kind)

    def _finalize(self, path: Path):
        try:
            with self._meta_lock:
                meta = finalize(path)
        except Exception as e:  # noqa: BLE001
            log.exception("finalize failed for %s", path.name)
            with self._meta_lock:
                mp = path / "meta.json"
                meta = json.loads(mp.read_text())
                meta.update(state="error", error=str(e))
                mp.write_text(json.dumps(meta, indent=2, ensure_ascii=False))
        with self.hub.lock:
            self.index[path.name] = self._summary(meta)
            self.changed = True
            self.hub.event({"type": "trial_ready", "id": path.name, "state": meta["state"]})


class Hub:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.arm_src = None
        self.imu_src = None
        self.clock = OnlineClock()
        self.motion = MotionDetector(cfg.motion_window_s, cfg.motion_thresh_deg)
        self._keep_ns = int((max(cfg.live_window_s, cfg.pre_roll_s) + 2.0) * 1e9)
        self.arm_hist: deque = deque()
        self.imu_hist: deque = deque()
        self.att_hist: deque = deque()
        self._live = {"arm": [], "imu": [], "att": []}
        self._events: list = []
        self.rates = {k: RateMeter() for k in ("arm", "imu", "att")}
        self.last_arm: dict | None = None
        self.last_motion_ns: int | None = None
        self.moving = False
        self.activity = "standby"
        self.trials = TrialManager(cfg, self)

    def attach(self, arm_src, imu_src):
        self.arm_src, self.imu_src = arm_src, imu_src

    def event(self, ev: dict):
        self._events.append(ev)

    # ------------------------------------------------------------ source callbacks
    def push_arm(self, s: dict):
        with self.lock:
            t = s["t"]
            self.rates["arm"].tick(t)
            self._append(self.arm_hist, s, t, "t")
            self._live["arm"].append(s)
            self.last_arm = s
            self.moving = self.motion.update(t, s["pos_deg"])
            if self.moving:
                self.last_motion_ns = t
            self.trials.on_arm(s, self.moving)
            self._update_activity(t)

    def push_imu(self, s: dict):
        self._push_cf("imu", self.imu_hist, s)

    def push_att(self, s: dict):
        self._push_cf("att", self.att_hist, s)

    def _push_cf(self, kind: str, hist: deque, s: dict):
        with self.lock:
            self.clock.update(s["cf_ms"], s["rx"] / 1e6)
            h = self.clock.to_host_ms(s["cf_ms"])
            s["t"] = int(h * 1e6) if h is not None else s["rx"]
            self.rates[kind].tick(s["rx"])
            self._append(hist, s, s["rx"], "rx")
            self._live[kind].append(s)
            self.trials.on_imu(kind, s)

    def _append(self, hist: deque, s: dict, t: int, key: str):
        hist.append(s)
        while hist and hist[0][key] < t - self._keep_ns:
            hist.popleft()

    def _update_activity(self, now: int):
        recent = self.last_motion_ns is not None and \
            now - self.last_motion_ns < self.cfg.standby_after_s * 1e9
        new = "active" if (self.trials.cur is not None or recent) else "standby"
        if new != self.activity:
            self.activity = new
            self.event({"type": "wake" if new == "active" else "standby"})

    # ------------------------------------------------------------ server side
    def tick(self):
        with self.lock:
            now = now_ns()
            self.trials.tick(now)
            self._update_activity(now)

    def start_trial(self, label: str = "") -> str:
        with self.lock:
            return self.trials.start(now_ns(), "manual", auto=False, label=label)

    def stop_trial(self) -> str | None:
        with self.lock:
            return self.trials.stop(now_ns(), "manual", manual=True)

    def drain(self):
        with self.lock:
            live, self._live = self._live, {"arm": [], "imu": [], "att": []}
            events, self._events = self._events, []
            changed, self.trials.changed = self.trials.changed, False
        return pack(live), events, changed

    def snapshot(self) -> dict:
        with self.lock:
            cut = now_ns() - int(self.cfg.live_window_s * 1e9)
            live = {"arm": [s for s in self.arm_hist if s["t"] >= cut],
                    "imu": [s for s in self.imu_hist if s["rx"] >= cut],
                    "att": [s for s in self.att_hist if s["rx"] >= cut]}
        d = pack(live)
        d["reset"] = True
        return d

    def status(self) -> dict:
        with self.lock:
            now = now_ns()
            tr = self.trials.cur
            s = self.last_arm
            joints = None
            if s is not None:
                joints = {"age_s": (now - s["t"]) / 1e9, "ok": s["ok"], "torque": s["torque"],
                          "temp": s["temp"]}
                for k, src, nd in (("pos", "pos_deg", 2), ("cmd", "goal_deg", 2),
                                   ("tgt", "target_deg", 2), ("vel", "vel_dps", 1),
                                   ("load", "load_pct", 1), ("cur", "cur_ma", 0),
                                   ("volt", "volt_v", 1)):
                    joints[k] = [_r(v, nd) for v in s[src]]
            arm = self.arm_src.status() if self.arm_src else {}
            arm["rate_hz"] = round(self.rates["arm"].rate(now), 1)
            cf = self.imu_src.status() if self.imu_src else {"state": "disabled"}
            cf["rate_hz"] = round(self.rates["imu"].rate(now), 1)
            cf["att_rate_hz"] = round(self.rates["att"].rate(now), 1)
            return {
                "type": "status", "now": rel_s(now), "activity": self.activity,
                "moving": self.moving and s is not None and now - s["t"] < 1e9,
                "arm": arm, "cf": cf, "clock": self.clock.stats(),
                "rec": {"recording": tr is not None, "id": tr.id if tr else None,
                        "auto": tr.auto if tr else None,
                        "elapsed_s": (now - tr.t0_ns) / 1e9 if tr else None,
                        "n_arm": tr.n_arm if tr else 0, "n_imu": tr.n_imu if tr else 0},
                "auto_trial": self.trials.auto,
                "joints": joints,
            }

    def shutdown(self):
        with self.lock:
            self.trials.stop(now_ns(), "shutdown")
        self.trials.shutdown()


def _current_calib(root: Path) -> dict | None:
    """Snapshot of data/calib/current.json (alignid result) stored in each trial's meta."""
    try:
        return json.loads((root / "calib" / "current.json").read_text())
    except (OSError, ValueError):
        return None


def pack(live: dict) -> dict:
    """Compact live payload: rel seconds + rounded values (None for missing)."""
    out: dict = {"type": "data"}
    arm = live["arm"]
    if arm:
        out["arm"] = {"t": [round(rel_s(s["t"]), 4) for s in arm]}
        for k, src, nd in (("pos", "pos_deg", 2), ("cmd", "goal_deg", 2), ("tgt", "target_deg", 2),
                           ("vel", "vel_dps", 1), ("load", "load_pct", 1), ("cur", "cur_ma", 0)):
            out["arm"][k] = [[_r(v, nd) for v in s[src]] for s in arm]
    if live["imu"]:
        out["imu"] = {"t": [round(rel_s(s["t"]), 4) for s in live["imu"]],
                      "acc": [[_r(v, 4) for v in s["acc"]] for s in live["imu"]],
                      "gyro": [[_r(v, 2) for v in s["gyro"]] for s in live["imu"]]}
    if live["att"]:
        out["att"] = {"t": [round(rel_s(s["t"]), 4) for s in live["att"]],
                      "rpy": [[_r(v, 2) for v in s["rpy"]] for s in live["att"]]}
    return out
