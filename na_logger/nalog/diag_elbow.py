"""Elbow (ID 3) click diagnostic.

Sweeps only the elbow at a fixed arm pose under whatever payload is taped on, then looks for
IMU impulses (drone gyro high-passed > 15 Hz) and relates them to elbow angle, direction, load,
current, tracking error and the servo status (error) register.

  .venv/bin/python -m nalog.diag_elbow run            # moves the arm (logger on :8080, this Mac)
  .venv/bin/python -m nalog.diag_elbow analyze T0410 T0411 ...

Reading the result:
  clicks at the same elbow angle at slow speed  -> damaged tooth / debris in the gear train
  clicks only at high speed or load, any angle  -> overload / backlash slam
  status bits set (0x20 overload, 0x08 overcurrent) with the load dropping -> servo protection
"""
from __future__ import annotations

import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

URL = "http://localhost:8080"
DATA = Path(__file__).resolve().parent.parent / "data"
ELBOW = "elbow_flex"
STATUS_BITS = {0x01: "voltage", 0x02: "angle", 0x04: "overheat", 0x08: "overcurrent", 0x20: "overload"}


def _req(path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode()
    r = urllib.request.Request(URL + path, data=data, method="GET" if body is None else "POST",
                               headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(r, timeout=10) as f:
        return json.loads(f.read())


def sweep(base, lo, hi, stroke_s, cycles, hold_s=0.6) -> list[list[float]]:
    """Waypoints [t, q1..q6]: elbow lo -> hi -> lo, `cycles` times, everything else at `base`."""
    def q(e):
        p = list(base)
        p[2] = e
        return p
    t, wp = 0.0, [[0.0, *q(lo)]]
    for _ in range(cycles):
        for e in (hi, lo):
            t += stroke_s
            wp.append([t, *q(e)])
            t += hold_s
            wp.append([t, *q(e)])
    return wp


def tests(grip: float) -> list[tuple[str, list]]:
    # shoulder_lift -15: the elbow carries ~1.3-1.7 N m of gravity torque over the whole sweep
    # with 407 g (model), about what it saw when the clicks happened (load 30-50 %)
    ext = [-27, -15, 0, -13, 103, grip]
    # the pose of the 2026-10-11 clicks (T0405, set_2/motion_002)
    t405 = [-30, -50, 0, -13, 103, grip]
    return [
        ("elbow_slow", sweep(ext, -20, 45, 4.0, 3)),       # ~16 deg/s mean
        ("elbow_med", sweep(ext, -20, 45, 1.3, 4)),        # ~50 deg/s mean, ~95 peak
        ("elbow_t405", sweep(t405, 0, 40, 0.8, 5)),        # ~50 deg/s mean, ~95 peak
    ]


def run():
    st = _req("/api/status")
    if not st["arm"]["follower"]["connected"] or st["cf"]["state"] != "connected":
        sys.exit("follower / Crazyflie not connected")
    grip = st["joints"]["pos"][5]
    trials = []
    for name, wp in tests(grip):
        r = _req("/api/replay/custom", {"name": name, "tag": "diag", "waypoints": wp})
        if not r.get("ok"):
            sys.exit(f"{name}: {r}")
        print(f"▶ {name} ({wp[-1][0]:.0f} s + approach)", flush=True)
        while True:
            time.sleep(1.0)
            s = _req("/api/replay/status")
            if s["state"] not in ("running", "approach", "settle", "hold", "starting") and not s["queue"]:
                break
        last = s["done"][-1] if s["done"] else {}
        if not last.get("ok"):
            sys.exit(f"{name} failed: {s['message']}")
        trials.append(last["trial"])
        print(f"  {last['trial']}", flush=True)
    analyze(trials)


def _csv(path: Path) -> dict:
    hdr = path.open().readline().strip().split(",")
    a = np.genfromtxt(path, delimiter=",", skip_header=1)
    return {h: a[:, i] for i, h in enumerate(hdr)}


def analyze(tids: list[str]):
    from scipy.signal import butter, filtfilt

    b, a = butter(4, 15 / 50, "high")
    for tid in tids:
        tdir = DATA / tid if (DATA / tid).exists() else next(DATA.glob(f"{tid}_*"))
        meta = json.loads((tdir / "meta.json").read_text())
        m = _csv(tdir / "merged.csv")
        t = m["t_s"]
        e, goal = m[f"{ELBOW}_pos_deg"], m[f"{ELBOW}_cmd_deg"]
        vel, load, cur = m[f"{ELBOW}_vel_dps"], m[f"{ELBOW}_load_pct"], m[f"{ELBOW}_cur_mA"]
        raw = _csv(tdir / "arm.csv")
        if f"{ELBOW}.status" in raw:                      # servo error register (recorded since 2026-10-11)
            k = np.clip(np.searchsorted(raw["t_ns"], m["t_host_ns"]), 0, len(raw["t_ns"]) - 1)
            status = np.nan_to_num(raw[f"{ELBOW}.status"][k]).astype(int)
        else:
            status = np.zeros(len(t), int)
        g = np.stack([m[f"imu_gyro_{c}_dps"] for c in "xyz"], 1)
        ok = np.isfinite(g).all(1)
        g = np.stack([np.interp(t, t[ok], g[ok, c]) for c in range(3)], 1)
        gh = np.linalg.norm(filtfilt(b, a, g, axis=0), axis=1)
        thr = max(40.0, 8 * float(np.median(gh)))
        hit = gh > thr
        on = np.flatnonzero(hit & np.r_[True, ~hit[:-1]])
        moving = np.abs(vel) > 5
        print(f"\n=== {tdir.name}  {meta.get('label', '')}")
        print(f"  elbow {np.nanmin(e[moving]):.0f}..{np.nanmax(e[moving]):.0f} deg, |load| max {np.nanmax(np.abs(load)):.0f} %, "
              f"|I| max {np.nanmax(np.abs(cur)):.0f} mA, track err max {np.nanmax(np.abs(goal - e)):.1f} deg, "
              f"gyro HP median {np.median(gh):.1f} dps -> threshold {thr:.0f}")
        bits = int(np.bitwise_or.reduce(status)) if len(status) else 0
        print(f"  status register OR = 0x{bits:02x} {[n for k_, n in STATUS_BITS.items() if bits & k_]}")
        print(f"  impulses: {len(on)}")
        for k in on:
            w = slice(max(0, k - 5), k + 3)
            print(f"   t={t[k]:6.2f}s  gyroHP {gh[k]:5.0f} dps | elbow {e[k]:6.1f} deg  v {vel[k]:+5.0f} dps  "
                  f"load {np.nanmax(np.abs(load[w])):3.0f} %  I {np.nanmax(np.abs(cur[w])):5.0f} mA  "
                  f"err {goal[k] - e[k]:+5.1f}  status 0x{int(np.bitwise_or.reduce(status[w])):02x}")
        # encoder-side check: position steps larger than the command can explain
        jumps = np.flatnonzero(np.abs(np.diff(e)) > np.abs(np.diff(goal)) + 2.0)
        print(f"  encoder jumps > command+2 deg: {len(jumps)}" +
              ("" if not len(jumps) else f" at elbow {np.round(e[jumps], 1)[:10]}"))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "run":
        run()
    elif len(sys.argv) > 2 and sys.argv[1] == "analyze":
        analyze(sys.argv[2:])
    else:
        print(__doc__)
