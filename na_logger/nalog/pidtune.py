"""Compare servo P/D/I gains on the real arm by replaying one trajectory per candidate.

Gains are written temporarily (EEPROM lock closed: gone after a power cycle) and the original
gains are restored at the end. Score per candidate, from the recorded trial:
  shake   : drone gyro magnitude above 3 Hz [deg/s rms] — the visible oscillation
  hold    : same, during the last second (arm commanded still) — sustained limit cycle
  track   : |goal - position| rms of the tuned joints [deg] — what lower P costs

python -m nalog.pidtune [--joints 2 3] [--motion excitation_v2_set_1/motion_002] [--dry]
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

URL = "http://localhost:8080"
DATA = Path(__file__).resolve().parent.parent / "data"
CANDIDATES = [  # (P, D, I) for the tuned joints; first = current, measured as the baseline
    None, (40, 32, 0), (32, 32, 0), (24, 32, 0), (16, 32, 0), (24, 48, 0),
]


def call(path, body=None, timeout=30):
    req = urllib.request.Request(URL + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"},
                                 method="GET" if body is None else "POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def score(tid: str, joints: list[int]) -> dict:
    from .finalize import load_streams
    s = load_streams(DATA / tid)
    meta = json.loads((DATA / tid / "meta.json").read_text())
    names = meta["joint_names"]
    rp = json.loads((DATA / tid / "replay.json").read_text()) if (DATA / tid / "replay.json").exists() else {}
    t_imu, gyro = s["t_imu"], s["gyro"]
    fs = 100.0
    grid = np.arange(t_imu[0], t_imu[-1], 1 / fs)
    g = np.stack([np.interp(grid, t_imu, gyro[:, k]) for k in range(3)], 1)
    b, a = butter(4, 3.0 / (fs / 2), "high")
    shake = np.linalg.norm(filtfilt(b, a, g, axis=0), axis=1)
    # replay.json times are seconds after script_t0_ns; trial times are seconds after t0_ns
    off = (rp["script_t0_ns"] - meta["t0_ns"]) / 1e9 if rp.get("script_t0_ns") else 0.0
    w0 = off + rp.get("reference_start_s", 0.0)
    w1 = off + rp.get("reference_end_s", grid[-1])
    run = (grid >= w0) & (grid <= w1)
    hold = (grid > w1 + 0.2) & (grid <= w1 + 1.2)        # commanded still after the clip
    arm = np.genfromtxt(DATA / tid / "arm.csv", delimiter=",", names=True)
    ta = (arm["t_ns"] - meta["t0_ns"]) / 1e9
    ra = (ta >= w0) & (ta <= w1)
    track = []
    for j in joints:
        n = names[j - 1]
        e = (arm[f"{n}goal"] - arm[f"{n}pos"]) * 360 / 4095
        track.append(float(np.sqrt(np.nanmean(e[ra] ** 2))))
    return {"trial": tid, "shake_dps": float(np.sqrt((shake[run] ** 2).mean())),
            "shake_max_dps": float(shake[run].max()),
            "hold_dps": float(np.sqrt((shake[hold] ** 2).mean())), "track_deg": track}


def run_one(motion: str, tag: str, timeout: float = 120.0) -> str:
    before = len(call("/api/replay/status")["done"])
    r = call("/api/replay/start", {"motions": [motion], "tag": tag})
    if not r.get("ok"):
        raise RuntimeError(r)
    t = time.time()
    while time.time() - t < timeout:
        st = call("/api/replay/status")
        if len(st["done"]) > before:
            e = st["done"][-1]
            if not e.get("ok"):
                raise RuntimeError(e)
            return e["trial"]
        time.sleep(1.0)
    call("/api/replay/abort", {})
    raise RuntimeError("timeout")


def wait_ready(tid: str, timeout=60):
    t = time.time()
    while time.time() - t < timeout:
        m = json.loads((DATA / tid / "meta.json").read_text())
        if m.get("state") in ("ready", "error", "empty"):
            return
        time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser(prog="nalog.pidtune")
    ap.add_argument("--joints", type=int, nargs="+", default=[2, 3])
    ap.add_argument("--motion", default="excitation_v2_set_1/motion_002")
    ap.add_argument("--dry", action="store_true", help="only check the trajectory, no motion")
    a = ap.parse_args()
    orig = call("/api/servo/gains")
    print("current gains:", {j: orig[str(j)] for j in a.joints}, flush=True)
    if a.dry:
        return
    results = []
    try:
        for cand in CANDIDATES:
            g = orig[str(a.joints[0])] if cand is None else dict(zip("pdi", cand))
            r = call("/api/servo/gains", {"gains": {str(j): g for j in a.joints}, "persist": False})
            if not r.get("ok"):
                raise RuntimeError(f"gain write failed: {r}")
            tid = run_one(a.motion, f"pidtune P{g['p']} D{g['d']} I{g['i']}")
            wait_ready(tid)
            sc = score(tid, a.joints) | {"gains": g}
            results.append(sc)
            print(f"P{g['p']:3d} D{g['d']:3d} I{g['i']}  shake {sc['shake_dps']:6.1f} dps "
                  f"(max {sc['shake_max_dps']:6.1f})  hold {sc['hold_dps']:5.1f} dps  "
                  f"track {['%.2f' % x for x in sc['track_deg']]} deg  [{tid}]", flush=True)
    finally:
        call("/api/servo/gains", {"gains": {str(j): orig[str(j)] for j in a.joints}, "persist": False})
        out = DATA / "calib" / f"pidtune_{time.strftime('%Y%m%d-%H%M%S')}.json"
        out.write_text(json.dumps({"joints": a.joints, "motion": a.motion, "original": orig,
                                   "results": results}, indent=2))
        print("original gains restored; results ->", out, flush=True)


if __name__ == "__main__":
    main()
