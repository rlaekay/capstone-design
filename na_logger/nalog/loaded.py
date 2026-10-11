"""Payload ("loaded") runs: one folder per trial under data/loaded/payload_<m>g/ with metadata and CSVs.

    data/loaded/payload_0g/T0114_excitation_v2_set_1_motion_000/
        meta.json      time, payload mass, motion, gripper hold, servo gains, windows, quality
        training.csv   12 s training window, uniform 100 Hz, telemetry + IMU + payload labels
        full.csv       whole trial (approach -> reference -> return), = trial merged.csv
        raw/           arm.csv imu.csv att.csv replay.json trial_meta.json (copies)

Force label: F = m (g - a) on the gripper [N], base frame, g = (0, 0, -9.81), a = payload-COM
acceleration from the IMU (model site payload_com).
"""
from __future__ import annotations

import csv
import json
import shutil
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .finalize import load_streams, point_acceleration

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
FS = 100.0
G_BASE = np.array([0.0, 0.0, -9.81])


def _merged(d: Path) -> dict:
    with open(d / "merged.csv") as f:
        hdr = f.readline().strip().split(",")
    a = np.genfromtxt(d / "merged.csv", delimiter=",", skip_header=1)
    return {h: a[:, i] for i, h in enumerate(hdr)}


def _interp(grid, t, y):
    ok = np.isfinite(y)
    return np.interp(grid, t[ok], y[ok]) if ok.sum() > 2 else np.full(len(grid), np.nan)


def mass_dir(data_dir: Path, mass_g: float) -> Path:
    return Path(data_dir) / "loaded" / f"payload_{mass_g:g}g"


def wait_ready(trial_dir: Path, timeout_s: float = 120.0) -> dict:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        try:
            meta = json.loads((trial_dir / "meta.json").read_text())
            if meta.get("state") == "ready" and (trial_dir / "merged.csv").exists():
                return meta
        except (OSError, json.JSONDecodeError):
            pass
        time.sleep(0.5)
    raise TimeoutError(f"{trial_dir.name}: finalize not done")


def export(trial_dir: Path, data_dir: Path, mass_g: float) -> Path:
    trial_dir = Path(trial_dir)
    meta = wait_ready(trial_dir)
    rp = json.loads((trial_dir / "replay.json").read_text())
    set_name, motion = rp["motion"].split("/")
    out = mass_dir(data_dir, mass_g) / f"{trial_dir.name[:5]}_{set_name}_{motion}"
    (out / "raw").mkdir(parents=True, exist_ok=True)

    calib = meta.get("align_calib")
    off = (rp["script_t0_ns"] - meta["t0_ns"]) / 1e9
    w0, w1 = off + rp["training_window_s"][0], off + rp["training_window_s"][1]
    grid = np.arange(w0, w1 + 1e-9, 1 / FS)
    m = _merged(trial_dir)
    t = m["t_s"]
    names, cols = ["t_s", "t_trial_s"], [grid - grid[0], grid]
    for j in J:
        for k in ("pos_deg", "cmd_deg", "vel_dps", "load_pct", "cur_mA", "volt_V", "temp_C"):
            names.append(f"{j}_{k}")
            cols.append(_interp(grid, t, m[f"{j}_{k}"]))
    corr = None
    s = load_streams(trial_dir)
    if calib and s.get("t_imu") is not None:
        lag = calib.get("lag_ms", 0.0) / 1000.0
        ti = s["t_imu"] - lag
        for k, ax in enumerate("xyz"):
            names.append(f"imu_acc_{ax}_g"); cols.append(np.interp(grid, ti, s["acc"][:, k]))
        for k, ax in enumerate("xyz"):
            names.append(f"imu_gyro_{ax}_dps"); cols.append(np.interp(grid, ti, s["gyro"][:, k]))
        # labels on a 1 s wider window so filter edges fall outside the crop
        gw = np.arange(w0 - 1.0, w1 + 1.0 + 1e-9, 1 / FS)
        pos_w = np.stack([_interp(gw, t, m[f"{j}_pos_deg"]) for j in J], 1)
        pa = point_acceleration(gw, pos_w, s["t_imu"], s["acc"], s["gyro"], calib, point="payload")
        sel = slice(int(round(FS)), int(round(FS)) + len(grid))
        a_imu, a_fk = pa["a_tcp_base"][sel], pa["a_tcp_fk"][sel]
        force = mass_g / 1000.0 * (G_BASE - a_imu)
        for arr, base in ((pa["omega_g"][sel], "gripper_omega_{}_rps"), (a_imu, "payload_acc_imu_base_{}_mps2"),
                          (a_fk, "payload_acc_fk_base_{}_mps2"), (force, "payload_force_on_gripper_base_{}_N")):
            for k, ax in enumerate("xyz"):
                names.append(base.format(ax)); cols.append(arr[:, k])
        try:
            from scipy.signal import butter, sosfiltfilt
            ok = np.all(np.isfinite(a_imu), 1) & np.all(np.isfinite(a_fk), 1)
            sos = butter(4, 3.0, fs=FS, output="sos")
            ai, af = sosfiltfilt(sos, a_imu[ok], axis=0), sosfiltfilt(sos, a_fk[ok], axis=0)
            corr = float(np.mean([np.corrcoef(ai[:, k], af[:, k])[0, 1] for k in range(3)]))
        except Exception:  # noqa: BLE001
            corr = None
    A = np.column_stack(cols)
    with open(out / "training.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(names)
        for row in A:
            w.writerow(["" if not np.isfinite(x) else f"{x:.6g}" for x in row])

    shutil.copy2(trial_dir / "merged.csv", out / "full.csv")
    for name in ("arm.csv", "imu.csv", "att.csv", "replay.json"):
        if (trial_dir / name).exists():
            shutil.copy2(trial_dir / name, out / "raw" / name)
    shutil.copy2(trial_dir / "meta.json", out / "raw" / "trial_meta.json")

    tr = (t >= w0) & (t <= w1)
    pos = np.stack([m[f"{j}_pos_deg"] for j in J], 1)
    cmd = np.stack([m[f"{j}_cmd_deg"] for j in J], 1)
    load = np.stack([m[f"{j}_load_pct"] for j in J], 1)
    sc = (meta.get("calib") or {}).get("follower", {}).get("servo_config", {})
    start_unix = meta.get("started_unix")
    end_unix = meta.get("stopped_unix")
    info = {
        "trial_id": trial_dir.name, "set": set_name, "motion": motion, "payload_mass_g": mass_g,
        "start_local": datetime.fromtimestamp(start_unix).isoformat(timespec="seconds") if start_unix else None,
        "end_local": datetime.fromtimestamp(end_unix).isoformat(timespec="seconds") if end_unix else None,
        "start_unix": start_unix, "end_unix": end_unix, "duration_s": meta.get("duration_s"),
        "source_reference": rp.get("source"),
        "windows_in_full_csv_s": {"approach": [off + a for a in rp["approach_s"]],
                                  "reference": [off + rp["reference_start_s"], off + rp["reference_end_s"]],
                                  "training": [w0, w1]},
        "gripper_hold_deg": rp.get("gripper_fixed_deg"),
        "gripper_start_deg": rp.get("gripper_start_deg"), "grip_squeeze_deg": rp.get("grip_squeeze_deg"),
        "servo_gains": {sid: {"p": c.get("p_coef"), "d": c.get("d_coef"), "i": c.get("i_coef")} for sid, c in sc.items()},
        "servo_protection": {sid: {"overload_torque_pct": c.get("overload_torque"),
                                   "protection_current": c.get("protection_current")} for sid, c in sc.items()},
        "imu_calib_id": (calib or {}).get("id"), "label_point": "payload_com (model site)",
        "rates_hz": {"arm": meta.get("arm_rate_hz"), "imu": meta.get("imu_rate_hz")},
        "imu_coverage": meta.get("imu_coverage"),
        "quality_training_window": {
            "track_rms_deg": [round(float(np.sqrt(np.nanmean((cmd[tr, j] - pos[tr, j]) ** 2))), 3) for j in range(5)],
            "load_max_pct": [round(float(np.nanmax(np.abs(load[tr, j]))), 1) for j in range(6)],
            "imu_vs_fk_corr_3hz": None if corr is None else round(corr, 4),
        },
        "scene": rp.get("scene"),
        "files": {"training.csv": "12 s training window, 100 Hz", "full.csv": "whole trial (= merged.csv)",
                  "raw/": "logger raw streams and trial meta"},
    }
    (out / "meta.json").write_text(json.dumps(info, indent=1, ensure_ascii=False))
    return out
