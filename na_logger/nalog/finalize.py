"""Post-trial processing: raw CSVs -> merged.csv (host timeline, physical units) + preview.json."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from . import units
from .sync import estimate_lag, fit_cf_clock

ARM_TIME_COLS = ["t_ns", "t_req_ns", "t_resp_ns", "seq", "leader_t_ns", "teleop"]
ARM_FIELDS = ["ok", "torque", "goal", "pos", "vel", "load", "volt", "temp", "moving", "cur",
              "leader", "target", "status"]
IMU_COLS = ["rx_ns", "cf_ms", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]
ATT_COLS = ["rx_ns", "cf_ms", "roll", "pitch", "yaw"]
TELEOP_CODE = {"off": 0, "active": 1, "hold": 2, "script": 3}


def arm_header(joints: list[str]) -> list[str]:
    return ARM_TIME_COLS + [f"{j}.{f}" for j in joints for f in ARM_FIELDS]


def file_arm_header(path: Path, joints: list[str]) -> list[str]:
    """Columns as written in this arm.csv (older trials have fewer per-joint fields)."""
    try:
        with open(path) as f:
            h = f.readline().strip().split(",")
        if len(h) > 1:
            return h
    except OSError:
        pass
    return arm_header(joints)


def _load(path: Path, ncols: int) -> np.ndarray:
    if not path.exists() or path.stat().st_size == 0:
        return np.empty((0, ncols))
    with open(path) as f:
        if sum(1 for _ in f) <= 1:
            return np.empty((0, ncols))
    return np.genfromtxt(path, delimiter=",", skip_header=1, ndmin=2)


def _clean(a, nd=3) -> list:
    return [None if (x is None or (isinstance(x, float) and math.isnan(x))) else x
            for x in np.round(np.asarray(a, float), nd).tolist()]


def _interp_nan(t_new, t, y):
    """Linear interpolation that returns NaN outside the sampled range."""
    out = np.interp(t_new, t, y)
    out[(t_new < t[0]) | (t_new > t[-1])] = np.nan
    return out


def imu_host_times(imu: np.ndarray, att: np.ndarray, t0_ns: int):
    """Host time [ms, trial timeline] of every imu and att row from the Crazyflie clock fit."""
    cf_all = np.r_[imu[:, 1], att[:, 1]] if len(att) else imu[:, 1]
    rx_all = np.r_[imu[:, 0], att[:, 0]] if len(att) else imu[:, 0]
    order = np.argsort(cf_all, kind="stable")
    host_sorted, stats = fit_cf_clock(cf_all[order], (rx_all[order] - t0_ns) / 1e6)
    host_all = np.empty_like(host_sorted)
    host_all[order] = host_sorted
    return host_all[: len(imu)], host_all[len(imu):], stats


def load_streams(trial_dir: Path) -> dict:
    """Arm joint degrees and IMU streams on the trial timeline [s], IMU lag not removed."""
    trial_dir = Path(trial_dir)
    meta = json.loads((trial_dir / "meta.json").read_text())
    joints = meta["joint_names"]
    hdr = file_arm_header(trial_dir / "arm.csv", joints)
    col = {h: i for i, h in enumerate(hdr)}
    arm = _load(trial_dir / "arm.csv", len(hdr))
    imu = _load(trial_dir / "imu.csv", len(IMU_COLS))
    att = _load(trial_dir / "att.csv", len(ATT_COLS))
    mids = meta.get("calib", {}).get("follower", {}).get("mid") or [2047.5] * len(joints)
    t_s = (arm[:, col["t_ns"]] - meta["t0_ns"]) / 1e9
    pos = np.stack([(arm[:, col[f"{j}.pos"]] - mids[i]) * units.POS_DEG_PER_STEP
                    for i, j in enumerate(joints)], 1)
    out = {"meta": meta, "t_s": t_s, "pos_deg": pos, "t_imu": None, "acc": None, "gyro": None}
    if len(imu) >= 10:
        h_imu, _, stats = imu_host_times(imu, att, meta["t0_ns"])
        o = np.argsort(h_imu)
        out.update(t_imu=h_imu[o] / 1000.0, acc=imu[o][:, 2:5], gyro=imu[o][:, 5:8], clock=stats)
    return out


def point_acceleration(t_s, pos_deg, t_imu_raw, acc_g, gyro_dps, calib: dict,
                       point: str = "payload") -> dict:
    """Gripper-frame specific force and base-frame acceleration of a gripper point
    (default: the payload centre of mass, model site payload_com)."""
    from .accel import G_BASE, imu_to_point, resample
    from .kin import POINTS, kin_from_calib

    k = kin_from_calib(calib)
    r = k.site_in_gripper(POINTS[point]) - np.asarray(calib["p_gi"])
    tg, omega, _alpha, f_p = imu_to_point(t_imu_raw, gyro_dps, acc_g, calib, r)
    n = len(t_s)
    out = {k_: np.full((n, 3), np.nan) for k_ in ("omega_g", "f_tcp_g", "a_tcp_base", "a_tcp_fk")}
    if len(tg) < 10:
        return out
    out["omega_g"] = resample(t_s, tg, omega)
    out["f_tcp_g"] = resample(t_s, tg, f_p)
    ok = np.all(np.isfinite(pos_deg), axis=1)
    if ok.sum() < 10:
        return out
    R_bg, _ = k.gripper_traj(pos_deg[ok])
    out["a_tcp_base"][ok] = np.einsum("nij,nj->ni", R_bg, out["f_tcp_g"][ok]) + G_BASE
    # independent check: TCP acceleration from joint positions alone (FK, double derivative)
    from scipy.signal import savgol_filter
    fs = 100.0
    grid = np.arange(t_s[ok][0], t_s[ok][-1], 1.0 / fs)
    if len(grid) > 21:
        q = resample(grid, t_s[ok], pos_deg[ok])
        sid = k.m.site(POINTS[point]).id
        p_tcp = np.array([k.fk(row).site_xpos[sid].copy() for row in q])
        a_fk = savgol_filter(p_tcp, 21, 3, deriv=2, delta=1.0 / fs, axis=0)
        out["a_tcp_fk"] = resample(t_s, grid, a_fk)
    return out


def finalize(trial_dir: Path) -> dict:
    trial_dir = Path(trial_dir)
    meta = json.loads((trial_dir / "meta.json").read_text())
    joints = meta["joint_names"]
    J = len(joints)
    t0 = meta["t0_ns"]
    latency_ms = float(meta["config"].get("imu_latency_ms", 0.0))
    period_ms = float(meta["config"].get("imu_period_ms", 10))

    hdr = file_arm_header(trial_dir / "arm.csv", joints)
    col = {h: i for i, h in enumerate(hdr)}
    arm = _load(trial_dir / "arm.csv", len(hdr))
    imu = _load(trial_dir / "imu.csv", len(IMU_COLS))
    att = _load(trial_dir / "att.csv", len(ATT_COLS))
    n = len(arm)
    if n == 0:
        meta.update(state="empty", n_arm=0, n_imu=int(len(imu)), finalized_unix=time.time())
        (trial_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
        return meta

    fcal = meta.get("calib", {}).get("follower", {})
    mids = fcal.get("mid") or [2047.5] * J
    t_ns = arm[:, col["t_ns"]]
    t_s = (t_ns - t0) / 1e9

    def jcol(j, f):
        return arm[:, col[f"{joints[j]}.{f}"]]

    pos_deg = np.stack([(jcol(j, "pos") - mids[j]) * units.POS_DEG_PER_STEP for j in range(J)], 1)
    cmd_deg = np.stack([(jcol(j, "goal") - mids[j]) * units.POS_DEG_PER_STEP for j in range(J)], 1)
    tgt_deg = np.stack([(jcol(j, "target") - mids[j]) * units.POS_DEG_PER_STEP for j in range(J)], 1)
    vel = np.stack([jcol(j, "vel") * units.VEL_DPS_PER_STEP for j in range(J)], 1)
    load = np.stack([jcol(j, "load") * units.LOAD_PCT_PER_UNIT for j in range(J)], 1)
    cur = np.stack([jcol(j, "cur") * units.CUR_MA_PER_UNIT for j in range(J)], 1)
    volt = np.stack([jcol(j, "volt") * units.VOLT_V_PER_UNIT for j in range(J)], 1)
    temp = np.stack([jcol(j, "temp") for j in range(J)], 1)

    # ---- Crazyflie clock -> host clock (imu + att packets share one Crazyflie clock)
    clock_stats, lag = {"n": 0}, {"lag_ms": None, "reason": "IMU 데이터 없음"}
    imu_cols = np.full((n, 9), np.nan)
    imu_age = np.full(n, np.nan)
    t_imu = gyro = acc = None
    if len(imu) >= 10:
        h_imu, h_att, clock_stats = imu_host_times(imu, att, t0)
        host_ms = h_imu - latency_ms
        o = np.argsort(host_ms)
        t_imu = host_ms[o] / 1000.0
        acc, gyro = imu[o][:, 2:5], imu[o][:, 5:8]
        for k in range(3):
            imu_cols[:, k] = _interp_nan(t_s, t_imu, acc[:, k])
            imu_cols[:, 3 + k] = _interp_nan(t_s, t_imu, gyro[:, k])
        if len(att):
            ha = h_att - latency_ms
            oa = np.argsort(ha)
            ta = ha[oa] / 1000.0
            for k in range(3):
                imu_cols[:, 6 + k] = _interp_nan(t_s, ta, att[oa][:, 2 + k])
        idx = np.clip(np.searchsorted(t_imu, t_s), 1, len(t_imu) - 1)
        imu_age = np.minimum(np.abs(t_s - t_imu[idx - 1]), np.abs(t_imu[idx] - t_s)) * 1000.0
        gap = imu_age > 2.5 * period_ms
        imu_cols[gap] = np.nan
        lag = estimate_lag(t_s, pos_deg[:, : max(1, J - 1)], t_imu + latency_ms / 1000.0, gyro)
        # the estimate is for the raw stream; report what remains after the configured correction
        if lag.get("lag_ms") is not None:
            lag["lag_ms_after_correction"] = lag["lag_ms"] - latency_ms

    # ---- merged.csv
    names = ["t_s", "t_host_ns", "unix_s", "teleop"]
    anchor = meta["clock_anchor"]
    unix = anchor["unix_ns"] / 1e9 + (t_ns - anchor["mono_ns"]) / 1e9
    cols = [t_s, t_ns, unix, arm[:, col["teleop"]]]
    fmts = ["%.6f", "%d", "%.6f", "%.0f"]
    per_joint = [("pos_deg", pos_deg, "%.4f"), ("cmd_deg", cmd_deg, "%.4f"),
                 ("target_deg", tgt_deg, "%.4f"), ("vel_dps", vel, "%.3f"),
                 ("load_pct", load, "%.1f"), ("cur_mA", cur, "%.1f"),
                 ("volt_V", volt, "%.1f"), ("temp_C", temp, "%.0f")]
    for j, name in enumerate(joints):
        for suffix, arr, fm in per_joint:
            names.append(f"{name}_{suffix}")
            cols.append(arr[:, j])
            fmts.append(fm)
        for suffix, f in (("torque", "torque"), ("pos_raw", "pos"), ("cmd_raw", "goal")):
            names.append(f"{name}_{suffix}")
            cols.append(jcol(j, f))
            fmts.append("%.0f")
    imu_names = ["imu_acc_x_g", "imu_acc_y_g", "imu_acc_z_g", "imu_gyro_x_dps", "imu_gyro_y_dps",
                 "imu_gyro_z_dps", "cf_roll_deg", "cf_pitch_deg", "cf_yaw_deg"]
    for k, nm in enumerate(imu_names):
        names.append(nm)
        cols.append(imu_cols[:, k])
        fmts.append("%.5f" if k < 6 else "%.3f")
    names.append("imu_age_ms")
    cols.append(imu_age)
    fmts.append("%.2f")

    pa = None
    calib = meta.get("align_calib")
    if calib and t_imu is not None:
        try:
            point = meta["config"].get("accel_point", "payload")
            pa = point_acceleration(t_s, pos_deg, t_imu + latency_ms / 1000.0, acc, gyro, calib,
                                    point)
            pa["f_tcp_g"][imu_age > 2.5 * period_ms] = np.nan
            pa["a_tcp_base"][imu_age > 2.5 * period_ms] = np.nan
            outs = [("omega_g", "gyro_grip_{}_rps", "%.5f"),
                    ("f_tcp_g", "acc_" + point + "_grip_{}_mps2", "%.4f"),
                    ("a_tcp_base", "acc_" + point + "_base_{}_mps2", "%.4f"),
                    ("a_tcp_fk", "acc_" + point + "_fk_base_{}_mps2", "%.4f")]
            mass = meta.get("payload_mass_g")
            if mass:
                # force the payload exerts on the robot (NeuralActuator sign: -m g z at rest)
                from .accel import G_BASE
                pa["force"] = mass / 1000.0 * (G_BASE - pa["a_tcp_base"])
                pa["force_fk"] = mass / 1000.0 * (G_BASE - pa["a_tcp_fk"])
                outs += [("force", "force_label_base_{}_N", "%.4f"),
                         ("force_fk", "force_fk_base_{}_N", "%.4f")]
            for key, prefix, fm in outs:
                for k_, ax in enumerate("xyz"):
                    names.append(prefix.format(ax))
                    cols.append(pa[key][:, k_])
                    fmts.append(fm)
        except Exception as e:  # noqa: BLE001
            meta["point_accel_error"] = str(e)
            pa = None
    table = np.column_stack(cols)
    with open(trial_dir / "merged.csv", "w") as f:
        np.savetxt(f, table, delimiter=",", fmt=fmts, header=",".join(names), comments="")

    # ---- preview.json (decimated, for the GUI review view)
    step = max(1, int(math.ceil(n / 3000)))
    sl = slice(None, None, step)
    preview = {"t": _clean(t_s[sl], 4), "joints": joints,
               "pos": [_clean(pos_deg[sl, j], 2) for j in range(J)],
               "cmd": [_clean(cmd_deg[sl, j], 2) for j in range(J)],
               "tgt": [_clean(tgt_deg[sl, j], 2) for j in range(J)],
               "vel": [_clean(vel[sl, j], 1) for j in range(J)],
               "load": [_clean(load[sl, j], 1) for j in range(J)],
               "cur": [_clean(cur[sl, j], 1) for j in range(J)],
               "acc": [_clean(imu_cols[sl, k], 3) for k in range(3)],
               "gyro": [_clean(imu_cols[sl, 3 + k], 2) for k in range(3)]}
    if pa is not None:
        preview["acc_pt"] = [_clean(pa["a_tcp_base"][sl, k], 3) for k in range(3)]
        preview["acc_pt_fk"] = [_clean(pa["a_tcp_fk"][sl, k], 3) for k in range(3)]
    (trial_dir / "preview.json").write_text(json.dumps(preview, separators=(",", ":")))

    dt = np.diff(t_s)
    meta.update(
        state="ready",
        duration_s=float(t_s[-1] - t_s[0]) if n > 1 else 0.0,
        n_arm=int(n), n_imu=int(len(imu)), n_att=int(len(att)),
        arm_rate_hz=float(1.0 / np.median(dt)) if len(dt) else None,
        arm_dt_p99_ms=float(np.percentile(dt, 99) * 1000) if len(dt) else None,
        imu_rate_hz=float(1.0 / np.median(np.diff(t_imu))) if t_imu is not None and len(t_imu) > 1 else None,
        imu_coverage=float(np.isfinite(imu_cols[:, 3]).mean()),
        clock_fit=clock_stats, imu_lag=lag, finalized_unix=time.time(),
        max_temp_C=float(np.nanmax(temp)) if np.isfinite(temp).any() else None,
    )
    (trial_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
    return meta
