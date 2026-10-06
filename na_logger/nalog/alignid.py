"""alignid: identify how the Crazyflie IMU sits on the gripper (one button in the GUI).

1. plan     : a ~70 s trajectory around the arm's current pose: 8 static poses (gravity) and
              single/multi-joint shaking (gyro). Checked against joint limits and the MuJoCo
              model for collisions (8 mm margin) before anything moves.
2. run      : the follower executes it (torque on, slew-limited) while a trial records.
3. solve    : time lag (joints vs gyro), gyro bias (static windows), pitch-zero offset (grid),
              rotation R_gi (Kabsch on body angular velocity: gyro vs FK), accelerometer bias and
              lever arm p_gi (linear least squares on specific force vs FK, ridge to nominal).
Result -> data/calib/align_*.json and data/calib/current.json (used by new trials).
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter

from .accel import G, G_BASE, resample, skew
from .clock import now_ns
from .kin import JointMap, Kin, rot_angle_deg

log = logging.getLogger(__name__)
ACC_EXTRA_LAG_MS = 4.7      # firmware LPF: accel 30 Hz vs gyro 80 Hz (2nd order) group delay gap
# The real arm adds 5-50 Hz vibration (printed links, tape, servo jitter) that the IMU sees and
# joint-encoder FK cannot. The excitation lives below ~1.5 Hz (its square up to ~3 Hz), so all
# fits use zero-phase low-passed IMU and FK signals.
FIT_LPF_HZ = 3.0
PITCH_JOINT = 3             # pitch-chain zero offset is applied to wrist_flex (same axis as 1, 2)


# ============================================================ plan
@dataclass
class Segment:
    kind: str                 # move | static | excite
    t0: float
    t1: float
    qa: np.ndarray
    qb: np.ndarray | None = None
    amp: np.ndarray | None = None
    freq: np.ndarray | None = None
    label: str = ""
    idx: int = -1                  # index into the static / excite list it came from

    def at(self, t: float) -> np.ndarray:
        if self.kind == "static":
            return self.qa
        u = min(1.0, max(0.0, (t - self.t0) / (self.t1 - self.t0)))
        if self.kind == "move":
            s = u ** 3 * (10 - 15 * u + 6 * u ** 2)           # min-jerk
            return self.qa + (self.qb - self.qa) * s
        T = self.t1 - self.t0
        tau = t - self.t0
        ramp = 0.8
        env = math.sin(0.5 * math.pi * min(1.0, tau / ramp, (T - tau) / ramp)) ** 2
        return self.qa + env * self.amp * np.sin(2 * math.pi * self.freq * tau)

    def as_dict(self) -> dict:
        return {"kind": self.kind, "t0": self.t0, "t1": self.t1, "label": self.label}


@dataclass
class Plan:
    q0: np.ndarray
    segments: list[Segment]
    limits: np.ndarray                                   # (6, 2) real degrees
    duration: float = 0.0
    issues: list[str] = field(default_factory=list)

    def goal(self, t: float) -> np.ndarray:
        for s in self.segments:
            if t <= s.t1:
                return s.at(t)
        return self.segments[-1].at(self.segments[-1].t1)

    def sample(self, dt: float = 0.02) -> np.ndarray:
        return np.array([self.goal(t) for t in np.arange(0, self.duration + dt, dt)])


STATIC_DELTAS = [  # pan, lift, elbow, flex, roll (deg, relative to start); gripper never moves
    (0, 0, 0, 0, 0), (0, 0, 0, 0, 90), (0, 0, 0, 0, -90), (0, 0, 0, 35, 0),
    (0, 0, 0, -35, 45), (0, -15, 20, 0, 45), (0, 12, -20, -20, -60), (0, 0, 0, 30, -120),
    (0, 0, 0, 0, 0)]
# precise mode: gravity seen from many directions (roll sweep x 3 flex levels) so the
# accelerometer's per-axis scale and bias can be fitted (consumer MEMS: ~1-2 % scale error)
PRECISE_STATICS = [(0, 0, 0, 0, 0)] + [
    (0, 0, 0, flex, roll) for flex in (0, 35, -35) for roll in (-150, -100, -50, 0, 50, 100, 150)
    if (flex, roll) != (0, 0)] + [(0, -15, 20, 0, 45), (0, 12, -20, -20, -60), (0, 0, 0, 60, 90),
                                  (0, 0, 0, 0, 0)]
EXCITE = [  # label, amplitude (deg) per joint, frequency (Hz) per joint, duration (s)
    ("wrist_roll 흔들기", (0, 0, 0, 0, 60), (0, 0, 0, 0, 0.5), 6.0),
    ("wrist_flex 흔들기", (0, 0, 0, 30, 0), (0, 0, 0, 0.6, 0), 5.0),
    ("elbow 흔들기", (0, 0, 20, 0, 0), (0, 0, 0.6, 0, 0), 5.0),
    ("shoulder_lift 흔들기", (0, 12, 0, 0, 0), (0, 0.6, 0, 0, 0), 5.0),
    ("shoulder_pan 흔들기", (30, 0, 0, 0, 0), (0.5, 0, 0, 0, 0), 5.0),
    ("전체 섞기", (20, 8, 12, 20, 40), (0.37, 0.61, 0.43, 0.71, 0.53), 10.0),
]


def real_limits(kin: Kin, margin_deg: float = 5.0) -> np.ndarray:
    """Model joint ranges converted to logger degrees, shrunk by a margin. (6, 2)"""
    sign = np.asarray(kin.jmap.sign, float)[:, None]
    off = np.asarray(kin.jmap.offset_deg, float)[:, None]
    lim = np.sort(kin.jnt_range_deg / sign - off, axis=1)
    return lim + np.array([margin_deg, -margin_deg])


def make_plan(q0_deg, kin: Kin, move_dps: float = 45.0, statics=None, excite_scale=None) -> Plan:
    q0 = np.asarray(q0_deg, float).copy()
    lim = real_limits(kin)
    lim[5] = [q0[5] - 1, q0[5] + 1]                       # gripper stays where it is
    issues = []
    out_of_range = [kin.m.joint(j).name for j in range(5) if not lim[j, 0] <= q0[j] <= lim[j, 1]]
    if out_of_range:
        issues.append(f"시작 자세가 모델 관절 범위 밖: {', '.join(out_of_range)} — 토크가 꺼진 상태에서 팔을 손으로 "
                      "들어 중간 자세(팔꿈치를 반쯤 편 자세, 그리퍼가 바닥·몸체에서 떨어진 자세)로 옮긴 뒤 다시 누르세요")
    clip = lambda q: np.clip(q, lim[:, 0], lim[:, 1])   # noqa: E731
    segs: list[Segment] = []
    t = 0.0
    cur = q0.copy()

    def add_move(target, label):
        nonlocal t, cur
        dq = float(np.max(np.abs(target - cur)))
        if dq > 0.05:
            T = max(1.0, 1.875 * dq / move_dps)
            segs.append(Segment("move", t, t + T, cur.copy(), target.copy(), label=label))
            t += T
        cur = target.copy()

    statics = STATIC_DELTAS if statics is None else statics
    excite_scale = [1.0] * len(EXCITE) if excite_scale is None else excite_scale
    for i, d in enumerate(statics):
        if d is None:                                  # dropped by plan_safe (collision)
            continue
        target = clip(q0 + np.r_[d, 0.0])
        add_move(target, f"정지 자세 {i + 1}로 이동")
        segs.append(Segment("static", t, t + 2.2, target.copy(), label=f"정지 자세 {i + 1}", idx=i))
        t += 2.2
    center = clip(q0)
    add_move(center, "흔들기 시작 자세로 이동")
    for (label, amp, freq, dur), sc in zip(EXCITE, excite_scale):
        if sc <= 0:
            continue
        a = np.r_[amp, 0.0].astype(float) * sc
        # shrink the amplitude where the centre is close to a limit
        room = np.minimum(center - lim[:, 0], lim[:, 1] - center)
        a = np.minimum(a, np.maximum(room, 0))
        segs.append(Segment("excite", t, t + dur, center.copy(), amp=a, freq=np.r_[freq, 0.0],
                            label=label, idx=EXCITE.index((label, amp, freq, dur))))
        t += dur
    segs.append(Segment("static", t, t + 1.0, center.copy(), label="완료"))
    t += 1.0
    return Plan(q0=q0, segments=segs, limits=lim, duration=t, issues=issues)


def plan_safe(q0_deg, kin: Kin, max_repairs: int = 80,
              precise: bool = False) -> tuple[Plan, list[str]]:
    """make_plan, then repair whatever the model says collides: a static pose is mirrored, then
    halved, then dropped; a shaking segment is halved, then dropped. Returns (plan, problems)."""
    base_statics = PRECISE_STATICS if precise else STATIC_DELTAS
    statics = [tuple(d) for d in base_statics]
    scale = [1.0] * len(EXCITE)
    tried: dict[int, int] = {}
    for _ in range(max_repairs + 1):
        plan = make_plan(q0_deg, kin, statics=statics, excite_scale=scale)
        if plan.issues:
            return plan, plan.issues
        hit = first_collision(plan, kin)
        if hit is None:
            problems = check_plan(plan, kin, collisions=False)
            n_static = sum(1 for s in plan.segments if s.kind == "static") - 1
            if n_static < (14 if precise else 6):
                problems.append("충돌을 피할 수 있는 정지 자세가 6개 미만 — 팔을 더 높고 넓은 자세로 옮긴 뒤 다시 시도")
            return plan, problems
        t, h = hit
        seg = next((s for s in plan.segments if s.t0 <= t <= s.t1), plan.segments[-1])
        if seg.kind == "move":                    # a move belongs to the static pose it goes to
            seg = next((s for s in plan.segments if s.t0 >= seg.t1 - 1e-9), seg)
        n = tried.get((seg.kind, seg.idx), 0)
        tried[(seg.kind, seg.idx)] = n + 1
        if seg.kind == "static" and 0 < seg.idx < len(statics) - 1:
            base = np.array(base_statics[seg.idx], float)
            statics[seg.idx] = tuple([-base, 0.5 * base, -0.5 * base][n]) if n < 3 else None
        elif seg.kind == "excite":
            scale[seg.idx] = 0.5 if n == 0 else 0.0
        else:
            break
    return plan, [f"충돌 예상 ({t:.1f}s, {seg.label}): {h['geom1']} ↔ {h['geom2']} "
                  f"(거리 {h['dist_mm']} mm) — 팔을 더 높고 넓은 자세로 옮긴 뒤 다시 시도"]


def first_collision(plan: Plan, kin: Kin, dt: float = 0.04):
    q = plan.sample(dt)
    hits = kin.collisions(q)
    if not hits:
        return None
    return hits[0]["index"] * dt, hits[0]


def check_plan(plan: Plan, kin: Kin, collisions: bool = True) -> list[str]:
    """Joint-limit, speed and collision check. Returns problems (empty = safe to run)."""
    problems = list(plan.issues)
    q = plan.sample(0.02)
    v = np.abs(np.diff(q, axis=0)) / 0.02
    if v.max() > 220:
        problems.append(f"계획 속도 {v.max():.0f}°/s가 너무 빠름")
    hits = kin.collisions(q, stride=2) if collisions else []
    if hits:
        h = hits[0]
        t = h["index"] * 0.02
        seg = next((s.label for s in plan.segments if s.t0 <= t <= s.t1), "")
        problems.append(f"충돌 예상 ({t:.1f}s, {seg}): {h['geom1']} ↔ {h['geom2']} "
                        f"(거리 {h['dist_mm']} mm) — 팔을 더 높고 넓은 자세로 옮긴 뒤 다시 시도")
    return problems


# ============================================================ solve
def _kabsch(A: np.ndarray, B: np.ndarray, w=None) -> np.ndarray:
    """R minimising sum w |B - R A|^2 (A, B: (N,3))."""
    w = np.ones(len(A)) if w is None else w
    H = (A * w[:, None]).T @ B
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T


def _body_rates(R: np.ndarray, fs: float) -> np.ndarray:
    """Body angular velocity [rad/s] (in the rotating frame) from a rotation sequence."""
    w = np.zeros((len(R), 3))
    M = np.einsum("nji,njk->nik", R[:-2], R[2:])           # R(t-h)^T R(t+h)
    S = 0.5 * (M - np.transpose(M, (0, 2, 1)))
    w[1:-1] = np.c_[S[:, 2, 1], S[:, 0, 2], S[:, 1, 0]] * fs / 2.0
    w[0], w[-1] = w[1], w[-2]
    return w


def _static_mask(t, segments, settle: float = 0.8) -> np.ndarray:
    m = np.zeros(len(t), bool)
    for s in segments:
        if s["kind"] == "static" and s["t1"] - s["t0"] > settle + 0.3:
            m |= (t >= s["t0"] + settle) & (t <= s["t1"] - 0.1)
    return m


def _lpf(x: np.ndarray, fc: float, fs: float) -> np.ndarray:
    b, a = butter(4, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


def solve(streams: dict, segments: list[dict], kin_nominal: Kin, fs: float = 100.0,
          lag_ms: float | None = None, lpf_hz: float = FIT_LPF_HZ) -> dict:
    """streams: load_streams() output with t_s/pos_deg/t_imu/acc/gyro on the trial timeline.
    segments: plan segments with times on the trial timeline."""
    from .sync import estimate_lag

    t_s, pos = streams["t_s"], streams["pos_deg"]
    t_imu, acc_g, gyro_dps = streams["t_imu"], streams["acc"], streams["gyro"]
    if t_imu is None or len(t_imu) < 500:
        raise ValueError("IMU 데이터가 부족합니다 (Crazyflie 연결 확인)")
    if lag_ms is None:
        lag = estimate_lag(t_s, pos[:, :5], t_imu, gyro_dps)
        lag_ms = lag["lag_ms"] if lag.get("lag_ms") is not None else 0.0
        lag_info = lag
    else:
        lag_info = {"lag_ms": lag_ms, "given": True}
    tau, tau_a = lag_ms / 1000.0, (lag_ms + ACC_EXTRA_LAG_MS) / 1000.0

    seg_t0 = min(s["t0"] for s in segments)
    seg_t1 = max(s["t1"] for s in segments)
    lo = max(seg_t0, t_s[0], t_imu[0] - tau) + 0.2
    hi = min(seg_t1, t_s[-1], t_imu[-1] - tau_a) - 0.2
    grid = np.arange(lo, hi, 1.0 / fs)
    ok = np.all(np.isfinite(pos), axis=1)
    q = savgol_filter(resample(grid, t_s[ok], pos[ok]), 9, 3, axis=0)
    gyro = np.radians(resample(grid + tau, t_imu, gyro_dps))
    acc = resample(grid + tau_a, t_imu, acc_g) * G
    good = np.all(np.isfinite(q), 1) & np.all(np.isfinite(gyro), 1) & np.all(np.isfinite(acc), 1)
    grid, q, gyro, acc = grid[good], q[good], gyro[good], acc[good]
    gyro_raw = gyro.copy()
    gyro = _lpf(gyro, lpf_hz, fs)
    acc = _lpf(acc, lpf_hz, fs)
    static = _static_mask(grid, segments)
    if static.sum() < 50:
        raise ValueError("정지 구간 데이터 부족")
    b_g = gyro[static].mean(axis=0)                        # robot still -> gyro reads its bias
    w_i = gyro - b_g

    kw = Kin(path=kin_nominal.path)

    def fk_series(jmap: JointMap, rows=slice(None)):
        kw.jmap = jmap
        return kw.gripper_traj(q[rows])

    def evaluate(jmap: JointMap, step: int):
        rows = slice(None, None, step)
        R_bg, _ = fk_series(jmap, rows)
        w_g = _lpf(_body_rates(R_bg, fs / step), lpf_hz, fs / step)
        dyn = np.linalg.norm(w_g, axis=1) > 0.15
        Rgi = _kabsch(w_i[rows][dyn], w_g[dyn])
        g_res = (w_g[dyn] - w_i[rows][dyn] @ Rgi.T)
        gyro_rms = float(np.sqrt((g_res ** 2).sum(1).mean()))
        st = static[rows]
        up_i = (R_bg[st] @ Rgi).transpose(0, 2, 1) @ np.array([0, 0, 1.0])   # gravity dir in imu
        f = acc[rows][st] / G
        b_a = (f - up_i).mean(axis=0)
        fd = f - b_a
        ang = np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", fd, up_i)
                                           / np.linalg.norm(fd, axis=1), -1, 1)))
        grav_rms = float(np.sqrt((ang ** 2).mean()))
        cost = gyro_rms / 0.02 + np.radians(grav_rms) / 0.01
        return cost, Rgi, gyro_rms, grav_rms, b_a

    base = kin_nominal.jmap
    def with_offset(d, signs=None):  # noqa: E306
        off = list(base.offset_deg)
        off[PITCH_JOINT] += d
        return JointMap(sign=list(signs or base.sign), offset_deg=off)

    # pitch-chain zero offset: coarse grid then refine (subsampled), then full resolution
    best = min(((evaluate(with_offset(d), 4)[0], d) for d in np.arange(-15, 15.1, 1.0)))
    d0 = best[1]
    best = min(((evaluate(with_offset(d), 2)[0], d) for d in np.arange(d0 - 1, d0 + 1.01, 0.1)))
    delta = float(best[1])
    jmap = with_offset(delta)
    cost, R_gi, gyro_rms, grav_rms, b_a = evaluate(jmap, 1)

    # sign sanity check: does flipping any single joint explain the data much better?
    sign_warn = []
    for j in range(5):
        s = list(jmap.sign)
        s[j] = -s[j]
        c_flip, _, gr, _, _ = evaluate(JointMap(sign=s, offset_deg=jmap.offset_deg), 4)
        if gr < 0.5 * gyro_rms:
            sign_warn.append({"joint": kin_nominal.m.joint(j).name, "gyro_rms_flipped": gr})

    # accelerometer intrinsics from static poses: f = s * u + b per axis (u = gravity dir in imu)
    R_bg, p_bg = fk_series(jmap)
    acc_scale = np.ones(3)
    poses = []
    for sg in segments:
        if sg["kind"] == "static" and sg["t1"] - sg["t0"] > 1.1:
            m = (grid >= sg["t0"] + 0.8) & (grid <= sg["t1"] - 0.1)
            if m.sum() > 20:
                u = (R_bg[m] @ R_gi).transpose(0, 2, 1) @ np.array([0, 0, 1.0])
                poses.append(((acc[m] / G).mean(0), u.mean(0)))
    if len(poses) >= 12:
        F = np.array([p_[0] for p_ in poses])
        U = np.array([p_[1] for p_ in poses])
        if np.all(U.max(0) - U.min(0) > 0.8):              # each axis saw gravity both ways
            for ax in range(3):
                A_ = np.c_[U[:, ax], np.ones(len(U))]
                (acc_scale[ax], b_a[ax]), *_ = np.linalg.lstsq(A_, F[:, ax], rcond=None)
            Fc = (F - b_a) / acc_scale
            angs = np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", Fc / np.linalg.norm(
                Fc, axis=1)[:, None], U), -1, 1)))
            grav_rms = float(np.sqrt((angs ** 2).mean()))
    acc = (acc / G - b_a) / acc_scale * G + b_a * G          # scale-corrected, bias kept for LS

    # lever arm + accelerometer bias: R_gi f = R_bg^T (x_dd - g) + K p + R_gi b_a
    w_g = savgol_filter(_body_rates(R_bg, fs), 9, 3, axis=0)
    a_g = savgol_filter(w_g, 9, 3, deriv=1, delta=1.0 / fs, axis=0)
    x_dd = savgol_filter(p_bg, 21, 3, deriv=2, delta=1.0 / fs, axis=0)
    y = acc @ R_gi.T - _lpf(np.einsum("nji,nj->ni", R_bg, x_dd - G_BASE), lpf_hz, fs)
    K = np.array([skew(a) + skew(w) @ skew(w) for a, w in zip(a_g, w_g)])
    K = _lpf(K.reshape(len(K), 9), lpf_hz, fs).reshape(-1, 3, 3)   # filter K p like y (linear)
    R_nom, p_nom = kin_nominal.nominal_imu()
    use = (np.linalg.norm(w_g, axis=1) > 0.15) | static

    def fit(mask, lam=0.02):
        A = np.concatenate([K[mask], np.repeat(np.eye(3)[None], mask.sum(), 0)], axis=2).reshape(-1, 6)
        reg = np.zeros((3, 6))
        reg[:, :3] = np.eye(3) * math.sqrt(lam * mask.sum())   # weak pull to the model mount
        x, *_ = np.linalg.lstsq(np.vstack([A, reg]), np.r_[y[mask].reshape(-1), reg[:, :3] @ p_nom],
                                rcond=None)
        r = A @ x - y[mask].reshape(-1)
        return x, float(np.sqrt((r ** 2).reshape(-1, 3).sum(1).mean()))

    x, acc_rms = fit(use)
    p_gi = x[:3]
    # consistency check: alternate 2 s blocks, so both halves contain every kind of motion
    half = (np.floor((grid - grid[0]) / 2.0).astype(int) % 2) == 0
    p1, p2 = fit(half)[0][:3], fit(use & ~half)[0][:3]
    split_mm = float(np.linalg.norm(p1 - p2) * 1000)
    sv = np.linalg.svd(K[use].reshape(-1, 3), compute_uv=False) / math.sqrt(use.sum())
    p_dev = float(np.linalg.norm(p_gi - p_nom))
    p_source = "estimated"
    if split_mm > 15.0 or sv[-1] < 0.2 or np.linalg.norm(p_gi) > 0.2:
        p_gi, p_source = p_nom.copy(), "nominal (추정 불안정)"
    gyro_full = np.radians(np.degrees(gyro_raw - b_g)) @ R_gi.T
    w_full = _body_rates(R_bg, fs)
    gyro_rms_full = float(np.degrees(np.sqrt(((w_full - gyro_full)[use] ** 2).sum(1).mean())))

    quality = "good" if (gyro_rms < np.radians(3) and grav_rms < 2.0 and not sign_warn
                         and p_source == "estimated") else "check"
    return {
        "R_gi": R_gi.tolist(), "p_gi": [float(v) for v in p_gi], "p_source": p_source,
        "p_estimated": [float(v) for v in x[:3]], "p_nominal": p_nom.tolist(),
        "p_split_halves_mm": [round(v * 1000, 1) for v in (*p1, *p2)], "p_split_diff_mm": split_mm,
        "p_from_nominal_mm": p_dev * 1000, "fit_lpf_hz": lpf_hz,
        "R_nominal": R_nom.tolist(), "angle_from_nominal_deg": rot_angle_deg(R_nom.T @ R_gi),
        "gyro_bias_dps": np.degrees(b_g).tolist(), "acc_bias_g": [float(v) for v in b_a],
        "acc_scale": [float(v) for v in acc_scale], "n_static_poses": len(poses),
        "lag_ms": float(lag_ms), "lag_info": lag_info, "acc_extra_lag_ms": ACC_EXTRA_LAG_MS,
        "joint_map": {"sign": jmap.sign, "offset_deg": jmap.offset_deg},
        "pitch_offset_deg": delta,
        "residuals": {"gyro_rms_dps": float(np.degrees(gyro_rms)), "gravity_rms_deg": grav_rms,
                      "gyro_rms_dps_unfiltered": gyro_rms_full,
                      "acc_rms_mps2": acc_rms, "n_samples": int(len(grid)),
                      "n_static": int(static.sum()), "lever_sv_min": float(sv[-1])},
        "sign_warnings": sign_warn, "quality": quality,
    }


# ============================================================ runner
class AlignRunner:
    """Orchestrates plan -> move+record -> solve; one instance owned by the server."""

    def __init__(self, hub, kin_factory):
        self.hub = hub
        self.kin_factory = kin_factory                   # () -> Kin with current calib applied
        self.state = "idle"        # idle | checking | running | solving | done | error | aborted
        self.step = ""
        self.progress = 0.0
        self.message = ""
        self.result: dict | None = None
        self.last_problems: list[str] = []
        self._abort = threading.Event()
        self._thread: threading.Thread | None = None
        self.plan: Plan | None = None

    def status(self) -> dict:
        return {"state": self.state, "step": self.step, "progress": round(self.progress, 3),
                "message": self.message, "problems": self.last_problems,
                "duration_s": self.plan.duration if self.plan else None,
                "result": _summary(self.result) if self.result else None}

    def busy(self) -> bool:
        return self.state in ("checking", "running", "solving")

    def _preconditions(self) -> tuple[list[str], np.ndarray | None]:
        hub = self.hub
        st = hub.status()
        problems = []
        arm = st["arm"]
        if not arm.get("follower", {}).get("connected"):
            problems.append("팔로워가 연결되지 않았습니다")
        cf = st["cf"]
        if cf.get("state") != "connected" or cf.get("rate_hz", 0) < 50:
            problems.append("Crazyflie IMU가 연결돼 있어야 합니다 (100 Hz 수신)")
        s = hub.last_arm
        if s is None or any(v is None for v in s["pos_deg"][:6]):
            problems.append("팔로워 관절값을 읽지 못했습니다")
            return problems, None
        return problems, np.array(s["pos_deg"], float)

    def check(self, precise: bool = False) -> dict:
        """Plan + safety check without moving."""
        problems, q0 = self._preconditions()
        if q0 is None:
            self.last_problems = problems
            return {"ok": False, "problems": problems}
        kin = self.kin_factory()
        plan, more = plan_safe(q0, kin, precise=precise)
        problems += more
        self.last_problems = problems
        return {"ok": not problems, "problems": problems, "duration_s": plan.duration}

    def start(self, precise: bool = False) -> dict:
        if self.busy():
            return {"ok": False, "problems": ["이미 실행 중"]}
        self._abort.clear()
        self.state, self.step, self.progress, self.message = "checking", "안전 점검", 0.0, ""
        problems, q0 = self._preconditions()
        kin = self.kin_factory()
        if q0 is not None:
            plan, more = plan_safe(q0, kin, precise=precise)
            problems += more
        self.last_problems = problems
        if problems:
            self.state, self.message = "error", problems[0]
            return {"ok": False, "problems": problems}
        self.plan = plan
        self._thread = threading.Thread(target=self._run, args=(plan,), name="alignid", daemon=True)
        self._thread.start()
        return {"ok": True, "duration_s": plan.duration}

    def abort(self):
        self._abort.set()
        if self.hub.arm_src:
            self.hub.arm_src.command("hold")

    def _run(self, plan: Plan):
        hub = self.hub
        arm = hub.arm_src
        tid = None
        try:
            self.state, self.step = "running", "기록 시작"
            hub.stop_trial()
            tid = hub.start_trial(label="alignid")
            cmd_ns = now_ns()
            arm.start_script(plan.goal, plan.duration)
            t_start = time.monotonic()
            script_t0 = None
            while True:
                if self._abort.is_set():
                    raise _Aborted()
                st = arm.status()
                script_t0 = script_t0 or st.get("script_t0_ns")
                el = time.monotonic() - t_start
                self.progress = min(0.85, 0.85 * el / plan.duration)
                self.step = next((s.label for s in plan.segments if s.t0 <= el <= s.t1), self.step)
                if st.get("teleop") not in ("script",) and el > 1.0:
                    if el < plan.duration - 0.5:
                        raise RuntimeError(f"팔 동작이 중단됨: {st.get('message', '')}")
                    break
                if not st.get("follower", {}).get("connected"):
                    raise RuntimeError("팔로워 연결 끊김")
                time.sleep(0.1)
            time.sleep(0.5)
            hub.stop_trial()
            self.state, self.step, self.progress = "solving", "데이터 처리", 0.88
            trial_dir = Path(hub.cfg.data_dir) / tid
            meta = _wait_ready(hub, tid)
            t0_trial = meta["t0_ns"]
            off = ((script_t0 or cmd_ns) - t0_trial) / 1e9
            segs = [{**s.as_dict(), "t0": s.t0 + off, "t1": s.t1 + off} for s in plan.segments]
            from .finalize import load_streams
            streams = load_streams(trial_dir)
            self.step = "정렬 계산"
            res = solve(streams, segs, self.kin_factory(nominal=True))
            res.update(id=f"align_{time.strftime('%Y%m%d-%H%M%S')}", created_unix=time.time(),
                       trial_id=tid, segments=segs, model=str(self.kin_factory().path.name),
                       q_start_deg=plan.q0.tolist(), sim=bool(meta.get("sim")))
            save_calib(Path(hub.cfg.data_dir), res)
            try:                                   # mount the drone in the MuJoCo model files too
                from .kin import MODEL_DIR
                from .model_update import edit_model
                for name in ("scene.xml", "torque_scene.xml"):
                    edit_model(MODEL_DIR / name, MODEL_DIR / name, res)
            except Exception as e:  # noqa: BLE001
                log.warning("model update failed: %s", e)
            hub.trials.update_meta(tid, {"alignid_result": res["id"], "segments": segs})
            self.result = res
            self.progress, self.state, self.step = 1.0, "done", "완료"
            self.message = (f"회전 {res['angle_from_nominal_deg']:.1f}° (모델 대비), "
                            f"자이로 잔차 {res['residuals']['gyro_rms_dps']:.2f}°/s, "
                            f"중력 잔차 {res['residuals']['gravity_rms_deg']:.2f}°")
            hub.event({"type": "align_done", "id": res["id"], "quality": res["quality"]})
        except _Aborted:
            self.state, self.step, self.message = "aborted", "중단됨", "사용자가 중단했습니다 (팔은 현재 자세 유지)"
            hub.stop_trial()
        except Exception as e:  # noqa: BLE001
            log.exception("alignid failed")
            arm.command("hold")
            hub.stop_trial()
            self.state, self.message = "error", str(e)


class _Aborted(Exception):
    pass


def _wait_ready(hub, tid: str, timeout: float = 60.0) -> dict:
    t = time.monotonic()
    while time.monotonic() - t < timeout:
        st = hub.trials.index.get(tid, {}).get("state")
        if st in ("ready", "error", "empty"):
            break
        time.sleep(0.2)
    return json.loads((Path(hub.cfg.data_dir) / tid / "meta.json").read_text())


def calib_dir(data_dir: Path) -> Path:
    d = Path(data_dir) / "calib"
    d.mkdir(parents=True, exist_ok=True)
    return d


def save_calib(data_dir: Path, res: dict) -> None:
    d = calib_dir(data_dir)
    txt = json.dumps(res, indent=2, ensure_ascii=False)
    (d / f"{res['id']}.json").write_text(txt)
    (d / "current.json").write_text(txt)


def load_current(data_dir: Path) -> dict | None:
    p = calib_dir(data_dir) / "current.json"
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return None


def _summary(res: dict) -> dict:
    R = np.asarray(res["R_gi"])
    return {
        "id": res.get("id"), "quality": res.get("quality"),
        "angle_from_nominal_deg": res.get("angle_from_nominal_deg"),
        "p_gi_mm": [round(v * 1000, 1) for v in res["p_gi"]], "p_source": res.get("p_source"),
        "p_nominal_mm": [round(v * 1000, 1) for v in res.get("p_nominal", [0, 0, 0])],
        "R_gi": np.round(R, 4).tolist(), "lag_ms": res.get("lag_ms"),
        "pitch_offset_deg": res.get("pitch_offset_deg"),
        "gyro_bias_dps": [round(v, 3) for v in res.get("gyro_bias_dps", [])],
        "acc_bias_g": [round(v, 4) for v in res.get("acc_bias_g", [])],
        "acc_scale": [round(v, 4) for v in res.get("acc_scale", [1, 1, 1])],
        "n_static_poses": res.get("n_static_poses"),
        "residuals": res.get("residuals"), "sign_warnings": res.get("sign_warnings"),
        "sim": res.get("sim"), "created_unix": res.get("created_unix"),
        "p_split_diff_mm": res.get("p_split_diff_mm"),
    }


def resolve(data_dir: Path, calib_id: str) -> dict:
    """Re-run the solver on a recorded alignid trial (no robot motion) and make it current."""
    from .finalize import load_streams
    old = json.loads((calib_dir(data_dir) / f"{calib_id}.json").read_text())
    streams = load_streams(Path(data_dir) / old["trial_id"])
    jm = old.get("joint_map", {})
    base = Kin()
    base.jmap = JointMap(sign=jm.get("sign", [1.0] * 6),
                         offset_deg=[o - (old.get("pitch_offset_deg", 0.0) if j == PITCH_JOINT else 0.0)
                                     for j, o in enumerate(jm.get("offset_deg", [0.0] * 6))])
    res = solve(streams, old["segments"], base)
    res.update({k: old[k] for k in ("trial_id", "segments", "model", "q_start_deg", "sim") if k in old})
    res.update(id=f"{calib_id}_v2", created_unix=time.time(), resolved_from=calib_id)
    save_calib(Path(data_dir), res)
    return res


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(prog="nalog.alignid")
    ap.add_argument("--resolve", required=True, help="calib id, e.g. align_20261006-212759")
    ap.add_argument("--data-dir", default=str(Path(__file__).resolve().parent.parent / "data"))
    a = ap.parse_args()
    r = resolve(Path(a.data_dir), a.resolve)
    print(json.dumps(_summary(r), ensure_ascii=False, indent=1))
