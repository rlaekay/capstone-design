"""Time alignment between the Crazyflie clock and the host clock.

Every IMU log packet carries the Crazyflie's own millisecond timestamp (cf_ms) and gets a host
receive stamp (rx). rx - cf = clock offset + radio/USB latency, and the latency is >= some
minimum. The fastest packets in each second therefore trace a line (offset + drift * t):
fitting that lower envelope maps cf_ms onto the host clock without the radio jitter.
What remains is a constant delay (sensor filtering, minimum transport latency), which
estimate_lag() measures by finding the time shift at which the joint rates best explain |gyro|.
"""
from __future__ import annotations

from collections import deque

import numpy as np


def fit_cf_clock(cf_ms: np.ndarray, rx_ms: np.ndarray, bin_ms: float = 1000.0,
                 outlier_ms: float = 1.0) -> tuple[np.ndarray, dict]:
    """Return host time (ms, same frame as rx_ms) for each cf_ms, plus fit statistics.

    Splits into segments if the Crazyflie clock resets (reboot)."""
    cf_ms = np.asarray(cf_ms, float)
    rx_ms = np.asarray(rx_ms, float)
    n = len(cf_ms)
    host = np.full(n, np.nan)
    if n == 0:
        return host, {"n": 0}
    breaks = np.where(np.diff(cf_ms) < -500)[0] + 1
    seg_stats = []
    for idx in np.split(np.arange(n), breaks):
        cf, d = cf_ms[idx], rx_ms[idx] - cf_ms[idx]
        b = np.floor((cf - cf[0]) / bin_ms).astype(int)
        order = np.lexsort((d, b))                     # by bin, then by delay
        first = np.r_[True, b[order][1:] != b[order][:-1]]
        sel = order[first]                             # fastest packet of each bin
        x, y = cf[sel] - cf[0], d[sel]
        keep = np.ones(len(sel), bool)
        a, s = float(y.min()), 0.0
        if len(sel) >= 3:
            for _ in range(4):
                A = np.c_[np.ones(keep.sum()), x[keep]]
                (a, s), *_ = np.linalg.lstsq(A, y[keep], rcond=None)
                # a bin whose fastest packet is still slow (congestion) is not on the envelope
                new_keep = (y - (a + s * x)) < outlier_ms
                if new_keep.sum() < 3 or (new_keep == keep).all():
                    break
                keep = new_keep
        env = a + s * (cf - cf[0])
        host[idx] = cf + env
        env_resid = y[keep] - (a + s * x[keep])
        excess = d - env
        seg_stats.append({
            "n": int(len(idx)), "bins": int(len(sel)), "bins_used": int(keep.sum()),
            "drift_ppm": float(s * 1e6),
            "envelope_jitter_ms": float(np.std(env_resid)) if len(env_resid) > 1 else None,
            "rx_excess_median_ms": float(np.median(excess)),
            "rx_excess_p95_ms": float(np.percentile(excess, 95)),
        })
    main = max(seg_stats, key=lambda z: z["n"])
    return host, {**main, "segments": len(seg_stats)}


class OnlineClock:
    """Live version of fit_cf_clock over the last ~30 s, for the GUI and status only."""

    def __init__(self, bin_ms: float = 1000.0, keep_bins: int = 30):
        self.bin_ms = bin_ms
        self.bins: deque = deque(maxlen=keep_bins)   # [bin_index, cf_ms, delay_ms]
        self._fit = None                             # (cf_ref, a, s)
        self._resid = None
        self._outliers = 0
        self._dirty = False

    def update(self, cf_ms: float, rx_ms: float) -> None:
        d = rx_ms - cf_ms
        b = int(cf_ms // self.bin_ms)
        if self.bins and b < self.bins[-1][0] - 2:   # Crazyflie clock went backwards (reboot)
            self.bins.clear()
            self._fit = None
        for entry in reversed(self.bins):           # usually the last bin; tolerate late packets
            if entry[0] == b:
                if d < entry[2]:
                    entry[1], entry[2] = cf_ms, d
                    self._dirty = True
                return
            if entry[0] < b:
                break
        if not self.bins or b > self.bins[-1][0]:
            self.bins.append([b, cf_ms, d])
            self._dirty = True

    def _refit(self):
        self._dirty = False
        pts = list(self.bins)
        self._resid = None
        if len(pts) >= 4:
            x = np.array([p[1] for p in pts[:-1]])   # last bin is still filling
            y = np.array([p[2] for p in pts[:-1]])
            ref = x[0]
            keep = np.ones(len(x), bool)
            for _ in range(3):
                s, a = np.polyfit(x[keep] - ref, y[keep], 1)
                new_keep = (y - (a + s * (x - ref))) < 1.0
                if new_keep.sum() < 3 or (new_keep == keep).all():
                    break
                keep = new_keep
            self._fit = (ref, a, s)
            self._resid = (y - (a + s * (x - ref)))[keep]
            self._outliers = int((~keep).sum())
        elif pts:
            self._fit = (pts[-1][1], min(p[2] for p in pts), 0.0)

    def to_host_ms(self, cf_ms: float) -> float | None:
        if self._dirty:
            self._refit()
        if not self._fit:
            return None
        ref, a, s = self._fit
        return cf_ms + a + s * (cf_ms - ref)

    def stats(self) -> dict:
        if self._dirty:
            self._refit()
        if not self._fit or self._resid is None:
            return {"ready": False}
        return {"ready": True, "drift_ppm": float(self._fit[2] * 1e6),
                "envelope_jitter_ms": float(np.std(self._resid)), "bins": len(self.bins) - 1,
                "outlier_bins": self._outliers}


def estimate_lag(t_arm: np.ndarray, q_deg: np.ndarray, t_imu: np.ndarray, gyro: np.ndarray,
                 fs: float = 200.0, max_lag_s: float = 0.25) -> dict:
    """Delay of the IMU stream relative to the joint stream (positive = IMU arrives late).

    The end-effector angular velocity is a sum of joint axes times joint rates, so
    |gyro|^2 = qd^T M qd, where M (axis dot products) depends only weakly on the pose. For each
    candidate lag, M is fitted by least squares and the lag with the best fit (R^2) wins. This
    does not need the kinematics or joint sign conventions, and parallel pitch joints (whose
    rates add) are handled exactly. A trial that shakes the arm quickly gives the best estimate."""
    out = {"lag_ms": None, "r2": None, "corr": None, "method": "lstsq(|gyro|^2 ~ qd^T M qd)"}
    ok_a = np.all(np.isfinite(q_deg), axis=1)
    ok_i = np.all(np.isfinite(gyro), axis=1)
    if ok_a.sum() < 50 or ok_i.sum() < 50:
        out["reason"] = "데이터 부족"
        return out
    ta, qa = t_arm[ok_a], q_deg[ok_a]
    ti, gi = t_imu[ok_i], gyro[ok_i]
    t_lo = max(ta[0], ti[0]) + max_lag_s
    t_hi = min(ta[-1], ti[-1]) - max_lag_s
    if t_hi - t_lo < 3.0:
        out["reason"] = "겹치는 구간이 3초 미만"
        return out
    L = int(round(max_lag_s * fs))
    grid = np.arange(t_lo - max_lag_s, t_hi + max_lag_s, 1.0 / fs)
    nj = qa.shape[1]
    q = np.stack([np.interp(grid, ta, qa[:, j]) for j in range(nj)], axis=1)
    qd = np.gradient(q, 1.0 / fs, axis=0)
    k = np.ones(5) / 5
    qd = np.stack([np.convolve(qd[:, j], k, mode="same") for j in range(nj)], axis=1)
    g2 = sum(np.interp(grid, ti, gi[:, j]) ** 2 for j in range(gi.shape[1]))
    core = np.where((grid >= t_lo) & (grid <= t_hi))[0]
    core = core[(core - L >= 0) & (core + L < len(grid))]
    speed = np.sqrt((qd[core] ** 2).sum(axis=1))
    if speed.std() < 3.0:
        out["reason"] = "움직임이 너무 작음"
        return out
    iu = np.triu_indices(nj)
    X = np.c_[np.ones(len(core)), (qd[core][:, :, None] * qd[core][:, None, :])[:, iu[0], iu[1]]]
    X /= np.maximum(np.abs(X).max(axis=0), 1e-9)
    r2 = np.empty(2 * L + 1)
    for i, lag in enumerate(range(-L, L + 1)):
        y = g2[core + lag]
        beta, *_ = np.linalg.lstsq(X, y, rcond=None)
        resid = y - X @ beta
        r2[i] = 1.0 - resid.var() / y.var() if y.var() > 0 else 0.0
    i = int(np.argmax(r2))
    frac = 0.0
    if 0 < i < len(r2) - 1:
        y0, y1, y2 = r2[i - 1], r2[i], r2[i + 1]
        den = y0 - 2 * y1 + y2
        frac = 0.5 * (y0 - y2) / den if den != 0 else 0.0
    out["lag_ms"] = float((i - L + frac) / fs * 1000.0)
    out["r2"] = float(r2[i])
    out["corr"] = float(np.sqrt(max(r2[i], 0.0)))
    out["reliable"] = bool(r2[i] > 0.5 and 0 < i < len(r2) - 1)
    return out


class MotionDetector:
    """Motion = any joint moved more than thresh_deg within the last window_s."""

    def __init__(self, window_s: float, thresh_deg: float):
        self.window_ns = int(window_s * 1e9)
        self.thresh = thresh_deg
        self.buf: deque = deque()

    def update(self, t_ns: int, pos_deg: list) -> bool:
        self.buf.append((t_ns, pos_deg))
        while len(self.buf) > 1 and self.buf[1][0] <= t_ns - self.window_ns:
            self.buf.popleft()
        ref = self.buf[0][1]
        return any(p is not None and r is not None and abs(p - r) > self.thresh
                   for p, r in zip(pos_deg, ref))

    def reset(self):
        self.buf.clear()


class RateMeter:
    def __init__(self, span_s: float = 2.0):
        self.span_ns = int(span_s * 1e9)
        self.ts: deque = deque()

    def tick(self, t_ns: int):
        self.ts.append(t_ns)
        while self.ts and self.ts[0] < t_ns - self.span_ns:
            self.ts.popleft()

    def rate(self, now_ns: int) -> float:
        while self.ts and self.ts[0] < now_ns - self.span_ns:
            self.ts.popleft()
        if len(self.ts) < 2:
            return 0.0
        return (len(self.ts) - 1) / ((self.ts[-1] - self.ts[0]) / 1e9) if self.ts[-1] > self.ts[0] else 0.0
