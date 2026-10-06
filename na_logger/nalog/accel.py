"""IMU (Crazyflie) measurements -> acceleration of a chosen gripper point.

Rigid body, all vectors in the gripper frame g:
    omega_g = R_gi (gyro - b_g)
    f_P     = R_gi (f_imu - b_a) + alpha_g x r + omega_g x (omega_g x r),   r = p_P - p_imu
where f is specific force (what an accelerometer measures, a - g). In the base frame:
    a_P     = R_bg(q) f_P + g_base           (kinematic acceleration, gravity removed)
So the transform needs the rotation AND the lever arm (T), plus omega and alpha at every instant.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter

G = 9.81
G_BASE = np.array([0.0, 0.0, -G])


def skew(v) -> np.ndarray:
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def resample(t_new, t, y):
    out = np.stack([np.interp(t_new, t, y[:, k]) for k in range(y.shape[1])], axis=1)
    out[(t_new < t[0]) | (t_new > t[-1])] = np.nan
    return out


def imu_to_point(t_imu, gyro_dps, acc_g, calib: dict, r_point_g, fs: float = 100.0,
                 window: int = 9):
    """IMU streams (host time, lag NOT yet removed) -> point acceleration on a uniform grid.

    Returns t_grid (physical time), omega_g [rad/s], alpha_g [rad/s^2], f_point_g [m/s^2]."""
    lag = calib.get("lag_ms", 0.0) / 1000.0
    acc_lag = lag + calib.get("acc_extra_lag_ms", 0.0) / 1000.0
    R = np.asarray(calib["R_gi"])
    bg = np.radians(np.asarray(calib.get("gyro_bias_dps", [0, 0, 0])))
    ba = np.asarray(calib.get("acc_bias_g", [0, 0, 0]))
    sc = np.asarray(calib.get("acc_scale", [1, 1, 1]))
    t0, t1 = t_imu[0] - lag, t_imu[-1] - acc_lag
    grid = np.arange(np.ceil(t0 * fs) / fs, t1, 1.0 / fs)
    gyro = resample(grid + lag, t_imu, np.radians(gyro_dps)) - bg
    acc = (resample(grid + acc_lag, t_imu, acc_g) - ba) / sc * G
    ok = np.all(np.isfinite(gyro), axis=1) & np.all(np.isfinite(acc), axis=1)
    grid, gyro, acc = grid[ok], gyro[ok], acc[ok]
    w = min(window, len(grid) - (1 - len(grid) % 2))
    if w < 5:
        return grid, gyro @ R.T, np.zeros_like(gyro), acc @ R.T
    omega = savgol_filter(gyro, w, 3, axis=0) @ R.T
    alpha = savgol_filter(gyro, w, 3, deriv=1, delta=1.0 / fs, axis=0) @ R.T
    r = np.asarray(r_point_g, float)
    f = acc @ R.T + np.cross(alpha, r) + np.cross(omega, np.cross(omega, r))
    return grid, omega, alpha, f
