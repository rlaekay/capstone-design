import numpy as np

from nalog.arms import ArmCalib, map_degrees, map_range
from nalog.sts import merge_reads, parse_status_packets, sm_decode
from nalog.sync import MotionDetector, estimate_lag, fit_cf_clock


def test_sign_magnitude():
    assert sm_decode(0x8005, 15) == -5
    assert sm_decode(0x0005, 15) == 5
    assert sm_decode(0x0400 | 300, 10) == -300


def _status(sid, payload):
    body = [sid, len(payload) + 2, 0, *payload]
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def test_decode_state_and_cmd():
    st = bytearray(15)

    def put(addr, v):
        st[addr - 56] = v & 0xFF
        st[addr - 56 + 1] = (v >> 8) & 0xFF

    put(56, 2048)              # position
    put(58, 0x8000 | 120)      # velocity -120
    put(60, 0x0400 | 250)      # load -250
    st[62 - 56] = 121          # 12.1 V
    st[63 - 56] = 38
    put(69, 40)
    cmd = bytes([1, 254, 2100 & 0xFF, 2100 >> 8])
    d = merge_reads(bytes(st), cmd)
    assert d == {"pos": 2048, "vel": -120, "load": -250, "volt": 121, "temp": 38, "status": 0,
                 "moving": 0, "cur": 40, "torque": 1, "accel": 254, "goal": 2100}
    assert merge_reads(bytes(st), None)["goal"] is None
    assert merge_reads(None, cmd) is None


def test_parser_keeps_servos_after_a_corrupt_reply():
    p = [bytes([i] * 15) for i in range(1, 7)]
    bad = bytearray(_status(4, p[3]))
    bad[7] ^= 0x55                                       # corrupt servo 4 payload
    buf = b"".join(_status(i + 1, p[i]) for i in range(3)) + bytes(bad) + \
        b"".join(_status(i + 1, p[i]) for i in range(4, 6))
    got = parse_status_packets(buf, 15)
    assert sorted(got) == [1, 2, 3, 5, 6]
    assert got[6] == p[5]


def test_teleop_mapping():
    lead = ArmCalib([1000, 500], [3000, 3500])
    fol = ArmCalib([800, 1000], [3200, 3000])
    assert map_range([1000, 2000], lead, fol) == [800, 2000]
    assert map_range([2000, 3500], lead, fol) == [2000, 3000]
    assert map_range([5000, None], lead, fol) == [3200, None]      # clipped to leader range
    assert map_degrees([2100, 2000], lead, fol) == [2100, 2000]


def test_clock_fit_recovers_drift_under_jitter():
    rng = np.random.default_rng(0)
    host_true = np.arange(0, 60_000, 10.0)                       # 60 s at 100 Hz (ms)
    drift = 40e-6
    cf = np.floor((host_true + 123456.0) * (1 + drift))
    rx = host_true + 2.5 + rng.exponential(3.0, len(host_true))
    rx[rng.random(len(rx)) < 0.05] += 20                         # retry bursts
    host, st = fit_cf_clock(cf, rx)
    err = host - (host_true + 2.5)
    assert abs(st["drift_ppm"] + drift * 1e6) < 5                # cf runs fast -> negative slope
    assert np.abs(err - np.median(err)).max() < 1.0              # jitter removed (< 1 ms)
    assert abs(np.median(err)) < 1.0


def test_lag_estimate():
    fs = 100.0
    t = np.arange(0, 20, 1 / fs)
    def motion(x):  # bursts of shaking with smooth on/off, like a real trajectory
        return 30 * np.sin(2 * np.pi * 0.7 * x) * np.sin(2 * np.pi * x / 12) ** 2

    q = np.stack([motion(t)] * 2, axis=1)
    t_imu = np.arange(0, 20, 0.01) + 0.003                       # IMU samples not aligned to arm
    true_lag = 0.015
    w = np.gradient(motion(t_imu - true_lag), 0.01)
    gyro = np.stack([w * 1.4, np.zeros_like(w), np.zeros_like(w)], axis=1)
    r = estimate_lag(t, q, t_imu, gyro)
    assert r["reliable"]
    assert abs(r["lag_ms"] - 15.0) < 2.0


def test_motion_detector():
    m = MotionDetector(0.2, 1.5)
    assert not m.update(0, [0.0, 0.0])
    assert not m.update(100_000_000, [1.0, 0.0])
    assert m.update(150_000_000, [2.0, 0.0])
    assert not m.update(600_000_000, [2.1, 0.0])


def test_lag_estimate_parallel_pitch_joints():
    """SO-101-like geometry: pitch joints add, signs differ, IMU sees a vector sum."""
    rng = np.random.default_rng(3)
    fs = 100.0
    t = np.arange(0, 30, 1 / fs)

    def traj(x, seed):
        r = np.random.default_rng(seed)
        return sum(r.uniform(5, 30) * np.sin(2 * np.pi * r.uniform(0.2, 1.2) * x + r.uniform(0, 6))
                   for _ in range(3)) * np.sin(2 * np.pi * x / 20) ** 2

    q = np.stack([traj(t, s) for s in range(5)], axis=1)
    t_imu = np.arange(0, 30, 0.01) + 0.004
    true_lag = 0.0145
    qi = np.stack([traj(t_imu - true_lag, s) for s in range(5)], axis=1)
    w = np.gradient(qi, 0.01, axis=0)
    gyro = np.stack([w[:, 4], w[:, 1] - w[:, 2] + w[:, 3], w[:, 0]], axis=1)
    gyro += rng.normal(0, 0.3, gyro.shape)
    r = estimate_lag(t, q, t_imu, gyro)
    assert r["reliable"]
    assert abs(r["lag_ms"] - 14.5) < 1.5


def test_kabsch_and_body_rates():
    from nalog.alignid import _body_rates, _kabsch
    from nalog.sim import _rotvec
    rng = np.random.default_rng(0)
    R_true = _rotvec([0.3, -0.2, 0.5])
    A = rng.normal(size=(200, 3))
    assert np.allclose(_kabsch(A, A @ R_true.T), R_true, atol=1e-9)
    fs, w = 100.0, np.array([0.4, -1.2, 2.0])                 # constant body rate
    R = np.array([_rotvec(w * k / fs) for k in range(50)])
    assert np.allclose(_body_rates(R, fs)[5:-5], w, atol=1e-3)


def test_point_acceleration_rigid_body():
    """Pure rotation about a fixed axis through the IMU: a point at r sees alpha x r + w x (w x r)."""
    from nalog.accel import imu_to_point
    fs = 100.0
    t = np.arange(0, 3, 1 / fs)
    wz = 2.0 * np.sin(2 * np.pi * 0.5 * t)                     # rad/s about IMU z
    gyro = np.c_[np.zeros_like(t), np.zeros_like(t), np.degrees(wz)]
    acc = np.zeros((len(t), 3))                                # IMU on the axis: no accel
    calib = {"R_gi": np.eye(3).tolist(), "lag_ms": 0.0}
    r = np.array([0.1, 0.0, 0.0])
    tg, om, al, f = imu_to_point(t, gyro, acc, calib, r)
    k = (tg > 0.5) & (tg < 2.5)
    w = np.interp(tg, t, wz)
    a = np.gradient(w, 1 / fs)
    expected = np.c_[-w ** 2 * 0.1, a * 0.1, np.zeros_like(w)]
    assert np.abs(f[k] - expected[k]).max() < 0.02
