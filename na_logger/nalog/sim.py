"""Simulated SO-101 + Crazyflie, for testing the pipeline and GUI without hardware (--sim).

The follower follows an analytic trajectory (delayed leader, a script, or a held pose), so the
simulated IMU can be computed exactly from the MuJoCo model: gyro and specific force at a hidden
"true" mount (SIM_TRUTH) with bias, noise, sensor delay, Crazyflie clock drift and radio jitter.
alignid and the clock/lag estimates can then be checked against ground truth.
"""
from __future__ import annotations

import bisect
import heapq
import math
import random
import threading
import time

import numpy as np

from . import units
from .accel import G, G_BASE
from .arms import ArmCalib, build_sample
from .clock import now_ns
from .config import Config

SIM_TRUTH = {
    "imu_sensor_delay_ms": 25.0, "radio_min_latency_ms": 2.5, "cf_drift_ppm": 40.0,
    "acc_extra_delay_ms": 4.7,
    "mount_rotvec_rad": [0.10, -0.06, 0.15],          # true mount = nominal * Exp(rotvec)
    "mount_offset_m": [0.004, -0.003, 0.005],          # true chip position = nominal + offset
    "pitch_offset_deg": 3.0,                            # model wrist_flex = logger deg + 3
    "gyro_bias_dps": [0.4, -0.3, 0.2], "acc_bias_g": [0.01, -0.02, 0.015],
    "acc_scale": [1.014, 1.011, 1.012],                 # like the real BMI088 (~1-1.5 % high)
}
HOME = [0.0, -20.0, 30.0, 40.0, 0.0, 10.0]


def _rotvec(v) -> np.ndarray:
    v = np.asarray(v, float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class SimArmSource:
    kind = "sim"

    def __init__(self, cfg: Config, hub, move_s: float = 9.0, idle_s: float = 6.0):
        self.cfg = cfg
        self.hub = hub
        self.ids = list(cfg.servo_ids)
        self.move_s, self.idle_s = move_s, idle_s
        self.fcal = ArmCalib([0] * 6, [4095] * 6)
        self.lcal = ArmCalib([0] * 6, [4095] * 6)
        self.teleop = "active"            # "active" = the simulated arm follows its demo motion
        self.message = "시뮬레이션"
        self.script_t0_ns = None
        self._t_epoch = now_ns()
        self._lock = threading.Lock()
        self._tl_t: list[int] = [self._t_epoch]                  # mode timeline (start times)
        self._tl: list[tuple] = [("teleop", None)]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="sim-arm", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)

    # ------------------------------------------------------------ motion model
    def _leader_deg(self, ts: float) -> list[float]:
        cycle = self.move_s + self.idle_s
        tau = ts % cycle - self.idle_s
        if tau < 0:
            return list(HOME)
        env = math.sin(0.5 * math.pi * min(1.0, tau / 0.8, (self.move_s - tau) / 0.8)) ** 2
        rnd = random.Random(int(ts // cycle))
        amps = [rnd.uniform(10, 30), rnd.uniform(5, 15), rnd.uniform(8, 20),
                rnd.uniform(10, 25), rnd.uniform(10, 30), rnd.uniform(3, 10)]
        freqs = [rnd.uniform(0.2, 0.5) for _ in range(6)]
        return [b + env * a * math.sin(2 * math.pi * f * tau) for b, a, f in zip(HOME, amps, freqs)]

    def q_at(self, t_ns: int) -> list[float]:
        """Follower joint degrees at host time t_ns (exact, for the IMU model)."""
        with self._lock:
            i = max(0, bisect.bisect_right(self._tl_t, t_ns) - 1)
            t_start, (kind, data) = self._tl_t[i], self._tl[i]
        if kind == "teleop":
            return self._leader_deg((t_ns - self._t_epoch) / 1e9 - 0.04)
        if kind == "hold":
            return list(data)
        fn, dur = data
        el = min(dur, max(0.0, (t_ns - t_start) / 1e9 - 0.03))
        return list(fn(el))

    def _switch(self, kind: str, data=None):
        t = now_ns()
        if kind == "hold" and data is None:
            data = self.q_at(t)
        with self._lock:
            self._tl_t.append(t)
            self._tl.append((kind, data))
        return t

    # ------------------------------------------------------------ control API (as ArmManager)
    def command(self, cmd: str):
        if cmd == "start":
            self._switch("teleop")
            self.teleop = "active"
        elif cmd in ("hold", "torque_off"):
            if self.teleop != ("off" if cmd == "torque_off" else "hold"):
                self._switch("hold")
            self.teleop = "off" if cmd == "torque_off" else "hold"

    def start_script(self, goal_fn, duration_s: float):
        self.script_t0_ns = self._switch("script", (goal_fn, float(duration_s)))
        self._script_dur = float(duration_s)
        self.teleop = "script"

    def status(self) -> dict:
        info = {"connected": True, "port": "SIM", "ids": self.ids, "missing": [],
                "calibrated": True}
        prog = None
        if self.teleop == "script" and self.script_t0_ns:
            prog = min(1.0, (now_ns() - self.script_t0_ns) / 1e9 / self._script_dur)
        return {"mode": "sim", "follower": {**info, "voltage": 12.1},
                "leader": {"connected": False, "disabled": True}, "teleop": self.teleop,
                "soft_start": False,
                "message": self.message, "port_notes": {},
                "script_t0_ns": self.script_t0_ns if self.teleop == "script" else None,
                "script_progress": prog}

    def calib_info(self) -> dict:
        return {"follower": {"port": "SIM", **self.fcal.as_dict()},
                "leader": {"port": "SIM", **self.lcal.as_dict()}}

    # ------------------------------------------------------------ loop
    def _loop(self):
        period = 1.0 / self.cfg.arm_rate_hz
        temp = [34.0, 36.0, 35.0, 33.0, 32.0, 31.0]
        seq = 0
        next_t = time.monotonic()
        step = units.POS_DEG_PER_STEP
        mids = self.fcal.mids
        while not self._stop.is_set():
            t0 = now_ns()
            if self.teleop == "script" and (t0 - self.script_t0_ns) / 1e9 > self._script_dur + 0.05:
                self._switch("hold")
                self.teleop = "hold"
            q = self.q_at(t0)
            qa, qb = self.q_at(t0 - 5_000_000), self.q_at(t0 + 5_000_000)
            vel = [(b - a) / 0.01 for a, b in zip(qa, qb)]
            acc = [(b - 2 * c + a) / 0.005 ** 2 for a, b, c in zip(qa, qb, q)]
            goal = self.q_at(t0 + 30_000_000) if self.teleop == "script" else q
            temp = [tt + (0.002 * abs(v) - (tt - 32) * 0.0005) * period for tt, v in zip(temp, vel)]
            blocks = []
            for j in range(6):
                grav = 120 * math.cos(math.radians(q[1] + q[2])) * (j in (1, 2))
                load = int(max(-1000, min(1000, 0.4 * acc[j] / 10 + 2 * vel[j] + grav + random.gauss(0, 8))))
                blocks.append({
                    "torque": 0 if self.teleop == "off" else 1, "accel": 254,
                    "goal": int(round(goal[j] / step + mids[j])),
                    "pos": int(round(q[j] / step + mids[j])),
                    "vel": int(round(vel[j] / units.VEL_DPS_PER_STEP)),
                    "load": load, "volt": 121 - abs(load) // 200, "temp": int(temp[j]),
                    "status": 0, "moving": int(abs(vel[j]) > 2), "cur": int(abs(load) * 0.6),
                })
            time.sleep(0.002)                            # bus transaction
            t1 = now_ns()
            self.hub.push_arm(build_sample(seq, t0, t1, blocks, None, None, None,
                                           self.teleop, self.fcal, None))
            seq += 1
            next_t += period
            d = next_t - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                next_t = time.monotonic()


class SimImuSource:
    """Crazyflie IMU computed from MuJoCo FK of the hidden true mount."""
    kind = "sim"

    def __init__(self, cfg: Config, hub, arm: SimArmSource):
        from .kin import JointMap, Kin
        self.cfg = cfg
        self.hub = hub
        self.arm = arm
        self.state = "connected"
        self.battery_v = 4.05
        self.link_quality = 100.0
        self.has_att = True
        off = [0.0] * 6
        off[3] = SIM_TRUTH["pitch_offset_deg"]
        self.kin = Kin(jmap=JointMap(offset_deg=off))
        R_nom, p_nom = self.kin.nominal_imu()
        self.R = R_nom @ _rotvec(SIM_TRUTH["mount_rotvec_rad"])
        self.p = p_nom + np.asarray(SIM_TRUTH["mount_offset_m"])
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="sim-imu", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)

    def status(self) -> dict:
        return {"mode": "sim", "state": self.state, "uri": "SIM", "message": "시뮬레이션",
                "battery_v": self.battery_v, "link_quality": self.link_quality, "has_att": True}

    def _pose(self, t_ns: int):
        Rbg, pbg = self.kin.gripper_pose(self.arm.q_at(t_ns))
        return Rbg, pbg

    def _gyro_dps(self, t_ns: int) -> np.ndarray:
        h = 2_500_000
        Ra, _ = self._pose(t_ns - h)
        Rb, _ = self._pose(t_ns + h)
        M = Ra.T @ Rb
        S = 0.5 * (M - M.T)
        w_g = np.array([S[2, 1], S[0, 2], S[1, 0]]) / (2 * h / 1e9)
        return np.degrees(self.R.T @ w_g)

    def _acc_g(self, t_ns: int) -> np.ndarray:
        h = 5_000_000
        pts = []
        for dt in (-h, 0, h):
            Rbg, pbg = self._pose(t_ns + dt)
            pts.append(pbg + Rbg @ self.p)
        a = (pts[0] - 2 * pts[1] + pts[2]) / (h / 1e9) ** 2
        Rbg, _ = self._pose(t_ns)
        return (Rbg @ self.R).T @ (a - G_BASE) / G

    def _loop(self):
        period_ns = self.cfg.imu_period_ms * 1_000_000
        delay = int(SIM_TRUTH["imu_sensor_delay_ms"] * 1e6)
        acc_delay = delay + int(SIM_TRUTH["acc_extra_delay_ms"] * 1e6)
        lat_ns = SIM_TRUTH["radio_min_latency_ms"] * 1e6
        drift = SIM_TRUTH["cf_drift_ppm"] * 1e-6
        bg, ba = np.asarray(SIM_TRUTH["gyro_bias_dps"]), np.asarray(SIM_TRUTH["acc_bias_g"])
        boot = now_ns() - 37_000_000_000                # Crazyflie booted 37 s "before" us
        pending: list = []
        last_rx = 0.0
        next_sample = now_ns() + 200_000_000
        rng = np.random.default_rng(7)
        while not self._stop.is_set():
            now = now_ns()
            if now >= next_sample:
                ts = next_sample
                next_sample += period_ns
                gyro = self._gyro_dps(ts - delay) + bg + rng.normal(0, 0.3, 3)
                acc = (self._acc_g(ts - acc_delay) * np.asarray(SIM_TRUTH["acc_scale"]) + ba
                       + rng.normal(0, 0.01, 3))
                Rbi = self._pose(ts - delay)[0] @ self.R
                rpy = [math.degrees(math.atan2(Rbi[2, 1], Rbi[2, 2])),
                       math.degrees(-math.asin(max(-1, min(1, Rbi[2, 0])))),
                       math.degrees(math.atan2(Rbi[1, 0], Rbi[0, 0]))]
                cf_ms = int((ts - boot) * (1 + drift) / 1e6)
                rx = ts + lat_ns + random.expovariate(1 / 3.0e6)
                if random.random() < 0.05:
                    rx += random.uniform(5e6, 25e6)  # occasional radio retry burst
                rx = last_rx = max(rx, last_rx)      # the radio link delivers in order
                heapq.heappush(pending, (rx, cf_ms, acc.tolist(), gyro.tolist(), rpy))
            while pending and pending[0][0] <= now:
                _, cf_ms, acc, gyro, rpy = heapq.heappop(pending)
                t = now_ns()
                self.hub.push_imu({"rx": t, "cf_ms": cf_ms, "acc": acc, "gyro": gyro})
                self.hub.push_att({"rx": t, "cf_ms": cf_ms, "rpy": rpy})
            time.sleep(0.001)
