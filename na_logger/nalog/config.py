"""Runtime configuration. Defaults live here; override any field from a TOML file (--config)."""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, field, fields
from pathlib import Path

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


@dataclass
class Config:
    # --- SO-101 (Feetech STS3215 serial bus) ---
    follower_port: str = "auto"         # "auto" or a path such as /dev/cu.usbmodem5A7A0185011
    leader_port: str = "none"           # no leader arm in this project (follower only)
    baudrate: int = 1_000_000
    servo_ids: list[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6])
    joint_names: list[str] = field(default_factory=lambda: list(JOINTS))
    arm_rate_hz: float = 100.0
    follower_min_voltage: float = 8.0   # role auto-detect: follower C018 runs on 12 V, leader on 5 V
    teleop_autostart: bool = False      # start teleop as soon as both arms are found
    teleop_mapping: str = "range"       # "range" (LeRobot default normalisation) | "degrees"
    follower_acceleration: int | None = 254   # RAM register written at teleop start, as LeRobot does
    soft_start_max_dps: float = 60.0    # goal slew limit right after teleop starts
    soft_start_timeout_s: float = 5.0

    # --- Crazyflie IMU ---
    cf_enabled: bool = True
    cf_uri: str = "radio://0/80/2M/E7E7E7E7E8"
    imu_period_ms: int = 10             # firmware log period resolution is 10 ms -> 100 Hz max
    imu_latency_ms: float = 0.0         # constant IMU delay removed in merged.csv (see estimated lag)

    # --- payload force label (F = m (g - a_com), environment-on-robot, base frame) ---
    payload_mass_g: float | None = None # set in the GUI; stored per trial
    accel_point: str = "payload"        # kin.POINTS key: payload | tcp | drone

    # --- trials ---
    auto_trial: bool = True             # start a trial when the follower starts moving
    motion_window_s: float = 0.2
    motion_thresh_deg: float = 1.5      # any joint moving more than this within the window = motion
    idle_stop_s: float = 3.0            # auto trial ends after this long without motion
    pre_roll_s: float = 1.0             # data kept from before the motion that started a trial
    standby_after_s: float = 60.0       # GUI goes to standby after this long without motion

    # --- server ---
    host: str = "0.0.0.0"
    port: int = 8080
    allow_remote_control: bool = False  # teleop/torque buttons only work from the logger Mac itself
    data_dir: str = "data"
    live_window_s: float = 15.0

    @classmethod
    def load(cls, path: str | None = None, **overrides) -> "Config":
        values: dict = {}
        if path:
            with open(path, "rb") as f:
                values.update(tomllib.load(f))
        values.update({k: v for k, v in overrides.items() if v is not None})
        known = {f.name for f in fields(cls)}
        unknown = set(values) - known
        if unknown:
            raise ValueError(f"unknown config keys: {sorted(unknown)}")
        cfg = cls(**values)
        cfg.data_dir = str(Path(cfg.data_dir).expanduser().resolve())
        if len(cfg.joint_names) != len(cfg.servo_ids):
            raise ValueError("joint_names and servo_ids must have the same length")
        return cfg

    def public(self) -> dict:
        return {f.name: getattr(self, f.name) for f in fields(self)}
