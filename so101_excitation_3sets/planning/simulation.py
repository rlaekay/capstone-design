"""Physics replay of the saved geometric path using MuJoCo position actuators."""
from __future__ import annotations

from collections.abc import Callable
import time
from typing import Any

import mujoco
import numpy as np

from .core import CollisionChecker


def simulate_trajectory(model: mujoco.MjModel, checker: CollisionChecker,
                        trajectory: dict[str, np.ndarray],
                        sync: Callable[[], bool] | None = None,
                        data: mujoco.MjData | None = None,
                        realtime_speed: float = 1.0,
                        tracking_tolerance_rad: float = 0.15,
                        observe: Callable[[mujoco.MjData, np.ndarray], None] | None = None) -> dict[str, Any]:
    """Step dynamics and check actual geometry each step, including the actual jaw.

    The geometric trajectory is only a reference: finite servo stiffness means
    the simulated robot can depart from it. A failed replay is reported as such.
    """
    if model.nu != model.njnt:
        raise ValueError("Physics replay requires exactly one position actuator per joint")
    actuator_qpos = []
    for actuator in range(model.nu):
        if model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT:
            raise ValueError("Physics replay requires joint position actuators")
        joint = int(model.actuator_trnid[actuator, 0])
        gain = model.actuator_gainprm[actuator, 0]
        # A motor has bias position coefficient 0, whereas position servo is -kp.
        if (gain <= 0 or not np.isclose(model.actuator_biasprm[actuator, 1], -gain)
                or model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_AFFINE
                or not np.allclose(model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])):
            raise ValueError("Physics replay requires simple unit-gear position servos; use scene.xml")
        actuator_qpos.append(int(model.jnt_qposadr[joint]))
    if len(set(actuator_qpos)) != model.njnt:
        raise ValueError("Each joint must have its own position servo")
    if not np.isfinite(tracking_tolerance_rad) or tracking_tolerance_rad <= 0:
        raise ValueError("tracking_tolerance_rad must be positive")
    data = data if data is not None else mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = trajectory["qpos"][0]
    data.ctrl[:] = data.qpos[actuator_qpos]
    mujoco.mj_forward(model, data)
    if observe is not None:
        observe(data, trajectory['qpos'][0])
    maximum_error = np.zeros(model.nq)
    maximum_velocity = np.zeros(model.nv)
    maximum_acceleration = np.zeros(model.nv)
    minimum = checker.distance_cap
    closest = None
    count = 0
    begun = time.monotonic()
    next_visualization = 0.0
    duration = float(trajectory["t"][-1])
    stopped = None
    initial_warnings = np.asarray([warning.number for warning in data.warning]).copy()
    maximum_steps = int(np.ceil(duration / model.opt.timestep)) + 2
    while data.time < duration:
        desired = np.asarray([np.interp(data.time, trajectory["t"], trajectory["qpos"][:, j])
                              for j in range(model.nq)])
        data.ctrl[:] = desired[actuator_qpos]
        mujoco.mj_step(model, data)
        desired_now = np.asarray([np.interp(data.time, trajectory["t"], trajectory["qpos"][:, j])
                                  for j in range(model.nq)])
        data.ctrl[:] = desired_now[actuator_qpos]
        # mj_step integrates qpos/qvel after computing sensors. Refresh derived
        # quantities so time, pose, acceleration and sensor rows are synchronous.
        mujoco.mj_forward(model, data)
        count += 1
        warnings = np.asarray([warning.number for warning in data.warning])
        if np.any(warnings > initial_warnings):
            stopped = "MuJoCo reported a simulation warning"
            break
        if count > maximum_steps:
            stopped = "simulation time failed to advance as expected"
            break
        if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
                and np.all(np.isfinite(data.qacc))):
            stopped = "non-finite simulated state"
            break
        if observe is not None:
            observe(data, desired_now)
        maximum_error = np.maximum(maximum_error, np.abs(data.qpos - desired_now))
        maximum_velocity = np.maximum(maximum_velocity, np.abs(data.qvel))
        maximum_acceleration = np.maximum(maximum_acceleration, np.abs(data.qacc))
        report = checker.check_full_qpos(data.qpos, full_report=True)
        if report.minimum_distance_m < minimum:
            minimum, closest = report.minimum_distance_m, report.pair
        if not report.valid:
            stopped = f"actual pose violates clearance/joint limits: {report.reason}, {report.pair}"
            break
        if np.max(maximum_error) > tracking_tolerance_rad:
            stopped = "simulated servo tracking error exceeds configured tolerance"
            break
        if sync is not None and data.time >= next_visualization:
            if not sync():
                stopped = "viewer closed before replay completed"
                break
            target_time = begun + data.time / realtime_speed
            delay = target_time - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, 0.05))
            next_visualization += 1 / 60
    return {"mode": "simulation_only_position_servo_physics",
            "completed": stopped is None, "stopped_reason": stopped,
            "simulated_seconds": float(data.time), "physics_steps": count,
            "collision_check_period_s": float(model.opt.timestep),
            "minimum_sampled_clearance_m": minimum, "closest_geom_pair": closest,
            "joint_order": [model.joint(j).name for j in range(model.njnt)],
            "max_absolute_tracking_error_rad": maximum_error.tolist(),
            "max_absolute_velocity_rad_s": maximum_velocity.tolist(),
            "max_absolute_acceleration_rad_s2": maximum_acceleration.tolist(),
            "tracking_tolerance_rad": tracking_tolerance_rad,
            "continuous_collision_guarantee": False,
            "hardware_safety_certification": False}
