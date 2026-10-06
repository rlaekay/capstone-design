"""Explicit MuJoCo distance checks, seeded RRT-Connect, and slow quintic timing.

These are sampled geometric checks, not continuous collision certification or a
hardware controller. All angles are radians, distances metres, and times seconds.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import tempfile
from typing import Any

import mujoco
import numpy as np


def _positive(value: float, name: str) -> float:
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _vector(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain exactly {size} finite numbers")
    return result


def _object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise ValueError(f"Unknown {kind.name}: {name}")
    return result


def compiled_model_sha256(model: mujoco.MjModel) -> str:
    """Hash compiled geometry, assets, joint ranges, exclusions, and dynamics."""
    with tempfile.TemporaryDirectory() as directory:
        file = Path(directory) / "model.mjb"
        mujoco.mj_saveModel(model, str(file), None)
        return hashlib.sha256(file.read_bytes()).hexdigest()


def load_scene(model_path: Path, obstacles: list[dict] | None = None) -> mujoco.MjModel:
    """Compile model plus explicitly specified static world obstacles."""
    if model_path.suffix.lower() == ".mjb":
        if obstacles:
            raise ValueError("Frozen binary models cannot accept extra obstacles; rebuild and validate the scene")
        return mujoco.MjModel.from_binary_path(str(model_path))
    spec = mujoco.MjSpec.from_file(str(model_path))
    names = set()
    for obstacle in obstacles or []:
        name = str(obstacle["name"])
        if not name or name in names or spec.geom(name) is not None:
            raise ValueError(f"Obstacle name must be nonempty and unique: {name}")
        names.add(name)
        shapes = {"box": (mujoco.mjtGeom.mjGEOM_BOX, 3),
                  "sphere": (mujoco.mjtGeom.mjGEOM_SPHERE, 1),
                  "cylinder": (mujoco.mjtGeom.mjGEOM_CYLINDER, 2),
                  "capsule": (mujoco.mjtGeom.mjGEOM_CAPSULE, 2)}
        if obstacle["type"] not in shapes:
            raise ValueError("Static obstacle type must be box, sphere, cylinder or capsule")
        geom_type, count = shapes[obstacle["type"]]
        size = _vector(obstacle["size_m"], count, "obstacle size_m")
        if np.any(size <= 0):
            raise ValueError("Obstacle dimensions must be positive")
        position = _vector(obstacle["position_m"], 3, "obstacle position_m")
        quaternion = _vector(obstacle.get("quaternion_wxyz", [1, 0, 0, 0]), 4, "obstacle quaternion")
        if not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-6):
            raise ValueError("Obstacle quaternion must have unit norm (wxyz)")
        spec.worldbody.add_geom(name=name, type=geom_type, pos=position, quat=quaternion,
                                size=np.pad(size, (0, 3 - count)),
                                rgba=[0.85, 0.3, 0.15, 0.85], contype=1, conaffinity=1)
    return spec.compile()


@dataclass(frozen=True)
class CheckResult:
    valid: bool
    minimum_distance_m: float
    pair: tuple[str, str] | None = None
    reason: str = ""


class CollisionChecker:
    """Check all active robot-environment and explicit robot-self geom pairs.

    MuJoCo's automatic parent/weld contact filtering is deliberately not used.
    Same-body geometry is one rigid manufactured assembly and is skipped. Other
    assembly overlaps must appear as explicit named exclusions in the policy.
    Geom masks select active geoms, but do not disable individual pair checks.
    """

    def __init__(self, model: mujoco.MjModel, joint_names: list[str],
                 fixed_joints: dict[str, float], policy: dict[str, Any],
                 clearance_m: float = 0.003, edge_resolution_rad: float = 0.01,
                 joint_margin_rad: float = 0.005, robot_root_body: str = "base",
                 broad_phase: bool = False):
        self.model = model
        self.data = mujoco.MjData(model)
        self.joint_names = list(joint_names)
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be nonempty and unique")
        self.joint_ids = np.asarray([
            _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in joint_names], dtype=int)
        allowed_joints = (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
        if any(model.jnt_type[i] not in allowed_joints or not model.jnt_limited[i]
               for i in self.joint_ids):
            raise ValueError("Planner requires limited one-DOF joints")
        # All other DOFs must be explicitly fixed: no silently moving environment.
        fixed_ids = {_object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name): value
                     for name, value in fixed_joints.items()}
        if set(fixed_ids) & set(self.joint_ids):
            raise ValueError("A joint cannot be both planned and fixed")
        if set(fixed_ids) | set(self.joint_ids) != set(range(model.njnt)):
            raise ValueError("Every model joint must be planned or explicitly fixed")
        self.baseline_qpos = model.qpos0.copy()
        for jid, value in fixed_ids.items():
            if model.jnt_type[jid] not in allowed_joints:
                raise ValueError("Fixed joints must also have one DOF")
            if not np.isfinite(value):
                raise ValueError("Fixed joint values must be finite")
            if model.jnt_limited[jid] and not (
                    model.jnt_range[jid, 0] <= value <= model.jnt_range[jid, 1]):
                raise ValueError("Fixed joint is outside its model range")
            self.baseline_qpos[model.jnt_qposadr[jid]] = value
        self.qpos_ids = model.jnt_qposadr[self.joint_ids].copy()
        self.clearance = _positive(clearance_m, "clearance_m")
        self.resolution = _positive(edge_resolution_rad, "edge_resolution_rad")
        self.margin = float(joint_margin_rad)
        if not np.isfinite(self.margin) or self.margin < 0:
            raise ValueError("joint_margin_rad must be finite and nonnegative")
        self.lower = model.jnt_range[self.joint_ids, 0] + self.margin
        self.upper = model.jnt_range[self.joint_ids, 1] - self.margin
        if np.any(self.lower >= self.upper):
            raise ValueError("Joint margins leave no planning range")
        self.distance_cap = max(1.0, 2 * self.clearance)
        self.evaluations = 0
        self.policy = policy
        root = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, robot_root_body)
        robot_bodies = {root}
        for body in range(root + 1, model.nbody):
            if int(model.body_parentid[body]) in robot_bodies:
                robot_bodies.add(body)
        body_exclusions = self._named_pairs(
            policy.get("planning_excluded_body_pairs", []), mujoco.mjtObj.mjOBJ_BODY)
        geom_exclusions = self._named_pairs(
            policy.get("planning_excluded_geom_pairs", []), mujoco.mjtObj.mjOBJ_GEOM)
        # Explicit MJCF pairs may activate geoms whose masks are otherwise zero.
        active = np.unique(np.concatenate((np.flatnonzero(model.geom_contype | model.geom_conaffinity),
                                           model.pair_geom1, model.pair_geom2))).astype(int)
        supported = {int(mujoco.mjtGeom.mjGEOM_PLANE), int(mujoco.mjtGeom.mjGEOM_SPHERE),
                     int(mujoco.mjtGeom.mjGEOM_CAPSULE), int(mujoco.mjtGeom.mjGEOM_ELLIPSOID),
                     int(mujoco.mjtGeom.mjGEOM_CYLINDER), int(mujoco.mjtGeom.mjGEOM_BOX),
                     int(mujoco.mjtGeom.mjGEOM_MESH)}
        if any(int(model.geom_type[g]) not in supported for g in active):
            raise ValueError("Collision planner supports primitive/convex-mesh geoms only")
        self.pairs: list[tuple[int, int]] = []
        for index, first in enumerate(active):
            first = int(first)
            b1 = int(model.geom_bodyid[first])
            for second in active[index + 1:]:
                second = int(second)
                b2 = int(model.geom_bodyid[second])
                if b1 not in robot_bodies and b2 not in robot_bodies:
                    continue
                if b1 == b2 or tuple(sorted((b1, b2))) in body_exclusions:
                    continue
                if tuple(sorted((first, second))) in geom_exclusions:
                    continue
                if model.geom_type[first] == model.geom_type[second] == mujoco.mjtGeom.mjGEOM_PLANE:
                    raise ValueError("Unsupported plane-plane collision pair")
                self.pairs.append((first, second))
        if not self.pairs:
            raise ValueError("Collision policy leaves no collision pairs to check")
        self.broad_phase = broad_phase
        self.pair_array = np.asarray(self.pairs, dtype=int)

    def _named_pairs(self, pairs: list, kind: mujoco.mjtObj) -> set[tuple[int, int]]:
        result = set()
        for pair in pairs:
            if len(pair) != 2 or pair[0] == pair[1]:
                raise ValueError("Exclusions must contain two distinct object names")
            result.add(tuple(sorted(_object_id(self.model, kind, name) for name in pair)))
        return result

    def full_qpos(self, q: np.ndarray) -> np.ndarray:
        q = _vector(q, len(self.joint_ids), "configuration")
        full = self.baseline_qpos.copy()
        full[self.qpos_ids] = q
        return full

    def check(self, q: np.ndarray, full_report: bool = False) -> CheckResult:
        q = _vector(q, len(self.joint_ids), "configuration")
        if np.any(q < self.lower) or np.any(q > self.upper):
            return CheckResult(False, float("-inf"), reason="joint limit or joint margin")
        return self.check_full_qpos(self.full_qpos(q), full_report)

    def check_full_qpos(self, qpos: np.ndarray, full_report: bool = False) -> CheckResult:
        """Check measured/simulated poses, including actual jaw position."""
        qpos = _vector(qpos, self.model.nq, "full qpos")
        q = qpos[self.qpos_ids]
        if np.any(q < self.lower) or np.any(q > self.upper):
            return CheckResult(False, float("-inf"), reason="joint limit or joint margin")
        for joint in range(self.model.njnt):
            value = qpos[self.model.jnt_qposadr[joint]]
            if self.model.jnt_limited[joint] and not (
                    self.model.jnt_range[joint, 0] <= value <= self.model.jnt_range[joint, 1]):
                return CheckResult(False, float("-inf"), reason="fixed joint outside model range")
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0
        mujoco.mj_kinematics(self.model, self.data)
        self.evaluations += 1
        minimum, closest = self.distance_cap, None
        order = range(len(self.pairs))
        lower_bounds = np.zeros(len(self.pairs))
        if self.broad_phase:
            # The compiled local AABBs enclose the complete primitive/convex
            # geometry. This optional path requires immutable compiled geometry.
            rotations = self.data.geom_xmat.reshape(-1, 3, 3)
            bounds = self.model.geom_aabb
            centers = self.data.geom_xpos + np.einsum('nij,nj->ni', rotations, bounds[:, :3])
            half = np.einsum('nij,nj->ni', np.abs(rotations), bounds[:, 3:])
            first, second = self.pair_array.T
            gap = np.maximum(np.abs(centers[first]-centers[second])-half[first]-half[second], 0)
            lower_bounds = np.linalg.norm(gap, axis=1)
            planes = self.model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE
            lower_bounds[planes[first] | planes[second]] = 0
            order = np.argsort(lower_bounds, kind='stable')
        for index in order:
            # Overlapping AABBs cannot bound penetration depth; always evaluate.
            if lower_bounds[index] > max(minimum, 0.) + 1e-9:
                break
            first, second = self.pairs[index]
            distance = float(mujoco.mj_geomDistance(
                self.model, self.data, first, second, self.distance_cap, None))
            if not np.isfinite(distance):
                raise ValueError("MuJoCo returned a non-finite geometric distance")
            if distance < minimum:
                minimum = distance
                closest = (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, first)
                           or f"geom_{first}",
                           mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, second)
                           or f"geom_{second}")
            if distance < self.clearance and not full_report:
                return CheckResult(False, minimum, closest, "collision or insufficient clearance")
        valid = minimum >= self.clearance
        return CheckResult(valid, minimum, closest,
                           "" if valid else "collision or insufficient clearance")

    def edge(self, first: np.ndarray, second: np.ndarray) -> bool:
        first = _vector(first, len(self.joint_ids), "edge start")
        second = _vector(second, len(self.joint_ids), "edge goal")
        count = max(1, int(np.ceil(np.max(np.abs(second - first)) / self.resolution)))
        return all(self.check(first + (second - first) * fraction).valid
                   for fraction in np.linspace(0, 1, count + 1))


@dataclass
class _Tree:
    points: list[np.ndarray]
    parents: list[int]
    from_start: bool

    def path(self, index: int) -> list[np.ndarray]:
        result = []
        while index >= 0:
            result.append(self.points[index])
            index = self.parents[index]
        return result[::-1]


def plan_rrt_connect(checker: CollisionChecker, start: np.ndarray, goal: np.ndarray,
                     seed: int = 7, step_rad: float = 0.25,
                     max_iterations: int = 5000, shortcut_attempts: int = 100) -> np.ndarray:
    start = _vector(start, len(checker.lower), "start")
    goal = _vector(goal, len(checker.lower), "goal")
    step = _positive(step_rad, "step_rad")
    if max_iterations < 1 or shortcut_attempts < 0:
        raise ValueError("max_iterations must be positive; shortcut_attempts nonnegative")
    for name, q in (("start", start), ("goal", goal)):
        report = checker.check(q, full_report=True)
        if not report.valid:
            raise ValueError(f"Invalid {name}: {report.reason}, pair={report.pair}, "
                             f"distance={report.minimum_distance_m:.6g} m")
    if checker.edge(start, goal):
        return np.asarray([start, goal])
    rng = np.random.default_rng(seed)
    a, b = _Tree([start], [-1], True), _Tree([goal], [-1], False)

    def extend(tree: _Tree, target: np.ndarray) -> tuple[int | None, bool]:
        distances = np.linalg.norm((np.asarray(tree.points) - target) /
                                   (checker.upper - checker.lower), axis=1)
        nearest = int(np.argmin(distances))
        difference = target - tree.points[nearest]
        distance = float(np.linalg.norm(difference))
        if distance < 1e-12:
            return nearest, True
        candidate = tree.points[nearest] + difference * min(1, step / distance)
        if not checker.edge(tree.points[nearest], candidate):
            return None, False
        tree.points.append(candidate)
        tree.parents.append(nearest)
        return len(tree.points) - 1, distance <= step

    path = None
    for _ in range(max_iterations):
        target = b.points[0] if rng.random() < 0.1 else rng.uniform(checker.lower, checker.upper)
        ia, _ = extend(a, target)
        if ia is not None:
            while True:
                ib, reached = extend(b, a.points[ia])
                if ib is None:
                    break
                if reached:
                    first, second = a.path(ia), b.path(ib)
                    path = first + second[-2::-1] if a.from_start else second + first[-2::-1]
                    break
            if path is not None:
                break
        a, b = b, a
    if path is None:
        raise RuntimeError(f"No sampled collision-free path found in {max_iterations} iterations; "
                           "this does not prove that no path exists")
    for _ in range(shortcut_attempts):
        if len(path) <= 2:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j > i + 1 and checker.edge(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    return np.asarray(path)


def parameterize(waypoints: np.ndarray, max_velocity: np.ndarray,
                 max_acceleration: np.ndarray, sample_period_s: float = 0.02) -> dict[str, np.ndarray]:
    """Quintic segments stop at every waypoint; analytical speed/acceleration bounds."""
    waypoints = np.asarray(waypoints, dtype=float)
    if waypoints.ndim != 2 or len(waypoints) < 2 or not np.all(np.isfinite(waypoints)):
        raise ValueError("At least two finite waypoints are required")
    n = waypoints.shape[1]
    velocity = _vector(max_velocity, n, "max_velocity_rad_s")
    acceleration = _vector(max_acceleration, n, "max_acceleration_rad_s2")
    if np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError("Velocity and acceleration limits must be positive")
    dt = _positive(sample_period_s, "sample_period_s")
    times, positions, speeds, accelerations, durations = [], [], [], [], []
    elapsed = 0.0
    for index, (first, second) in enumerate(zip(waypoints[:-1], waypoints[1:])):
        delta = second - first
        # max(s')=15/8; max(abs(s''))=10/sqrt(3), for s=10u^3-15u^4+6u^5.
        duration = max(dt, float(np.max(1.875 * np.abs(delta) / velocity)),
                       float(np.sqrt(np.max((10 / np.sqrt(3)) * np.abs(delta) / acceleration))))
        duration *= 1.001  # protect against floating point boundary comparisons
        count = max(1, int(np.ceil(duration / dt)))
        u = np.linspace(0, 1, count + 1)
        if index:
            u = u[1:]
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
        dds = (60 * u - 180 * u**2 + 120 * u**3) / duration**2
        times.extend(elapsed + u * duration)
        positions.extend(first + s[:, None] * delta)
        speeds.extend(ds[:, None] * delta)
        accelerations.extend(dds[:, None] * delta)
        durations.append(duration)
        elapsed += duration
    return {"t": np.asarray(times), "q": np.asarray(positions), "qd": np.asarray(speeds),
            "qdd": np.asarray(accelerations), "waypoints": waypoints,
            "segment_durations": np.asarray(durations)}


def validate_trajectory(checker: CollisionChecker, trajectory: dict[str, np.ndarray],
                        max_velocity: np.ndarray, max_acceleration: np.ndarray) -> dict[str, Any]:
    """Recheck every saved pose plus spatially subdivided edges at configured resolution."""
    n = len(checker.joint_ids)
    t = np.asarray(trajectory["t"], dtype=float)
    if t.ndim != 1 or len(t) < 2 or not np.all(np.isfinite(t)) or t[0] != 0 or np.any(np.diff(t) <= 0):
        raise ValueError("Trajectory timestamps must begin at zero and strictly increase")
    for name in ("q", "qd", "qdd"):
        array = np.asarray(trajectory[name], dtype=float)
        if array.shape != (len(t), n) or not np.all(np.isfinite(array)):
            raise ValueError(f"Invalid trajectory {name}")
    velocity = _vector(max_velocity, n, "max_velocity_rad_s")
    acceleration = _vector(max_acceleration, n, "max_acceleration_rad_s2")
    if np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError("Velocity/acceleration limits must be positive")
    if np.any(np.abs(trajectory["qd"]) > velocity + 1e-9):
        raise ValueError("Trajectory exceeds velocity limits")
    if np.any(np.abs(trajectory["qdd"]) > acceleration + 1e-9):
        raise ValueError("Trajectory exceeds acceleration limits")
    sample_velocity = np.diff(trajectory["q"], axis=0) / np.diff(t)[:, None]
    if np.any(np.abs(sample_velocity) > velocity + 1e-9):
        raise ValueError("Trajectory timestamps imply excessive velocity")
    if np.any(np.abs(np.diff(trajectory["qd"], axis=0) / np.diff(t)[:, None]) > acceleration + 1e-9):
        raise ValueError("Trajectory timestamps imply excessive acceleration")
    if "qpos" in trajectory:
        expected = np.asarray([checker.full_qpos(q) for q in trajectory["q"]])
        actual = trajectory["qpos"]
        if actual.shape != expected.shape or not np.allclose(actual, expected, rtol=0, atol=1e-12):
            raise ValueError("Trajectory full qpos differs from planned/fixed joint positions")
    minimum, pair, checked = checker.distance_cap, None, 0
    for index, q in enumerate(trajectory["q"]):
        if index:
            first = trajectory["q"][index - 1]
            count = max(1, int(np.ceil(np.max(np.abs(q - first)) / checker.resolution)))
            configurations = [first + (q - first) * fraction for fraction in np.linspace(0, 1, count + 1)[1:]]
        else:
            configurations = [q]
        for configuration in configurations:
            report = checker.check(configuration, full_report=True)
            checked += 1
            if not report.valid:
                raise ValueError(f"Trajectory fails at sample {index}: {report}")
            if report.minimum_distance_m < minimum:
                minimum, pair = report.minimum_distance_m, report.pair
    return {"validated_configurations": checked, "minimum_sampled_clearance_m": minimum,
            "closest_geom_pair": pair, "distance_reporting_cap_m": checker.distance_cap,
            "collision_pairs_checked_per_configuration": len(checker.pairs),
            "continuous_collision_guarantee": False}


def make_checker(model: mujoco.MjModel, config: dict, policy: dict) -> CollisionChecker:
    return CollisionChecker(model, config["joint_names"], config["fixed_joints"], policy,
                            config["clearance_m"], config["edge_resolution_rad"],
                            config["joint_margin_rad"], config.get("robot_root_body", "base"),
                            config.get('broad_phase', False))


def save_trajectory(path: Path, trajectory: dict[str, np.ndarray], metadata: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Open the requested path explicitly, avoiding np.savez's automatic suffix.
    with path.open("wb") as file:
        np.savez_compressed(file, **trajectory, metadata=np.asarray(json.dumps(metadata)))


def load_trajectory(path: Path) -> tuple[dict[str, np.ndarray], dict]:
    with np.load(path, allow_pickle=False) as archive:
        metadata = json.loads(str(archive["metadata"].item()))
        trajectory = {key: archive[key].copy() for key in archive.files if key != "metadata"}
    if metadata.get("format_version") != 1 or metadata.get("mode") != "simulation_only_kinematic":
        raise ValueError("Unsupported trajectory format or mode")
    return trajectory, metadata
