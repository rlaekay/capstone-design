"""SO-101 + Crazyflie kinematics on the user's MuJoCo model (model/scene.xml).

Frames
  base     : MuJoCo world = robot base (z up)
  gripper  : body "gripper" (fixed jaw, after wrist_roll); the Crazyflie is rigidly attached here
  imu      : Crazyflie body frame (x front, y left, z up) at the BMI088 chip (U9)
Calibration (alignid) gives R_gi (imu -> gripper rotation) and p_gi (chip position in gripper).

Real joint angles (logger degrees, LeRobot convention) map to model qpos as
  qpos[j] = sign[j] * radians(deg[j] + offset_deg[j])
"""
from __future__ import annotations

import json
import threading
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

MODEL_DIR = Path(__file__).resolve().parent.parent / "model"
SCENE = MODEL_DIR / "scene.xml"
# BMI088 (U9) centre relative to the Crazyflie board centre, in the Crazyflie body frame [m]
# (measured from bitcraze cf2.1_component_placement.pdf, see docs/crazyflie_imu_frame.svg)
BMI088_IN_CF = np.array([0.0, 0.0046, 0.0005])
# named points for acceleration conversion -> model site. The force label F = m (g - a_com)
# needs the payload's centre of mass; in the user's model payload_com sits at gripperframe
# (between the jaws). Use "drone_com" if the payload is the Crazyflie itself (+ weights on it).
POINTS = {"payload": "payload_com", "tcp": "gripperframe", "drone": "drone_com"}


def quat_to_mat(q) -> np.ndarray:
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(q, float))
    return m.reshape(3, 3)


def mat_to_quat(R) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def rot_angle_deg(R) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


@dataclass
class JointMap:
    sign: list[float] = field(default_factory=lambda: [1.0] * 6)
    offset_deg: list[float] = field(default_factory=lambda: [0.0] * 6)

    def qpos(self, deg) -> np.ndarray:
        d = np.asarray(deg, float)
        return np.asarray(self.sign) * np.radians(d + np.asarray(self.offset_deg))

    def deg(self, qpos) -> np.ndarray:
        return np.degrees(np.asarray(qpos) / np.asarray(self.sign)) - np.asarray(self.offset_deg)


class Kin:
    """Thread-safe: the MjModel is shared, every thread gets its own MjData."""

    def __init__(self, path: Path = SCENE, jmap: JointMap | None = None):
        self.path = Path(path)
        self.m = mujoco.MjModel.from_xml_path(str(self.path))
        self.jmap = jmap or JointMap()
        self._tls = threading.local()
        self.gripper_id = self.m.body("gripper").id
        self.cf_id = self.m.body("crazyflie").id
        self.cf_nominal = (self.m.body_pos[self.cf_id].copy(), self.m.body_quat[self.cf_id].copy())
        # the model's imu site sits at the board centre; the real BMI088 is 4.6 mm off it
        self.imu_site = self.m.site("imu").id
        self.m.site_pos[self.imu_site] = BMI088_IN_CF
        self.jnt_range_deg = np.degrees(self.m.jnt_range.copy())

    @property
    def d(self) -> mujoco.MjData:
        d = getattr(self._tls, "d", None)
        if d is None:
            d = self._tls.d = mujoco.MjData(self.m)
        return d

    # ------------------------------------------------------------- mounting
    def nominal_imu(self) -> tuple[np.ndarray, np.ndarray]:
        """(R_gi, p_gi) of the model's nominal mount with the IMU at the BMI088 position."""
        pos, quat = self.cf_nominal
        R = quat_to_mat(quat)
        return R, pos + R @ BMI088_IN_CF

    def set_imu_mount(self, R_gi, p_gi) -> None:
        """Place the Crazyflie body so its IMU chip sits at (R_gi, p_gi) in the gripper frame."""
        R_gi = np.asarray(R_gi, float)
        self.m.body_pos[self.cf_id] = np.asarray(p_gi, float) - R_gi @ BMI088_IN_CF
        self.m.body_quat[self.cf_id] = mat_to_quat(R_gi)

    # ------------------------------------------------------------- FK
    def fk(self, deg) -> mujoco.MjData:
        d = self.d
        d.qpos[:] = self.jmap.qpos(deg)
        mujoco.mj_kinematics(self.m, d)
        return d

    def gripper_pose(self, deg) -> tuple[np.ndarray, np.ndarray]:
        d = self.fk(deg)
        return d.xmat[self.gripper_id].reshape(3, 3).copy(), d.xpos[self.gripper_id].copy()

    def gripper_traj(self, deg_rows: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """R_bg (N,3,3), p_bg (N,3) for each row of joint degrees."""
        n = len(deg_rows)
        R = np.empty((n, 3, 3))
        p = np.empty((n, 3))
        for k, row in enumerate(deg_rows):
            R[k], p[k] = self.gripper_pose(row)
        return R, p

    def site_in_gripper(self, site: str) -> np.ndarray:
        """Position of a model site expressed in the gripper frame (constant). Sites on the
        Crazyflie body follow its (calibrated) mount."""
        sid = self.m.site(site).id
        body = self.m.site_bodyid[sid]
        if body == self.gripper_id:
            return self.m.site_pos[sid].copy()
        if body == self.cf_id:
            R = quat_to_mat(self.m.body_quat[self.cf_id])
            return self.m.body_pos[self.cf_id] + R @ self.m.site_pos[sid]
        raise ValueError(f"site {site} is not on the gripper or the Crazyflie")

    def body_poses(self, deg) -> np.ndarray:
        """(nbody, 7) [x y z qw qx qy qz] world poses, for the 3D viewer."""
        d = self.fk(deg)
        return np.c_[d.xpos, d.xquat]

    # ------------------------------------------------------------- collisions
    def collisions(self, deg_rows, margin: float = 0.008, stride: int = 1) -> list[dict]:
        """Contacts closer than `margin` along a joint trajectory. Ignored: base-floor, and
        contacts among gripper / moving jaw / Crazyflie (the gripper joint never moves in these
        trajectories, and the drone is taped straight onto the fixed jaw)."""
        m, d = self.m, self.d
        old_margin = m.geom_margin.copy()
        floor = m.geom("floor").id
        base = m.body("base").id
        rigid = {self.gripper_id, self.cf_id}
        for name in ("moving_jaw_so101_v1",):
            try:
                rigid.add(m.body(name).id)
            except KeyError:
                pass
        try:
            m.geom_margin[:] = margin
            hits = []
            for k in range(0, len(deg_rows), stride):
                d.qpos[:] = self.jmap.qpos(deg_rows[k])
                mujoco.mj_forward(m, d)
                for c in d.contact[: d.ncon]:
                    b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                    if {c.geom1, c.geom2} & {floor} and base in (b1, b2):
                        continue
                    if b1 in rigid and b2 in rigid:
                        continue
                    # Crazyflie propeller/motor/arm cylinders: the real arm ran alignid paths that
                    # put these 15 mm into the wrist in the model without touching anything, so
                    # only the drone's body box is checked
                    n1, n2 = m.geom(c.geom1).name, m.geom(c.geom2).name
                    if any(n.startswith(("cf_collision_prop", "cf_collision_motor",
                                         "cf_collision_arm")) for n in (n1, n2)):
                        continue
                    if c.dist < margin:
                        hits.append({"index": k, "geom1": m.geom(c.geom1).name,
                                     "geom2": m.geom(c.geom2).name, "dist_mm": round(c.dist * 1000, 1)})
                if hits:
                    break
            return hits
        finally:
            m.geom_margin[:] = old_margin


def load_calib(path: Path) -> dict | None:
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def kin_from_calib(calib: dict | None) -> Kin:
    """Kin with the calibrated joint map and Crazyflie mount applied (nominal if calib is None)."""
    jm = JointMap(**calib["joint_map"]) if calib and calib.get("joint_map") else JointMap()
    k = Kin(jmap=jm)
    if calib:
        k.set_imu_mount(calib["R_gi"], calib["p_gi"])
    return k
