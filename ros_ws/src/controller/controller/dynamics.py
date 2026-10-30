"""
dynamics.py
-----------
Rigid-body model of one Interbotix arm, evaluated with pinocchio.

All public methods work on the ACTIVE ARM joints only
(waist, shoulder, elbow, wrist_angle, wrist_rotate). Gripper and finger
joints are held at their neutral value inside the model and never appear
in controller vectors. Joints are mapped BY NAME, so the code is valid for
any URDF that contains these joint names (with or without forearm_roll,
with a continuous or revolute gripper joint).

Joints are matched by name, so a locked forearm_roll is handled without
inserting or deleting vector entries by index.
"""

import os
from typing import Optional, Sequence

import numpy as np
import pinocchio as pin

ARM_JOINTS = ("waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate")


def resolve_urdf(name: str) -> str:
    """
    Find <name>.urdf without absolute, user-specific paths.

    Search order:
      1. $MERLIN_URDF_DIR/<name>.urdf
      2. <ament share dir of 'controller'>/urdf/<name>.urdf
      3. the directory of this file (source checkout)
    """
    candidates = []
    env_dir = os.environ.get("MERLIN_URDF_DIR")
    if env_dir:
        candidates.append(os.path.join(env_dir, f"{name}.urdf"))
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("controller"), "urdf", f"{name}.urdf"))
    except Exception:
        pass
    candidates.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{name}.urdf"))
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(f"URDF '{name}.urdf' not found in: {candidates}")


class ArmModel:
    """Gravity, Coriolis, mass matrix, Jacobian and FK on the active arm joints."""

    def __init__(
        self,
        urdf_path: str,
        arm_joints: Sequence[str] = ARM_JOINTS,
        ee_frame: str = "ee_gripper",
        locked: Optional[dict] = None,
    ):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.arm_joints = tuple(arm_joints)
        self.n = len(self.arm_joints)

        self.idx_q = np.zeros(self.n, dtype=int)
        self.idx_v = np.zeros(self.n, dtype=int)
        for i, name in enumerate(self.arm_joints):
            if not self.model.existJointName(name):
                raise ValueError(f"Joint '{name}' not in {urdf_path}")
            jid = self.model.getJointId(name)
            joint = self.model.joints[jid]
            if joint.nq != 1:
                raise ValueError(f"Arm joint '{name}' must be 1-DOF (nq={joint.nq})")
            self.idx_q[i] = joint.idx_q
            self.idx_v[i] = joint.idx_v

        # Neutral handles continuous joints correctly (cos=1, sin=0)
        self.q_neutral = pin.neutral(self.model)
        for name, value in (locked or {}).items():
            jid = self.model.getJointId(name)
            self.q_neutral[self.model.joints[jid].idx_q] = value

        if not self.model.existFrame(ee_frame):
            raise ValueError(f"Frame '{ee_frame}' not in {urdf_path}")
        self.ee_id = self.model.getFrameId(ee_frame)

    # ------------------------------------------------------------------
    def _full(self, q: np.ndarray, dq: Optional[np.ndarray] = None):
        q_full = self.q_neutral.copy()
        q_full[self.idx_q] = q
        v_full = np.zeros(self.model.nv)
        if dq is not None:
            v_full[self.idx_v] = dq
        return q_full, v_full

    def gravity(self, q: np.ndarray) -> np.ndarray:
        q_full, _ = self._full(q)
        return pin.computeGeneralizedGravity(self.model, self.data, q_full)[self.idx_v].copy()

    def coriolis(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Coriolis/centrifugal torque VECTOR C(q, dq) dq."""
        q_full, v_full = self._full(q, dq)
        nle = pin.nonLinearEffects(self.model, self.data, q_full, v_full)
        g = pin.computeGeneralizedGravity(self.model, self.data, q_full)
        return (nle - g)[self.idx_v].copy()

    def coriolis_matrix(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """C(q, dq) such that C dq is the Coriolis vector and Ṁ − 2C is skew."""
        q_full, v_full = self._full(q, dq)
        C = pin.computeCoriolisMatrix(self.model, self.data, q_full, v_full)
        return C[np.ix_(self.idx_v, self.idx_v)].copy()

    def mass(self, q: np.ndarray) -> np.ndarray:
        q_full, _ = self._full(q)
        M = pin.crba(self.model, self.data, q_full)
        M = np.triu(M) + np.triu(M, 1).T
        return M[np.ix_(self.idx_v, self.idx_v)].copy()

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """(6 x n) end-effector Jacobian, linear part on top, world-aligned axes."""
        q_full, _ = self._full(q)
        J = pin.computeFrameJacobian(
            self.model, self.data, q_full, self.ee_id, pin.LOCAL_WORLD_ALIGNED)
        return J[:, self.idx_v].copy()

    def fk(self, q: np.ndarray) -> np.ndarray:
        """End-effector position in the base frame."""
        q_full, _ = self._full(q)
        pin.framesForwardKinematics(self.model, self.data, q_full)
        return self.data.oMf[self.ee_id].translation.copy()
