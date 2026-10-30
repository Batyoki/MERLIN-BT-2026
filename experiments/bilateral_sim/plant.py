"""
MuJoCo plants for the RX200 master and VX300s slave, built from the same
URDFs the controller uses (meshes stripped, inertials kept), with deliberate
model mismatch (mass scale, joint friction/damping) so the controller's
pinocchio model is not identical to the plant.

The master is torque driven (Dynamixel current mode). The slave is driven by
a Dynamixel-like position loop  τ = sat(K_p (u − q) − K_d q̇)  with no
gravity feed-forward, which is what the xs_sdk position mode does.
"""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ARM = ("waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate")
HOLD = ("gripper", "left_finger", "right_finger")


def load_urdf(path: str, mass_scale: float, damping, frictionloss) -> mujoco.MjModel:
    root = ET.parse(path).getroot()
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                link.remove(e)
    for tag in ("transmission", "gazebo", "material"):
        for e in root.findall(tag):
            root.remove(e)
    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler", fusestatic="false", discardvisual="true")
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    m.opt.timestep = 1e-3
    m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    m.body_mass[:] *= mass_scale
    m.body_inertia[:] *= mass_scale
    for i, name in enumerate(ARM):
        dof = m.joint(name).dofadr[0]
        m.dof_damping[dof] = damping[i]
        m.dof_frictionloss[dof] = frictionloss[i]
    return m


class Arm:
    def __init__(self, urdf: str, q0, mass_scale=1.08,
                 damping=(0.03, 0.05, 0.04, 0.02, 0.01),
                 frictionloss=(0.04, 0.08, 0.06, 0.03, 0.01), ee_body: str = None):
        self.m = load_urdf(urdf, mass_scale, damping, frictionloss)
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in ARM])
        self.vadr = np.array([self.m.joint(n).dofadr[0] for n in ARM])
        self.hold_v = np.array([self.m.joint(n).dofadr[0] for n in HOLD])
        self.hold_q = np.array([self.m.joint(n).qposadr[0] for n in HOLD])
        self.ee = self.m.body(ee_body).id
        self.d.qpos[self.qadr] = q0
        mujoco.mj_forward(self.m, self.d)
        self._jacp = np.zeros((3, self.m.nv))

    @property
    def q(self):
        return self.d.qpos[self.qadr].copy()

    @property
    def dq(self):
        return self.d.qvel[self.vadr].copy()

    def ee_pos(self):
        return self.d.xpos[self.ee].copy()

    def gravity_true(self):
        """Exact plant gravity torque (for the 'felt force' metric only)."""
        d2 = mujoco.MjData(self.m)
        d2.qpos[:] = self.d.qpos
        mujoco.mj_forward(self.m, d2)
        return d2.qfrc_bias[self.vadr].copy()

    def ee_jacobian(self):
        mujoco.mj_jacBody(self.m, self.d, self._jacp, None, self.ee)
        return self._jacp[:, self.vadr].copy()

    def step(self, tau_arm: np.ndarray, f_ee: np.ndarray = None):
        self.d.qfrc_applied[:] = 0.0
        self.d.qfrc_applied[self.vadr] = tau_arm
        # gripper/fingers held still
        self.d.qfrc_applied[self.hold_v] = (-20.0 * (self.d.qpos[self.hold_q] - 0.0)
                                            - 0.5 * self.d.qvel[self.hold_v])
        self.d.qfrc_applied[self.hold_v] += self.d.qfrc_bias[self.hold_v]
        if f_ee is not None:
            mujoco.mj_applyFT(self.m, self.d, f_ee, np.zeros(3), self.ee_pos(),
                              self.ee, self.d.qfrc_applied)
        mujoco.mj_step(self.m, self.d)


class DynamixelPositionLoop:
    """xs_sdk position mode: PD on the motor, saturated at stall torque, no gravity FF."""

    def __init__(self, Kp, Kd, tau_max):
        self.Kp, self.Kd, self.tau_max = map(np.asarray, (Kp, Kd, tau_max))

    def __call__(self, u, q, dq):
        return np.clip(self.Kp * (u - q) - self.Kd * dq, -self.tau_max, self.tau_max)


class Wall:
    """Plane x = x_w in the slave base frame; penalty contact gives the TRUE force."""

    def __init__(self, x_w: float, k: float = 2000.0, b: float = 30.0):
        self.x_w, self.k, self.b = x_w, k, b

    def force(self, p: np.ndarray, v: np.ndarray) -> np.ndarray:
        pen = p[0] - self.x_w
        if pen <= 0.0:
            return np.zeros(3)
        return np.array([-max(self.k * pen + self.b * v[0], 0.0), 0.0, 0.0])
