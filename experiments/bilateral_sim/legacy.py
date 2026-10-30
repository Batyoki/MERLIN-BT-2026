"""
Faithful re-implementation of the ORIGINAL controller (git HEAD 0658f52:
controller.py + arm_interface.py + misa_mpc_controller.py), so its behaviour
can be measured in the same simulator. Parameter values are the defaults
declared in controller.py (the YAML was never loaded: the launch file asks for
'controllers.yaml', the package ships 'controller.yaml', and setup.py does not
install config/). The slave MPC is replaced by its proven output dq* = 0
(see test_legacy_mpc_returns_zero in run.py).

fix_units=True divides the effort by 1000 (the only change), to show what is
still wrong once the unit bug is gone.
"""

import numpy as np
import pinocchio as pin

# controller.py declared defaults
DMD = np.array([1.0, 2.2, 1.2, 0.3, 0.2])
KMD = np.array([25.0, 60.0, 30.0, 12.0, 6.0])
DSD = np.array([2.0, 3.0, 1.5, 0.6, 0.4])
KSD = np.array([50.0, 80.0, 40.0, 30.0, 10.0])
KOF, KOM = 0.8, 0.8
# slave_Msd from controller.yaml comment (= data.json inertia), arm 5x5 block
MSD = np.array([
    [0.06876, -0.00051, -0.00028, 0.00001, 0.00042],
    [-0.00051, 0.15829, 0.05101, 0.01189, 0.00014],
    [-0.00028, 0.05101, 0.10419, 0.01908, 0.00024],
    [0.00001, 0.01189, 0.01908, 0.00566, 0.00008],
    [0.00042, 0.00014, 0.00024, 0.00008, 0.00073]])
DEAD_ZONE = np.array([0.5, 0.8, 0.5, 0.3, 0.2])
TAU_CLAMP = np.array([3.0, 5.0, 3.0, 1.0, 0.5])          # VIPERX300S_TAU_MAX[:5]
SAFE_TAU_MAX = np.array([2.0, 3.0, 2.0, 0.8, 0.4])
NODE_TAU_MAX = np.array([3.0, 4.0, 3.0, 1.5, 0.5])        # REACTORX150_TAU_MAX[:5]
RX_KT = np.array([1.783, 1.783, 1.783, 1.071, 1.071])     # REACTORX150_KT[:5]
VX_KT = np.array([2.409, 2.409, 2.409, 2.409, 2.409])     # VIPERX300S_KT[:5]
HAPTIC_MA = 0.5 * np.array([100.0, 150.0, 100.0, 50.0, 50.0])
ALPHA_FILTER = 0.15


class LegacyEstimator:
    def __init__(self, model, kt, dt, buggy_slave_gravity: bool, fix_units: bool):
        self.model, self.kt, self.dt = model, kt, dt
        self.buggy = buggy_slave_gravity
        self.scale = 1e-3 if fix_units else 1.0
        self.q_prev = None
        self.dq = np.zeros(5)
        self.tau = np.zeros(5)

    def gravity(self, q):
        if not self.buggy:
            return self.model.gravity(q)
        q8 = np.concatenate([q, np.zeros(3)])
        qf = np.insert(q8, 3, 0.0)          # old get_G for "vx300s"
        G = pin.computeGeneralizedGravity(self.model.model, self.model.data, qf)
        return np.delete(G, 3)[:5]

    def update(self, q, effort_mA):
        dq_raw = np.zeros(5) if self.q_prev is None else (q - self.q_prev) / self.dt
        self.q_prev = q.copy()
        self.dq = ALPHA_FILTER * dq_raw + (1 - ALPHA_FILTER) * self.dq
        self.tau = ALPHA_FILTER * (self.kt * effort_mA * self.scale) + (1 - ALPHA_FILTER) * self.tau
        G = self.gravity(q)
        C = self.model.coriolis(q, self.dq)
        return self.dq, self.tau - (C + G)      # old sign: tau_motor - (C + G)


class LegacyController:
    def __init__(self, master_model, slave_model, dt, fix_units=False):
        self.master, self.dt = master_model, dt
        self.est_m = LegacyEstimator(master_model, RX_KT, dt, False, fix_units)
        self.est_s = LegacyEstimator(slave_model, VX_KT, dt, True, fix_units)
        self.q_e = np.zeros(5)
        self.dq_e = np.zeros(5)
        self.q_int = None
        self.last_cmd = None
        self.Msd_inv = np.linalg.inv(MSD + 1e-4 * np.eye(5))

    def step(self, q_m, I_m_mA, q_s, I_s_mA):
        dq_m, tau_ext_m = self.est_m.update(q_m, I_m_mA)
        dq_s, tau_ext_s = self.est_s.update(q_s, I_s_mA)
        if self.q_int is None:
            self.q_int = q_s.copy()
            self.last_cmd = q_s.copy()
            return np.zeros(5), q_s.copy(), {}

        # admittance (explicit Euler, physical inertia, clamps)
        tau_adm = np.clip(tau_ext_s - KOF * tau_ext_m, -TAU_CLAMP, TAU_CLAMP)
        ddq_e = np.clip(self.Msd_inv @ (tau_adm - DSD * self.dq_e - KSD * self.q_e), -30, 30)
        self.dq_e = self.dq_e + self.dt * ddq_e
        self.q_e = np.clip(self.q_e + self.dt * self.dq_e, -0.05, 0.05)
        self.dq_e = np.clip(self.dq_e, -0.5, 0.5)
        q_ref = q_m + self.q_e

        # master
        tau_s_f = np.abs(tau_ext_s) >= DEAD_ZONE
        tau_s_f = np.sign(tau_ext_s) * np.maximum(np.abs(tau_ext_s) - DEAD_ZONE, 0.0) * tau_s_f
        total = tau_ext_m - KOF * tau_s_f - KOM * KMD * (q_m - q_s)
        tau_m = self.master.gravity(q_m) - DMD * dq_m + total
        tau_m = np.clip(np.clip(tau_m, -SAFE_TAU_MAX, SAFE_TAU_MAX), -NODE_TAU_MAX, NODE_TAU_MAX)
        I_cmd_mA = np.clip(tau_m / RX_KT * 1000.0, -HAPTIC_MA, HAPTIC_MA)

        # slave: dq* = 0 from the MPC, leaky integrator, clip, rate limit
        self.q_int = 0.95 * self.q_int + 0.05 * q_ref
        self.q_int = np.clip(self.q_int, q_s - 0.5, q_s + 0.5)
        cmd = self.last_cmd + np.clip(self.q_int - self.last_cmd, -0.12, 0.12)
        self.last_cmd = cmd
        return I_cmd_mA, cmd, dict(q_ref=q_ref)
