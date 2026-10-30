"""
force_mapping.py
----------------
Maps the slave's estimated external joint torque onto master joint torque
between two arms of different kinematics.

The RX200 master and VX300s slave have different link lengths, so equal
joint torques are NOT equal end-effector forces. Reflecting τ̂_ext,s joint
by joint (Γ = I) renders a contact force on the
operator's hand that is wrong in magnitude and direction.

Cartesian-consistent mapping:

    F̂_s   = (J_sᵀ)⁺_λ τ̂_ext,s                 slave contact wrench
    τ_f   = J_mᵀ  S_f  F̂_s                    rendered on the master
    (J_sᵀ)⁺_λ = J_s (J_sᵀ J_s + λ² I)⁻¹       damped least squares

S_f = diag(s_force·I3, s_moment·I3) scales force and moment separately.
Γ(q_m, q_s) = J_mᵀ S_f (J_sᵀ)⁺_λ is not power-preserving under the joint-space
position mapping, which is why it must be combined with the energy tank in
passivity.py.
"""

import numpy as np

from .dynamics import ArmModel


class ForceMapping:
    def __init__(self, master: ArmModel, slave: ArmModel, mode: str = "jacobian",
                 s_force: float = 1.0, s_moment: float = 1.0, damping: float = 0.02):
        if mode not in ("jacobian", "identity"):
            raise ValueError("mode must be 'jacobian' or 'identity'")
        self.master = master
        self.slave = slave
        self.mode = mode
        self.S = np.diag([s_force] * 3 + [s_moment] * 3)
        self.lam2 = damping ** 2

    def slave_wrench(self, q_s: np.ndarray, tau_s: np.ndarray) -> np.ndarray:
        J_s = self.slave.jacobian(q_s)
        return J_s @ np.linalg.solve(J_s.T @ J_s + self.lam2 * np.eye(J_s.shape[1]), tau_s)

    def __call__(self, q_m: np.ndarray, q_s: np.ndarray, tau_s: np.ndarray) -> np.ndarray:
        if self.mode == "identity":
            return tau_s.copy()
        F_s = self.slave_wrench(q_s, tau_s)
        return self.master.jacobian(q_m).T @ (self.S @ F_s)
