"""
observer.py
-----------
Generalized-momentum observer for the external joint torque of the SLAVE
(De Luca & Mattone, ICRA 2003; Haddadin et al., IEEE T-RO 2017).

Why not the static residual τ̂ = G + C q̇ − τ_motor: it equals τ_ext − M q̈, so on a
position-controlled arm the torques that accelerate the links appear as "contact".
In simulation its free-motion error grows with speed (shoulder 0.25 → 1.08 N·m RMS
at 1× → 3× speed), and inside the force loop it destabilised k_of = 0.6.
Adding M q̈ would need a double-differentiated encoder signal.

The momentum observer avoids q̈. With p = M(q) q̇ and
    ṗ = τ_motor + τ_ext + C(q, q̇)ᵀ q̇ − G(q),
the residual
    r[k] = K_O ( p[k] − p[0] − Σ_{i<k} (τ_motor + Cᵀ q̇ − G + r)[i] Δt )
obeys ṙ = K_O (τ_ext − r): a first-order low-pass of the true external
torque with bandwidth K_O (rad/s), independent of the motion.

Only the slave uses it: on the master (current mode) the measured current echoes
the command and carries no new information (docs/derivations, master law).
"""

import numpy as np

from .dynamics import ArmModel


class MomentumObserver:
    def __init__(self, model: ArmModel, dt: float, gain):
        self.model = model
        self.dt = dt
        self.K = np.asarray(gain, dtype=float) * np.ones(model.n)
        self._integral = None
        self._p0 = None
        self.r = np.zeros(model.n)

    def reset(self):
        self._integral = None
        self.r[:] = 0.0

    def update(self, q: np.ndarray, dq: np.ndarray, tau_motor: np.ndarray) -> np.ndarray:
        p = self.model.mass(q) @ dq
        if self._integral is None:
            self._p0 = p.copy()
            self._integral = np.zeros_like(p)
            return self.r.copy()
        beta = self.model.gravity(q) - self.model.coriolis_matrix(q, dq).T @ dq
        self._integral += (tau_motor - beta + self.r) * self.dt
        self.r = self.K * (p - self._p0 - self._integral)
        return self.r.copy()
