"""
admittance.py
-------------
Slave admittance filter, per joint:

    M_d q̈_e + D_d q̇_e + K_d q_e = τ_adm

discretised EXACTLY with a zero-order hold on τ_adm:

    x[k+1] = A_d x[k] + B_d τ_adm[k],   x = [q_e, q̇_e]
    A_d = e^{A Δt},   B_d = ∫_0^Δt e^{A s} ds B

The exact discretisation is stable for every Δt whenever M_d, D_d, K_d > 0.
Explicit Euler is not: with the link inertia as M_d (wrist ≈ 1e-4..1e-6 kg·m²)
and K_d = 5..10 N·m/rad, ω_n Δt ≈ 2..4.5 at Δt = 20 ms lies outside its
stability region.

M_d, D_d, K_d are DESIGN parameters (the desired contact behaviour), not the
physical inertia. Choose the bandwidth below the slave position loop:
  ω_n = sqrt(K_d / M_d) ≈ 6–15 rad/s,   ζ = D_d / (2 sqrt(K_d M_d)) ≈ 0.7–1.
"""

import numpy as np
from scipy.linalg import expm


class AdmittanceFilter:
    def __init__(self, M: np.ndarray, D: np.ndarray, K: np.ndarray, dt: float,
                 q_e_max: np.ndarray):
        M, D, K = (np.asarray(x, dtype=float) for x in (M, D, K))
        if np.any(M <= 0) or np.any(D < 0) or np.any(K < 0):
            raise ValueError("Admittance M_d must be > 0 and D_d, K_d >= 0")
        self.n = M.size
        self.q_e_max = np.asarray(q_e_max, dtype=float)
        self.Ad = np.zeros((self.n, 2, 2))
        self.Bd = np.zeros((self.n, 2))
        for i in range(self.n):
            A = np.array([[0.0, 1.0], [-K[i] / M[i], -D[i] / M[i]]])
            B = np.array([0.0, 1.0 / M[i]])
            # Van Loan: expm([[A, B], [0, 0]] dt) = [[A_d, B_d], [0, 1]]
            aug = np.zeros((3, 3))
            aug[:2, :2] = A
            aug[:2, 2] = B
            E = expm(aug * dt)
            self.Ad[i] = E[:2, :2]
            self.Bd[i] = E[:2, 2]
        self.q_e = np.zeros(self.n)
        self.dq_e = np.zeros(self.n)

        self.omega_n = np.sqrt(K / M)
        self.zeta = D / (2.0 * np.sqrt(np.maximum(K * M, 1e-12)))

    def reset(self):
        self.q_e[:] = 0.0
        self.dq_e[:] = 0.0

    def step(self, tau_adm: np.ndarray) -> np.ndarray:
        for i in range(self.n):
            x = self.Ad[i] @ np.array([self.q_e[i], self.dq_e[i]]) + self.Bd[i] * tau_adm[i]
            self.q_e[i], self.dq_e[i] = x
        # Saturate the offset; stop the velocity when pushing into the limit (anti-windup)
        hit = np.abs(self.q_e) >= self.q_e_max
        self.q_e = np.clip(self.q_e, -self.q_e_max, self.q_e_max)
        self.dq_e[hit & (np.sign(self.dq_e) == np.sign(self.q_e))] = 0.0
        return self.q_e.copy()
