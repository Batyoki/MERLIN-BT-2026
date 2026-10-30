"""
mpc_optimizer.py
----------------
Constrained, offset-free MPC for the POSITION-CONTROLLED slave.

The VX300s servos accept position commands, so the decision variable is the
position command u, and the model is the identified servo position loop.

Formulation used here
---------------------
Each slave joint i, under its Dynamixel position loop, is modelled (and
identified, tools/identify_position_loop.py) as

    q̈ = ω_i² (u − q) − 2 ζ_i ω_i q̇              u = position command

ZOH-discretised at Δt:  x[k+1] = A_i x[k] + B_i u[k],  x = [q, q̇].

    min_{u_0..u_{N-1}}  Σ_{k=1}^{N-1} Q (q_k − r_k)² + P (q_N − r_N)²
                        + R Σ_{k=0}^{N-1} (u_k − u_{k-1})²
    s.t.  q_min ≤ u_k ≤ q_max
          |u_k − u_{k-1}| ≤ Δu_max
          |q̇_k| ≤ q̇_max                (predicted)
          x_0 = measured state, u_{-1} = last applied command

Offset-free: the xs_sdk position loop has no gravity feed-forward, so each
joint settles at u − G/K_p (static sag), which the linear model cannot
predict. The MPC therefore runs the model in parallel (internal-model form)
and estimates an output disturbance d̂ = LPF(q_meas − q̂_model); predictions
are q̂ + d̂. Without it the model is biased by the static sag (≈ 26 mrad in simulation).

r_k is the admittance reference q_s,ref previewed with the master velocity
(r_k = r_0 + k Δt q̇_m), which lets the MPC cancel the position-loop lag.
Joints are decoupled (diagonal model), so the problem is n small QPs, each
condensed to N variables and solved with a fixed-iteration ADMM (OSQP
scheme). The KKT matrix is constant and factorised once, so a solve is a few
hundred 10×10 mat-vecs (< 1 ms), which fits the 20 ms budget easily.

ρ matters with a fixed iteration budget: ρ = 0.3 brings the warm-started first move
within 0.2 mrad of the exact QP solution in 60 iterations, whereas ρ = 10 leaves it
about 13 mrad away (tests: test/test_derivations.py::test_admm_*).
Only u*_0 is applied (receding horizon).
"""

import time
from typing import Optional

import numpy as np
from scipy.linalg import expm


def discretize_position_loop(omega: float, zeta: float, dt: float):
    A = np.array([[0.0, 1.0], [-omega ** 2, -2.0 * zeta * omega]])
    B = np.array([0.0, omega ** 2])
    aug = np.zeros((3, 3))
    aug[:2, :2] = A
    aug[:2, 2] = B
    E = expm(aug * dt)
    return E[:2, :2], E[:2, 2]


class PositionMPC:
    def __init__(self, omega, zeta, dt: float, horizon: int,
                 q_min, q_max, dq_max, du_max,
                 Q: float = 100.0, P: float = 300.0, R: float = 1.0,
                 rho: float = 0.3, sigma: float = 1e-6, iters: int = 60,
                 dist_alpha: float = 0.2):
        self.n = len(omega)
        self.N = N = horizon
        self.dt = dt
        self.q_min, self.q_max = np.asarray(q_min, float), np.asarray(q_max, float)
        self.dq_max = np.asarray(dq_max, float)
        self.du_max = np.asarray(du_max, float)
        self.rho, self.iters = rho, iters
        self.dist_alpha = dist_alpha
        self.A = np.zeros((self.n, 2, 2))
        self.B = np.zeros((self.n, 2))

        w = np.full(N, Q, dtype=float)
        w[-1] = P
        D = np.eye(N) - np.eye(N, k=-1)            # (D u)_k = u_k − u_{k-1}

        self.Phi_q = np.zeros((self.n, N, 2))
        self.Phi_v = np.zeros((self.n, N, 2))
        self.G_q = np.zeros((self.n, N, N))
        self.G_v = np.zeros((self.n, N, N))
        self.H = np.zeros((self.n, N, N))
        self.C = np.zeros((self.n, 3 * N, N))
        self.Kinv = np.zeros((self.n, N, N))
        for i in range(self.n):
            A, B = discretize_position_loop(omega[i], zeta[i], dt)
            self.A[i], self.B[i] = A, B
            Ak = np.eye(2)
            powers = []
            for k in range(N):
                Ak = A @ Ak
                powers.append(Ak.copy())                  # A^{k+1}
            for k in range(N):
                self.Phi_q[i, k] = powers[k][0]
                self.Phi_v[i, k] = powers[k][1]
                for j in range(k + 1):
                    Akj = np.eye(2) if k == j else powers[k - j - 1]
                    col = Akj @ B
                    self.G_q[i, k, j] = col[0]
                    self.G_v[i, k, j] = col[1]
            Gq = self.G_q[i]
            self.H[i] = 2.0 * (Gq.T @ np.diag(w) @ Gq + R * D.T @ D)
            self.C[i] = np.vstack([np.eye(N), D, self.G_v[i]])
            K = self.H[i] + sigma * np.eye(N) + rho * self.C[i].T @ self.C[i]
            self.Kinv[i] = np.linalg.inv(K)
        self.w, self.D, self.R, self.sigma = w, D, R, sigma

        self._xhat = None       # parallel model state (n, 2)
        self.d_hat = np.zeros(self.n)
        self._u = None          # warm start (n, N)
        self._z = None
        self._y = None
        self.last_solve_ms = 0.0

    def reset(self):
        self._u = self._z = self._y = self._xhat = None
        self.d_hat[:] = 0.0

    def solve(self, q: np.ndarray, dq: np.ndarray, r: np.ndarray,
              u_prev: np.ndarray) -> np.ndarray:
        """
        q, dq   : measured slave joint state (n,)
        r       : reference preview (N, n), r[k] is the target at step k+1
        u_prev  : last applied position command (n,)
        returns : u*_0 (n,)
        """
        t0 = time.perf_counter()
        N = self.N
        if self._u is None:
            self._u = np.tile(u_prev[:, None], (1, N))
            self._z = np.einsum("imn,in->im", self.C, self._u)
            self._y = np.zeros((self.n, 3 * N))
        else:   # shift warm start by one step
            self._u = np.concatenate([self._u[:, 1:], self._u[:, -1:]], axis=1)

        if self._xhat is None:
            self._xhat = np.stack([q, dq], axis=1)
        self.d_hat += self.dist_alpha * ((q - self._xhat[:, 0]) - self.d_hat)
        x0 = self._xhat                                        # (n, 2)
        free_q = np.einsum("ikj,ij->ik", self.Phi_q, x0) + self.d_hat[:, None]   # (n, N)
        free_v = np.einsum("ikj,ij->ik", self.Phi_v, x0)
        d = np.zeros((self.n, N))
        d[:, 0] = u_prev
        f = (2.0 * np.einsum("ikj,ik->ij", self.G_q, self.w * (free_q - r.T))
             - 2.0 * self.R * d @ self.D)                      # (n, N)

        lo = np.concatenate([
            np.tile(self.q_min[:, None], (1, N)),
            d - self.du_max[:, None],
            -self.dq_max[:, None] - free_v], axis=1)
        hi = np.concatenate([
            np.tile(self.q_max[:, None], (1, N)),
            d + self.du_max[:, None],
            self.dq_max[:, None] - free_v], axis=1)

        u, z, y, rho = self._u, np.clip(self._z, lo, hi), self._y, self.rho
        for _ in range(self.iters):
            rhs = self.sigma * u - f + np.einsum("imn,im->in", self.C, rho * z - y)
            u = np.einsum("inm,im->in", self.Kinv, rhs)
            Cu = np.einsum("imn,in->im", self.C, u)
            z = np.clip(Cu + y / rho, lo, hi)
            y = y + rho * (Cu - z)
        self._u, self._z, self._y = u, z, y

        u0 = np.clip(u[:, 0], self.q_min, self.q_max)
        u0 = np.clip(u0, u_prev - self.du_max, u_prev + self.du_max)
        self._xhat = np.einsum("ijk,ik->ij", self.A, self._xhat) + self.B * u0[:, None]
        self.last_solve_ms = (time.perf_counter() - t0) * 1e3
        return u0


def preview_reference(r0: np.ndarray, r_dot: Optional[np.ndarray], dt: float,
                      horizon: int) -> np.ndarray:
    """Constant-velocity preview r_k = r_0 + k Δt ṙ, k = 1..N  -> (N, n)."""
    k = np.arange(1, horizon + 1)[:, None]
    if r_dot is None:
        return np.tile(r0, (horizon, 1))
    return r0[None, :] + k * dt * r_dot[None, :]
