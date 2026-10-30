"""
Numerical checks of the statements proved in docs/derivations/merlin_derivations.tex.
Each test names the lemma / proposition it backs. Run: python3 -m pytest test/test_derivations.py
"""

import os

import numpy as np
import pinocchio as pin
import pytest
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.optimize import minimize

from controller.admittance import AdmittanceFilter
from controller.dynamics import ArmModel
from controller.force_mapping import ForceMapping
from controller.mpc_optimizer import PositionMPC
from controller.passivity import EnergyTank

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "controller")


@pytest.fixture(scope="module")
def vx():
    return ArmModel(os.path.join(URDF, "vx300s.urdf"))


@pytest.fixture(scope="module")
def rx():
    return ArmModel(os.path.join(URDF, "rx200.urdf"))


# --------------------------------------------------------------------------
# Model identities (Assumption 1)
# --------------------------------------------------------------------------
def test_inverse_dynamics_decomposition(vx):
    """M a + C(q,v) v + G = tau (rnea): ArmModel.mass/coriolis/gravity are consistent."""
    rng = np.random.default_rng(1)
    for _ in range(20):
        q, v, a = rng.uniform(-1, 1, 5), rng.normal(0, 1, 5), rng.normal(0, 2, 5)
        q_full, v_full = vx._full(q, v)
        a_full = np.zeros(vx.model.nv)
        a_full[vx.idx_v] = a
        tau = pin.rnea(vx.model, vx.data, q_full, v_full, a_full)[vx.idx_v]
        assert np.allclose(vx.mass(q) @ a + vx.coriolis(q, v) + vx.gravity(q), tau, atol=1e-8)


def test_Mdot_equals_C_plus_Ct(vx):
    """Assumption 1, used by the observer theorem:  dM/dt = C + C^T  (equivalently Mdot - 2C skew)."""
    rng = np.random.default_rng(2)
    eps = 1e-6
    for _ in range(10):
        q, v = rng.uniform(-1, 1, 5), rng.normal(0, 1, 5)
        Mdot = (vx.mass(q + eps * v) - vx.mass(q - eps * v)) / (2 * eps)
        C = vx.coriolis_matrix(q, v)
        assert np.allclose(Mdot, C + C.T, atol=1e-6)
        S = Mdot - 2 * C
        assert np.allclose(S, -S.T, atol=1e-6)


def test_static_residual_equals_tau_ext_minus_Mqdd(vx):
    """Static-residual lemma: G + C v - tau_m = tau_ext - M a (exact, from the dynamics)."""
    rng = np.random.default_rng(3)
    q, v = rng.uniform(-1, 1, 5), rng.normal(0, 1, 5)
    tau_m, tau_ext = rng.normal(0, 1, 5), rng.normal(0, 1, 5)
    a = np.linalg.solve(vx.mass(q), tau_m + tau_ext - vx.coriolis(q, v) - vx.gravity(q))
    static = vx.gravity(q) + vx.coriolis(q, v) - tau_m
    assert np.allclose(static, tau_ext - vx.mass(q) @ a, atol=1e-9)


# --------------------------------------------------------------------------
# External-torque estimation: the discrete momentum observer is the IIR filter a = K dt
# --------------------------------------------------------------------------
def test_discrete_observer_recursion_is_exact_iir():
    """
    Discrete-observer proposition: if the sampled momentum balance
    p[k]-p[k-1] = dt (tau_m - beta + tau_ext)[k] holds (zero quadrature
    error), the observer obeys r[k] = (1 - K dt) r[k-1] + K dt tau_ext[k] EXACTLY.
    A one-joint stub model (M = 1, C = 0, G(q) = 0.3 q) makes p = dq so the balance can be imposed.
    """
    from controller.observer import MomentumObserver

    class Stub:
        n = 1
        def mass(self, q): return np.array([[1.0]])
        def gravity(self, q): return 0.3 * np.asarray(q, float)
        def coriolis_matrix(self, q, dq): return np.zeros((1, 1))

    dt, K = 0.02, 15.0
    obs = MomentumObserver(Stub(), dt, K)
    rng = np.random.default_rng(9)
    q = rng.normal(0, 1, 80)
    tau_m = rng.normal(0, 1, 80)
    tau_ext = np.cumsum(rng.normal(0, 0.1, 80)) + 0.5
    dq = np.zeros(80)
    r_prev, a = 0.0, K * dt
    obs.update(np.array([q[0]]), np.array([dq[0]]), np.array([tau_m[0]]))      # initialises p0
    for k in range(1, 80):
        dq[k] = dq[k - 1] + dt * (tau_m[k] - 0.3 * q[k] + tau_ext[k])           # beta = G - C^T dq = 0.3 q
        r = obs.update(np.array([q[k]]), np.array([dq[k]]), np.array([tau_m[k]]))[0]
        assert r == pytest.approx((1 - a) * r_prev + a * tau_ext[k], abs=1e-10)
        r_prev = r
    assert 0 < a < 2


def test_observer_converges_to_constant_external_torque(vx):
    """
    Observer theorem: for a constant tau_ext the estimate converges to it (final value within 3 %).
    Plant: motor torque recomputed every 1 ms (gravity compensation + damping + a stabilising spring, so the arm
    settles); observer sampled at 50 Hz from the same signals.
    """
    from controller.observer import MomentumObserver
    dt, sim_dt = 0.02, 1e-3
    obs = MomentumObserver(vx, dt, 15.0)
    q0 = np.array([0.0, -0.2, 0.3, 0.5, 0.0])
    q, dq = q0.copy(), np.zeros(5)
    M0 = vx.mass(q0)
    D, Ks = 200.0 * M0, 400.0 * M0                         # M^-1 D = 200 I, M^-1 K = 400 I
    tau_ext = np.array([0.0, 1.0, -0.5, 0.2, 0.0])
    r = None
    for _ in range(300):                                    # 6 s
        for i in range(int(dt / sim_dt)):
            tau_m = vx.gravity(q) - D @ dq - Ks @ (q - q0)
            if i == 0:
                r = obs.update(q, dq, tau_m)
            a = np.linalg.solve(vx.mass(q), tau_m + tau_ext - vx.coriolis(q, dq) - vx.gravity(q))
            dq = dq + sim_dt * a
            q = q + sim_dt * dq
    assert np.allclose(r[1:4], tau_ext[1:4], rtol=0.03, atol=0.01)


# --------------------------------------------------------------------------
# Slave admittance: exact ZOH discretisation, Euler instability with the physical inertia
# --------------------------------------------------------------------------
def test_zoh_matches_fine_integration():
    """Exact-ZOH theorem: the ZOH step equals numerical integration of the ODE under constant input."""
    M, D, K, dt = 0.3, 6.0, 30.0, 0.02
    f = AdmittanceFilter(np.array([M]), np.array([D]), np.array([K]), dt, np.array([10.0]))
    x = np.array([0.02, -0.1])
    u = 1.7
    sol = solve_ivp(lambda t, y: [y[1], (u - D * y[1] - K * y[0]) / M], [0, dt], x, rtol=1e-12, atol=1e-14)
    exact = sol.y[:, -1]
    f.q_e[0], f.dq_e[0] = x
    f.step(np.array([u]))
    assert np.allclose([f.q_e[0], f.dq_e[0]], exact, atol=1e-10)


def test_euler_unstable_with_physical_wrist_inertia():
    """Stability proposition and remark: wrist-rotate link inertia 7.29e-4 as M_d, D=0.4, K=10 at dt=20 ms."""
    M, D, K, dt = 7.29e-4, 0.4, 10.0, 0.02
    lam = np.roots([M, D, K])
    assert np.all(lam.real < 0)                                  # the continuous system is stable
    euler_mult = np.abs(1 + lam * dt)
    zoh_mult = np.abs(np.exp(lam * dt))
    assert np.max(euler_mult) > 5.0                              # explicit Euler diverges (|1+lambda dt| >> 1)
    assert np.max(zoh_mult) < 1.0                                # exact discretisation is stable
    A = np.array([[0, 1], [-K / M, -D / M]])
    assert np.max(np.abs(np.linalg.eigvals(expm(A * dt)))) < 1.0


def test_admittance_static_deflection_is_tau_over_K():
    f = AdmittanceFilter(np.array([0.3]), np.array([6.0]), np.array([30.0]), 0.02, np.array([10.0]))
    for _ in range(600):
        f.step(np.array([3.0]))
    assert f.q_e[0] == pytest.approx(3.0 / 30.0, rel=1e-6)


# --------------------------------------------------------------------------
# Master law: a command that feeds back the master's own current integrates itself
# --------------------------------------------------------------------------
def test_master_law_with_own_estimate_has_a_pole_at_one():
    """Design-rule lemma: unit-gain loop (pole at z = 1): a constant bias b makes tau_cmd ramp by alpha*b per sample."""
    alpha, b = 0.15, 0.01                                        # IIR coefficient of the measured current
    tau_cmd, tau_meas_filt = 0.0, 0.0
    hist = []
    for k in range(400):
        # measured current = last command, IIR filtered
        tau_meas_filt = alpha * tau_cmd + (1 - alpha) * tau_meas_filt
        # law G - D dq + (tau_meas - C - G) + tau_f, i.e. tau_meas + b
        tau_cmd = tau_meas_filt + b
        hist.append(tau_cmd)
    slope = np.diff(hist[-50:]).mean()
    assert slope == pytest.approx(alpha * b, rel=1e-2)           # grows linearly (alpha*b per sample): an integrator
    assert hist[-1] == pytest.approx(alpha * b * 400, rel=0.05)


# --------------------------------------------------------------------------
# Force reflection between different arms
# --------------------------------------------------------------------------
def test_dls_wrench_reproduces_joint_torque(vx):
    """Damped-least-squares proposition: J^T F_hat = tau for F_hat = J (J^T J + lam^2 I)^-1 tau, up to O(lam^2)."""
    rng = np.random.default_rng(4)
    q = rng.uniform(-0.5, 0.5, 5)
    tau = rng.normal(0, 1, 5)
    fm = ForceMapping(vx, vx, "jacobian", damping=1e-6)
    F = fm.slave_wrench(q, tau)
    assert np.allclose(vx.jacobian(q).T @ F, tau, atol=1e-6)


def test_identical_arms_give_identity_map(vx):
    """Consistency proposition."""
    q = np.array([0.1, -0.3, 0.4, 0.5, 0.0])
    tau = np.array([0.2, -1.0, 0.7, 0.1, 0.0])
    fm = ForceMapping(vx, vx, "jacobian", damping=1e-6)
    assert np.allclose(fm(q, q, tau), tau, atol=1e-5)


def test_different_arms_over_render_joint_torque_reflection(rx, vx):
    """Consistency proposition, second part: same joint torque => larger Cartesian force on the shorter RX200."""
    q = np.array([0.0, -0.3, 0.3, 0.6, 0.0])
    reach_ratio = np.linalg.norm(vx.fk(q)[[0, 2]]) / np.linalg.norm(rx.fk(q)[[0, 2]])
    assert 1.25 < reach_ratio < 1.4
    tau = np.array([0.0, 1.0, 0.5, 0.1, 0.0])
    F_identity = np.linalg.norm(np.linalg.pinv(rx.jacobian(q).T) @ tau)      # force the operator would feel
    F_true = np.linalg.norm(np.linalg.pinv(vx.jacobian(q).T) @ tau)          # force that made the slave torque
    assert F_identity / F_true > 1.15


# --------------------------------------------------------------------------
# Slave MPC: the QP, ADMM and offset-free tracking
# --------------------------------------------------------------------------
def _reference_qp(mpc, i, x0, r, u_prev, d_hat):
    """Solve the same per-joint QP with SLSQP from the problem data (independent of the ADMM code)."""
    N = mpc.N
    W = np.diag(mpc.w)
    free_q = mpc.Phi_q[i] @ x0 + d_hat
    free_v = mpc.Phi_v[i] @ x0
    Gq, Gv, D = mpc.G_q[i], mpc.G_v[i], mpc.D
    d0 = np.zeros(N)
    d0[0] = u_prev

    def cost(U):
        e = Gq @ U + free_q - r
        du = D @ U - d0
        return e @ W @ e + mpc.R * du @ du

    cons = [
        {"type": "ineq", "fun": lambda U: D @ U - d0 + mpc.du_max[i]},
        {"type": "ineq", "fun": lambda U: mpc.du_max[i] - (D @ U - d0)},
        {"type": "ineq", "fun": lambda U: mpc.dq_max[i] - (Gv @ U + free_v)},
        {"type": "ineq", "fun": lambda U: mpc.dq_max[i] + (Gv @ U + free_v)},
    ]
    res = minimize(cost, np.full(N, u_prev), bounds=[(mpc.q_min[i], mpc.q_max[i])] * N,
                   constraints=cons, method="SLSQP", options={"maxiter": 500, "ftol": 1e-14})
    return res.x, res.fun


def test_admm_solves_the_stated_qp():
    """PositionMPC.solve returns the minimiser of the QP written in the derivation."""
    n, N, dt = 3, 10, 0.02
    omega, zeta = np.array([25.0, 30.0, 90.0]), np.array([0.3, 0.5, 0.6])
    qmin, qmax = np.full(n, -1.0), np.full(n, 1.0)
    rng = np.random.default_rng(5)
    worst = 0.0
    for trial in range(6):
        mpc = PositionMPC(omega, zeta, dt, N, qmin, qmax, np.full(n, 3.0), np.full(n, 0.08),
                          Q=100.0, P=300.0, R=0.5, iters=60)
        q, dq = rng.uniform(-0.5, 0.5, n), rng.normal(0, 0.3, n)
        u_prev = q + rng.normal(0, 0.02, n)
        r = (q + rng.normal(0, 0.4, n))[None, :] + np.arange(1, N + 1)[:, None] * 0.02 * rng.normal(0, 0.5, n)[None, :]
        u0 = mpc.solve(q, dq, r, u_prev)
        x0 = np.stack([q, dq], axis=1)
        for i in range(n):
            U, _ = _reference_qp(mpc, i, x0[i], r[:, i], u_prev[i], d_hat=0.0)
            worst = max(worst, abs(u0[i] - U[0]))
    # cold start, 60 fixed ADMM iterations: first move within 2 mrad of the exact QP solution
    assert worst < 2e-3


def test_applied_command_always_respects_limits():
    """Admissibility proposition: the applied u0 satisfies the position and rate limits (final clipping)."""
    n, N, dt = 2, 10, 0.02
    mpc = PositionMPC(np.array([25.0, 25.0]), np.array([0.3, 0.3]), dt, N, np.full(n, -0.5), np.full(n, 0.5),
                      np.full(n, 3.0), np.full(n, 0.05), iters=3)       # deliberately unconverged
    rng = np.random.default_rng(6)
    u_prev = np.zeros(n)
    for _ in range(200):
        r = rng.normal(0, 2.0, (N, n))
        u = mpc.solve(rng.normal(0, 1, n), rng.normal(0, 1, n), r, u_prev)
        assert np.all(np.abs(u) <= 0.5 + 1e-12) and np.all(np.abs(u - u_prev) <= 0.05 + 1e-12)
        u_prev = u


def test_offset_free_estimate_converges_and_removes_steady_state_error():
    """Offset-free proposition: a constant output disturbance c/omega^2 is learned; zero steady-state error."""
    N, dt = 10, 0.02
    omega, zeta, c = 25.0, 0.5, -30.0                      # plant: q'' = w^2 (u - q) - 2 z w q' + c  (static sag c/w^2)
    A = np.array([[0, 1], [-omega ** 2, -2 * zeta * omega]])
    B = np.array([0, omega ** 2])
    aug = np.zeros((4, 4))
    aug[:2, :2], aug[:2, 2], aug[3, 3] = A, B, 0
    # plant with the constant acceleration disturbance, exact ZOH on [x; 1]
    Ae = np.block([[A, np.array([[0.0], [c]])], [np.zeros((1, 3))]])
    Be = np.concatenate([B, [0.0]])
    big = np.zeros((4, 4))
    big[:3, :3], big[:3, 3] = Ae, Be
    E = expm(big * dt)
    Ad, Bd = E[:3, :3], E[:3, 3]

    def run(alpha):
        mpc = PositionMPC(np.array([omega]), np.array([zeta]), dt, N, np.array([-5.0]), np.array([5.0]),
                          np.array([10.0]), np.array([1.0]), dist_alpha=alpha)
        x = np.array([0.0, 0.0, 1.0])
        u = np.zeros(1)
        for _ in range(400):
            u = mpc.solve(np.array([x[0]]), np.array([x[1]]), np.full((N, 1), 0.3), u)
            x = Ad @ x + Bd * u[0]
        return x[0], mpc.d_hat[0]

    q_with, d_with = run(0.2)
    q_without, _ = run(0.0)
    assert d_with == pytest.approx(c / omega ** 2, rel=0.02)
    assert q_with == pytest.approx(0.3, abs=1e-3)           # zero steady-state error
    assert abs(q_without - 0.3) > 0.03                       # without the term the sag (48 mrad) remains


# --------------------------------------------------------------------------
# Energy tank
# --------------------------------------------------------------------------
def test_tank_energy_bound_theorem():
    """Tank theorem: sum(gamma P_out dt) <= T0 - Tmin + sum(P_in dt) for any signals, including T_max clipping."""
    rng = np.random.default_rng(7)
    dt, T0, Tmin, Tmax = 0.02, 0.5, 0.05, 1.0
    tank = EnergyTank(dt, T0, Tmin, Tmax)
    delivered, dissipated = 0.0, 0.0
    for _ in range(5000):
        dq = rng.normal(0, 2, 5)
        tau = rng.normal(0, 3, 5) + 4 * dq * rng.uniform(0, 1)
        p_diss = float(rng.uniform(0, 0.2))
        out = tank.filter(tau, dq, p_diss)
        delivered += float(out @ dq) * dt
        dissipated += p_diss * dt
        assert tank.T >= Tmin - 1e-12 and 0.0 <= tank.gamma <= 1.0
    assert delivered <= T0 - Tmin + dissipated + 1e-9


# --------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------
def test_mean_of_joint_rmse_never_exceeds_pooled_rmse():
    rng = np.random.default_rng(8)
    for _ in range(100):
        e = rng.normal(0, 1, (200, 5)) * rng.uniform(0.1, 2, 5)
        per = np.sqrt(np.mean(e ** 2, axis=0))
        assert per.mean() <= np.sqrt(np.mean(e ** 2)) + 1e-12
