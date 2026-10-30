"""Unit tests for the control math (no ROS needed): python3 -m pytest test/test_control_math.py"""

import os

import numpy as np
import pinocchio as pin  # noqa: F401  (used in the gravity test)
import pytest

from controller.admittance import AdmittanceFilter
from controller.dynamics import ArmModel
from controller.metrics import tracking_metrics
from controller.mpc_optimizer import PositionMPC, preview_reference
from controller.observer import MomentumObserver
from controller.passivity import EnergyTank

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "controller")


@pytest.fixture(scope="module")
def vx():
    return ArmModel(os.path.join(URDF, "vx300s.urdf"))


@pytest.fixture(scope="module")
def rx():
    return ArmModel(os.path.join(URDF, "rx200.urdf"))


def test_gravity_by_name_matches_full_model(vx):
    q = np.array([0.1, 0.3, -0.2, 0.4, 0.2])
    q_full = pin.neutral(vx.model)
    for name, val in zip(vx.arm_joints, q):
        q_full[vx.model.joints[vx.model.getJointId(name)].idx_q] = val
    G = pin.computeGeneralizedGravity(vx.model, vx.data, q_full)
    assert np.allclose(vx.gravity(q), G[vx.idx_v])
    # the old np.insert(q, 3, 0) mapping lost the wrist_angle gravity
    assert abs(vx.gravity(q)[3]) > 0.2


def test_mass_matrix_symmetric_positive(rx):
    M = rx.mass(np.array([0.0, -0.3, 0.3, 0.6, 0.0]))
    assert np.allclose(M, M.T)
    assert np.all(np.linalg.eigvalsh(M) > 0)


def test_admittance_exact_discretisation_is_stable_for_tiny_inertia():
    # wrist values of the old setup: M = 7e-4, K = 10 -> ω_n Δt ≈ 2.4 (Euler unstable)
    f = AdmittanceFilter(np.array([7e-4]), np.array([0.4]), np.array([10.0]), 0.02, np.array([10.0]))
    for _ in range(500):
        q = f.step(np.array([1.0]))
    assert np.isfinite(q).all()
    assert q[0] == pytest.approx(1.0 / 10.0, rel=1e-3)          # static: q_e = τ / K


def test_mpc_tracks_and_respects_bounds():
    n, N, dt = 2, 10, 0.02
    mpc = PositionMPC(np.array([25.0, 25.0]), np.array([0.7, 0.7]), dt, N,
                      q_min=np.array([-1.0, -0.2]), q_max=np.array([1.0, 0.2]),
                      dq_max=np.full(n, 3.0), du_max=np.full(n, 0.05), iters=200)
    q, dq, u = np.zeros(n), np.zeros(n), np.zeros(n)
    A, B = mpc.A, mpc.B
    target = np.array([0.5, 0.5])                 # joint 2 target is outside q_max
    for _ in range(150):
        u_new = mpc.solve(q, dq, preview_reference(target, None, dt, N), u)
        assert np.all(np.abs(u_new - u) <= 0.05 + 1e-9)
        assert np.all(u_new <= mpc.q_max + 1e-9)
        u = u_new
        x = np.einsum("ijk,ik->ij", A, np.stack([q, dq], 1)) + B * u[:, None]
        q, dq = x[:, 0], x[:, 1]
    assert q[0] == pytest.approx(0.5, abs=1e-2)
    assert q[1] == pytest.approx(0.2, abs=1e-2)


def test_momentum_observer_recovers_external_torque(vx):
    """Arm moves under a known external torque; r must converge to it (no q̈ used)."""
    dt = 1e-3
    obs = MomentumObserver(vx, dt, 50.0)
    q, dq = np.array([0.0, -0.2, 0.3, 0.5, 0.0]), np.zeros(5)
    tau_ext = np.array([0.0, 1.0, -0.5, 0.2, 0.0])
    D = 20.0 * np.diag(vx.mass(q))                  # keeps the test plant bounded
    r = None
    for k in range(3000):
        tau_motor = (vx.gravity(q) - D * dq
                     + 0.3 * np.diag(vx.mass(q)) * 100 * np.sin(2 * np.pi * 0.5 * k * dt))
        r = obs.update(q, dq, tau_motor)
        a = np.linalg.solve(vx.mass(q), tau_motor + tau_ext - vx.coriolis(q, dq) - vx.gravity(q))
        dq = dq + dt * a
        q = q + dt * dq
    assert np.abs(dq).max() > 0.1                   # the arm really moved
    assert np.allclose(r, tau_ext, atol=0.05)


def test_energy_tank_never_below_minimum():
    tank = EnergyTank(0.02, T0=0.2, T_min=0.05, T_max=1.0)
    rng = np.random.default_rng(0)
    for _ in range(2000):
        dq = rng.normal(0, 1, 5)
        tau = 5.0 * dq            # pure energy injection
        tank.filter(tau, dq, p_dissipated=0.0)
        assert tank.T >= tank.T_min - 1e-12


def test_pooled_rmse_is_not_mean_of_rmse():
    # paper Table I per-joint RMSEs
    rmse = np.array([0.2250, 0.2715, 0.3609, 0.3152, 0.5366])
    assert np.sqrt(np.mean(rmse ** 2)) == pytest.approx(0.3583, abs=1e-4)
    assert np.mean(rmse) == pytest.approx(0.3418, abs=1e-4)
    t = np.arange(0, 10, 0.02)
    ref = np.sin(t)[:, None] * np.ones((1, 2))
    act = np.sin(t - 0.1)[:, None] * np.ones((1, 2))
    m = tracking_metrics(t, ref, act, ["a", "b"])
    assert m["overall"]["lag_s_mean"] == pytest.approx(0.1, abs=0.021)
    assert m["overall"]["rmse_lag_comp_pooled"] < 0.01
