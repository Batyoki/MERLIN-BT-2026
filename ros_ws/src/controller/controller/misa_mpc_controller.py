"""
misa_mpc_controller.py
----------------------
Bilateral controller for a current-controlled master and a
position-controlled slave, in joint space (5 active joints).

Modes (same inputs/outputs, so baselines and ablations share one code path):

  "unilateral"  slave ← mapped master position, master = gravity comp only
                (Interbotix xsarm_puppet behaviour)
  "pp"          position–position:   τ_f = −K_p (q_m − q_s) − K_d (q̇_m − q̇_s)
  "pf"          position–force:      τ_f = k_of Γ dz(τ̂_ext,s)
  "misa"        proposed: master impedance / slave admittance + MPC,
                τ_f = k_of Γ dz(τ̂_ext,s) − k_om K_md (q_m − q_s)
                optionally passed through the energy tank.

Master (impedance)
    τ_m,cmd = Ĝ_m(q_m) − D_md q̇_m + γ τ_f
  Ĝ_m is evaluated at the RAW master angles (the master→slave offset is part
  of the reference mapping only). The master's own current is NOT fed back:
  in current mode it echoes the command, and a term built from it would
  close a unit-gain loop that integrates any bias (pole at z = 1).

Slave (admittance)
    q_s,ref = S q_m + o + q_e,       M_sd q̈_e + D_sd q̇_e + K_sd q_e = dz(τ̂_ext,s)
  exact ZOH discretisation (admittance.py), then the position MPC
  (mpc_optimizer.py) produces the position command for the Dynamixel loop.
"""

from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np

from .admittance import AdmittanceFilter
from .arm_interface import ArmState
from .dynamics import ArmModel
from .force_mapping import ForceMapping
from .mpc_optimizer import PositionMPC, preview_reference
from .passivity import EnergyTank


def dead_zone(x: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Soft dead-zone dz_δ(x) = sign(x) max(|x| − δ, 0)."""
    return np.sign(x) * np.maximum(np.abs(x) - delta, 0.0)


@dataclass
class BilateralParams:
    mode: str = "misa"
    dt: float = 0.02
    # joint mapping master -> slave reference: q_ref = scale * q_m + offset
    map_scale: np.ndarray = field(default_factory=lambda: np.ones(5))
    map_offset: np.ndarray = field(default_factory=lambda: np.zeros(5))
    # master impedance
    Dmd: np.ndarray = field(default_factory=lambda: np.array([0.08, 0.12, 0.08, 0.03, 0.01]))
    Kmd: np.ndarray = field(default_factory=lambda: np.array([3.0, 5.0, 3.0, 1.0, 0.3]))
    tau_master_max: np.ndarray = field(default_factory=lambda: np.array([2.0, 4.0, 2.0, 2.0, 0.75]))
    # force / position channel gains
    kof: float = 0.3                     # robust to 60 ms delay in sim; 0.6 is not
    kom: float = 1.0
    tau_dead_zone: np.ndarray = field(default_factory=lambda: np.array([0.15, 0.3, 0.2, 0.1, 0.05]))
    # p-p baseline gains
    pp_Kp: np.ndarray = field(default_factory=lambda: np.array([3.0, 5.0, 3.0, 1.0, 0.3]))
    pp_Kd: np.ndarray = field(default_factory=lambda: np.array([0.05, 0.08, 0.05, 0.02, 0.005]))
    # slave admittance (design values, ω_n ≈ 10 rad/s, ζ = 1)
    Msd: np.ndarray = field(default_factory=lambda: np.array([0.30, 0.40, 0.30, 0.10, 0.04]))
    Dsd: np.ndarray = field(default_factory=lambda: np.array([6.0, 8.0, 6.0, 2.0, 0.8]))
    Ksd: np.ndarray = field(default_factory=lambda: np.array([30.0, 40.0, 30.0, 10.0, 4.0]))
    q_e_max: np.ndarray = field(default_factory=lambda: np.full(5, 0.3))
    # slave MPC (identified position loop + constraints)
    use_mpc: bool = True
    mpc_horizon: int = 10
    pos_loop_omega: np.ndarray = field(default_factory=lambda: np.array([25.0, 20.0, 25.0, 30.0, 35.0]))
    pos_loop_zeta: np.ndarray = field(default_factory=lambda: np.array([0.8, 0.8, 0.8, 0.8, 0.8]))
    dq_slave_max: np.ndarray = field(default_factory=lambda: np.full(5, 3.0))
    du_slave_max: np.ndarray = field(default_factory=lambda: np.full(5, 0.15))
    mpc_Q: float = 100.0
    mpc_P: float = 300.0
    mpc_R: float = 0.5
    mpc_rho: float = 0.3          # ADMM penalty; 0.3 converges to the exact QP in <= 60 iterations (see docs)
    mpc_iters: int = 60
    # force mapping + passivity
    force_map: str = "jacobian"           # "jacobian" | "identity"
    s_force: float = 1.0
    use_tank: bool = True
    tank_T0: float = 0.5
    tank_T_min: float = 0.05
    tank_T_max: float = 1.0


class BilateralController:
    MODES = ("unilateral", "pp", "pf", "misa")

    def __init__(self, p: BilateralParams, master: ArmModel, slave: ArmModel,
                 q_slave_min: np.ndarray, q_slave_max: np.ndarray):
        if p.mode not in self.MODES:
            raise ValueError(f"mode must be one of {self.MODES}")
        self.p = p
        self.master = master
        self.slave = slave
        self.q_slave_min, self.q_slave_max = q_slave_min, q_slave_max
        self.mapping = ForceMapping(master, slave, mode=p.force_map, s_force=p.s_force)
        self.admittance = AdmittanceFilter(p.Msd, p.Dsd, p.Ksd, p.dt, p.q_e_max)
        self.tank = EnergyTank(p.dt, p.tank_T0, p.tank_T_min, p.tank_T_max)
        self.mpc = PositionMPC(
            p.pos_loop_omega, p.pos_loop_zeta, p.dt, p.mpc_horizon,
            q_slave_min, q_slave_max, p.dq_slave_max, p.du_slave_max,
            Q=p.mpc_Q, P=p.mpc_P, R=p.mpc_R, rho=p.mpc_rho, iters=p.mpc_iters) if p.use_mpc else None
        self._u_prev = None

    def reset(self):
        self.admittance.reset()
        if self.mpc:
            self.mpc.reset()
        self._u_prev = None

    def map_to_slave(self, q_m: np.ndarray) -> np.ndarray:
        return self.p.map_scale * q_m + self.p.map_offset

    def step(self, m: ArmState, s: ArmState) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """Returns (τ_master_cmd [N·m], q_slave_cmd [rad], diagnostics)."""
        p = self.p
        if self._u_prev is None:
            self._u_prev = s.q.copy()

        q_m_mapped = self.map_to_slave(m.q)
        dq_m_mapped = p.map_scale * m.dq
        tau_s = dead_zone(s.tau_ext, p.tau_dead_zone)

        # ---------------- feedback torque on the master -----------------
        if p.mode == "unilateral":
            tau_f = np.zeros_like(m.q)
        elif p.mode == "pp":
            tau_f = (-p.pp_Kp * (q_m_mapped - s.q) - p.pp_Kd * (dq_m_mapped - s.dq)) * p.map_scale
        else:
            tau_f = p.kof * self.mapping(m.q, s.q, tau_s)
            if p.mode == "misa":
                tau_f = tau_f - p.kom * p.Kmd * (q_m_mapped - s.q) * p.map_scale

        # ---------------- slave reference ------------------------------
        if p.mode == "misa":
            q_e = self.admittance.step(tau_s)
            q_ref = q_m_mapped + q_e
        else:
            q_e = np.zeros_like(m.q)
            q_ref = q_m_mapped
        q_ref = np.clip(q_ref, self.q_slave_min, self.q_slave_max)

        if self.mpc is not None and p.mode == "misa":
            r = preview_reference(q_ref, dq_m_mapped, p.dt, p.mpc_horizon)
            q_cmd = self.mpc.solve(s.q, s.dq, r, self._u_prev)
        else:
            q_cmd = np.clip(q_ref, self._u_prev - p.du_slave_max, self._u_prev + p.du_slave_max)
        self._u_prev = q_cmd.copy()

        # ---------------- passivity (energy tank) ----------------------
        damping = p.Dmd * m.dq
        if p.use_tank and p.mode in ("pf", "misa"):
            p_diss = float(m.dq @ damping) + float(self.admittance.dq_e @ (p.Dsd * self.admittance.dq_e))
            tau_f = self.tank.filter(tau_f, m.dq, p_diss)

        tau_m = self.master.gravity(m.q) - damping + tau_f
        tau_m = np.clip(tau_m, -p.tau_master_max, p.tau_master_max)

        diag = dict(tau_f=tau_f, q_ref=q_ref, q_e=q_e, tau_s=tau_s,
                    tank=self.tank.T, gamma=self.tank.gamma,
                    mpc_ms=self.mpc.last_solve_ms if self.mpc else 0.0)
        return tau_m, q_cmd, diag
