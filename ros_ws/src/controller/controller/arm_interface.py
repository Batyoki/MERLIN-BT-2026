"""
arm_interface.py
----------------
State estimation for one Interbotix arm from its /joint_states topic.

Teleoperation pair used in the paper:
  Master: ReactorX 200 (5-DOF)                 — current (torque) mode
  Slave:  ViperX 300s (6-DOF, forearm_roll locked at 0 -> 5 active DOF)
                                               — position mode

All controller vectors are 5-long, in ARM_JOINTS order:
  [waist, shoulder, elbow, wrist_angle, wrist_rotate]

Sign convention (used everywhere):
  M(q) q̈ + C(q, q̇) q̇ + G(q) = τ_motor + τ_ext
  =>  τ̂_ext = G(q) + C(q, q̇) q̇ − τ_motor            (M q̈ neglected)
  τ̂_ext is the torque the ENVIRONMENT applies on the joint.

Units
-----
The Interbotix xs_sdk publishes JointState.effort as PRESENT CURRENT in mA.
It must be scaled by 1e-3 before multiplying by k_t [N·m/A]. The previous
version used the raw mA value, so every torque estimate was 1000x too large
and every downstream clamp was saturated.

Joints driven by two motors (shadow motors in the Interbotix motor config:
rx200 shoulder; vx300s shoulder and elbow) only report the primary motor's
current, so the joint torque is n_motors * k_t * I.

NOTE: on the MASTER, which runs in current mode, the measured current is
(almost) the commanded current. A residual G + C − k_t I computed from it
only echoes the controller's own command and is NOT an external-torque
estimate. The master therefore does not compute τ̂_ext here; that requires a
momentum/disturbance observer driven by the commanded torque (future work).
"""

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .dynamics import ARM_JOINTS, ArmModel
from .observer import MomentumObserver

# ---------------------------------------------------------------------------
# Per-arm hardware description (5 active arm joints)
# <<USER: verify motor models against interbotix_xsarm_control/config/<robot>.yaml>>
# k_t values: XM430-W350 ≈ 1.78, XM540-W270 ≈ 2.41, XL430-W250 ≈ 1.07  [N·m/A]
# (stall torque / stall current at 12 V from the ROBOTIS e-manual). The real
# current→torque map of geared Dynamixels is not linear; calibrate k_t with
# known masses before trusting absolute values.
# ---------------------------------------------------------------------------
CURRENT_UNIT_SCALE = 1e-3     # xs_sdk effort is mA -> A

RX200 = dict(
    urdf="rx200",
    kt=np.array([1.78, 1.78, 1.78, 1.78, 1.07]),
    n_motors=np.array([1, 2, 1, 1, 1]),
    stall_torque=np.array([4.1, 8.2, 4.1, 4.1, 1.5]),
    q_min=np.deg2rad([-180, -108, -108, -100, -180]),
    q_max=np.deg2rad([180, 113, 93, 123, 180]),
)

VX300S = dict(
    urdf="vx300s",
    kt=np.array([2.41, 2.41, 2.41, 2.41, 1.78]),
    n_motors=np.array([1, 2, 2, 1, 1]),
    stall_torque=np.array([10.6, 21.2, 21.2, 10.6, 4.1]),
    q_min=np.deg2rad([-180, -101, -101, -107, -180]),
    q_max=np.deg2rad([180, 101, 92, 130, 180]),
)

ARM_SPECS = {"rx200": RX200, "vx300s": VX300S}


@dataclass
class ArmState:
    """Snapshot of one arm at one control tick (all arrays shape (5,))."""
    q: np.ndarray
    dq: np.ndarray
    tau_motor: np.ndarray        # n_motors * k_t * I   [N·m]
    tau_ext: np.ndarray          # environment torque estimate [N·m] (zeros on master)
    stamp: float = 0.0


def extract_joints(names: Sequence[str], values: Sequence[float],
                   wanted: Sequence[str] = ARM_JOINTS) -> Optional[np.ndarray]:
    """Pick `wanted` joints by name; None if any is missing or the field is empty."""
    if len(values) != len(names):
        return None
    lookup = dict(zip(names, values))
    if any(w not in lookup for w in wanted):
        return None
    return np.array([lookup[w] for w in wanted], dtype=float)


class ArmInterface:
    """Turns raw joint_states into an ArmState (velocity, torque, τ̂_ext)."""

    def __init__(
        self,
        arm: str,
        model: ArmModel,
        dt: float,
        filter_alpha: float = 0.3,
        estimator: str = "momentum",
        observer_gain: float = 15.0,
        current_unit_scale: float = CURRENT_UNIT_SCALE,
        kt: Optional[np.ndarray] = None,
    ):
        spec = ARM_SPECS[arm]
        self.model = model
        self.dt = dt
        self.alpha = filter_alpha
        if estimator not in ("momentum", "static", "none"):
            raise ValueError("estimator must be 'momentum', 'static' or 'none'")
        self.estimator = estimator
        self.observer = MomentumObserver(model, dt, observer_gain) if estimator == "momentum" else None
        self.current_unit_scale = current_unit_scale
        self.kt = np.asarray(kt if kt is not None else spec["kt"], dtype=float)
        self.n_motors = spec["n_motors"]

        n = model.n
        self._q_prev: Optional[np.ndarray] = None
        self._t_prev: Optional[float] = None
        self._dq_filt = np.zeros(n)
        self._tau_filt = np.zeros(n)

    def update(self, q: np.ndarray, effort: np.ndarray, stamp: float,
               dq_meas: Optional[np.ndarray] = None) -> ArmState:
        """
        q       : arm joint positions [rad]
        effort  : JointState.effort for the same joints (mA from xs_sdk)
        stamp   : message time [s] — used for the finite difference
        dq_meas : JointState.velocity if the driver publishes it (preferred)
        """
        # Velocity: backward difference with the real sample time
        if dq_meas is not None:
            dq_raw = dq_meas
        elif self._q_prev is None:
            dq_raw = np.zeros_like(q)
        else:
            h = stamp - self._t_prev if stamp > self._t_prev else self.dt
            dq_raw = (q - self._q_prev) / h
        self._q_prev, self._t_prev = q.copy(), stamp
        self._dq_filt = self.alpha * dq_raw + (1 - self.alpha) * self._dq_filt

        # Torque from current: n_motors * k_t * I
        current_A = effort * self.current_unit_scale
        tau_raw = self.n_motors * self.kt * current_A
        self._tau_filt = self.alpha * tau_raw + (1 - self.alpha) * self._tau_filt

        # External torque (environment torque on the joints)
        if self.estimator == "momentum":
            # UNFILTERED velocity and torque: the observer is itself a low-pass of
            # bandwidth K_O. Feeding the IIR-filtered q̇ (≈47 ms lag at α = 0.3)
            # makes p = M q̇ lag the torque integral, which acts as negative inertia
            # in the force loop and caused a master–slave limit cycle in simulation.
            tau_ext = self.observer.update(q, dq_raw, tau_raw)
        elif self.estimator == "static":
            # static residual, M q̈ neglected: only valid when the arm is (nearly) static
            tau_ext = (self.model.gravity(q)
                       + self.model.coriolis(q, self._dq_filt)
                       - self._tau_filt)
        else:
            tau_ext = np.zeros_like(q)

        return ArmState(q=q.copy(), dq=self._dq_filt.copy(),
                        tau_motor=self._tau_filt.copy(), tau_ext=tau_ext, stamp=stamp)
