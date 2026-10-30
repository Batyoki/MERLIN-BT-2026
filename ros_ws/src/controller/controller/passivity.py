"""
passivity.py
------------
Energy tank on the force-feedback port of the master: bounds the energy that
sensorless force reflection between different arms can return to the operator.

The feedback torque τ_f that the controller renders on the master can
inject energy into the human–master system at rate

    P_out[k] = τ_f[k] · q̇_m[k]          (> 0: energy flows to the operator)

The tank stores energy T that the controller has *dissipated* through the
master damping term (−D_md q̇_m) and the slave admittance damping:

    P_in[k]  = η (q̇_mᵀ D_md q̇_m + q̇_eᵀ D_sd q̇_e),   0 < η ≤ 1

and the rendered torque is scaled by γ ∈ [0, 1] so the tank never drops
below T_min:

    T[k+1] = min(T_max, T[k] + Δt (P_in − γ P_out))
    γ = 1                                  if T[k] + Δt(P_in − P_out) ≥ T_min
    γ = max(0, (T[k] + Δt P_in − T_min) / (Δt P_out))    otherwise

Since T ≥ T_min > 0 always, the energy delivered by the feedback channel up
to any time is bounded by the energy the controller has dissipated plus
T[0] − T_min. The bound holds regardless of torque-estimation errors and of the
joint-space ≠ Cartesian-space mismatch of the Jacobian force mapping. It concerns
the master port only: it does not make a delayed teleoperation loop passive
(that needs a two-port tank / TDPA). T_max limits how much energy can be released
in a burst.

Refs: Franken et al., IEEE T-RO 2011; Ferraguti et al., IEEE T-RO 2015;
proof in docs/derivations (energy-tank theorem).
"""

import numpy as np


class EnergyTank:
    def __init__(self, dt: float, T0: float = 0.5, T_min: float = 0.05,
                 T_max: float = 1.0, eta: float = 1.0):
        self.dt = dt
        self.T = T0
        self.T_min = T_min
        self.T_max = T_max
        self.eta = eta
        self.gamma = 1.0

    def filter(self, tau_f: np.ndarray, dq_m: np.ndarray, p_dissipated: float) -> np.ndarray:
        p_in = self.eta * max(p_dissipated, 0.0)
        p_out = float(tau_f @ dq_m)
        budget = self.T + self.dt * p_in - self.T_min
        if p_out <= 0.0 or budget >= self.dt * p_out:
            self.gamma = 1.0
        else:
            self.gamma = max(0.0, budget / (self.dt * p_out))
        self.T = min(self.T_max, self.T + self.dt * (p_in - self.gamma * p_out))
        return self.gamma * tau_f
