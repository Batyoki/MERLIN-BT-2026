"""Second-order step response used for position-loop identification."""

import numpy as np


def simulate_2nd_order(omega: float, zeta: float, t: np.ndarray, step: float) -> np.ndarray:
    """Response of q̈ = ω²(u − q) − 2ζω q̇ to u = step, q(0) = q̇(0) = 0."""
    y = np.zeros_like(t)
    q, v = 0.0, 0.0
    h = 1e-4
    k_out, tk = 0, 0.0
    while k_out < len(t):
        if tk >= t[k_out] - 1e-12:
            y[k_out] = q
            k_out += 1
            continue
        a = omega ** 2 * (step - q) - 2 * zeta * omega * v
        v += h * a
        q += h * v
        tk += h
    return y
