#!/usr/bin/env python3
"""
Identify the slave's Dynamixel position loop  q̈ = ω²(u − q) − 2ζω q̇ + c
per joint, from any CSV logged by the controller (log_dir parameter):
the commanded position q_cmd_<joint> is the input u, the measured q_s_<joint>
the output. c absorbs the gravity sag (no gravity feed-forward in xs_sdk).

  python3 tools/identify_position_loop.py log.csv [--plot]

Paste the printed ω and ζ into config/controller.yaml (pos_loop_omega/zeta).
Best data: teleoperate in free space for 30–60 s with varied speeds, or
command small steps (±0.1 rad) one joint at a time.
"""

import argparse
import csv

import numpy as np
from scipy.optimize import least_squares

JOINTS = ("waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate")


def simulate(params, t, u, q0, dq0=0.0):
    omega, zeta, c = params
    q, v = q0, dq0
    out = np.empty_like(u)
    for k in range(len(t)):
        out[k] = q
        if k + 1 == len(t):
            break
        h = t[k + 1] - t[k]
        n = max(1, int(np.ceil(h / 1e-3)))
        for _ in range(n):           # u held constant over the sample (ZOH)
            a = omega ** 2 * (u[k] - q) - 2 * zeta * omega * v + c
            v += (h / n) * a
            q += (h / n) * v
    return out


def identify(t, u, q):
    def res(p):
        return simulate(p, t, u, q[0]) - q
    sol = least_squares(res, x0=[20.0, 0.7, 0.0], bounds=([1.0, 0.05, -50.0], [300.0, 3.0, 50.0]))
    fit = simulate(sol.x, t, u, q[0])
    r2 = 1 - np.sum((fit - q) ** 2) / np.sum((q - q.mean()) ** 2)
    return sol.x, r2, fit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    with open(a.csv) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    omegas, zetas = [], []
    for j in JOINTS:
        u = np.array([float(r[f"q_cmd_{j}"]) for r in rows])
        q = np.array([float(r[f"q_s_{j}"]) for r in rows])
        (w, z, c), r2, fit = identify(t, u, q)
        omegas.append(w)
        zetas.append(z)
        print(f"{j:13s} omega={w:7.2f} rad/s  zeta={z:5.2f}  sag_term={c:7.3f}  R^2={r2:.3f}")
        if a.plot:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.plot(t, u, label="u (cmd)")
            plt.plot(t, q, label="q (measured)")
            plt.plot(t, fit, "--", label="model")
            plt.title(j)
            plt.legend()
    print("\npos_loop_omega:", [round(w, 1) for w in omegas])
    print("pos_loop_zeta: ", [round(z, 2) for z in zetas])
    if a.plot:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
