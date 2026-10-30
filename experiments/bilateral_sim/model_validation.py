"""
Validation of the slave model used by the MPC (position loop + offset-free term).

  python experiments/bilateral_sim/model_validation.py

For a logged run, at every control tick k the controller's internal model
(controller.mpc_optimizer.PositionMPC: parallel model state x_hat and output
disturbance d_hat, exactly as in the controller) predicts the slave position
for the next N = 10 ticks given the commands that were then actually sent.
The prediction is compared with the simulated plant. Baselines use no model of
the actuator at all:

  hold           q(k+h) = q(k)
  const-vel      q(k+h) = q(k) + h * dt * (q(k) - q(k-1)) / dt
  loop           position-loop model, d_hat = 0 (no offset-free term)
  loop + d_hat   position-loop model + output-disturbance estimate (the MPC's model)

The old MPC's rigid-body torque model (M^-1 (tau - C - G)) is not in this
comparison on purpose: the VX300s is commanded in POSITION, so the torque that
model needs as an input does not exist on the real interface (docs/EVALUATION_METHODOLOGY.md).
"""

import json
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import run as R                                              # noqa: E402
from controller.misa_mpc_controller import BilateralParams   # noqa: E402
from controller.mpc_optimizer import PositionMPC              # noqa: E402

N = 10


def predict_errors(L, omega, zeta, dist_alpha=0.2, use_dhat=True):
    """RMS prediction error per horizon step h = 1..N (rad), pooled over joints and time."""
    n = 5
    mpc = PositionMPC(omega, zeta, R.DT_CTRL, N, np.full(n, -3.0), np.full(n, 3.0), np.full(n, 3.0),
                      np.full(n, 1.0), dist_alpha=dist_alpha)
    q, u = L["q_s"], L["u"]                      # q[k]: plant state at the END of tick k; u[k]: command during tick k
    K = len(q)
    xhat = np.stack([q[0], np.zeros(n)], axis=1)
    dhat = np.zeros(n)
    err = [[] for _ in range(N)]
    for k in range(1, K - N):
        # measurement update (as in PositionMPC.solve) with the state at the end of tick k-1 ...
        dhat = dhat + dist_alpha * ((q[k - 1] - xhat[:, 0]) - dhat) if use_dhat else dhat
        d = dhat if use_dhat else np.zeros(n)
        # ... then free + forced response for the commands that are actually applied next
        free = np.einsum("ikj,ij->ik", mpc.Phi_q, xhat) + d[:, None]                    # (n, N)
        pred = np.zeros((n, N))
        for i in range(n):
            pred[i] = free[i] + mpc.G_q[i] @ u[k:k + N, i]
        for h in range(N):
            err[h].append(pred[:, h] - q[k + h])
        # propagate the parallel model with the command applied during tick k
        xhat = np.einsum("ijk,ik->ij", mpc.A, xhat) + mpc.B * u[k][:, None]
    return np.array([np.sqrt(np.mean(np.concatenate(e) ** 2)) for e in err])


def baseline_errors(L):
    q = L["q_s"]
    K = len(q)
    hold, cv = [[] for _ in range(N)], [[] for _ in range(N)]
    for k in range(2, K - N):
        v = (q[k - 1] - q[k - 2]) / R.DT_CTRL
        for h in range(N):
            hold[h].append(q[k - 1] - q[k + h])
            cv[h].append(q[k - 1] + (h + 1) * R.DT_CTRL * v - q[k + h])
    f = lambda e: np.array([np.sqrt(np.mean(np.concatenate(x) ** 2)) for x in e])
    return f(hold), f(cv)


def main():
    ident = json.load(open(os.path.join(HERE, "results", "results.json")))["identified"]
    omega, zeta = np.array(ident["omega"]), np.array(ident["zeta"])
    base = BilateralParams(pos_loop_omega=omega, pos_loop_zeta=zeta, dt=R.DT_CTRL)
    rows = {}
    for seed in range(3):
        _, L = R.run("val", "ours", replace(base, mode="unilateral"), seed=seed, intent_fn=R.make_intent(seed))
        hold, cv = baseline_errors(L)
        rows.setdefault("hold", []).append(hold)
        rows.setdefault("const-vel", []).append(cv)
        rows.setdefault("loop (d_hat = 0)", []).append(predict_errors(L, omega, zeta, use_dhat=False))
        rows.setdefault("loop + d_hat (MPC model)", []).append(predict_errors(L, omega, zeta, use_dhat=True))
    mean = {k: np.mean(v, axis=0) for k, v in rows.items()}
    hs = [1, 2, 5, 10]
    lines = ["# Slave-model validation (MuJoCo, 3 operator seeds, unilateral mode)", "",
             "RMS N-step prediction error of the slave joint positions (rad), pooled over 5 joints.", "",
             "| Predictor | " + " | ".join(f"h={h} ({20 * h} ms)" for h in hs) + " |", "|---|" + "---|" * len(hs)]
    for k, v in mean.items():
        lines.append(f"| {k} | " + " | ".join(f"{v[h - 1]:.4f}" for h in hs) + " |")
    out = os.path.join(HERE, "results")
    open(os.path.join(out, "model_validation.md"), "w").write("\n".join(lines) + "\n")
    json.dump({k: v.tolist() for k, v in mean.items()}, open(os.path.join(out, "model_validation.json"), "w"), indent=2)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
