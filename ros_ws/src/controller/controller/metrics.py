"""
metrics.py
----------
Tracking / transparency metrics used in the paper (fixes the RMSE issue).

Conventions
  e[k, j] = q_slave[k, j] − q_ref[k, j]   (rad), K samples, J joints
  per-joint RMSE_j = sqrt(mean_k e²)
  OVERALL (pooled) RMSE = sqrt(mean_{k,j} e²) = sqrt(mean_j RMSE_j²)
  — NOT mean_j RMSE_j (the paper's 0.3418 is that mean; pooled is 0.3583).
  lag_j: delay (s) maximising the cross-correlation of slave vs reference
  lag-compensated RMSE: RMSE after shifting the slave back by lag_j, which
  separates pure delay from shape error.
"""

from typing import Dict, Sequence

import numpy as np


def tracking_metrics(t: np.ndarray, q_ref: np.ndarray, q_act: np.ndarray,
                     names: Sequence[str], max_lag_s: float = 1.0) -> Dict:
    e = q_act - q_ref
    dt = float(np.median(np.diff(t)))
    out = {"joints": {}}
    for j, name in enumerate(names):
        lag = estimate_lag(q_ref[:, j], q_act[:, j], dt, max_lag_s)
        s = int(round(lag / dt))
        e_c = q_act[s:, j] - q_ref[: len(q_ref) - s, j] if s > 0 else e[:, j]
        out["joints"][name] = dict(
            rmse=float(np.sqrt(np.mean(e[:, j] ** 2))),
            mae=float(np.mean(np.abs(e[:, j]))),
            max=float(np.max(np.abs(e[:, j]))),
            lag_s=lag,
            rmse_lag_comp=float(np.sqrt(np.mean(e_c ** 2))),
        )
    per = np.array([v["rmse"] for v in out["joints"].values()])
    out["overall"] = dict(
        rmse_pooled=float(np.sqrt(np.mean(e ** 2))),
        rmse_mean_of_joints=float(per.mean()),
        mae=float(np.mean(np.abs(e))),
        max=float(np.max(np.abs(e))),
        lag_s_mean=float(np.mean([v["lag_s"] for v in out["joints"].values()])),
        rmse_lag_comp_pooled=float(np.sqrt(np.mean(
            [v["rmse_lag_comp"] ** 2 for v in out["joints"].values()]))),
    )
    return out


def estimate_lag(ref: np.ndarray, act: np.ndarray, dt: float, max_lag_s: float) -> float:
    """Shift s ≥ 0 maximising the Pearson correlation of ref[:-s] and act[s:]."""
    if np.ptp(ref) < 1e-9 or np.ptp(act) < 1e-9:
        return 0.0
    max_s = min(int(max_lag_s / dt), len(ref) // 2)
    best, best_s = -np.inf, 0
    for s in range(0, max_s + 1):
        a, b = ref[: len(ref) - s], act[s:]
        if np.ptp(a) < 1e-12 or np.ptp(b) < 1e-12:
            continue
        c = np.corrcoef(a, b)[0, 1]
        if c > best:
            best, best_s = c, s
    return best_s * dt


def force_metrics(f_true: np.ndarray, f_rendered: np.ndarray, contact: np.ndarray) -> Dict:
    """Transparency on the contact axis: f_* are (K,) along the wall normal."""
    if not np.any(contact):
        return dict(rmse=float("nan"), peak_true=0.0, peak_rendered=0.0, corr=float("nan"))
    a, b = f_true[contact], f_rendered[contact]
    corr = float(np.corrcoef(a, b)[0, 1]) if a.std() > 1e-9 and b.std() > 1e-9 else float("nan")
    return dict(rmse=float(np.sqrt(np.mean((a - b) ** 2))),
                peak_true=float(np.max(np.abs(a))), peak_rendered=float(np.max(np.abs(b))),
                corr=corr)
