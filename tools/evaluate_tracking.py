#!/usr/bin/env python3
"""
Paper Table I from a real run: per-joint and pooled tracking metrics, lag and
lag-compensated RMSE, computed from a controller CSV log (log_dir parameter).

  python3 tools/evaluate_tracking.py log.csv [--ref q_ref|q_m] [--markdown]

--ref q_m   : error w.r.t. the mapped master angles (what the paper reports)
--ref q_ref : error w.r.t. the admittance reference (excludes intended compliance)
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "controller"))
from controller.metrics import tracking_metrics  # noqa: E402

JOINTS = ("waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate")
LABEL = dict(waist="Waist", shoulder="Shoulder", elbow="Elbow",
             wrist_angle="Wrist Angle", wrist_rotate="Wrist Rotate")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--ref", default="q_m", choices=("q_m", "q_ref"))
    ap.add_argument("--markdown", action="store_true")
    a = ap.parse_args()
    with open(a.csv) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    ref = np.array([[float(r[f"{a.ref}_{j}"]) for j in JOINTS] for r in rows])
    act = np.array([[float(r[f"q_s_{j}"]) for j in JOINTS] for r in rows])
    m = tracking_metrics(t, ref, act, JOINTS)
    if not a.markdown:
        print(json.dumps(m, indent=2))
        return
    print("| Joint | RMSE (rad) | MAE (rad) | Max (rad) | Lag (ms) | Lag-comp. RMSE (rad) |")
    print("|---|---|---|---|---|---|")
    for j in JOINTS:
        v = m["joints"][j]
        print(f"| {LABEL[j]} | {v['rmse']:.4f} | {v['mae']:.4f} | {v['max']:.4f} | "
              f"{1e3 * v['lag_s']:.0f} | {v['rmse_lag_comp']:.4f} |")
    o = m["overall"]
    print(f"| Overall (pooled) | {o['rmse_pooled']:.4f} | {o['mae']:.4f} | {o['max']:.4f} | "
          f"{1e3 * o['lag_s_mean']:.0f} | {o['rmse_lag_comp_pooled']:.4f} |")


if __name__ == "__main__":
    main()
