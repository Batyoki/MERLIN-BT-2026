"""
Multi-seed version of the benchmark in run.py.

  python experiments/bilateral_sim/multi_seed.py [--seeds 5] [--out results]

Each seed changes (a) the sensor-noise realisation and (b) the operator's
trajectory: amplitudes +-25 %, frequencies +-15 %, random phase per joint, and
the wall-push amplitude/frequency (run.make_intent). Seed 0 is the canonical
trajectory of run.py. All controllers see the same operator trajectory and the
same noise for a given seed, so differences between controllers are paired.

Reported per controller: mean +- sample standard deviation (ddof = 1) over seeds,
and, against the proposed controller, in how many seeds it was better.
"""

import argparse
import json
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import run as R                                              # noqa: E402
from controller.misa_mpc_controller import BilateralParams   # noqa: E402

METRICS = [
    ("rmse_all", "RMSE pooled, whole task (rad)"),
    ("rmse_free", "RMSE pooled, free motion (rad)"),
    ("max_err_free", "max abs. error, free motion (rad)"),
    ("max_err", "max abs. error, whole task (rad)"),
    ("lag_ms", "lag, free motion (ms)"),
    ("wall_peak", "peak wall force (N)"),
    ("force_corr", "force correlation (felt vs true)"),
]


def one_run(name, kind, params, seed):
    res, _ = R.run(name, kind, params, seed=seed, intent_fn=R.make_intent(seed))
    if res.get("unstable") or "tracking_all" not in res:
        return None
    return dict(
        rmse_all=res["tracking_all"]["rmse_pooled"],
        rmse_free=res["tracking_free"]["overall"]["rmse_pooled"],
        max_err_free=res["tracking_free"]["overall"]["max"],
        max_err=res["tracking_all"]["max"],
        lag_ms=1e3 * res["tracking_free"]["overall"]["lag_s_mean"],
        wall_peak=res["force"]["peak_true"],
        force_corr=res["force"]["corr"],
        joints=res["tracking_joints"],
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    a = ap.parse_args()
    ident = json.load(open(os.path.join(HERE, "results", "results.json")))["identified"]
    base = BilateralParams(pos_loop_omega=np.array(ident["omega"]), pos_loop_zeta=np.array(ident["zeta"]),
                           dt=R.DT_CTRL)
    cases = [
        ("Original code (re-implemented)", "legacy", None),
        ("Unilateral position", "ours", replace(base, mode="unilateral")),
        ("Position-position", "ours", replace(base, mode="pp")),
        ("Position-force", "ours", replace(base, mode="pf", use_tank=False, force_map="identity")),
        ("Proposed, no MPC", "ours", replace(base, use_mpc=False)),
        ("Proposed", "ours", base),
    ]
    runs = {n: [] for n, _, _ in cases}
    for seed in range(a.seeds):
        for name, kind, params in cases:
            print(f"seed {seed}: {name}", flush=True)
            runs[name].append(one_run(name, kind, params, seed))

    summary = {}
    for name, rows in runs.items():
        ok = [r for r in rows if r is not None]
        summary[name] = dict(n_ok=len(ok), n=len(rows))
        for key, _ in METRICS:
            v = np.array([r[key] for r in ok], float)
            summary[name][key] = dict(mean=float(np.nanmean(v)), std=float(np.nanstd(v, ddof=1)) if len(v) > 1 else 0.0,
                                      min=float(np.nanmin(v)), max=float(np.nanmax(v)), values=v.tolist())
    # per-joint whole-task metrics (paper Table I layout), mean over seeds
    per_joint = {}
    for name, rows in runs.items():
        ok = [r for r in rows if r is not None]
        per_joint[name] = {
            j: {m: float(np.mean([r["joints"][j][m] for r in ok])) for m in ("rmse", "mae", "max", "lag_s", "rmse_lag_comp")}
            for j in R.ARM}
    prop = runs["Proposed"]
    for name, rows in runs.items():
        wins = sum(1 for p, r in zip(prop, rows) if p and r and p["rmse_all"] < r["rmse_all"])
        summary[name]["proposed_better_rmse_in"] = "–" if name == "Proposed" else f"{wins}/{len(rows)}"

    os.makedirs(a.out, exist_ok=True)
    json.dump(dict(software=R.software_versions(), seeds=a.seeds, summary=summary, per_joint=per_joint),
              open(os.path.join(a.out, "multi_seed.json"), "w"), indent=2)
    L = [f"# Multi-seed benchmark ({a.seeds} seeds, MuJoCo, RX200 -> VX300s)", "",
         "mean ± sample std over seeds; each seed = new sensor noise and a randomised operator trajectory.", "",
         "| Controller | " + " | ".join(lbl for _, lbl in METRICS) + " | proposed lower RMSE in |",
         "|---|" + "---|" * (len(METRICS) + 1)]
    for name, s in summary.items():
        cells = []
        for key, _ in METRICS:
            m = s[key]
            fmt = "{:.1f} ± {:.1f}" if key in ("lag_ms", "wall_peak") else "{:.3f} ± {:.3f}"
            cells.append(fmt.format(m["mean"], m["std"]))
        L.append(f"| {name} | " + " | ".join(cells) + f" | {s['proposed_better_rmse_in']} |")
    L += ["", "## Per-joint tracking, whole task, mean over seeds (rad)", "",
          "| Controller | Joint | RMSE | MAE | Max | lag (ms) |", "|---|---|---|---|---|---|"]
    for name in ("Original code (re-implemented)", "Position-position", "Proposed"):
        for j in R.ARM:
            m = per_joint[name][j]
            L.append(f"| {name} | {j} | {m['rmse']:.4f} | {m['mae']:.4f} | {m['max']:.4f} | {1e3 * m['lag_s']:.0f} |")
    open(os.path.join(a.out, "multi_seed.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
