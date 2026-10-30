"""
Figures for the paper (vector PDF, IEEE column width), from the same simulator as run.py and torque_studies.py.

  python experiments/bilateral_sim/paper_figures.py [--out paper/figures]

fig_benchmark.pdf   benchmark task (seed 0): shoulder tracking error and force felt at the master,
                    for unilateral, position-position, position-force and the proposed controller
fig_estimation.pdf  external joint torque on the slave: true vs static residual vs momentum observer (study S1)
fig_mimicking.pdf   force reproduced at the master while known forces act on the slave (study S2)
fig_stability.pdf   maximum tracking error over force gain x delay, without/with energy tank (study S5,
                    read from results/torque/studies.json)
"""

import argparse
import json
import os
import sys
from dataclasses import replace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
import numpy as np                                                   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import run as R                                                      # noqa: E402
import torque_studies as TS                                          # noqa: E402
from controller.misa_mpc_controller import BilateralParams           # noqa: E402

# one colour (and line style) per controller, the same in every figure
STYLE = {
    "Proposed": dict(color="#2a78d6", ls="-", lw=1.0),
    "Position-force": dict(color="#eb6834", ls="--", lw=1.0),
    "Position-position": dict(color="#1baf7a", ls="-.", lw=1.0),
    "Unilateral": dict(color="#eda100", ls=":", lw=1.2),
}
REF = dict(color="black", ls=(0, (4, 2)), lw=0.9)
COL_W = 3.45                     # IEEE column width (in)

plt.rcParams.update({
    "font.family": "serif", "font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.linewidth": 0.6, "grid.linewidth": 0.4,
    "grid.color": "#d9d9d9", "axes.grid": True, "legend.frameon": False, "pdf.fonttype": 42,
    "axes.spines.top": False, "axes.spines.right": False,
})


def base_params():
    ident = json.load(open(os.path.join(HERE, "results", "results.json")))["identified"]
    return BilateralParams(pos_loop_omega=np.array(ident["omega"]), pos_loop_zeta=np.array(ident["zeta"]),
                           dt=R.DT_CTRL)


def controllers(base):
    return {
        "Unilateral": replace(base, mode="unilateral"),
        "Position-position": replace(base, mode="pp"),
        "Position-force": replace(base, mode="pf", force_map="identity", use_tank=False),
        "Proposed": base,
    }


def fig_benchmark(base, out):
    logs = {name: R.run(name, "ours", p, seed=0)[1] for name, p in controllers(base).items()}
    fig, ax = plt.subplots(2, 1, figsize=(COL_W, 3.15), sharex=True)
    for name in ("Unilateral", "Position-position", "Position-force", "Proposed"):
        L = logs[name]
        ax[0].plot(L["t"], 1e3 * (L["q_s"][:, 1] - L["q_m"][:, 1]), label=name, **STYLE[name])
    for name in ("Position-position", "Position-force", "Proposed"):
        L = logs[name]
        ax[1].plot(L["t"], L["f_felt"], label=name, **STYLE[name])
    L = logs["Proposed"]
    ax[1].plot(L["t"], -base.kof * L["f_env"], label=r"$k_{of}F_{wall}$", **REF)
    ax[0].set_ylabel("shoulder error (mrad)")
    ax[1].set_ylabel("force at master (N)")
    ax[1].set_xlabel("time (s)")
    for a in ax:
        a.axvline(R.T_FREE, color="#7f7f7f", lw=0.6)
    ax[0].text(R.T_FREE / 2, 0.92, "free motion", transform=ax[0].get_xaxis_transform(), ha="center", fontsize=7)
    ax[0].text(R.T_FREE * 1.5, 0.92, "contact with wall", transform=ax[0].get_xaxis_transform(), ha="center",
               fontsize=7)
    handles, labels = ax[0].get_legend_handles_labels()
    h1, l1 = ax[1].get_legend_handles_labels()
    handles.append(h1[-1]), labels.append(l1[-1])
    fig.legend(handles, labels, ncol=3, loc="upper center", handlelength=2.2, columnspacing=1.0,
               bbox_to_anchor=(0.5, 1.0))
    ax[0].set_ylim(-45, 45)
    ax[1].set_ylim(-1.5, 2.0)
    fig.tight_layout(pad=0.3, h_pad=0.4, rect=(0, 0, 1, 0.85))
    fig.savefig(os.path.join(out, "fig_benchmark.pdf"))
    plt.close(fig)


def fig_estimation(base, out):
    S1 = TS.study_s1(base)
    t, true = S1["t"], S1["true"]
    est = {"static residual": S1["est"]["Static residual (Eq. 2, fixed)"],
           "momentum observer": S1["est"]["Momentum observer"]}
    st = {"static residual": dict(color="#eb6834", ls="--", lw=0.8),
          "momentum observer": dict(color="#2a78d6", ls="-", lw=0.8)}
    fig, ax = plt.subplots(2, 1, figsize=(COL_W, 2.55), sharex=True)
    for a, j, lbl in ((ax[0], 1, "shoulder"), (ax[1], 2, "elbow")):
        keep = t > 0.5                                  # skip the observer start-up transient
        a.plot(t[keep], true[keep, j], color="black", lw=1.4, label="true")
        for name, e in est.items():
            a.plot(t[keep], e[keep, j], label=name, **st[name])
        a.set_ylabel(f"{lbl} (N$\\cdot$m)")
        a.set_ylim(-2.5, 3.2)
    for x, s in ((6.0, "0.2 kg"), (14.0, "0.5 kg"), (20.0, "4.5 N push")):
        ax[0].text(x, 0.06, s, transform=ax[0].get_xaxis_transform(), ha="center", fontsize=7)
    ax[1].set_xlabel("time (s)")
    fig.legend(*ax[0].get_legend_handles_labels(), ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(pad=0.3, h_pad=0.4, rect=(0, 0, 1, 0.9))
    fig.savefig(os.path.join(out, "fig_estimation.pdf"))
    plt.close(fig)
    return S1["table"]


def fig_mimicking(base, out):
    runs = {"Position-force": replace(base, mode="pf", force_map="identity", use_tank=False), "Proposed": base}
    logs = {n: R.run("S2", "ours", p, T=19.0, intent_fn=TS.hold_still, env_fn=TS.s2_env)[1] for n, p in runs.items()}
    fig, ax = plt.subplots(2, 1, figsize=(COL_W, 2.55), sharex=True)
    for a, i, lbl in ((ax[0], 0, "$x$"), (ax[1], 2, "$z$")):
        L = logs["Proposed"]
        a.plot(L["t"], base.kof * L["f_env3"][:, i], label=r"$k_{of}F_{env}$", **REF)
        for n in ("Position-force", "Proposed"):
            a.plot(logs[n]["t"], logs[n]["f_felt3"][:, i], label=n, **STYLE[n])
        a.set_ylabel(f"force {lbl} (N)")
    ax[1].set_xlabel("time (s)")
    fig.legend(*ax[0].get_legend_handles_labels(), ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(pad=0.3, h_pad=0.4, rect=(0, 0, 1, 0.9))
    fig.savefig(os.path.join(out, "fig_mimicking.pdf"))
    plt.close(fig)


def fig_stability(out):
    S5 = json.load(open(os.path.join(HERE, "results", "torque", "studies.json")))["S5"]
    kofs, delays = S5["kofs"], S5["delays_ms"]
    fig, ax = plt.subplots(1, 2, figsize=(COL_W, 1.95), sharey=True)
    for a, key, title in ((ax[0], "no_tank", "without tank"), (ax[1], "tank", "with tank")):
        g = np.array(S5[key]["max_err"])
        im = a.imshow(np.minimum(g, 1.0), origin="lower", cmap="Blues", vmin=0.0, vmax=1.0, aspect="auto")
        for i in range(len(kofs)):
            for j in range(len(delays)):
                a.text(j, i, f"{g[i, j]:.2f}", ha="center", va="center", fontsize=6.5,
                       color="white" if g[i, j] > 0.55 else "black")
        a.set_xticks(range(len(delays)), [str(d) for d in delays])
        a.set_xlabel("one-way delay (ms)")
        a.set_title(title, fontsize=8)
        a.grid(False)
    ax[0].set_yticks(range(len(kofs)), [f"{k:g}" for k in kofs])
    ax[0].set_ylabel("force gain $k_{of}$")
    cb = fig.colorbar(im, ax=ax, shrink=0.9, pad=0.02)
    cb.set_label("max error (rad)", fontsize=7)
    cb.ax.tick_params(labelsize=6.5)
    fig.savefig(os.path.join(out, "fig_stability.pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "..", "..", "paper", "figures"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    base = base_params()
    fig_stability(a.out)
    fig_benchmark(base, a.out)
    fig_estimation(base, a.out)
    fig_mimicking(base, a.out)
    print("written to", os.path.abspath(a.out))


if __name__ == "__main__":
    main()
