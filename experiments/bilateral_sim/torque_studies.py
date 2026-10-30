"""
Torque studies for the paper (MuJoCo, RX200 master -> VX300s slave).

  python experiments/bilateral_sim/torque_studies.py [--out .../results/torque] [--quick]

S1  Sensorless external-torque estimation accuracy
    (reviewer: "the force-feedback path is never measured")
S2  Torque mimicking: external force on the slave -> torque on the master
S3  Object-weight rendering: mass held by the slave -> force felt at the master
S4  Transparency vs environment stiffness (wall 500 ... 10 000 N/m)
S5  Stability map: force gain k_of x communication delay, with / without tank

Writes studies.json, studies.md and fig_S1..S5.png.
"""

import argparse
import json
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import run as R                                                    # noqa: E402
from controller.arm_interface import ArmInterface                  # noqa: E402
from controller.dynamics import ArmModel                           # noqa: E402
from controller.misa_mpc_controller import BilateralParams         # noqa: E402
from legacy import VX_KT, LegacyEstimator                          # noqa: E402

JOINT_LABEL = ["waist", "shoulder", "elbow", "wrist angle", "wrist rotate"]
G = 9.81


def hold_still(_t):
    return R.Q0.copy()


def shifted(true, est):
    """The estimate at tick k uses the current measured at the end of tick k-1."""
    return true[:-1], est[1:]


# ---------------------------------------------------------------------------
# S1 — estimation accuracy
# ---------------------------------------------------------------------------
def s1_env(t, slave, wall):
    f = np.zeros(3)
    if 4.0 <= t < 8.0:
        f[2] -= 0.2 * G                     # 0.2 kg hung at the gripper
    if 12.0 <= t < 16.0:
        f[2] -= 0.5 * G                     # 0.5 kg
    if 18.0 <= t < 22.0:
        f += np.array([-4.0, 2.0, 0.0])     # lateral push 4.5 N
    return f


def s1_intent(t):
    if t < 2.0:
        return R.Q0.copy()
    A = np.array([0.4, 0.15, 0.2, 0.3, 0.4])
    f = np.array([0.2, 0.25, 0.3, 0.35, 0.4])
    return R.Q0 + A * np.sin(2 * np.pi * f * (t - 2.0))


def study_s1(base):
    """Unilateral teleop (no feedback, so all estimators see identical data)."""
    _, L = R.run("S1", "ours", replace(base, mode="unilateral"), T=24.0,
                 intent_fn=s1_intent, env_fn=s1_env, estimator="none")
    slave_model = ArmModel(os.path.join(R.URDF_DIR, "vx300s.urdf"))
    t = L["t"]
    est = {}
    # original code (units bug + index bug), sign flipped to environment torque
    leg = LegacyEstimator(slave_model, VX_KT, R.DT_CTRL, buggy_slave_gravity=True, fix_units=False)
    est["Original code"] = np.array([-leg.update(q, I)[1] for q, I in zip(L["q_s_meas"], L["I_s"])])
    leg = LegacyEstimator(slave_model, VX_KT, R.DT_CTRL, buggy_slave_gravity=True, fix_units=True)
    est["Original, units fixed"] = np.array([-leg.update(q, I)[1] for q, I in zip(L["q_s_meas"], L["I_s"])])
    for name, kind in (("Static residual (Eq. 2, fixed)", "static"), ("Momentum observer", "momentum")):
        ai = ArmInterface("vx300s", slave_model, R.DT_CTRL, 0.3, estimator=kind)
        est[name] = np.array([ai.update(q, I, tk).tau_ext for q, I, tk in zip(L["q_s_meas"], L["I_s"], t)])

    true = L["tau_ext_true"]
    loaded = np.linalg.norm(true, axis=1) > 1e-6
    free = ~loaded & (t > 3.0)
    table = {}
    for name, e in est.items():
        tr, es = shifted(true, e)
        fr, ld = free[:-1], loaded[:-1]
        err = es - tr
        settle = np.ones(len(tr), bool)
        for t_on in (4.0, 8.0, 12.0, 16.0, 18.0, 22.0):     # exclude 0.3 s after load switches
            settle &= ~((t[:-1] >= t_on) & (t[:-1] < t_on + 0.3))
        table[name] = dict(
            rmse_loaded=np.sqrt(np.mean(err[ld & settle] ** 2, axis=0)).tolist(),
            false_torque_free_rms=np.sqrt(np.mean(es[fr] ** 2, axis=0)).tolist(),
            rmse_all=float(np.sqrt(np.mean(err ** 2))),
        )
    return dict(t=t, true=true, est=est, table=table)


def study_s1b(base):
    """Estimator inside the closed force loop: static residual vs momentum observer."""
    out = {}
    for kof in (0.3, 0.6):
        for name, p in (("Position-force", replace(base, mode="pf", force_map="identity", use_tank=False, kof=kof)),
                        ("Proposed", replace(base, kof=kof))):
            for est in ("static", "momentum"):
                r, L = R.run("S1b", "ours", p, estimator=est)
                free = L["t"] < R.T_FREE
                out[f"{name}, k_of={kof}, {est}"] = dict(
                    rmse=r["tracking_all"]["rmse_pooled"], max_err=r["tracking_all"]["max"],
                    peak_wall=r["force"]["peak_true"],
                    false_torque_shoulder=float(np.sqrt(np.mean(L["tau_hat_s"][free, 1] ** 2))))
    return out


def s1_speed_sweep(base, scales=(1.0, 2.0, 3.0)):
    """Open loop: false torque in free motion vs motion speed, both estimators."""
    slave_model = ArmModel(os.path.join(R.URDF_DIR, "vx300s.urdf"))
    out = []
    for sc in scales:
        def intent(t, sc=sc):
            if t < 2.0:
                return R.Q0.copy()
            A = np.array([0.4, 0.15, 0.2, 0.3, 0.4])
            f = np.array([0.2, 0.25, 0.3, 0.35, 0.4]) * sc
            return R.Q0 + A * np.sin(2 * np.pi * f * (t - 2.0))
        _, L = R.run("S1s", "ours", replace(base, mode="unilateral"), T=12.0, intent_fn=intent,
                     env_fn=lambda t, s, w: np.zeros(3), estimator="none")
        free = L["t"] > 3.0
        row = dict(speed=sc)
        for kind in ("static", "momentum"):
            ai = ArmInterface("vx300s", slave_model, R.DT_CTRL, 0.3, estimator=kind)
            e = np.array([ai.update(q, I, tk).tau_ext for q, I, tk in zip(L["q_s_meas"], L["I_s"], L["t"])])
            row[kind] = np.sqrt(np.mean(e[free] ** 2, axis=0)).tolist()
        out.append(row)
    return out


# ---------------------------------------------------------------------------
# S2 — torque mimicking (operator holds the master still)
# ---------------------------------------------------------------------------
def s2_env(t, slave, wall):
    """Smooth pushes on the slave gripper in x, z and a 0.5 Hz oscillation."""
    def bump(t0, t1):
        if t < t0 or t > t1:
            return 0.0
        return np.sin(np.pi * (t - t0) / (t1 - t0)) ** 2
    f = np.zeros(3)
    f[0] = -8.0 * bump(2.0, 6.0)
    f[2] = -6.0 * bump(7.0, 11.0)
    if 12.0 <= t < 18.0:
        f[0] += 4.0 * np.sin(2 * np.pi * 0.5 * (t - 12.0))
    return f


def study_s2(base):
    runs = {
        "Original code": ("legacy", None),
        "Position-force (2ch)": ("ours", replace(base, mode="pf", force_map="identity", use_tank=False)),
        "Proposed": ("ours", base),
        "Proposed, k_of=0.6": ("ours", replace(base, kof=0.6)),
    }
    out = {}
    for name, (kind, p) in runs.items():
        _, L = R.run("S2", kind, p, T=19.0, intent_fn=hold_still, env_fn=s2_env)
        kof = p.kof if p is not None else 0.8
        f_ref = kof * L["f_env3"]                       # ideal force on the operator's hand
        f_m = L["f_felt3"]
        act = np.linalg.norm(L["f_env3"], axis=1) > 0.2
        err = f_m[act] - f_ref[act]
        corr = [float(np.corrcoef(f_m[act, i], f_ref[act, i])[0, 1])
                if np.std(f_ref[act, i]) > 1e-6 and np.std(f_m[act, i]) > 1e-6 else float("nan")
                for i in (0, 2)]
        gain = [float(np.dot(f_m[act, i], f_ref[act, i]) / max(np.dot(f_ref[act, i], f_ref[act, i]), 1e-9))
                for i in (0, 2)]
        drift = float(np.max(np.abs(L["q_m"] - R.Q0)))
        out[name] = dict(L=L, kof=kof, rmse_xz=float(np.sqrt(np.mean(err[:, [0, 2]] ** 2))),
                         corr_x=corr[0], corr_z=corr[1], gain_x=gain[0], gain_z=gain[1],
                         master_drift_rad=drift)
    return out


def study_s2b(base):
    """Which part of the controller sets the rendered force gain (x and z)."""
    cases = {
        "Proposed": base,
        "Proposed, k_om=0": replace(base, kom=0.0),
        "Proposed, identity map": replace(base, force_map="identity"),
        "Position-force, identity map": replace(base, mode="pf", force_map="identity", use_tank=False),
        "Position-force, Jacobian map": replace(base, mode="pf", force_map="jacobian", use_tank=False),
    }
    out = {}
    for name, p in cases.items():
        _, L = R.run("S2b", "ours", p, T=19.0, intent_fn=hold_still, env_fn=s2_env)
        act = np.linalg.norm(L["f_env3"], axis=1) > 0.2
        fr, fm = p.kof * L["f_env3"], L["f_felt3"]
        out[name] = {ax: float(np.dot(fm[act, i], fr[act, i]) / np.dot(fr[act, i], fr[act, i]))
                     for ax, i in (("gain_x", 0), ("gain_z", 2))}
    return out


# ---------------------------------------------------------------------------
# S3 — object weight rendering
# ---------------------------------------------------------------------------
def study_s3(base, masses=(0.0, 0.1, 0.2, 0.3, 0.5, 0.75)):
    out = {}
    for name, p, kind in (("Original code", None, "legacy"),
                          ("Position-force (2ch)", replace(base, mode="pf", force_map="identity", use_tank=False), "ours"),
                          ("Proposed", base, "ours")):
        rows = []
        for m in masses:
            env = (lambda mm: (lambda t, s, w: np.array([0.0, 0.0, -mm * G]) if t >= 2.0 else np.zeros(3)))(m)
            _, L = R.run("S3", kind, p, T=6.0, intent_fn=hold_still, env_fn=env)
            sel = L["t"] > 4.0
            fz = float(np.mean(L["f_felt3"][sel, 2]))
            rows.append(dict(mass=m, true_N=m * G, felt_z_N=fz))
        out[name] = rows
    return out


# ---------------------------------------------------------------------------
# S4 — transparency vs environment stiffness
# ---------------------------------------------------------------------------
def study_s4(base, ks=(500.0, 1000.0, 2000.0, 5000.0, 10000.0)):
    out = {}
    for name, p in (("Position-force (2ch)", replace(base, mode="pf", force_map="identity", use_tank=False)),
                    ("Position-position", replace(base, mode="pp")),
                    ("Proposed", base)):
        rows = []
        for k in ks:
            r, L = R.run("S4", "ours", p, wall_k=k)
            fo = r["force"]
            contact = L["contact"]
            ratio = float(np.mean(L["f_felt"][contact]) / max(np.mean(-L["f_env"][contact]), 1e-6)) if contact.any() else float("nan")
            rows.append(dict(k=k, rmse=r["tracking_all"]["rmse_pooled"], max_err=r["tracking_all"]["max"],
                             peak_wall=fo["peak_true"], felt_over_true=ratio, corr=fo["corr"]))
        out[name] = rows
    return out


# ---------------------------------------------------------------------------
# S5 — stability map k_of x delay
# ---------------------------------------------------------------------------
def study_s5(base, kofs=(0.1, 0.3, 0.6, 1.0, 1.5), delays=(0, 1, 2, 3)):
    out = {}
    for tank in (False, True):
        grid = np.zeros((len(kofs), len(delays)))
        peak = np.zeros_like(grid)
        for i, kof in enumerate(kofs):
            for j, d in enumerate(delays):
                r, _ = R.run("S5", "ours", replace(base, kof=kof, use_tank=tank), delay_steps=d)
                grid[i, j] = r["tracking_all"]["max"] if "tracking_all" in r else np.inf
                peak[i, j] = r["force"]["peak_true"] if "force" in r else np.inf
        out["tank" if tank else "no_tank"] = dict(max_err=grid.tolist(), peak_wall=peak.tolist())
    out["kofs"], out["delays_ms"] = list(kofs), [20 * d for d in delays]
    return out


# ---------------------------------------------------------------------------
def plots(S1, S2, S3, S4, S5, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # S1
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for a, j in zip(ax, (1, 2, 3)):
        a.plot(S1["t"], S1["true"][:, j], "k", lw=2, label="true external torque")
        for name in ("Static residual (Eq. 2, fixed)", "Momentum observer"):
            a.plot(S1["t"], S1["est"][name][:, j], lw=0.9, label=name)
        a.set_ylabel(f"{JOINT_LABEL[j]} (N·m)")
        a.set_ylim(np.array([-1, 1]) * max(3 * np.abs(S1["true"][:, j]).max(), 0.5))
    for t0, txt in ((4, "0.2 kg"), (12, "0.5 kg"), (18, "push 4.5 N")):
        ax[0].annotate(txt, (t0 + 0.2, ax[0].get_ylim()[1] * 0.8), fontsize=8)
    ax[0].legend(fontsize=8, loc="lower left")
    ax[-1].set_xlabel("time (s)  — slave moving throughout (unilateral teleoperation)")
    fig.suptitle("S1  Sensorless external-torque estimation on the VX300s slave")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_S1_estimation.png"), dpi=140)

    # S2
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for name, r in S2.items():
        L = r["L"]
        ax[0].plot(L["t"], L["f_felt3"][:, 0], label=name)
        ax[1].plot(L["t"], L["f_felt3"][:, 2], label=name)
    L = S2["Proposed"]["L"]
    k = S2["Proposed"]["kof"]
    ax[0].plot(L["t"], k * L["f_env3"][:, 0], "k--", label=f"ideal: k_of·F_env (k_of={k})")
    ax[1].plot(L["t"], k * L["f_env3"][:, 2], "k--")
    ax[0].set_ylabel("force on operator, x (N)")
    ax[1].set_ylabel("force on operator, z (N)")
    master_model = ArmModel(os.path.join(R.URDF_DIR, "rx200.urdf"))
    tau_ideal = np.array([k * master_model.jacobian(q)[:3].T @ f for q, f in zip(L["q_m"], L["f_env3"])])
    for j, c in ((1, "C0"), (2, "C3")):
        ax[2].plot(L["t"], tau_ideal[:, j], "--", color=c, label=f"ideal k_of·J_mᵀF_env, {JOINT_LABEL[j]}")
        ax[2].plot(L["t"], L["tau_felt"][:, j], color=c, lw=0.9, label=f"master τ felt, {JOINT_LABEL[j]}")
    ax[2].set_ylabel("master joint torque (N·m), proposed")
    ax[2].set_xlabel("time (s)  — operator holds the master still; forces applied to the slave gripper")
    for a in ax:
        a.legend(fontsize=7, ncol=2)
    fig.suptitle("S2  Torque mimicking: external force on the slave is reproduced on the master")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_S2_mimicking.png"), dpi=140)

    # S3
    fig, ax = plt.subplots(figsize=(6, 4.5))
    for name, rows in S3.items():
        m = [r["mass"] for r in rows]
        ax.plot(m, [-r["felt_z_N"] for r in rows], "o-", label=name)
    m = np.array([r["mass"] for r in S3["Proposed"]])
    ax.plot(m, 0.3 * m * G, "k--", label="ideal k_of·m·g (k_of = 0.3)")
    ax.plot(m, m * G, "k:", label="m·g")
    ax.set_xlabel("mass held by the slave (kg)")
    ax.set_ylabel("downward force felt at the master (N)")
    ax.legend(fontsize=8)
    ax.set_title("S3  Object-weight rendering")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_S3_weight.png"), dpi=140)

    # S4
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    for name, rows in S4.items():
        k = [r["k"] for r in rows]
        ax[0].semilogx(k, [r["peak_wall"] for r in rows], "o-", label=name)
        ax[1].semilogx(k, [r["felt_over_true"] for r in rows], "o-", label=name)
        ax[2].semilogx(k, [r["rmse"] for r in rows], "o-", label=name)
    ax[0].set_ylabel("peak wall force (N)")
    ax[1].set_ylabel("mean felt / mean true force")
    ax[1].axhline(0.3, color="k", ls="--", lw=0.8)
    ax[2].set_ylabel("tracking RMSE (rad)")
    for a in ax:
        a.set_xlabel("wall stiffness (N/m)")
        a.legend(fontsize=7)
    fig.suptitle("S4  Contact behaviour vs environment stiffness")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_S4_stiffness.png"), dpi=140)

    # S5
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    for a, key in zip(ax, ("no_tank", "tank")):
        g = np.array(S5[key]["max_err"])
        im = a.imshow(g, cmap="RdYlGn_r", vmin=0, vmax=1.0, origin="lower", aspect="auto")
        a.set_xticks(range(len(S5["delays_ms"])), [f"{d} ms" for d in S5["delays_ms"]])
        a.set_yticks(range(len(S5["kofs"])), [str(k) for k in S5["kofs"]])
        a.set_xlabel("one-way delay")
        a.set_ylabel("force gain k_of")
        a.set_title("without energy tank" if key == "no_tank" else "with energy tank")
        for i in range(g.shape[0]):
            for j in range(g.shape[1]):
                a.text(j, i, f"{g[i, j]:.2f}", ha="center", va="center", fontsize=8)
    fig.colorbar(im, ax=ax, label="max tracking error (rad)")
    fig.suptitle("S5  Stability map (max |q_s − q_m| over the 24 s task)")
    fig.savefig(os.path.join(out, "fig_S5_stability.png"), dpi=140, bbox_inches="tight")


def markdown(S1, S2, S3, S4, S5, path, S1b=None, S1s=None, S2b=None):
    L = ["# Torque studies (MuJoCo, RX200 → VX300s)", ""]
    L += ["## S1 Estimation accuracy", "",
          "| Estimator | RMSE under load, shoulder / elbow / wrist angle (N·m) | false torque in free motion, RMS (N·m) |",
          "|---|---|---|"]
    for n, r in S1["table"].items():
        a, b = r["rmse_loaded"], r["false_torque_free_rms"]
        L.append(f"| {n} | {a[1]:.2f} / {a[2]:.2f} / {a[3]:.2f} | {b[1]:.2f} / {b[2]:.2f} / {b[3]:.2f} |")
    if S1s:
        L += ["", "Free-motion false torque vs speed (shoulder / elbow, N·m RMS):", "",
              "| speed | static residual | momentum observer |", "|---|---|---|"]
        for r in S1s:
            L.append(f"| x{r['speed']:.0f} | {r['static'][1]:.2f} / {r['static'][2]:.2f} | "
                     f"{r['momentum'][1]:.2f} / {r['momentum'][2]:.2f} |")
    if S1b:
        L += ["", "### S1b Estimator inside the closed force loop", "",
              "| Case | RMSE (rad) | max err (rad) | peak wall (N) | false shoulder torque, free (N·m) |",
              "|---|---|---|---|---|"]
        for n, r in S1b.items():
            L.append(f"| {n} | {r['rmse']:.3f} | {r['max_err']:.2f} | {r['peak_wall']:.1f} | "
                     f"{r['false_torque_shoulder']:.2f} |")
    L += ["", "## S2 Torque mimicking", "",
          "| Controller | force RMSE x,z (N) | corr x | corr z | gain x | gain z | master drift (rad) |",
          "|---|---|---|---|---|---|---|"]
    for n, r in S2.items():
        L.append(f"| {n} | {r['rmse_xz']:.2f} | {r['corr_x']:.2f} | {r['corr_z']:.2f} | "
                 f"{r['gain_x']:.2f} | {r['gain_z']:.2f} | {r['master_drift_rad']:.3f} |")
    L += ["", "gain = least-squares ratio felt / (k_of·F_env); 1.0 = exact scaled reproduction.", ""]
    if S2b:
        L += ["### S2b What sets the rendered gain", "", "| Case | gain x | gain z |", "|---|---|---|"]
        for n, r in S2b.items():
            L.append(f"| {n} | {r['gain_x']:.2f} | {r['gain_z']:.2f} |")
        L.append("")
    L += [
          "## S3 Object weight", "", "| mass (kg) | m·g (N) | " + " | ".join(S3) + " |",
          "|---|---|" + "---|" * len(S3)]
    for i, r in enumerate(S3["Proposed"]):
        L.append(f"| {r['mass']:.2f} | {r['true_N']:.2f} | " +
                 " | ".join(f"{-S3[n][i]['felt_z_N']:.2f}" for n in S3) + " |")
    L += ["", "## S4 Stiffness", "", "| Controller | k (N/m) | peak wall (N) | felt/true | RMSE (rad) |", "|---|---|---|---|---|"]
    for n, rows in S4.items():
        for r in rows:
            L.append(f"| {n} | {r['k']:.0f} | {r['peak_wall']:.1f} | {r['felt_over_true']:.2f} | {r['rmse']:.3f} |")
    L += ["", "## S5 Stability (max tracking error, rad)", ""]
    for key in ("no_tank", "tank"):
        L += [f"**{key.replace('_', ' ')}**", "", "| k_of \\ delay | " + " | ".join(f"{d} ms" for d in S5["delays_ms"]) + " |",
              "|---|" + "---|" * len(S5["delays_ms"])]
        for k, row in zip(S5["kofs"], S5[key]["max_err"]):
            L.append(f"| {k} | " + " | ".join(f"{v:.2f}" for v in row) + " |")
        L.append("")
    open(path, "w").write("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "results", "torque"))
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    cfg = os.path.join(HERE, "results", "results.json")
    if os.path.exists(cfg):
        ident = json.load(open(cfg))["identified"]
        omega, zeta = np.array(ident["omega"]), np.array(ident["zeta"])
    else:
        omega, zeta = R.identify_position_loop()
    base = BilateralParams(pos_loop_omega=omega, pos_loop_zeta=zeta, dt=R.DT_CTRL)

    print("S1 estimation ...")
    S1 = study_s1(base)
    print("S1 speed sweep / closed loop ...")
    S1s = s1_speed_sweep(base, scales=(1.0, 3.0) if a.quick else (1.0, 2.0, 3.0))
    S1b = study_s1b(base)
    print("S2 mimicking ...")
    S2 = study_s2(base)
    S2b = study_s2b(base)
    print("S3 weight ...")
    S3 = study_s3(base, masses=(0.0, 0.2, 0.5) if a.quick else (0.0, 0.1, 0.2, 0.3, 0.5, 0.75))
    print("S4 stiffness ...")
    S4 = study_s4(base, ks=(500.0, 5000.0) if a.quick else (500.0, 1000.0, 2000.0, 5000.0, 10000.0))
    print("S5 stability map ...")
    S5 = study_s5(base, kofs=(0.3, 1.5) if a.quick else (0.1, 0.3, 0.6, 1.0, 1.5),
                  delays=(0, 3) if a.quick else (0, 1, 2, 3))
    plots(S1, S2, S3, S4, S5, a.out)
    markdown(S1, S2, S3, S4, S5, os.path.join(a.out, "studies.md"), S1b, S1s, S2b)
    json.dump(dict(S1=S1["table"], S1_speed=S1s, S1b=S1b, S2b=S2b, S2={k: {kk: vv for kk, vv in v.items() if kk != "L"} for k, v in S2.items()},
                   S3=S3, S4=S4, S5=S5), open(os.path.join(a.out, "studies.json"), "w"), indent=2)
    print(open(os.path.join(a.out, "studies.md")).read())


if __name__ == "__main__":
    main()
