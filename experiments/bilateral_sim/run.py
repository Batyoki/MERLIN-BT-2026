"""
Bilateral teleoperation benchmark in MuJoCo (RX200 master -> VX300s slave).

  python experiments/bilateral_sim/run.py [--quick] [--out experiments/bilateral_sim/results]

What it does
  1. Identifies the slave Dynamixel position loop (ω, ζ per joint) from step
     responses, exactly as tools/identify_position_loop.py does on hardware.
  2. Runs a 24 s scenario: 12 s free motion (sinusoids on all joints),
     then 12 s in which the operator pushes the slave end-effector into a
     wall (k = 2000 N/m). A simulated operator holds the master with a
     joint-space impedance towards an intended trajectory.
  3. Runs: the ORIGINAL controller (as in the repo, and with only the units
     fixed), baselines (unilateral, position–position, position–force) and
     the proposed controller, plus ablations (no MPC, no tank, identity force
     map, horizon N, delay).
  4. Writes results.json, results.md and plots.

Sensor model: 4096-count encoders, xs_sdk effort in mA with 2.69 mA
quantisation and 5 mA noise, 50 Hz control, 1 kHz physics, plant masses
+8 % vs. the controller model, joint friction not in the controller model.
"""

import argparse
import json
import os
import sys
from collections import deque
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "ros_ws", "src", "controller"))
sys.path.insert(0, HERE)

from controller.arm_interface import RX200, VX300S, ArmInterface          # noqa: E402
from controller.dynamics import ArmModel                                  # noqa: E402
from controller.metrics import force_metrics, tracking_metrics            # noqa: E402
from controller.misa_mpc_controller import BilateralController, BilateralParams  # noqa: E402
from legacy import LegacyController                                       # noqa: E402
from plant import ARM, Arm, DynamixelPositionLoop, Wall                    # noqa: E402

URDF_DIR = os.path.join(REPO, "ros_ws", "src", "controller", "controller")
DT_CTRL, DT_SIM = 0.02, 0.001
SUB = int(round(DT_CTRL / DT_SIM))
Q0 = np.array([0.0, -0.3, 0.3, 0.6, 0.0])
ENC = 2 * np.pi / 4096
SLAVE_KP = np.array([60.0, 150.0, 100.0, 25.0, 6.0])
SLAVE_KD = np.array([1.2, 3.0, 2.0, 0.25, 0.05])
HUMAN_K = np.array([6.0, 10.0, 6.0, 2.0, 0.5])
HUMAN_B = np.array([0.2, 0.3, 0.2, 0.05, 0.02])
T_FREE, T_TOTAL = 12.0, 24.0


def human_intent(t: float) -> np.ndarray:
    if t < T_FREE:
        A = np.array([0.6, 0.25, 0.3, 0.4, 0.6])
        f = np.array([0.25, 0.3, 0.35, 0.4, 0.5])
        ramp = min(t / 1.0, 1.0)
        return Q0 + ramp * A * np.sin(2 * np.pi * f * t)
    tc = t - T_FREE
    push = 0.5 * (1 - np.cos(2 * np.pi * 0.125 * tc))     # two slow pushes
    return Q0 + np.array([0.0, 0.45, -0.1, 0.0, 0.0]) * push


def make_intent(seed: int):
    """
    Randomised operator for multi-seed studies: the same two-phase task as
    human_intent() (free motion, then two pushes towards the wall) with +-25 %
    amplitude, +-15 % frequency and a random phase per joint.
    seed = 0 reproduces human_intent() exactly.
    """
    if seed == 0:
        return human_intent
    rng = np.random.default_rng(1000 + seed)
    A = np.array([0.6, 0.25, 0.3, 0.4, 0.6]) * rng.uniform(0.75, 1.25, 5)
    f = np.array([0.25, 0.3, 0.35, 0.4, 0.5]) * rng.uniform(0.85, 1.15, 5)
    phi = rng.uniform(0, 2 * np.pi, 5)
    push_amp = 0.45 * rng.uniform(0.8, 1.2)
    push_f = 0.125 * rng.uniform(0.85, 1.15)

    def intent(t):
        if t < T_FREE:
            ramp = min(t / 1.0, 1.0)
            return Q0 + ramp * A * (np.sin(2 * np.pi * f * t + phi) - np.sin(phi))
        push = 0.5 * (1 - np.cos(2 * np.pi * push_f * (t - T_FREE)))
        return Q0 + np.array([0.0, push_amp, -0.1, 0.0, 0.0]) * push
    return intent


def software_versions() -> dict:
    """Versions of everything the numbers depend on (stored next to every result)."""
    import platform
    import mujoco
    import pinocchio
    import scipy
    out = dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
               mujoco=mujoco.__version__, pinocchio=pinocchio.__version__, platform=platform.platform())
    return out


def quantize(x, step):
    return np.round(x / step) * step


def sensor_effort_mA(tau, kt, n_motors, rng):
    I = tau / (kt * n_motors) * 1e3
    return quantize(I + rng.normal(0, 5.0, I.shape), 2.69)


def identify_position_loop(seed=0):
    """Step tests on the slave plant -> (ω, ζ) per joint (log-decrement / rise fit)."""
    from scipy.optimize import least_squares
    from mpc_like import simulate_2nd_order
    omegas, zetas = np.zeros(5), np.zeros(5)
    for j in range(5):
        slave = Arm(os.path.join(URDF_DIR, "vx300s.urdf"), Q0, ee_body="vx300s/ee_gripper_link")
        loop = DynamixelPositionLoop(SLAVE_KP, SLAVE_KD, VX300S["stall_torque"])
        # settle at Q0 first (no gravity FF => static sag)
        for _ in range(1500):
            slave.step(loop(Q0, slave.q, slave.dq))
        u = Q0.copy()
        u[j] += 0.15
        q0 = slave.q[j]
        ys = []
        for k in range(600):
            slave.step(loop(u, slave.q, slave.dq))
            if k % 5 == 0:
                ys.append(slave.q[j] - q0)
        y = np.array(ys)
        t = np.arange(len(y)) * 5 * DT_SIM
        gain = y[-1] / 0.15

        def res(p):
            return simulate_2nd_order(p[0], p[1], t, 0.15 * gain) - y
        sol = least_squares(res, x0=[20.0, 0.8], bounds=([1.0, 0.1], [200.0, 3.0]))
        omegas[j], zetas[j] = sol.x
    return omegas, zetas


class OurRunner:
    def __init__(self, params: BilateralParams, master_model, slave_model, estimator="momentum"):
        self.ctrl = BilateralController(params, master_model, slave_model,
                                        VX300S["q_min"], VX300S["q_max"])
        self.mi = ArmInterface("rx200", master_model, DT_CTRL, 0.3, estimator="none")
        self.si = ArmInterface("vx300s", slave_model, DT_CTRL, 0.3, estimator=estimator)

    def step(self, t, q_m, I_m, q_s, I_s):
        ms = self.mi.update(q_m, I_m, t)
        ss = self.si.update(q_s, I_s, t)
        tau_m, q_cmd, diag = self.ctrl.step(ms, ss)
        diag["tau_hat_s"] = ss.tau_ext
        return tau_m, q_cmd, diag


def run(name, controller_kind, params=None, delay_steps=0, seed=0, T=T_TOTAL, estimator="momentum",
        intent_fn=None, env_fn=None, wall_k=2000.0):
    """
    intent_fn(t) -> operator's intended master pose (default: human_intent)
    env_fn(t, slave, wall) -> force [N] on the slave end-effector (default: the
        wall, active after T_FREE). Extra logs: true/estimated slave external
        joint torque, master feedback torque, 3-D forces on both arms.
    """
    intent_fn = intent_fn or human_intent
    rng = np.random.default_rng(seed)
    master_model = ArmModel(os.path.join(URDF_DIR, "rx200.urdf"))
    slave_model = ArmModel(os.path.join(URDF_DIR, "vx300s.urdf"))
    master = Arm(os.path.join(URDF_DIR, "rx200.urdf"), Q0, ee_body="rx200/ee_gripper_link")
    slave = Arm(os.path.join(URDF_DIR, "vx300s.urdf"), Q0, ee_body="vx300s/ee_gripper_link")
    loop = DynamixelPositionLoop(SLAVE_KP, SLAVE_KD, VX300S["stall_torque"])
    wall = Wall(x_w=slave_model.fk(Q0)[0] + 0.03, k=wall_k)

    if controller_kind == "legacy":
        ctrl = LegacyController(master_model, slave_model, DT_CTRL, fix_units=False)
    elif controller_kind == "legacy_units_fixed":
        ctrl = LegacyController(master_model, slave_model, DT_CTRL, fix_units=True)
    else:
        ctrl = OurRunner(params, master_model, slave_model, estimator)
        # with delay, master side and slave side run their own instance and
        # each sees the other arm's data delayed (distributed implementation)
        ctrl_slave_side = OurRunner(params, master_model, slave_model, estimator) if delay_steps else ctrl

    # pre-settle the slave at Q0
    for _ in range(1500):
        slave.step(loop(Q0, slave.q, slave.dq))

    tau_m_applied = np.zeros(5)
    tau_m_cmd_prev = master_model.gravity(Q0)
    q_cmd = slave.q.copy()
    tau_s_motor = np.zeros(5)
    to_slave = deque([None] * (delay_steps + 1), maxlen=delay_steps + 1)
    to_master = deque([None] * (delay_steps + 1), maxlen=delay_steps + 1)

    log = {k: [] for k in ("t", "q_m", "q_s", "q_ref", "f_env", "f_felt", "contact",
                           "tank", "mpc_ms", "tau_cmd", "q_s_meas", "I_s", "tau_ext_true",
                           "tau_hat_s", "tau_felt", "f_env3", "f_felt3", "gamma", "u")}
    n_ctrl = int(T / DT_CTRL)
    unstable = False
    for k in range(n_ctrl):
        t = k * DT_CTRL
        q_m_meas = quantize(master.q, ENC)
        q_s_meas = quantize(slave.q, ENC)
        I_m = sensor_effort_mA(tau_m_cmd_prev, RX200["kt"], RX200["n_motors"], rng)
        I_s = sensor_effort_mA(tau_s_motor, VX300S["kt"], VX300S["n_motors"], rng)

        # communication delay (both directions) for the robustness test
        to_slave.append((q_m_meas, I_m))
        to_master.append((q_s_meas, I_s))
        q_m_seen, I_m_seen = to_slave[0] if isinstance(to_slave[0], tuple) else (q_m_meas, I_m)
        q_s_seen, I_s_seen = to_master[0] if to_master[0] is not None else (q_s_meas, I_s)

        if controller_kind.startswith("legacy"):
            I_cmd_mA, q_cmd, diag = ctrl.step(q_m_seen if delay_steps else q_m_meas, I_m,
                                              q_s_seen, I_s_seen)
            tau_m_applied = RX200["n_motors"] * RX200["kt"] * I_cmd_mA * 1e-3
            tau_m_cmd_prev = tau_m_applied
            diag.setdefault("tank", np.nan)
            diag.setdefault("mpc_ms", 0.0)
        else:
            tau_m, q_cmd, diag = ctrl.step(t, q_m_meas, I_m, q_s_seen, I_s_seen)
            if delay_steps:
                _, q_cmd, _ = ctrl_slave_side.step(t, q_m_seen, I_m_seen, q_s_meas, I_s)
            tau_m_applied = np.clip(tau_m, -RX200["stall_torque"], RX200["stall_torque"])
            tau_m_cmd_prev = tau_m_applied

        f_env = np.zeros(3)
        for _ in range(SUB):
            ts = t + _ * DT_SIM
            tau_h = HUMAN_K * (intent_fn(ts) - master.q) - HUMAN_B * master.dq
            master.step(tau_m_applied + tau_h)
            if env_fn is not None:
                f_env = env_fn(ts, slave, wall)
            else:
                f_env = (wall.force(slave.ee_pos(), slave.ee_jacobian() @ slave.dq)
                         if ts >= T_FREE else np.zeros(3))
            tau_s_motor = loop(q_cmd, slave.q, slave.dq)
            slave.step(tau_s_motor, f_env)

        if not np.all(np.isfinite(master.q)) or np.abs(master.q).max() > 10:
            unstable = True
            break

        # force the operator feels (net non-gravity torque from the controller), in Cartesian x
        tau_felt = tau_m_applied - master.gravity_true()
        Jm = master.ee_jacobian()
        f_felt = np.linalg.lstsq(Jm.T, tau_felt, rcond=None)[0]

        log["t"].append(t)
        log["q_m"].append(master.q)
        log["q_s"].append(slave.q)
        log["q_ref"].append(ctrl.ctrl.map_to_slave(master.q) if hasattr(ctrl, "ctrl") else master.q)
        log["f_env"].append(f_env[0])
        log["f_felt"].append(-f_felt[0])   # reaction felt against the push direction
        log["contact"].append(f_env[0] != 0.0)
        log["tank"].append(diag.get("tank", np.nan))
        log["mpc_ms"].append(diag.get("mpc_ms", 0.0))
        log["tau_cmd"].append(tau_m_applied)
        log["q_s_meas"].append(q_s_meas)
        log["I_s"].append(I_s)
        log["tau_ext_true"].append(slave.ee_jacobian().T @ f_env)
        log["tau_hat_s"].append(diag.get("tau_hat_s", np.full(5, np.nan)))
        log["tau_felt"].append(tau_felt)
        log["f_env3"].append(f_env)
        log["f_felt3"].append(f_felt)
        log["gamma"].append(diag.get("gamma", np.nan))
        log["u"].append(np.asarray(q_cmd, float).copy())

    L = {k: np.array(v) for k, v in log.items()}
    res = dict(name=name, unstable=unstable)
    if len(L["t"]) > 10:
        tm = tracking_metrics(L["t"], L["q_m"], L["q_s"], ARM)
        res["tracking_all"] = tm["overall"]
        res["tracking_joints"] = tm["joints"]
        free = L["t"] < T_FREE
        res["tracking_free"] = tracking_metrics(L["t"][free], L["q_m"][free], L["q_s"][free], ARM)
        res["force"] = force_metrics(-L["f_env"], L["f_felt"], L["contact"])
        res["force"]["max_wall_force"] = float(np.max(-L["f_env"])) if len(L["f_env"]) else 0.0
        res["mpc_ms_mean"] = float(np.mean(L["mpc_ms"]))
        res["mpc_ms_max"] = float(np.max(L["mpc_ms"]))
        res["master_torque_saturation_frac"] = float(np.mean(
            np.any(np.isclose(np.abs(L["tau_cmd"]), np.abs(L["tau_cmd"]).max(axis=0)) &
                   (np.abs(L["tau_cmd"]) > 0), axis=1)))
    return res, L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    print("Identifying slave position loop ...")
    omega, zeta = identify_position_loop()
    print("  omega =", np.round(omega, 1), " zeta =", np.round(zeta, 2))

    base = BilateralParams(pos_loop_omega=omega, pos_loop_zeta=zeta, dt=DT_CTRL)
    runs = [
        ("Original code (as in repo)", "legacy", None, 0),
        ("Original code, units fixed only", "legacy_units_fixed", None, 0),
        ("B1 Unilateral position", "ours", replace(base, mode="unilateral"), 0),
        ("B2 Position-position", "ours", replace(base, mode="pp"), 0),
        ("B3 Position-force (2ch)", "ours", replace(base, mode="pf", use_tank=False, force_map="identity"), 0),
        ("Proposed (MISA+MPC+J-map+tank)", "ours", base, 0),
        ("Ablation: no MPC", "ours", replace(base, use_mpc=False), 0),
        ("Ablation: no tank", "ours", replace(base, use_tank=False), 0),
        ("Ablation: identity force map", "ours", replace(base, force_map="identity"), 0),
        ("Ablation: horizon N=3", "ours", replace(base, mpc_horizon=3), 0),
        ("Ablation: kof=0.6", "ours", replace(base, kof=0.6), 0),
        ("Ablation: kof=1.5, no tank", "ours", replace(base, kof=1.5, use_tank=False), 0),
        ("Ablation: kof=1.5, tank", "ours", replace(base, kof=1.5), 0),
        ("Delay 20 ms: proposed", "ours", base, 1),
        ("Delay 20 ms: position-force", "ours", replace(base, mode="pf", use_tank=False, force_map="identity"), 1),
        ("Delay 60 ms: proposed", "ours", base, 3),
        ("Delay 60 ms: proposed, kof=0.6", "ours", replace(base, kof=0.6), 3),
        ("Delay 60 ms: position-position", "ours", replace(base, mode="pp"), 3),
        ("Delay 60 ms: position-force", "ours", replace(base, mode="pf", use_tank=False, force_map="identity"), 3),
    ]
    if args.quick:
        runs = runs[:6]

    results, logs = [], {}
    for name, kind, params, delay in runs:
        print(f"Running: {name}")
        res, L = run(name, kind, params, delay)
        results.append(res)
        logs[name] = L

    with open(os.path.join(args.out, "results.json"), "w") as f:
        json.dump(dict(software=software_versions(),
                       identified=dict(omega=omega.tolist(), zeta=zeta.tolist()),
                       results=results), f, indent=2)
    write_markdown(results, omega, zeta, os.path.join(args.out, "results.md"))
    if not args.no_plots:
        plot(logs, args.out)
    print(open(os.path.join(args.out, "results.md")).read())


def write_markdown(results, omega, zeta, path):
    lines = ["# MuJoCo bilateral benchmark (RX200 -> VX300s)", "",
             f"Identified slave position loop: ω = {np.round(omega, 1).tolist()} rad/s, "
             f"ζ = {np.round(zeta, 2).tolist()}", "",
             "| Run | no blow-up | RMSE pooled (rad) | RMSE free (rad) | max err (rad) | lag (ms) | "
             "lag-comp RMSE | force RMSE (N) | felt/true peak (N) | force corr | MPC ms (mean/max) |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        if "tracking_all" not in r:
            lines.append(f"| {r['name']} | NO | – | – | – | – | – | – | – | – | – |")
            continue
        a, fr, fo = r["tracking_all"], r["tracking_free"]["overall"], r["force"]
        lines.append(
            f"| {r['name']} | {'NO' if r['unstable'] else 'yes'} | {a['rmse_pooled']:.3f} | "
            f"{fr['rmse_pooled']:.3f} | {a['max']:.3f} | {1e3 * fr['lag_s_mean']:.0f} | "
            f"{fr['rmse_lag_comp_pooled']:.3f} | {fo['rmse']:.2f} | "
            f"{fo['peak_rendered']:.1f}/{fo['peak_true']:.1f} | {fo['corr']:.2f} | "
            f"{r['mpc_ms_mean']:.2f}/{r['mpc_ms_max']:.2f} |")
    open(path, "w").write("\n".join(lines) + "\n")


def plot(logs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pick = [n for n in logs if n.startswith(("Original code (as", "B2", "B3", "Proposed"))]
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for n in pick:
        L = logs[n]
        ax[0].plot(L["t"], L["q_s"][:, 1] - L["q_m"][:, 1], label=n)
        ax[1].plot(L["t"], L["f_felt"], label=n)
    L = logs[pick[-1]]
    ax[0].set_ylabel("shoulder error q_s − q_m (rad)")
    ax[1].plot(L["t"], -L["f_env"], "k--", label="true wall force")
    ax[1].set_ylabel("force felt by operator (N)")
    ax[2].plot(L["t"], L["tank"])
    ax[2].set_ylabel("tank energy (J), proposed")
    ax[2].set_xlabel("time (s)")
    for a in ax[:2]:
        a.legend(fontsize=7)
        a.axvline(T_FREE, color="grey", lw=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "comparison.png"), dpi=130)

    fig, ax = plt.subplots(5, 1, figsize=(10, 11), sharex=True)
    for j, name in enumerate(ARM):
        for n in ("Original code (as in repo)", "Proposed (MISA+MPC+J-map+tank)"):
            if n in logs:
                ax[j].plot(logs[n]["t"], logs[n]["q_s"][:, j], label=f"slave, {n}")
        ref = logs[pick[-1]]
        ax[j].plot(ref["t"], ref["q_m"][:, j], "k--", lw=0.8, label="master")
        ax[j].set_ylabel(name)
    ax[0].legend(fontsize=7)
    ax[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "joint_tracking.png"), dpi=130)


if __name__ == "__main__":
    main()
