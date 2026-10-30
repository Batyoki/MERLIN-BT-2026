# Evaluation methodology, and what changed in the MPC model

This document answers three questions that came up while revising the paper:

1. Did the dynamics used by the MPC change, or only its tuning? (§1)
2. Where does each number in the new result tables come from: which software, which plant, which formula? (§2–§5)
3. What do those numbers prove, and what do they not prove? (§7)

**Short answer to the third question:** all new numbers are from simulation. They are not comparable to the
hardware numbers in the paper's Table I, and they must be labelled as simulation wherever they are quoted.

---

## 1. Did the MPC dynamics change?

**Yes. The model class was replaced, not re-tuned.** The old optimizer modelled the slave as a rigid body driven by
joint *torques*. The VX300s is driven by joint *position* commands through the Dynamixel position loop; it has no
torque input, so the torque the old optimizer computed was never applied. The revised optimizer models the actuator
the controller really has: the closed position loop, identified per joint.

"Paper" below is Eq. 7 of the submitted draft, "Code" is the repository at commit `0658f52`
(`mpc_optimizer.py` + `misa_mpc_controller.py`). The paper and the code already differed from each other.

| | Paper (Eq. 7) | Original code | Revised controller |
|---|---|---|---|
| Input the optimizer chooses | joint torques τ₀…τ_{N−1} | velocities dq₀…dq_{N−1} **and** torques τ₀…τ_{N−1} | joint position commands u₀…u_{N−1} |
| Dynamics model | ẋ = [q̇; M(q)⁻¹(τ − C(q,q̇) − G(q))], explicit Euler | q̈ = pinv(M)(τ − C − G + τ_ext), **constant M** from `data.json`, C and G frozen at the current state for the whole horizon; q advanced with a second-order Taylor step | per joint: q̈ = ω_i²(u − q) − 2ζ_iω_i q̇, exact zero-order-hold discretisation; ω_i, ζ_i identified from step responses; plus an output-disturbance estimate d̂ |
| Does the decision variable enter the model? | yes (τ) | **τ yes, dq no**: dq appears only in the cost | yes (u) |
| Cost | Σ ‖q−r‖²_Q + ‖τ‖²_R + ‖Δτ‖²_S + terminal ‖·‖²_P | Σ eᵀQe + dqᵀR dq + τᵀSτ + terminal (no Δτ; the k = 0 tracking term is constant) | Σ Q(q̂+d̂−r)² + terminal P + R Σ (Δu)² |
| Constraints | q, q̇, τ in boxes | box bounds on dq and τ only; **`q_min`/`q_max` are stored but never used** | u in the joint limits, \|Δu\| ≤ Δu_max, predicted \|q̇\| ≤ q̇_max |
| Value applied to the slave | τ₀\* | dq₀\* (then integrated) | u₀\* |
| Solver | – | SciPy SLSQP (general NLP, 48 variables for 8 joints), 17–48 ms per solve | condensed QP per joint, ADMM (ρ = 0.3, 60 iterations) with a pre-factorised matrix, 0.9 ms mean |
| Horizon | N = 3 (60 ms) | N = 3 | N = 10 (200 ms) |
| Reference | admittance reference q_{s,ref} | same | same, previewed with the master velocity |

**What stayed the same:** the receding-horizon principle, the 50 Hz loop, a quadratic tracking cost with a terminal
weight, box constraints on the quantities the optimizer chooses, and the admittance-generated reference q_{s,ref}.

**A solver finding that changed the numbers (disclosed).** The first version of the revised MPC used an ADMM penalty
ρ = 10, which at the 60-iteration budget was *not converged* to the QP optimum (the iterate was biased towards the
previous command). Measuring the solution against a reference SLSQP solve (`test_admm_matches_reference_qp`) showed
that ρ = 0.3 converges to the exact QP solution within the same budget, so it became the default (`mpc_rho`,
`mpc_iters` in `controller.yaml`). Effect, same plant and seeds: free-motion RMSE 0.016 → 0.007 rad and lag 8 → 0 ms.
All simulation tables in this repository were regenerated with ρ = 0.3. Earlier numbers (including the 0.019 rad
quoted in an earlier revision of this file) are superseded.

**Why it had to change**

1. *Interface.* A torque model cannot drive a position-controlled arm.
2. *Dead decision variable.* Because dq never enters the rollout, the minimum over dq is at dq = 0 for any
   reference. Checked on the original optimizer: with reference errors of 0.4–0.7 rad it returned
   `max|dq₀*| = 0.00` in 5 of 5 trials. The slave therefore moved only through the smoothing step
   `q ← 0.95 q + 0.05 q_ref` (a first-order lag with time constant ≈ 0.39 s at 50 Hz).
3. *Missing constraint.* The joint-limit constraint stated in the paper was not enforced.

**Effect on closed-loop behaviour (simulation, §6):** measured lag 338 ± 6 ms with the original code
(re-implemented) against 0 ms with the revised controller (the lag is resolved at the 20 ms control period, so 0 means
< 10 ms).

**Validity of the new model.** `experiments/bilateral_sim/model_validation.py` compares the N-step prediction of the
controller's own model with the simulated slave:

| Predictor | h=1 (20 ms) | h=2 (40 ms) | h=5 (100 ms) | h=10 (200 ms) |
|---|---|---|---|---|
| hold | 0.0120 | 0.0238 | 0.0576 | 0.1089 |
| const-vel | 0.0028 | 0.0078 | 0.0286 | 0.0719 |
| loop (d_hat = 0) | 0.0264 | 0.0264 | 0.0265 | 0.0266 |
| loop + d_hat (MPC model) | 0.0051 | 0.0061 | 0.0075 | 0.0068 |

The "loop + d̂" row is the model the MPC uses. Two honest observations: without the offset term (d̂ = 0) the model
is off by a constant ≈ 0.026 rad (the static sag of a position loop without gravity feed-forward), which is why the
offset-free term exists; and plain constant-velocity extrapolation is *better* than the model for the first 20–40 ms,
so the model's advantage is at the longer horizons the MPC actually uses (≥ 100 ms).
This validation is in simulation, where the plant's position loop is a PD law and the model was identified on that
same plant. It must be repeated on the real arm (`tools/identify_position_loop.py` reports R²; see
`docs/HARDWARE_VALIDATION.md`).

---

## 2. Where the numbers come from

| Numbers | Source | Controller | Plant |
|---|---|---|---|
| Paper Table I (RMSE 0.3583 rad pooled, 0.3418 mean of joints) | **hardware**, bimanual RX200/VX300s, human operator, free and contact motions | original code | real arms |
| `results.md`, `multi_seed.md`, `model_validation.md`, `torque/*` | **MuJoCo simulation** (this repository) | revised controller, original code re-implemented, baselines | simulated RX200/VX300s |

The 0.3583 rad (hardware, original controller) and the 0.007–0.016 rad (simulation, revised controller; free motion / whole task) are
**not** a before/after comparison of the same experiment: different plant, different operator, different motions.
The only like-for-like comparison available today is in simulation: original code (re-implemented) 0.164 ± 0.010 rad
against revised 0.016 ± 0.005 rad (§6). A hardware measurement of the revised controller is still outstanding.

**Fidelity check of the simulator (weak).** The re-implemented original controller reproduces the *structure* of the
hardware error, with the distal wrist-rotate joint worst and a lag-dominated error, but not its magnitude
(simulation 0.164 rad pooled, hardware 0.358 rad), because the hardware operator moved faster than the simulated one.

---

## 3. Software used

| Component | Version used for the reported numbers | Role |
|---|---|---|
| Python | 3.13.17 | interpreter |
| NumPy | 2.5.4 | arrays, linear algebra |
| SciPy | 1.18.1 | matrix exponential (discretisation), least squares (identification) |
| MuJoCo | 3.17.0 | rigid-body physics of both arms, contact |
| Pinocchio | 4.1.0 | controller-side gravity, Coriolis, mass matrix, Jacobians |
| OS | Linux-6.18.44-fc-v165-x86_64-with-glibc2.39 |  |

The repository's `requirements.txt` pins `mujoco==3.3.7`; the benchmark was run with the version above. The
versions are written to `results/results.json` and `results/multi_seed.json` on every run.

The arms are built from the same URDFs the controller uses (`ros_ws/src/controller/controller/{rx200,vx300s}.urdf`),
with meshes removed and inertials kept. The controller's rigid-body model (pinocchio) and the plant (MuJoCo) load
the same inertias; they are deliberately made different, see §4.

---

## 4. Simulation set-up

| Item | Value |
|---|---|
| Physics | MuJoCo, 1 ms step, implicit-fast integrator |
| Control loop | 50 Hz (Δt = 20 ms); the controller sees only sampled signals |
| Plant ≠ controller model | link masses and inertias ×1.08; joint damping [0.03, 0.05, 0.04, 0.02, 0.01] N·m·s/rad and Coulomb friction [0.04, 0.08, 0.06, 0.03, 0.01] N·m, neither known to the controller |
| Slave actuator | Dynamixel-like position loop τ = sat(K_p(u − q) − K_d q̇), K_p = [60, 150, 100, 25, 6] N·m/rad, K_d = [1.2, 3.0, 2.0, 0.25, 0.05] N·m·s/rad, saturated at stall torque, no gravity feed-forward. **These gains are my choice**, not measured on the arm |
| Master actuator | joint torque, as in Dynamixel current mode |
| Position-loop identification | step test of 0.15 rad per joint on the simulated slave, least-squares fit of (ω, ζ): ω = [25.7, 23.4, 31.1, 98.7, 94.6] rad/s, ζ = [0.27, 0.30, 0.42, 0.66, 0.46] |
| Sensors | encoders quantised to 2π/4096; motor current in mA with 2.69 mA steps and 5 mA Gaussian noise (the xs_sdk resolution); torque constants k_t as in `arm_interface.py` |
| Operator | joint-space spring-damper (K_h = [6, 10, 6, 2, 0.5] N·m/rad, B_h = [0.2, 0.3, 0.2, 0.05, 0.02]) pulling the master towards an intended pose |
| Task (24 s) | 0–12 s free motion, sinusoids on all joints (amplitudes [0.6, 0.25, 0.3, 0.4, 0.6] rad, 0.25–0.5 Hz); 12–24 s two slow pushes towards a wall |
| Wall | penalty contact on the slave end effector, plane x = x₀ + 0.03 m, k = 2000 N/m, b = 30 N·s/m (varied in S4) |
| Ground truth | the wall force is known exactly (F_env), hence also the true external joint torque τ_ext = J_sᵀF_env |
| Controller parameters | defaults of `BilateralParams` (`misa_mpc_controller.py`): k_of = 0.3, k_om = 1, N = 10, energy tank on |
| Original controller | `legacy.py`: a faithful re-implementation of `controller.py` + `arm_interface.py` + `misa_mpc_controller.py` at commit `0658f52`, with the declared default gains (the YAML was never loaded). The MPC is replaced by its verified output dq\* = 0. The real original node was **not** run (it needs ROS and the arms) |

---

## 5. Metric definitions

Let e[k, j] = q_s[k, j] − q_m[k, j] be the slave minus the (mapped) master joint position at tick k, joint j
(the scale S = I and offset o = 0 in simulation), K ticks, J = 5 joints.

* **Per-joint RMSE**  RMSE_j = √( (1/K) Σ_k e[k, j]² ); **MAE_j** = (1/K) Σ_k |e[k, j]|; **Max_j** = max_k |e[k, j]|.
* **Pooled (overall) RMSE**  √( (1/(KJ)) Σ_{k,j} e[k, j]² ) = √( (1/J) Σ_j RMSE_j² ).
  This is the number to quote as "overall RMSE". The *mean of the per-joint RMSEs*, (1/J) Σ_j RMSE_j, is always ≤ the
  pooled value (quadratic mean ≥ arithmetic mean). On the paper's Table I: pooled 0.3583 rad, mean of joints 0.3418 rad;
  the draft's conclusion quoted the latter while the table's "Overall" row is the former. Both are correct numbers
  for different quantities, which is why the definition must be stated.
* **Lag**  s\* = argmax over 0 ≤ s ≤ 1 s of the Pearson correlation of q_m[0..K−s) and q_s[s..K); lag_j = s\*Δt;
  reported as the mean over joints. Resolution is one tick (20 ms) per joint.
* **Lag-compensated RMSE** RMSE of q_s[s\*..] − q_m[..K−s\*), pooled over joints: the error that remains after
  removing a pure delay (the shape error).
* **Free-motion** metrics use the first 12 s only; **whole-task** metrics use all 24 s, so they include the intended
  compliance of the slave while it presses on the wall.
* **Peak wall force** max |F_env,x| over contact samples.
* **Force felt by the operator**  f_felt = −x-component of (J_mᵀ)⁺(τ_applied − τ_gravity,true) at the master end effector:
  the *net non-gravity torque the controller applies to the master*, mapped to a Cartesian force by least squares.
  It therefore includes the damping and position-coupling terms, not only the force channel.
* **Force correlation** Pearson correlation of f_felt and the true wall force over the contact samples.
* **Torque-estimation error (S1)** RMS of τ̂_ext[k+1] − τ_ext[k]: the estimate at a tick uses the current measured at the
  end of the previous tick; 0.3 s after each load change are excluded.
* **Rendered gain (S2)** least-squares slope of the felt force on k_of·F_env; 1 means exact scaled reproduction.

Per-run numbers come from `ros_ws/src/controller/controller/metrics.py`, shared with `tools/evaluate_tracking.py`,
so the same code produces the simulation tables and, later, the hardware table.

---

## 6. Results

### 6.1 Multi-seed benchmark

`experiments/bilateral_sim/multi_seed.py`: 5 seeds. Each seed draws a new sensor-noise realisation **and** a randomised
operator (amplitudes ±25 %, frequencies ±15 %, random phase per joint, wall-push amplitude ±20 %). All controllers
see the same operator and noise within a seed. Entries are mean ± sample standard deviation (n = 5).
The single-seed tables elsewhere (`results.md`, `TORQUE_STUDIES.md`) use the canonical seed 0, which is one
draw and must not be quoted without this table.

| Controller | RMSE pooled, whole task (rad) | RMSE pooled, free motion (rad) | max abs. error, free motion (rad) | max abs. error, whole task (rad) | lag, free motion (ms) | peak wall force (N) | force correlation (felt vs true) | proposed lower RMSE in |
|---|---|---|---|---|---|---|---|---|
| Original code (re-implemented) | 0.164 ± 0.010 | 0.220 ± 0.016 | 0.560 ± 0.087 | 0.735 ± 0.202 | 338.4 ± 6.1 | 0.6 ± 0.6 | -0.580 ± 0.029 | 5/5 |
| Unilateral position | 0.031 ± 0.002 | 0.025 ± 0.001 | 0.054 ± 0.003 | 0.229 ± 0.132 | 30.4 ± 2.2 | 21.1 ± 17.7 | -0.230 ± 0.308 | 5/5 |
| Position-position | 0.027 ± 0.001 | 0.024 ± 0.001 | 0.054 ± 0.003 | 0.207 ± 0.116 | 30.4 ± 2.2 | 8.3 ± 0.2 | 0.413 ± 0.235 | 5/5 |
| Position-force | 0.025 ± 0.002 | 0.025 ± 0.001 | 0.055 ± 0.004 | 0.231 ± 0.133 | 30.4 ± 2.2 | 22.5 ± 23.0 | 0.230 ± 0.639 | 5/5 |
| Proposed, no MPC | 0.026 ± 0.001 | 0.025 ± 0.001 | 0.054 ± 0.003 | 0.215 ± 0.116 | 20.0 ± 0.0 | 11.8 ± 17.9 | 0.296 ± 0.752 | 5/5 |
| Proposed | 0.016 ± 0.005 | 0.007 ± 0.001 | 0.039 ± 0.000 | 0.303 ± 0.195 | 0.0 ± 0.0 | 8.9 ± 7.7 | 0.482 ± 0.343 | – |

Per-joint, whole task, mean over seeds (rad):

| Controller | Joint | RMSE | MAE | Max | lag (ms) |
|---|---|---|---|---|---|
| Original code (re-implemented) | waist | 0.1705 | 0.1283 | 0.4830 | 376 |
| Original code (re-implemented) | shoulder | 0.0925 | 0.0782 | 0.2385 | 352 |
| Original code (re-implemented) | elbow | 0.0982 | 0.0704 | 0.2893 | 344 |
| Original code (re-implemented) | wrist_angle | 0.1430 | 0.0991 | 0.3707 | 312 |
| Original code (re-implemented) | wrist_rotate | 0.2537 | 0.1869 | 0.6272 | 280 |
| Position-position | waist | 0.0171 | 0.0094 | 0.1307 | 40 |
| Position-position | shoulder | 0.0218 | 0.0192 | 0.0799 | 36 |
| Position-position | elbow | 0.0372 | 0.0347 | 0.1224 | 40 |
| Position-position | wrist_angle | 0.0304 | 0.0243 | 0.1312 | 20 |
| Position-position | wrist_rotate | 0.0204 | 0.0127 | 0.0968 | 20 |
| Proposed | waist | 0.0129 | 0.0042 | 0.1449 | 8 |
| Proposed | shoulder | 0.0056 | 0.0032 | 0.0409 | 0 |
| Proposed | elbow | 0.0144 | 0.0100 | 0.0692 | 0 |
| Proposed | wrist_angle | 0.0193 | 0.0114 | 0.1427 | 0 |
| Proposed | wrist_rotate | 0.0142 | 0.0049 | 0.1681 | 0 |

How to read it

* **Free-motion tracking:** the revised controller is lower than every other controller in 5 of 5 seeds
  (0.007 ± 0.001 rad against 0.024–0.025 rad for the baselines), with 0 ms lag against 30 ms.
* **Whole-task maximum error:** the revised controller's is *larger* (0.30 ± 0.20 rad against ≈ 0.21–0.23 rad). In
  contact the admittance deliberately lets the slave deviate from the master (up to 0.3 rad), and this counts as
  "error" against the master pose. Report the average and the maximum together.
* **Force correlation in the benchmark task is moderate and noisy** (0.48 ± 0.34 for the revised controller;
  position-force 0.23 ± 0.64). In the benchmark the operator is also moving the arm, which confounds the felt-force
  signal. The cleaner torque-mimicking test (`TORQUE_STUDIES.md`, S2: operator holds the master still) is where
  correlations of 0.94–0.98 were obtained; those are one canonical scenario, and the benchmark shows how much
  these numbers vary with the operator.
* The original code almost never reaches the wall (peak 0.6 N) because its slave lags by about 0.34 s.

### 6.2 Single-seed studies
Torque estimation (S1), torque mimicking (S2), object weight (S3), stiffness (S4) and the force-gain × delay
stability map (S5) are described in `docs/TORQUE_STUDIES.md`. They use seed 0 and the same plant.

---

## 7. What the results do and do not show

**They show**, in simulation: the original code's slow response is explained by its own implementation (a 0.39 s
smoothing step and an MPC whose output is zero); the revised controller removes the lag and lowers free-motion error
by about 35 % relative to simple baselines; the momentum observer allows a higher stable force gain than the static
residual; the energy tank helps without communication delay but not with it.

**They do not show**

* that the revised controller works on the real arms: the position-loop model, the current-to-torque constants
  k_t, the dead zones and the gains are untested on hardware;
* anything about the real arms' backlash, velocity profiles, current limiting and communication jitter, none of
  which is modelled; the plant's position loop is a PD law *I chose*, and the model was identified on that same plant
  (the model validation in §1 is therefore partly in-sample);
* that the simulated operator resembles a person: it is a spring-damper;
* any statistical significance beyond n = 5 seeds of one task type; with a paired design the ranking was identical in
  5 of 5 seeds for free-motion RMSE, but this is not a hypothesis test;
* a proof of stability of the whole teleoperation loop. The energy tank bounds the energy delivered by the force
  channel (see `docs/derivations/`); it does not cover delay, and the simulation shows it does not help there.

Hardware measurements that replace each caveat are listed in `docs/HARDWARE_VALIDATION.md`.

---

## 8. Reproduce

```bash
pip install mujoco pin numpy scipy matplotlib
python3 experiments/bilateral_sim/run.py             # canonical benchmark -> results/results.{md,json}, figures
python3 experiments/bilateral_sim/multi_seed.py      # 5 seeds            -> results/multi_seed.{md,json}
python3 experiments/bilateral_sim/model_validation.py  # §1 table          -> results/model_validation.{md,json}
python3 experiments/bilateral_sim/torque_studies.py  # S1–S5              -> results/torque/
```
