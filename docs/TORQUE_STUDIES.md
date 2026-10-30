# Torque studies — results and descriptions for the paper

Reproduce: `python3 experiments/bilateral_sim/torque_studies.py` (~5 min).
Outputs: `experiments/bilateral_sim/results/torque/` (`fig_S1…S5.png`, `studies.md`, `studies.json`).

## Common setup (state this once in the paper)
The simulation is in MuJoCo (1 kHz physics, 50 Hz control):
- **Arms:** RX200 master and VX300s slave, built from the same URDFs the controller uses.
- **Model mismatch:** link masses +8 % and joint friction (Coulomb 0.01–0.08 N·m) that the controller doesn't model.
- **Sensors:** 4096-count encoders, and motor current reported like the Interbotix driver (mA, 2.69 mA steps, 5 mA noise).
- **Slave:** driven like the xs_sdk position mode (saturated PD, no gravity feed-forward).
- **Master:** torque-driven (current mode).
- **Operator:** a joint-space spring-damper towards an intended pose.
- **Ground truth:** external loads are applied at the slave gripper, so the true external joint torque τ_ext = J_sᵀF is known exactly. That's what makes these studies possible without a force/torque sensor.
- **Force-feedback gain:** k_of = 0.3 unless stated, so the ideal rendered force is 0.3·F_env.

---

## S1 — Sensorless external-torque estimation (`fig_S1_estimation.png`)
**What:** the slave follows a moving master (no feedback, so every estimator sees identical data) while
- 0.2 kg and then 0.5 kg masses are hung on the gripper, and
- a 4.5 N lateral push is applied.

Four estimators are run on the same logged current and position:
- the original code;
- the original code with only the units fixed;
- the static residual G + C q̇ − τ_m;
- the momentum observer.

**Results**

| Estimator | RMSE under load, shoulder/elbow/wrist (N·m) | false torque in free motion (N·m RMS) |
|---|---|---|
| Original code | 1786 / 2185 / 824 | 1340 / 1591 / 365 |
| Original, units fixed | 1.67 / 2.02 / 0.37 | 1.22 / 1.43 / 0.36 |
| Static residual (fixed) | 0.25 / 0.30 / 0.06 | 0.29 / 0.34 / 0.08 |
| Momentum observer | 0.27 / 0.30 / 0.06 | 0.28 / 0.31 / 0.07 |

| Slave speed | false shoulder torque: static | momentum |
|---|---|---|
| ×1 | 0.25 | 0.27 |
| ×2 | 0.48 | 0.44 |
| ×3 | 1.08 | 0.75 |

**Caption:** *Estimated vs. true external joint torque on the VX300s slave during motion, under 0.2 kg and 0.5 kg payloads and a 4.5 N push. No force sensor is used: τ̂_ext is computed from motor current and joint positions only.*

**Interpretation for the text**
- The original estimator was wrong by three orders of magnitude (mA treated as A). Even with the units fixed, it still carries about 1.5 N·m errors from the joint-index bug in the gravity model.
- After the fixes, a 0.2 kg object (≈ 0.9 N·m at the shoulder) is clearly resolved. The residual error of about 0.3 N·m is mostly the 8 % mass mismatch and joint friction. On hardware, calibrate k_t and the link masses with known weights.
- In slow motion the static residual and the momentum observer are equivalent. As speed rises, the static residual's error grows, because it neglects M q̈; at 3× speed the observer is about 30 % better.

### S1b — The estimator inside the closed force loop
| Case | RMSE (rad) | peak wall force (N) | false shoulder torque in free motion (N·m) |
|---|---|---|---|
| Proposed, k_of 0.3, static | 0.012 | 4.2 | 0.50 |
| Proposed, k_of 0.3, momentum | 0.011 | 4.3 | 0.45 |
| Proposed, k_of 0.6, static | **0.110** | **73.4** | **5.87** |
| Proposed, k_of 0.6, momentum | **0.012** | **12.6** | 0.46 |
| Position–force, k_of 0.6, static | 0.238 | 110.5 | 11.30 |
| Position–force, k_of 0.6, momentum | 0.100 | 99.3 | 0.47 |

**Interpretation:** the estimator's main effect shows up in closed loop. With the static residual, a force gain of 0.6 makes the master and slave excite each other: the "contact" estimate reaches 5–11 N·m in free space, and the arm presses the wall with 73–110 N. With the momentum observer, the same gain is stable (0.012 rad, 12.6 N). **The observer roughly doubles the usable force gain.**


---

## S2 — Torque mimicking (`fig_S2_mimicking.png`)
**What:** this is the experiment you wanted to show: "if arm 1 gets an external force, arm 2 should experience it". The operator holds the master still. Known forces act on the slave gripper:
- an 8 N push along −x (2–6 s);
- a 6 N downward load (7–11 s);
- a ±4 N, 0.5 Hz oscillation along x (12–18 s).

The force that the master exerts on the operator's hand is compared with the ideal k_of·F_env.

| Controller | force RMSE x,z (N) | corr. x | corr. z | gain x | gain z |
|---|---|---|---|---|---|
| Original code | 3.41 | 0.53 | 0.25 | −0.12 | 1.13 |
| Position–force (2-ch) | 0.34 | 0.96 | 0.98 | 0.94 | 1.40 |
| **Proposed** | 0.43 | 0.94 | 0.98 | 1.02 | 1.60 |
| Proposed, k_of = 0.6 | 0.59 | 0.96 | 0.97 | 0.91 | 1.30 |

(The gain is the least-squares ratio of felt force to k_of·F_env; 1 means exact scaled reproduction.)

**Caption:** *Torque mimicking. External forces applied to the slave gripper (dashed: scaled ideal) are reproduced at the operator's hand by the master. Bottom: master joint torques versus the ideal mapped torque k_of·J_mᵀF_env.*

**Interpretation**
- The original code renders a constant 1 N (x) and 4 N (z) regardless of the applied force. That bias is the master sagging under gravity, because its current cap is below the gravity torque. This is the "no change at all" behaviour you saw on hardware.
- After the fixes, the master reproduces the timing and shape of every load (correlation 0.94–0.98), including the 0.5 Hz oscillation.
- The bottom panel shows the magnitudes aren't exact. The elbow carries more torque than ideal and the shoulder less (see S2b).

### S2b — What sets the rendered magnitude
| Case | gain x | gain z |
|---|---|---|
| Proposed | 1.02 | 1.60 |
| Proposed, k_om = 0 (no position channel) | 0.70 | 1.11 |
| Proposed, identity map | 1.33 | 1.89 |
| Position–force, identity map | 0.94 | 1.40 |
| Position–force, Jacobian map | 0.67 | 1.10 |

**Interpretation:** two effects add up in the felt force.
1. **Kinematic mismatch.** Reflecting joint torques one-to-one between arms with different link lengths over-renders vertical loads by about 1.4× (the VX300s is longer than the RX200). The Jacobian mapping removes this (1.40 → 1.10).
2. **The position channel adds force under sustained load.** The slave yields a little under load (admittance plus position-loop sag), so k_om·K_md(q_m − q_s) pushes the master in the same direction as the force channel (1.10 → 1.60).

This is the classic four-channel transparency trade-off (Lawrence 1993). Exact transparency needs the force and position channels tuned together, e.g. by reducing k_om or compensating the expected slave deflection K_sd⁻¹τ̂. **Report this honestly as a design insight, not as a solved problem.**

---

## S3 — Object weight rendering (`fig_S3_weight.png`)
**What:** the slave holds a mass of 0–0.75 kg with the master held still. The steady downward force felt at the master is measured.

| m (kg) | m·g (N) | Original | Position–force | Proposed |
|---|---|---|---|---|
| 0.00 | 0.00 | 4.08 | 0.21 | 0.22 |
| 0.20 | 1.96 | 4.08 | 0.93 | 1.01 |
| 0.50 | 4.91 | 4.08 | 2.05 | 2.33 |
| 0.75 | 7.36 | 4.08 | 3.00 | 3.43 |

**Caption:** *Weight of an object held by the slave as felt at the master (ideal: k_of·m·g).*

**Interpretation**
- The original code gives the same 4.08 N for every mass, so the operator can't feel weight at all.
- The fixed controller is linear in the mass, with slope 0.44 for the proposed controller and 0.38 for position–force, against the ideal 0.3 (the over-rendering is explained by S2b).
- The 0.2 N offset at 0 kg is the 8 % gravity-model error on the master. It's a calibration target, not a controller issue.

---

## S4 — Environment stiffness (`fig_S4_stiffness.png`)
**What:** the 24 s benchmark task (free motion, then two pushes into a wall), with wall stiffness from 500 to 10 000 N/m.

| Controller | peak wall force (N), k = 500 → 10 000 | felt/true | RMSE (rad) |
|---|---|---|---|
| Position–position | 5.7 → 9.4 | ≈ 0 | 0.025–0.027 |
| Position–force | 3.0 → 3.6 | 0.31–0.33 | 0.024 |
| **Proposed** | 3.5 → 4.9 | 0.30–0.31 | **0.011–0.012** |

**Caption:** *Contact behaviour across environment stiffness: peak contact force, ratio of rendered to true force (dashed: k_of), and tracking RMSE.*

**Interpretation**
- With force feedback the operator stops at a contact force that hardly depends on the wall stiffness (3–5 N). Without it (P-P) contact force grows with stiffness, up to 9.4 N.
- The rendered/true ratio stays at k_of over a 20× stiffness range: the force channel is calibrated, not tuned to one environment.
- The proposed controller keeps about 50 % lower tracking error (0.011–0.012 rad against 0.024–0.027 rad) than both baselines throughout.

---

## S5 — Stability map: force gain × delay (`fig_S5_stability.png`)
**What:** the benchmark task for k_of ∈ {0.1, 0.3, 0.6, 1.0, 1.5} and one-way delay ∈ {0, 20, 40, 60 ms}, with and without the energy tank. Each cell shows the maximum tracking error (above 0.5 rad means the operator loses control).

| k_of | 0 ms, no tank → tank | 20 ms | 60 ms |
|---|---|---|---|
| 0.3 | 0.12 → 0.12 | 0.21 → 0.21 | 0.27 → 0.27 |
| 0.6 | 0.15 → 0.15 | 0.73 → 0.73 | 1.53 → 1.38 |
| 1.0 | **0.87 → 0.50** | 0.82 → 0.88 | 2.62 → 1.94 |
| 1.5 | **1.99 → 0.72** | 0.96 → 0.94 | 2.37 → 2.25 |

**Caption:** *Maximum tracking error versus force-feedback gain and communication delay, without (left) and with (right) the energy tank.*

**Interpretation**
- Without delay, the tank reduces the worst-case error at k_of = 1.0 from 0.87 to 0.50 rad and at 1.5 from 1.99 to 0.72 rad, but neither is below the 0.5 rad "usable" line by a margin, so the safe gain stays at k_of ≤ 0.6. This is the passivity property of Eq. 8 working as designed.
- **With delay the single tank does not help reliably** (differences of ±0.1–0.7 rad that change sign between cells; a single run per cell, so treat them as noise-level). It only guards the master port; under delay, the energy comes in on the slave side as it tracks the delayed master.
- k_of ≤ 0.3 is the safe default region, which is why it is the default.

**Say it in the paper:** extending passivity to delayed operation needs a two-port tank or TDPA with energy exchange between the sites (future work).

---

## Suggested figure set for the revision
1. **S2** (torque mimicking): the headline figure, since it shows exactly the capability that couldn't be demonstrated on hardware.
2. **S1**: answers "the force-feedback path is never measured".
3. **S5**: stability and the role of passivity.
4. **S3 or S4**: whichever fits; S4 is the stronger baseline comparison.

These are **simulation** results. The hardware protocol in `HARDWARE_VALIDATION.md` (known masses, force against a scale) repeats S1 and S3 on the real arms.
