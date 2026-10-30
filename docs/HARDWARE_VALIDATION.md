# Hardware validation protocol

**Status: nothing in this document has been run on the real arms.** All controller results in this repository
are MuJoCo simulation (`docs/EVALUATION_METHODOLOGY.md`). This protocol is what has to be done, in order, to
replace the simulation caveats by measurements. Each step lists what is measured, the pass criterion, and what to
do on failure. Record every number in a lab notebook / the `log_dir` CSVs; the paper's hardware results must come
from step 8 only.

Tools used: `python -m arm_bench.hardware_check` (cameras, joystick, WebXR loopback, ROS rates),
`tools/identify_position_loop.py`, `tools/evaluate_tracking.py`, the controller's CSV log (`log_dir:=`), and the
episode recorder (GUI "Record" tab or `arm-bench record`), which now also stores the commanded actions.

Safety: first runs with the slave's workspace clear, one hand on the power switch, `kof: 0.1`, operator
holding the master. Never enable `misa` before steps 1–5 pass.

---

## 0. Software pre-flight (no arms powered)

```bash
cd ros_ws && colcon build --packages-select controller && source install/setup.bash
python3 -m pytest ros_ws/src/controller/test -q           # 24 tests: control math + derivations
python3 -m arm_bench.hardware_check webxr                  # server code loopback (not the phone)
```

## 1. Cameras (multi-camera)

```bash
python3 -m arm_bench.hardware_check cameras --seconds 10          # all /dev/video* at the same time
python3 -m arm_bench.hardware_check cameras --ids 0 2 4 --fps 15
```

* **Measured:** frame rate, longest gap and jitter of every camera while all of them stream *simultaneously*
  (USB bandwidth problems only appear then), and the median offset between cameras.
* **Pass:** every camera ≥ 80 % of the requested fps, no gap > 0.5 s.
* **Note:** USB cameras are not hardware-synchronised. The offset printed is the error to expect when frames are
  paired by time; if it matters for learning, record `t_capture` (the recorder stores it per frame) and
  interpolate, or use a hardware-triggered camera.
* **On failure:** put cameras on separate USB controllers, lower resolution/fps, or use a powered hub.
* Then calibrate each camera (GUI "Calibrate camera") and keep the printed reprojection error in the notebook.

## 2. Joystick

```bash
python3 -m arm_bench.hardware_check joystick --seconds 10
```

* **Measured:** idle offset of every axis (stick untouched for the first 2 s) and the range reached when swept.
* **Pass:** idle offset < the dead-zone (0.1) on all axes used; every used axis reaches its two ends.
* **On failure:** raise `deadzone` in `JoystickTeleoperation` or recalibrate the device (`jstest-gtk`).

## 3. WebXR phone link

1. `python3 -m arm_bench.hardware_check webxr` passes on the PC (server code only).
2. Start the server (`arm-bench teleop --mode webxr` or the GUI), open `https://<pc-ip>:8443` on the phone, accept the
   self-signed certificate, start the AR session.
3. **Measured:** `/server-info` `clients_connected` becomes 1; `/latest_pose` changes when the phone moves.
   Round-trip latency: have the phone page timestamp a message and read the echo (the server broadcasts every pose
   back); record the median and 95th percentile over 60 s on the real WiFi.
* **Pass:** connection stable for 5 min, 95th-percentile round trip < 50 ms.
* Note the origin of the WebXR front-end code (`docs/` third-party notes in the README): it is adapted from
  `manipulator_teleop` and must stay credited.

## 4. ROS topics and rates

```bash
ros2 launch controller controller_misa.launch.py launch_drivers:=true control_mode:=unilateral log_dir:=/tmp/merlin
python3 -m arm_bench.hardware_check ros --seconds 10
```

* **Pass:** `joint_states` of both arms ≥ 40 Hz (the loop is 50 Hz); the slave `commands/joint_group` topic is seen.
* **Measured but not yet quantified anywhere:** the real latency of the loop. Log the `stamp` of `joint_states` and
  the age of the command; the revised controller assumes ≤ 1 sample (20 ms) one-way. The delay experiments
  (`TORQUE_STUDIES.md` S5) show the force channel degrades from 20 ms on.

## 5. Joint conventions and effort units

1. **Sleep pose offsets.** With both arms at the Interbotix sleep pose,
   `ros2 topic echo /masterR/joint_states --once`: the RX200 shoulder should read ≈ −1.80 rad. If it reads ≈ +1.31
   set `master_joint_offset[1] = -3.11`. Check every joint of both arms against `sleep_positions`.
2. **Effort unit and k_t calibration** (the sensorless force path depends on it). The slave is held still in a pose
   with the arm horizontal. Hang known masses (0.1, 0.2, 0.5 kg; verify them on a scale) at the gripper. For each
   mass record the shoulder and elbow currents and compare `τ = n_motors·k_t·I·10⁻³` (`arm_interface.py`) with
   `Jᵀ m g` from the model. Fit one `k_t` per joint type by least squares.
   * **Pass:** the fitted `k_t` is within ±15 % of the datasheet values in `arm_interface.py`; the residual after
     the fit is < 0.3 N·m (the same order as the simulated S1 residual, which is an *expectation*, not a result).
   * Put the fitted values into `kt` of the arm specs. The 8 % mass mismatch in simulation was an assumption.
3. **Friction and gravity-model error.** Move each joint slowly (0.05 rad/s) up and down with the controller in
   `unilateral` mode and record the master's current: the difference between up and down is 2× the Coulomb friction,
   their mean is the gravity-model error. Compare with the simulated 0.01–0.08 N·m.

## 6. Position-loop identification (the MPC model)

The slave model `q̈ = ω²(u − q) − 2ζωq̇ + c` was identified on the *simulated* arm (ω = 25.7…98.7 rad/s). It must be
identified on the real arm before the MPC is trusted.

1. `control_mode:=unilateral`, teleoperate in free space for 60 s with varied speeds, plus ±0.1 rad steps one joint
   at a time (`log_dir` CSV).
2. `python3 tools/identify_position_loop.py /tmp/merlin/bilateral_R_*.csv --plot`
3. **Pass:** R² of the fit ≥ 0.9 per joint (the tool prints it). A lower R² means the model class is wrong for that
   joint (e.g. backlash, profile velocity limits in the xs_sdk). Then keep `use_mpc: false` for it.
4. Paste ω, ζ into `pos_loop_omega/zeta` and re-run `python3 experiments/bilateral_sim/model_validation.py` logic on
   the real log (compare the N-step prediction error with the hold / const-velocity baselines: the model should beat
   both for h ≥ 5).

## 7. Staged bringing-up of the force path

| stage | `control_mode` | `kof` | what to check | stop if |
|---|---|---|---|---|
| a | `unilateral` | – | the master only compensates gravity; hand-guide it, the slave follows | the master drifts or buzzes |
| b | `pp` | – | position–position baseline; contact is felt as a position error | oscillation |
| c | `misa` | 0.1 | push the slave gently with a **scale** (kitchen scale under the gripper): the master torque must change in the same direction | wrong sign, or force felt in free motion |
| d | `misa` | 0.3 | repeat c at 3–4 force levels; compute the least-squares gain felt/(kof·F) as in S2 | the diagnostics `gamma` < 1 repeatedly (tank limiting), or any limit cycle |
| e | `misa` | 0.6 | only if d is clean | limit cycle: go back to 0.3 |

* **Force path measurement (what the paper is missing):** at stage d, with the slave pressing a force sensor or
  scale at known forces 1, 2, 4, 8 N, log the master's commanded and measured torque. Report the same two numbers
  as simulation S2: correlation and gain. Do not quote the simulated 0.94–0.98 / 1.0–1.6 for hardware.
* Weight rendering (S3): hang 0.1…0.5 kg on the slave gripper, master still; the steady master torque change must
  be linear in the mass.

## 8. The measurement that goes into the paper

Same protocol as paper Table I so that it is comparable: bimanual free + contact motions, human operator, 5 active
joints, controller log → `python3 tools/evaluate_tracking.py log.csv --markdown`. Run **both** the original code
(`git checkout` the archived version in `archive/`) and the revised controller on the same session if possible, and
≥ 5 trials with different operators/trajectories so a mean ± std can be given. Report pooled RMSE, MAE, max, lag and
lag-compensated RMSE per joint. Do **not** mix them with the simulation numbers in one table without a column
saying which is which.

## 9. Data collection run

Record 10 episodes with the GUI recorder (cameras + both arms). Check in `metadata.json`: `joint_state_source`,
`actions_recorded` (slave and master command topics), `dropped_frames` = 0, `calibrated_cameras` lists every camera.
Load `episode.h5` and verify `arms/<arm>/action` is not all-NaN and that `action_age_s` is below one control period.

---

## What was and was not verified without hardware

| item | verified here | how |
|---|---|---|
| hardware_check statistics (fps, gaps, skew, joystick offsets) | yes | unit tests on synthetic data |
| WebXR server handler (poses → callback, `/latest_pose`) | yes | loopback with a WebSocket test client |
| recorder stores commanded actions | yes | unit test with a stub command source |
| real cameras, joystick, phone, WiFi | **no** | needs the devices |
| `k_t`, position loop, friction on the real arms | **no** | steps 5–6 |
| force path on hardware, delay of the real loop | **no** | steps 4, 7 |
