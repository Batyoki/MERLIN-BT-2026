# Multi-seed benchmark (5 seeds, MuJoCo, RX200 -> VX300s)

mean ± sample std over seeds; each seed = new sensor noise and a randomised operator trajectory.

| Controller | RMSE pooled, whole task (rad) | RMSE pooled, free motion (rad) | max abs. error, free motion (rad) | max abs. error, whole task (rad) | lag, free motion (ms) | peak wall force (N) | force correlation (felt vs true) | proposed lower RMSE in |
|---|---|---|---|---|---|---|---|---|
| Original code (re-implemented) | 0.164 ± 0.010 | 0.220 ± 0.016 | 0.560 ± 0.087 | 0.735 ± 0.202 | 338.4 ± 6.1 | 0.6 ± 0.6 | -0.580 ± 0.029 | 5/5 |
| Unilateral position | 0.031 ± 0.002 | 0.025 ± 0.001 | 0.054 ± 0.003 | 0.229 ± 0.132 | 30.4 ± 2.2 | 21.1 ± 17.7 | -0.230 ± 0.308 | 5/5 |
| Position-position | 0.027 ± 0.001 | 0.024 ± 0.001 | 0.054 ± 0.003 | 0.207 ± 0.116 | 30.4 ± 2.2 | 8.3 ± 0.2 | 0.413 ± 0.235 | 5/5 |
| Position-force | 0.025 ± 0.002 | 0.025 ± 0.001 | 0.055 ± 0.004 | 0.231 ± 0.133 | 30.4 ± 2.2 | 22.5 ± 23.0 | 0.230 ± 0.639 | 5/5 |
| Proposed, no MPC | 0.026 ± 0.001 | 0.025 ± 0.001 | 0.054 ± 0.003 | 0.215 ± 0.116 | 20.0 ± 0.0 | 11.8 ± 17.9 | 0.296 ± 0.752 | 5/5 |
| Proposed | 0.016 ± 0.005 | 0.007 ± 0.001 | 0.039 ± 0.000 | 0.303 ± 0.195 | 0.0 ± 0.0 | 8.9 ± 7.7 | 0.482 ± 0.343 | – |

## Per-joint tracking, whole task, mean over seeds (rad)

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
