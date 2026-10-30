# Torque studies (MuJoCo, RX200 → VX300s)

## S1 Estimation accuracy

| Estimator | RMSE under load, shoulder / elbow / wrist angle (N·m) | false torque in free motion, RMS (N·m) |
|---|---|---|
| Original code | 1786.10 / 2185.30 / 824.25 | 1339.74 / 1591.34 / 364.64 |
| Original, units fixed | 1.67 / 2.02 / 0.37 | 1.22 / 1.43 / 0.36 |
| Static residual (Eq. 2, fixed) | 0.25 / 0.30 / 0.06 | 0.29 / 0.34 / 0.08 |
| Momentum observer | 0.27 / 0.30 / 0.06 | 0.28 / 0.31 / 0.07 |

Free-motion false torque vs speed (shoulder / elbow, N·m RMS):

| speed | static residual | momentum observer |
|---|---|---|
| x1 | 0.25 / 0.27 | 0.27 / 0.28 |
| x2 | 0.48 / 0.48 | 0.44 / 0.44 |
| x3 | 1.08 / 0.87 | 0.75 / 0.73 |

### S1b Estimator inside the closed force loop

| Case | RMSE (rad) | max err (rad) | peak wall (N) | false shoulder torque, free (N·m) |
|---|---|---|---|---|
| Position-force, k_of=0.3, static | 0.024 | 0.15 | 3.4 | 0.47 |
| Position-force, k_of=0.3, momentum | 0.024 | 0.16 | 3.4 | 0.44 |
| Proposed, k_of=0.3, static | 0.012 | 0.12 | 4.2 | 0.50 |
| Proposed, k_of=0.3, momentum | 0.011 | 0.12 | 4.3 | 0.45 |
| Position-force, k_of=0.6, static | 0.238 | 0.98 | 110.5 | 11.30 |
| Position-force, k_of=0.6, momentum | 0.100 | 0.73 | 99.3 | 0.47 |
| Proposed, k_of=0.6, static | 0.110 | 0.40 | 73.4 | 5.87 |
| Proposed, k_of=0.6, momentum | 0.012 | 0.15 | 12.6 | 0.46 |

## S2 Torque mimicking

| Controller | force RMSE x,z (N) | corr x | corr z | gain x | gain z | master drift (rad) |
|---|---|---|---|---|---|---|
| Original code | 3.41 | 0.53 | 0.25 | -0.12 | 1.13 | 0.221 |
| Position-force (2ch) | 0.34 | 0.96 | 0.98 | 0.94 | 1.40 | 0.151 |
| Proposed | 0.43 | 0.94 | 0.98 | 1.02 | 1.60 | 0.189 |
| Proposed, k_of=0.6 | 0.59 | 0.96 | 0.97 | 0.91 | 1.30 | 0.402 |

gain = least-squares ratio felt / (k_of·F_env); 1.0 = exact scaled reproduction.

### S2b What sets the rendered gain

| Case | gain x | gain z |
|---|---|---|
| Proposed | 1.02 | 1.60 |
| Proposed, k_om=0 | 0.70 | 1.11 |
| Proposed, identity map | 1.33 | 1.89 |
| Position-force, identity map | 0.94 | 1.40 |
| Position-force, Jacobian map | 0.67 | 1.10 |

## S3 Object weight

| mass (kg) | m·g (N) | Original code | Position-force (2ch) | Proposed |
|---|---|---|---|---|
| 0.00 | 0.00 | 4.08 | 0.21 | 0.22 |
| 0.10 | 0.98 | 4.08 | 0.53 | 0.56 |
| 0.20 | 1.96 | 4.08 | 0.93 | 1.01 |
| 0.30 | 2.94 | 4.08 | 1.31 | 1.46 |
| 0.50 | 4.91 | 4.08 | 2.05 | 2.33 |
| 0.75 | 7.36 | 4.08 | 3.00 | 3.43 |

## S4 Stiffness

| Controller | k (N/m) | peak wall (N) | felt/true | RMSE (rad) |
|---|---|---|---|---|
| Position-force (2ch) | 500 | 3.0 | 0.31 | 0.024 |
| Position-force (2ch) | 1000 | 3.2 | 0.32 | 0.024 |
| Position-force (2ch) | 2000 | 3.4 | 0.32 | 0.024 |
| Position-force (2ch) | 5000 | 3.5 | 0.33 | 0.024 |
| Position-force (2ch) | 10000 | 3.6 | 0.33 | 0.024 |
| Position-position | 500 | 5.7 | -0.01 | 0.025 |
| Position-position | 1000 | 7.1 | 0.00 | 0.026 |
| Position-position | 2000 | 8.2 | 0.01 | 0.026 |
| Position-position | 5000 | 9.0 | 0.01 | 0.027 |
| Position-position | 10000 | 9.4 | 0.01 | 0.027 |
| Proposed | 500 | 3.5 | 0.30 | 0.011 |
| Proposed | 1000 | 4.0 | 0.31 | 0.011 |
| Proposed | 2000 | 4.3 | 0.31 | 0.011 |
| Proposed | 5000 | 4.7 | 0.31 | 0.012 |
| Proposed | 10000 | 4.9 | 0.31 | 0.012 |

## S5 Stability (max tracking error, rad)

**no tank**

| k_of \ delay | 0 ms | 20 ms | 40 ms | 60 ms |
|---|---|---|---|---|
| 0.1 | 0.10 | 0.20 | 0.27 | 0.27 |
| 0.3 | 0.12 | 0.21 | 0.27 | 0.27 |
| 0.6 | 0.15 | 0.73 | 0.53 | 1.53 |
| 1.0 | 0.87 | 0.82 | 2.24 | 2.62 |
| 1.5 | 1.99 | 0.96 | 2.17 | 2.37 |

**tank**

| k_of \ delay | 0 ms | 20 ms | 40 ms | 60 ms |
|---|---|---|---|---|
| 0.1 | 0.10 | 0.20 | 0.27 | 0.27 |
| 0.3 | 0.12 | 0.21 | 0.27 | 0.27 |
| 0.6 | 0.15 | 0.73 | 0.53 | 1.38 |
| 1.0 | 0.50 | 0.88 | 1.63 | 1.94 |
| 1.5 | 0.72 | 0.94 | 1.45 | 2.25 |

