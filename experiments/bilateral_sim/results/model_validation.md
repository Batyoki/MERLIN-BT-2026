# Slave-model validation (MuJoCo, 3 operator seeds, unilateral mode)

RMS N-step prediction error of the slave joint positions (rad), pooled over 5 joints.

| Predictor | h=1 (20 ms) | h=2 (40 ms) | h=5 (100 ms) | h=10 (200 ms) |
|---|---|---|---|---|
| hold | 0.0120 | 0.0238 | 0.0576 | 0.1089 |
| const-vel | 0.0028 | 0.0078 | 0.0286 | 0.0719 |
| loop (d_hat = 0) | 0.0264 | 0.0264 | 0.0265 | 0.0266 |
| loop + d_hat (MPC model) | 0.0051 | 0.0061 | 0.0075 | 0.0068 |
