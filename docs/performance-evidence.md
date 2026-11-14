# Performance evidence and current limitations

## Homepage status

The source repository has been published, but a public website service has not been deployed. See [deployment instructions](deployment.md). A local working UI and saved cloud setup are not public hosting.

## Does the added solver formulation outperform the baseline?

**No general performance improvement has been demonstrated.** The present ADMM/DRS reference runs on CPU, and it is slower on the saved Panda problem. Skew retention does not reduce the 126 outer iterations in that case. The original ROS/C++ executable was not rebuilt and benchmarked here; the reduced variants below are the new URDF-driven Python reference, not an original-paper reproduction.

Panda seed 7, identical initial configuration/target/tolerances; the saved single-run CPU loop timings exclude model/constraint generation. This is exploratory evidence, not multi-scenario statistical superiority.

| Method | Wall time (ms) | Outer iterations | Position error (mm) |
|---|---:|---:|---:|
| reduced-osqp | 141.2 | 99 | 0.096 |
| reduced-qpoases | 118.8 | 99 | 0.096 |
| maximal-qp-schur | 150.7 | 126 | 0.097 |
| maximal-schur | 193.8 | 126 | 0.097 |
| maximal-admm | 1816.2 | 126 | 0.097 |
| maximal-drs | 1402.2 | 126 | 0.097 |

Comparing maximal OSQP QP against dense Skew-VI changes both operator and backend, so its timing difference cannot be attributed to the skew term alone. Compare the two Schur variants for an operator ablation. ADMM/DRS presently establish solution agreement and a parallelizable structure; they do not establish CPU/GPU speedup.

## Obstacle preview ablation

One deterministic sphere-proxy case, 7 timing repeats per variant after warmup; order alternated. Identical initial poses, final goal, distance inequalities and tolerances. Not 7 distinct scenarios. Geometric preview, not full MPC.

| Setting | Converged timing repeats | Median solve time (ms) | Outer iterations | Minimum sampled clearance (mm) |
|---|---:|---:|---:|---:|
| Distance inequalities only | 7/7 | 233.7 | 200 | 1.69 |
| Distance inequalities + preview | 7/7 | 190.6 | 161 | 8.51 |

In this case the preview reduces median measured loop time by **18.4%** and outer iterations by **19.5%**. Both variants converge to the same position tolerance. The unassisted inequality variant does **not** stall in this particular case, so this experiment is not evidence of curing all obstacle-boundary local minima.

The preview also gives greater sampled clearance in this case. These measurements cover one end-effector sphere and one deterministic obstacle, not full-body collision safety or full MPC. Seven timing repeats are not seven different scenes. Broader obstacle layouts, blocked goals, high-DoF embodiments and an actual original C++ baseline are still required before claiming a research-level performance advantage.
