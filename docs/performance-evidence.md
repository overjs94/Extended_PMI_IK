# Performance evidence and current limitations

## Homepage status

The recorded-viewer deployment target is https://overjs94.github.io/Extended_PMI_IK/. Its first remote build and data validation passed, but GitHub refused initial Pages site creation; one-time [Pages settings](https://github.com/overjs94/Extended_PMI_IK/settings/pages) enablement is pending. Public availability is not yet verified. See [deployment instructions](deployment.md).

## Current native and CPU parallel measurements

The [264-run pilot](../benchmarks/README.md) compares 22 frozen cases and 12 methods with identical inputs, common final validation and a one-second loop budget. It retains every timeout, failure and unsupported row. Pink/Mink are executed native software baselines; SciPy SLSQP is executed under its own name. Other literature candidates have no invented timings.

For the three-seed nearby-Panda stratum, Pink/Mink/SLSQP each succeed 3/3 with median loop times of approximately **14.2 / 13.6 / 10.3 ms**. Reduced OSQP and maximal Skew Schur succeed 3/3 at **126.3 / 172.9 ms**. Compiled one-thread ADMM/DRS succeed 3/3 at **457.7 / 403.9 ms**. These observations do not show PMI outperforming those native baselines; broader success-rate differences remain visible in the report.

CPU parallel execution is implemented and measured, rather than inferred from the solver structure. The independent-query process batch finishes the same 24 frozen queries in **4473.7 / 2199.4 / 1136.9 ms** with 1/2/4 processes, all 24/24 successful: **3.93× four-process batch speedup**. This is a single measured batch per worker count, with startup excluded and separately reported, not a single-query latency reduction.

Within-query compilation reduces the ergoCub ADMM linearized solve from about **295 ms NumPy to 84 ms compiled one-thread**. Four compiled threads take about **121 ms**, slower than one compiled thread. The small body/factor tasks incur synchronization costs. Full results and scope are in [cpu-parallel.md](../benchmarks/reports/cpu-parallel.md); compilation speedup must not be relabelled as threading speedup or GPU performance.

## Earlier single-case formulation check

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

Comparing maximal OSQP QP against dense Skew-VI changes both operator and backend, so its timing difference cannot be attributed to the skew term alone. Compare the two Schur variants for an operator ablation. These earlier NumPy ADMM/DRS measurements establish solution agreement; use the separate current CPU measurements above for compilation, threading and process-throughput conclusions. No GPU speedup has been measured.

## Obstacle preview ablation

One deterministic sphere-proxy case, 7 timing repeats per variant after warmup; order alternated. Identical initial poses, final goal, distance inequalities and tolerances. Not 7 distinct scenarios. Geometric preview, not full MPC.

| Setting | Converged timing repeats | Median solve time (ms) | Outer iterations | Minimum sampled clearance (mm) |
|---|---:|---:|---:|---:|
| Distance inequalities only | 7/7 | 233.7 | 200 | 1.69 |
| Distance inequalities + preview | 7/7 | 190.6 | 161 | 8.51 |

In this case the preview reduces median measured loop time by **18.4%** and outer iterations by **19.5%**. Both variants converge to the same position tolerance. The unassisted inequality variant does **not** stall in this particular case, so this experiment is not evidence of curing all obstacle-boundary local minima.

The preview also gives greater sampled clearance in this case. These measurements cover one end-effector sphere and one deterministic obstacle, not full-body collision safety or full MPC. Seven timing repeats are not seven different scenes. Broader obstacle layouts, blocked goals, high-DoF embodiments and an actual original C++ baseline are still required before claiming a research-level performance advantage.
