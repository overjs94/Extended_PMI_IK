# CPU parallelism: measured scopes

There are two different questions: speeding one coupled solve with worker threads, and increasing throughput for independent IK queries. Neither is automatically improved by moving to GPU or by naming an algorithm ADMM/DRS.

The NumPy implementation is a serial vectorized reference. The new Numba kernels use real GIL-free CPU threads for body updates, factor projection and CSR consensus. Inverse incidence lists prevent concurrent scatter races. No fastmath or looser stopping tolerance is used.

## Independent-query process parallelism

Identical 24 frozen Panda nearby queries, one BLAS thread per process, same solver and tolerances, warmed workers. Process startup/import costs are reported separately. This is batch throughput, not lower single-query latency.

| Processes | Batch wall time (ms) | Successful queries | Batch speedup |
|---:|---:|---:|---:|
| 1 | 4473.7 | 24/24 | 1.00× |
| 2 | 2199.4 | 24/24 | 2.03× |
| 4 | 1136.9 | 24/24 | 3.93× |

Actual job process IDs are retained in `cpu-batch.json`. The selected machine has a four-core CPU quota and no CUDA device. A single batch at each worker count is a pilot, not a statistical performance guarantee.

## Within-query compiled kernels

Each row solves one frozen linearized midpoint problem from a real URDF. It includes inverse/incidence setup and the inner solve; compilation/warmup is excluded. Five cold-iterate timing repeats per variant, randomized order. This is not total IK latency.

| Scene | Method | Kernel | Threads | Median (ms) | Converged |
|---|---|---|---:|---:|---:|
| panda | admm | numpy | 1 | 16.943 | 5/5 |
| panda | admm | numba | 1 | 3.948 | 5/5 |
| panda | admm | numba | 2 | 6.101 | 5/5 |
| panda | admm | numba | 4 | 10.861 | 5/5 |
| panda | drs | numpy | 1 | 13.459 | 5/5 |
| panda | drs | numba | 1 | 3.042 | 5/5 |
| panda | drs | numba | 2 | 6.782 | 5/5 |
| panda | drs | numba | 4 | 13.728 | 5/5 |
| dual | admm | numpy | 1 | 79.562 | 5/5 |
| dual | admm | numba | 1 | 18.948 | 5/5 |
| dual | admm | numba | 2 | 30.618 | 5/5 |
| dual | admm | numba | 4 | 40.158 | 5/5 |
| dual | drs | numpy | 1 | 52.410 | 5/5 |
| dual | drs | numba | 1 | 12.706 | 5/5 |
| dual | drs | numba | 2 | 27.132 | 5/5 |
| dual | drs | numba | 4 | 47.784 | 5/5 |
| quad | admm | numpy | 1 | 100.169 | 5/5 |
| quad | admm | numba | 1 | 28.550 | 5/5 |
| quad | admm | numba | 2 | 44.954 | 5/5 |
| quad | admm | numba | 4 | 53.220 | 5/5 |
| quad | drs | numpy | 1 | 69.213 | 5/5 |
| quad | drs | numba | 1 | 18.355 | 5/5 |
| quad | drs | numba | 2 | 32.559 | 5/5 |
| quad | drs | numba | 4 | 55.096 | 5/5 |
| unitree-g1 | admm | numpy | 1 | 84.537 | 5/5 |
| unitree-g1 | admm | numba | 1 | 23.970 | 5/5 |
| unitree-g1 | admm | numba | 2 | 36.173 | 5/5 |
| unitree-g1 | admm | numba | 4 | 41.684 | 5/5 |
| unitree-g1 | drs | numpy | 1 | 68.870 | 5/5 |
| unitree-g1 | drs | numba | 1 | 17.481 | 5/5 |
| unitree-g1 | drs | numba | 2 | 33.763 | 5/5 |
| unitree-g1 | drs | numba | 4 | 42.926 | 5/5 |
| ergocub | admm | numpy | 1 | 295.175 | 5/5 |
| ergocub | admm | numba | 1 | 83.616 | 5/5 |
| ergocub | admm | numba | 2 | 106.487 | 5/5 |
| ergocub | admm | numba | 4 | 121.099 | 5/5 |
| ergocub | drs | numpy | 1 | 244.667 | 5/5 |
| ergocub | drs | numba | 1 | 62.815 | 5/5 |
| ergocub | drs | numba | 2 | 86.282 | 5/5 |
| ergocub | drs | numba | 4 | 111.465 | 5/5 |

All linearized reference comparisons were available. Maximum infinity-norm difference from the centralized Schur solution was 3.65e-06. Results do not establish global convergence or physical trajectory passivity.

Compilation substantially reduces the Python-loop cost, while extra threads do not necessarily help these small 6×6 body/factor workloads. On the ergoCub linearized ADMM case, the measured NumPy / compiled-one-thread / compiled-four-thread times were approximately 295 / 84 / 121 ms. The four-thread variant is slower than the compiled one-thread variant, despite being faster than the NumPy reference.

End-to-end matched-input IK results remain in `pilot.json` and `pilot-by-embodiment.csv`. Faster inner kernels do not imply superiority over Pink/Mink/SLSQP or the centralized Schur method. The current nearby-Panda three-seed pilot still favors those native baselines.

Next optimization targets: group joint rows into block projections; improve conditioning/penalty selection; reduce inner iterations and synchronization; retain body-local inverses; compare sparse/iterative Schur solves; only then investigate GPU crossover. Virtual IK solver iterations must not be described as a physical robot control trajectory.
