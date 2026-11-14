# Native-baseline pilot benchmark

Literature-informed synthetic/adapted cases, not a reproduction of published paper performance. No overall SOTA ranking. Native software baselines are distinct from peer-reviewed algorithms.

Settings: {'dt': 0.04, 'iterations': 600, 'position_tolerance': 0.0001, 'rotation_tolerance': 0.001, 'constraint_tolerance': 1e-05, 'contraction': 0.25, 'alpha': 1.0, 'stiffness': 120.0, 'damping': 4.0, 'mass': 0.1, 'preview': False, 'wall_time_limit_s': 1.0, 'record_frames': False, 'strict_closure_check': True, 'splitting_backend': 'numpy', 'cpu_workers': 1}

| Recipe | Method | Success / eligible feasible | Median success (ms) | p95 success (ms) | Timeouts | Unsupported |
|---|---|---:|---:|---:|---:|---:|
| closed-loop | maximal-admm | 0 / 6 | — | — | 6 | 0 |
| closed-loop | maximal-admm-numba-1 | 0 / 6 | — | — | 6 | 0 |
| closed-loop | maximal-admm-numba-4 | 0 / 6 | — | — | 6 | 0 |
| closed-loop | maximal-drs | 0 / 6 | — | — | 6 | 0 |
| closed-loop | maximal-drs-numba-1 | 0 / 6 | — | — | 6 | 0 |
| closed-loop | maximal-drs-numba-4 | 0 / 6 | — | — | 6 | 0 |
| closed-loop | maximal-qp-schur | 2 / 6 | 686.8 | 713.3 | 4 | 0 |
| closed-loop | maximal-schur | 2 / 6 | 773.9 | 804.7 | 4 | 0 |
| closed-loop | mink-osqp | 5 / 6 | 51.3 | 84.6 | 0 | 0 |
| closed-loop | pink-osqp | 5 / 6 | 51.0 | 89.9 | 0 | 0 |
| closed-loop | reduced-osqp | 4 / 6 | 562.5 | 644.8 | 2 | 0 |
| closed-loop | scipy-slsqp | 6 / 6 | 70.2 | 264.5 | 0 | 0 |
| clutter-boxes | maximal-admm | 0 / 3 | — | — | 3 | 0 |
| clutter-boxes | maximal-admm-numba-1 | 0 / 3 | — | — | 2 | 0 |
| clutter-boxes | maximal-admm-numba-4 | 0 / 3 | — | — | 2 | 0 |
| clutter-boxes | maximal-drs | 0 / 3 | — | — | 3 | 0 |
| clutter-boxes | maximal-drs-numba-1 | 0 / 3 | — | — | 2 | 0 |
| clutter-boxes | maximal-drs-numba-4 | 0 / 3 | — | — | 2 | 0 |
| clutter-boxes | maximal-qp-schur | 3 / 3 | 288.1 | 346.5 | 0 | 0 |
| clutter-boxes | maximal-schur | 3 / 3 | 307.9 | 398.2 | 0 | 0 |
| clutter-boxes | mink-osqp | 2 / 3 | 42.5 | 44.0 | 1 | 0 |
| clutter-boxes | pink-osqp | 2 / 3 | 36.8 | 38.2 | 1 | 0 |
| clutter-boxes | reduced-osqp | 0 / 0 | — | — | 0 | 3 |
| clutter-boxes | scipy-slsqp | 3 / 3 | 16.1 | 16.3 | 0 | 0 |
| joint-limits | maximal-admm | 0 / 3 | — | — | 3 | 0 |
| joint-limits | maximal-admm-numba-1 | 0 / 3 | — | — | 3 | 0 |
| joint-limits | maximal-admm-numba-4 | 0 / 3 | — | — | 3 | 0 |
| joint-limits | maximal-drs | 0 / 3 | — | — | 3 | 0 |
| joint-limits | maximal-drs-numba-1 | 0 / 3 | — | — | 3 | 0 |
| joint-limits | maximal-drs-numba-4 | 0 / 3 | — | — | 3 | 0 |
| joint-limits | maximal-qp-schur | 1 / 3 | 363.9 | 363.9 | 0 | 0 |
| joint-limits | maximal-schur | 1 / 3 | 408.6 | 408.6 | 0 | 0 |
| joint-limits | mink-osqp | 3 / 3 | 24.1 | 24.5 | 0 | 0 |
| joint-limits | pink-osqp | 3 / 3 | 26.0 | 26.5 | 0 | 0 |
| joint-limits | reduced-osqp | 1 / 3 | 129.7 | 129.7 | 0 | 0 |
| joint-limits | scipy-slsqp | 3 / 3 | 15.7 | 22.9 | 0 | 0 |
| near-singular | maximal-admm | 0 / 3 | — | — | 3 | 0 |
| near-singular | maximal-admm-numba-1 | 0 / 3 | — | — | 3 | 0 |
| near-singular | maximal-admm-numba-4 | 0 / 3 | — | — | 3 | 0 |
| near-singular | maximal-drs | 0 / 3 | — | — | 3 | 0 |
| near-singular | maximal-drs-numba-1 | 0 / 3 | — | — | 3 | 0 |
| near-singular | maximal-drs-numba-4 | 0 / 3 | — | — | 3 | 0 |
| near-singular | maximal-qp-schur | 2 / 3 | 606.3 | 624.6 | 0 | 0 |
| near-singular | maximal-schur | 2 / 3 | 697.7 | 714.5 | 0 | 0 |
| near-singular | mink-osqp | 3 / 3 | 12.1 | 12.7 | 0 | 0 |
| near-singular | pink-osqp | 3 / 3 | 12.4 | 12.7 | 0 | 0 |
| near-singular | reduced-osqp | 2 / 3 | 654.2 | 663.5 | 0 | 0 |
| near-singular | scipy-slsqp | 3 / 3 | 6.4 | 8.4 | 0 | 0 |
| nearby | maximal-admm | 0 / 3 | — | — | 3 | 0 |
| nearby | maximal-admm-numba-1 | 3 / 3 | 457.7 | 468.8 | 0 | 0 |
| nearby | maximal-admm-numba-4 | 1 / 3 | 898.2 | 898.2 | 2 | 0 |
| nearby | maximal-drs | 1 / 3 | 956.8 | 956.8 | 2 | 0 |
| nearby | maximal-drs-numba-1 | 3 / 3 | 403.9 | 408.8 | 0 | 0 |
| nearby | maximal-drs-numba-4 | 1 / 3 | 949.7 | 949.7 | 2 | 0 |
| nearby | maximal-qp-schur | 3 / 3 | 149.4 | 164.9 | 0 | 0 |
| nearby | maximal-schur | 3 / 3 | 172.9 | 226.3 | 0 | 0 |
| nearby | mink-osqp | 3 / 3 | 13.6 | 15.9 | 0 | 0 |
| nearby | pink-osqp | 3 / 3 | 14.2 | 14.6 | 0 | 0 |
| nearby | reduced-osqp | 3 / 3 | 126.3 | 145.5 | 0 | 0 |
| nearby | scipy-slsqp | 3 / 3 | 10.3 | 11.8 | 0 | 0 |
| reachable-random | maximal-admm | 0 / 3 | — | — | 3 | 0 |
| reachable-random | maximal-admm-numba-1 | 0 / 3 | — | — | 3 | 0 |
| reachable-random | maximal-admm-numba-4 | 0 / 3 | — | — | 3 | 0 |
| reachable-random | maximal-drs | 0 / 3 | — | — | 3 | 0 |
| reachable-random | maximal-drs-numba-1 | 0 / 3 | — | — | 3 | 0 |
| reachable-random | maximal-drs-numba-4 | 0 / 3 | — | — | 3 | 0 |
| reachable-random | maximal-qp-schur | 1 / 3 | 434.7 | 434.7 | 1 | 0 |
| reachable-random | maximal-schur | 1 / 3 | 519.9 | 519.9 | 1 | 0 |
| reachable-random | mink-osqp | 2 / 3 | 22.6 | 22.9 | 0 | 0 |
| reachable-random | pink-osqp | 2 / 3 | 23.5 | 24.1 | 0 | 0 |
| reachable-random | reduced-osqp | 1 / 3 | 349.2 | 349.2 | 0 | 0 |
| reachable-random | scipy-slsqp | 2 / 3 | 15.8 | 15.9 | 0 | 0 |
| unreachable | maximal-admm | 0 / 0 | — | — | 1 | 0 |
| unreachable | maximal-admm-numba-1 | 0 / 0 | — | — | 1 | 0 |
| unreachable | maximal-admm-numba-4 | 0 / 0 | — | — | 1 | 0 |
| unreachable | maximal-drs | 0 / 0 | — | — | 1 | 0 |
| unreachable | maximal-drs-numba-1 | 0 / 0 | — | — | 1 | 0 |
| unreachable | maximal-drs-numba-4 | 0 / 0 | — | — | 1 | 0 |
| unreachable | maximal-qp-schur | 0 / 0 | — | — | 0 | 0 |
| unreachable | maximal-schur | 0 / 0 | — | — | 0 | 0 |
| unreachable | mink-osqp | 0 / 0 | — | — | 0 | 0 |
| unreachable | pink-osqp | 0 / 0 | — | — | 0 | 0 |
| unreachable | reduced-osqp | 0 / 0 | — | — | 0 | 0 |
| unreachable | scipy-slsqp | 0 / 0 | — | — | 0 | 0 |

Timing is end-to-end iteration-loop time including common FK/constraint diagnostics; model conversion and setup are recorded separately. No frame serialization is timed. Successful runs exceeding the budget are timeouts. Inner calls can overshoot the boundary; overshoot is recorded.

Do not rank methods by median-success latency without their success counts. Unsupported cases are explicitly marked, never relaxed or counted as successes. Negative unreachable controls are separate from feasible-case success rates. No sampled solver path is claimed as a continuous-time collision-safe motion plan.
