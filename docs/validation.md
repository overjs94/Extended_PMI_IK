# Validation — 2026-11-14

Current cloud machine: Python 3.12, CPU reference, `OPENBLAS_NUM_THREADS=1`. Results below are functional checks, not published performance claims.

- Core suite: **42 passed** (including optional downloaded model tests); one upstream Starlette/httpx deprecation warning.
- Web production build: passed. Vite reports one >500 kB Three.js bundle warning.
- Headless Chromium: collision/realistic mesh modes, playback, synthetic humanoid solve, obstacle detour, actual G1 visualization, custom URDF import/solve, CSV export and mobile overflow checks passed.
- Panda: all **10** solver/backend variants converged on seed 7. Actual timings and residuals: `panda-method-validation.json` / `.csv`.
- Obstacle detour: converged, 161 iterations, approximately **8.51 mm minimum sampled sphere clearance**. This is not full-body or continuous collision checking.
- All **15** external models: pinned URDF and local asset checksums verified; details in `model-assets.json`.
- ROS bridge, GPU/distributed execution, full-mesh collision checking and full MPC: **not run / not implemented as readiness claims**.

## External-model IK check

One seed (7), maximal block Schur reference, default 600-iteration budget. `iteration_limit` means the requested tolerance was not reached; it is not counted as convergence. This initial common-gain sweep is retained to guide robot-specific conditioning and solver tuning.

| Model | Configuration DoF | Outcome | Position error (mm) | Rotation error (rad) | Solve time (ms) |
|---|---:|---|---:|---:|---:|
| Unitree G1 (29 DoF) | 29 | converged | 0.099 | 8.2e-06 | 2538.9 |
| Unitree H1 | 19 | iteration_limit | 0.155 | 5.06e-05 | 2577.7 |
| Booster T1 | 23 | iteration_limit | 0.581 | 1.4e-05 | 3191.0 |
| Berkeley Humanoid | 12 | converged | 0.100 | 4.4e-06 | 400.2 |
| Fourier GR1 T1 | 32 | iteration_limit | 0.233 | 2.27e-08 | 5413.9 |
| Fourier N1 | 23 | converged | 0.099 | 1.68e-05 | 1639.7 |
| ToddlerBot 2XC | 36 | iteration_limit | 0.958 | 1.82e-05 | 9915.6 |
| KUKA iiwa 7 | 7 | iteration_limit | 0.297 | 0.000192 | 805.4 |
| KUKA iiwa 14 | 7 | iteration_limit | 0.276 | 0.000169 | 820.3 |
| Universal Robots UR5 | 6 | iteration_limit | 0.181 | 2.04e-05 | 736.4 |
| Universal Robots UR10 | 6 | converged | 0.100 | 9.37e-06 | 658.8 |
| KUKA iiwa 7 + Allegro | 23 | iteration_limit | 4.087 | 0.00207 | 3461.2 |
| Allegro dexterous hand | 16 | iteration_limit | 0.325 | 1.02e-05 | 2478.9 |
| ANYmal C | 12 | converged | 0.096 | 3.5e-05 | 204.9 |
| ergoCub SN002 | 57 | iteration_limit | 1.654 | 5.98e-07 | 68562.5 |

The real humanoid examples use anchored-feet kinematics and arm pose targets; the synthetic carry scenario explicitly includes an object and rigid grasps. The Berkeley entry is a biped baseline, not one of the seven whole-body humanoid families.

Fresh-task snapshot restoration and remote GitHub Actions execution have not been verified here. Saved cloud configuration is distinct from publication.
