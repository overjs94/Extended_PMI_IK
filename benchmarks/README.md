# Literature-informed comparison

The source-reviewed candidate registry is [literature.json](literature.json). NVIDIA cuRoboV2-Fast (2026 preprint), MIT Drake Global IK, IKFlow, TRAC-IK and TrajOpt are distinguished from maintained software baselines Pink/Mink and the MotionBenchMaker/Robometrics datasets. Official source revisions and reference protocols are recorded. This is not an exhaustive 2026 SOTA survey; not all original paper full texts have been read.

## Measured comparisons

- [264-run CPU pilot](reports/pilot.md), [per-embodiment results](reports/pilot-by-embodiment.csv), [raw measurements](reports/pilot.json).
- [CPU thread/process comparison](reports/cpu-parallel.md).
- [cuRoboV2 upstream GPU protocol](protocols/curobo-v2-ik.json): reference measurements are quoted from the official benchmark documentation, not executed here.

All 22 cases have frozen URDF hashes, initial configurations, targets, hard relative constraints and obstacle proxies. Recipes cover random reachable targets, joint limits, near-singular starts, a certified distant-target control, dual/four-arm closed loops, anchored-feet G1/ergoCub, and multi-link sphere-versus-box clutter. These are literature-inspired adaptations, **not original benchmark dataset reproductions**. The original MotionBenchMaker Panda download was blocked at Dropbox by the environment egress proxy.

Actual Pink 4.4.0, Mink 1.4.0 and SciPy SLSQP execute alongside PMI variants. Native model conversions are checked against common URDF FK at initial and perturbed configurations; collision rows are checked by finite differences. No custom solver is relabelled as a paper method. Full mesh/self collision, torque dynamics, original learned weights and GPU methods are not emulated.

## Repeat

```bash
bash scripts/setup_benchmarks.sh
OPENBLAS_NUM_THREADS=1 NUMBA_NUM_THREADS=4 .benchmark-venv/bin/python -m extended_pmi_ik.benchmarks.runner generate
OPENBLAS_NUM_THREADS=1 NUMBA_NUM_THREADS=4 .benchmark-venv/bin/python -m extended_pmi_ik.benchmarks.runner run \
  --budget 1 --iterations 600 --methods reduced-osqp maximal-qp-schur maximal-schur maximal-admm maximal-drs \
  maximal-admm-numba-1 maximal-admm-numba-4 maximal-drs-numba-1 maximal-drs-numba-4 pink-osqp mink-osqp scipy-slsqp
OPENBLAS_NUM_THREADS=1 NUMBA_NUM_THREADS=4 .benchmark-venv/bin/python -m extended_pmi_ik.benchmarks.parallel kernels \
  --repeats 5 --output benchmarks/reports/cpu-kernels.json
OPENBLAS_NUM_THREADS=1 NUMBA_NUM_THREADS=4 .benchmark-venv/bin/python -m extended_pmi_ik.benchmarks.parallel batch \
  --queries 24 --output benchmarks/reports/cpu-batch.json
```

Run timing experiments serially with no other build/render benchmark competing for CPU. Source hashes, package versions, CPU quota, setup time and deadline overshoot are retained. One untimed warmup step is discarded and every measured run restarts from the frozen q. `--resume` rejects changed code/versions/settings/cases.

Common final validation uses URDF FK, position norm ≤0.1 mm, geodesic rotation ≤1 mrad, hard closure translation/rotation ≤1e-5, limits and declared obstacle clearances. Pose solves exceeding the one-second loop budget are timeouts even if a solver reports convergence. Limits are checked at iteration boundaries, so inner-call overshoot remains visible. No frame serialization is timed in this pilot. Negative unreachable controls do not enter feasible-case success rates. Unsupported capabilities remain explicit rather than dropping constraints.

Do not compare successful-only medians without success counts. `closed-loop` aggregates different embodiments in the overview: use the per-embodiment CSV or the public viewer's robot/recipe selector. Three seeds per small stratum and one ergoCub case do not establish statistically general SOTA superiority.

The CUDA methods remain unrun because no CUDA device is available. TRAC-IK/Drake/IKFlow/TrajOpt adapters are not qualified; they have no invented measured rows. The upstream cuRoboV2 reference uses 100-query GPU batches, 5 mm / 0.05 rad tolerances, self collision and a different G1 model with 35 variables; those values are not direct speed ratios against this strict CPU pilot.

## Timed trajectories

The separate [trajectory pilot](../docs/trajectory-input.md) executes LASDRA, dual/four-arm manipulation and floating G1 contact-sequence tracking. [Per-knot results](reports/tracking.json) and [summary CSV](reports/tracking.csv) include failures, phase transitions and sample-interval misses. The four inputs are authored examples; a particular external CITO export has not been supplied. Python API/CLI accept actual planner contact/EE JSON without labelling authored data as CITO output. These measurements have their own two-second loop budget and must not be mixed into the one-second 264-run pose pilot. Exact measurement source bytes are preserved in [source-snapshots](source-snapshots/README.md).
