# Contact/EE trajectory input and measured tracking

The [public viewer](https://overjs94.github.io/Extended_PMI_IK/#tracking) replays computed trajectories. The local Python API and CLI perform new per-knot IK. No new optimization runs in the GitHub Pages viewer.

The four current examples are LASDRA 3D lemniscate tracking, two- and four-Panda rigid-object tracking, and Unitree G1 with a floating base and stance/swing/contact transitions. Inputs are in [`benchmarks/trajectory-inputs`](../benchmarks/trajectory-inputs), per-knot results in [`benchmarks/reports/tracking.json`](../benchmarks/reports/tracking.json), and the comparison in [`benchmarks/reports/tracking.csv`](../benchmarks/reports/tracking.csv).

## LASDRA model

[`models/lasdra.urdf`](../models/lasdra.urdf) transcribes the supplied `LASDRA_kinematics.cpp` and `System_Param.h`: ten one-metre links, a Z/Y/X base and nine Y/X universal joints, 21 coordinates, a one-metre terminal offset, and ten half-metre link-centre frames. Regenerate with `python scripts/build_lasdra_urdf.py`. The independent test compiles the **unmodified original C++**, comparing the EE and all ten link centres at zero and twelve random configurations. The tolerance is 2e-12.

No LASDRA vendor mesh or mechanical joint bounds were present in this upload. Its displayed rods/joint spheres are labelled schematic; continuous coordinates preserve the original FK without inventing limits. This is a source-derived model, not a CAD reconstruction. The small lemniscate is an adapted input around a bent start, without the original paper's 12-obstacle experiment or original gains.

## Planner output contract

CITO generates a contact sequence and EE/body pose samples upstream; this package reconstructs joint configurations. The current G1 input is explicitly an **authored short-step example**, not an executed CITO result. A particular external CITO repository/export has not yet been supplied. No synthetic schedule is labelled as a paper planner result.

The JSON contract is `version: 1`, a provenance object and 2–300 `samples`. Every sample has strictly increasing `time_s`, `targets` mapping URDF link names to 4×4 world SE(3) matrices, optional `contacts` with the same mapping, and an optional `phase` string. Positions are metres, rotations are proper rotation matrices, and the last row is `[0, 0, 0, 1]`. All transforms use this URDF's world frame; exporters must convert their link names, units and frames.

An active contact adds a hard world-to-link pose equality. Consecutive active knots must preserve the contact pose. Lift-off removes that equality, and touchdown adds it at the supplied landing pose. Changing an active contact pose without release is rejected. Existing object-to-tool grasp equalities persist throughout manipulation. Contact forces, friction cones, centroidal dynamics and balance are not inferred from the schedule. Results are kinematic samples, not a dynamically feasible gait certificate.

Set `provenance.kind` to `external_cito_output` for an actual exported CITO result, and provide its source/run identifier in `provenance.source`. A caller-supplied provenance declaration is retained; it is not independently authenticated. `authored_example` and `external_trajectory` distinguish other inputs. Use the downloadable example as a complete format template.

```bash
# Compute a built-in trajectory
OPENBLAS_NUM_THREADS=1 .venv/bin/extended-pmi track \
  --trajectory dual-carry --method maximal-schur --output results/dual-track.json

# Consume an external planner output for the floating G1 model
OPENBLAS_NUM_THREADS=1 .venv/bin/extended-pmi track \
  --trajectory g1-contact-walk --plan planner-contact-ee.json \
  --method maximal-admm --kernel numba --workers 4 --output results/gait.json
```

`GET /api/trajectories` lists models/methods, `GET /api/trajectories/{id}` returns the initial model and plan, and `POST /api/trajectory/solve` accepts `trajectory`, `method`, optional `plan`, and solver settings. The live UI imports planner JSON; Pages disables new-plan import and labels recordings. Native Pink/Mink require `bash scripts/setup_benchmarks.sh`.

## Timing and limitations

Each knot executes real IK and starts from the preceding joint configuration. Virtual velocity, inner duals and factorization state currently restart per knot. The shown configuration is common-URDF FK of the returned coordinates, independently checked against requested poses, hard contacts and limits. Failures remain in the sequence; they are not dropped or interpolated into successes. All current trajectory inputs are obstacle-free.

The default two-second loop budget is distinct from the input sample interval. Per-knot wall time includes model/constraint setup and endpoint validation; the table marks when it exceeds the requested interval. Total time also includes frame serialization. Playback holds actual sampled configurations and advances according to input times; it does not manufacture continuous joint motion. There is no feed-forward trajectory controller, continuous collision check or hard real-time claim.

One measured run per method/input gives this small pilot:

| Input | Knots | PMI Skew Schur | Native Pink | Native Mink |
|---|---:|---:|---:|---:|
| LASDRA lemniscate | 17 | 9/17 | 17/17 | 17/17 |
| Dual-arm carry | 17 | 17/17 | 17/17 | 17/17 |
| Four-arm carry | 17 | 1/17 | 17/17 | 17/17 |
| G1 contact walking | 9 | 2/9 | 9/9 | 9/9 |

Success requires pose/closure/limit validation and the solver loop deadline. PMI remains weaker in this common-default trajectory pilot; no improvement or SOTA superiority is asserted. Pink G1 RMS position error is about 0.060 mm, maximum stance error about 0.0097 mm and p95 knot wall time about 153 ms, all on the recorded four-core-quota CPU. This is one short authored gait, not a general locomotion result.
