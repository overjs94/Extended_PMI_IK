# Public website deployment

The public recorded-viewer deployment target is **https://overjs94.github.io/Extended_PMI_IK/**. The Pages workflow verifies the actual public response after publishing; source pushes alone are not evidence of a successful deployment.

The [first Pages run](https://github.com/overjs94/Extended_PMI_IK/actions/runs/43567128265) passed the asset fetch, web build and 21-scene/63-record validation, then failed site creation with `Resource not accessible by integration`. The new repository did not yet have Pages enabled. Open [repository Pages settings](https://github.com/overjs94/Extended_PMI_IK/settings/pages), choose **Build and deployment → Source → GitHub Actions**, and rerun the workflow. The standard `GITHUB_TOKEN` cannot perform this initial site creation: [configure-pages' enablement input](https://github.com/actions/configure-pages/blob/v5/action.yml) explicitly requires a different token with higher permissions. No credential extraction or extra hosting account is needed for the manual settings route. The owner has now selected GitHub Actions in Pages settings; the next push retries publication. Public reachability remains unverified until deployment and its post-publish check succeed.

## Recorded viewer: GitHub Pages

This follows the already published `overjs94/Contact_contraction` workflow: recorded solver data and URDF visuals are static files, not a Python service. The public page labels recorded execution, fixes parameters to their measured values, and never pretends a saved response is a new solve. Realistic meshes and collision geometry use separate URDF origins. Native baseline energy is shown as unreported rather than invented.

`web/recordings` contains 21 scenario descriptions and 63 measured runs. Some have timeouts/failures, which remain visible. Up to 72 actual solver states are retained per run with their original iteration indices; there is no pose interpolation or physical-dynamics claim. The five-second recording budget is separate from the frame-disabled one-second research pilot.

To regenerate records, use the optional benchmark environment and `OPENBLAS_NUM_THREADS=1 .benchmark-venv/bin/python scripts/record_viewer.py --overwrite`. Publication does **not** rerun solvers or change measured values. It fetches pinned model assets, builds `VITE_STATIC_REPLAY=1 PUBLIC_BASE_PATH=/Extended_PMI_IK/`, copies measured records, validates all files, and deploys through GitHub Pages. Both encoded and already HTTP-decoded gzip datasets are supported.

Internal static browser checks passed with the exact subpath and zero `/api/` requests. A failed Pages setup/deployment must be reported as failed; do not assert the target URL is live before the post-deployment check succeeds.

## Optional live optimization service

Live optimization requires both the frontend and Python/CasADi API. Static Pages can replay all saved results and models, but cannot execute the current Python solver. The earlier assumption that all public visualization requires a separate live server was incorrect.

## Prepared deployment

[`render.yaml`](../render.yaml) defines one Render Python web service serving the UI, meshes and API together. It declares the free plan, Python/Node versions, complete build command, start command and health check. No account credentials are stored in the repository. Automatic redeploy is disabled; hosting-plan availability and provider limits are controlled by Render.

[Open Render deployment setup](https://render.com/deploy?repo=https://github.com/overjs94/Extended_PMI_IK)

This link opens **deployment setup**, not an existing homepage. A Render account must connect/authorize the repository and create the service. Once deployment succeeds, Render provides the actual public URL; use that returned address rather than guessing a hostname. No hosting connection or deployment token was available to the agent, so remote service creation and public reachability remain unverified.

Build: `bash scripts/deploy-build.sh`. It installs the pinned dependencies, builds the frontend and downloads all 15 robot descriptions and their local assets. Start: `bash scripts/deploy-start.sh`. It binds to the hosting platform's `PORT`, with single-threaded OpenBLAS for repeatable CPU measurements.

The build/start scripts are also usable on an existing Python + Node server. `EXTENDED_PMI_PYTHON` can select an installed Python environment without modifying the scripts.

After public deployment, verify `/api/health`, run one Panda solve from the UI, switch both geometry modes, and test a real humanoid. Public-host performance must be measured there; the repository's timings describe the current cloud machine. Full-model benchmarks, especially ergoCub, can take much longer than small Panda cases. The model cache occupies approximately 270 MB before Python/Node dependencies; retained cache availability and free-plan runtime limits are provider-specific.
