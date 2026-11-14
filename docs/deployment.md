# Public website deployment

**There is no verified public homepage URL yet.** The GitHub repository link is source code. The cloud workbench has been tested internally, which does not make it publicly hosted.

The application needs both the Vite/Three.js frontend and the Python/CasADi solver API. A static GitHub Pages upload alone cannot run the current solvers.

## Prepared deployment

[`render.yaml`](../render.yaml) defines one Render Python web service serving the UI, meshes and API together. It declares the free plan, Python/Node versions, complete build command, start command and health check. No account credentials are stored in the repository. Automatic redeploy is disabled; hosting-plan availability and provider limits are controlled by Render.

[Open Render deployment setup](https://render.com/deploy?repo=https://github.com/overjs94/Extended_PMI_IK)

This link opens **deployment setup**, not an existing homepage. A Render account must connect/authorize the repository and create the service. Once deployment succeeds, Render provides the actual public URL; use that returned address rather than guessing a hostname. No hosting connection or deployment token was available to the agent, so remote service creation and public reachability remain unverified.

Build: `bash scripts/deploy-build.sh`. It installs the pinned dependencies, builds the frontend and downloads all 15 robot descriptions and their local assets. Start: `bash scripts/deploy-start.sh`. It binds to the hosting platform's `PORT`, with single-threaded OpenBLAS for repeatable CPU measurements.

The build/start scripts are also usable on an existing Python + Node server. `EXTENDED_PMI_PYTHON` can select an installed Python environment without modifying the scripts.

After public deployment, verify `/api/health`, run one Panda solve from the UI, switch both geometry modes, and test a real humanoid. Public-host performance must be measured there; the repository's timings describe the current cloud machine. Full-model benchmarks, especially ergoCub, can take much longer than small Panda cases. The model cache occupies approximately 270 MB before Python/Node dependencies; retained cache availability and free-plan runtime limits are provider-specific.
