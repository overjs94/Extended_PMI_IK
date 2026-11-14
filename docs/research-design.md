# Solver design and obstacle-guidance direction

## What was preserved

The supplied `IK_new.cpp` uses `Ak = 2 M/dt + B + dt Jᵀ K J`, `Bk = 2 M dq/dt - Jᵀ K e`, QP Hessian `Ak`, linear term `-Bk`, and the midpoint update `q += dt v_hat`, `dq = 2 v_hat - dq`. This is the reduced reference's starting point; arbitrary URDF replaces robot-specific hard-coded FK/Jacobian. Original gains, posture terms, collision engine, stopping rules and ROS scenarios are not claimed to be reproduced.

The older `Inverse_kinematics.cpp` also contains `AkᵀAk` least-squares variants. Those are retained only in the source snapshot, not used to disguise the new nonsymmetric operator as a QP.

The maximal reference uses angular-first right/body twists, `T_next = T Exp(dt v_hat)`, fixed SPD body metrics, and the white-note operator:

```text
H = 2G/dt + B + alpha dt J_task.T K J_task
C_i = G_i ad(v_i) - ad(v_i).T G_i
D = H + C
b = 2G v/dt - grad(V_task)
```

`C` remains skew at every outer iteration. `v_hat.T C v_hat = 0` is tested. Body-local task costs keep `D` block diagonal. Relative grasp costs should remain factors/constraints; introducing arbitrary cross-body objective terms would require revisiting the block Schur and splitting implementations.

Structural URDF joints create five independent local rows for revolute/prismatic joints; fixed links are merged. Floating objects have six unconstrained configuration coordinates. Revolute alignment uses two perpendicular-axis rows. Mimic joints are represented by reduced-coordinate substitution and an extra maximal-coordinate coupling equality. Rotational constraints use a local orientation chart and do not guarantee global branch selection near pi.

## QP, Schur, ADMM and DRS

For a fixed active set, solve `D v + A.T lambda = b`, `A v = rhs`. The dense reference checks the full residual; rank-revealing least squares handles redundant contact multipliers when needed. Block Schur independently applies each 6x6 inverse, then solves the dual system. The current dual solve is dense rank-revealing least squares; sparse GMRES/preconditioners are a future optimization, not implemented performance claims.

QP ADMM minimizes the symmetric quadratic body costs with local factor copies. All incident body copies are stored locally rather than as a dense edge-by-body tensor. The body update solves `(H_i + rho degree_i I) v_i = b_i + rho sum(z-u)`; factor copies project to their equality/half-space sets, then update scaled duals.

DRS operates on the product space of body variables and local factor copies. One resolvent applies `(I+gamma D_i)^-1` to bodies and projects each factor; the other projects all incident copies onto consensus by averaging. The same implementation applies to `C=0` and retained-skew operators. Ordinary scalar-QP ADMM is intentionally unavailable for nonsymmetric `D`.

Warm-started splitting states are reused across midpoint steps. Inner tolerance, feasibility residual and iteration limits are checked. Nonconvergence is recorded as `solver_failed`, not silently accepted. The NumPy path is a serial vectorized reference. A separate Numba backend now runs actual CPU-threaded body/factor/CSR-consensus phases. Threaded subproblem speed and independent-query process throughput are measured separately in `benchmarks/reports/cpu-parallel.md`. There is no GPU/distributed-machine implementation or universal parallel speedup claim.

## Restore true obstacle inequalities

Define `h = ||p_robot - p_obstacle|| - r_robot - r_obstacle`. With `h>=0`, enforce:

```text
J_h v_hat + ((1-P_h)/dt) h >= epsilon_h
lambda_h <= 0
```

The reference uses `P_h=1-contraction`, `epsilon_h=0`, and `lambda_h<=0` in its VI active set. Joint limits use the same inequality convention. Collision rows are **not** turned into always-active equalities with zero normal velocity. At an active boundary, positive outward normal velocity remains allowed. The UI obstacle scenario uses one declared end-effector sphere; the API reports actual sampled clearance separately from joint-limit residuals.

An inequality alone cannot eliminate local minima of the task potential: a symmetric straight-line approach can stop at a boundary where the goal gradient points into the obstacle. Unconditionally constraining normal velocity to zero can additionally prevent leaving the boundary.

## Receding horizon: implemented layer and proposed extension

The implemented outer layer checks the segment from the current task frame to the final goal against sphere obstacles, searches local detour waypoints if blocked, and repeatedly re-evaluates the reference from the current state. Once the direct segment is clear, it returns to the final goal. Preview can be disabled in the UI or with `Settings(preview=False)` for an ablation. The inner constrained PMI problem and final-goal convergence criteria remain intact.

This is **receding geometric look-ahead**, not finite-horizon trajectory MPC. It has no dynamic obstacle prediction, terminal feasibility proof, swept full-body mesh check, or guarantee of a feasible homotopy class. The actual nonlinear post-integration sphere distance is logged at every recorded state. Positive sampled clearance is not a continuous-time collision certificate.

For the next research stage, retain an inner PMI IK loop and add an outer `N`-step reference optimizer:

1. Optimize task-frame/object reference positions with goal progress, velocity smoothness and clearance costs over a short preview horizon.
2. Enforce signed-distance inequalities at all preview knots, trust regions and velocity bounds; use a collision-free terminal waypoint or terminal set where available.
3. Apply only the first reference increment, warm-start the horizon and replan from measured/current configuration.
4. Evaluate both direct inequalities without preview and inequalities with preview on identical blocked cases; record success rate, minimum swept clearance, wall time, path length, inner iterations and constraint residuals.

Changing references changes the potential. Track reference work `V(X_k, r_new)-V(X_k, r_old)` separately; the fixed-reference passivity argument does not automatically prove passivity of the complete outer/inner system. Equality recovery also introduces sign-indefinite constraint work, as the white note already identifies. Finite-step constraint drift should be handled with an explicit step-size/recovery policy and its energy accounting, rather than by claiming linearized inequalities are exact nonlinear safety guarantees.

The original humanoid pose examples are **kinematic, fixed-root / anchored-feet tasks**. The new G1 trajectory example adds a six-coordinate floating base, timed stance/swing/contact switching and EE/body target knots; [trajectory-input.md](trajectory-input.md) describes the CITO-output contract and explicitly authored current input. These remain offline sampled IK tasks. They do not enforce support polygons, friction cones, centroidal dynamics or actuator feasibility and are not locomotion controllers. The synthetic object-carry example adds both feet and both rigid grasps as equalities. Vendor closed-loop linkages, tendon couplings and collision semantics require model-specific validation before broader claims.
