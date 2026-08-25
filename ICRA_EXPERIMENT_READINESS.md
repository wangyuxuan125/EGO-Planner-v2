# ICRA experiment readiness

## Current evidence-ready scope

- Original EGO, PCA/Frenet OBB-SFC, and Liu/DecompUtil EllipsoidDecomp can be selected explicitly.
- Strict experiments disable EGO fallback and retain generation/optimization failures.
- Valid prefix corridors constrain MINCO junctions through a GCOPTER-style vertex-hull parameterization and enter the curve objective through a frozen piece-wise half-space penalty; EllipsoidDecomp also supplies the A* guided initial points.
- Schema v8 records goal/replan/attempt identity, timing, geometry, hard-parameterization coverage and junction violation, sampled curve penalty/violation, seed strategy and velocity-seed fallback provenance, continuation candidates, monotonic acceptance, rollback reason, and separate final obstacle/swarm-clearance failures.
- A sampled final-corridor gate performs bounded penalty continuation and rejects unresolved corridor violations instead of counting them as TF-SFC successes. Every continuation candidate must remain finite and collision-free and improve the previous best violation; otherwise v6 restores the best safe iterate before returning failure.
- EllipsoidDecomp seeding handles same-voxel/zero-length queries with a collision-checked short segment, optionally aligns the first seed segment with measured velocity, traverses every intersected voxel, and retries ordinary A* once when final velocity-seed validation reports occupancy.
- DecompROS publication is kept separate from corridor generation and optimization.

## Required before freezing paper results

1. Run clean Release builds with the exact ROS Noetic, Eigen, DecompUtil and DecompROS revisions recorded in a lock/setup document.
2. Execute regression tests for at least 30 fixed seeds before large experiments. Confirm that every active hard junction has near-zero `max_junction_violation_final_m`, compare v8 success/latency against the v7 hard-junction result and v6 soft-only ablation, and verify no increase in final obstacle or swarm-clearance failures.
3. Freeze one CSV schema and aggregation script. Aggregate optimizer calls by independent `goal_id`; report replan-call success only as a secondary runtime metric. Add an FSM-level goal-arrival/timeout/collision record before calling the aggregate a mission-success rate: schema v8 identifies a goal but does not yet prove that the vehicle reached it.
4. Add continuous or sufficiently conservative trajectory-versus-corridor verification. The current `max_corridor_violation_*` values are quadrature-sampled diagnostics, not a mathematical continuous-time certificate.
5. Define identical maps, start/goal pairs, dynamics, obstacle inflation, time limits, warm-up policy and failure denominators for all EGO and GCOPTER baselines.
6. Run the full method matrix and ablations: original EGO, OBB without/with overlap handling, EllipsoidDecomp with extension 0/0.20 m, and GCOPTER FIRI/EllipsoidDecomp/TF-SFC.
7. Report distributions and confidence intervals, not only means: optimizer success, externally checked collision-free/goal-arrival rate, total/corridor/optimizer time, trajectory duration/length, constrained coverage, violation reduction and fallback rate.

## Required if the paper claims the full TF-SFC method

- Construct and validate the actual per-piece MINCO sensitivity Gramian instead of reporting the current PCA fallback as sensitivity mode.
- Implement the claimed obstacle cutting-plane selection, face pruning/budgeting and general overlap optimization, with ablations for each component.
- Add corridor volume or Chebyshev-radius metrics and a clearly defined coverage metric.
- Validate multi-drone interactions if swarm performance is part of the paper claim; current evidence is mainly single-drone corridor integration.
- Add simulation stress tests and, if claimed, real-platform experiments with estimator/control failures separated from planner failures.

Until these items are complete, the branch should be described as an integration MVP plus reproducible corridor baselines, not the final TF-SFC implementation.
