# ICRA experiment readiness

The paper claim, development milestones, fairness rules and freeze gates are defined in [ICRA_PAPER_MAINLINE_AND_ALGORITHM_ROADMAP.md](ICRA_PAPER_MAINLINE_AND_ALGORITHM_ROADMAP.md).

## Current evidence-ready scope

- Original EGO, PCA/Frenet OBB-SFC, and Liu/DecompUtil EllipsoidDecomp can be selected explicitly.
- Strict experiments disable EGO fallback and retain generation/optimization failures.
- Valid prefix corridors constrain MINCO junctions through a GCOPTER-style vertex-hull parameterization and enter the curve objective through a frozen piece-wise half-space penalty; EllipsoidDecomp also supplies the A* guided initial points.
- Schema v17 records goal/replan/retry/topology-attempt identity; separates A* search, certified seed-front-end success and actual corridor-generation attempts; records requested/used direction modes and whether fallback was permitted; measures direct versus hard spatial variables and face–sample pairs per constraint evaluation; retains line-seed containment, timing, continuation, rollback and final safety failures.
- A sampled final-corridor gate performs bounded penalty continuation and rejects unresolved corridor violations instead of counting them as TF-SFC successes. Every continuation candidate must remain finite and collision-free and improve the previous best violation; otherwise v6 restores the best safe iterate before returning failure.
- EllipsoidDecomp seeding handles same-voxel/zero-length queries with a collision-checked short segment, optionally aligns the first seed segment with measured velocity, traverses every intersected voxel, and retries ordinary A* once when final velocity-seed validation reports occupancy. v17 also performs one shared ordinary-A* rebuild when a velocity-aligned seed fails junction-clearance certification; TF-SFC and EllipsoidDecomp use the same trigger, map, goal, piece budget and search cost, and the retry is logged explicitly.
- DecompROS publication is kept separate from corridor generation and optimization.

## Required before freezing paper results

1. Run clean Release builds with the exact ROS Noetic, Eigen, DecompUtil and DecompROS revisions recorded in a lock/setup document.
2. Execute regression tests for at least 30 fixed seeds before large experiments. Confirm that every active hard junction has near-zero `max_junction_violation_final_m`, compare v17 success/latency with the shared clearance retry enabled versus disabled, and retain the v7 hard-junction and v6 soft-only ablations, and verify no increase in final obstacle or swarm-clearance failures.
3. Freeze the final CSV schema (v17 is the current diagnostic schema) and its compatible aggregation script after clean runtime validation. The tool separates candidate calls, planning events, goals, search, corridor generation and conditional optimization. Add an FSM-level goal-arrival/timeout/collision record before calling any aggregate a mission-success rate: schema v17 identifies a goal, retry chain and seed-rebuild provenance but still does not prove execution reached it.
4. Add continuous or sufficiently conservative trajectory-versus-corridor verification. The current `max_corridor_violation_*` values are quadrature-sampled diagnostics, not a mathematical continuous-time certificate.
5. Define identical maps, start/goal pairs, dynamics, obstacle inflation, time limits, warm-up policy and failure denominators for all EGO and GCOPTER baselines.
6. Run the full method matrix and ablations: original EGO, OBB without/with overlap handling, EllipsoidDecomp with extension 0/0.20 m, and GCOPTER FIRI/EllipsoidDecomp/TF-SFC.
7. Report distributions and confidence intervals, not only means: optimizer success, externally checked collision-free/goal-arrival rate, total/corridor/optimizer time, trajectory duration/length, constrained coverage, violation reduction and fallback rate.

## Required if the paper claims the full TF-SFC method

- Construct and validate an anisotropic per-piece deformation metric from the complete local MINCO objective (including environment/dynamic curvature), then feed it through the existing Gramian API. Formal mode-2 runs must set `tf_sfc_allow_direction_fallback:=false`; PCA fallback data is an ablation, never the main method.
- Implement the claimed obstacle cutting-plane selection, face pruning/budgeting and general overlap optimization, with ablations for each component.
- Add corridor MVIE volume (or a justified Chebyshev-radius approximation). The current schema provides line-seed containment and local-map coverage, but these must not be presented as region-volume quality.
- Validate multi-drone interactions if swarm performance is part of the paper claim; current evidence is mainly single-drone corridor integration.
- Add simulation stress tests and, if claimed, real-platform experiments with estimator/control failures separated from planner failures.

Until these items are complete, the branch should be described as an integration MVP plus reproducible corridor baselines, not the final TF-SFC implementation.
