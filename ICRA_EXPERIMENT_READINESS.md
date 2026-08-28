# ICRA experiment readiness

The paper claim, development milestones, fairness rules and freeze gates are defined in [ICRA_PAPER_MAINLINE_AND_ALGORITHM_ROADMAP.md](ICRA_PAPER_MAINLINE_AND_ALGORITHM_ROADMAP.md).

## Current evidence-ready scope

- Original EGO, PCA/Frenet OBB-SFC, and Liu/DecompUtil EllipsoidDecomp can be selected explicitly.
- Strict experiments disable EGO fallback and retain generation/optimization failures.
- Valid prefix corridors constrain MINCO junctions through a GCOPTER-style vertex-hull parameterization and enter the curve objective through a frozen piece-wise half-space penalty; EllipsoidDecomp also supplies the A* guided initial points.
- Schema v24 records goal/replan/retry/topology-attempt identity; separates A* search, certified seed-front-end success and actual corridor-generation attempts; records requested/used direction modes, metric source and velocity alignment; measures direct versus hard spatial variables and face–sample pairs per constraint evaluation; retains line-seed containment, timing, continuation, rollback and final safety failures.
- A sampled final-corridor gate performs bounded penalty continuation and rejects unresolved corridor violations instead of counting them as TF-SFC successes. Every continuation candidate must remain finite and collision-free and improve the previous best violation; otherwise v6 restores the best safe iterate before returning failure.
- EllipsoidDecomp seeding handles same-voxel/zero-length queries with a collision-checked short segment, optionally aligns the first seed segment with measured velocity, traverses every intersected voxel, and retries ordinary A* once when final velocity-seed validation reports occupancy. v18 performs one shared clearance-aware A* rebuild whenever the ordinary corridor seed fails junction-clearance certification; TF-SFC and EllipsoidDecomp use the same edge certificate, map, goal, overlap radius and piece budget, and the retry time/provenance is logged explicitly.
- DecompROS publication is kept separate from corridor generation and optimization.
- Schema v23 retains explicit counts of MINCO pieces, resampled seed
  segments and corridor slots. `samples_per_piece=8` is a verification/
  quadrature resolution, not an eight-corridor backend. After the first v22
  regression, seed-geometric direct acceptance and outer repair are disabled
  by default and the v21 differentiable hard parameterization is restored.
- Schema v24 adds an automatic per-piece MINCO differential-state Gramian,
  explicit trajectory-weighted-width minus face-count quality, and a bounded
  post-collision progress guard. The 12-face cap and 0.02 m adjacent-overlap
  threshold remain hard gates. These mechanisms are diagnostic until Release
  runtime and closed-loop arrival tests pass.
- Schema v25 keeps v24 as the proxy ablation and adds a separate strict mode 3.
  It estimates the complete fixed-duration pre-corridor spatial-objective
  Hessian by central differences of the analytic MINCO waypoint gradient,
  symmetrizes and PSD-regularizes it, and projects its inverse to a 3x3
  workspace compliance per piece. Raw curvature, regularization, condition,
  evaluation count and time are logged explicitly.

## Required before freezing paper results

1. Run clean Release builds with the exact ROS Noetic, Eigen, DecompUtil and DecompROS revisions recorded in a lock/setup document.
2. Execute regression tests for at least 30 fixed seeds before large experiments. Confirm that every active hard junction has near-zero `max_junction_violation_final_m`, compare v18 success/latency with the shared clearance-aware A* enabled versus the v17 occupancy-A* front end, and retain the v7 hard-junction and v6 soft-only ablations, and verify no increase in final obstacle or swarm-clearance failures.
3. Freeze the final CSV schema (v25 is the current diagnostic schema) and its compatible aggregation script after clean runtime validation. The tool separates candidate calls, planning events, goals, search, metric construction, corridor generation and conditional optimization. Add an FSM-level goal-arrival/timeout/collision record before calling any aggregate a mission-success rate: schema v25 identifies a goal, retry chain, seed/MINCO alignment, direction provenance, local-curvature regularization, progress rejection and repair provenance but still does not prove execution reached it.
4. Add continuous or sufficiently conservative trajectory-versus-corridor verification. The current `max_corridor_violation_*` values are quadrature-sampled diagnostics, not a mathematical continuous-time certificate.
5. Define identical maps, start/goal pairs, dynamics, obstacle inflation, time limits, warm-up policy and failure denominators for all EGO and GCOPTER baselines.
6. Run the full method matrix and ablations: original EGO, OBB without/with overlap handling, EllipsoidDecomp with extension 0/0.20 m, and GCOPTER FIRI/EllipsoidDecomp/TF-SFC.
7. Report distributions and confidence intervals, not only means: optimizer success, externally checked collision-free/goal-arrival rate, total/corridor/optimizer time, trajectory duration/length, constrained coverage, violation reduction and fallback rate.

## Required if the paper claims the full TF-SFC method

- Runtime-validate v25 mode 3 against v24 mode 2 and finite-difference-step/eigenvalue-floor sensitivity tests. Formal mode-3 runs must set `tf_sfc_allow_direction_fallback:=false`; PCA fallback data is an ablation, never the main method. The correct claim is local fixed-duration full-spatial-objective compliance, not a global nonlinear Hessian certificate.
- Validate the v24 bounded face-aware candidate selection, then implement any stronger claimed obstacle cutting-plane pruning or general overlap optimization with separate ablations. The present overlap is a hard feasibility gate, not a globally optimized objective.
- Add corridor MVIE volume (or a justified Chebyshev-radius approximation). The current schema provides line-seed containment and local-map coverage, but these must not be presented as region-volume quality.
- Validate multi-drone interactions if swarm performance is part of the paper claim; current evidence is mainly single-drone corridor integration.
- Add simulation stress tests and, if claimed, real-platform experiments with estimator/control failures separated from planner failures.

Until these items are complete, the branch should be described as an
application-friendly, face-bounded MINCO differential-state proxy plus
reproducible corridor baselines, not an exact full-objective sensitivity or
maximum-volume implementation.
