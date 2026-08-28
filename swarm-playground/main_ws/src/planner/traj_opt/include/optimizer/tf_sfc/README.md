# TF-SFC integration status

This directory contains the first, deliberately conservative integration step for
Trajectory-Favorable Safe Flight Corridors. It is disabled by default and keeps
EGO-Planner-v2's original rebound, restart, swarm, and final collision-check paths.

## Implemented in this MVP

- Frenet and trajectory-sample PCA direction providers.
- A sensitivity-Gramian provider with an explicit per-piece Gramian input API.
  Direction fallback is configurable: engineering runs may fall back and record
  requested/used modes, while formal mode-2 runs disable fallback and fail closed.
  The repository does not yet derive the anisotropic Gramian from the complete
  local MINCO objective; PCA remains a named ablation.
- A six-face trajectory-aligned OBB corridor baseline. Inflation follows the
  direction utility order, treats space outside the local inflated map as invalid,
  and conservatively rejects occupied voxels whose volume intersects the box.
- Per-piece face count, generation time, weighted directional width, sample slack,
  overlap radius, and direction-fallback metrics.
- GCOPTER-style hard parameterization of MINCO junctions as convex combinations
  of vertices of adjacent-corridor intersections, with analytic gradient
  back-propagation. Optional projection remains an initialization preconditioner.
- A frozen, piece-wise sampled soft corridor penalty and strict final sampled
  gate for the polynomial between hard-constrained junctions.
- A certified-prefix policy: a piece outside the current local map no longer
  discards preceding valid corridors. Invalid and skipped pieces carry explicit
  failure reasons and receive no corridor penalty.
- EllipsoidDecomp uses the same certified-prefix policy: its A* seed terminates
  one voxel inside the rolling-map boundary and preserves at least one
  unconstrained tail piece when the planning horizon extends beyond known space.
  The TF-SFC seed search treats the rolling-map exterior as invalid without
  changing EGO's other A* calls, and records distinct start/search/boundary/
  occupancy failures.
  If the simplified A* prefix has more bends than the available prefix pieces,
  the certified path stops at the farthest bend that fits instead of rejecting
  every corridor near the goal.
- If the ordinary corridor seed fails capsule-clearance certification, TF-SFC
  and EllipsoidDecomp retry once with the same clearance-aware global A* edge
  validator. Original EGO A* calls retain their default behavior, and the
  clearance-search time and provenance are logged separately.
- EllipsoidDecomp dilates each seed segment independently. Collision-checked,
  tangent-aligned endpoint extensions give adjacent polytopes a larger shared
  interior while preserving the configured minimum-overlap certificate.
- When a fully visible near-goal A* polyline has more bends than the remaining
  MINCO piece budget, the optimizer keeps a certified prefix and labels the
  unconstrained final segment `piece_budget_tail` instead of rapidly rejecting
  every replan.
- Schema-v18 experiment logs group optimizer calls by goal/replan/attempt
  and separately record the commanded global GoalSet and the effective local
  planning start/target. They expose seed-front-end success, actual corridor
  attempts, requested/used direction modes, direct-versus-hard spatial variables,
  and face–sample pairs per constraint evaluation. Seed clearance/repair failures
  remain separate from true corridor-overlap failures.
- Configurable EGO fallback. Operational launches may allow fallback; strict
  experiments can reject generation failures instead of silently counting an
  original-EGO result as TF-SFC success.
- Optional DecompROS `PolyhedronArray` publication. When `decomp_ros_msgs` is
  available at build time, every valid corridor is converted from `n.dot(x) <= b`
  to the point/outer-normal representation used by the DecompROS RViz plugin.

This is an integration and OBB-baseline milestone, not the paper's full TF-SFC
method. In particular, obstacle cutting planes, face pruning, overlap refinement,
and construction of `G_i = integral(J H^-1 J^T dt)` from EGO's MINCO model are not
yet implemented.

## Parameters

The launch files expose the following private ROS parameters:

| Parameter | Meaning |
| --- | --- |
| `tf_sfc/enabled` | Master switch; defaults to `false`. |
| `tf_sfc/corridor_method` | `obb` for the TF-SFC MVP or `ellipsoid_decomp` for the Liu et al. baseline. |
| `tf_sfc/direction_mode` | `0`: Frenet, `1`: PCA, `2`: sensitivity Gramian. |
| `tf_sfc/allow_direction_fallback` | Permit a failed direction provider to fall back to PCA/Frenet. Set `false` for formal mode-2 experiments. |
| `tf_sfc/use_projection` | Project inner junctions into adjacent-corridor intersections. |
| `tf_sfc/hard_corridor_parameterization` | Keep MINCO junctions inside corridor/intersection vertex hulls for every optimizer iterate. |
| `tf_sfc/hard_max_vertices` | Maximum retained vertices per constrained junction; retained hull remains a safe subset. |
| `tf_sfc/hard_vertex_tolerance` | Numerical tolerance for H-to-V intersection enumeration. |
| `tf_sfc/use_soft_penalty` | Add the frozen corridor hinge-squared penalty. |
| `tf_sfc/allow_partial_corridors` | Use a continuous certified prefix in the known local map. |
| `tf_sfc/allow_ego_fallback` | Allow generation-time operational fallback; set false for strict method evaluation. An active hard mapping is not relabelled as original EGO mid-solve. |
| `tf_sfc/visualization_enabled` | Publish valid corridor candidates for the DecompROS RViz plugin. |
| `tf_sfc/visualization_frame` | Frame ID used by the corridor message; defaults to `world`. |
| `tf_sfc/min_valid_pieces` | Minimum certified prefix length required to enable TF-SFC. |
| `tf_sfc/max_faces` | Face budget; the MVP needs at least 6 and produces 6. |
| `tf_sfc/samples_per_piece` | Samples used for the trajectory envelope. |
| `tf_sfc/safety_margin` | Padding around sampled trajectory points. |
| `tf_sfc/min_overlap_radius` | Required certified overlap radius at each junction. |
| `tf_sfc/max_inflation_distance` | Per-side inflation cap. |
| `tf_sfc/inflation_step` | Directional inflation step. |
| `tf_sfc/weight` | Soft corridor penalty weight. |
| `tf_sfc/penalty_epsilon` | Interior buffer used by the soft penalty. |
| `tf_sfc/decomp_local_bbox_{forward,lateral,vertical}` | Local EllipsoidDecomp bounding-box dimensions. |
| `tf_sfc/decomp_overlap_extension` | Collision-checked tangent extension in metres; `0` disables it for ablation. |
| `tf_sfc/decomp_retry_seed_validation_without_velocity` | Retry ordinary A* once when a velocity-aligned seed fails final occupancy validation; defaults to `true`. |
| `tf_sfc/seed_retry_without_velocity_on_clearance_failure` | Permit one shared seed rebuild after clearance certification fails. |
| `tf_sfc/seed_clearance_astar_enabled` | Use global clearance-certified edges for that rebuild; disable only for the v17 occupancy-A* ablation. |
| `tf_sfc/seed_clearance_astar_time_limit` | Clearance-aware search timeout in seconds; defaults to 0.20 and is included in total planning time. |
| `tf_sfc/trajectory_repair_enabled` | Rebuild the worst proposed corridor around actual initial MINCO samples before hard-parameterization setup. TF-SFC only. |
| `tf_sfc/trajectory_repair_max_passes` | Bounded repair count; defaults to one. |
| `tf_sfc/trajectory_repair_trigger` | Initial sampled violation in metres required before attempting repair. |
| `tf_sfc/trajectory_repair_min_improvement` | Required piece-wise reduction unless the repaired piece already satisfies the final tolerance. |

The hard mapping constrains junction points only. Polynomial interiors still use
the sampled soft penalty and final sampled certification; this is not a
continuous-time containment proof. Disable `hard_corridor_parameterization` only
for the explicit soft-only ablation.

The visualization topic is private to the planner node:
`/drone_<id>_ego_planner_node/tf_sfc/polyhedron_array`. An empty array means the
latest request did not produce a valid corridor; it is not necessarily an RViz
plugin error.

## Bounded trajectory-feasibility repair

Schema v20 may replace only the worst violating TF-SFC corridor with a
candidate inflated around the actual initial MINCO curve. The candidate must
remain obstacle-free and face-bounded, preserve both adjacent overlaps, improve
the selected piece, and not worsen the global sampled violation. The accepted
set is then frozen before hard junction parameterization and L-BFGS. This is a
bounded corridor-front-end operation, not an optimizer-time corridor mutation.

## Recommended next implementation step

Expose or compute the per-piece local MINCO mapping and Hessian approximation, then
populate `SensitivityDirectionProvider::setPieceGramians()`. Only after that path is
validated should direction mode `2` be reported as the main method. The current PCA
fallback must be counted separately in experiments.
