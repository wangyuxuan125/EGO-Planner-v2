# TF-SFC integration status

This directory contains the first, deliberately conservative integration step for
Trajectory-Favorable Safe Flight Corridors. It is disabled by default and keeps
EGO-Planner-v2's original rebound, restart, swarm, and final collision-check paths.

## Implemented in this MVP

- Frenet and trajectory-sample PCA direction providers.
- A sensitivity-Gramian provider with an explicit per-piece Gramian input API.
  Requesting sensitivity directions without Gramians falls back to PCA and records
  the fallback in the corridor metrics.
- A six-face trajectory-aligned OBB corridor baseline. Inflation follows the
  direction utility order, treats space outside the local inflated map as invalid,
  and conservatively rejects occupied voxels whose volume intersects the box.
- Per-piece face count, generation time, weighted directional width, sample slack,
  overlap radius, and direction-fallback metrics.
- Optional junction projection and a frozen, piece-wise soft corridor penalty in
  the existing MINCO/L-BFGS integration loop.
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
- EllipsoidDecomp dilates each seed segment independently. Collision-checked,
  tangent-aligned endpoint extensions give adjacent polytopes a larger shared
  interior while preserving the configured minimum-overlap certificate.
- When a fully visible near-goal A* polyline has more bends than the remaining
  MINCO piece budget, the optimizer keeps a certified prefix and labels the
  unconstrained final segment `piece_budget_tail` instead of rapidly rejecting
  every replan.
- Schema-v5 experiment logs group optimizer calls by goal/replan/attempt,
  record sampled corridor penalty and maximum violation before and after
  L-BFGS, and expose bounded penalty-continuation passes plus strict final
  rejection.
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
| `tf_sfc/use_projection` | Project inner junctions into adjacent-corridor intersections. |
| `tf_sfc/use_soft_penalty` | Add the frozen corridor hinge-squared penalty. |
| `tf_sfc/allow_partial_corridors` | Use a continuous certified prefix in the known local map. |
| `tf_sfc/allow_ego_fallback` | Allow operational fallback; set false for strict method evaluation. |
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

For a first simulation ablation, enable PCA corridors and the soft penalty while
leaving projection disabled. Projection is expected to do little when a corridor
is generated from the same initial trajectory; the frozen penalty is the component
that changes the subsequent optimization feasible region.

The visualization topic is private to the planner node:
`/drone_<id>_ego_planner_node/tf_sfc/polyhedron_array`. An empty array means the
latest request did not produce a valid corridor; it is not necessarily an RViz
plugin error.

## Recommended next implementation step

Expose or compute the per-piece local MINCO mapping and Hessian approximation, then
populate `SensitivityDirectionProvider::setPieceGramians()`. Only after that path is
validated should direction mode `2` be reported as the main method. The current PCA
fallback must be counted separately in experiments.
