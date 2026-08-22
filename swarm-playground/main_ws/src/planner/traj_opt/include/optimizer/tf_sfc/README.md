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
- Automatic fallback to the original EGO optimizer if any corridor or overlap
  certificate fails.

This is an integration and OBB-baseline milestone, not the paper's full TF-SFC
method. In particular, obstacle cutting planes, face pruning, overlap refinement,
and construction of `G_i = integral(J H^-1 J^T dt)` from EGO's MINCO model are not
yet implemented.

## Parameters

The launch files expose the following private ROS parameters:

| Parameter | Meaning |
| --- | --- |
| `tf_sfc/enabled` | Master switch; defaults to `false`. |
| `tf_sfc/direction_mode` | `0`: Frenet, `1`: PCA, `2`: sensitivity Gramian. |
| `tf_sfc/use_projection` | Project inner junctions into adjacent-corridor intersections. |
| `tf_sfc/use_soft_penalty` | Add the frozen corridor hinge-squared penalty. |
| `tf_sfc/max_faces` | Face budget; the MVP needs at least 6 and produces 6. |
| `tf_sfc/samples_per_piece` | Samples used for the trajectory envelope. |
| `tf_sfc/safety_margin` | Padding around sampled trajectory points. |
| `tf_sfc/min_overlap_radius` | Required certified overlap radius at each junction. |
| `tf_sfc/max_inflation_distance` | Per-side inflation cap. |
| `tf_sfc/inflation_step` | Directional inflation step. |
| `tf_sfc/weight` | Soft corridor penalty weight. |
| `tf_sfc/penalty_epsilon` | Interior buffer used by the soft penalty. |

For a first simulation ablation, enable PCA corridors and the soft penalty while
leaving projection disabled. Projection is expected to do little when a corridor
is generated from the same initial trajectory; the frozen penalty is the component
that changes the subsequent optimization feasible region.

## Recommended next implementation step

Expose or compute the per-piece local MINCO mapping and Hessian approximation, then
populate `SensitivityDirectionProvider::setPieceGramians()`. Only after that path is
validated should direction mode `2` be reported as the main method. The current PCA
fallback must be counted separately in experiments.
