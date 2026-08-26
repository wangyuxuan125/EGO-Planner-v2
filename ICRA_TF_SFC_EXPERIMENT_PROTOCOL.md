# ICRA TF-SFC paper experiment protocol

This branch separates the proposed corridor front end from all baselines. Do not
report a method under a different name, and do not count a planning-attempt
success as a mission success.

## Method matrix

| Label | EGO parameters | Corridor role |
|---|---|---|
| EGO | `tf_sfc_enabled:=false` | original obstacle-cost planner |
| OBB-PCA | `corridor_method:=obb direction_mode:=1` | six-face trajectory PCA baseline |
| OBB-Frenet | `corridor_method:=obb direction_mode:=0` | six-face Frenet baseline |
| Liu | `corridor_method:=ellipsoid_decomp` | EllipsoidDecomp / Liu et al. baseline |
| Proposed | `corridor_method:=tf_sfc` | trajectory-aligned inflation plus obstacle cutting planes under a face budget |

The proposed method uses at most `max_faces` planes. Six planes bound the
trajectory-aligned box and at most `max_obs_faces` additional planes exclude
nearby occupied voxels. Obstacle planes retain a ball of radius `min_overlap_radius` at piece
endpoints, where adjacent-corridor overlap is required. Interior samples use
the independent `interior_sample_margin` (default `0 m`). This prevents a
junction-overlap requirement from silently becoming a conservative tube-radius
requirement along the entire piece.

Direction mode 2 may be reported as the sensitivity variant only when
`direction_fallback_count == 0`. Until an actual per-piece MINCO sensitivity
Gramian is supplied, use direction mode 1 for the proposed main row and describe
it as trajectory-PCA, not MINCO sensitivity.

## Build

```bash
cd ~/ICRA2027/EGO-Planner-v2/swarm-playground/main_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

After every pull, verify the loaded library rather than only the node executable:

```bash
ldd devel/lib/ego_planner/ego_planner_node | grep traj_opt
strings devel/lib/libtraj_opt.so | grep face_budget_exhausted
```

## Local-planner simulation commands

Use the same `map_seed`, start, goal, dynamics, sensing range and trial timeout
for all rows. EGO has a 7.5 m planning horizon but a 5.5 m rolling-map update
range in the single-drone demo. Requiring every horizon piece to lie in the
current local map is therefore structurally impossible near the map frontier.
Use certified-prefix mode for the online local planner, but keep EGO fallback
disabled. All corridor methods use the same existing A* seed builder.

Proposed, 12-face budget:

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=true \
  tf_sfc_corridor_method:=tf_sfc \
  tf_sfc_direction_mode:=1 \
  tf_sfc_max_faces:=12 \
  tf_sfc_max_obs_faces:=6 \
  tf_sfc_interior_sample_margin:=0.0 \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=proposed_pca_f12_seed42
```

OBB-PCA baseline:

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 tf_sfc_enabled:=true tf_sfc_corridor_method:=obb \
  tf_sfc_direction_mode:=1 tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=obb_pca_seed42
```

Liu baseline:

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 tf_sfc_enabled:=true \
  tf_sfc_corridor_method:=ellipsoid_decomp \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=liu_seed42
```

## Required ablations

- Face budget: 6/0, 8/2, 10/4 and 12/6 for
  `max_faces/max_obs_faces`.
- Direction: Frenet (0), PCA (1), sensitivity (2, only after the no-fallback
  gate passes).
- Constraint: hard parameterization on/off; keep final corridor validation on.
- Front end: EGO, OBB, Liu and proposed.

Use at least 30 fixed simulation seeds. Do not replace failed trials or tune a
method per seed.

## Recorded metrics and validity gates

Schema v12 writes `ego_runs_v12_drone_0.csv` and
`ego_corridors_v12_drone_0.csv`. In addition to face-budget fields, the
corridor file records `min_obstacle_sample_distance_m`,
`separation_failure_sample_id`, and `separation_failure_at_endpoint` so a
failed cut can be classified as an endpoint-overlap, interior-clearance, or
face-selection problem.

A paper-valid online-local-planner sample must satisfy all of:

- `success == 1`, `collision_free == 1`, and `fallback_to_ego == 0`;
- the valid corridors form one consecutive prefix and
  `corridor_count >= min_valid_pieces`;
- `hard_parameterization_active == 1`;
- `max_corridor_violation_final_m <= max_final_violation_allowed_m`;
- every valid prefix corridor respects the face budget and each valid adjacent
  pair has `overlap_radius_to_next >= min_overlap_radius`.

Report the certified-prefix coverage ratio separately. Do not present it as
full-horizon containment. A later mission logger must verify that replanning
occurs before the vehicle reaches the unconstrained tail. Offline/full-map
corridor experiments may additionally require full piece coverage.

Report planning-attempt success separately from mission success. Mission success
requires reaching the commanded goal within the timeout with no collision,
emergency stop or operator takeover.

## Real-world gate

Real-world tests are run only after the fixed-seed matrix passes. Freeze the
commit SHA and all ROS parameters. Report at least: missions/trials, goal
completion, emergency stops, operator takeovers, average and 95th-percentile
planning time, command rate, speed, tracking RMSE and observed minimum
clearance. GCOPTER is a corridor/optimizer benchmark; EGO is the primary
closed-loop and real-vehicle platform.

## Current paper-readiness boundary

This revision implements the proposed face-bounded obstacle-plane front end and
logs its geometric budget. It does not yet justify a “MINCO sensitivity”
claim, because the optimizer does not yet populate per-piece sensitivity
Gramians. It also does not yet provide an FSM-level mission CSV or continuous
polynomial containment certificate. Those are required before final paper data
collection.


## v11 seed correction

The v10 proposed run generated valid corridors for the first one to three
pieces, then failed because EGO's ordinary initial MINCO trajectory crossed
inflated occupancy. This is expected for the original rebound optimizer but is
invalid as an SFC seed. From v11, OBB and proposed TF-SFC reuse the existing
collision-free A* seed builder already used by the Liu baseline. The optimizer
itself is unchanged, and search timing/coverage remain separately logged.


## v12 separation-margin correction

The v11 A* seed was edge-valid, but the first proposed corridor repeatedly failed with `obstacle_separation_failure`. The cause was not search: the obstacle cutting plane required `min_overlap_radius` around every seed sample. That parameter is a junction-overlap radius and must not be interpreted as a whole-piece tube radius. v12 applies it only to the two piece endpoints and introduces `interior_sample_margin` for interior samples. The default interior margin is zero because the occupancy map already contains the configured obstacle inflation. The cutting-plane offset uses the exact support of the axis-aligned occupied voxel along the selected normal, rather than a direction-independent circumscribed-sphere padding. This preserves voxel exclusion without adding a second artificial clearance. These are geometric-semantics corrections; they do not change the A* algorithm or optimizer objective.

For the first v12 validation, keep `interior_sample_margin:=0.0`, use certified-prefix mode, and keep EGO fallback disabled. If generation still fails, classify the failure using the new per-piece fields before changing search or safety parameters.


## v12.1 overlap-clearance junction refinement

The v12 diagnostic run contained 820/820 identical failures at piece 0, sample 8. Every failing sample was the first internal MINCO junction (`separation_failure_at_endpoint=1`), its nearest occupied-voxel center was approximately `0.1424 m` away, and the face budget never saturated. The segment interior was no longer the blocker; the greedy line-of-sight simplifier selected the farthest visible A* bend, which lay too close to inflated occupancy to contain the requested `0.08 m` junction ball.

The seed post-processor now checks point-to-inflated-voxel-AABB clearance while simplifying the unchanged raw A* path. A visible bend that cannot support `min_overlap_radius` is backed off to an earlier raw-path point that can. This changes MINCO/SFC junction placement, not A* graph search, costs, or the raw A* result. The same post-processing is used for Liu and proposed methods so the corridor-front-end comparison retains a shared seed policy. When refinement is used, `seed_path_strategy` contains the suffix `_overlap_clearance_refined`.

If no reachable internal A* point can support the requested radius, generation fails explicitly with `overlap_too_small`; the implementation does not silently lower the experiment parameter.


## v12.2 local overlap refiner

The first v12.1 run failed before corridor construction: all 940 attempts had `seed_path_point_count=0`, `seed_path_edge_valid=0`, one empty failure row, and `overlap_too_small`. The strategy remained `edge_validated_astar`, proving that requiring raw A* voxel centers themselves to contain the overlap ball was an over-early gate rather than an adjacent-polytope measurement.

v12.2 restores ordinary collision-free line-of-sight simplification, constructs the retained fixed-piece seed, and then locally relocates only its internal junctions. Candidates are searched by increasing displacement within `junction_refine_radius` at `junction_refine_step`; an accepted candidate must satisfy point-to-inflated-voxel-AABB clearance and keep both neighboring seed segments continuously collision-free. Raw A* search remains unchanged. Successful relocation appends `_overlap_local_refined` to `seed_path_strategy`; an exhausted search appends `_overlap_local_refine_failed` and returns `overlap_too_small`.

The default local search radius/step are `0.50 m / 0.05 m`. These parameters must be fixed across corridor methods and included in the experiment manifest. The policy is applied to the final retained seed prefix, including subdivision junctions but excluding the start and terminal prefix endpoint.


## v12.3 safe-polyline envelope correction

The v12.2 run moved all retained junctions successfully: 180/180 attempts used `edge_validated_astar_overlap_local_refined`, produced four seed points, and passed continuous edge validation. Piece 0 generated a valid corridor in every attempt with minimum sample slack about `0.091 m`. Piece 1 then failed in every attempt; most failures were at interior samples and the nearest obstacle-center distance fell to approximately `0.019 m`. This isolates polynomial overshoot: the collision-free piecewise-linear seed was valid, but the unoptimized MINCO polynomial through the same junctions curved toward inflated occupancy.

OBB and proposed TF-SFC now inflate around uniformly sampled straight seed segments, not around the unoptimized MINCO curve. The corresponding initial MINCO piece is retained only as a direction hint for Frenet/PCA/sensitivity orientation. Because the generated H-polytope is convex and contains the line endpoints, it contains the entire straight seed segment. The optimized polynomial is still governed by hard junction parameterization, in-piece sampled penalties, collision checks and final corridor rejection.

This correction implements the intended SeedProvider/EnvelopeProvider separation. It does not make the raw A* path or unoptimized polynomial part of the proposed contribution, and it is applied to both OBB and proposed TF-SFC. Liu already consumes the same validated piecewise-linear seed through EllipsoidDecomp.


## v12.4 continuous-seed separation and overlap floor

The v12.3 run produced one valid corridor in all 70 attempts but failed piece 1 before optimization. All failures occurred with an unsaturated face budget; the reported support sample was the shared endpoint. The cutting-plane normal was still derived from a nearest discrete sample. v12.4 computes the closest point on the continuous sampled seed polyline, evaluates that normal together with every sample-to-obstacle candidate normal, includes exact occupied-voxel AABB support, and selects the plane with maximum certified separation gap.

The overlap threshold also receives a necessary semantic correction. A convex corridor containing radius-`r` balls at both endpoints contains the convex hull of those balls, which is a radius-`r` capsule along the whole segment. Therefore the previous `0.08 m` hard threshold imposed a whole-piece clearance tube on top of the map's `0.10 m` obstacle inflation; it was not merely a junction-connectivity condition. The paper protocol now uses a `0.02 m` hard full-dimensional overlap floor for the 0.1 m grid and reports the achieved overlap radius as a quality metric. The `0.08 m` configuration remains a strict-clearance ablation and must be labelled as such.
