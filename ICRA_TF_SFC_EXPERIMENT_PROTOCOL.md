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
nearby occupied voxels. Every accepted obstacle plane must retain a ball of
radius `min_overlap_radius` around every seed sample. This makes overlap a
construction invariant rather than only a post-hoc check.

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

## Strict simulation commands

Use the same `map_seed`, start, goal, dynamics, sensing range and trial timeout
for all rows. Paper runs must disable both partial coverage and planner fallback.

Proposed, 12-face budget:

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=true \
  tf_sfc_corridor_method:=tf_sfc \
  tf_sfc_direction_mode:=1 \
  tf_sfc_max_faces:=12 \
  tf_sfc_max_obs_faces:=6 \
  tf_sfc_allow_partial_corridors:=false \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=proposed_pca_f12_seed42
```

OBB-PCA baseline:

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 tf_sfc_enabled:=true tf_sfc_corridor_method:=obb \
  tf_sfc_direction_mode:=1 tf_sfc_allow_partial_corridors:=false \
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
  tf_sfc_allow_partial_corridors:=false \
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

Schema v10 writes `ego_runs_v10_drone_0.csv` and
`ego_corridors_v10_drone_0.csv`. The corridor file adds
`obstacle_face_count`, `obstacle_point_count`,
`face_budget_saturated`, and `anchor_clearance_radius`.

A paper-valid strict planning sample must satisfy all of:

- `success == 1`, `collision_free == 1`, and `fallback_to_ego == 0`;
- `corridor_count == piece_count` and `failed_piece_count == 0`;
- `hard_parameterization_active == 1`;
- `max_corridor_violation_final_m <= max_final_violation_allowed_m`;
- every corridor is valid, respects the face budget, and has
  `overlap_radius_to_next >= min_overlap_radius`.

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
