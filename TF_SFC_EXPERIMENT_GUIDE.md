# TF-SFC 实验编译、运行与数据记录指南

本文档对应 `feat/tf-sfc-mvp` 分支。当前包含保守六面 OBB MVP 和 Liu et al. (ICRA 2017) / DecompUtil 两种走廊生成器；功能默认关闭。启用后，单机实验启动文件默认采用严格模式，生成或认证失败会记为失败而不会伪装成 EGO 成功；工程运行仍可显式开启回退。

## 1. 环境与编译

推荐 Ubuntu 20.04、ROS Noetic 和 Release 编译。打开终端：

```bash
cd /你的路径/EGO-Planner-v2/swarm-playground/main_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

每次新开终端都要重新执行：

```bash
cd /你的路径/EGO-Planner-v2/swarm-playground/main_ws
source devel/setup.bash
```

### 1.1 可选：安装 DecompROS 走廊可视化

DecompROS 提供 ROS 消息与 RViz 显示，其递归子模块 DecompUtil 提供 Liu 算法。为避免把 `catkin_make` 和 DecompROS 推荐的 `catkin build` 混在同一构建目录，建议建立独立工作空间：

```bash
sudo apt install python3-catkin-tools ros-noetic-catkin-simple

mkdir -p $HOME/decomp_ws/src
cd $HOME/decomp_ws/src
git clone --recursive https://github.com/sikang/DecompROS.git
cd DecompROS
git submodule update --init --recursive

cd $HOME/decomp_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash

cd /你的路径/EGO-Planner-v2/swarm-playground/main_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

以后新开终端时必须先 source DecompROS，再 source EGO：

```bash
source $HOME/decomp_ws/devel/setup.bash
source /你的路径/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash
```

编译输出出现 `traj_opt: DecompROS corridor visualization enabled` 表示可视化已接入；出现 `traj_opt: Liu/DecompUtil corridor baseline enabled` 表示 Liu 基线可运行。两项是独立能力：只有消息依赖时可显示 OBB，但不能将其称为 Liu 方法。

## 2. 单机仿真与对比命令

所有命令均在 `swarm-playground/main_ws` 下运行。默认把 CSV 保存到 `$HOME/tf_sfc_results/ego`。

### EGO 原始基线

日志仍会保存，但 `tf_sfc_enabled:=false`，可用于计算成功率和规划耗时基线。

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=false \
  tf_sfc_experiment_tag:=ego_original
```

### EGO + PCA OBB-SFC

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=true \
  tf_sfc_corridor_method:=obb \
  tf_sfc_direction_mode:=1 \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_min_valid_pieces:=1 \
  tf_sfc_safety_margin:=0.10 \
  tf_sfc_min_overlap_radius:=0.08 \
  tf_sfc_use_projection:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=pca_obb_hard_junction_v9
```

### EGO + Frenet OBB-SFC 消融

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=true \
  tf_sfc_corridor_method:=obb \
  tf_sfc_direction_mode:=0 \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_use_projection:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=frenet_obb_hard_junction_v9
```

方向模式为 `0=Frenet`、`1=PCA`、`2=Sensitivity Gramian`。当前 MVP 尚未从 MINCO 自动构造 Gramian；模式 2 没有外部有效 Gramian 时会回退到 PCA，并在 CSV 中记录 `direction_fallback_count`。因此当前不能把模式 2 的回退结果作为完整灵敏度方法结果。

### EGO + Liu et al. EllipsoidDecomp SFC

该基线要求 piecewise-linear seed 的连续边无碰撞。原 EGO A* 只保证 26 邻域节点为空闲，可能沿体素边角穿越占据空间；因此 Liu 调用显式开启连续边验证，原 EGO 和其他默认 A* 调用保持关闭。该搜索仍计入 `astar_search_ms` 和总规划时间，`seed_path_strategy` 会记录为 `edge_validated_astar`。不要把修正前的 `liu_hard_junction_v9`（典型失败为 `seed_path_occupied`）与修正后的标签合并统计。

建议先把旧 CSV 移到归档目录，或至少使用下方的新标签，避免追加模式把两种实现混在同一实验组。



```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=true \
  tf_sfc_corridor_method:=ellipsoid_decomp \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_enforce_final_corridor:=true \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_hard_max_vertices:=64 \
  tf_sfc_max_enforcement_passes:=2 \
  tf_sfc_enforcement_weight_multiplier:=3.0 \
  tf_sfc_enforcement_min_improvement:=0.00001 \
  tf_sfc_max_final_violation:=0.001 \
  tf_sfc_use_projection:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_decomp_local_bbox_forward:=0.5 \
  tf_sfc_decomp_local_bbox_lateral:=1.0 \
  tf_sfc_decomp_local_bbox_vertical:=1.0 \
  tf_sfc_decomp_overlap_extension:=0.20 \
  tf_sfc_decomp_initial_velocity_segment:=0.40 \
  tf_sfc_decomp_initial_velocity_threshold:=0.20 \
  tf_sfc_decomp_degenerate_seed_length:=0.10 \
  tf_sfc_decomp_retry_seed_validation_without_velocity:=true \
  tf_sfc_experiment_tag:=liu_edge_validated_hard_junction_v9
```

该分支先调用 EGO 已有 A* 生成无碰撞骨架，做视线简化后按 MINCO piece 预算细分，再逐段调用 `EllipsoidDecomp3D::dilate`。TF-SFC 的这次 A* 调用严格限制在当前膨胀局部地图内，不能把地图外未知空间当作自由空间；这一限制不改变原始 EGO 的其他 A* 调用。为提高相邻多面体在折点处的公共内域，每个分解 seed 段会沿自身切向向共享折点外延伸，延伸部分逐点检查局部地图；遇到障碍或边界时自动折半回退。延伸量由 `tf_sfc_decomp_overlap_extension` 控制并限制为当前段长度的 45% 以下。`0` 可关闭该优化，用于消融实验；它不会降低 `tf_sfc_min_overlap_radius` 的认证阈值。

EGO 使用滚动局部地图；当规划视野大于当前地图范围时，只为从当前状态开始、位于已知地图内的连续前缀生成走廊。若 A* 前缀的转折数超过可用 prefix piece 数，则保留能够放入预算的最远可视折点，而不是丢弃整个走廊序列。第一个未覆盖 piece 记录为 `outside_local_map`，其后记录为 `skipped_after_failure`。因此 `tf_sfc_allow_partial_corridors:=true` 是该局部规划器的正常严格配置；`allow_ego_fallback` 仍保持 `false`，不会把原始 EGO 结果计作走廊方法成功。A*、简化和分解耗时都计入 `corridor_generation_ms` 与 `total_planning_ms`。若 DecompUtil 未被发现，会以 `decomp_util_unavailable` 失败，不会静默退化为 OBB 或 EGO。

`single_drone_interactive.launch` 对 EGO 回退默认采用严格模式（`tf_sfc_allow_ego_fallback=false`）；只有明确进行工程可用性测试时才建议手动开启回退。

v9 延续 v7 的 `tf_sfc_hard_corridor_parameterization`。对于每个 MINCO 内部连接点，若左右 piece 都有有效走廊，代码枚举两个半空间集合交集的顶点；若只有一侧有效，则枚举该侧走廊顶点。优化变量经归一化平方权重映射为这些顶点的凸组合，因此连接点在每次 L-BFGS 迭代中都严格位于对应交集/走廊内。这与 GCOPTER 的 `forwardP/backwardGradP` 参数化作用相同。完全没有走廊覆盖的尾部连接点仍使用 3 维自由坐标。`tf_sfc_hard_max_vertices` 只在异常复杂多面体上限制变量数；被保留顶点的凸包仍是原交集的安全子集。

这里的“硬约束”特指 **MINCO 连接点硬约束**，不能写成“整条连续多项式已被严格限制在走廊内”。piece 内曲线仍通过 `tf_sfc_use_soft_penalty` 的采样半空间代价参与优化，并由最终密集采样门限拒绝未解决越界。Liu et al. 论文的式 (3) 将 `A_i^T Phi_i(t) < b_i` 写成 QP 约束，但正文明确说明实际采用 sample-based confinement；Elastic-Tracker 同样组合了走廊/交集内的结点参数化与曲线采样软惩罚。因此论文中建议使用“hard corridor-junction parameterization + sampled trajectory penalty/certification”这一准确表述。

严格走廊终检默认开启。优化后的轨迹若在任一有效走廊 piece 上出现超过 `tf_sfc_max_final_violation` 的采样半空间越界，优化器会将走廊权重乘以 `tf_sfc_enforcement_weight_multiplier`，从当前解继续优化，最多执行 `tf_sfc_max_enforcement_passes` 次。v6 将倍率从 10 降为 3，并只接受有限、通过 EGO 精细碰撞检查、且走廊最大越界至少改善 `tf_sfc_enforcement_min_improvement` 的候选；发散、碰撞、群体净空失败、求解器异常或不再改善时恢复本轮最佳安全候选并将本次规划记为失败。这样回滚用于保存诊断和避免污染下一次重规划，不会把仍超过 1 mm 阈值的轨迹伪装成成功。无走廊的 `outside_local_map`、`piece_budget_tail` 等尾段不参与该终检。

EllipsoidDecomp 的 seed 有两项稳定性处理：有足够初速度时，首段先沿速度方向构造并验碰，再从该点执行 A*；起终点落在同一体素甚至完全重合时，使用按“速度方向、±x、±y、±z”依次尝试的短探针，只有局部地图确认无碰撞后才用于分解。短探针会按现有 MINCO piece 数细分，因此在 `allow_partial_corridors=false` 下也不会因零长度 A* 路径直接退化为 `insufficient_pieces`。

v9 将 seed 线段检查改为三维体素 DDA 射线遍历，覆盖对角线穿过的每个栅格，避免均匀采样漏掉薄障碍体素。如果速度对齐 A* 已返回路径、但最终细分 seed 在 DecompUtil 前复检为 `seed_path_occupied`，默认以当前状态重建一次普通 A* seed；该行为由 `tf_sfc_decomp_retry_seed_validation_without_velocity` 控制。正式配置保持 `true`；设置为 `false` 可复现无二次重建的消融。日志会记录尝试、回退原因和原失败点，二次重建仍失败时不会回退 EGO。

### v12：TF-SFC 段内裕量与接头重叠分离

v11 的 `edge_validated_astar` 已成功生成连续边无碰撞的 A* seed，但 proposed TF-SFC 在第 0 段以 `obstacle_separation_failure` 失败。原因是 `min_overlap_radius` 被错误用于每一个段内采样点，相当于额外要求整段轨迹带有固定半径的自由管道。v12 只在每段首尾端点使用该重叠半径，并增加独立参数 `tf_sfc_interior_sample_margin`；首轮验证使用 `0.0`。

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=true \
  tf_sfc_corridor_method:=tf_sfc \
  tf_sfc_direction_mode:=1 \
  tf_sfc_max_faces:=12 \
  tf_sfc_max_obs_faces:=6 \
  tf_sfc_safety_margin:=0.10 \
  tf_sfc_min_overlap_radius:=0.08 \
  tf_sfc_interior_sample_margin:=0.0 \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_min_valid_pieces:=2 \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_hard_corridor_parameterization:=true \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=proposed_pca_f12_v12
```

若仍失败，不要先调整 A*。查看 `ego_corridors_v12_drone_0.csv` 的 `min_obstacle_sample_distance_m`、`separation_failure_sample_id` 和 `separation_failure_at_endpoint`：端点失败指向重叠构造，内部失败指向 seed/包络净空，`face_budget_saturated=1` 则指向平面选择或裁剪。

### v12.1：接头净空感知的 seed 分段

v12 CSV 若同时满足 `separation_failure_at_endpoint=1`、`separation_failure_sample_id=8` 且 `face_budget_saturated=0`，说明失败点是第一个 MINCO 内部接头，而不是段内样本或面预算。当前修正保持 raw A* 不变，在视线简化过程中把净空不足的最远可见折点回退到原始 A* 路径上能够容纳 `min_overlap_radius` 的点。使用该修正时，`seed_path_strategy` 会带有 `_overlap_clearance_refined` 后缀。

重新测试时使用新的目录或标签 `proposed_pca_f12_v12_overlap_refined`，不要把修正前后的追加行放在同一实验组。若结果变为 `overlap_too_small`，表示该 raw A* 局部路径上确实没有满足当前固定半径的可达接头；此时应进入 overlap refiner/clearance-aware path ablation，而不是把失败计为面受限生成器或优化器失败。

### v12.2：局部 overlap refiner

v12.1 的 `overlap_too_small` 若伴随 `seed_path_point_count=0`，表示 raw A* 栅格点门槛在走廊生成前拒绝了 seed。v12.2 改为先构造最终保留的 fixed-piece seed，再只移动内部接头；每个候选必须同时满足膨胀体素 AABB 净空和左右两条 seed 边的连续无碰撞。

新增参数：

- `tf_sfc_junction_refine_radius:=0.50`：接头局部搜索半径；
- `tf_sfc_junction_refine_step:=0.05`：候选搜索步长。

正式比较时两项必须在 Liu、OBB 和 TF-SFC 间保持相同。成功移动时 `seed_path_strategy` 带 `_overlap_local_refined`；搜索空间内无可行接头时带 `_overlap_local_refine_failed`。

### 自定义日志目录

```bash
roslaunch ego_planner single_drone_interactive.launch \
  tf_sfc_enabled:=true \
  tf_sfc_direction_mode:=1 \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_log_directory:=$HOME/experiments/icra/scene_01 \
  tf_sfc_experiment_tag:=pca_scene_01
```

启动后，在 RViz 中使用 `2D Nav Goal` 设置目标。为了公平比较，方法之间应保持地图 seed、起终点、速度/加速度限制和重复次数一致。

### RViz 走廊显示

`local_map_off.rviz` 已加入 `TF-SFC Corridors` 显示项，默认订阅：

```text
/drone_0_ego_planner_node/tf_sfc/polyhedron_array
```

也可以先确认消息是否存在：

```bash
rostopic echo -n 1 /drone_0_ego_planner_node/tf_sfc/polyhedron_array
```

发布器只发送 `valid=1` 的走廊。若消息中的 `polyhedrons` 为空，应先查看 CSV 的 `corridor_count` 和 `first_failure_reason`；这表示本次没有有效走廊，并非一定是 RViz 设置错误。

## 3. 输出文件

默认输出：

```text
$HOME/tf_sfc_results/ego/ego_runs_v12_drone_0.csv
$HOME/tf_sfc_results/ego/ego_corridors_v12_drone_0.csv
```

- `ego_runs_v12_drone_<id>.csv`：每次优化候选调用一行。`planning_event_id` 标识一次重规划事件，`retry_index` 表示同一目标下连续失败后的重试序号，`attempt_id` 只表示多拓扑候选，三者不能混用。
- 搜索阶段单独记录 `astar_search_attempted/success/call_count/ms`、原始 A* 路径点数/长度、最终 seed 点数/长度、连续边有效性和局部地图覆盖率。`seed_path_build_ms` 包含搜索、视线简化和 piece 对齐；`corridor_inflation_ms` 单独记录区域生成调用；两者仍包含在 `corridor_generation_ms` 和 `total_planning_ms` 内。
- `ego_corridors_v12_drone_<id>.csv`：每个轨迹分段一行。EllipsoidDecomp 额外记录种子包含是否被评估、是否完整包含线段以及最大归一化半空间违反量。由于多面体是凸集，只要两个端点位于多面体内，整条线 seed 就位于多面体内。
- 最终安全失败继续拆分为 `obstacle_collision_failure`、`swarm_clearance_failure` 和走廊违反；硬连接点与采样曲线违反量保持独立。

文件使用追加模式。不要在同一标签下混入不同地图或参数；每个场景应使用独立目录或唯一 `tf_sfc_experiment_tag`。

快速查看：

```bash
head -n 5 $HOME/tf_sfc_results/ego/ego_runs_v12_drone_0.csv
head -n 5 $HOME/tf_sfc_results/ego/ego_corridors_v12_drone_0.csv
```

## 4. 当前可用于论文统计的字段

论文统计应分层：

1. 搜索前端：`astar_search_success`、`astar_search_ms`、`seed_path_edge_valid` 和路径长度。
2. 走廊前端：种子包含率、走廊生成率、`corridor_inflation_ms`、面数、重叠半径和安全余量。
3. 轨迹后端：只在 `tf_sfc_generated=1` 条件下统计优化成功率、优化时间、轨迹时长/长度及约束违反。
4. 完整系统：仍需要 FSM/仿真器的目标到达、超时和执行期碰撞记录；不能把单次 `success` 或“某个目标至少生成过一次轨迹”写成 mission success。

`mean_weighted_width` 与 `min_overlap_radius` 是解释性几何指标。当前 MVP 尚未严谨提供 corridor volume、MVIE/Chebyshev radius、连续轨迹最小净空、真实 sensitivity Gramian、`alpha_max`、逐迭代 objective decrease 和严格 constraint count，不应以宽度代理冒充体积。

## 5. 分层统计与 Wilson 置信区间

使用仓库内的聚合工具；它明确把候选调用、重规划事件、目标和有效走廊条件下的后端成功率分开：

```bash
python3 tools/analyze_tf_sfc_v9.py \
  $HOME/tf_sfc_results/ego/ego_runs_v12_drone_0.csv
```

输出中的 `optimizer-call success` 仅为诊断项；论文主表应优先使用独立场景/目标上的系统结果、固定 seed 输入的走廊结果，以及有效走廊条件下的后端结果。工具给出的 “goals with >=1 successful plan” 也不是目标到达率。

## 6. 实验注意事项

1. 正式计时使用 Release 编译，关闭不必要的录屏和调试输出。
2. 先做若干次预热运行，再开始记录正式试验。
3. 每种方法、每个场景使用相同 seed 和起终点，建议至少重复 30 次。
4. 保留原始 CSV，只在副本上清洗或聚合数据。
5. 正式 TF-SFC 实验使用 `tf_sfc_allow_ego_fallback:=false`；`fallback_to_ego=1` 的工程运行样本不能算作 TF-SFC 成功样本。
6. `allow_partial_corridors=true` 表示只约束从当前状态开始、位于已知局部地图内的连续有效前缀；末端未知空间不会导致前面已认证走廊全部丢失。
7. 对当前默认参数，局部地图横向半径为 `5.5 m`，规划视野为 `7.5 m`。设置 `allow_partial_corridors=false` 要求整个 7.5 m 轨迹都在 5.5 m 地图内，通常会得到 `outside_local_map`，不应作为默认实验配置。
8. 当剩余轨迹只有 2 个 piece 且目标仍在局部地图外时，局部前缀最多使用 1 个走廊 piece。若目标已在同一体素内，v6 使用碰撞检查短探针并按完整 piece 数细分，避免接近目标时的 `insufficient_pieces` 重试风暴。
9. 正式对比时固定 `tf_sfc_decomp_overlap_extension`。建议主实验使用 `0.20 m`，并额外报告 `0 m` 消融，以判断成功率提升来自重叠构造还是其他参数变化。
10. 当目标位于局部地图内但 A* 折点数超过短轨迹的 piece 预算时，实现会保留认证前缀，并把最后一段标记为 `piece_budget_tail`。该尾段只受原始 EGO 障碍代价和最终碰撞检查约束，必须在论文中与全走廊覆盖样本分开统计。

论文最终实验前还需要完成的项目见 [ICRA_EXPERIMENT_READINESS.md](ICRA_EXPERIMENT_READINESS.md)。

## 7. 将 Liu et al. ICRA 2017 作为对比方法

可行，但必须区分两个组件：

- `DecompROS` 是消息、转换工具和 RViz 插件；把现有 OBB 发布成 `PolyhedronArray` 只属于可视化，不能记作 Liu et al. 方法。
- 论文对比基线必须实际调用 DecompUtil 的 `EllipsoidDecomp3D::dilate(path)`，使用它生成的多面体约束驱动同一套轨迹优化器，并单独记录走廊生成耗时与成功率。

建议将方法命名为 `EGO + EllipsoidDecomp SFC (Liu et al., ICRA 2017)`，并与 `EGO original`、`EGO + PCA OBB-SFC` 分开。所有方法必须使用相同地图 seed、起终点、局部地图范围、障碍膨胀、动力学限制和失败计数规则。若 EllipsoidDecomp 使用预先无碰撞路径，而 OBB 使用未优化初始轨迹，需要把路径搜索/预处理耗时纳入总规划时间，或为所有方法提供相同的无碰撞参考路径，否则比较不公平。

本分支现在已通过 `tf_sfc_corridor_method:=ellipsoid_decomp` 实际调用 DecompUtil，并用 A* 无碰撞骨架驱动同一套 EGO 优化器。论文中应将它标记为独立基线，不与 DecompROS 可视化或 OBB-SFC 混称。
