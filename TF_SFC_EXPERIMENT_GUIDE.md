# TF-SFC 实验编译、运行与数据记录指南

本文档对应 `feat/tf-sfc-mvp` 分支。当前 TF-SFC 是保守的六面 OBB MVP；功能默认关闭。启用后，单机实验启动文件默认采用严格模式，生成或认证失败会记为失败而不会伪装成 EGO 成功；工程运行仍可显式开启回退。

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
  tf_sfc_direction_mode:=1 \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_min_valid_pieces:=1 \
  tf_sfc_safety_margin:=0.10 \
  tf_sfc_min_overlap_radius:=0.08 \
  tf_sfc_use_projection:=false \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=pca_obb_soft
```

### EGO + Frenet OBB-SFC 消融

```bash
roslaunch ego_planner single_drone_interactive.launch \
  map_seed:=42 \
  tf_sfc_enabled:=true \
  tf_sfc_direction_mode:=0 \
  tf_sfc_allow_partial_corridors:=true \
  tf_sfc_allow_ego_fallback:=false \
  tf_sfc_use_projection:=false \
  tf_sfc_use_soft_penalty:=true \
  tf_sfc_experiment_tag:=frenet_obb_soft
```

方向模式为 `0=Frenet`、`1=PCA`、`2=Sensitivity Gramian`。当前 MVP 尚未从 MINCO 自动构造 Gramian；模式 2 没有外部有效 Gramian 时会回退到 PCA，并在 CSV 中记录 `direction_fallback_count`。因此当前不能把模式 2 的回退结果作为完整灵敏度方法结果。

`single_drone_interactive.launch` 对 EGO 回退默认采用严格模式（`tf_sfc_allow_ego_fallback=false`）；只有明确进行工程可用性测试时才建议手动开启回退。

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

## 3. 输出文件

默认输出：

```text
$HOME/tf_sfc_results/ego/ego_runs_v2_drone_0.csv
$HOME/tf_sfc_results/ego/ego_corridors_v2_drone_0.csv
```

- `ego_runs_v2_drone_<id>.csv`：每次优化调用一行，包含实验标签、状态、成功/无碰撞标志、规划与优化耗时、LBFGS 迭代、有效/失败分段数、首个失败原因、最终代价、轨迹时长和采样近似轨迹长度。
- `ego_corridors_v2_drone_<id>.csv`：每个轨迹分段一行；有效段记录面数、生成时间、加权方向宽度、样本余量和相邻重叠半径，无效段记录 `outside_local_map`、`initial_obb_occupied`、`overlap_too_small` 等原因。

文件使用追加模式。不要在同一标签下混入不同地图或参数；建议每个场景使用独立目录或唯一 `tf_sfc_experiment_tag`。

快速查看：

```bash
head -n 5 $HOME/tf_sfc_results/ego/ego_runs_v2_drone_0.csv
head -n 5 $HOME/tf_sfc_results/ego/ego_corridors_v2_drone_0.csv
```

## 4. 当前可用于论文统计的字段

当前可靠记录：成功率、collision-free rate、mean/p95/max planning time、优化耗时、走廊生成时间、LBFGS 迭代、最终目标值、轨迹时长、采样近似轨迹长度、重规划/反弹计数、面数、加权方向宽度、样本余量、重叠半径和回退率。

当前 MVP 尚未严谨提供 corridor volume、Chebyshev radius、连续轨迹最小净空、真实 sensitivity Gramian、`alpha_max`、逐迭代 objective decrease 和严格 constraint count。发表实验前应在后续完整模块中实现这些量，不要用占位值代替。

## 5. 批量统计 mean / p95 / max

下面的命令只使用 Python 标准库：

```bash
python3 - $HOME/tf_sfc_results/ego/ego_runs_v2_drone_0.csv <<'PY'
import csv, math, statistics, sys

rows = list(csv.DictReader(open(sys.argv[1], newline='')))
for tag in sorted({r['experiment_tag'] for r in rows}):
    group = [r for r in rows if r['experiment_tag'] == tag]
    values = sorted(float(r['total_planning_ms']) for r in group
                    if math.isfinite(float(r['total_planning_ms'])))
    if not values:
        continue
    p95 = values[math.ceil(0.95 * len(values)) - 1]
    success = sum(int(r['success']) for r in group) / len(group)
    print(tag, 'n=', len(group), 'success=', success,
          'mean_ms=', statistics.fmean(values), 'p95_ms=', p95,
          'max_ms=', max(values))
PY
```

## 6. 实验注意事项

1. 正式计时使用 Release 编译，关闭不必要的录屏和调试输出。
2. 先做若干次预热运行，再开始记录正式试验。
3. 每种方法、每个场景使用相同 seed 和起终点，建议至少重复 30 次。
4. 保留原始 CSV，只在副本上清洗或聚合数据。
5. 正式 TF-SFC 实验使用 `tf_sfc_allow_ego_fallback:=false`；`fallback_to_ego=1` 的工程运行样本不能算作 TF-SFC 成功样本。
6. `allow_partial_corridors=true` 表示只约束从当前状态开始、位于已知局部地图内的连续有效前缀；末端未知空间不会导致前面已认证走廊全部丢失。
