# ICRA paper mainline and algorithm roadmap

本文档把论文大纲、当前 EGO/GCOPTER 代码状态和实验有效性要求收束为一条可检验的研究主线。它同时是算法开发顺序、实验冻结门槛和论文表述边界。

## 1. 建议的论文主线

**核心问题：** 传统 SFC 主要追求大体积自由空间，但轨迹优化器真正需要的是沿“有效下降方向”有足够空间、相邻走廊可连续衔接、且半空间数量可控的可行域。体积大不等于对 MINCO 优化有利；面数过多还会增加约束评估和梯度计算成本。

**核心方法：** Trajectory-Favorable, Face-Bounded SFC (TF-SFC) 使用轨迹局部几何或 MINCO 灵敏度确定优先方向，在固定面预算内选择支撑面和障碍分离面，显式维护相邻走廊的接头内域，并把走廊以连接点硬参数化、段内采样代价和最终安全认证三层方式接入 MINCO。

**建议的一句话贡献：**

> TF-SFC 在固定半空间预算内，针对 MINCO 的有效局部下降方向分配自由空间并维护相邻走廊接头，从而以更少约束提供更有利于轨迹优化的安全可行域。

在真实 MINCO sensitivity Gramian 和 `alpha_max` 验证完成前，论文只能把当前主版本称为 **trajectory-PCA TF-SFC**，不能称为完整的 optimization-sensitivity TF-SFC。

## 2. 论文证据链

| 论文主张 | 必需算法机制 | 必需证据 | 当前状态 |
|---|---|---|---|
| 走廊安全并包含 seed | 连续边有效 seed、障碍分离面、seed containment | 边有效率、包含违反量、独立碰撞检查 | 部分完成；v12 正在修复分离裕量语义 |
| 面数严格受控 | `max_faces/max_obs_faces` 和确定性选面 | 每段面数、饱和率、约束数、耗时 | 基本完成；缺少面效用裁剪 |
| 相邻走廊可衔接 | 接头球/交集构造和 overlap refiner | 最小重叠半径、交集可行率、接头失败率 | 部分完成；当前主要是构造与检查 |
| 走廊对轨迹优化有利 | 实际 MINCO sensitivity 方向和方向宽度目标 | sensitivity width、`alpha_max`、目标下降、迭代数 | PCA/Frenet 已有；真实 sensitivity 尚未完成 |
| 更少约束带来优化收益 | 面预算和冗余面裁剪 | 半空间约束数、LBFGS 次数、优化耗时分布 | 日志部分完成；统一约束计数缺失 |
| 连续轨迹安全 | 连接点硬参数化、段内代价、连续/保守认证 | 最大连续违反、最小净空、拒绝原因 | 连接点硬参数化已完成；连续认证缺失 |
| 在线系统收益 | EGO 闭环滚动规划 | mission success、p95 规划时间、重规划/急停/接管 | 优化调用日志已有；FSM mission logger 缺失 |
| 跨优化器可复现 | GCOPTER 固定 seed/地图走廊基准 | 相同输入下的走廊和优化结果 | 记录框架已有；共享 replay 数据集缺失 |

## 3. 算法最终形态

最终方法按以下顺序实现，每一层都必须可单独消融：

1. **Collision-free seed provider**：统一 A* 路径、连续体素边验证、视线简化和 piece 对齐。搜索是公平输入条件，不作为论文贡献。
2. **Envelope provider**：SampleTube、OBB/capsule 和可选 Bernstein/MINVO verifier 使用统一接口。MINVO 只作为验证器或消融，不作为论文主线。
3. **Direction provider**：Frenet、PCA 和真实 MINCO sensitivity Gramian 三种模式。
4. **Fixed-normal directional inflator**：沿优先方向生成基础支撑面，从占据点候选中选择最有价值的分离面。
5. **Face selector/pruner**：在固定面预算下根据安全必要性、灵敏度方向宽度增益、重叠贡献和冗余度选择面。
6. **Overlap refiner**：在相邻走廊交界处最大化或至少保证公共内域，接头半径与段内安全裕量独立。
7. **Corridor certifier**：验证 face budget、seed containment、相邻交集、障碍分离和连续多项式包含/保守上界。
8. **MINCO integration**：连接点采用走廊/交集顶点凸组合硬参数化；piece 内使用采样半空间代价；最终由 certifier 拒绝未解决违反。
9. **System safety wrapper**：保留 EGO 原始 rebound、碰撞复检、fallback 和 emergency 机制；论文严格实验关闭 EGO fallback，工程/真机模式单独报告 fallback。

## 4. 当前 v12 bug 的方法学解释

v11 的 A* seed 已满足连续边有效，但 proposed TF-SFC 在第 0 段发生 `obstacle_separation_failure`。实现把 `min_overlap_radius` 施加在每一个 seed 采样点，等价于要求整段中心线都具有固定半径自由管道。这不是“相邻走廊接头重叠”的定义，也重复叠加了地图障碍膨胀与占据体素半对角线的保守量。

v12 将两种量拆开：

- piece 首尾端点：使用 `min_overlap_radius`，服务于相邻走廊公共内域；
- piece 内部样本：使用 `interior_sample_margin`，默认 0 m；
- CSV 记录最近障碍距离、失败样本编号及端点标志。

这是几何约束语义修正，不改变 A* 搜索算法，也不把前端搜索改动包装成论文贡献。

## 5. 分阶段修改路径和冻结门槛

### M0 — v12 稳定化

工作：

- proposed TF-SFC 使用 edge-validated A* seed；
- 修正端点 overlap 与 interior margin；
- 输出可定位的分离失败诊断；
- 保持 `allow_partial_corridors=true`、`allow_ego_fallback=false` 的在线严格配置。

通过门槛：

- 30 个固定 seed 中不存在无诊断的生成失败；
- 有效走廊形成从当前状态开始的连续前缀；
- 每段面数不超过预算；
- 失败能分类为 endpoint overlap、interior clearance、face budget 或 local-map coverage。

### M1 — 统一 seed、包络和 replay 输入

工作：

- 抽象 SeedProvider/EnvelopeProvider；
- 保存地图快照、起终点、seed polyline、piece 时间和动力学参数；
- EGO 与 GCOPTER 读取同一 replay 输入；
- 分离 search、envelope、inflation 和 optimizer 计时。

通过门槛：

- 所有走廊方法在完全相同 seed 上运行；
- replay 重复运行得到一致的几何结果；
- 不再用跨系统不同地图的“成功率”直接作结论。

### M2 — 完成真实 MINCO sensitivity

工作：

- 从当前 MINCO 变量化构造每段 `G_i = integral J_i H^{-1} J_i^T dt` 或等价稳定计算；
- 将 Gramian 传入 DirectionProvider；
- 记录特征值、条件数、fallback 原因；
- 计算局部可行下降步长 `alpha_max`。

通过门槛：

- sensitivity 主实验中 `direction_fallback_count=0`；
- 数值差分验证方向/梯度一致性；
- sensitivity、PCA、Frenet 在固定输入上完成消融；
- 论文只主张“扩大有用局部可行下降空间”，不主张全局最优性。

### M3 — 面选择、裁剪和 overlap refiner

工作：

- 候选障碍面按安全必要性和 trajectory-favorable utility 排序；
- 删除冗余或低效用面；
- 在固定 `max_faces` 下联合考虑方向宽度和接头重叠；
- 对不可行接头执行局部重定位/缩放，而不是全局重跑 A*。

通过门槛：

- 任意有效段严格满足面预算；
- 相邻走廊 overlap 合格率达到预设阈值；
- 与“不裁剪”“不优化 overlap”消融相比，约束数/优化耗时和成功率变化可解释。

### M4 — 连续安全认证

工作：

- 使用 Bernstein/Bezier convex-hull、区间界或足够保守的自适应细分验证连续多项式；
- 独立计算连续最小净空；
- 把 sampled penalty 与 final certificate 明确分离。

通过门槛：

- 所有计为成功的轨迹通过独立认证；
- 注入越界轨迹的回归测试必须被拒绝；
- 论文不把采样违反量表述为数学连续保证，除非认证器确实提供该保证。

### M5 — 论文级日志与批处理

工作：

- FSM mission logger：目标到达、超时、碰撞、急停、接管；
- 统一 constraint count、目标函数初末值、逐迭代下降、重规划次数；
- 固定 seed 批处理、参数 manifest、commit/dependency lock；
- 自动输出 Wilson 区间、median/p95 和失败分解。

通过门槛：

- optimizer-call、planning-event、goal/mission 三个统计层级不可混淆；
- CSV schema、聚合脚本、实验 manifest 冻结；
- 任何失败样本都保留在分母中。

### M6 — GCOPTER 基准完成

GCOPTER 只承担论文所需的离线走廊/优化器基准，不完整移植 EGO 在线前端：

- 原始 FIRI；
- Liu/EllipsoidDecomp；
- OBB；
- TF-SFC 的同一核心 inflator；
- 固定 face budget 与共享 replay 输入；
- 统一记录 corridor generation、faces、volume/半径、overlap、constraint count、optimizer time 和轨迹质量。

通过门槛：所有方法使用相同障碍、seed、边界条件、动力学限制、时间限制和失败分母。

### M7 — 闭环与真机

主平台为 EGO。先仿真压力测试，再做真机：

- EGO original；
- OBB-PCA；
- full TF-SFC；
- 可选 Liu 作为仿真对比，不强制真机。

通过门槛：固定代码 SHA 和参数；报告任务完成、碰撞、急停、接管、p95 规划时间、跟踪 RMSE、最小净空和速度。工程 fallback 开启的结果与严格算法结果分表。

### M8 — 多机兼容性（可选）

只验证 TF-SFC 与现有 swarm collision cost、广播和 emergency 机制兼容。除非完成专门的共享走廊/时空走廊设计，不把多机协同走廊作为本文贡献。

## 6. 实验矩阵

### 6.1 Corridor-level shared replay

方法：

- Liu / EllipsoidDecomp；
- GCOPTER FIRI；
- OBB-Frenet；
- OBB-PCA；
- TF-SFC without face pruning；
- TF-SFC without overlap refinement；
- full TF-SFC；
- 可选 MINVO/Bernstein verifier 消融。

指标：

- generation time（mean/median/p95）；
- face count 与 constraint count；
- volume 或 MVIE/Chebyshev radius；
- tangent width、sensitivity-weighted width；
- adjacent overlap radius；
- seed containment、障碍分离和认证成功率；
- face-budget saturation 和失败分解。

### 6.2 EGO optimization and closed-loop

方法：

- original EGO；
- EGO + Liu；
- EGO + OBB-PCA；
- EGO + TF-SFC PCA；
- EGO + TF-SFC sensitivity；
- 关键消融。

指标分三层报告：

- **优化调用：** 后端成功、LBFGS 迭代、初末目标、optimizer time、违反量；
- **规划事件：** total time、corridor time、search time、重试/回滚/重规划；
- **任务：** goal completion、collision、timeout、emergency、takeover、轨迹时长/长度/jerk/clearance。

### 6.3 理论机制验证

散点或分箱分析：

- `alpha_max` vs sensitivity-weighted width；
- `alpha_max` vs overlap radius；
- optimizer iterations/time vs face/constraint count；
- objective decrease vs sensitivity alignment；
- success probability vs certified-prefix coverage。

这组实验连接“走廊几何”与“优化表现”，是论文说服力的核心，不能只用系统成功率替代。

### 6.4 场景

- 狭窄门/窄通道；
- S 弯与连续急转；
- 密集森林；
- dead end / 需要明显侧向调整；
- 室内杂物；
- 真机窄门和室内障碍。

每个方法使用相同 seed、起终点、地图、动力学、障碍膨胀、超时和预热规则；至少 30 个固定随机 seed，并报告置信区间。

## 7. 公平性规则

1. A* 是共享 seed provider，不把搜索改动当 TF-SFC 收益。
2. 若某基线需要搜索/预处理，该时间计入总规划时间，并单独列出。
3. EGO 与 GCOPTER 的系统成功率不能直接相减；跨系统结论只能来自 shared replay 或各自系统内的成对比较。
4. 在线局部地图结果报告 certified-prefix coverage，不冒充 full-horizon containment。
5. `fallback_to_ego=1` 的样本不能计为 TF-SFC 算法成功。
6. 不删除失败 seed，不按方法逐 seed 调参。
7. 不以 weighted width 代理 volume；二者分别报告。
8. 连接点硬参数化不能表述为整条连续曲线硬约束。
9. DecompROS 可视化不等于 Liu 基线；只有实际调用 EllipsoidDecomp 并参与优化才是 Liu 方法。
10. 真机失败按 planner、estimator、controller 和 operator 分层，但所有任务仍保留在总分母。

## 8. 建议论文结构

1. Introduction：大体积 SFC 与优化有效空间、约束复杂度之间的缺口。
2. Related Work：Liu SFC、FIRI/GCOPTER、EGO/MINCO、optimization-aware corridors。
3. Problem Formulation：face-bounded corridor、seed safety、overlap、MINCO local feasible descent。
4. Method：
   - sensitivity/direction estimation；
   - fixed-normal inflation；
   - face selection and pruning；
   - overlap refinement；
   - hard junction + sampled penalty + certificate。
5. Analysis：安全条件、面预算、局部 `alpha_max` 关系和复杂度。
6. Experiments：shared replay、EGO 闭环、消融、真机、可选 swarm compatibility。
7. Limitations：局部地图前缀、动态障碍、非全局最优性。
8. Conclusion。

## 9. 建议图表

- 方法总览图：seed → directions → face-bounded inflation → overlap → MINCO；
- 二维示意图：volume 大但 sensitivity width 小 vs TF-SFC；
- `alpha_max` 与 sensitivity width/overlap 的相关图；
- face count/constraint count 与 optimizer time 的分布；
- 固定场景中 Liu、FIRI、OBB、TF-SFC 走廊可视化；
- EGO 闭环成功/延迟/安全主表；
- 真机轨迹、走廊和最小净空图。

## 10. 当前版本的论文表述边界

当前代码可以称为：

- face-bounded, trajectory-PCA corridor front end；
- GCOPTER-style hard junction parameterization；
- sampled in-piece corridor penalty with final rejection；
- EGO rolling-map certified-prefix integration；
- Liu/OBB/FIRI comparison infrastructure。

当前不能称为：

- 完整 MINCO-sensitivity TF-SFC；
- 连续时间严格走廊包含；
- 已证明的全局优化改进；
- 已完成的 mission-success 统计；
- 已完成的多机协同走廊方法。

只有在 M0–M6 全部通过、schema 和 commit 冻结后，才开始生成论文最终数值；M7 真机通过后再冻结论文最终主表。
