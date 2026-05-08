---
title: "Lattice Planner 格栅规划器函数级源码解析"
---

# Lattice Planner 格栅规划器函数级源码解析

本文聚焦 `modules/planning/planners/lattice/` 目录，按函数级粒度拆解 Lattice 规划器的完整流程：行为空块（`PathTimeGraph`、`CollisionChecker`、`FeasibleRegion`、`PredictionQuerier`）、轨迹生成（`Trajectory1dGenerator`、`EndConditionSampler`、`TrajectoryCombiner`）、轨迹评估（`TrajectoryEvaluator`）以及备选轨迹（`BackupTrajectoryGenerator`）。

## 1. 模块定位

Lattice Planner 是 Apollo 的**横向+纵向解耦规划器**。它将 2D 轨迹规划问题分解为独立的 1D 纵向（s 方向）和 1D 横向（l 方向）轨迹生成，然后通过多项式曲线拟合和代价评估选取最优轨迹对。

核心思想：

- **纵向规划**：在 ST 图（空间-时间图）中生成候选速度曲线（巡航/停车/跟车/超车）
- **横向规划**：在 Frenet 坐标系中生成候选横向偏移曲线（五次多项式）
- **轨迹组合**：纵向 + 横向 1D 轨迹 → 2D 轨迹，逐对评估代价，返回第一个无碰撞可行轨迹

```
PlanningInitPoint → Frenet 转换 → 1D 轨迹生成（纵向×横向）
                                        ↓
                              轨迹评估（代价排序）
                                        ↓
                              轨迹组合 → 约束检查 → 碰撞检查
                                        ↓
                              可行轨迹 / 备选轨迹
```

## 2. 目录结构

```
modules/planning/planners/lattice/
├── lattice_planner.h / .cc                              # 规划器主入口
├── behavior/                                            # 行为决策
│   ├── collision_checker.h / .cc                        # 碰撞检测器
│   ├── feasible_region.h / .cc                          # 可行域计算
│   ├── path_time_graph.h / .cc                          # ST 图构建
│   └── prediction_querier.h / .cc                       # 预测查询器
└── trajectory_generation/                               # 轨迹生成
    ├── backup_trajectory_generator.h / .cc              # 备选轨迹生成器
    ├── end_condition_sampler.h / .cc                    # 终态采样器
    ├── lateral_osqp_optimizer.h / .cc                   # 横向 OSQP 优化器
    ├── lateral_qp_optimizer.h / .cc                     # 横向 QP 优化器
    ├── lattice_trajectory1d.h / .cc                     # 1D 轨迹包装类
    ├── piecewise_braking_trajectory_generator.h / .cc   # 分段制动轨迹
    ├── trajectory1d_generator.h / .cc                   # 1D 轨迹生成器
    ├── trajectory_combiner.h / .cc                      # 轨迹组合器
    └── trajectory_evaluator.h / .cc                     # 轨迹评估器
```

## 3. LatticePlanner 主入口

### 3.1 类声明

```cpp
class LatticePlanner : public PlannerWithReferenceLine {
 public:
  std::string Name() override { return "LATTICE"; }
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame, ADCTrajectory* ptr_computed_trajectory) override;
  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
};
```

通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册为 CyberRT 插件。

### 3.2 `Plan` — 多参考线遍历

```cpp
Status LatticePlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame, ADCTrajectory* ptr_computed_trajectory);
```

- 遍历 `frame->mutable_reference_line_info()` 中的所有参考线
- 第一条参考线优先级代价为 0，后续每条附加 `FLAGS_cost_non_priority_reference_line`
- 对每条参考线调用 `PlanOnReferenceLine`
- 只要有一条成功即返回 OK

### 3.3 `PlanOnReferenceLine` — 核心规划流程

```cpp
Status LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info);
```

**7 步流程**：

#### Step 1：参考线离散化

```cpp
auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(
    ToDiscretizedReferenceLine(reference_line_info->reference_line().reference_points()));
```

- 将 `ReferencePoint` 序列转换为 `PathPoint` 序列
- `ToDiscretizedReferenceLine`：遍历参考点，计算累积弧长 s，填入 x, y, theta, kappa, dkappa

#### Step 2：匹配点计算

```cpp
PathPoint matched_point = PathMatcher::MatchToPath(
    *ptr_reference_line, planning_init_point.path_point().x(),
    planning_init_point.path_point().y());
```

- 在参考线上找到与规划起始点最近的投影点

#### Step 3：Frenet 初始状态

```cpp
std::array<double, 3> init_s;  // [s, ds, dds]
std::array<double, 3> init_d;  // [l, dl, ddl]
ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
```

- `ComputeInitFrenetState`：调用 `CartesianFrenetConverter::cartesian_to_frenet`
- 输出纵向 `[s, ṡ, s̈]` 和横向 `[l, l̇, l̈]` 初始条件

#### Step 4：决策解析与 ST 图构建

```cpp
auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
    frame->obstacles(), ptr_reference_line);
auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
    ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
    reference_line_info, init_s[0],
    init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
    FLAGS_trajectory_time_length, init_d);
```

- 创建 `PredictionQuerier` 管理障碍物预测信息
- 创建 `PathTimeGraph` 构建 ST 图，范围 `[init_s, init_s + horizon] × [0, T]`

#### Step 5：1D 轨迹生成

```cpp
Trajectory1dGenerator trajectory1d_generator(init_s, init_d, ptr_path_time_graph,
                                             ptr_prediction_querier);
trajectory1d_generator.GenerateTrajectoryBundles(
    planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);
```

- 生成纵向候选轨迹（巡航/停车/跟车超车）
- 生成横向候选轨迹（不同横向偏移的五次多项式）

#### Step 6：轨迹评估

```cpp
TrajectoryEvaluator trajectory_evaluator(
    init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
    ptr_path_time_graph, ptr_reference_line);
```

- 为每对 (纵向, 横向) 轨迹计算代价
- 使用优先队列按代价排序

#### Step 7：轨迹组合与验证

```cpp
while (trajectory_evaluator.has_more_trajectory_pairs()) {
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
    auto combined_trajectory = TrajectoryCombiner::Combine(
        *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
        planning_init_point.relative_time());
    // 约束检查
    if (ConstraintChecker::ValidTrajectory(combined_trajectory) != VALID) continue;
    // 碰撞检查
    if (collision_checker.InCollision(combined_trajectory)) continue;
    // 找到可行轨迹，设置并返回
    reference_line_info->SetTrajectory(combined_trajectory);
    break;
}
```

- 从代价最低的轨迹对开始尝试
- 依次做动力学约束检查（速度/加速度/jerk/曲率/横向加速度/横向 jerk）
- 通过约束检查后做碰撞检测
- 第一个通过所有检查的轨迹即为输出

**约束检查失败统计**：

| 约束类型 | 说明 |
|---------|------|
| `LON_VELOCITY_OUT_OF_BOUND` | 纵向速度越界 |
| `LON_ACCELERATION_OUT_OF_BOUND` | 纵向加速度越界 |
| `LON_JERK_OUT_OF_BOUND` | 纵向 jerk 越界 |
| `CURVATURE_OUT_OF_BOUND` | 曲率越界 |
| `LAT_ACCELERATION_OUT_OF_BOUND` | 横向加速度越界 |
| `LAT_JERK_OUT_OF_BOUND` | 横向 jerk 越界 |

**备选轨迹**：若所有候选轨迹均失败且 `FLAGS_enable_backup_trajectory` 为 true，调用 `BackupTrajectoryGenerator` 生成安全停车轨迹。

## 4. 行为模块

### 4.1 PredictionQuerier 预测查询器

```cpp
class PredictionQuerier {
 public:
  PredictionQuerier(const std::vector<const Obstacle*>& obstacles,
                    const std::shared_ptr<std::vector<PathPoint>>& ptr_reference_line);
  std::vector<const Obstacle*> GetObstacles() const;
  double ProjectVelocityAlongReferenceLine(const std::string& obstacle_id,
                                           double s, double t) const;
};
```

- `GetObstacles()`：返回所有障碍物列表
- `ProjectVelocityAlongReferenceLine`：将障碍物速度投影到参考线方向，用于纵向跟车/超车决策
- 内部维护 `id_obstacle_map_` 做 ID 到障碍物的快速查找

### 4.2 FeasibleRegion 可行域

```cpp
class FeasibleRegion {
 public:
  explicit FeasibleRegion(const std::array<double, 3>& init_s);
  double SUpper(double t) const;   // t 时刻最大可达 s
  double SLower(double t) const;   // t 时刻最小可达 s
  double VUpper(double t) const;   // t 时刻最大可达速度
  double VLower(double t) const;   // t 时刻最小可达速度
  double TLower(double s) const;   // 到达 s 的最短时间
};
```

- 基于初始状态 `init_s = [s₀, v₀, a₀]` 和车辆动力学约束计算物理可行域
- `t_at_zero_speed_`：速度降为 0 的时刻（最大减速）
- `s_at_zero_speed_`：速度降为 0 时的位移
- 被 `EndConditionSampler` 用于约束采样范围

### 4.3 PathTimeGraph ST 图

```cpp
class PathTimeGraph {
 public:
  PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                const std::vector<PathPoint>& discretized_ref_points,
                const ReferenceLineInfo* ptr_reference_line_info,
                double s_start, double s_end, double t_start, double t_end,
                const std::array<double, 3>& init_d);
  const std::vector<STBoundary>& GetPathTimeObstacles() const;
  std::vector<std::pair<double, double>> GetPathBlockingIntervals(double t) const;
  std::vector<std::pair<double, double>> GetLateralBounds(
      double s_start, double s_end, double s_resolution);
};
```

**核心方法**：

#### 构造函数

- 调用 `SetupObstacles` 处理所有障碍物
- 区分静态障碍物（`SetStaticObstacle`）和动态障碍物（`SetDynamicObstacle`）

#### `SetStaticObstacle`

- 将障碍物投影到参考线的 SL 坐标系
- 计算 SL 边界，生成 STBoundary（时间段内 s 范围不变）

#### `SetDynamicObstacle`

- 对障碍物在多个时间步做预测轨迹投影
- 计算每个时间步的 SL 边界
- 生成随时间变化的 STBoundary

#### `GetPathBlockingIntervals(double t)`

- 返回时刻 t 所有障碍物在 s 方向上的阻挡区间
- 用于纵向轨迹评估时的碰撞代价计算

#### `GetLateralBounds`

- 沿参考线按 `s_resolution` 离散化
- 对每个 s 处计算左右可行驶宽度（排除障碍物占据区域）
- 用于横向轨迹规划的边界约束

### 4.4 CollisionChecker 碰撞检测器

```cpp
class CollisionChecker {
 public:
  CollisionChecker(const std::vector<const Obstacle*>& obstacles,
                   double ego_vehicle_s, double ego_vehicle_d,
                   const std::vector<PathPoint>& discretized_reference_line,
                   const ReferenceLineInfo* ptr_reference_line_info,
                   const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph);
  bool InCollision(const DiscretizedTrajectory& discretized_trajectory);
  static bool InCollision(const std::vector<const Obstacle*>& obstacles,
                          const DiscretizedTrajectory& ego_trajectory,
                          double ego_length, double ego_width,
                          double ego_edge_to_center);
};
```

**实例方法 `InCollision`**：

1. 使用 `BuildPredictedEnvironment` 预计算所有障碍物在各时间步的 bounding box
2. 对组合轨迹的每个时间点，获取自车 box
3. 检查自车 box 与所有障碍物 box 是否重叠

**静态方法 `InCollision`**：

- 通用碰撞检测，接受自车轨迹和障碍物列表
- 不依赖 PathTimeGraph，可用于任意场景

**私有辅助**：

- `BuildPredictedEnvironment`：预计算 `predicted_bounding_rectangles_[obstacle_idx][time_idx]`
- `IsEgoVehicleInLane`：判断自车是否在车道内
- `IsObstacleBehindEgoVehicle`：判断障碍物是否在自车后方（后方障碍物可忽略）

## 5. 轨迹生成模块

### 5.1 Trajectory1dGenerator 1D 轨迹生成器

```cpp
class Trajectory1dGenerator {
 public:
  Trajectory1dGenerator(
      const std::array<double, 3>& lon_init_state,
      const std::array<double, 3>& lat_init_state,
      std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
      std::shared_ptr<PredictionQuerier> ptr_prediction_querier);
  void GenerateTrajectoryBundles(
      const PlanningTarget& planning_target,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle);
};
```

#### `GenerateTrajectoryBundles`

分别调用纵向和横向轨迹生成：

- `GenerateLongitudinalTrajectoryBundle`：根据 `planning_target` 生成纵向轨迹
  - `GenerateSpeedProfilesForCruising(target_speed)`：巡航模式
  - `GenerateSpeedProfilesForStopping(stop_point)`：停车模式
  - `GenerateSpeedProfilesForPathTimeObstacles()`：跟车/超车模式
- `GenerateLateralTrajectoryBundle`：生成横向轨迹束

#### `GenerateTrajectory1DBundle<N>`（模板特化）

```cpp
template <size_t N>
void GenerateTrajectory1DBundle(
    const std::array<double, 3>& init_state,
    const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
    std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const;
```

- **N=4（四次多项式）**：用于纵向巡航（给定终态速度和加速度，不定位移）
  - `QuarticPolynomialCurve1d(init_state, {v_end, a_end}, t_end)`
  - 设置 `target_velocity` 和 `target_time`

- **N=5（五次多项式）**：用于纵向停车和横向规划（给定终态位置、速度、加速度）
  - `QuinticPolynomialCurve1d(init_state, end_condition, t_end)`
  - 设置 `target_position`、`target_velocity` 和 `target_time`

每条生成的轨迹都包装为 `LatticeTrajectory1d`，携带目标位置/速度/时间信息。

### 5.2 EndConditionSampler 终态采样器

```cpp
class EndConditionSampler {
 public:
  EndConditionSampler(
      const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
      std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
      std::shared_ptr<PredictionQuerier> ptr_prediction_querier);
  std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions() const;
  std::vector<std::pair<std::array<double, 3>, double>>
      SampleLonEndConditionsForCruising(double ref_cruise_speed) const;
  std::vector<std::pair<std::array<double, 3>, double>>
      SampleLonEndConditionsForStopping(double ref_stop_point) const;
  std::vector<std::pair<std::array<double, 3>, double>>
      SampleLonEndConditionsForPathTimePoints() const;
};
```

**采样策略**：

- **横向采样 `SampleLatEndConditions`**：
  - 在不同时间点（0.5s, 1.0s, ..., 8.0s）采样不同横向偏移（-0.5m, 0m, 0.5m 等）
  - 终态速度和加速度设为 0（横向稳定）

- **纵向巡航 `SampleLonEndConditionsForCruising`**：
  - 在不同时间点采样不同速度（0, 0.5×ref, ref, 1.5×ref 等）
  - 使用 `FeasibleRegion` 约束采样范围

- **纵向停车 `SampleLonEndConditionsForStopping`**：
  - 在不同时间点采样接近 stop_point 的位置
  - 终态速度为 0

- **纵向跟车超车 `SampleLonEndConditionsForPathTimePoints`**：
  - 从 `PathTimeGraph` 中查询障碍物周围的采样点
  - `QueryFollowPathTimePoints`：跟车采样（在障碍物后方）
  - `QueryOvertakePathTimePoints`：超车采样（在障碍物前方）

### 5.3 LatticeTrajectory1d 1D 轨迹包装

```cpp
class LatticeTrajectory1d : public Curve1d {
 public:
  explicit LatticeTrajectory1d(std::shared_ptr<Curve1d> ptr_trajectory1d);
  double Evaluate(uint32_t order, double param) const override;
  double ParamLength() const override;
  // target info
  bool has_target_position() const;
  double target_position() const;
  void set_target_position(double target_position);
  // similar for target_velocity, target_time
};
```

- 包装底层 `Curve1d`（四次/五次多项式），增加目标位置/速度/时间元信息
- `Evaluate(order, param)`：委托给内部轨迹，order=0 返回位置，order=1 返回速度，order=2 返回加速度

### 5.4 TrajectoryCombiner 轨迹组合器

```cpp
class TrajectoryCombiner {
 public:
  static DiscretizedTrajectory Combine(
      const std::vector<PathPoint>& reference_line,
      const Curve1d& lon_trajectory, const Curve1d& lat_trajectory,
      double init_relative_time);
};
```

**算法**：

1. 沿时间轴以 `FLAGS_trajectory_time_resolution`（0.1s）步进
2. 对每个时间 t：
   - 纵向：`s = lon_trajectory.Evaluate(0, t)`，`s_dot = Evaluate(1, t)`，`s_ddot = Evaluate(2, t)`
   - 横向：`l = lat_trajectory.Evaluate(0, s)`，`l_dot = Evaluate(1, s)`，`l_ddl = Evaluate(2, s)`
   - 在参考线上查找 s 处的匹配点
   - 使用 `CartesianFrenetConverter::frenet_to_cartesian` 将 Frenet → Cartesian
   - 计算曲率：`kappa = (ddl + kappa_ref * cos(d_theta)) / cos(d_theta)`（近似）
3. 组装为 `TrajectoryPoint` 序列

### 5.5 PiecewiseBrakingTrajectoryGenerator 分段制动轨迹

```cpp
class PiecewiseBrakingTrajectoryGenerator {
 public:
  static std::shared_ptr<Curve1d> Generate(
      double s_target, double s_curr, double v_target,
      double v_curr, double a_comfort, double d_comfort, double max_time);
  static double ComputeStopDistance(double v, double dec);
  static double ComputeStopDeceleration(double dist, double v);
};
```

- 生成从当前状态 `(s_curr, v_curr)` 减速到目标状态 `(s_target, v_target)` 的分段加速度轨迹
- 使用 `PiecewiseAccelerationTrajectory1d` 表示
- 用于停车场景和跟车场景的候选轨迹

### 5.6 BackupTrajectoryGenerator 备选轨迹

```cpp
class BackupTrajectoryGenerator {
 public:
  BackupTrajectoryGenerator(
      const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
      double init_relative_time,
      const std::shared_ptr<CollisionChecker>& ptr_collision_checker,
      const Trajectory1dGenerator* trajectory1d_generator);
  DiscretizedTrajectory GenerateTrajectory(
      const std::vector<PathPoint>& discretized_ref_points);
};
```

- 当所有候选轨迹均失败时的**安全回退方案**
- `GenerateTrajectory1dPairs`：生成简单的减速轨迹对
  - 纵向：使用 `ConstantDecelerationTrajectory1d` 做恒减速
  - 横向：使用 `QuinticPolynomialCurve1d` 保持当前横向位置
- 按末端速度排序（优先选择能保留更多速度的轨迹，避免急刹）
- 逐对检查碰撞，返回第一个无碰撞的备选轨迹

## 6. 轨迹评估模块

### 6.1 TrajectoryEvaluator 轨迹评估器

```cpp
class TrajectoryEvaluator {
 public:
  TrajectoryEvaluator(
      const std::array<double, 3>& init_s,
      const PlanningTarget& planning_target,
      const std::vector<std::shared_ptr<Curve1d>>& lon_trajectories,
      const std::vector<std::shared_ptr<Curve1d>>& lat_trajectories,
      std::shared_ptr<PathTimeGraph> path_time_graph,
      std::shared_ptr<std::vector<PathPoint>> reference_line);
  bool has_more_trajectory_pairs() const;
  size_t num_of_trajectory_pairs() const;
  std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>> next_top_trajectory_pair();
  double top_trajectory_pair_cost() const;
};
```

**构造过程**：

1. 先用 `ConstraintChecker` 过滤不满足 1D 约束的轨迹（不进入组合评估）
2. 计算纵向引导速度 `ComputeLongitudinalGuideVelocity`
3. 对每对 (lon, lat) 轨迹计算代价 `Evaluate`
4. 按代价插入优先队列 `cost_queue_`

#### `Evaluate` — 代价函数

总代价 = 纵向代价 + 横向代价，包含以下分量：

| 代价分量 | 方法 | 说明 |
|---------|------|------|
| 横向偏移 | `LatOffsetCost` | 沿引导速度采样 s 值处的横向偏移平方和 |
| 横向舒适性 | `LatComfortCost` | 横向 jerk 的积分 |
| 纵向舒适性 | `LonComfortCost` | 纵向 jerk 的积分 |
| 纵向碰撞 | `LonCollisionCost` | 与 ST 图中障碍物的时间接近程度 |
| 纵向目标 | `LonObjectiveCost` | 与引导速度的偏差 |
| 向心加速度 | `CentripetalAccelerationCost` | 曲率相关的向心加速度 |

#### `ComputeLongitudinalGuideVelocity`

- 根据 `PlanningTarget`（巡航/停车）计算参考速度曲线
- 巡航模式：趋向目标速度
- 停车模式：趋向减速到 0

#### 优先队列输出

- `CostComparator`：按代价升序排列
- `has_more_trajectory_pairs()`：队列非空
- `next_top_trajectory_pair()`：弹出代价最低的轨迹对
- 供 `PlanOnReferenceLine` 的 Step 7 逐对尝试

## 7. 关键设计决策

### 7.1 纵横解耦

- 降低搜索空间维度：2D 轨迹 → 1D + 1D
- 纵向使用四次多项式（4 个约束：初态 s, v, a + 终态 v, a）
- 横向使用五次多项式（6 个约束：初态 l, l̇, l̈ + 终态 l, l̇, l̈）

### 7.2 贪心搜索策略

- 不穷举所有组合，而是按代价排序逐对尝试
- 返回第一个满足约束且无碰撞的轨迹
- 大幅减少计算量，满足 100ms 实时性要求

### 7.3 多层安全网

1. **1D 约束预过滤**：不满足动力学约束的 1D 轨迹不进入组合评估
2. **2D 约束检查**：组合后检查速度/加速度/jerk/曲率
3. **碰撞检查**：使用预计算的障碍物 bounding box
4. **备选轨迹**：所有候选失败时的安全停车轨迹

### 7.4 局限性

- 适用于结构化道路（高速公路、城市主干道）
- 对非结构化场景（停车场、越野）支持有限
- 纵横解耦假设在极端工况下可能失效（如高速急转弯）