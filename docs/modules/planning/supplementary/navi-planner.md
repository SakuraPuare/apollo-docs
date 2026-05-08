# Navi Planner 导航规划器函数级源码解析

本文聚焦 `modules/planning/planners/navi/` 目录，按函数级粒度拆解导航模式规划器的完整实现：`NaviPlanner` 主入口、`NaviPathDecider` 路径决策、`NaviSpeedDecider` 速度决策、`NaviObstacleDecider` 障碍物决策，以及 `NaviSpeedTsGraph` 速度约束图。

## 1. 模块定位

Navi Planner 是 Apollo 导航模式（`FLAGS_use_navigation_mode = true`）下的专用规划器。与标准模式（Lattice/PublicRoad）使用高精地图不同，导航模式基于**实时相对地图（Relative Map）**，使用车辆 FLU（Front-Left-Up）坐标系完成巡航、跟车、超车、避让、变道和停车等任务。

核心特点：

- 仅在导航模式下启用，标准模式下初始化即失败
- 基于 `relative_map` 提供的导航路径生成参考线
- 任务通过 `NaviTask` 工厂模式注册，支持插件化扩展
- 路径和速度分别由 `NaviPathDecider` 和 `NaviSpeedDecider` 独立决策
- 障碍物避让由 `NaviObstacleDecider` 统一处理

```
RelativeMap ──> ReferenceLineProvider ──> NaviPlanner
                                            │
                      ┌─────────────────────┼─────────────────────┐
                      ▼                     ▼                     ▼
              NaviPathDecider        NaviSpeedDecider      NaviObstacleDecider
              (路径生成)             (速度决策)            (障碍物避让)
                      │                     │
                      └──────────┬──────────┘
                                 ▼
                          CombinePathAndSpeed
                                 ▼
                         DiscretizedTrajectory
```

## 2. 目录结构

```
modules/planning/planners/navi/
├── navi_planner.h / .cc                    # 规划器主入口
├── navi_planner_test.cc                    # 单元测试
├── BUILD                                   # Bazel 构建规则
├── conf/                                   # 运行配置
├── decider/                                # 决策器
│   ├── navi_task.h / .cc                   # 决策器基类
│   ├── navi_path_decider.h / .cc           # 路径决策
│   ├── navi_path_decider_test.cc
│   ├── navi_speed_decider.h / .cc          # 速度决策
│   ├── navi_speed_decider_test.cc
│   ├── navi_speed_ts_graph.h / .cc         # 速度约束图
│   ├── navi_speed_ts_graph_test.cc
│   ├── navi_obstacle_decider.h / .cc       # 障碍物决策
│   └── navi_obstacle_decider_test.cc
└── proto/                                  # Protobuf 配置定义
```

## 3. NaviPlanner 主入口

### 3.1 类声明

```cpp
class NaviPlanner : public PlannerWithReferenceLine {
 public:
  std::string Name() override { return "NAVI"; }
  common::Status Init(const std::shared_ptr<DependencyInjector>& injector,
                      const std::string& config_path = "") override;
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame, ADCTrajectory* ptr_computed_trajectory) override;
  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
 private:
  common::util::Factory<NaviTaskType, NaviTask> task_factory_;
  std::vector<std::unique_ptr<NaviTask>> tasks_;
};
```

通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册为 CyberRT 插件。

### 3.2 `Init` — 初始化

```cpp
Status NaviPlanner::Init(const std::shared_ptr<DependencyInjector>& injector,
                         const std::string& config_path);
```

1. 校验 `FLAGS_use_navigation_mode`，非导航模式直接返回错误
2. `RegisterTasks()`：在工厂中注册 `NAVI_PATH_DECIDER` 和 `NAVI_SPEED_DECIDER`
3. 从配置文件 `PlannerNaviConfig` 加载任务列表
4. 按配置顺序创建任务实例并调用 `task->Init(planner_conf)`

### 3.3 `Plan` — 多参考线遍历

```cpp
Status NaviPlanner::Plan(const TrajectoryPoint& planning_init_point,
                         Frame* frame, ADCTrajectory* ptr_computed_trajectory);
```

- 遍历所有参考线，按 `priority` 设置代价（`priority * kStraightForwardLineCost`）
- **仅对目标车道（`priority == KDestLanePriority = 0`）执行规划**
- 非目标车道直接标记 `SetDrivable(false)` 并跳过
- 只要有一条参考线规划成功即返回 OK

### 3.4 `PlanOnReferenceLine` — 核心规划流程

```cpp
Status NaviPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info);
```

**流程**：

#### Step 1：初始速度剖面生成

```cpp
auto speed_profile = GenerateInitSpeedProfile(planning_init_point, reference_line_info);
if (speed_profile.empty()) {
  speed_profile = GenerateSpeedHotStart(planning_init_point);
}
*heuristic_speed_data = SpeedData(speed_profile);
```

- 优先从上一帧的速度剖面继承（`GenerateInitSpeedProfile`）
- 若无历史数据，使用热启动（`GenerateSpeedHotStart`）：以当前速度匀速行驶

#### Step 2：串行执行任务

```cpp
for (auto& task : tasks_) {
  ret = task->Execute(frame, reference_line_info);
  if (!ret.ok()) break;
  RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);
}
```

- 按配置顺序执行 `NaviPathDecider` → `NaviSpeedDecider`
- 记录每个任务的执行耗时

#### Step 3：回退策略

```cpp
if (reference_line_info->path_data().Empty()) {
  GenerateFallbackPathProfile(reference_line_info, reference_line_info->mutable_path_data());
  reference_line_info->AddCost(kPathOptimizationFallbackClost);  // 2e4
}
if (!ret.ok() || reference_line_info->speed_data().empty()) {
  GenerateFallbackSpeedProfile(reference_line_info->mutable_speed_data());
  reference_line_info->AddCost(kSpeedOptimizationFallbackClost);  // 2e4
}
```

- 路径为空时生成沿参考线的回退路径
- 速度为空时生成减速停车的回退速度

#### Step 4：轨迹组合与校验

```cpp
reference_line_info->CombinePathAndSpeedProfile(
    planning_init_point.relative_time(), planning_init_point.path_point().s(), &trajectory);
```

- 将路径和速度组合为 `DiscretizedTrajectory`
- 若 `FLAGS_enable_trajectory_check`，校验轨迹动力学约束
- 为静态停车障碍物附加额外代价

### 3.5 `GenerateInitSpeedProfile` — 历史速度继承

```cpp
std::vector<SpeedPoint> NaviPlanner::GenerateInitSpeedProfile(
    const TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info);
```

1. 获取上一帧的 `DriveReferenceLineInfo`
2. 检查当前参考线是否与上一帧驾驶参考线连续（`IsStartFrom`）
3. 计算自车在上一帧参考线上的 SL 坐标差 `s_diff`
4. 从上一帧速度剖面中截取 `s >= s_diff` 的部分，重新标定 s 和 t 的起点

### 3.6 `GenerateSpeedHotStart` — 热启动

```cpp
std::vector<SpeedPoint> NaviPlanner::GenerateSpeedHotStart(
    const TrajectoryPoint& planning_init_point);
```

- 以当前速度（clamp 到 `[5.0, planning_upper_speed_limit]`）匀速生成速度剖面
- 时间步长 `FLAGS_trajectory_time_min_interval`，总时长 `FLAGS_trajectory_time_length`

### 3.7 回退路径/速度生成

#### `GenerateFallbackPathProfile`

- 沿参考线从 `adc_s` 到 150m 以 1m 步长采样
- 在参考点基础上偏移自车相对参考线的横向偏移 `(dx, dy)`

#### `GenerateFallbackSpeedProfile`

- 优先使用五次多项式减速（`GenerateStopProfileFromPolynomial`）
- 失败时使用分段恒减速（`GenerateStopProfile`）

#### `GenerateStopProfileFromPolynomial`

- 网格搜索：时间 `[2.0, 4.0]` 步长 0.5s，距离 `[0, 50]` 步长 1m
- 使用 `QuinticPolynomialCurve1d(0, v₀, a₀, s_target, 0, 0, t)` 构造减速曲线
- `IsValidProfile` 校验：速度 ≥ 0 且加速度 ≥ -5.0

#### `GenerateStopProfile`

- 两阶段恒 jerk 减速：
  - Phase 1（`t ≤ t_mid`）：jerk = -1.0 m/s³，加速度从 `init_acc` 线性减小到 `FLAGS_slowdown_profile_deceleration`
  - Phase 2（`t > t_mid`）：以 `FLAGS_slowdown_profile_deceleration` 恒减速至停车

## 4. NaviTask 决策器基类

```cpp
class NaviTask {
 public:
  explicit NaviTask(const std::string& name);
  virtual const std::string& Name() const;
  virtual common::Status Execute(Frame* frame, ReferenceLineInfo* reference_line_info);
  virtual bool Init(const PlannerNaviConfig& config);
 protected:
  bool is_init_ = false;
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;
};
```

- 所有导航决策器的基类
- `Execute` 保存 `frame` 和 `reference_line_info` 到成员变量供子类使用
- 工厂模式：`NaviTaskType` 枚举 → `NaviTask` 实例

## 5. NaviPathDecider 路径决策

### 5.1 类声明

```cpp
class NaviPathDecider : public NaviTask {
 public:
  bool Init(const PlannerNaviConfig& config) override;
  common::Status Execute(Frame* frame, ReferenceLineInfo* reference_line_info) override;
 private:
  common::Status Process(const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         const std::vector<const Obstacle*>& obstacles,
                         PathDecision* path_decision, PathData* path_data);
  bool GetBasicPathData(const ReferenceLine& reference_line,
                        std::vector<common::PathPoint>* path_points);
  void MoveToDestLane(double dest_ref_line_y,
                      std::vector<common::PathPoint>* path_points);
  void KeepLane(double dest_ref_line_y,
                std::vector<common::PathPoint>* path_points);
  bool IsSafeChangeLane(const ReferenceLine& reference_line,
                        const PathDecision& path_decision);
  double NudgeProcess(const ReferenceLine& reference_line,
                      const std::vector<common::PathPoint>& path_data_points,
                      const std::vector<const Obstacle*>& obstacles,
                      const PathDecision& path_decision,
                      const common::VehicleState& vehicle_state);
  double CalculateDistanceToDestLane();
};
```

### 5.2 `Execute` — 主流程

1. 获取参考线、起始点、障碍物列表
2. 调用 `Process` 生成路径

### 5.3 `Process` — 路径生成

1. `GetBasicPathData`：从参考线截取基本路径点序列
2. 判断自车是否在目标车道：
   - **不在目标车道** → `MoveToDestLane`：逐步向目标车道横向平移
   - **在目标车道** → `KeepLane`：保持车道中心
3. `NudgeProcess`：计算障碍物避让的横向偏移量
4. `IsSafeChangeLane`：变道前安全检查

### 5.4 `GetBasicPathData`

- 从参考线起始位置按步长截取路径点
- 包含 x, y, theta, kappa, dkappa, s

### 5.5 `MoveToDestLane` / `KeepLane`

- `MoveToDestLane`：根据配置表 `move_dest_lane_config_talbe_`（速度→偏移量映射）计算横向偏移
- `KeepLane`：当横向偏差 < `max_keep_lane_distance_` 时保持，否则微调

### 5.6 `IsSafeChangeLane`

- 检查变道目标车道是否有足够安全空间
- 查询 `PathDecision` 中的障碍物决策

### 5.7 `NudgeProcess`

- 委托 `NaviObstacleDecider::GetNudgeDistance` 计算避让距离
- 将避让距离应用到路径点的横向偏移上

## 6. NaviObstacleDecider 障碍物决策

### 6.1 核心职责

- 分析路径周围的障碍物分布
- 计算安全的横向避让距离（nudge distance）
- 标记不安全障碍物信息

### 6.2 `GetNudgeDistance` — 核心接口

```cpp
double GetNudgeDistance(
    const std::vector<const Obstacle*>& obstacles,
    const ReferenceLine& reference_line,
    const PathDecision& path_decision,
    const std::vector<common::PathPoint>& path_data_points,
    const common::VehicleState& vehicle_state,
    int* lane_obstacles_num);
```

**算法**：

1. `GetMinLaneWidth`：获取路径范围内的最小车道宽度
2. `ProcessObstacle`：遍历障碍物
   - `IsNeedFilterObstacle`：过滤安全障碍物（距离远、不在路径范围内）
   - `AddObstacleOffsetDirection`：计算障碍物相对路径的横向偏移方向（左正右负）
   - 填充 `obstacle_lat_dist_`（横向距离 → 纵向位置映射）
3. `GetObstacleActualOffsetDistance`：根据障碍物位置计算实际避让偏移
4. `SmoothNudgeDistance`：平滑避让距离，消除传感器噪声抖动
5. `KeepNudgePosition`：保持避让位置的稳定性

### 6.3 `GetUnsafeObstaclesInfo`

```cpp
void GetUnsafeObstaclesInfo(
    const std::vector<common::PathPoint>& path_data_points,
    const std::vector<const Obstacle*>& obstacles);
```

- 检测轨迹与参考线之间是否存在不安全障碍物
- 结果存入 `unsafe_obstacle_info_`（ID、横向距离、纵向距离）

### 6.4 状态管理

- `last_nudge_dist_`：上一帧避让距离（用于平滑）
- `no_nudge_num_`：连续无避让帧数
- `is_obstacle_stable_`：障碍物是否稳定
- `keep_nudge_flag_`：是否保持避让
- `eliminate_clutter_num_`：消除杂波计数

## 7. NaviSpeedDecider 速度决策

### 7.1 核心职责

- 基于 TS 图（时间-空间图）生成速度曲线
- 考虑障碍物约束、向心加速度约束、配置约束

### 7.2 `Execute` — 主流程

1. 获取起始速度、加速度、jerk
2. 初始化 `NaviSpeedTsGraph`（TS 约束图）
3. 依次添加约束：
   - `AddPerceptionRangeConstraints`：感知范围约束
   - `AddObstaclesConstraints`：障碍物约束
   - `AddTrafficDecisionConstraints`：交通决策约束
   - `AddCentricAccelerationConstraints`：向心加速度约束（弯道限速）
   - `AddConfiguredConstraints`：配置约束（最大速度、加速度等）
4. `ts_graph_.Solve()` 求解速度曲线
5. 调用 `MakeSpeedDecision` 生成最终 `SpeedData`

### 7.3 约束类型

| 约束方法 | 说明 |
|---------|------|
| `AddPerceptionRangeConstraints` | 感知范围外的区域限制最大速度 |
| `AddObstaclesConstraints` | 根据障碍物距离、速度设置跟车/停车约束 |
| `AddTrafficDecisionConstraints` | 停车标志、红绿灯等交通决策约束 |
| `AddCentricAccelerationConstraints` | 弯道处限制速度以满足向心加速度上限 |
| `AddConfiguredConstraints` | 全局配置的最大速度/加速度/jerk 约束 |

### 7.4 向心加速度约束

- 计算路径点处的曲率 `kappa`
- `kappa > kappa_threshold_` 时触发限速
- 限速公式：`v_max = sqrt(a_centric / |kappa|)`
- `soft_centric_accel_limit_`：软约束（限速）
- `hard_centric_accel_limit_`：硬约束（强制约束）

## 8. NaviSpeedTsGraph 速度约束图

### 8.1 数据结构

```cpp
struct NaviSpeedTsConstraints {
  double t_min = 0.0;           // 最早到达时间
  double v_max = MAX;           // 最大速度
  double v_preffered = MAX;     // 推荐速度
  double a_max = MAX;           // 最大加速度
  double a_preffered = MAX;     // 推荐加速度
  double b_max = MAX;           // 最大减速度
  double b_preffered = MAX;     // 推荐减速度
};

struct NaviSpeedTsPoint {
  double s, t, v, a, da;        // 空间、时间、速度、加速度、jerk
};
```

### 8.2 `NaviSpeedTsGraph` 类

```cpp
class NaviSpeedTsGraph {
 public:
  void Reset(double s_step, double s_max, double start_v, double start_a, double start_da);
  void UpdateConstraints(const NaviSpeedTsConstraints& constraints);
  void UpdateRangeConstraints(double start_s, double end_s,
                              const NaviSpeedTsConstraints& constraints);
  void UpdateObstacleConstraints(double distance, double safe_distance,
                                 double following_accel_ratio, double v,
                                 double cruise_speed);
  common::Status Solve(std::vector<NaviSpeedTsPoint>* points);
};
```

**核心方法**：

#### `Reset`

- 按 `s_step` 步长在 `[0, s_max]` 范围内创建约束点
- 设置起始点的速度、加速度、jerk

#### `UpdateConstraints`

- 为所有约束点设置全局约束

#### `UpdateRangeConstraints`

- 为 `[start_s, end_s]` 范围内的约束点设置局部约束
- 取已有约束和新约束的最小值（更严格）

#### `UpdateObstacleConstraints`

- 根据障碍物距离和安全距离计算跟车约束
- `following_accel_ratio` 控制跟车时的加速度限制级别

#### `Solve`

- 从起始点开始，逐点前向求解
- 每个点的速度受 `v_max`、加速度受 `a_max`/`b_max` 约束
- 输出满足所有约束的 `(s, t, v, a, da)` 序列

## 9. 与标准规划器的对比

| 特性 | NaviPlanner | PublicRoad/Lattice |
|------|------------|-------------------|
| 地图依赖 | 实时相对地图 | 高精地图 |
| 坐标系 | FLU（车辆坐标系） | Frenet（参考线坐标系） |
| 规划策略 | 路径+速度分离决策 | 场景-阶段-任务流水线 |
| 适用场景 | 导航模式、无高精地图区域 | 标准自动驾驶 |
| 障碍物处理 | NaviObstacleDecider 统一避让 | 各 Task 独立决策 |
| 速度规划 | TS 约束图 + 网格搜索 | Lattice / PiecewiseJerk |
| 变道决策 | 基于车道优先级 | 基于场景机 |

## 10. 关键设计决策

### 10.1 任务工厂模式

- `NaviTaskType` 枚举 → `NaviTask` 工厂注册
- 支持通过配置文件灵活组合任务顺序
- 目前注册了 `NAVI_PATH_DECIDER` 和 `NAVI_SPEED_DECIDER`

### 10.2 多层回退策略

1. **路径回退**：沿参考线生成直行路径（代价 2e4）
2. **速度回退**：五次多项式减速 → 分段恒减速（代价 2e4）
3. **静态障碍物代价**：每遇到一个静态停车障碍物附加 1e3 代价

### 10.3 速度继承与热启动

- 优先从上一帧继承速度剖面（平滑过渡）
- 无历史数据时使用当前速度匀速热启动
- 避免每帧从零开始规划导致的速度抖动

### 10.4 障碍物避让平滑

- 使用 `SmoothNudgeDistance` 消除传感器噪声
- `KeepNudgePosition` 保持避让位置稳定
- `eliminate_clutter_num_` 计数器过滤杂波信号
- 多帧统计 `statist_count_` 判断障碍物稳定性