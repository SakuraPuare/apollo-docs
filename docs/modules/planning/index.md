# Planning 规划模块

> Apollo 自动驾驶规划模块负责根据感知、预测、定位和路由信息，生成安全、舒适、可执行的行驶轨迹。

## 模块职责

Planning 模块是 Apollo 自动驾驶系统的核心决策与规划层，其主要职责包括：

- 接收上游模块（感知、预测、定位、路由）的数据，构建当前帧的规划环境
- 基于高精地图和路由结果生成参考线（Reference Line）
- 根据当前驾驶场景选择合适的 Scenario 进行决策
- 在参考线上执行路径规划（Path Planning）和速度规划（Speed Planning）
- 将路径和速度合成为时空轨迹（Trajectory），发布给下游控制模块执行

## 目录结构

```
modules/planning/
├── planning_component/       # 组件入口，CyberRT Component 实现
├── planning_base/            # 基础库：参考线、数学工具、公共数据结构、proto 定义
├── planning_interface_base/  # 接口层：Planner、Scenario、Stage、Task 基类
├── planners/                 # Planner 实现（PublicRoad、Lattice、Navi、RTK）
├── scenarios/                # 各驾驶场景实现（lane_follow、traffic_light 等）
├── tasks/                    # 各规划任务实现（路径生成、速度优化、决策器等）
├── traffic_rules/            # 交通规则（红绿灯、人行横道、停车标志等）
├── planning_open_space/      # 开放空间规划（泊车等）
├── pnc_map/                  # PnC 地图（从路由结果提取可行驶区域）
└── park_data_center/         # 泊车数据中心
```

## 核心类与接口

### 组件入口

`PlanningComponent`（`planning_component/planning_component.h`）是 CyberRT 的 Component，接收三路触发消息：

- `prediction::PredictionObstacles` — 预测障碍物
- `canbus::Chassis` — 底盘状态
- `localization::LocalizationEstimate` — 定位信息

它在 `Proc()` 中将所有输入汇聚到 `LocalView` 结构体，然后调用 `PlanningBase::RunOnce()` 完成一帧规划，最终将 `ADCTrajectory` 发布到 `/apollo/planning` 话题。

```
PlanningComponent::Proc()
  → 汇聚 LocalView（prediction, chassis, localization, traffic_light, planning_command ...）
  → PlanningBase::RunOnce(local_view, &adc_trajectory)
  → planning_writer_->Write(adc_trajectory)
```

### PlanningBase 与 OnLanePlanning

`PlanningBase`（`planning_component/planning_base.h`）是规划的抽象基类，定义了 `RunOnce()` 和 `Plan()` 纯虚接口。

`OnLanePlanning`（`planning_component/on_lane_planning.h`）是结构化道路场景下的主要实现，其核心流程：

1. 更新车辆状态，对齐时间戳
2. 计算轨迹拼接点（Trajectory Stitching）
3. 更新 `ReferenceLineProvider`，获取参考线
4. 初始化 `Frame`（包含参考线、障碍物等一帧数据）
5. 执行交通规则决策（`TrafficDecider`）
6. 调用 `Planner::Plan()` 进行规划
7. 合成最终可发布轨迹

### Planner 接口

`Planner`（`planning_interface_base/planner_base/planner.h`）是规划器基类：

```cpp
class Planner {
  virtual Status Plan(const TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) = 0;
};

class PlannerWithReferenceLine : public Planner {
  virtual Status PlanOnReferenceLine(const TrajectoryPoint& planning_init_point,
                                     Frame* frame,
                                     ReferenceLineInfo* reference_line_info);
};
```

系统提供四种 Planner 实现：

| Planner | 路径 | 说明 |
|---------|------|------|
| `PublicRoadPlanner` | `planners/public_road/` | 公共道路规划器，基于 Scenario 的主力规划器 |
| `LatticePlanner` | `planners/lattice/` | Lattice 采样规划器 |
| `NaviPlanner` | `planners/navi/` | 导航模式规划器 |
| `RTKReplayPlanner` | `planners/rtk/` | RTK 轨迹回放规划器 |

### DependencyInjector

`DependencyInjector`（`planning_base/common/dependency_injector.h`）是全局依赖注入容器，持有规划过程中的共享状态：

- `PlanningContext` — 跨帧持久化的规划上下文
- `FrameHistory` — 历史帧缓存
- `History` — 历史轨迹记录
- `EgoInfo` — 自车信息
- `VehicleStateProvider` — 车辆状态提供者
- `LearningBasedData` — 学习模型数据

### Frame 与 LocalView

`Frame`（`planning_base/common/frame.h`）封装了一帧规划所需的全部数据：参考线信息列表（`ReferenceLineInfo`）、障碍物列表、车辆状态、开放空间信息等。

`LocalView`（`planning_base/common/local_view.h`）是输入数据的聚合结构：

```cpp
struct LocalView {
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  std::shared_ptr<PlanningCommand> planning_command;
  std::shared_ptr<storytelling::Stories> stories;
  // ...
};
```

## 算法概述

### Scenario-based Planning 框架

Apollo Planning 采用基于场景的分层规划架构，核心层次为 **Scenario → Stage → Task**。

#### 层次结构

```
PublicRoadPlanner
  └── ScenarioManager          # 场景管理器，负责场景切换
        └── Scenario            # 当前活跃场景
              └── Stage         # 当前阶段
                    └── Task[]  # 任务流水线（顺序执行）
```

#### Scenario（场景）

`Scenario`（`planning_interface_base/scenario_base/scenario.h`）是场景基类。每个场景代表一种特定的驾驶情境。系统通过 `ScenarioManager` 管理场景切换：

- `ScenarioManager::Update()` 遍历所有已注册场景，调用 `IsTransferable()` 判断是否应切换
- 当前场景处于 `STATUS_PROCESSING` 时具有更高优先级，不会被抢占
- 场景切换时依次调用 `Exit()` → `Reset()` → `Enter()`
- 默认场景为 `LANE_FOLLOW`（车道跟随）

系统注册的场景列表（按优先级排序，定义在 `conf/public_road_planner_config.pb.txt`）：

| 场景 | 类名 | 说明 |
|------|------|------|
| EMERGENCY_PULL_OVER | `EmergencyPullOverScenario` | 紧急靠边停车 |
| EMERGENCY_STOP | `EmergencyStopScenario` | 紧急停车 |
| VALET_PARKING | `ValetParkingScenario` | 代客泊车 |
| BARE_INTERSECTION_UNPROTECTED | `BareIntersectionUnprotectedScenario` | 无保护裸交叉口 |
| STOP_SIGN_UNPROTECTED | `StopSignUnprotectedScenario` | 无保护停车标志 |
| YIELD_SIGN | `YieldSignScenario` | 让行标志 |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN | `TrafficLightUnprotectedLeftTurnScenario` | 无保护左转 |
| TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN | `TrafficLightUnprotectedRightTurnScenario` | 无保护右转 |
| TRAFFIC_LIGHT_PROTECTED | `TrafficLightProtectedScenario` | 有保护信号灯 |
| PULL_OVER | `PullOverScenario` | 靠边停车 |
| PARK_AND_GO | `ParkAndGoScenario` | 停车起步 |
| LANE_FOLLOW | `LaneFollowScenario` | 车道跟随（默认） |

#### Stage（阶段）

`Stage`（`planning_interface_base/scenario_base/stage.h`）是阶段基类。一个 Scenario 可包含多个 Stage，按 pipeline 配置顺序执行。Stage 的 `Process()` 返回状态决定阶段转换：

- `RUNNING` — 继续执行当前 Stage
- `FINISHED` — 切换到 `NextStage()`，若为空则场景完成
- `ERROR` — 场景异常

Stage 提供三种任务执行模式：

- `ExecuteTaskOnReferenceLine()` — 在参考线上依次执行 Task 列表
- `ExecuteTaskOnOpenSpace()` — 在开放空间中执行 Task 列表
- `ExecuteTaskOnReferenceLineForOnlineLearning()` — 在线学习模式

#### Task（任务）

`Task`（`planning_interface_base/task_base/task.h`）是任务基类，提供 `Execute(Frame*, ReferenceLineInfo*)` 接口。Task 的主要子类型：

- `PathGeneration` — 路径生成基类（`task_base/common/path_generation.h`）
- `SpeedOptimizer` — 速度优化基类（`task_base/common/speed_optimizer.h`）
- `Decider` — 决策器基类（`task_base/common/decider.h`）
- `TrajectoryFallbackTask` — 轨迹降级基类

以 `LANE_FOLLOW` 场景为例，其 Task 流水线（定义在 `scenarios/lane_follow/conf/pipeline.pb.txt`）：

```
LANE_FOLLOW_STAGE:
  1. LANE_CHANGE_PATH        — 换道路径生成
  2. LANE_FOLLOW_PATH         — 车道跟随路径生成
  3. LANE_BORROW_PATH         — 借道路径生成
  4. FALLBACK_PATH            — 降级路径生成
  5. PATH_DECIDER             — 路径决策（对障碍物做 nudge/ignore 决策）
  6. RULE_BASED_STOP_DECIDER  — 基于规则的停车决策
  7. SPEED_BOUNDS_PRIORI_DECIDER — 速度边界先验决策
  8. SPEED_HEURISTIC_OPTIMIZER  — 速度启发式优化（DP 搜索）
  9. SPEED_DECIDER            — 速度决策
  10. SPEED_BOUNDS_FINAL_DECIDER — 速度边界最终决策
  11. PIECEWISE_JERK_SPEED     — 分段 Jerk 速度优化（QP 求解）
```

### Reference Line（参考线）

参考线是规划的几何基础，代表车辆应当跟随的道路中心线。

#### 生成流程

`ReferenceLineProvider`（`planning_base/reference_line/reference_line_provider.h`）负责参考线的生成和维护：

1. 从 `PlanningCommand`（路由结果）中获取路由信息
2. 通过 `PncMapBase`（默认 `LaneFollowMap`）将路由转换为 `RouteSegments`
3. 从 `RouteSegments` 提取原始参考线点
4. 对原始参考线进行平滑处理
5. 缓存并提供给规划主流程使用

#### ReferenceLine 类

`ReferenceLine`（`planning_base/reference_line/reference_line.h`）提供参考线的核心功能：

- Frenet 坐标系转换：`XYToSL()` / `SLToXY()` / `GetFrenetPoint()`
- 参考点查询：`GetReferencePoint(s)` / `GetNearestReferencePoint()`
- 车道信息查询：`GetLaneWidth()` / `GetRoadWidth()` / `GetSpeedLimitFromS()`
- SL 边界计算：`GetSLBoundary()`
- 参考线拼接：`Stitch()` — 将新旧参考线拼接以保持连续性
- 参考线裁剪：`Segment()` — 根据车辆位置裁剪前后视距

#### 参考线平滑

`ReferenceLineSmoother`（`planning_base/reference_line/reference_line_smoother.h`）是平滑器基类，提供三种实现：

| 平滑器 | 类名 | 算法 |
|--------|------|------|
| 离散点平滑 | `DiscretePointsReferenceLineSmoother` | FEM 位置偏差平滑 / CosTheta 平滑 |
| QP 样条平滑 | `QpSplineReferenceLineSmoother` | 二次规划样条拟合 |
| 螺旋线平滑 | `SpiralReferenceLineSmoother` | 螺旋线优化（IPOPT 求解） |

默认使用离散点平滑器的 FEM 位置偏差平滑方法（`FEM_POS_DEVIATION_SMOOTHING`），通过最小化参考点偏差和曲率变化来生成平滑的参考线。

### Path Optimizer（路径优化）

路径规划在 Frenet 坐标系（SL 坐标系）下进行，核心流程为"边界计算 → 路径优化 → 路径评估"。

以 `LaneFollowPath`（`tasks/lane_follow_path/lane_follow_path.h`）为例：

```
LaneFollowPath::Process()
  → DecidePathBounds()     # 计算路径边界（考虑车道边界、静态障碍物）
  → OptimizePath()         # 在边界内优化路径（Piecewise Jerk Path Optimizer）
  → AssessPath()           # 评估候选路径，选择最优
```

路径优化使用 Piecewise Jerk 方法，将路径优化建模为二次规划（QP）问题：

- 优化变量：沿参考线的横向偏移 l(s)
- 目标函数：最小化横向偏移、横向速度、横向加速度、横向 jerk
- 约束条件：路径边界约束、曲率约束、起点状态约束

系统提供多种路径生成 Task：

| Task | 说明 |
|------|------|
| `LaneFollowPath` | 车道跟随路径 |
| `LaneChangePath` | 换道路径 |
| `LaneBorrowPath` | 借道路径 |
| `FallbackPath` | 降级路径 |
| `PullOverPath` | 靠边停车路径 |
| `ReusePath` | 路径复用 |

### Speed Optimizer（速度优化）

速度规划在 ST 坐标系（纵向距离-时间）下进行，采用两阶段策略：

#### 第一阶段：速度启发式搜索（DP）

`PathTimeHeuristicOptimizer`（`tasks/path_time_heuristic/`）使用动态规划在 ST 图上搜索粗略的速度曲线，为后续 QP 优化提供初始解和边界。

#### 第二阶段：Piecewise Jerk Speed 优化（QP）

`PiecewiseJerkSpeedOptimizer`（`tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h`）将速度优化建模为 QP 问题：

- 优化变量：s(t)、s'(t)、s''(t)
- 目标函数：最小化与 DP 结果的偏差、加速度、jerk
- 约束条件：ST 边界约束（来自障碍物决策）、速度限制、加速度限制、jerk 限制

速度规划相关的 Task 链：

```
SPEED_BOUNDS_PRIORI_DECIDER   → 构建 ST 边界（先验）
SPEED_HEURISTIC_OPTIMIZER     → DP 搜索粗略速度曲线
SPEED_DECIDER                 → 对障碍物做纵向决策（超车/让行）
SPEED_BOUNDS_FINAL_DECIDER    → 更新 ST 边界（最终）
PIECEWISE_JERK_SPEED          → QP 优化最终速度曲线
```

### 轨迹合成

路径和速度规划完成后，`ReferenceLineInfo::CombinePathAndSpeedProfile()` 将二者合成为时空轨迹（`DiscretizedTrajectory`），包含每个轨迹点的 (x, y, theta, kappa, s, t, v, a) 信息。

## 数据流

```
                    ┌─────────────────┐
                    │  PlanningCommand │ (路由/导航指令)
                    └────────┬────────┘
                             │
  ┌──────────────┐  ┌───────┴────────┐  ┌──────────────────┐
  │ Prediction   │  │ Localization   │  │ Chassis          │
  │ Obstacles    │  │ Estimate       │  │ (底盘状态)        │
  └──────┬───────┘  └───────┬────────┘  └────────┬─────────┘
         │                  │                     │
         └──────────────────┼─────────────────────┘
                            │
                   ┌────────▼────────┐
                   │ PlanningComponent│  ← CyberRT Component 入口
                   │   Proc()        │
                   └────────┬────────┘
                            │
                   ┌────────▼────────┐
                   │  OnLanePlanning │
                   │   RunOnce()     │
                   └────────┬────────┘
                            │
              ┌─────────────┼─────────────┐
              │             │             │
    ┌─────────▼──────┐ ┌───▼────┐ ┌──────▼───────┐
    │ ReferenceLine  │ │ Frame  │ │ TrafficDecider│
    │ Provider       │ │ Init   │ │ (交通规则)     │
    └─────────┬──────┘ └───┬────┘ └──────┬───────┘
              │            │             │
              └────────────┼─────────────┘
                           │
                  ┌────────▼────────┐
                  │ PublicRoadPlanner│
                  │   Plan()        │
                  └────────┬────────┘
                           │
                  ┌────────▼────────┐
                  │ ScenarioManager │
                  │   Update()      │
                  └────────┬────────┘
                           │
                  ┌────────▼────────┐
                  │ Current Scenario│
                  │   Process()     │
                  └────────┬────────┘
                           │
                  ┌────────▼────────┐
                  │ Current Stage   │
                  │   Process()     │
                  └────────┬────────┘
                           │
            ┌──────────────┼──────────────┐
            │              │              │
   ┌────────▼───────┐ ┌───▼────┐ ┌───────▼──────┐
   │ Path Tasks     │ │Deciders│ │ Speed Tasks  │
   │ (路径生成)      │ │(决策器) │ │ (速度优化)    │
   └────────┬───────┘ └───┬────┘ └───────┬──────┘
            │             │              │
            └─────────────┼──────────────┘
                          │
                 ┌────────▼────────┐
                 │ CombinePath &   │
                 │ SpeedProfile    │
                 └────────┬────────┘
                          │
                 ┌────────▼────────┐
                 │ ADCTrajectory   │ → 发布到 /apollo/planning
                 └─────────────────┘
```

### 输入数据

| 数据源 | 话题 | 说明 |
|--------|------|------|
| 预测模块 | `/apollo/prediction` | 障碍物预测轨迹（触发消息之一） |
| 底盘 | `/apollo/canbus/chassis` | 车速、档位、转向等（触发消息之一） |
| 定位 | `/apollo/localization/pose` | 车辆位姿（触发消息之一） |
| 交通灯感知 | `/apollo/perception/traffic_light` | 交通灯检测结果 |
| 路由指令 | `/apollo/planning/command` | 导航路由指令（PlanningCommand） |
| 故事讲述 | `/apollo/storytelling` | 场景触发信息 |
| 控制交互 | `/apollo/control/interactive` | 控制模块反馈 |

### 输出数据

| 数据 | 话题 | 说明 |
|------|------|------|
| ADCTrajectory | `/apollo/planning` | 规划轨迹，包含轨迹点序列、决策信息、档位等 |

## 配置方式

Planning 模块采用 Protobuf 文本格式（`.pb.txt`）进行配置，配置文件分布在多个层级。

### 全局配置

`planning_component/conf/planning_config.pb.txt` — 模块级配置：

```protobuf
// planning_base/proto/planning_config.proto
message PlanningConfig {
  optional TopicConfig topic_config = 1;       // 话题配置
  optional PlanningLearningMode learning_mode = 2;  // 学习模式
  optional ReferenceLineConfig reference_line_config = 3;  // 参考线配置
  optional string planner = 4;                 // Planner 插件名
}
```

### Planner 配置

`planning_component/conf/public_road_planner_config.pb.txt` — 定义场景列表及优先级顺序。场景按列表顺序进行优先级判断，排在前面的场景优先级更高。

### Scenario Pipeline 配置

每个场景目录下的 `conf/pipeline.pb.txt` 定义了该场景的 Stage 和 Task 流水线。例如 `scenarios/lane_follow/conf/pipeline.pb.txt`：

```protobuf
stage: {
  name: "LANE_FOLLOW_STAGE"
  type: "LaneFollowStage"
  task { name: "LANE_CHANGE_PATH"  type: "LaneChangePath" }
  task { name: "LANE_FOLLOW_PATH"  type: "LaneFollowPath" }
  task { name: "LANE_BORROW_PATH"  type: "LaneBorrowPath" }
  // ... 更多 task
}
```

### Task 配置

每个 Task 在其目录下有独立的 proto 配置文件（如 `tasks/lane_follow_path/proto/lane_follow_path.proto`），通过 `Task::LoadConfig()` 加载。Task 支持两级配置合并：默认配置（Task 插件目录）和场景级覆盖配置（Scenario 的 conf 目录下按 Stage 名分子目录）。

### 参考线平滑器配置

`planning_component/conf/discrete_points_smoother_config.pb.txt` — 参考线平滑参数：

```protobuf
// planning_base/proto/reference_line_smoother_config.proto
message ReferenceLineSmootherConfig {
  optional double max_constraint_interval = 1;     // 约束点最大间距
  optional double longitudinal_boundary_bound = 2;  // 纵向边界
  optional double max_lateral_boundary_bound = 3;   // 最大横向边界
  optional double min_lateral_boundary_bound = 4;   // 最小横向边界
  oneof SmootherConfig {
    QpSplineSmootherConfig qp_spline = 20;
    SpiralSmootherConfig spiral = 21;
    DiscretePointsSmootherConfig discrete_points = 22;
  }
}
```

### 交通规则配置

`planning_component/conf/traffic_rule_config.pb.txt` — 定义启用的交通规则插件列表：

| 规则 | 类名 | 说明 |
|------|------|------|
| BACKSIDE_VEHICLE | `BacksideVehicle` | 后方车辆处理 |
| CROSSWALK | `Crosswalk` | 人行横道 |
| DESTINATION | `Destination` | 目的地处理 |
| KEEP_CLEAR | `KeepClear` | 禁停区 |
| REFERENCE_LINE_END | `ReferenceLineEnd` | 参考线末端处理 |
| REROUTING | `Rerouting` | 重新路由 |
| STOP_SIGN | `StopSign` | 停车标志 |
| TRAFFIC_LIGHT | `TrafficLight` | 交通信号灯 |
| YIELD_SIGN | `YieldSign` | 让行标志 |

### 插件机制

Planning 模块大量使用 Apollo 的 CyberRT 插件机制（`cyber::plugin_manager::PluginManager`）。Planner、Scenario、Stage、Task、TrafficRule 均通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册为插件，运行时通过配置文件中的类名动态加载。这使得新增场景或任务只需：

1. 实现对应基类的子类
2. 注册为插件
3. 在 pipeline 配置中引用

无需修改框架代码。
