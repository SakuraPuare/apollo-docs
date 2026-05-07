# Scenarios 场景机模块函数级源码解析

本文聚焦 `modules/planning/scenarios/` 目录，按函数级粒度拆解 Apollo 规划模块的 **17 个场景机**实现。场景机是规划流水线的顶层调度单元，负责根据当前驾驶环境选择合适的场景，并按阶段（Stage）驱动任务执行。

## 1. 模块定位

场景机（Scenario）是 Apollo 规划模块的**顶层状态机**。`ScenarioManager` 根据车辆状态、地图信息和交通信号，在每帧规划中选择合适的场景。每个场景包含一个或多个阶段（Stage），阶段按顺序执行，每个阶段内部调用具体的规划任务（Task）。

```
ScenarioManager
    │
    ├── IsTransferable? → 选择新场景
    │
    └── Scenario (当前场景)
            │
            ├── Stage[0] → Process() → Task[0..N]
            │       └── FinishStage() → Stage[1]
            ├── Stage[1] → Process() → Task[0..N]
            │       └── FinishStage() → Stage[2]
            └── ...
```

## 2. 基类框架

### 2.1 Scenario 基类

所有场景继承自 `Scenario`（定义在 `planning_interface_base/scenario_base/scenario.h`）：

- `Init(injector, name)`：加载配置、注册阶段
- `IsTransferable(current_scenario, frame)`：判断是否应切换到本场景
- `Enter(frame)` / `Exit(frame)`：场景进入/退出回调
- `Process(planning_init_point, frame)`：驱动当前阶段执行
- `GetContext()`：返回场景上下文（各场景特有的状态数据）

### 2.2 Stage 基类

所有阶段继承自 `Stage` 或其子类：

| 基类 | 用途 | 使用场景 |
|------|------|---------|
| `Stage` | 通用阶段基类 | 大多数阶段 |
| `BaseStageCruise` | 交叉口巡航基类 | 路口通过阶段 |
| `BaseStageCreep` | 蠕行基类 | 缓行探查阶段 |
| `BaseStageTrafficLightCruise` | 交通灯路口巡航 | 交通灯路口通过 |
| `BaseStageTrafficLightCreep` | 交通灯路口蠕行 | 交通灯路口探查 |

### 2.3 ScenarioContext 上下文

每个场景定义自己的 `Context` 结构体，继承自 `ScenarioContext`，用于在阶段间传递状态数据。

## 3. 场景分类总览

| 类别 | 场景 | 阶段数 | 说明 |
|------|------|--------|------|
| **车道保持** | LaneFollow | 1 | 默认场景，正常车道行驶 |
| | LaneFollowPark | 2 | 泊车区域车道保持+脱困 |
| | LargeCurvature | 1 | 大曲率弯道 |
| **信号灯路口** | TrafficLightProtected | 2 | 保护相位直行 |
| | TrafficLightUnprotectedLeftTurn | 3 | 无保护左转 |
| | TrafficLightUnprotectedRightTurn | 3 | 无保护右转 |
| **标志路口** | StopSignUnprotected | 4 | 停车标志无保护通过 |
| | BareIntersectionUnprotected | 2 | 无信号无标志路口 |
| | YieldSign | 2 | 让行标志 |
| **紧急** | EmergencyPullOver | 3 | 紧急靠边停车 |
| | EmergencyStop | 2 | 紧急原地停车 |
| **泊车** | ValetParking | 2 | 代客泊车 |
| | ValetParkingPark | 3 | 代客泊车（含重试） |
| | ParkAndGo | 4 | 泊车起步 |
| | PullOver | 3 | 靠边停车 |
| **特殊** | FreeSpace | 1 | 无地图自由空间 |
| | Square | 2 | 广场/环岛 |

## 4. 车道保持类场景

### 4.1 LaneFollowScenario — 车道跟随

**默认场景**，车辆大部分时间处于此场景。

```cpp
class LaneFollowScenario : public Scenario {
  void GetContext() → nullptr  // 无上下文
  bool IsTransferable(const Scenario*, const Frame&);
};
```

**LaneFollowStage** 核心方法：

- `PlanOnReferenceLine`：在单条参考线上执行完整规划流程
  - 执行所有已注册的 Task（路径决策、速度决策等）
  - 处理停车决策（`GetStopSL` 将 `ObjectStop` 转为 SL 坐标）
- `PlanFallbackTrajectory`：生成回退轨迹
- `RecordObstacleDebugInfo`：记录障碍物调试信息

**特殊设计**：无 Context、无 Enter/Exit，是最轻量的场景。

### 4.2 LaneFollowParkScenario — 泊车区车道保持

处理被停放车辆阻挡时的车道保持和脱困。

```cpp
class LaneFollowParkScenario : public Scenario {
  void GetContext() → LaneFollowParkContext*
};
```

**Stage 1: LaneFollowParkStage** — 车道保持

- 与 `LaneFollowStage` 类似，增加阻塞检测：
  - `IsNeedEscape`：判断是否需要脱困
  - `IsQueueSence`：检测排队场景（前方多辆车排队）
  - `IsStableBlockObs`：检测稳定阻塞障碍物（持续多帧）
  - `IsEnoughSpace`：判断是否有足够空间脱困
  - `GetClosestStopDecisionObs`：获取最近的停车决策障碍物

**Stage 2: LaneEscapeParkStage** — 脱困

- `GenerateStraightReversePath`：生成直行-倒车脱困路径
- `HasCollision`：碰撞检测
- 状态管理：`success_path_times_`（成功路径次数）、`stop_check_count_`（停车检查计数）

### 4.3 LargeCurvatureScenario — 大曲率弯道

处理急弯路段的特殊规划。

```cpp
class LargeCurvatureContext : public ScenarioContext {
  ScenarioLargeCurvatureConfig scenario_config;
};
```

**StageLargeCurvature**：

- `IsCurvatureSmall`：检查曲率是否已减小到可退出本场景
- 根据地图路径信息调整规划参数

## 5. 信号灯路口类场景

### 5.1 TrafficLightProtectedScenario — 保护相位通过

绿灯保护相位下的直行通过。

```cpp
class TrafficLightProtectedContext : public ScenarioContext {
  vector<string> current_traffic_light_overlap_ids;
};
```

**Stage 1: Approach** — 接近路口

- 检测交通灯状态
- 红灯/黄灯时在停车线前停车

**Stage 2: IntersectionCruise**（继承 `BaseStageTrafficLightCruise`）— 路口巡航

- 绿灯时正常通过路口
- 监控交通灯状态变化

### 5.2 TrafficLightUnprotectedLeftTurnScenario — 无保护左转

绿灯但无保护相位的左转，需让行对向直行车辆。

```cpp
class TrafficLightUnprotectedLeftTurnContext : public ScenarioContext {
  vector<string> current_traffic_light_overlap_ids;
  double creep_start_time;
};
```

**Stage 1: Approach** — 接近路口，等待绿灯

**Stage 2: Creep**（继承 `BaseStageTrafficLightCreep`）— 蠕行探查

- 缓慢进入路口
- 检查对向来车间隙
- 确认安全后进入巡航

**Stage 3: IntersectionCruise** — 完成左转通过路口

### 5.3 TrafficLightUnprotectedRightTurnScenario — 无保护右转

红灯时的右转（需先停车检查）。

```cpp
class TrafficLightUnprotectedRightTurnContext : public ScenarioContext {
  vector<string> current_traffic_light_overlap_ids;
  double stop_start_time;
  double creep_start_time;
};
```

**Stage 1: Stop** — 停车检查

- `CheckTrafficLightNoRightTurnOnRed`：检查是否禁止红灯右转
- 若禁止，等待绿灯；若允许，停车后进入蠕行

**Stage 2: Creep** — 蠕行探查右侧来车

**Stage 3: IntersectionCruise** — 完成右转

## 6. 标志路口类场景

### 6.1 StopSignUnprotectedScenario — 停车标志

最复杂的场景（4 个阶段），处理无信号灯的停车标志路口。

```cpp
class StopSignUnprotectedContext : public ScenarioContext {
  string current_stop_sign_overlap_id;
  double stop_start_time;
  double creep_start_time;
  unordered_map<string, vector<string>> watch_vehicles;  // 监控车辆
  vector<pair<LaneInfoConstPtr, OverlapInfoConstPtr>> associated_lanes;
};
```

**Stage 1: PreStop** — 预停车

- `AddWatchVehicle`：记录路口处的来车
- `CheckADCStop`：确认自车已停稳
- 到达停车线后进入下一阶段

**Stage 2: Stop** — 停车等待

- `RemoveWatchVehicle`：移除已通过的车辆
- 等待配置时间（`stop_duration`）
- 所有关联车道无来车后进入蠕行

**Stage 3: Creep**（继承 `BaseStageCreep`）— 蠕行探查

- 低速进入路口
- `GetOverlapStopInfo`：获取交叉口重叠区域的停车信息
- 确认安全后进入巡航

**Stage 4: IntersectionCruise**（继承 `BaseStageCruise`）— 路口巡航

- `GetTrafficSignOverlap`：获取交通标志重叠信息
- 通过路口后退出场景

### 6.2 BareIntersectionUnprotectedScenario — 无信号路口

无交通灯也无停车标志的路口。

```cpp
class BareIntersectionUnprotectedContext : public ScenarioContext {
  string current_pnc_junction_overlap_id;
};
```

**Stage 1: Approach** — 接近并检查

- `CheckClear`：检查路口是否畅通（无障碍物）
- `counter_`：计数器用于确认连续多帧畅通

**Stage 2: IntersectionCruise**（继承 `BaseStageCruise`）— 通过路口

### 6.3 YieldSignScenario — 让行标志

```cpp
class YieldSignContext : public ScenarioContext {
  vector<string> current_yield_sign_overlap_ids;
  double creep_start_time;
};
```

**Stage 1: Approach** — 接近让行标志，减速

**Stage 2: Creep**（继承 `BaseStageCreep`）— 蠕行探查

- `GetOverlapStopInfo`：获取让行区域信息
- 确认安全后通过

## 7. 紧急类场景

### 7.1 EmergencyPullOverScenario — 紧急靠边停车

```cpp
class EmergencyPullOverContext : public ScenarioContext {
  double target_slow_down_speed = 0.0;
};
```

**Stage 1: Approach** — 寻找并接近靠边停车位置

**Stage 2: SlowDown** — 减速至 `target_slow_down_speed`

**Stage 3: Standby** — 停车等待

### 7.2 EmergencyStopScenario — 紧急原地停车

```cpp
class EmergencyStopScenario : public Scenario {
  void Process(const TrajectoryPoint&, Frame*) override;  // 场景级 Process
};
```

**特殊设计**：唯一重写场景级 `Process` 方法的场景。

**Stage 1: Approach** — 减速至停车

**Stage 2: Standby** — 保持停车状态

## 8. 泊车类场景

### 8.1 ValetParkingScenario — 代客泊车

```cpp
class ValetParkingContext : public ScenarioContext {
  string target_parking_spot_id;
  bool pre_stop_rightaway_flag;
  hdmap::MapPathPoint pre_stop_rightaway_point;
};
```

**场景初始化**：

- `SearchTargetParkingSpotOnPath`：在地图路径上搜索目标车位
- `CheckDistanceToParkingSpot`：检查到目标车位的距离

**Stage 1: ApproachingParkingSpot** — 接近车位

- `CheckADCStop`：确认自车已停在车位前

**Stage 2: Parking** — 执行泊车

### 8.2 ValetParkingParkScenario — 代客泊车（含重试）

扩展版泊车场景，增加泊车重试机制。

```cpp
enum ParkingMissionStatus { RUNNING = 0, DONE = 1 };
struct ParkingMissionInfo {
  string parking_spot_id;
  int command_sequence_num;
  ParkingMissionStatus status;
};
```

**Stage 1: ApproachingParkingSpotPark**

- `GetTargetS`：获取目标纵向位置
- `CheckADCInParkingRange`：检查是否在泊车范围内

**Stage 2: ParkingPark**

- `CheckADCParkingCompleted`：检查泊车是否完成
- `CheckParkingMissionInfo`：检查泊车任务状态
- 完成后更新 `ParkingMissionStatus`

**Stage 3: ParkingRetryPark** — 泊车重试

- `CheckParkingAccuracy`：检查泊车精度
- 若精度不满足，重新执行泊车

### 8.3 ParkAndGoScenario — 泊车起步

从停车位起步驶入道路。

```cpp
class ParkAndGoContext : public ScenarioContext {
  ScenarioParkAndGoConfig scenario_config;
};
```

**Stage 1: Check** — 环境检查

- `CheckObstacle`：检查周围障碍物
- `ADCInitStatus`：初始化自车状态

**Stage 2: Adjust** — 调整车位姿

- `ResetInitPostion`：重置初始位置
- 调整车头方向使其朝向道路

**Stage 3: PreCruise** — 预巡航准备

**Stage 4: Cruise** — 正式巡航

- `CheckADCParkAndGoCruiseCompleted`：检查巡航完成状态
- 状态枚举：`CRUISING → CRUISE_COMPLETE → ADJUST → ADJUST_COMPLETE → FAIL`

**工具函数（util.h）**：

- `CheckADCSurroundObstacles`：检查自车周围障碍物
- `CheckADCHeading`：检查自车航向是否对齐道路
- `CheckADCReadyToCruise`：检查自车是否准备好巡航

### 8.4 PullOverScenario — 靠边停车

```cpp
enum PullOverState {
  UNKNOWN = 0, APPROACHING = 1, PARK_COMPLETE = 2,
  PARK_FAIL = 3, PASS_DESTINATION = 4
};
```

**Stage 1: Approach** — 接近靠边停车位置

**Stage 2: RetryApproachParking** — 重试接近

- `CheckADCStop`：确认自车已停

**Stage 3: RetryParking** — 重试泊车

- `CheckADCPullOverOpenSpace`：检查开放空间

**工具函数（util.h）**：

- `CheckADCPullOver`：检查靠边停车状态
- `CheckADCPullOverPathPoint`：检查路径点级别的靠边停车状态
- `CheckPullOverPositionBySL`：通过 SL 坐标检查靠边停车位置

## 9. 特殊场景

### 9.1 FreeSpaceScenario — 自由空间

无高精地图车道信息的区域（停车场、空地等）。

```cpp
class FreeSpaceContext : public ScenarioContext {
  external_command::FreeSpaceCommand free_space_command;
};
```

**StageFreeSpace**：

- 使用 `FreeSpaceCommand` 导航
- 依赖感知模块的自由空间边界检测

### 9.2 SquareScenario — 广场/环岛

```cpp
class SquareContext : public ScenarioContext {
  string junction_id;
  string blocking_obstacle_id;
};
```

**Stage 1: SquareLaneFollowStage** — 车道跟随

- `blocking_times_`：阻塞计数
- 检测阻塞障碍物

**Stage 2: ExtricateStage** — 脱困

- `GenerateStraightReversePath`：生成直行-倒车脱困路径
- `HasCollision`：碰撞检测

## 10. 场景切换机制

### 10.1 IsTransferable 判断

每个场景实现 `IsTransferable` 方法，判断是否应从当前场景切换到本场景：

- **LaneFollow**：默认场景，其他场景都不适用时回退到此
- **TrafficLightProtected**：检测到保护相位的交通灯重叠区域
- **StopSignUnprotected**：检测到停车标志重叠区域
- **EmergencyPullOver/EmergencyStop**：收到紧急停车命令
- **ValetParking**：收到泊车命令且检测到目标车位

### 10.2 Enter/Exit 回调

- `Enter`：进入场景时初始化上下文、记录状态
- `Exit`：退出场景时清理状态
- 并非所有场景都重写这两个方法（如 LaneFollow 没有）

### 10.3 阶段切换

- `FinishStage()`：返回 `StageResult`，包含下一阶段名称
- 阶段名称在配置文件中定义
- `ScenarioManager` 根据 `StageResult` 切换到下一阶段

## 11. 设计模式

### 11.1 状态机模式

- 场景级状态机：`ScenarioManager` 管理场景切换
- 阶段级状态机：每个场景内部管理阶段切换
- 双层状态机解耦了宏观驾驶模式和微观执行步骤

### 11.2 上下文传递

- `ScenarioContext` 子类在阶段间共享状态
- 通过 `GetContext()` 获取，类型安全
- 避免全局变量，保持阶段间松耦合

### 11.3 继承复用

- `BaseStageCruise`：统一交叉口巡航逻辑
- `BaseStageCreep`：统一蠕行探查逻辑
- `BaseStageTrafficLightCruise/Creep`：交通灯特定的路口逻辑
- 子类只需重写 `GetTrafficSignOverlap` 或 `GetCreepStageConfig` 即可定制行为