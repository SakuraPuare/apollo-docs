---
title: "红绿灯无保护右转场景"
---

# 红绿灯无保护右转场景

> 源码路径: `modules/planning/scenarios/traffic_light_unprotected_right_turn/`

## 概述

红绿灯无保护右转场景（Traffic Light Unprotected Right Turn）处理车辆在红绿灯路口执行无保护右转的情形。当自动驾驶车辆在右转路径上遇到非绿色信号灯时，系统需要按照交通规则进行停车等待、低速蠕行通过路口，最终完成巡航。该场景通过三个阶段依次执行：**停车（Stop）**、**蠕行（Creep）**、**路口巡航（Intersection Cruise）**。

场景激活条件：参考线上最近的重叠元素为交通信号灯、当前路径为右转弯、且信号灯非绿色（RED/YELLOW/UNKNOWN）。

## 核心类

### TrafficLightUnprotectedRightTurnScenario

场景主类，继承自 `Scenario`，负责场景生命周期管理。

```cpp
class TrafficLightUnprotectedRightTurnScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;
  TrafficLightUnprotectedRightTurnContext* GetContext() override;
  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;
  bool Exit(Frame* frame) override;
  bool Enter(Frame* frame) override;
};
```

### TrafficLightUnprotectedRightTurnContext

场景上下文结构体，继承自 `ScenarioContext`，存储场景运行时状态。

```cpp
struct TrafficLightUnprotectedRightTurnContext : public ScenarioContext {
  ScenarioTrafficLightUnprotectedRightTurnConfig scenario_config;
  std::vector<std::string> current_traffic_light_overlap_ids;
  double stop_start_time = 0.0;
  double creep_start_time = 0.0;
};
```

### TrafficLightUnprotectedRightTurnStageStop

停车阶段，继承自 `Stage`。在路口停止线前停车，检查信号灯状态和"红灯禁止右转"标志，决定后续阶段。

```cpp
class TrafficLightUnprotectedRightTurnStageStop : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
 private:
  bool CheckTrafficLightNoRightTurnOnRed(const std::string& traffic_light_id);
  StageResult FinishStage(const bool protected_mode);
};
```

### TrafficLightUnprotectedRightTurnStageCreep

蠕行阶段，继承自 `BaseStageTrafficLightCreep`。以低速缓慢通过路口，持续检查是否到达蠕行终点或超时。

```cpp
class TrafficLightUnprotectedRightTurnStageCreep
    : public BaseStageTrafficLightCreep {
 public:
  bool Init(const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context) override;
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
 private:
  const CreepStageConfig& GetCreepStageConfig() const override;
  StageResult FinishStage();
};
```

### TrafficLightUnprotectedRightTurnStageIntersectionCruise

路口巡航阶段，继承自 `BaseStageTrafficLightCruise`。在通过路口后沿参考线巡航，完成后结束整个场景。

```cpp
class TrafficLightUnprotectedRightTurnStageIntersectionCruise
    : public BaseStageTrafficLightCruise {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
 private:
  StageResult FinishStage();
};
```

## 核心函数

### IsTransferable

判断场景是否可转入。核心逻辑：

1. 当前规划命令必须为车道跟随（`lane_follow_command`）
2. 参考线上第一个遇到的重叠元素必须是信号灯（排除停车标志和让行标志）
3. 将距离首个信号灯 2.0m 以内的其他信号灯归为同一组
4. 逐个检查同组信号灯，任一信号灯非 GREEN 且非 BLACK 时触发场景
5. 当前路径在信号灯位置必须为右转弯（`RIGHT_TURN`）

### Enter

进入场景时，找到当前遇到的信号灯重叠组，将同组（2.0m 范围内）的所有信号灯 ID 写入 `PlanningContext`。

### StageStop::Process

停车阶段核心处理逻辑：

1. 遍历当前信号灯组，设置路口优先权为无（`right_of_way = false`）
2. 若车辆已越过停止线且距离超过 `min_pass_s_distance`，直接完成
3. 检查 `CheckTrafficLightNoRightTurnOnRed`：查询高精地图中信号灯的 `SignInfo`（NO_RIGHT_TURN_ON_RED）和子信号类型（ARROW_RIGHT）
4. 全绿灯时以保护模式（`protected_mode=true`）完成，跳过蠕行直接进入路口巡航
5. 红灯且允许红灯右转时，等待 `red_light_right_turn_stop_duration_sec` 后完成

### FinishStage (Stop)

根据信号灯状态和车速决定下一阶段：

- `protected_mode=true`（全绿灯）：直接进入 `INTERSECTION_CRUISE`
- `protected_mode=false` 且车速超过 `max_adc_speed_before_creep`：跳过蠕行，进入 `INTERSECTION_CRUISE`
- 否则：进入 `CREEP` 阶段，记录蠕行开始时间

### StageCreep::Process

蠕行阶段处理：调用 `ProcessCreep` 执行蠕行决策器，在每帧检查是否已到达蠕行终点或超时（`creep_timeout_sec`），满足条件则进入路口巡航阶段。

## 配置

配置定义在 `proto/traffic_light_unprotected_right_turn.proto` 中，消息类型为 `ScenarioTrafficLightUnprotectedRightTurnConfig`。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `start_traffic_light_scenario_distance` | double | 5.0 m | 距信号灯多远时触发场景 |
| `enable_right_turn_on_red` | bool | false | 是否允许红灯右转 |
| `max_valid_stop_distance` | double | 3.5 m | 车辆距停止线的有效停车距离 |
| `min_pass_s_distance` | double | 3.0 m | 车辆越过停止线多远后可继续通行 |
| `red_light_right_turn_stop_duration_sec` | float | 3.0 s | 红灯右转前需等待的时间 |
| `creep_timeout_sec` | float | 10.0 s | 蠕行阶段超时时间 |
| `max_adc_speed_before_creep` | double | 3.0 m/s | 超过此速度则跳过蠕行阶段 |
| `creep_stage_config` | CreepStageConfig | - | 蠕行阶段详细配置 |

实际运行配置见 `conf/scenario_conf.pb.txt`，其中 `max_valid_stop_distance` 覆盖为 2.0m。

## 调用关系

场景通过流水线（Pipeline）机制配置三个阶段，每个阶段包含完整的规划任务链：

```text
TrafficLightUnprotectedRightTurnScenario
  |
  +-- Stage 1: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_STOP
  |     (TrafficLightUnprotectedRightTurnStageStop)
  |     Tasks: LaneFollowPath -> LaneBorrowPath -> FallbackPath ->
  |            PathDecider -> RuleBasedStopDecider -> STBoundsDecider ->
  |            SpeedBoundsDecider -> PathTimeHeuristicOptimizer ->
  |            SpeedDecider -> SpeedBoundsDecider -> PiecewiseJerkSpeedOptimizer
  |
  +-- Stage 2: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP
  |     (TrafficLightUnprotectedRightTurnStageCreep)
  |     Tasks: [同上]
  |
  +-- Stage 3: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE
        (TrafficLightUnprotectedRightTurnStageIntersectionCruise)
        Tasks: [同上]
```

阶段转移规则：

- **Stop -> IntersectionCruise**：全绿灯（保护模式），或红灯允许右转且车速较高时跳过蠕行
- **Stop -> Creep**：非绿灯且车速低于阈值，需低速蠕行通过路口
- **Creep -> IntersectionCruise**：蠕行通过路口区域或蠕行超时
- **IntersectionCruise -> 场景结束**：路口巡航完成
