---
title: "Stop Sign Unprotected 无保护停车标志"
---

# Stop Sign Unprotected

> 源码路径：`modules/planning/scenarios/stop_sign_unprotected/`

## 概述

Stop Sign Unprotected 场景处理车辆在无保护停车标志（即无信号灯控制的路口停车标志）处的完整通行流程。场景包含四个阶段：预停车、停车等待、蠕行观察、路口巡航，实现了"停车→观察→缓行通过"的标准驾驶行为。

## 阶段流转

```mermaid
graph LR
    PreStop[PreStop 预停车] --> Stop[Stop 停车等待]
    Stop --> Creep[Creep 蠕行]
    Creep --> IntersectionCruise[IntersectionCruise 路口巡航]
```

## 上下文

```cpp
// context.h
struct StopSignUnprotectedContext : public ScenarioContext {
  ScenarioStopSignUnprotectedConfig scenario_config;
  std::string current_stop_sign_overlap_id;
  double stop_start_time = 0.0;
  double creep_start_time = 0.0;
  std::unordered_map<std::string, std::vector<std::string>> watch_vehicles;
  std::vector<std::pair<hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>>
      associated_lanes;
};
```

| 字段 | 说明 |
| --- | --- |
| `current_stop_sign_overlap_id` | 当前停车标志在参考线上的 overlap ID |
| `stop_start_time` | 车辆停稳的时刻（秒） |
| `creep_start_time` | 蠕行开始时刻（秒） |
| `watch_vehicles` | 需要观察的车辆，按车道 ID 分组 |
| `associated_lanes` | 与当前停车标志关联的所有车道 |

## 核心类

### StopSignUnprotectedScenario

```cpp
// stop_sign_unprotected_scenario.h
class StopSignUnprotectedScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;
  StopSignUnprotectedContext* GetContext() override;
  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;
  bool Exit(Frame* frame) override;
  bool Enter(Frame* frame) override;
 private:
  int GetAssociatedLanes(const hdmap::StopSignInfo& stop_sign_info);
};
```

## 核心函数

### StopSignUnprotectedScenario::IsTransferable()

**职责**：判断是否应从其他场景切换到本场景
**关键步骤**：

1. 从 PlanningContext 获取 `current_stop_sign_overlap_id`
2. 在参考线上查找对应 overlap
3. 从 HDMap 获取停车标志信息
4. 调用 `GetAssociatedLanes()` 获取关联车道

### StopSignUnprotectedScenario::GetAssociatedLanes()

```cpp
int StopSignUnprotectedScenario::GetAssociatedLanes(
    const StopSignInfo& stop_sign_info) {
  context_.associated_lanes.clear();
  std::vector<StopSignInfoConstPtr> associated_stop_signs;
  HDMapUtil::BaseMap().GetStopSignAssociatedStopSigns(
      stop_sign_info.id(), &associated_stop_signs);
  // 遍历关联停车标志，获取其 overlap 车道
  for (const auto stop_sign : associated_stop_signs) {
    const auto& associated_lane_ids = stop_sign->OverlapLaneIds();
    for (const auto& lane_id : associated_lane_ids) {
      const auto lane = HDMapUtil::BaseMap().GetLaneById(lane_id);
      // 存入 context_.associated_lanes
    }
  }
  return 0;
}
```

**职责**：获取与当前停车标志关联的所有车道（用于观察来车）

### StagePreStop::Process()

**职责**：在接近停车线时添加需要观察的车辆
**关键步骤**：

1. 执行参考线上的 task pipeline
2. 检查是否已越过停车线（`kPassStopLineBuffer = 0.3m`）
3. 遍历关联车道上的障碍物，调用 `AddWatchVehicle()` 添加到观察列表
4. 检查车辆是否已停稳（`CheckADCStop()`），停稳后进入 Stop 阶段

### StagePreStop::CheckADCStop()

```cpp
bool StopSignUnprotectedStagePreStop::CheckADCStop(
    const double adc_front_edge_s, const double stop_line_s) {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
      ->GetConfig().vehicle_param().max_abs_speed_when_stopped();
  if (adc_speed > max_adc_stop_speed) return false;
  const double distance_stop_line_to_adc_front_edge =
      stop_line_s - adc_front_edge_s;
  if (distance_stop_line_to_adc_front_edge >
      context->scenario_config.max_valid_stop_distance()) return false;
  return true;
}
```

**职责**：判断车辆是否在停车线前有效停稳
**条件**：速度低于停车阈值 且 距停车线不超过 `max_valid_stop_distance`

### StageStop::Process()

**职责**：停车等待，观察关联车道上的车辆是否离开
**关键步骤**：

1. 设置路口无路权（`SetJunctionRightOfWay(false)`）
2. 检查停车时长是否超过 `stop_duration_sec`
3. 调用 `RemoveWatchVehicle()` 移除已离开的车辆
4. 当 `watch_vehicles` 为空（所有观察车辆已离开）时进入 Creep 阶段

### StageStop::RemoveWatchVehicle()

```cpp
int StopSignUnprotectedStageStop::RemoveWatchVehicle(
    const PathDecision& path_decision,
    StopSignLaneVehicles* watch_vehicles) {
  // 遍历 watch_vehicles 中的每个 obstacle_id
  // 如果障碍物已不在 path_decision 中，或距离 > 10m，则移除
}
```

**职责**：移除已驶离路口的观察车辆

### StageCreep::Process()

**职责**：蠕行通过路口，确认安全后加速
**关键步骤**：

1. 运行 CreepDecider 生成蠕行轨迹
2. 计算蠕行终点 `creep_stop_s`
3. 检查蠕行是否完成（`CheckCreepDone()`）或超时（`creep_timeout_sec`）
4. 完成后进入 IntersectionCruise 阶段

### StageIntersectionCruise::Process()

**职责**：路口内正常巡航行驶，直到驶出路口区域后场景结束

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `stop_duration_sec` | double | 停车等待最短时长（秒） |
| `max_valid_stop_distance` | double | 有效停车距离阈值（米） |
| `creep_timeout_sec` | double | 蠕行超时时间（秒） |
| `creep_stage_config` | CreepStageConfig | 蠕行阶段详细配置 |

## 调用关系

- **上游**：Planning 模块的 ScenarioManager 根据 PlanningContext 中的 stop_sign overlap 信息触发本场景
- **依赖**：HDMap（停车标志和车道信息）、Perception（障碍物检测）、VehicleState（车辆状态）
- **下游**：各 Stage 通过 `ExecuteTaskOnReferenceLine()` 调用 task pipeline 生成轨迹
