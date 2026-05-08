---
title: "Traffic Light Protected 有保护信号灯"
---

# Traffic Light Protected

> 源码路径：`modules/planning/scenarios/traffic_light_protected/`

## 概述

Traffic Light Protected 场景处理车辆在有信号灯保护的路口通行。当车辆接近信号灯且检测到非绿灯（红灯/黄灯/未知）时触发本场景，车辆在停车线前等待，直到信号灯变绿后通过路口。场景包含两个阶段：接近等待（Approach）和路口巡航（IntersectionCruise）。

## 阶段流转

```mermaid
graph LR
    Approach[Approach 接近等待] --> IntersectionCruise[IntersectionCruise 路口巡航]
```

## 上下文

```cpp
// context.h
struct TrafficLightProtectedContext : public ScenarioContext {
  ScenarioTrafficLightProtectedConfig scenario_config;
  std::vector<std::string> current_traffic_light_overlap_ids;
};
```

| 字段 | 说明 |
| --- | --- |
| `scenario_config` | 场景配置（距离阈值等） |
| `current_traffic_light_overlap_ids` | 当前路口所有关联信号灯的 overlap ID 列表 |

## 核心类

### TrafficLightProtectedScenario

```cpp
// traffic_light_protected_scenario.h
class TrafficLightProtectedScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;
  TrafficLightProtectedContext* GetContext() override;
  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;
  bool Exit(Frame* frame) override;
  bool Enter(Frame* frame) override;
};
```

### TrafficLightProtectedStageApproach

```cpp
// stage_approach.h
class TrafficLightProtectedStageApproach : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
 private:
  StageResult FinishStage();
};
```

### TrafficLightProtectedStageIntersectionCruise

```cpp
// stage_intersection_cruise.h
class TrafficLightProtectedStageIntersectionCruise
    : public BaseStageTrafficLightCruise {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
 private:
  StageResult FinishStage();
};
```

## 核心函数

### TrafficLightProtectedScenario::IsTransferable()

```cpp
bool TrafficLightProtectedScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  // 1. 必须有 lane_follow_command
  // 2. 检查 first_encountered_overlaps，如果先遇到 STOP_SIGN/YIELD_SIGN 则不切换
  // 3. 找到 SIGNAL 类型的 overlap
  // 4. 将距离 < kTrafficLightGroupingMaxDist(2m) 的信号灯归为同组
  // 5. 检查组内是否有非 GREEN/BLACK 的信号灯
  // 6. 满足条件则记录所有 overlap_id 到 context
}
```

**职责**：判断是否应切换到本场景
**关键逻辑**：

1. 排除优先级更高的 STOP_SIGN / YIELD_SIGN
2. 将距离在 2m 内的信号灯视为同一组
3. 只要组内有一个非绿灯，即触发场景
4. 距离检查：车辆前沿到信号灯的距离在 `(0, start_traffic_light_scenario_distance]` 范围内

### StageApproach::Process()

```cpp
StageResult TrafficLightProtectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  bool traffic_light_all_done = true;
  for (const auto& traffic_light_overlap_id :
       context->current_traffic_light_overlap_ids) {
    PathOverlap* current_traffic_light_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            traffic_light_overlap_id, ReferenceLineInfo::SIGNAL);
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();

    if (distance_adc_to_stop_line > scenario_config.max_valid_stop_distance()) {
      traffic_light_all_done = false;
      break;
    }
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_done = false;
      break;
    }
  }
  if (traffic_light_all_done) return FinishStage();
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

**职责**：在信号灯前等待，直到所有关联信号灯变绿且车辆已接近停车线
**输入**：规划起始点、当前帧
**关键步骤**：

1. 执行 task pipeline 生成轨迹（含停车决策）
2. 设置路口无路权（`SetJunctionRightOfWay(false)`）
3. 遍历所有关联信号灯，检查：
   - 车辆距停车线 ≤ `max_valid_stop_distance`
   - 信号灯颜色为 GREEN
4. 全部满足时进入 IntersectionCruise 阶段

### StageApproach::FinishStage()

```cpp
StageResult TrafficLightProtectedStageApproach::FinishStage() {
  auto* traffic_light = injector_->planning_context()
      ->mutable_planning_status()->mutable_traffic_light();
  traffic_light->clear_done_traffic_light_overlap_id();
  for (const auto& id : context->current_traffic_light_overlap_ids) {
    traffic_light->add_done_traffic_light_overlap_id(id);
  }
  next_stage_ = "TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE";
  return StageResult(StageStatusType::FINISHED);
}
```

**职责**：标记信号灯已通过，更新 PlanningContext，切换到路口巡航阶段

### StageIntersectionCruise::Process()

```cpp
StageResult TrafficLightProtectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  bool stage_done = CheckDone(*frame, injector_->planning_context(), true);
  if (stage_done) return FinishStage();
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

**职责**：路口内正常巡航，驶出路口后结束场景
**关键步骤**：继承 `BaseStageTrafficLightCruise` 的 `CheckDone()` 判断是否已驶出路口

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `start_traffic_light_scenario_distance` | double | 触发场景的最大距离（米） |
| `max_valid_stop_distance` | double | 判定车辆已到达停车线的距离阈值（米） |

## 调用关系

- **上游**：ScenarioManager 通过 `IsTransferable()` 检测到非绿灯信号触发本场景
- **依赖**：Perception（信号灯颜色检测）、HDMap（信号灯 overlap 信息）、PlanningContext（状态传递）
- **下游**：各 Stage 通过 `ExecuteTaskOnReferenceLine()` 调用 task pipeline 生成轨迹
