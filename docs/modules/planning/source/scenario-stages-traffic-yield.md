# TrafficLight & YieldSign 场景阶段详解

> 源码位置：`modules/planning/scenarios/traffic_light_protected/`、`modules/planning/scenarios/yield_sign/`

## 模块定位

本文档覆盖两类交通标志场景的阶段实现：
- **TrafficLightProtected**（有保护信号灯）：等待绿灯 → 通过路口
- **YieldSign**（让行标志）：接近让行线 → 蠕行观察 → 通过

---

## 一、TrafficLightProtected 有保护信号灯场景

### 场景上下文

```cpp
struct TrafficLightProtectedContext : public ScenarioContext {
  ScenarioTrafficLightProtectedConfig scenario_config;
  std::vector<std::string> current_traffic_light_overlap_ids;
};
```

### 阶段流程

```
Approach → IntersectionCruise → 场景结束
```

### 1. TrafficLightProtectedStageApproach — 接近阶段

> `stage_approach.cc`

```cpp
class TrafficLightProtectedStageApproach : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
 private:
  StageResult FinishStage();
};
```

#### Process()

```cpp
StageResult TrafficLightProtectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  bool traffic_light_all_done = true;
  for (const auto& traffic_light_overlap_id : overlap_ids) {
    PathOverlap* overlap = reference_line_info.GetOverlapOnReferenceLine(
        traffic_light_overlap_id, ReferenceLineInfo::SIGNAL);
    reference_line_info.SetJunctionRightOfWay(overlap->start_s, false);

    const double distance_adc_to_stop_line = overlap->start_s - adc_front_edge_s;
    // 距离停止线太远，继续等待
    if (distance_adc_to_stop_line > scenario_config.max_valid_stop_distance()) {
      traffic_light_all_done = false; break;
    }
    // 非绿灯，继续等待
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_done = false; break;
    }
  }
  if (traffic_light_all_done) return FinishStage();
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- 遍历所有关联信号灯 overlap
- 完成条件：所有信号灯为绿灯 **且** 自车距停止线 ≤ `max_valid_stop_distance`
- `stage_approach.cc:L37-L100`

#### FinishStage()

- 将所有 traffic_light_overlap_id 标记为 done
- 设置 `next_stage_ = "TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE"`
- `stage_approach.cc:L102-L115`

### 2. TrafficLightProtectedStageIntersectionCruise — 路口巡航阶段

> `stage_intersection_cruise.cc`

```cpp
class TrafficLightProtectedStageIntersectionCruise : public BaseStageIntersectionCruise {
 public:
  StageResult Process(...) override;
 private:
  StageResult FinishStage();
};
```

#### Process()

```cpp
StageResult TrafficLightProtectedStageIntersectionCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  bool stage_done = CheckDone(*frame, injector_->planning_context(), true);
  if (stage_done) return FinishStage();
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- `CheckDone` 第三个参数 `true` 表示有路权（绿灯已确认）
- 驶出路口后调用 `FinishScenario()` 结束场景
- `stage_intersection_cruise.cc:L28-L48`

---

## 二、YieldSign 让行标志场景

### 阶段流程

```
Approach → Creep → 场景结束
```

### 1. YieldSignStageApproach — 接近阶段

> `stage_approach.cc`

```cpp
class YieldSignStageApproach : public Stage {
 public:
  StageResult Process(...) override;
 private:
  StageResult FinishStage();
  ScenarioYieldSignConfig scenario_config_;
};
```

#### Process()

```cpp
StageResult YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  for (const auto& yield_sign_overlap_id : overlap_ids) {
    PathOverlap* overlap = ...;
    reference_line_info.SetJunctionRightOfWay(overlap->start_s, false);

    // 已越过停止线
    if (adc_front_edge_s - overlap->start_s > kPassStopLineBuffer)
      return FinishStage();

    // 接近停止线时检查是否可安全通过
    if (distance_adc_to_stop_line < max_valid_stop_distance) {
      bool yield_sign_done = true;
      for (const auto* obstacle : path_decision.obstacles().Items()) {
        if (obstacle->IsVirtual()) continue;
        if (obstacle->reference_line_st_boundary().IsEmpty()) continue;
        if (obstacle->reference_line_st_boundary().min_t() > 6.0) continue;
        // 忽略已在参考线上同向行驶的障碍物
        if (traveled_s < epsilon && min_t < 0.1 && min_s > 15.0) continue;
        yield_sign_done = false;
      }
      if (yield_sign_done) return FinishStage();
    }
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- `kPassStopLineBuffer = 0.3m`
- 让行判定：检查 ST 边界中 `min_t ≤ 6s` 的障碍物是否会与自车冲突
- 忽略条件：障碍物已在参考线上、同向行驶、距离 > 15m
- `stage_approach.cc:L37-L146`

#### FinishStage()

- 更新 `PlanningContext` 中的 `done_yield_sign_overlap_id`
- 记录 `creep_start_time`
- 设置 `next_stage_ = "YIELD_SIGN_CREEP"`
- `stage_approach.cc:L148-L165`

### 2. YieldSignStageCreep — 蠕行阶段

> `stage_creep.cc`

```cpp
class YieldSignStageCreep : public BaseStageCreep {
 public:
  bool Init(...) override;
  StageResult Process(...) override;
 private:
  const CreepStageConfig& GetCreepStageConfig() const override;
  bool GetOverlapStopInfo(Frame*, ReferenceLineInfo*,
                          double* overlap_end_s, std::string* overlap_id) const override;
  StageResult FinishStage();
};
```

#### Process()

```cpp
StageResult YieldSignStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  if (!pipeline_config_.enabled()) return FinishStage();

  // 执行 CreepDecider
  for (auto& rli : *frame->mutable_reference_line_info()) {
    ProcessCreep(frame, &rli);
  }
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  // 设置无路权
  reference_line_info.SetJunctionRightOfWay(yield_sign_start_s, false);

  // 检查蠕行完成
  double creep_stop_s = GetCreepFinishS(yield_sign_end_s, *frame, rli);
  if (distance <= 0.0) {
    SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }
  if (CheckCreepDone(*frame, rli, yield_sign_end_s, wait_time, timeout_sec)) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- 蠕行超时由 `creep_timeout_sec` 控制
- 完成后直接调用 `FinishScenario()` 结束整个场景（无 IntersectionCruise 阶段）
- `stage_creep.cc:L54-L127`

---

## 阶段转换对比

| 场景 | 阶段序列 | 路权设置 |
|------|----------|----------|
| TrafficLightProtected | Approach → IntersectionCruise | 绿灯后有路权 |
| YieldSign | Approach → Creep | 始终无路权 |
| StopSignUnprotected | PreStop → Stop → Creep → IntersectionCruise | 始终无路权 |

## 调用关系

- **上游**：各自的 Scenario 类创建并管理 Stage
- **下游**：`ExecuteTaskOnReferenceLine` → Task 流水线
- **共用基类**：`BaseStageCreep`（蠕行逻辑）、`BaseStageIntersectionCruise`（路口巡航判定）
