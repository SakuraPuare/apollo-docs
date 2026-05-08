# TrafficLight Unprotected 场景阶段详解

> 源码位置：`modules/planning/scenarios/traffic_light_unprotected_left_turn/`、`modules/planning/scenarios/traffic_light_unprotected_right_turn/`

## 模块定位

无保护信号灯场景处理没有专用转向信号灯保护的左转和右转。与有保护场景不同，这些场景需要在绿灯时额外判断对向/侧向交通是否安全，通过蠕行（creep）方式谨慎通过路口。

---

## 一、TrafficLightUnprotectedLeftTurn 无保护左转

### 场景上下文

```cpp
struct TrafficLightUnprotectedLeftTurnContext : public ScenarioContext {
  ScenarioTrafficLightUnprotectedLeftTurnConfig scenario_config;
  std::vector<std::string> current_traffic_light_overlap_ids;
  double creep_start_time;
};
```

### 阶段流程

```
Approach → Creep（可跳过）→ IntersectionCruise → 场景结束
```

### 1. StageApproach — 减速接近阶段

> `stage_approach.cc`

```cpp
class TrafficLightUnprotectedLeftTurnStageApproach : public Stage {
 public:
  StageResult Process(...) override;
 private:
  StageResult FinishStage(Frame* frame);
};
```

#### Process()

```cpp
StageResult TrafficLightUnprotectedLeftTurnStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  // 限制巡航速度为 approach_cruise_speed（减速接近）
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(
      scenario_config.approach_cruise_speed());

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  for (const auto& overlap_id : overlap_ids) {
    PathOverlap* overlap = ...;
    reference_line_info.SetJunctionRightOfWay(overlap->start_s, false);
    if (distance_adc_to_stop_line < 0) return FinishStage(frame);
    if (signal_color != GREEN || distance >= max_valid_stop_distance) {
      traffic_light_all_done = false; break;
    }
  }
  if (traffic_light_all_done) return FinishStage(frame);
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- 与有保护场景的关键区别：**主动限速**（`approach_cruise_speed`）
- 已越过停止线（`distance < 0`）时直接完成
- `stage_approach.cc:L41-L116`

#### FinishStage()

```cpp
StageResult TrafficLightUnprotectedLeftTurnStageApproach::FinishStage(Frame* frame) {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  if (adc_speed > scenario_config.max_adc_speed_before_creep()) {
    next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE";
  } else {
    context->creep_start_time = Clock::NowInSeconds();
    next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP";
  }
  // 恢复默认巡航速度
  reference_line_info.LimitCruiseSpeed(FLAGS_default_cruise_speed);
}
```

- **速度判定**：若当前速度 > `max_adc_speed_before_creep`，跳过蠕行直接巡航
- 否则进入蠕行阶段
- `stage_approach.cc:L118-L153`

### 2. StageCreep — 蠕行阶段

> `stage_creep.cc`，继承 `BaseStageCreep`

#### Process()

- 执行 `ProcessCreep` + `ExecuteTaskOnReferenceLine`
- 设置无路权
- 蠕行完成条件：`CheckCreepDone`（距离+超时）
- **不检查信号灯颜色**（注释：don't check traffic light color while creeping）
- 完成后设置 `next_stage_ = "..._INTERSECTION_CRUISE"`
- `stage_creep.cc:L53-L127`

### 3. TrafficLightUnprotectedLeftTurnStageIntersectionCruise — 路口巡航阶段

- 与有保护场景完全相同：`ExecuteTaskOnReferenceLine` + `CheckDone`
- `stage_intersection_cruise.cc:L30-L51`

### 阶段流程

```
Stop → Creep（可跳过）→ IntersectionCruise → 场景结束
```

与左转不同，右转场景有一个 **Stop** 阶段（红灯右转需要先停车）。

### 1. StageStop — 停车判定阶段

> `stage_stop.cc`

```cpp
class TrafficLightUnprotectedRightTurnStageStop : public Stage {
 public:
  StageResult Process(...) override;
 private:
  bool CheckTrafficLightNoRightTurnOnRed(const std::string& traffic_light_id);
  StageResult FinishStage(const bool protected_mode);
};
```

#### Process()

```cpp
StageResult TrafficLightUnprotectedRightTurnStageStop::Process(...) {
  bool traffic_light_all_green = true;
  bool traffic_light_no_right_turn_on_red = false;

  for (const auto& overlap_id : overlap_ids) {
    if (distance > max_valid_stop_distance) { all_stop = false; break; }
    if (signal_color != GREEN) {
      all_green = false;
      no_right_turn_on_red = CheckTrafficLightNoRightTurnOnRed(overlap_id);
      break;
    }
  }

  // 全绿灯 → 有保护模式通过
  if (all_stop && all_green) return FinishStage(true);

  // 有"禁止红灯右转"标志
  if (no_right_turn_on_red) {
    if (distance_pass > min_pass_s_distance) return FinishStage(false);
    if (enable_right_turn_on_red && wait_time > duration) return FinishStage(false);
  } else {
    // 无禁止标志 → 可直接右转
    return FinishStage(false);
  }
}
```

- 三种通过模式：
  1. 绿灯 → `protected_mode = true`，跳过蠕行
  2. 无"禁止红灯右转" → 直接进入蠕行/巡航
  3. 有"禁止红灯右转" → 等待 `red_light_right_turn_stop_duration_sec` 后通过
- `stage_stop.cc:L41-L141`

#### CheckTrafficLightNoRightTurnOnRed()

```cpp
bool CheckTrafficLightNoRightTurnOnRed(const std::string& traffic_light_id) {
  // 检查信号灯是否有 NO_RIGHT_TURN_ON_RED 标志
  // 或者是否有 ARROW_RIGHT 子信号（箭头灯）
}
```

- 查询 HDMap 中信号灯的 `sign_info` 和 `subsignal` 属性
- `stage_stop.cc:L143-L165`

#### FinishStage()

- `protected_mode = true`：直接进入 IntersectionCruise
- `protected_mode = false`：根据速度决定进入 Creep 或 IntersectionCruise
- `stage_stop.cc:L168-L201`

### 2. StageCreep — 蠕行阶段

- 与左转蠕行逻辑完全相同
- **不检查信号灯颜色**（红灯右转时已确认可通行）
- `stage_creep.cc:L57-L131`

### 3. TrafficLightUnprotectedRightTurnStageIntersectionCruise — 路口巡航阶段

- 标准 `CheckDone` + `FinishScenario`

---

## 阶段转换对比

| 场景 | 阶段序列 | 特殊逻辑 |
|------|----------|----------|
| UnprotectedLeftTurn | Approach → Creep → IntersectionCruise | 主动限速接近；高速时跳过 Creep |
| UnprotectedRightTurn | Stop → Creep → IntersectionCruise | 红灯右转判定；禁止标志检查 |

## 调用关系

- **上游**：各自的 Scenario 类
- **下游**：`ExecuteTaskOnReferenceLine` → Task 流水线
- **共用基类**：`BaseStageCreep`（蠕行）、`BaseStageIntersectionCruise`（路口巡航）
- **依赖**：`HDMapUtil`（信号灯标志查询）、`SpeedProfileGenerator`（蠕行速度剖面）
