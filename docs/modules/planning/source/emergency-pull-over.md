# EmergencyPullOver 紧急靠边停车场景

> 源码位置：`modules/planning/scenarios/emergency_pull_over/`

## 模块定位

紧急靠边停车场景处理驾驶员通过 PadMessage 发出 `PULL_OVER` 指令时的安全停车流程。车辆需要先减速到安全速度，然后靠边停车，最后进入待命状态直到指令取消。

该场景包含三个阶段：
```
SlowDown（减速） → Approach（接近停车点） → Standby（待命）
```

## 场景类

### EmergencyPullOverContext

```cpp
struct EmergencyPullOverContext : public ScenarioContext {
  ScenarioEmergencyPullOverConfig scenario_config;
  double target_slow_down_speed = 0.0;
};
```

- `target_slow_down_speed`：计算得到的目标减速速度，在 SlowDown 阶段设置，Approach 阶段复用

### EmergencyPullOverScenario

```cpp
class EmergencyPullOverScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector>, const std::string&) override;
  EmergencyPullOverContext* GetContext() override;
  bool IsTransferable(const Scenario*, const Frame&) override;
  bool Exit(Frame* frame) override;
  bool Enter(Frame* frame) override;
};
```

### IsTransferable()

```cpp
bool EmergencyPullOverScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();
  if (pad_msg_driving_action == PadMessage::PULL_OVER) {
    return true;
  }
  return false;
}
```

- 触发条件：收到 `PadMessage::PULL_OVER` 驾驶指令
- `emergency_pull_over_scenario.cc:L54-L64`

### Enter()

- 设置 `PullOverStatus::EMERGENCY_PULL_OVER` 类型到 PlanningContext
- `emergency_pull_over_scenario.cc:L71-L77`

### Exit()

- 清除 PlanningContext 中的 pull_over 状态
- `emergency_pull_over_scenario.cc:L66-L69`

## Stage 1: SlowDown — 减速阶段

### Process()

```cpp
StageResult EmergencyPullOverStageSlowDown::Process(...) {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  double target_slow_down_speed = scenario_context->target_slow_down_speed;
  if (target_slow_down_speed <= 0) {
    target_slow_down_speed = std::max(
        scenario_config_.target_slow_down_speed(),
        adc_speed - scenario_config_.max_stop_deceleration() *
                        scenario_config_.slow_down_deceleration_time());
  }
  reference_line_info.LimitCruiseSpeed(target_slow_down_speed);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  static constexpr double kSpeedTolarence = 1.0;
  if (adc_speed - target_slow_down_speed <= kSpeedTolarence) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- 计算目标减速速度：取配置值和「当前速度 - 最大减速度 × 减速时间」的较大值
- 限制巡航速度为目标值
- 当实际速度与目标速度差 ≤ 1.0 m/s 时，认为减速完成
- `stage_slow_down.cc:L36-L69`

### FinishStage()

- 设置 `plan_pull_over_path = true`，通知路径规划任务生成靠边停车路径
- 转入 `EMERGENCY_PULL_OVER_APPROACH` 阶段
- `stage_slow_down.cc:L71-L79`

## Stage 2: Approach — 接近停车点阶段

### Process()

```cpp
StageResult EmergencyPullOverStageApproach::Process(...) {
  reference_line_info.LimitCruiseSpeed(scenario_context->target_slow_down_speed);
  reference_line_info.SetTurnSignal(VehicleSignal::TURN_RIGHT);

  // 在 pull_over 位置建立停车围栏
  if (pull_over_status.has_position()) {
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    stop_line_s = pull_over_sl.s() + stop_distance + front_edge_to_center;
    BuildStopDecision("EMERGENCY_PULL_OVER", stop_line_s, ...);
  }

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  // 检查是否到达停车点
  static constexpr double kStopSpeedTolerance = 0.4;
  static constexpr double kStopDistanceTolerance = 3.0;
  if (adc_speed <= max_adc_stop_speed + kStopSpeedTolerance &&
      fabs(distance) <= kStopDistanceTolerance) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- 保持限速并开启右转向灯
- 在 PullOverPath 任务规划的停车位置建立虚拟停车围栏
- 完成条件：车速 ≤ 停车速度阈值 + 0.4 m/s 且距停车点 ≤ 3.0m
- `stage_approach.cc:L40-L102`

### FinishStage()

- 转入 `EMERGENCY_PULL_OVER_STANDBY` 阶段
- `stage_approach.cc:L104-L107`

## Stage 3: Standby — 待命阶段

### Process()

```cpp
StageResult EmergencyPullOverStageStandby::Process(...) {
  reference_line_info.SetEmergencyLight();
  reference_line_info.SetTurnSignal(VehicleSignal::TURN_NONE);
  reference_line_info.LimitCruiseSpeed(FLAGS_default_cruise_speed);

  // 维持停车围栏（如果已越过则前推）
  if (distance <= 0.0) {
    stop_line_s = adc_front_edge_s + stop_distance;
  }
  BuildStopDecision("EMERGENCY_PULL_OVER", stop_line_s, ...);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  // 退出条件：PadMessage 不再是 PULL_OVER
  if (pad_msg_driving_action != PadMessage::PULL_OVER) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

- 开启双闪灯（`SetEmergencyLight`），关闭转向灯
- 持续维持停车围栏，防止车辆移动
- 如果车辆已越过停车点，将围栏前推到当前位置前方
- 退出条件：驾驶员取消 PULL_OVER 指令
- `stage_standby.cc:L39-L99`

## 配置项

| 字段 | 说明 |
|------|------|
| `target_slow_down_speed` | 目标减速速度 (m/s) |
| `max_stop_deceleration` | 最大停车减速度 (m/s²) |
| `slow_down_deceleration_time` | 减速时间窗口 (s) |
| `stop_distance` | 停车距离缓冲 (m) |

## 状态机流转

```mermaid
stateDiagram-v2
    [*] --> SlowDown: PadMessage::PULL_OVER
    SlowDown --> Approach: 速度达标 + plan_pull_over_path=true
    Approach --> Standby: 到达停车点
    Standby --> [*]: PadMessage != PULL_OVER
```
