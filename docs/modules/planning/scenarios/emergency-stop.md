---
title: "EmergencyStop 紧急停车场景"
---

# EmergencyStop 紧急停车场景

> 源码位置：`modules/planning/scenarios/emergency_stop/`

## 模块定位

紧急停车场景处理驾驶员通过 PadMessage 发出 `STOP` 指令时的原地紧急制动流程。与 EmergencyPullOver 不同，紧急停车**不靠边**，而是在当前车道内尽快停车。

该场景包含两个阶段：

```
Approach（制动停车） → Standby（待命）
```

## 场景类

### EmergencyStopContext

```cpp
struct EmergencyStopContext : public ScenarioContext {
  ScenarioEmergencyStopConfig scenario_config;
};
```

### EmergencyStopScenario

```cpp
class EmergencyStopScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector>, const std::string&) override;
  EmergencyStopContext* GetContext() override;
  bool IsTransferable(const Scenario*, const Frame&) override;
  ScenarioResult Process(const TrajectoryPoint&, Frame*) override;
  bool Exit(Frame* frame);
};
```

### IsTransferable()

```cpp
bool EmergencyStopScenario::IsTransferable(...) {
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();
  return pad_msg_driving_action == PadMessage::STOP;
}
```

- 触发条件：收到 `PadMessage::STOP` 驾驶指令
- `emergency_stop_scenario.cc:L51-L58`

### Process()

- 重写 `Scenario::Process()`，增加参考线为空时的错误处理
- `emergency_stop_scenario.cc:L60-L70`

### Exit()

- 清除 PlanningContext 中的 emergency_stop 状态
- `emergency_stop_scenario.cc:L72-L78`

## Stage 1: Approach — 制动阶段

### Process()

```cpp
StageResult EmergencyStopStageApproach::Process(...) {
  // 1. 开启双闪灯
  frame->mutable_reference_line_info()->front().SetEmergencyLight();

  // 2. 计算或复用停车围栏位置
  if (emergency_stop_status.has_stop_fence_point()) {
    // 复用已有围栏（如果仍在前方）
    stop_line_s = stop_fence_sl.s();
  } else {
    // 首次计算：基于运动学公式
    const double travel_distance = ceil(v² / (2 * deceleration));
    stop_line_s = adc_front_edge_s + travel_distance + stop_distance + kBuffer;
    // 持久化到 PlanningContext
    emergency_stop_fence_point->set_x(...);
    emergency_stop_fence_point->set_y(...);
  }

  // 3. 建立停车围栏
  BuildStopDecision("EMERGENCY_STOP", stop_line_s, ...);

  // 4. 执行任务链
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  // 5. 完成条件：车速 ≤ 停车速度阈值
  if (adc_speed <= max_adc_stop_speed) {
    return FinishStage();
  }
}
```

- 停车距离计算：`v² / (2a)` + `stop_distance` + 2m 缓冲
- 停车围栏位置持久化到 `PlanningContext::emergency_stop::stop_fence_point`，确保跨帧一致
- 完成条件：车速降至 `max_abs_speed_when_stopped`（通常 0.2 m/s）
- `stage_approach.cc:L38-L114`

### FinishStage()

- 转入 `EMERGENCY_STOP_STANDBY` 阶段
- `stage_approach.cc:L116-L119`

## Stage 2: Standby — 待命阶段

### Process()

```cpp
StageResult EmergencyStopStageStandby::Process(...) {
  // 1. 维持停车围栏
  if (emergency_stop_status.has_stop_fence_point()) {
    stop_line_s = stop_fence_sl.s();  // 复用已有位置
  } else {
    stop_line_s = adc_front_edge_s + stop_distance + kBuffer;
  }
  BuildStopDecision("EMERGENCY_STOP", stop_line_s, ...);

  // 2. 执行任务链
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  // 3. 退出条件：PadMessage 不再是 STOP
  if (pad_msg_driving_action != PadMessage::STOP) {
    return FinishStage();
  }
}
```

- 持续维持停车围栏，防止车辆移动
- 退出条件：驾驶员取消 STOP 指令
- `stage_standby.cc:L36-L98`

## 与 EmergencyPullOver 的区别

| 特性 | EmergencyStop | EmergencyPullOver |
|------|--------------|-------------------|
| 触发指令 | `PadMessage::STOP` | `PadMessage::PULL_OVER` |
| 停车方式 | 原地制动 | 靠边停车 |
| 阶段数 | 2（制动→待命） | 3（减速→接近→待命） |
| 路径规划 | 不生成新路径 | 生成靠边路径 |
| 转向灯 | 无（仅双闪） | 右转向灯→双闪 |

## 配置项

| 字段 | 说明 |
|------|------|
| `max_stop_deceleration` | 最大制动减速度 (m/s²) |
| `stop_distance` | 停车距离缓冲 (m) |

## 状态机流转

```mermaid
stateDiagram-v2
    [*] --> Approach: PadMessage::STOP
    Approach --> Standby: 车速 ≤ 停车阈值
    Standby --> [*]: PadMessage != STOP
```
