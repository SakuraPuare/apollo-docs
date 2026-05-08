# PullOver 靠边停车场景阶段详解

> 源码位置：`modules/planning/scenarios/pull_over/`

## 模块定位

PullOver 场景处理车辆到达目的地时的靠边停车流程。先通过参考线规划接近停车点，若路径规划失败则切换到开放空间规划进行精确泊入。

### 阶段流程

```
Approach → [成功] → 场景结束
    │
    └─[失败]→ RetryApproachParking → RetryParking → 场景结束
```

## 场景上下文

```cpp
struct PullOverContext : public ScenarioContext {
  ScenarioPullOverConfig scenario_config;
};
```

通过 `PlanningContext::pull_over()` 持久化：
- `position`：目标停车位置 (x, y)
- `theta`：目标停车航向
- `length_front/back`、`width_left/right`：停车位尺寸

## 状态枚举

```cpp
enum PullOverState {
  UNKNOWN, APPROACHING, PASS_DESTINATION, PARK_COMPLETE, PARK_FAIL
};
```

---

## 1. PullOverStageApproach — 接近阶段

> `stage_approach.cc`

### Process()

```cpp
StageResult PullOverStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  PullOverState state = CheckADCPullOver(...);
  if (state == PASS_DESTINATION || state == PARK_COMPLETE)
    return FinishStage(true);   // 成功
  if (state == PARK_FAIL)
    return FinishStage(false);  // 失败，进入重试

  // 检查路径数据是否提前失败
  for (const auto& path_data : candidate_path_data) {
    if (path_data.path_label().find("pullover") == npos) break;
    PullOverState state = CheckADCPullOverPathPoint(...);
    if (state == PARK_FAIL) path_fail = true;
  }

  // 路径失败时添加停车围栏，等待切换到开放空间
  if (path_fail) {
    BuildStopDecision("DEST_PULL_OVER_PREPARKING", stop_line_s, ...);
    if (distance <= 1.0m && angle <= 0.2rad)
      return FinishStage(false);
  }
  return RUNNING;
}
```

- 正常情况：参考线规划 + pullover 路径直接停到位
- 失败处理：检测路径末端是否能到达目标点，不能则添加预停车围栏
- 预停车围栏距目标点 `s_distance_to_stop_for_open_space_parking`
- `stage_approach.cc:L40-L137`

### FinishStage()

- `success = true` → `FinishScenario()`
- `success = false` → `next_stage_ = "PULL_OVER_RETRY_APPROACH_PARKING"`
- `stage_approach.cc:L139-L146`

---

## 2. PullOverStageRetryApproachParking — 重试接近阶段

> `stage_retry_approach_parking.cc`

### Process()

```cpp
StageResult PullOverStageRetryApproachParking::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (CheckADCStop(*frame)) return FinishStage();
  return RUNNING;
}
```

- 使用参考线规划驶向预停车点
- 停稳后进入开放空间泊车阶段
- `stage_retry_approach_parking.cc:L39-L55`

### CheckADCStop()

```cpp
bool PullOverStageRetryApproachParking::CheckADCStop(const Frame& frame) {
  // 1. 速度 < max_abs_speed_when_stopped
  // 2. 距停车围栏 < max_valid_stop_distance
}
```

- 停车围栏位置从 `open_space_info().open_space_pre_stop_fence_s()` 获取
- `stage_retry_approach_parking.cc:L57-L83`

### FinishStage()

- `next_stage_ = "PULL_OVER_RETRY_PARKING"`
- `stage_retry_approach_parking.cc:L34-L37`

---

## 3. PullOverStageRetryParking — 开放空间泊车阶段

> `stage_retry_parking.cc`

### Process()

```cpp
StageResult PullOverStageRetryParking::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);

  // 设置调试信息
  pull_over_debug->mutable_position()->CopyFrom(pull_over_status.position());
  pull_over_debug->set_theta(pull_over_status.theta());
  ...

  if (CheckADCPullOverOpenSpace()) return FinishStage();
  return RUNNING;
}
```

- 使用开放空间规划精确泊入目标位置
- `stage_retry_parking.cc:L37-L72`

### CheckADCPullOverOpenSpace()

```cpp
bool PullOverStageRetryParking::CheckADCPullOverOpenSpace() {
  const double distance_diff = adc_position.DistanceTo(target_position);
  const double theta_diff = |NormalizeAngle(target_theta - adc_heading)|;
  return (distance_diff <= max_distance_error_to_end_point &&
          theta_diff <= max_theta_error_to_end_point);
}
```

- 同时检查位置距离和航向角差
- `stage_retry_parking.cc:L78-L104`

---

## 工具函数（util.cc）

### CheckADCPullOver()

```cpp
PullOverState CheckADCPullOver(vehicle_state, reference_line_info,
                                scenario_config, planning_context) {
  // 1. 检查是否已越过目标点 → PASS_DESTINATION
  // 2. 检查是否已停车 → 未停则 APPROACHING
  // 3. 检查距离是否在检查范围内（3m）
  // 4. 通过 SL 坐标检查位置精度 → PARK_COMPLETE / PARK_FAIL
}
```

- `pass_destination_threshold`：越过目标点的阈值
- `kStartParkCheckRange = 3.0m`：开始精确检查的距离
- `util.cc:L35-L89`

### CheckADCPullOverPathPoint()

- 对路径规划末端点进行位置检查（仅检查 l 和 theta，不检查 s）
- `util.cc:L94-L118`

### CheckPullOverPositionBySL()

```cpp
bool CheckPullOverPositionBySL(reference_line_info, scenario_config,
    adc_position, adc_theta, target_position, target_theta, check_s) {
  // l_diff <= max_l_error_to_end_point
  // theta_diff <= max_theta_error_to_end_point
  // (可选) s_diff >= 0 && s_diff <= max_s_error_to_end_point
}
```

- 核心位置检查函数，被上述两个函数调用
- `util.cc:L120-L151`

---

## 调用关系

- **上游**：`PullOverScenario` 管理阶段切换
- **Approach**：`ExecuteTaskOnReferenceLine`（含 pullover 路径生成）
- **RetryParking**：`ExecuteTaskOnOpenSpace`（开放空间精确泊车）
- **依赖**：`PlanningContext::pull_over()`、`OpenSpaceInfo`、`BuildStopDecision`
