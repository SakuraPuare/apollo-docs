# ParkAndGo 驶出停车位场景阶段详解

> 源码位置：`modules/planning/scenarios/park_and_go/`

## 模块定位

ParkAndGo 场景处理车辆从停车位驶出并汇入车道的完整流程。使用开放空间规划（Open Space）进行姿态调整，直到车辆航向与参考线对齐后切换到常规车道巡航。

### 阶段流程

```
Check → Adjust（可循环）→ PreCruise → Cruise → 场景结束
         ↑                    │
         └────────────────────┘（方向盘角度过大时回退）
```

## 场景上下文

```cpp
struct ParkAndGoContext : public ScenarioContext {
  ScenarioParkAndGoConfig scenario_config;
};
```

配置通过 `PlanningContext::park_and_go()` 持久化以下状态：
- `adc_init_position`：初始位置（x, y, z）
- `adc_init_heading`：初始航向
- `in_check_stage`：是否处于检查阶段

---

## 1. ParkAndGoStageCheck — 初始检查阶段

> `stage_check.cc`

```cpp
class ParkAndGoStageCheck : public Stage {
 public:
  StageResult Process(...) override;
 private:
  StageResult FinishStage(const bool success);
  void ADCInitStatus();
};
```

### Process()

```cpp
StageResult ParkAndGoStageCheck::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADCInitStatus();  // 记录初始位姿
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  bool ready_to_cruise = CheckADCReadyToCruise(...);
  return FinishStage(ready_to_cruise);
}
```

- 记录自车初始位姿到 `PlanningContext`
- 使用开放空间任务流水线规划
- 若已满足巡航条件 → 直接进入 Cruise；否则 → Adjust
- `stage_check.cc:L29-L47`

### ADCInitStatus()

- 将当前车辆 x/y/heading 写入 `park_and_go_status`
- 设置 `in_check_stage = true`
- `stage_check.cc:L62-L75`

### FinishStage()

- `success = true` → `next_stage_ = "PARK_AND_GO_CRUISE"`
- `success = false` → `next_stage_ = "PARK_AND_GO_ADJUST"`
- `stage_check.cc:L49-L60`

---

## 2. ParkAndGoStageAdjust — 姿态调整阶段

> `stage_adjust.cc`

```cpp
class ParkAndGoStageAdjust : public Stage {
 public:
  StageResult Process(...) override;
 private:
  StageResult FinishStage();
  void ResetInitPostion();
};
```

### Process()

```cpp
StageResult ParkAndGoStageAdjust::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);

  const bool is_ready_to_cruise = CheckADCReadyToCruise(...);
  // 检查轨迹是否已执行完毕
  bool is_end_of_trajectory =
      (trajectory_points.rbegin()->relative_time() < 0.0);

  if (!is_ready_to_cruise && !is_end_of_trajectory)
    return RUNNING;
  return FinishStage();
}
```

- 持续执行开放空间规划直到满足巡航条件或轨迹执行完毕
- `stage_adjust.cc:L30-L59`

### FinishStage()

```cpp
StageResult ParkAndGoStageAdjust::FinishStage() {
  if (std::fabs(vehicle_status->steering_percentage()) <
      scenario_config.max_steering_percentage_when_cruise()) {
    next_stage_ = "PARK_AND_GO_CRUISE";
  } else {
    ResetInitPostion();
    next_stage_ = "PARK_AND_GO_PRE_CRUISE";
  }
}
```

- 方向盘角度小 → 直接巡航
- 方向盘角度大 → 进入 PreCruise 继续调整
- `stage_adjust.cc:L62-L74`

### ResetInitPostion()

- 更新 `adc_init_position` 和 `adc_init_heading` 为当前值
- `stage_adjust.cc:L76-L87`

---

## 3. ParkAndGoStagePreCruise — 预巡航阶段

> `stage_pre_cruise.cc`

### Process()

```cpp
StageResult ParkAndGoStagePreCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);

  if ((std::fabs(steering_percentage) < max_steering_percentage_when_cruise)
      && CheckADCReadyToCruise(...)) {
    return FinishStage();
  }
  return RUNNING;
}
```

- 同时检查方向盘角度和巡航就绪条件
- 满足后进入 Cruise
- `stage_pre_cruise.cc:L32-L60`

---

## 4. ParkAndGoStageCruise — 巡航阶段

> `stage_cruise.cc`

### Process()

```cpp
StageResult ParkAndGoStageCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  ParkAndGoStatus status = CheckADCParkAndGoCruiseCompleted(reference_line_info);
  if (status == CRUISE_COMPLETE) return FinishStage();
  return RUNNING;
}
```

- 切换到**参考线规划**（非开放空间）
- 完成条件：自车横向偏移 `|l| < 0.5m`
- `stage_cruise.cc:L30-L51`

### CheckADCParkAndGoCruiseCompleted()

- 将自车位置投影到参考线，检查横向偏移
- `kLBuffer = 0.5m`
- `stage_cruise.cc:L55-L91`

---

## 工具函数（util.cc）

### CheckADCReadyToCruise()

```cpp
bool CheckADCReadyToCruise(vehicle_state, frame, scenario_config) {
  // 条件：
  // 1. 档位为 D 档（或速度极低）
  // 2. 前方无近距离障碍物
  // 3. 航向与参考线对齐
  // 4. 横向偏移 |l| < 0.5m
}
```

- 找最近参考线（按横向距离排序）
- `util.cc:L28-L66`

### CheckADCSurroundObstacles()

- 构建自车前方扩展包围盒（加 `front_obstacle_buffer`）
- 检查是否与任何非虚拟障碍物多边形重叠
- `util.cc:L72-L103`

### CheckADCHeading()

- 计算自车航向与参考线航向的差值
- 非对称阈值：`[-heading_buffer + 0.2, heading_buffer]`
- `util.cc:L109-L131`

---

## 调用关系

- **上游**：`ParkAndGoScenario` 管理阶段切换
- **Check/Adjust/PreCruise**：使用 `ExecuteTaskOnOpenSpace`（开放空间规划）
- **Cruise**：使用 `ExecuteTaskOnReferenceLine`（常规参考线规划）
- **依赖**：`OpenSpaceInfo`、`VehicleStateProvider`、`ReferenceLineInfo`
