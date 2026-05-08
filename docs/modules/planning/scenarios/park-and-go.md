---
title: "即停即走场景"
---

# Park And Go

> 源码路径: `modules/planning/scenarios/park_and_go/`

## 概述

Park And Go（即停即走）场景处理车辆从非道路区域（如停车场、路边停车）静止状态启动并汇入正常车道行驶的完整流程。该场景适用于车辆不在城市驾驶车道上或未在车道上、且与目的地距离较远时的起步规划。

场景包含四个阶段，按顺序执行：

1. **CHECK** -- 初始化 ADC 状态，检查车辆是否可直接进入巡航
2. **ADJUST** -- 在开放空间内调整车辆位姿，使朝向和位置对齐参考线
3. **PRE_CRUISE** -- 在开放空间轨迹上继续调整，确保转向角满足巡航条件
4. **CRUISE** -- 切换到车道跟随模式，沿参考线行驶直至横向偏差收敛

场景触发条件（`IsTransferable`）：

- 存在 `lane_follow_command`
- 车辆静止（速度低于 `max_abs_speed_when_stopped`）
- 距目的地超过 `min_dist_to_dest`
- 车辆不在城市驾驶车道上，或不在车道中心线上

## 核心类

| 类名 | 文件 | 说明 |
|------|------|------|
| `ParkAndGoScenario` | `park_and_go_scenario.h/cc` | 场景主类，继承 `Scenario`，负责初始化配置和判断场景可转移性 |
| `ParkAndGoContext` | `context.h` | 场景上下文，持有 `ScenarioParkAndGoConfig` 配置 |
| `ParkAndGoStageCheck` | `stage_check.h/cc` | CHECK 阶段，初始化 ADC 位姿并判断是否可直接巡航 |
| `ParkAndGoStageAdjust` | `stage_adjust.h/cc` | ADJUST 阶段，通过开放空间轨迹调整车辆位姿 |
| `ParkAndGoStagePreCruise` | `stage_pre_cruise.h/cc` | PRE_CRUISE 阶段，继续微调至转向角满足巡航阈值 |
| `ParkAndGoStageCruise` | `stage_cruise.h/cc` | CRUISE 阶段，车道跟随行驶直至横向偏差收敛 |

`ParkAndGoStageCruise` 内部定义了状态枚举：

```cpp
enum ParkAndGoStatus {
  CRUISING = 1,
  CRUISE_COMPLETE = 2,
  ADJUST = 3,
  ADJUST_COMPLETE = 4,
  FAIL = 5,
};
```

## 核心函数

### ParkAndGoScenario

- `Init(injector, name)` -- 加载 `ScenarioParkAndGoConfig` 配置并初始化场景
- `IsTransferable(other_scenario, frame)` -- 判断当前帧是否满足进入即停即走场景的条件：车辆静止、距目的地足够远、不在城市驾驶车道上

### ParkAndGoStageCheck

- `Process(planning_init_point, frame)` -- 调用 `ADCInitStatus` 记录初始位姿，执行开放空间任务，通过 `CheckADCReadyToCruise` 判断能否直接巡航
- `ADCInitStatus()` -- 将当前车辆位置和朝向写入 `planning_status->park_and_go`
- `FinishStage(success)` -- 成功则下一阶段为 `PARK_AND_GO_CRUISE`，否则为 `PARK_AND_GO_ADJUST`

### ParkAndGoStageAdjust

- `Process(planning_init_point, frame)` -- 执行开放空间轨迹规划，循环检查是否就绪或轨迹已结束
- `FinishStage()` -- 若转向角低于阈值则进入 CRUISE，否则重置初始位姿进入 PRE_CRUISE
- `ResetInitPostion()` -- 用当前车辆状态刷新 `park_and_go` 中的初始位姿

### ParkAndGoStagePreCruise

- `Process(planning_init_point, frame)` -- 执行开放空间任务，当转向角低于 `max_steering_percentage_when_cruise` 且满足巡航就绪条件时完成
- `FinishStage()` -- 下一阶段为 `PARK_AND_GO_CRUISE`

### ParkAndGoStageCruise

- `Process(planning_init_point, frame)` -- 执行车道跟随任务，检查横向偏差是否收敛
- `CheckADCParkAndGoCruiseCompleted(reference_line_info)` -- 当 ADC 横向偏差 `|l| < 0.5m` 时判定巡航完成

### 工具函数（util.h/cc）

- `CheckADCReadyToCruise(vehicle_state_provider, frame, config)` -- 综合判断：挡位为前进挡或车速极低、前方无障碍、朝向对齐参考线、横向偏差 < 0.5m
- `CheckADCSurroundObstacles(adc_position, adc_heading, frame, buffer)` -- 以 ADC 包围盒（含前向缓冲）检测是否与障碍物重叠
- `CheckADCHeading(adc_position, adc_heading, reference_line_info, heading_buffer)` -- 检查 ADC 朝向与参考线切线方向的角度差是否在容许范围内

## 配置

配置项定义于 `proto/park_and_go.proto` 中的 `ScenarioParkAndGoConfig`：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `front_obstacle_buffer` | double | 4.0 m | 前方障碍物距离缓冲区，用于判断前方是否被阻挡 |
| `heading_buffer` | double | 0.5 rad | 朝向容许偏差，ADC 朝向与参考线角度差需小于此值 |
| `min_dist_to_dest` | double | 25.0 m | 触发场景的最小距目的地距离，低于此值不进入该场景 |
| `max_steering_percentage_when_cruise` | double | 90.0 | 转向百分比阈值，低于此值才允许切换到巡航阶段 |

实际运行配置（`conf/scenario_conf.pb.txt`）：

| 字段 | 运行值 |
|------|--------|
| `front_obstacle_buffer` | 10.0 m |
| `heading_buffer` | 0.3 rad |
| `min_dist_to_dest` | 10.0 m |
| `max_steering_percentage_when_cruise` | 20.0 |

## 调用关系

场景按流水线（`pipeline.pb.txt`）定义的顺序依次执行四个阶段：

```text
ParkAndGoScenario::IsTransferable()
  |
  v
PARK_AND_GO_CHECK (ParkAndGoStageCheck)
  |  ADCInitStatus() + CheckADCReadyToCruise()
  |-- 就绪 --> PARK_AND_GO_CRUISE
  |-- 未就绪 --> PARK_AND_GO_ADJUST
  v
PARK_AND_GO_ADJUST (ParkAndGoStageAdjust)
  |  ExecuteTaskOnOpenSpace() + CheckADCReadyToCruise()
  |  循环直到就绪或轨迹结束
  |-- 转向角小 --> PARK_AND_GO_CRUISE
  |-- 转向角大 --> ResetInitPostion() -> PARK_AND_GO_PRE_CRUISE
  v
PARK_AND_GO_PRE_CRUISE (ParkAndGoStagePreCruise)
  |  ExecuteTaskOnOpenSpace() + CheckADCReadyToCruise()
  |  循环直到转向角 < max_steering_percentage_when_cruise
  v
PARK_AND_GO_CRUISE (ParkAndGoStageCruise)
  |  ExecuteTaskOnReferenceLine()
  |  CheckADCParkAndGoCruiseCompleted()
  |-- |l| < 0.5m --> 场景完成，切换到 LANE_FOLLOW
```

CHECK / ADJUST / PRE_CRUISE 三个阶段均使用开放空间规划任务链：
`OpenSpaceRoiDecider` -> `OpenSpaceTrajectoryProvider` -> `OpenSpaceTrajectoryPartition` -> `OpenSpaceFallbackDecider`

CRUISE 阶段使用车道跟随任务链：
`LaneFollowPath` -> `LaneBorrowPath` -> `FallbackPath` -> `PathDecider` -> `RuleBasedStopDecider` -> `STBoundsDecider` -> `SpeedBoundsDecider` -> `PathTimeHeuristicOptimizer` -> `SpeedDecider` -> `SpeedBoundsDecider` -> `PiecewiseJerkSpeedOptimizer` -> `RssDecider`
