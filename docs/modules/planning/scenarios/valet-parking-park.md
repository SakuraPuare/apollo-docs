---
title: "代客泊车场景"
---

# Valet Parking Park

> 源码路径: `modules/planning/scenarios/valet_parking_park/`

## 概述

代客泊车场景 (Valet Parking Park) 实现代客泊车的完整规划流程。收到路由下发的泊车指令后，从车道巡航切换到泊车模式，依次完成接近车位、执行泊车轨迹规划、泊车精度校验三个阶段。若精度不满足要求，自动进入重试阶段。场景采用 `Scenario` / `Stage` 两层架构，开放空间 (Open Space) 规划器负责生成泊入轨迹。

## 核心类

### ValetParkingContext

场景上下文，继承自 `ScenarioContext`，在泊车全流程中传递状态：目标车位 ID (`target_parking_spot_id`)、预停车标志 (`pre_stop_rightaway_flag`)、预停车点 (`pre_stop_rightaway_point`)、指令序列号 (`command_sequence_num`) 及场景配置 (`scenario_config`)。

### ValetParkingParkScenario

场景主类，继承自 `Scenario`。负责场景初始化（加载配置、获取高精地图指针）、入场条件判断 (`IsTransferable`)。入场条件包括：存在泊车指令、参考线有效、在路径上搜索到目标车位 (`SearchTargetParkingSpotOnPath`)、车辆与车位距离在配置范围内 (`CheckDistanceToParkingSpot`)。

### StageApproachingParkingSpotPark

第一阶段：接近车位。沿参考线巡航接近目标泊车位，将目标车位 ID 和预停车信息写入 `OpenSpaceInfo`，忽略目的地障碍物决策，执行参考线规划任务。车辆停下且在有效范围内后转入泊车阶段。

- `CheckADCStop`：检查车辆线速度是否低于 `max_abs_speed_when_stopped`
- `GetTargetS`：获取目标车位中心在参考线上的 `s` 坐标
- `CheckADCInParkingRange`：检查车辆前端与停车围栏距离是否在 `max_valid_stop_distance` 内

### StageParkingPark

第二阶段：执行泊车。调用 `ExecuteTaskOnOpenSpace` 生成泊车轨迹。到达目标后通过 `CheckADCParkingCompleted` 校验精度（横向误差、纵向误差、航向角误差），不满足则切换到重试阶段。通过 `CheckParkingMissionInfo` 跟踪泊车任务状态，防止重复执行。

### StageParkingRetryPark

第三阶段：泊车重试。当第二阶段精度不足时进入，重新执行开放空间规划修正泊入位置。到达目标后直接标记完成，通过 `CheckParkingAccuracy` 记录精度日志但不影响流程。

### ParkingMissionInfo / ParkingMissionStatus

泊车任务状态管理：`RUNNING`（执行中）/ `DONE`（已完成）。`ParkingMissionInfo` 记录车位 ID、指令序列号和状态，用于判断是否需要重新执行泊车任务。

## 核心函数

| 函数 | 类 | 说明 |
|------|-----|------|
| `Init` | ValetParkingParkScenario | 加载 `ScenarioValetParkingParkConfig` 配置，获取高精地图指针 |
| `IsTransferable` | ValetParkingParkScenario | 判断入场条件：泊车指令有效、目标车位在路径上、距离在范围内 |
| `SearchTargetParkingSpotOnPath` | ValetParkingParkScenario | 在参考线 `parking_space_overlaps` 中匹配目标车位 ID |
| `CheckDistanceToParkingSpot` | ValetParkingParkScenario | 计算车位中心与车辆在参考线上的 `s` 值差，判断是否在范围内 |
| `Process` | StageApproachingParkingSpotPark | 接近阶段主流程，执行参考线规划，车辆停下且在范围内则切换阶段 |
| `Process` | StageParkingPark | 泊车主流程，调用开放空间规划，到达后校验精度决定完成或重试 |
| `CheckADCParkingCompleted` | StageParkingPark | 校验泊入精度：横向 < `max_lat_error`、纵向 < `max_lon_error`、航向 < `max_heading_error`；平行泊车未启用重试时直接通过 |
| `CheckParkingMissionInfo` | StageParkingPark | 检查车位 ID 或指令序列号是否变化，判断是否需要重新泊车 |
| `Process` | StageParkingRetryPark | 重试主流程，执行开放空间规划，到达后直接标记完成 |
| `CheckParkingAccuracy` | StageParkingRetryPark | 计算并输出横向、纵向和航向角误差日志 |

## 配置

配置定义在 `proto/valet_parking_park.proto`，消息类型 `ScenarioValetParkingParkConfig`：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `parking_spot_range_to_start` | double | 20.0 m | 进入场景的最大车辆-车位距离 |
| `max_valid_stop_distance` | double | 1.0 m | 接近阶段最大有效停车距离 |
| `max_heading_error` | double | 0.1 rad | 航向角误差阈值，超过触发重试 |
| `max_lat_error` | double | 0.3 m | 横向误差阈值，超过触发重试 |
| `max_lon_error` | double | 0.1 m | 纵向误差阈值，超过触发重试 |
| `enable_parallel_retry_parking` | bool | false | 是否允许平行泊车重试 |

运行时配置 (`conf/scenario_conf.pb.txt`) 实测值：`max_heading_error: 0.08`，`max_lon_error: 0.15`，其余与默认值一致。

## 调用关系

流水线 (`conf/pipeline.pb.txt`) 定义三个阶段：

```text
ValetParkingParkScenario
├── Stage 1: APPROACHING_PARKING_SPOT_PARK (接近车位)
│   ├── OpenSpacePreStopDecider / ObstacleNudgeDecider
│   ├── LaneChangePathGeneric / LaneFollowPath / LaneBorrowPathGeneric
│   ├── FallbackPath / PathDecider / RuleBasedStopDecider
│   ├── SpeedBoundsDecider → PathTimeHeuristicOptimizer
│   ├── SpeedDecider / SpeedBoundsDecider / PiecewiseJerkSpeedOptimizer
│   └── fallback: SmoothStopTrajectoryFallback
├── Stage 2: PARKING_PARK (执行泊车)
│   ├── OpenSpaceReplanDecider / OpenSpaceRoiDecider
│   ├── OpenSpacePathPlanning / OpenSpaceTrajectoryOptimizerPark
│   └── OpenSpaceTrajectoryPostProcess / OpenSpaceFallbackDeciderPark
└── Stage 3: RETRY_PARK (精度不足重试，任务同 Stage 2)
```

阶段切换逻辑：

1. **Stage 1 -> Stage 2**：`CheckADCStop` 且 `CheckADCInParkingRange` 均为 true
2. **Stage 2 -> 完成**：到达车位且 `CheckADCParkingCompleted` 通过
3. **Stage 2 -> Stage 3**：到达车位但精度不足（非未启用重试的平行泊车）
4. **Stage 3 -> 完成**：到达车位后直接标记完成，记录精度日志

所有场景和阶段类通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册为 CyberRT 插件。
