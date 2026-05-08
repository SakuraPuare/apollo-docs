---
title: "Lane Follow"
---

# Lane Follow Scenario

> 源码路径: `modules/planning/scenarios/lane_follow/`

## 概述

Lane Follow 是 Apollo 规划模块中最基础、最高频的驾驶场景，负责在当前车道内沿参考线行驶。当规划命令（PlanningCommand）包含 `lane_follow_command` 且存在有效参考线时，系统会切入该场景。

场景采用单阶段（Single-Stage）架构：`LaneFollowScenario` 管理场景入口与转移条件，`LaneFollowStage` 执行实际的路径和速度规划。阶段内按配置顺序依次调用 Task 链（路径规划 -> 速度规划），并支持路径/速度回退（Fallback）策略保障安全性。

## 核心类

### LaneFollowScenario

继承自 `Scenario`，是场景注册与转移的入口。

```cpp
class LaneFollowScenario : public Scenario {
 public:
  ScenarioContext* GetContext() override { return nullptr; }
  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
};
```

- `GetContext()` -- 返回 `nullptr`，该场景无需额外上下文信息
- `IsTransferable()` -- 判断是否可切入本场景，需同时满足：
  1. `planning_command` 包含 `lane_follow_command`
  2. `reference_line_info` 非空
- 通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册为 `Scenario` 插件

### LaneFollowStage

继承自 `Stage`，负责在参考线上执行路径与速度规划。

```cpp
class LaneFollowStage : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
  StageResult PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);
  void PlanFallbackTrajectory(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);
  common::SLPoint GetStopSL(const ObjectStop& stop_decision,
                            const ReferenceLine& reference_line) const;
  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);
};
```

- 同样通过插件机制注册为 `Stage` 插件

## 核心函数

### LaneFollowStage::Process

阶段主入口函数，遍历所有参考线并调用规划：

1. 若 `reference_line_info` 为空，直接返回 `FINISHED`
2. 遍历每条参考线，依次调用 `PlanOnReferenceLine`
3. 对于非变道参考线，一旦规划成功即标记为可行驶（drivable）
4. 对于变道参考线，仅当代价低于 `kStraightForwardLineCost`（10.0）时才标记可行驶
5. 最终根据是否有可行驶参考线返回 `RUNNING` 或 `ERROR`

### LaneFollowStage::PlanOnReferenceLine

单条参考线上的完整规划流程：

1. 非变道参考线自动增加 10.0 的基础代价，使其在多参考线竞争中被优先选择
2. 按顺序执行 `task_list_` 中所有 Task，每个 Task 通过 `Execute()` 执行并记录耗时
3. 若某个 Task 报错，中断 Task 链并调用 `fallback_task_` 执行回退
4. 调用 `CombinePathAndSpeedProfile` 聚合路径与速度曲线为完整轨迹
5. 计算目的地停车点 `dest_stop_s`，对路径上静态障碍物的停车决策添加额外代价（`1e3`），避免参考线因靠近静态障碍而被过度惩罚
6. 若开启 `FLAGS_enable_trajectory_check`，通过 `ConstraintChecker` 校验轨迹合法性
7. 最终将轨迹设置到 `reference_line_info` 中

### LaneFollowStage::RecordObstacleDebugInfo

记录障碍物调试信息。遍历 `path_decision` 中所有障碍物，将 SL 边界、决策标签和决策内容写入 debug 数据。受 `FLAGS_enable_record_debug` 标志控制。

### LaneFollowStage::GetStopSL

将障碍物停车决策的笛卡尔坐标（XY）转换为参考线坐标系（SL），用于停车代价计算。

## 配置

场景通过 `conf/pipeline.pb.txt` 定义阶段和 Task 链：

| Task 名称 | 类型 | 说明 |
|-----------|------|------|
| LANE_CHANGE_PATH | LaneChangePath | 变道路径规划 |
| LANE_FOLLOW_PATH | LaneFollowPath | 跟车路径规划 |
| LANE_BORROW_PATH | LaneBorrowPath | 借道路径规划 |
| FALLBACK_PATH | FallbackPath | 路径回退 |
| PATH_DECIDER | PathDecider | 路径决策 |
| RULE_BASED_STOP_DECIDER | RuleBasedStopDecider | 基于规则的停车决策 |
| SPEED_BOUNDS_PRIORI_DECIDER | SpeedBoundsDecider | 先验速度边界决策 |
| SPEED_HEURISTIC_OPTIMIZER | PathTimeHeuristicOptimizer | 速度启发式优化 |
| SPEED_DECIDER | SpeedDecider | 速度决策 |
| SPEED_BOUNDS_FINAL_DECIDER | SpeedBoundsDecider | 最终速度边界决策 |
| PIECEWISE_JERK_SPEED | PiecewiseJerkSpeedOptimizer | 分段 Jerk 速度优化 |

Task 链遵循"路径规划 -> 路径决策 -> 速度边界 -> 速度优化"的两阶段流水线模式。

## 调用关系

```text
LaneFollowScenario::IsTransferable()
  |-- 检查 lane_follow_command 存在性
  |-- 检查 reference_line_info 非空
  |
LaneFollowStage::Process()
  |-- 遍历参考线:
  |     |-- PlanOnReferenceLine()
  |           |-- Task 链顺序执行 (task_list_)
  |           |     |-- LaneChangePath / LaneFollowPath / LaneBorrowPath
  |           |     |-- PathDecider
  |           |     |-- RuleBasedStopDecider
  |           |     |-- SpeedBoundsDecider (priori)
  |           |     |-- PathTimeHeuristicOptimizer
  |           |     |-- SpeedDecider
  |           |     |-- SpeedBoundsDecider (final)
  |           |     |-- PiecewiseJerkSpeedOptimizer
  |           |-- fallback_task_ (Task 失败时)
  |           |-- CombinePathAndSpeedProfile() 聚合轨迹
  |           |-- ConstraintChecker::ValidTrajectory() 校验
  |           |-- RecordObstacleDebugInfo() 记录调试信息
  |
  |-- 返回 StageResult (RUNNING / ERROR)
```
