---
title: "Lane Follow Park"
---

# Lane Follow Park Scenario

> 源码路径: `modules/planning/scenarios/lane_follow_park/`

## 概述

`LaneFollowParkScenario` 是泊车场景下的车道跟随场景，在标准车道跟随的基础上增加了障碍物阻塞检测与脱困决策能力。当前方存在稳定阻塞的障碍物且满足脱困条件时，场景会触发倒车脱困流程。

该场景通过 `ParkDataCenter` 共享数据，利用 `LaneEscapeUtil` 工具类判断是否需要脱困，并通过流水线阶段机制切换正常跟车与脱困模式。

插件通过 `plugins.xml` 注册 `LaneFollowParkScenario`（场景）和 `LaneFollowParkStage`（阶段）两个类。

## 核心类

### LaneFollowParkContext

继承自 `ScenarioContext`，承载场景专属配置。

```cpp
struct LaneFollowParkContext : public ScenarioContext {
  ScenarioLaneFollowParkConfig scenario_config;
};
```

### LaneFollowParkScenario

场景入口类，继承自 `Scenario`，负责场景的初始化和转移判断。

```cpp
class LaneFollowParkScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;
  ScenarioContext* GetContext() override { return &context_; }
  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
 private:
  LaneFollowParkContext context_;
};
```

场景转移条件（`IsTransferable`）：

1. 规划命令中必须包含 `lane_follow_command`
2. 参考线信息不能为空
3. 当前无其他场景（`other_scenario == nullptr`）时直接可转入

### LaneFollowParkStage

车道跟随规划阶段，继承自 `Stage`，负责在参考线上执行路径和速度规划任务，并判断是否需要触发脱困。

```cpp
class LaneFollowParkStage : public Stage {
 public:
  bool Init(const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context);
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
  StageResult PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);
 private:
  ScenarioLaneFollowParkConfig scenario_config_;
  std::pair<std::shared_ptr<Obstacle>, int> queue_sence_obs_info_;
  std::pair<std::string, int> stable_block_obs_count_;
};
```

### LaneEscapeUtil（外部依赖）

位于 `modules/planning/park_data_center/util/lane_escape_util.h`，提供脱困判断的静态工具方法：

- `IsNeedEscape` — 综合判断是否需要脱困
- `IsEnoughSpace` — 判断障碍物侧是否有足够空间绕行
- `IsQueueSence` — 判断是否为排队场景
- `IsStableBlockObs` — 判断障碍物是否稳定阻塞
- `GetClosestStopDecisionObs` — 获取最近的 stop 决策障碍物

## 核心函数

### LaneFollowParkScenario::Init

场景初始化函数，调用基类 `Init` 并加载 `ScenarioLaneFollowParkConfig` 配置到 `context_`。

```cpp
bool LaneFollowParkScenario::Init(
    std::shared_ptr<DependencyInjector> injector,
    const std::string& name) {
  if (!Scenario::Init(injector, name)) return false;
  if (!Scenario::LoadConfig<ScenarioLaneFollowParkConfig>(
          &context_.scenario_config)) return false;
  return true;
}
```

### LaneFollowParkStage::Process

阶段主处理函数，遍历所有参考线执行规划，并通过 `LaneEscapeUtil::IsNeedEscape` 判断脱困需求：

1. 遍历 `frame` 中的每条参考线，调用 `PlanOnReferenceLine` 执行规划任务
2. 对每条参考线调用 `LaneEscapeUtil::IsNeedEscape` 判断是否需要脱困
3. 若任意参考线判定需要脱困（`need_escape == 1`），通过 `ParkDataCenter::Instance()->set_need_escape(true)` 设置脱困标志，清除下一阶段并返回 `FINISHED`
4. 若无需脱困，根据是否有可行驶参考线返回 `RUNNING` 或 `ERROR`

### LaneFollowParkStage::PlanOnReferenceLine

在单条参考线上执行完整规划流程：

1. 为非变道参考线添加额外代价（`kStraightForwardLineCost = 10.0`）
2. 依次执行 `task_list_` 中的规划任务（路径规划、速度规划等）
3. 若任务出错，调用 `fallback_task_` 生成回退轨迹
4. 调用 `CombinePathAndSpeedProfile` 聚合路径和速度
5. 对有 stop 决策的静态障碍物添加额外代价，使参考线优先选择无障碍物阻塞的路线
6. 执行轨迹约束检查（`ConstraintChecker::ValidTrajectory`）
7. 设置最终轨迹和可行驶标志

### LaneFollowParkStage::GetStopSL

将障碍物的 stop 决策点从 XY 坐标转换为参考线上的 SL 坐标。

```cpp
SLPoint LaneFollowParkStage::GetStopSL(
    const ObjectStop& stop_decision,
    const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
  return sl_point;
}
```

### LaneFollowParkStage::RecordObstacleDebugInfo

记录障碍物决策的调试信息，包括 SL 边界和决策标签。

## 配置

### ScenarioLaneFollowParkConfig（Proto 定义）

定义于 `lane_follow_park_scenario.proto`：

```protobuf
message ScenarioLaneFollowParkConfig {
  optional int32 queue_check_count = 1 [default = 100];
  optional int32 stable_block_count = 2 [default = 30];
  optional double passby_min_gap = 3 [default = 0.5];
  optional double passby_kappa_ratio = 4 [default = 0.5];
  optional double min_distance_block_obs_to_junction = 5 [default = 22.0];
  optional bool enable_junction_borrow = 6 [default = false];
}
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `queue_check_count` | int32 | 100 | 排队场景检测的帧数阈值 |
| `stable_block_count` | int32 | 30 | 障碍物稳定阻塞的帧数阈值 |
| `passby_min_gap` | double | 0.5 | 通过障碍物时的最小间隙（米） |
| `passby_kappa_ratio` | double | 0.5 | 通过障碍物时的曲率比率 |
| `min_distance_block_obs_to_junction` | double | 22.0 | 阻塞障碍物距路口的最小距离（米） |
| `enable_junction_borrow` | bool | false | 是否启用路口借道 |

### 配置文件路径

| 文件 | 类型 | 说明 |
|------|------|------|
| `conf/scenario_conf.pb.txt` | `ScenarioLaneFollowParkConfig` | 场景参数配置 |
| `conf/pipeline.pb.txt` | `ScenarioPipeline` | 场景流水线阶段配置 |

## 调用关系

```text
LaneFollowParkScenario::IsTransferable
  |- 检查 planning_command 是否有 lane_follow_command
  |- 检查 reference_line_info 是否为空

LaneFollowParkScenario::Init
  |- Scenario::Init
  |- LoadConfig<ScenarioLaneFollowParkConfig>

LaneFollowParkStage::Process
  |- PlanOnReferenceLine (每条参考线)
  |   |- task_list_ 中各 Task::Execute (路径/速度规划)
  |   |- fallback_task_->Execute (出错时回退)
  |   |- CombinePathAndSpeedProfile (轨迹聚合)
  |   |- ConstraintChecker::ValidTrajectory (约束检查)
  |- LaneEscapeUtil::IsNeedEscape (脱困判断)
  |   |- LaneEscapeUtil::IsEnoughSpace
  |   |- LaneEscapeUtil::IsQueueSence
  |   |- LaneEscapeUtil::IsStableBlockObs
  |   |- LaneEscapeUtil::GetClosestStopDecisionObs
  |- ParkDataCenter::set_need_escape (设置脱困标志)
```

依赖模块：`planning-interface-base`（`Scenario`/`Stage` 基类）、`planning-park-data-center`（`ParkDataCenter` 与 `LaneEscapeUtil`）。
