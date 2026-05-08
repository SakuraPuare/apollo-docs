---
title: "路口方形场景"
---

# 路口方形场景

> 源码路径: `modules/planning/scenarios/square/`

## 概述

路口方形场景（Square Scenario）处理车辆在路口（Junction）内需要借道或被障碍物阻挡时的规划问题。当车辆处于 `lane_follow_command` 模式、仅有单条参考线、且车辆 SL 边界完全位于路口 overlap 范围内时，该场景被激活。

场景包含两个阶段：先执行 `SQUARE_LANE_FOLLOW_STAGE` 沿路口内参考线正常跟车行驶；当路径被阻挡且车速接近零持续 50 个规划周期后，切换到 `EXTRICATE_STAGE` 执行倒车脱困，倒车过程中找到可行正向路径并停车后，再切回跟车阶段直至驶出路口。

## 核心类

### SquareContext

```cpp
struct SquareContext : public ScenarioContext {
    ScenarioSquareConfig scenario_config;
    std::string junction_id;
    std::string blocking_obstacle_id;
};
```

继承 `ScenarioContext`，存储场景配置、当前路口 ID 以及阻挡障碍物 ID。

### SquareScenario

```cpp
class SquareScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name);
  SquareContext* GetContext() override;
  bool IsTransferable(const Scenario* other_scenario, const Frame& frame) override;

 private:
  SquareContext context_;
};
```

场景入口类，通过 Cyber 插件机制注册为 `Scenario`，负责配置加载和场景切换判断。

### SquareLaneFollowStage

```cpp
class SquareLaneFollowStage : public Stage {
 public:
  bool Init(const StagePipeline& config, const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context);
  StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;
  StageResult FinishStage();

 private:
  int blocking_times_ = 0;
};
```

路口跟车阶段。在路口内执行参考线规划任务链，当路径被阻塞且车辆静止时累计 `blocking_times_`，达到阈值后切换到脱困阶段。

### ExtricateStage

```cpp
class ExtricateStage : public Stage {
 public:
  bool Init(...);
  StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;
  bool HasCollision(Frame* frame, const common::math::Box2d& adc_box);
  bool GenerateStraightReversePath(PathData* path_data, const common::TrajectoryPoint& init_point);
 private:
  StageResult FinishStage();
  ScenarioSquareConfig scenario_config_;
  bool need_to_stop_ = false;
  int success_path_times_ = 0;
};
```

脱困阶段。未挂倒挡时生成静止轨迹等待换挡；倒挡就绪后在后方 10m 设置停车点执行倒车规划；连续检测到可行正向路径超 10 次且车辆停稳后，切回跟车阶段。

## 核心函数

### SquareScenario::Init() / IsTransferable()

```cpp
bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name);
bool IsTransferable(const Scenario* other_scenario, const Frame& frame);
```

`Init` 调用基类初始化并通过 `LoadConfig<ScenarioSquareConfig>()` 加载配置。`IsTransferable` 判断场景切换条件：需存在 `lane_follow_command`、单条参考线、车辆 SL 起止范围完全包含在路口 overlap 内，满足后将 `junction_id` 写入 context。

`square_scenario.cc:L28-L61`

### SquareLaneFollowStage::Process()

```cpp
StageResult Process(const common::TrajectoryPoint& planning_start_point, Frame* frame)
```

- 若处于倒挡，生成静止轨迹并请求切换到前进挡
- 调用 `ExecuteTaskOnReferenceLine()` 执行参考线规划任务链
- 获取路口 overlap，若车辆已驶出路口则 `FinishScenario()` 退出场景
- 判断路径是否被阻塞：路径标签不含 `regular` 或存在阻挡障碍物时标记为阻塞
- 累计阻塞次数，达到 50 次后切换到 `EXTRICATE_STAGE`

`square_lane_follow_stage.cc:L49-L122`

### ExtricateStage::Process()

```cpp
StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame)
```

- 未挂倒挡时：生成静止轨迹（速度 0、倒挡目标），返回 `RUNNING` 等待换挡完成
- 已挂倒挡时：在当前位置后方 10m 设置停车点，构建停车决策，执行参考线规划任务链
- 检测候选路径中是否存在无阻挡的 `regular` 路径，连续成功超过 10 次后标记 `need_to_stop_`
- 车辆停稳后调用 `FinishStage()` 切回 `SQUARE_LANE_FOLLOW_STAGE`

`extricate_stage.cc:L51-L128`

### ExtricateStage 辅助函数

- `HasCollision()` — 将自车包围盒转为多边形，遍历障碍物检查碰撞重叠（`extricate_stage.cc:L137-L146`）
- `GenerateStraightReversePath()` — 沿航向角反向生成 10m 直线倒车路径（步长 0.5m），标签 `reverse_straight_path`（`extricate_stage.cc:L148-L168`）

## 配置

Proto 定义于 `proto/square.proto`：

```protobuf
message ScenarioSquareConfig {
  optional double filtering_distance = 1 [default = 100];
  optional double perception_obstacle_buffer = 2 [default = 2];
}
```

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `filtering_distance` | 100 m | 障碍物过滤距离 |
| `perception_obstacle_buffer` | 2 m | 感知障碍物膨胀缓冲 |

Pipeline 配置（`conf/pipeline.pb.txt`）定义了两个阶段：

**SQUARE_LANE_FOLLOW_STAGE**（`SquareLaneFollowStage`）任务链：

1. `ObstacleNudgeDecider` — 障碍物绕行决策
2. `SquarePath` — 路口专用路径规划
3. `LaneBorrowPathGeneric` — 通用借道路径
4. `FallbackPath` — 兜底路径
5. `RuleBasedStopDecider` — 规则停车决策
6. `SpeedBoundsPrioriDecider` — 先验速度边界
7. `PathTimeHeuristicOptimizer` — 速度启发优化
8. `SpeedDecider` — 速度决策
9. `SpeedBoundsFinalDecider` — 最终速度边界
10. `PiecewiseJerkSpeedOptimizer` — 分段 Jerk 速度优化

**EXTRICATE_STAGE**（`ExtricateStage`）任务链：

1. `ObstacleNudgeDecider` — 障碍物绕行决策
2. `LaneFollowPath` — 跟车路径
3. `LaneBorrowPathGeneric` — 通用借道路径
4. `ReversePath` — 倒车路径
5. `ReverseSpeed` — 倒车速度决策
6. `PiecewiseJerkSpeedOptimizer` — 分段 Jerk 速度优化

## 调用关系

```text
SquareScenario::IsTransferable()
  |  检查 lane_follow_command + 单参考线 + 车辆完全在路口内
  v
SquareScenario::Init()
  |  加载 ScenarioSquareConfig
  v
SquareLaneFollowStage::Process()
  |-- ExecuteTaskOnReferenceLine()     [执行参考线规划任务链]
  |-- 判断是否驶出路口 → FinishScenario()
  |-- 路径阻塞计数 blocking_times_
        |  blocking_times_ >= 50
        v
      ExtricateStage::Process()
        |-- 未倒挡 → 生成静止轨迹等待换挡
        |-- 已倒挡 → 设置停车点 + 倒车规划
        |-- 检测正向可行路径 success_path_times_
              |  success_path_times_ > 10 且车辆停稳
              v
            FinishStage() → SQUARE_LANE_FOLLOW_STAGE
```
