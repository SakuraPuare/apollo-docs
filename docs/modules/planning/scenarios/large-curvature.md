---
title: "大曲率场景"
---

# 大曲率场景

> 源码路径: `modules/planning/scenarios/large_curvature/`

## 概述

大曲率场景处理参考线曲率超出车辆最小转弯半径对应曲率的工况。当车辆前方参考线存在急弯（曲率大于车辆物理极限的一定比例）且距目的地超过 10m 时，该场景被激活。

激活后，场景使用 Open Space 系列任务进行路径规划与轨迹优化，在车辆到达目的地或前方曲率恢复到安全范围时退出。

## 核心类

### LargeCurvatureContext

```cpp
struct LargeCurvatureContext : public ScenarioContext {
    ScenarioLargeCurvatureConfig scenario_config;
};
```

继承 `ScenarioContext`，持有大曲率场景的配置信息。

### LargeCurvatureScenario

```cpp
class LargeCurvatureScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;
  LargeCurvatureContext* GetContext() override;
  bool IsTransferable(const Scenario* const other_scenario, const Frame& frame) override;

 private:
  bool init_ = false;
  LargeCurvatureContext context_;
  const hdmap::HDMap* hdmap_ = nullptr;
};
```

场景入口类，负责初始化配置和判断是否切换到大曲率场景。

### StageLargeCurvature

```cpp
class StageLargeCurvature : public Stage {
 public:
  bool Init(const StagePipeline& config, const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context);
  StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

 private:
  StageResult FinishStage();
  bool IsCurvatureSmall(Frame* frame);
  ScenarioLargeCurvatureConfig scenario_config_;
};
```

大曲率场景的唯一执行阶段，负责 Open Space 规划和曲率退出判断。

## 核心函数

### LargeCurvatureScenario::Init()

```cpp
bool LargeCurvatureScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name)
```

- 调用 `Scenario::Init()` 初始化基类
- 通过 `Scenario::LoadConfig<ScenarioLargeCurvatureConfig>()` 加载配置
- 获取 HD 地图指针

`large_curvature_scenario.cc:L35-L55`

### LargeCurvatureScenario::IsTransferable()

```cpp
bool LargeCurvatureScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame)
```

场景切换条件判断：

1. 必须存在 `lane_follow_command` 路由指令
2. 获取车辆当前位置在参考线上的 SL 坐标
3. 读取前方 3m 处参考点的曲率 `kappa`
4. 当 `|kappa| > curvature_ratio / min_turn_radius` 且距目的地超过 10m 时返回 `true`

`large_curvature_scenario.cc:L57-L87`

### StageLargeCurvature::Process()

```cpp
StageResult StageLargeCurvature::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame)
```

- 设置 `is_on_open_space_trajectory = true`
- 调用 `ExecuteTaskOnOpenSpace()` 执行 Open Space 任务链
- 结束条件：到达目的地 或 `IsCurvatureSmall()` 返回 `true`
- 结束时清空 `next_stage_`，状态设为 `FINISHED`

`stage_large_curvature.cc:L46-L60`

### StageLargeCurvature::IsCurvatureSmall()

```cpp
bool StageLargeCurvature::IsCurvatureSmall(Frame* frame)
```

- 获取车辆当前位置的 SL 坐标
- 沿参考线向前，以 `curvature_check_delta_s`（默认 0.3m）为步长，检查 `curvature_check_distance`（默认 3.0m）范围内各参考点的曲率
- 如果所有检查点的 `|kappa| <= curvature_check_max`（默认 0.1），则认为曲率已恢复正常，返回 `true`

`stage_large_curvature.cc:L62-L96`

## 配置

Proto 定义于 `proto/large_curvature_scenario.proto`：

```protobuf
message ScenarioLargeCurvatureConfig {
  optional double min_curvature = 1 [default = 0.2];
  optional double curvature_check_delta_s = 2 [default = 0.3];
  optional double curvature_check_distance = 3 [default = 3.0];
  optional double curvature_check_max = 4 [default = 0.1];
  optional double curvature_ratio = 5 [default = 1.0];
}
```

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `min_curvature` | 0.2 | 最小曲率阈值 |
| `curvature_check_delta_s` | 0.3 m | 曲率检查步长 |
| `curvature_check_distance` | 3.0 m | 前方曲率检查总距离 |
| `curvature_check_max` | 0.1 | 曲率恢复判定阈值 |
| `curvature_ratio` | 1.0 | 曲率比例系数，乘以车辆最大曲率作为进入阈值 |

Pipeline 配置（`conf/pipeline.pb.txt`）定义了单阶段 `LARGE_CURVATURE`，包含以下 Open Space 任务链：

1. `OpenSpaceReplanDecider` — 重规划决策
2. `OpenSpaceRoiDeciderPark` — 感兴趣区域决策
3. `OpenSpacePathPlanning` — 路径规划
4. `OpenSpaceTrajectoryOptimizerPark` — 轨迹优化
5. `OpenSpaceTrajectoryPostProcess` — 轨迹后处理
6. `OpenSpaceFallbackDeciderPark` — 兜底决策

## 调用关系

```text
LargeCurvatureScenario::IsTransferable()
  |  检查参考线曲率 vs 车辆最大曲率 * ratio
  v
LargeCurvatureScenario::Init()
  |  加载 ScenarioLargeCurvatureConfig
  v
StageLargeCurvature::Process()
  |-- ExecuteTaskOnOpenSpace()       [执行 Open Space 任务链]
  |-- IsCurvatureSmall()             [判断是否退出]
        |  前方 curvature_check_distance 范围内
        |  所有检查点 |kappa| <= curvature_check_max
        v
      FINISHED / next_stage 清空
```
