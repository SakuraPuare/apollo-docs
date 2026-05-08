---
title: "Free Space"
---

# Free Space Scenario

> 源码路径: `modules/planning/scenarios/free_space/`

## 概述

`FreeSpaceScenario` 用于在指定可行驶区域边界与目标位姿条件下，通过 OpenSpace 算法生成从当前位置驶向目标停车位的轨迹。该场景属于泊车/低速场景规划的一部分，适用于用户通过外部命令指定可行驶区域（drivable ROI）和目标位姿的场景。

场景切入条件：

1. `planning_command` 中包含 `FreeSpaceCommand` 自定义命令
2. 自车处于静止状态（线速度低于 `max_abs_speed_when_stopped`）
3. 可行驶区域边界以逆时针顺序发布，且包含自车起点与目标终点
4. 不可行驶区域边界以逆时针顺序发布，且不与自车起点或目标终点重叠

## 架构

场景采用经典的 **Scenario - Stage - Task** 三层架构：

- **FreeSpaceScenario** -- 场景入口，负责初始化和场景切换判断
- **StageFreeSpace** -- 唯一的阶段，负责构建 ROI 边界约束并执行 OpenSpace 轨迹规划任务
- **Task** -- 三个子任务依次执行：
  - `OpenSpaceTrajectoryProvider` -- 生成开空间轨迹
  - `OpenSpaceTrajectoryPartition` -- 轨迹分区
  - `OpenSpaceFallbackDecider` -- 回退决策

通过 Cyber 插件机制注册，在 `plugins.xml` 中声明：

```xml
<library path="modules/planning/scenarios/free_space/libfree_space_scenario.so">
    <class type="apollo::planning::FreeSpaceScenario" base_class="apollo::planning::Scenario"></class>
    <class type="apollo::planning::StageFreeSpace" base_class="apollo::planning::Stage"></class>
</library>
```

## 核心类

### FreeSpaceContext

继承自 `ScenarioContext`，承载场景运行时上下文数据：

```cpp
struct FreeSpaceContext : public ScenarioContext {
    ScenarioFreeSpaceConfig scenario_config;
    apollo::external_command::FreeSpaceCommand free_space_command;
};
```

- `scenario_config` -- 场景级配置（感知障碍物开关、过滤距离、缓冲距离）
- `free_space_command` -- 来自外部的自由空间停车命令，包含可行驶区域边界和目标位姿

### FreeSpaceScenario

场景主类，负责初始化和场景可转移性判断：

```cpp
class FreeSpaceScenario : public Scenario {
public:
    bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;
    FreeSpaceContext* GetContext() override { return &context_; }
    bool IsTransferable(const Scenario* const other_scenario, const Frame& frame) override;
private:
    bool init_ = false;
    FreeSpaceContext context_;
    const hdmap::HDMap* hdmap_ = nullptr;
};
```

### StageFreeSpace

场景唯一阶段，负责将外部命令中的 ROI 信息转化为 OpenSpace 约束并执行轨迹规划：

```cpp
class StageFreeSpace : public Stage {
public:
    bool Init(const StagePipeline& config,
              const std::shared_ptr<DependencyInjector>& injector,
              const std::string& config_dir, void* context);
    StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;
private:
    StageResult FinishStage();
    ScenarioFreeSpaceConfig scenario_config_;
};
```

## 核心函数

### FreeSpaceScenario::Init

```cpp
bool FreeSpaceScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name)
```

**职责**: 初始化场景，加载配置和高精地图。

**关键步骤**:

1. 若已初始化则直接返回（防重复初始化）
2. 调用 `Scenario::Init` 完成基类初始化
3. 通过 `Scenario::LoadConfig` 加载 `ScenarioFreeSpaceConfig`
4. 获取高精地图指针 `HDMapUtil::BaseMapPtr()`

### FreeSpaceScenario::IsTransferable

```cpp
bool FreeSpaceScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame)
```

**职责**: 判断当前帧是否满足切入 FreeSpace 场景的条件。

**输入**: 当前帧 `Frame`，包含 `planning_command` 和车辆状态

**输出**: `bool`，满足所有条件时返回 `true`

**关键步骤**:

1. 检查 `planning_command` 是否包含自定义命令且类型为 `FreeSpaceCommand`
2. 解包命令到 `context_.free_space_command`
3. 检查自车是否处于静止状态
4. 构建自车和目标停车位的 `Polygon2d` 包围盒
5. 校验可行驶区域（`drivable_roi`）存在且为逆时针顺序
6. 校验自车和目标包围盒均在可行驶区域内
7. 校验不可行驶区域（`non_drivable_roi`）为逆时针顺序，且不与自车或目标重叠

### StageFreeSpace::Init

```cpp
bool StageFreeSpace::Init(const StagePipeline& config,
                          const std::shared_ptr<DependencyInjector>& injector,
                          const std::string& config_dir, void* context)
```

**职责**: 初始化阶段，从上下文中拷贝场景配置。

**关键步骤**:

1. 调用 `Stage::Init` 完成基类初始化（包括加载 Task 插件）
2. 从 `FreeSpaceContext` 中拷贝 `scenario_config_`

### StageFreeSpace::Process

```cpp
StageResult StageFreeSpace::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame)
```

**职责**: 核心执行逻辑，将外部命令转化为 OpenSpace 约束并执行轨迹规划。

**输入**: 规划起始点 `planning_init_point`，当前帧 `Frame`

**输出**: `StageResult`，包含规划状态

**关键步骤**:

1. **构建可行驶区域边界** -- 从 `free_space_command.drivable_roi()` 提取边界点，首尾不闭合时自动补全
2. **设置 XY 边界** -- 调用 `OpenSpaceRoiUtil::GetRoiXYBoundary` 计算 ROI 的 XY 范围
3. **添加不可行驶区域** -- 遍历 `non_drivable_roi`，校验其在可行驶区域范围内后追加到边界
4. **设置终点位姿** -- 从 `parking_spot_pose()` 提取 x/y/heading，加上速度 0
5. **加载感知障碍物** -- 若配置启用，调用 `OpenSpaceRoiUtil::LoadObstacles` 将障碍物边界加入 roi_boundary
6. **坐标变换** -- 以 ROI 左下角为原点进行坐标平移
7. **构建线性约束** -- 调用 `FormulateBoundaryConstraints` 将边界约束转为 Ax < b 形式
8. **执行任务** -- 调用 `ExecuteTaskOnOpenSpace` 依次执行三个 Task（Provider -> Partition -> Fallback）

### StageFreeSpace::FinishStage

```cpp
StageResult StageFreeSpace::FinishStage()
```

**职责**: 阶段结束，返回 `StageStatusType::FINISHED`。

## 配置

### 场景配置 (scenario_conf.pb.txt)

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled_perception_obstacles` | `bool` | `true` | 是否启用感知障碍物 |
| `perception_obstacle_buffer` | `double` | 0.2 | 感知障碍物缓冲距离（米） |
| `filtering_distance` | `double` | 1000.0 | 过滤距离阈值，超出该距离的障碍物被忽略（米） |

### 流水线配置 (pipeline.pb.txt)

定义 `STAGE_FREE_SPACE` 阶段的三个 Task 执行顺序：

1. `OpenSpaceTrajectoryProvider` -- 开空间轨迹生成
2. `OpenSpaceTrajectoryPartition` -- 轨迹分区
3. `OpenSpaceFallbackDecider` -- 回退决策

### 轨迹生成配置 (open_space_trajectory_provider.pb.txt)

包含完整的 OpenSpace 轨迹优化器配置，涵盖四个子模块：

- **warm_start_config** -- Hybrid A* 预热搜索参数（网格分辨率、惩罚权重、最大探索数等）
- **dual_variable_warm_start_config** -- 对偶变量预热配置（OSQP/IPOPT 求解器参数）
- **distance_approach_config** -- 距离逼近优化配置（状态权重、速度/加速度限制、IPOPT 求解器参数）
- **iterative_anchoring_smoother_config** -- 迭代锚点平滑配置（曲率约束、速度限制等）

### 轨迹分区配置 (open_space_trajectory_partition.pb.txt)

包含轨迹分区的换挡控制和平滑参数，如换挡最大时间、插值段数、速度重规划距离等。

## 调用关系

```text
PlanningComponent
  └─ ScenarioManager::Update
       └─ FreeSpaceScenario::IsTransferable   // 判断是否切入场景
            └─ [满足条件] FreeSpaceScenario::Init
                 └─ StageFreeSpace::Init
                      └─ StageFreeSpace::Process
                           ├─ 构建可行驶/不可行驶区域边界
                           ├─ 设置终点位姿
                           ├─ 加载感知障碍物
                           ├─ 坐标变换 & 线性约束构建
                           └─ ExecuteTaskOnOpenSpace
                                ├─ OpenSpaceTrajectoryProvider  // 轨迹生成
                                ├─ OpenSpaceTrajectoryPartition // 轨迹分区
                                └─ OpenSpaceFallbackDecider     // 回退决策
```
