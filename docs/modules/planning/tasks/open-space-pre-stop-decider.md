---
title: "开放空间预停车决策器"
---

# OpenSpacePreStopDecider

> 源码路径: `modules/planning/tasks/open_space_pre_stop_decider/`

## 概述

开放空间预停车决策器（`OpenSpacePreStopDecider`）是规划模块中的一个 Task 插件，负责在进入开放空间（Open Space）场景前，根据停车类型（泊车或靠边停车）计算并设置预停车围栏（stop fence）。该决策器在参考线上确定目标停车点的纵向坐标 `s`，然后构建停车决策，使自车在进入开放空间规划区域前完成减速停车。

支持两种停车模式：

- **PARKING（泊车）**：根据目标停车位的几何中心计算预停车位置
- **PULL_OVER（靠边停车）**：根据规划上下文中的靠边停车目标点计算预停车位置

## 核心类

### OpenSpacePreStopDecider

继承自 `Decider`，通过 Cyber 插件机制注册为 `Task` 类型。

```cpp
class OpenSpacePreStopDecider : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool CheckParkingSpotPreStop(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info,
                               double* target_s);

  bool CheckPullOverPreStop(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info,
                            double* target_s);

  void SetParkingSpotStopFence(const double target_s, Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);

  void SetPullOverStopFence(const double target_s, Frame* const frame,
                            ReferenceLineInfo* const reference_line_info);

  OpenSpacePreStopDeciderConfig config_;
  static constexpr const char* OPEN_SPACE_STOP_ID = "OPEN_SPACE_PRE_STOP";
};
```

## 核心函数

### Init

加载 `OpenSpacePreStopDeciderConfig` 配置，包括停车类型、目标停车距离等参数。

### Process

主处理入口，根据配置中的 `stop_type` 分发到对应的检查和设置流程：

1. `PARKING` 模式：调用 `CheckParkingSpotPreStop` 获取目标 `s`，再调用 `SetParkingSpotStopFence` 设置停车围栏
2. `PULL_OVER` 模式：调用 `CheckPullOverPreStop` 获取目标 `s`，再调用 `SetPullOverStopFence` 设置停车围栏

### CheckParkingSpotPreStop

在参考线路径的停车位重叠信息中查找目标停车位，通过停车位四个顶点计算几何中心，再投影到参考线上得到目标纵向坐标 `target_s`。若未找到目标停车位则返回失败。

### CheckPullOverPreStop

从规划上下文（`PlanningContext`）中读取靠边停车状态，若存在有效的目标位置（x, y），则将其转换为参考线坐标系的纵向坐标 `target_s`。

### SetParkingSpotStopFence

计算泊车场景的停车围栏位置。停车线位置为目标中心 `s` 加上车辆前缘到质心距离和配置的缓冲距离，然后通过 `util::BuildStopDecision` 构建停车决策。

### SetPullOverStopFence

计算靠边停车场景的停车围栏位置。根据自车与目标的距离是否大于 `stop_distance_to_target` 决定停车线位置：

- 距离充足时：在目标前 `stop_distance_to_target` 处设置停车线
- 距离不足时：使用 `rightaway_stop_distance` 立即设置近距离停车线，并通过 `pre_stop_rightaway_point` 和 `pre_stop_rightaway_flag` 保持停车点稳定

## 配置

通过 `OpenSpacePreStopDeciderConfig` protobuf 消息配置：

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `stop_type` | `StopType` 枚举 | - | 停车类型：`PARKING`（泊车）或 `PULL_OVER`（靠边停车） |
| `rightaway_stop_distance` | `double` | `2.0` 米 | 立即停车距离，距离不足时自车前缘到停车线的偏移 |
| `stop_distance_to_target` | `double` | `5.0` 米 | 目标停车距离，靠边停车时提前多远开始停车 |
| `stop_buffer_to_target` | `double` | `0.0` 米 | 泊车场景中目标停车位的额外缓冲距离 |

## 调用关系

```text
PlanningPipeline
  -> Task (插件调度)
    -> OpenSpacePreStopDecider::Process
        |-- CheckParkingSpotPreStop (PARKING 模式)
        |     |-- HDMap::GetParkingSpaceById
        |     |-- MapPath::GetNearestPoint
        |
        |-- CheckPullOverPreStop (PULL_OVER 模式)
        |     |-- ReferenceLine::XYToSL
        |
        |-- SetParkingSpotStopFence
        |     |-- util::BuildStopDecision
        |
        |-- SetPullOverStopFence
              |-- util::BuildStopDecision
```
