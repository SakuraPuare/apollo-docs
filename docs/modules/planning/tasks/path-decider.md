---
title: "路径决策器"
---

# 路径决策器 (PathDecider)

> 源码路径: `modules/planning/tasks/path_decider/`

## 概述

PathDecider 是规划模块中的路径决策任务，负责基于已生成的路径对静态障碍物做出决策（忽略、停车或推挤）。它是 `Task` 基类的插件实现，通过 Cyber RT 插件机制动态加载。核心逻辑包括：检测阻塞障碍物并维护其周期计数器，遍历路径范围内的静态障碍物并根据横向距离判断做出 IGNORE / STOP / NUDGE 决策，以及可选地忽略自车后方的动态障碍物。

## 核心类

### PathDecider

继承自 `Task`，通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册为规划任务插件。

```cpp
class PathDecider : public Task {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;
  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  PathDeciderConfig config_;
};
```

## 核心函数

### Init

```cpp
bool PathDecider::Init(const std::string &config_dir, const std::string &name,
                       const std::shared_ptr<DependencyInjector> &injector);
```

调用 `Task::Init` 完成基类初始化，随后通过 `Task::LoadConfig<PathDeciderConfig>` 加载路径决策器配置。

### Execute

```cpp
Status PathDecider::Execute(Frame *frame,
                            ReferenceLineInfo *reference_line_info);
```

任务入口函数。调用基类 `Task::Execute` 后，将 `reference_line_info` 中的路径数据和路径决策对象传递给 `Process` 处理。

### Process

```cpp
Status PathDecider::Process(const ReferenceLineInfo *reference_line_info,
                            const PathData &path_data,
                            PathDecision *const path_decision);
```

主处理流程：

1. 若启用了 `FLAGS_enable_skip_path_tasks` 且路径可复用，则跳过决策
2. 输出调试信息（起点包围盒和离散化路径点）
3. 检测阻塞障碍物，更新 `path_decider` 状态中的周期计数器（范围 `[-10, 10]`），计数器低于 `-2` 时清除阻塞障碍物 ID
4. 调用 `MakeObjectDecision` 生成障碍物决策

### MakeObjectDecision

```cpp
bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     const std::string &blocking_obstacle_id,
                                     PathDecision *const path_decision);
```

依次调用 `MakeStaticObstacleDecision` 处理静态障碍物，若配置了 `ignore_backward_obstacle` 则额外调用 `IgnoreBackwardObstacle`。

### MakeStaticObstacleDecision

```cpp
bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, const std::string &blocking_obstacle_id,
    PathDecision *const path_decision);
```

遍历所有障碍物，对每个静态非虚拟障碍物进行判断：

- 已有纵向和横向 IGNORE 决策 → 跳过
- 已有 STOP 决策 → 跳过
- 是阻塞障碍物且不在借道场景 → 添加纵向 STOP
- KEEP_CLEAR 区域 → 跳过
- 不在路径 s 范围内 → 添加纵向+横向 IGNORE
- 横向超出 `lateral_radius` → 添加横向 IGNORE
- 横向重叠（进入 `min_nudge_l` 范围）→ 添加纵向 STOP（通过 `MergeWithMainStop` 合并）
- 横向接近但未重叠 → 添加 LEFT_NUDGE 或 RIGHT_NUDGE

### GenerateObjectStopDecision

```cpp
ObjectStop PathDecider::GenerateObjectStopDecision(const Obstacle &obstacle) const;
```

根据障碍物的最小半径停车距离生成停车决策，设置停车点坐标和停车朝向。

### IgnoreBackwardObstacle

```cpp
bool PathDecider::IgnoreBackwardObstacle(PathDecision *const path_decision);
```

遍历所有非静态非虚拟障碍物，若其 SL 坐标 `end_s` 小于自车 `start_s`，则添加纵向 IGNORE 决策。

## 配置

配置通过 `PathDeciderConfig` protobuf 定义：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `static_obstacle_buffer` | double | 0.3 | 静态障碍物横向缓冲距离（米），用于计算推挤距离 |
| `ignore_backward_obstacle` | bool | false | 是否忽略自车后方的动态障碍物 |
| `skip_overlap_stop_check` | bool | false | 是否跳过横向重叠时的停车检查 |

## 调用关系

```text
PathDecider::Execute
  └─ Task::Execute (基类)
  └─ Process
       ├─ 检测阻塞障碍物，更新 path_decider 状态
       └─ MakeObjectDecision
            ├─ MakeStaticObstacleDecision
            │    ├─ GenerateObjectStopDecision (停车决策)
            │    └─ PathDecision::MergeWithMainStop (合并停车)
            └─ IgnoreBackwardObstacle (可选)
```
