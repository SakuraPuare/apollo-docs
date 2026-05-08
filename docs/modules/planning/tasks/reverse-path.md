---
title: "倒车路径规划"
---

# ReversePath 倒车路径规划

> 源码路径: `modules/planning/tasks/reverse_path/`

## 概述

`ReversePath` 是 Apollo 规划模块中的倒车路径生成任务，继承自 `PathGeneration` 基类。该任务负责在倒车场景下规划一条安全可行的路径，主要处理以下场景：

- **路口倒车**：在交叉路口（junction）区域内，根据路口多边形边界约束生成路径
- **车道内倒车**：基于当前车道边界进行倒车路径规划

路径规划采用分阶段流程：先计算路径边界（`DecidePathBounds`），再通过 OSQP 二次规划优化器求解最优路径（`OptimizePathOsqp`）。规划结果以 Frenet 坐标系表示，路径标记为 `is_reverse_path = true`。优化过程中会对初始 SL 状态的横向速度和加速度取反，以适配倒车方向。

该任务通过 CyberRT 插件机制注册为 `Task` 插件。

## 核心类

### ReversePath

```cpp
class ReversePath : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;
  bool DecidePathBounds(PathBoundary* boundary, double& reference_line_backward_length);
  bool OptimizePath(PathBoundary* path_boundary, PathData* candidate_path_data);
  bool OptimizePathOsqp(PathBoundary& path_boundary, PathData* candidate_path_data);
  bool InitPathBoundary(PathBoundary* const path_bound, SLState init_sl_state,
                        double& reference_line_backward_length);
  bool GetBoundaryFromSquare(const ReferenceLineInfo& reference_line_info,
                             PathBoundary* const path_boundary, const SLState& init_sl_state);

  hdmap::PathOverlap junction_overlap_;  // 路口重叠信息
  ReversePathConfig config_;             // 任务配置
};
```

**继承关系**: `ReversePath` -> `PathGeneration` -> `Task`

## 核心函数

### Init

```cpp
bool ReversePath::Init(const std::string& config_dir, const std::string& name,
                       const std::shared_ptr<DependencyInjector>& injector);
```

初始化任务，加载 `ReversePathConfig` 配置。

### Process

```cpp
apollo::common::Status ReversePath::Process(Frame* frame, ReferenceLineInfo* reference_line_info);
```

核心处理流程：

1. 调用 `GetStartPointSLState()` 获取起始 SL 状态
2. 若配置了方形边界约束（`is_considered_square_boundary`），检查当前位置是否在路口内
3. 计算参考线反向长度 `reference_line_backward_length`
4. 调用 `DecidePathBounds` 计算路径边界
5. 调用 `OptimizePathOsqp` 通过 OSQP 优化生成路径
6. 将结果写入 `reference_line_info->mutable_path_data()`

### DecidePathBounds

```cpp
bool ReversePath::DecidePathBounds(PathBoundary* boundary, double& reference_line_backward_length);
```

分三步计算路径边界：

1. `InitPathBoundary` 初始化边界（以 `delta_s = -0.1` 步进，从起始点向后延伸）
2. 根据配置选择边界策略：
   - `is_considered_square_boundary`：调用 `GetBoundaryFromSquare`，基于路口多边形约束边界
   - `is_considered_lane_boundary`：调用 `PathBoundsDeciderUtil::GetBoundaryFromSelfLane`，基于车道约束边界
3. 若初始横向位置超出边界范围，调用 `PathBoundsDeciderUtil::ExtendBoundaryByADC` 扩展边界

### OptimizePathOsqp

```cpp
bool ReversePath::OptimizePathOsqp(PathBoundary& path_boundary, PathData* candidate_path_data);
```

使用 OSQP 二次规划求解最优倒车路径，关键步骤：

1. 计算曲率约束 `ddl_bounds` 并反转符号（适配倒车方向）
2. 反转初始状态的横向速度 `dl` 和加速度 `ddl`
3. 调用 `PathOptimizerUtil::OptimizePath` 求解
4. 将优化结果的 `s` 坐标反转（`start_s - delta_s`），并反转 `dl`/`ddl` 符号
5. 可选地将路径从前轴中心转换到后轴中心（`ConvertPathPointRefFromFrontAxeToRearAxe`）

### OptimizePath

```cpp
bool ReversePath::OptimizePath(PathBoundary* path_boundary, PathData* candidate_path_data);
```

备用路径生成方法，基于线性插值生成简单路径。使用初始横向偏移和横向速度线性外推，逐点检查是否在边界内，超出则截断。

### InitPathBoundary

```cpp
bool ReversePath::InitPathBoundary(PathBoundary* const path_bound, SLState init_sl_state,
                                   double& reference_line_backward_length);
```

初始化路径边界：以 `delta_s = -0.1` 步进，从起始 `s` 向后延伸至 `max_s_distance` 或参考线末端，每点设横向边界为 `[-max_lateral_distance, max_lateral_distance]`。

### GetBoundaryFromSquare

```cpp
bool ReversePath::GetBoundaryFromSquare(const ReferenceLineInfo& reference_line_info,
                                        PathBoundary* const path_boundary,
                                        const SLState& init_sl_state);
```

基于路口多边形（junction polygon）计算方形边界约束。将路口多边形转换为 SL 多边形，遍历路径边界点，在路口范围内用多边形的左右边界收紧路径边界。

## 配置

配置通过 protobuf 消息 `ReversePathConfig` 定义：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_s_distance` | `double` | `10.0` | 路径纵向最大延伸距离（米） |
| `max_lateral_distance` | `double` | `3.0` | 路径横向最大偏移距离（米） |
| `is_considered_square_boundary` | `bool` | `false` | 是否使用路口方形边界约束 |
| `is_considered_lane_boundary` | `bool` | `true` | 是否使用车道边界约束 |
| `path_optimizer_config` | `PiecewiseJerkPathConfig` | — | 路径优化器配置（OSQP 参数等） |

默认配置文件路径：`conf/default_conf.pb.txt`

## 调用关系

```text
Task::Execute()
  └── PathGeneration::Execute()
        └── ReversePath::Process()
              ├── GetStartPointSLState()           // 继承自 PathGeneration
              ├── DecidePathBounds()
              │     ├── InitPathBoundary()          // 初始化边界
              │     ├── GetBoundaryFromSquare()     // 路口方形边界
              │     ├── PathBoundsDeciderUtil::GetBoundaryFromSelfLane()  // 车道边界
              │     └── PathBoundsDeciderUtil::ExtendBoundaryByADC()      // 扩展边界
              ├── OptimizePathOsqp()               // OSQP 优化主路径
              │     ├── PathOptimizerUtil::CalculateAccBound()
              │     ├── PathOptimizerUtil::EstimateJerkBoundary()
              │     ├── PathOptimizerUtil::UpdatePathRefWithBound()
              │     ├── PathOptimizerUtil::OptimizePath()
              │     └── PathOptimizerUtil::ToPiecewiseJerkPath()
              └── (可选) OptimizePath()            // 线性插值备用路径
```

关键依赖模块：

- `PathGeneration`（路径生成基类，提供 SL 状态获取和执行框架）
- `PathOptimizerUtil`（路径优化工具，封装 OSQP 求解器）
- `PathBoundsDeciderUtil`（路径边界计算工具）
- `SLPolygon`（SL 坐标系下的多边形，用于路口边界表示）
