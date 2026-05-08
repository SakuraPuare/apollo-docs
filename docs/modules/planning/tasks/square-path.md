---
title: "方形路径规划"
---

# 方形路径规划 (SquarePath)

> 源码路径: `modules/planning/tasks/square_path/`

## 概述

SquarePath 是 Apollo 规划模块中的一个路径生成任务，继承自 `PathGeneration` 基类。其核心功能是在交叉路口（Junction）区域生成方形路径边界，并基于该边界进行路径优化和评估。任务通过插件机制注册为 `Task`，在规划流程中负责处理路口场景下的路径规划。

整体流程遵循三阶段模式：路径边界决策（DecidePathBounds） -> 路径优化（OptimizePath） -> 路径评估（AssessPath）。

## 核心类

### SquarePath

```cpp
class SquarePath : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;
  bool DecidePathBounds(std::vector<PathBoundary>* boundary);
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);
  bool AssessPath(std::vector<PathData>* candidate_path_data,
                  ReferenceLineInfo* reference_line_info);
  bool GetBoundaryFromSquare(const ReferenceLineInfo& reference_line_info,
                             PathBoundary* const path_boundary,
                             const SLState& init_sl_state);
  SquarePathConfig config_;
  common::VehicleParam vehicle_param_;
};
```

- 继承自 `PathGeneration`，通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册为插件
- `config_` 存储该任务的专属配置
- `vehicle_param_` 从全局车辆配置中获取车辆物理参数

## 核心函数

### Init

初始化函数，调用父类 `Task::Init` 并加载 `SquarePathConfig` 配置。

### Process

主处理入口，按以下步骤执行：

1. 调用 `GetStartPointSLState` 获取起始点的 SL 坐标
2. 调用 `DecidePathBounds` 计算路径边界
3. 调用 `OptimizePath` 对边界进行路径优化
4. 调用 `AssessPath` 评估路径可行性

### DecidePathBounds

计算路径边界：

1. 调用 `PathBoundsDeciderUtil::InitPathBoundary` 初始化路径边界
2. 调用 `GetBoundaryFromSelfLane` 获取自车道边界
3. 调用 `GetBoundaryFromSquare` 获取路口方形区域的边界约束
4. 调用 `GetBoundaryFromStaticObstacles` 考虑静态障碍物后微调边界
5. 当存在阻塞障碍物时，扩展路径边界尾部点
6. 验证初始位置的横向坐标是否在边界范围内

### OptimizePath

对每个路径边界进行优化：

1. 计算加速度边界 `ddl_bounds`
2. 估计 jerk 边界
3. 更新路径参考线权重
4. 调用 `PathOptimizerUtil::OptimizePath` 执行优化
5. 将优化结果转为 `PiecewiseJerkPath`，根据配置决定是否将前轴中心转换为后轴中心

### AssessPath

评估候选路径：

1. 记录调试信息
2. 通过 `PathAssessmentDeciderUtil::IsValidRegularPath` 验证路径合法性
3. 根据车辆最大转向角计算最大曲率 `max_kappa` 并进行日志输出
4. 将路径加入候选路径列表，若当前无最终路径则直接设为最终路径

### GetBoundaryFromSquare

获取路口方形区域的边界约束：

1. 通过 `GetJunction` 查询当前位置是否处于路口区域
2. 获取路口的多边形几何信息并转为 SL 坐标
3. 遍历路径边界点，对处于路口范围内的点，根据路口左右边界和道路宽度约束进行收紧
4. 标记边界类型为 `BoundType::ROAD`

## 配置

配置通过 `SquarePathConfig` protobuf 消息定义：

```protobuf
message SquarePathConfig {
  optional double max_lateral_distance = 1;
  optional bool enable_road_boundary_constraint = 2 [default = true];
  optional PiecewiseJerkPathConfig path_optimizer_config = 3;
}
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_lateral_distance` | double | - | 最大横向距离限制 |
| `enable_road_boundary_constraint` | bool | true | 是否启用道路边界约束 |
| `path_optimizer_config` | PiecewiseJerkPathConfig | - | 路径优化器配置，使用分段 jerk 模型 |

## 调用关系

```text
Planning 调度
  └── SquarePath::Process
        ├── PathGeneration::GetStartPointSLState  (获取起始 SL 状态)
        ├── SquarePath::DecidePathBounds
        │     ├── PathBoundsDeciderUtil::InitPathBoundary
        │     ├── PathBoundsDeciderUtil::GetBoundaryFromSelfLane
        │     ├── SquarePath::GetBoundaryFromSquare
        │     │     └── hdmap::HDMapUtil::BaseMapPtr (查询路口几何)
        │     ├── PathBoundsDeciderUtil::GetSLPolygons
        │     ├── PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles
        │     └── PathGeneration::RecordDebugInfo
        ├── SquarePath::OptimizePath
        │     ├── PathOptimizerUtil::CalculateAccBound
        │     ├── PathOptimizerUtil::EstimateJerkBoundary
        │     ├── PathOptimizerUtil::UpdatePathRefWithBound
        │     ├── PathOptimizerUtil::OptimizePath
        │     └── PathOptimizerUtil::ToPiecewiseJerkPath
        └── SquarePath::AssessPath
              ├── PathAssessmentDeciderUtil::IsValidRegularPath
              └── ReferenceLineInfo::MutableCandidatePathData (添加候选路径)
```
