---
title: "开放空间轨迹提供器"
---

# Open Space Trajectory Provider

> 源码路径: `modules/planning/tasks/open_space_trajectory_provider/`

## 概述

开放空间轨迹提供器（Open Space Trajectory Provider）负责在非结构化场景（如泊车、掉头等）中生成可行轨迹。它继承自 `TrajectoryOptimizer`，支持单线程和多线程两种执行模式。在多线程模式下，轨迹优化在独立线程中异步执行，主流程每帧检查是否有新轨迹可用，若无则复用上一帧结果或生成停车轨迹兜底。

核心优化流程分为三步：Hybrid A* 粗路径搜索 -> 对偶变量热启动 -> 距离逼近平滑或迭代锚点平滑。平滑后的轨迹会经过坐标反变换、拼接拼合后输出到 `OpenSpaceInfo`。

## 核心类

### OpenSpaceTrajectoryProvider

主入口类，注册为 `Task` 插件。管理异步线程生命周期，协调帧间轨迹复用与重新规划。

```cpp
class OpenSpaceTrajectoryProvider : public TrajectoryOptimizer {
 private:
  std::unique_ptr<OpenSpaceTrajectoryOptimizer> open_space_trajectory_optimizer_;
  OpenSpaceTrajectoryThreadData thread_data_;
  std::future<void> task_future_;
  std::atomic<bool> is_generation_thread_stop_{false};
  std::atomic<bool> trajectory_updated_{false};
  std::atomic<bool> data_ready_{false};
  std::atomic<bool> trajectory_error_{false};
  std::atomic<bool> trajectory_skipped_{false};
  std::mutex open_space_mutex_;
  OpenSpaceTrajectoryProviderConfig config_;
};
```

### OpenSpaceTrajectoryThreadData

线程间数据传输结构体，用于在主线程与优化线程之间安全传递障碍物、目标位姿等规划输入。

```cpp
struct OpenSpaceTrajectoryThreadData {
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  std::vector<double> end_pose;
  std::vector<double> XYbounds;
  double rotate_angle;
  apollo::common::math::Vec2d translate_origin;
  Eigen::MatrixXi obstacles_edges_num;
  Eigen::MatrixXd obstacles_A;
  Eigen::MatrixXd obstacles_b;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
};
```

### OpenSpaceTrajectoryOptimizer

底层优化器，封装 Hybrid A*、对偶变量热启动、距离逼近和迭代锚点平滑器，执行完整的轨迹生成与优化流水线。

```cpp
class OpenSpaceTrajectoryOptimizer {
 private:
  std::unique_ptr<HybridAStar> warm_start_;
  std::unique_ptr<DistanceApproachProblem> distance_approach_;
  std::unique_ptr<DualVariableWarmStartProblem> dual_variable_warm_start_;
  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;
};
```

## 核心函数

### OpenSpaceTrajectoryProvider::Process()

主流程入口，每帧调用。在 ParkAndGo 检查阶段直接生成停车轨迹；否则根据配置选择多线程或单线程模式。多线程模式下通过原子变量 (`trajectory_updated_`, `data_ready_`, `trajectory_error_`) 与优化线程同步；单线程模式下同步调用 `OpenSpaceTrajectoryOptimizer::Plan()`。

### OpenSpaceTrajectoryProvider::GenerateTrajectoryThread()

独立线程的循环函数。当 `data_ready_` 为 true 且 `trajectory_updated_` 为 false 时，从 `thread_data_` 读取数据并调用 `Plan()`。成功时设置 `trajectory_updated_`，失败时设置 `trajectory_error_` 或 `trajectory_skipped_`。

### OpenSpaceTrajectoryOptimizer::Plan()

完整的轨迹优化流水线：

1. **输入校验** - 检查边界、终点、障碍物数据完整性
2. **坐标归一化** - `PathPointNormalizing()` 将世界坐标旋转平移到局部坐标系
3. **Hybrid A* 粗搜索** - `warm_start_->Plan()` 生成初始路径
4. **轨迹分段平滑**（`FLAGS_enable_parallel_trajectory_smoothing` 时）- 对每段调用 `GenerateDistanceApproachTraj()` 或 `GenerateDecoupledTraj()`
5. **轨迹拼接** - `CombineTrajectories()` 将分段结果合并
6. **坐标反归一化** - `PathPointDeNormalizing()` 变换回世界坐标系
7. **加载输出** - `LoadTrajectory()` 将 Eigen 矩阵转为 `DiscretizedTrajectory`

### OpenSpaceTrajectoryOptimizer::GenerateDistanceApproachTraj()

距离逼近轨迹平滑。先通过 `DualVariableWarmStartProblem::Solve()` 热启动对偶变量，再调用 `DistanceApproachProblem::Solve()` 求解非线性优化问题。若平滑失败且 `FLAGS_enable_smoother_failsafe` 为 true，则回退使用粗路径作为结果。

### OpenSpaceTrajectoryOptimizer::GenerateDecoupledTraj()

解耦轨迹平滑。调用 `IterativeAnchoringSmoother::Smooth()` 进行迭代锚点平滑，适用于不需要严格动力学约束的场景。

### OpenSpaceTrajectoryProvider::IsVehicleNearDestination()

判断车辆是否到达目标位姿。将目标点从局部坐标变换到世界坐标，比较距离和航向角差值，满足阈值时标记目的地已到达。

### OpenSpaceTrajectoryProvider::LoadResult()

将优化器的输出拼接为完整轨迹。先获取优化轨迹和拼接轨迹，调整时间戳和弧长后拼接。若存在目标停车位，还会在轨迹末尾追加一段减速停止的直线段。

## 配置

配置通过 `OpenSpaceTrajectoryProviderConfig` protobuf 定义：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_open_space_planner_thread` | bool | true | 是否启用多线程模式 |
| `open_space_planning_period` | double | 4.0 | 开放空间规划周期（秒） |
| `open_space_standstill_acceleration` | double | 0.0 | 到达终点停车的减速度（m/s^2） |
| `open_space_straight_trajectory_length` | double | 0.0 | 泊车完成后直行轨迹长度（米） |
| `open_space_trajectory_stitching_preserved_length` | double | inf | 轨迹拼接保留长度 |

`OpenSpaceTrajectoryOptimizerConfig` 中的 `PlannerOpenSpaceConfig` 包含：

- **warm_start_config** - Hybrid A* 网格分辨率、步长、惩罚系数
- **dual_variable_warm_start_config** - 对偶变量热启动的 IPOPT/OSQP 参数
- **distance_approach_config** - 距离逼近平滑的权重、速度/加速度约束、IPOPT 参数
- **iterative_anchoring_smoother_config** - 迭代锚点平滑的 FEM 偏差平滑、S 曲线、碰撞缩减比等
- `delta_t` (0.5) - 离散时间步长
- `near_destination_threshold` (0.1) - 到达终点距离阈值
- `is_near_destination_theta_threshold` (0.1) - 到达终点航向角阈值

## 调用关系

```text
PlanningComponent
  -> Task::Execute()
    -> OpenSpaceTrajectoryProvider::Process()
      ├── [ParkAndGo 检查阶段] GenerateStopTrajectory()
      ├── [多线程模式]
      │     ├── GenerateTrajectoryThread()  (异步线程)
      │     │     └── OpenSpaceTrajectoryOptimizer::Plan()
      │     ├── IsVehicleNearDestination()
      │     ├── LoadResult()
      │     └── ReuseLastFrameResult()
      └── [单线程模式]
            ├── IsVehicleNearDestination()
            └── OpenSpaceTrajectoryOptimizer::Plan()
                  ├── HybridAStar::Plan()            (粗路径)
                  ├── TrajectoryPartition()           (分段)
                  ├── DualVariableWarmStartProblem    (对偶热启动)
                  ├── DistanceApproachProblem          (距离逼近平滑)
                  │   或 IterativeAnchoringSmoother   (迭代锚点平滑)
                  ├── CombineTrajectories()           (分段拼接)
                  └── LoadTrajectory()                (输出)
```
