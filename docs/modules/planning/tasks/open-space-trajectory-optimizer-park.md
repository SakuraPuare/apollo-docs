---
title: "泊车开放空间轨迹优化器"
---

# 泊车开放空间轨迹优化器

> 源码路径: `modules/planning/tasks/open_space_trajectory_optimizer_park/`

## 概述

泊车开放空间轨迹优化器是 Apollo 规划模块中的一个 Task 插件，专门用于泊车场景下的开放空间轨迹平滑与优化。该模块接收路径规划阶段生成的粗略轨迹（warm start trajectory），通过数值优化方法对轨迹进行平滑处理，使其满足车辆运动学约束和障碍物避碰要求。

模块支持两种执行模式：同步执行和异步线程执行。在异步模式下，优化过程在独立线程中运行，不阻塞主流程；若优化失败则自动触发重规划。模块还支持两种轨迹平滑算法：距离逼近法（Distance Approach）和解耦迭代锚点平滑法（Iterative Anchoring），以及并行分段平滑能力。

## 核心类

### `OpenSpaceTrajectoryOptimizerPark`

继承自 `TrajectoryOptimizer`，作为 Cyber RT 插件注册到任务框架中。负责管理优化流程的生命周期，包括初始化、异步调度、结果加载和调试信息复用。

```cpp
class OpenSpaceTrajectoryOptimizerPark : public TrajectoryOptimizer {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;
 private:
  void Process() override;
  void Optimize();
  void GenerateTrajectoryThread();
  void LoadResult(DiscretizedTrajectory* const trajectory_data);
  void ReuseLastFrameDebug(const Frame* last_frame);

  std::unique_ptr<Optimizer> trajectory_optimizer_;
  OpenSpaceTrajectoryOptimizerParkConfig config_;
  bool thread_init_flag_;
  bool replan_flag_;
  std::future<void> task_future_;
  bool trajectory_update_;
  std::mutex data_mutex_;
};
```

### `Optimizer`

核心优化器类，封装了全部轨迹平滑算法。持有三种平滑器实例：`DistanceApproachProblem`、`DualVariableWarmStartProblem` 和 `IterativeAnchoringSmoother`，根据配置和 gflags 选择不同平滑策略。

```cpp
class Optimizer {
 public:
  Optimizer(const OpenSpaceTrajectoryOptimizerParkConfig& config);
  common::Status Plan(const OpenSpaceInfo open_space_info);
  void GetOptimizedTrajectory(DiscretizedTrajectory& optimized_trajectory);
  void UpdateDebugInfo(planning_internal::OpenSpaceDebug* open_space_debug);
 private:
  // ...
  std::unique_ptr<DistanceApproachProblem> distance_approach_;
  std::unique_ptr<DualVariableWarmStartProblem> dual_variable_warm_start_;
  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;
  DiscretizedTrajectory optimized_trajectory_;
};
```

## 核心函数

### `OpenSpaceTrajectoryOptimizerPark::Process()`

主流程入口。若当前帧无需重规划（`replan_flag` 为 false），直接复用上一帧的优化结果；否则检查是否有可用的初始轨迹，然后调用 `Optimize()` 执行优化。优化完成后加载结果并同步调试信息。

### `OpenSpaceTrajectoryOptimizerPark::Optimize()`

根据配置决定执行模式。若启用异步线程（`enable_trajectory_optimize_thread`），通过 `cyber::Async` 在后台线程中执行 `GenerateTrajectoryThread`；否则同步执行并在完成后立即加载结果。

### `Optimizer::Plan()`

核心优化逻辑。执行流程如下：

1. 从 `OpenSpaceInfo` 中提取障碍物信息、ROI 边界和粗略轨迹
2. 将轨迹坐标归一化到局部坐标系
3. 根据 `FLAGS_enable_parallel_trajectory_smoothing` 决定是否并行分段处理
4. 对每个轨迹段调用 `GenerateDistanceApproachTraj` 或 `GenerateDecoupledTraj`
5. 如为并行模式，通过 `CombineTrajectories` 合并各段结果
6. 将优化后轨迹坐标反归一化回世界坐标系
7. 通过 `LoadTrajectory` 加载最终轨迹

### `Optimizer::GenerateDistanceApproachTraj()`

使用距离逼近法进行轨迹平滑。先通过 `DualVariableWarmStartProblem` 计算对偶变量初始值，再调用 `DistanceApproachProblem::Solve` 求解非线性优化问题。若求解失败且启用了 failsafe（`FLAGS_enable_smoother_failsafe`），则直接使用 warm start 结果作为输出。

### `Optimizer::GenerateDecoupledTraj()`

使用迭代锚点平滑法进行轨迹平滑，调用 `IterativeAnchoringSmoother::Smooth` 完成解耦优化。

### `Optimizer::PartitionTrajectory()`

根据前进/后退挡位变化将轨迹分段。通过比较航向角与相邻点连线角度判断当前挡位，挡位切换处分割为独立子轨迹，用于并行平滑。

### `Optimizer::CombineTrajectories()`

将并行平滑后的多个子轨迹段拼接为完整轨迹，去除段间重复的状态点。

### `Optimizer::LoadTrajectory()`

将 Eigen 矩阵形式的优化结果转换为 `DiscretizedTrajectory`，计算每个点的相对时间和累计弧长。

## 配置

模块通过 protobuf 配置文件 `OpenSpaceTrajectoryOptimizerParkConfig` 进行参数化，主要配置项包括：

| 配置项 | 说明 |
|---|---|
| `enable_trajectory_optimize_thread` | 是否启用异步线程执行优化 |
| `dual_variable_warm_start_config` | 对偶变量暖启动求解器配置 |
| `distance_approach_config` | 距离逼近法求解器配置 |
| `iterative_anchoring_smoother_config` | 迭代锚点平滑器配置 |

此外，模块行为受以下全局 gflags 控制：

| gflag | 说明 |
|---|---|
| `FLAGS_enable_parallel_trajectory_smoothing` | 是否启用并行分段平滑 |
| `FLAGS_use_iterative_anchoring_smoother` | 是否使用迭代锚点平滑（否则使用距离逼近法） |
| `FLAGS_use_dual_variable_warm_start` | 是否启用对偶变量暖启动 |
| `FLAGS_open_space_delta_t` | 离散时间步长 |
| `FLAGS_enable_smoother_failsafe` | 优化失败时是否回退到 warm start 结果 |
| `FLAGS_enable_record_debug` | 是否记录调试信息 |

## 调用关系

```text
OpenSpaceTrajectoryOptimizerPark::Process()
  |-- [非重规划] 复用上一帧轨迹 + ReuseLastFrameDebug()
  |-- [重规划] Optimize()
        |-- [异步] cyber::Async -> GenerateTrajectoryThread()
        |-- [同步] GenerateTrajectoryThread()
              |-- Optimizer::Plan()
                    |-- PathPointNormalizing() 坐标归一化
                    |-- [并行模式]
                    |     |-- PartitionTrajectory() 按挡位分段
                    |     |-- 各段 GenerateDistanceApproachTraj() / GenerateDecoupledTraj()
                    |     |-- CombineTrajectories() 合并
                    |-- [串行模式]
                    |     |-- GenerateDistanceApproachTraj()
                    |-- RecordDebugInfo() 记录调试信息
                    |-- PathPointDeNormalizing() 坐标反归一化
                    |-- LoadTrajectory() 生成最终轨迹
  |-- LoadResult() 加载结果到帧
  |-- UpdateDebugInfo() 同步调试信息
```

子模块调用关系：

- `GenerateDistanceApproachTraj` -> `DualVariableWarmStartProblem::Solve` -> `DistanceApproachProblem::Solve`
- `GenerateDecoupledTraj` -> `IterativeAnchoringSmoother::Smooth`
