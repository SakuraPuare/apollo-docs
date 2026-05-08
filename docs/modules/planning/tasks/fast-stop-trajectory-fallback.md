---
title: "FastStopTrajectoryFallback 快速停车轨迹回退"
---

# FastStopTrajectoryFallback 快速停车轨迹回退

> 源码路径：`modules/planning/tasks/fast_stop_trajectory_fallback/`

## 概述

`FastStopTrajectoryFallback` 是规划模块中 `TrajectoryFallbackTask` 的插件实现，用于在紧急或异常情况下生成快速停车的速度曲线。当规划器无法生成正常轨迹时，该任务作为回退策略被调用，使用分段二次规划（Piecewise Jerk）求解器生成平滑减速至停车的速度剖面。

## 核心类

### FastStopTrajectoryFallback

继承自 `TrajectoryFallbackTask`，通过 CyberRT 插件机制注册为 `Task` 类型插件。

```cpp
class FastStopTrajectoryFallback : public TrajectoryFallbackTask {
 private:
  SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                  const double stop_distance = 0.0) override;

  static SpeedData GenerateStopProfile(const double init_speed,
                                       const double init_acc);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::FastStopTrajectoryFallback, Task)
```

**源码**：`modules/planning/tasks/fast_stop_trajectory_fallback/fast_stop_trajectory_fallback.h`

## 核心函数

### FastStopTrajectoryFallback::GenerateFallbackSpeed()

重写父类方法，使用 Piecewise Jerk 优化器生成停车速度曲线。

```cpp
SpeedData FastStopTrajectoryFallback::GenerateFallbackSpeed(
    const EgoInfo* ego_info, const double stop_distance) {
  const double init_v = ego_info->start_point().v();
  const double init_a = ego_info->start_point().a();
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // 已停车则直接返回零速点
  if (init_v <= 0.0 && init_a <= 0.0) {
    SpeedData speed_data;
    speed_data.AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
    SpeedProfileGenerator::FillEnoughSpeedPoints(&speed_data);
    return speed_data;
  }

  // 构建 PiecewiseJerkSpeedProblem 并求解
  std::array<double, 3> init_s = {0.0, init_v, init_a};
  PiecewiseJerkSpeedProblem problem(num_of_knots, delta_t, init_s);
  problem.set_x_ref(1.0, end_state_ref);   // 目标位置参考
  problem.set_x_bounds(0.0, fmin(stop_distance, 100.0));
  problem.set_dx_bounds(0.0, fmin(upper_speed_limit, init_v));
  problem.set_ddx_bounds(veh_param.max_deceleration(), 0.0);
  problem.set_dddx_bound(jerk_lower_bound, 0.0);

  if (!problem.Optimize()) {
    return GenerateStopProfile(init_v, init_a);  // 优化失败用恒减速回退
  }

  // 提取结果构建 SpeedData（略）
}
```

**职责**：生成快速停车的速度剖面
**输入**：`ego_info`（当前车辆状态），`stop_distance`（目标停车距离，默认 0）
**输出**：`SpeedData` 速度曲线
**关键步骤**：

1. 读取初始速度和加速度，若已停车则返回零速点
2. 构建 `PiecewiseJerkSpeedProblem`，设置位置/速度/加速度/jerk 约束边界
3. 以 `stop_distance` 为位置参考进行优化
4. 优化成功则从结果中提取 s/v/a 构建 `SpeedData`
5. 优化失败则降级为 `GenerateStopProfile()` 恒减速方案

### FastStopTrajectoryFallback::GenerateStopProfile()

静态方法，当优化求解失败时的降级方案，使用恒定减速度生成停车曲线。

```cpp
SpeedData FastStopTrajectoryFallback::GenerateStopProfile(
    const double init_speed, const double init_acc) {
  const double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;
  double acc = FLAGS_slowdown_profile_deceleration;

  SpeedData speed_data;
  speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, init_acc, 0.0);
  for (double t = unit_t; t < max_t; t += unit_t) {
    v = fmax(0.0, pre_v + unit_t * acc);
    s = fmax(pre_s, pre_s + 0.5 * (pre_v + (pre_v + v)) * unit_t);
    speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}
```

**职责**：以恒定减速度生成紧急停车曲线
**输入**：`init_speed`（初始速度），`init_acc`（初始加速度）
**输出**：`SpeedData` 速度曲线
**关键步骤**：

1. 以 `FLAGS_slowdown_profile_deceleration` 为减速度
2. 按 `FLAGS_fallback_time_unit` 步长逐点计算 v 和 s
3. 速度降至 0 后保持不动

## 配置

| 标志位 | 说明 |
|--------|------|
| `FLAGS_fallback_time_unit` | 回退曲线时间步长（秒） |
| `FLAGS_fallback_total_time` | 回退曲线总时长（秒） |
| `FLAGS_slowdown_profile_deceleration` | 降级方案使用的恒定减速度 |
| `FLAGS_planning_upper_speed_limit` | 速度上界 |
| `FLAGS_longitudinal_jerk_lower_bound` | 纵向 jerk 下界 |

## 调用关系

- **父类**：`TrajectoryFallbackTask`（定义回退任务接口）
- **依赖**：`PiecewiseJerkSpeedProblem`（二次规划求解器）、`SpeedProfileGenerator`（速度曲线工具）
- **被调用**：规划器在正常轨迹生成失败时，通过插件机制调用此 Task
