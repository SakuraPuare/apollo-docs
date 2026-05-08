---
title: "速度 Fallback 任务"
---

# 速度 Fallback 任务

> 源码位置：
> - `modules/planning/tasks/fast_stop_trajectory_fallback/`
> - `modules/planning/tasks/smooth_stop_trajectory_fallback/`

## 模块定位

当正常速度优化失败时（如 QP 求解失败、约束不可行），系统需要一个安全的兜底速度曲线来保证车辆平稳停车。Apollo 提供两种 fallback 策略：

| 任务 | 策略 | 适用场景 |
|------|------|----------|
| `FastStopTrajectoryFallback` | QP 优化停车 + 恒减速兜底 | 需要快速停车 |
| `SmoothStopTrajectoryFallback` | S 形 jerk 平滑停车 | 追求舒适性的停车 |

继承关系：
```
Task → TrajectoryFallbackTask → FastStopTrajectoryFallback
                              → SmoothStopTrajectoryFallback
```

---

## FastStopTrajectoryFallback

### GenerateFallbackSpeed

```cpp
SpeedData FastStopTrajectoryFallback::GenerateFallbackSpeed(
    const EgoInfo* ego_info, double stop_distance) {
  double init_v = ego_info->start_point().v();
  double init_a = ego_info->start_point().a();

  // 已停车则直接返回
  if (init_v <= 0 && init_a <= 0) return {(0, 0, 0, 0, 0)};

  // 方法1：QP 优化停车
  PiecewiseJerkSpeedProblem problem(num_knots, delta_t, {0, init_v, init_a});
  problem.set_x_ref(1.0, {stop_distance, ...});  // 参考终点
  problem.set_x_bounds(0, min(stop_distance, 100));
  problem.set_dx_bounds(0, min(speed_limit, init_v));  // 速度只减不增
  problem.set_ddx_bounds(max_deceleration, 0);         // 只允许减速
  problem.set_dddx_bound(jerk_lower, 0);              // jerk 只允许负值

  if (problem.Optimize()) return qp_result;

  // 方法2：恒减速兜底
  return GenerateStopProfile(init_v, init_a);
}
```

### GenerateStopProfile — 恒减速停车

```cpp
SpeedData GenerateStopProfile(double init_speed, double init_acc) {
  double acc = FLAGS_slowdown_profile_deceleration;  // 恒定减速度
  for (t = 0; t < max_t; t += unit_t) {
    v = max(0, pre_v + unit_t * acc);
    s = max(pre_s, pre_s + 0.5 * (pre_v + v) * unit_t);
    speed_data.AppendSpeedPoint(s, t, v, acc, 0);
  }
}
```

---

## SmoothStopTrajectoryFallback

### GenerateFallbackSpeed

更复杂的分支逻辑，优先保证平滑性：

```cpp
SpeedData SmoothStopTrajectoryFallback::GenerateFallbackSpeed(
    const EgoInfo* ego_info, double stop_distance) {
  double v0 = ego_info->start_point().v();
  double a0 = ego_info->start_point().a();

  // 获取 ST 下界用于碰撞检测
  GetLowerSTBound(lower_bounds);

  if (a0 <= 0) {
    if (v0 <= 0) return stop_profile;  // 已停车

    // 计算最小 jerk 使车辆恰好停下
    min_jerk = a0² / (2 * v0);

    if (jerk_upper < min_jerk) {
      // 用最小 jerk 生成停车曲线
      // s = v0*t + 0.5*a0*t² + jerk*t³/6
      if (!IsCollisionWithSpeedBoundaries(speed_data))
        return smooth_profile;
      else
        return SpeedPlanner::FastStop(v0, a0, max_decel);
    } else {
      // 用 S 形平滑停车
      SpeedPlanner::SmoothStop(v0, a0, max_decel, max_pos_jerk, max_neg_jerk);
      if (!IsCollisionWithSpeedBoundaries(speed_data))
        return smooth_profile;
      else
        return SpeedPlanner::FastStop(v0, a0, max_decel);
    }
  } else {
    // a0 > 0：先用 S 形停车
    SpeedPlanner::SmoothStop(v0, a0, max_decel, max_pos_jerk, max_neg_jerk);
    if (!IsCollisionWithSpeedBoundaries(speed_data))
      return smooth_profile;
    else
      return SpeedPlanner::FastStop(v0, a0, max_decel);
  }
}
```

### IsCollisionWithSpeedBoundaries — 碰撞检测

检查生成的速度曲线是否与 ST 边界下界碰撞：

```cpp
bool IsCollisionWithSpeedBoundaries(const SpeedData& speed_data) {
  for (lower_bound : lower_bounds) {
    for (point : lower_bound) {
      // 在 speed_data 中插值获取对应时刻的 s
      SpeedPoint sp;
      speed_data.EvaluateByTime(point.x(), &sp);
      if (sp.s() >= point.y())  // 超过障碍物下界
        return true;
    }
  }
  return false;
}
```

### GetLowerSTBound — 获取 ST 下界

```cpp
void GetLowerSTBound(vector<vector<Vec2d>>& lower_bounds) {
  for (boundary : st_boundaries) {
    // 提取每个 ST 边界的下边界点
    // 对同一时刻取最小 s 值
    // 按时间排序
    lower_bounds.push_back(sorted_lower_points);
  }
}
```

---

## SpeedPlanner 工具类

| 方法 | 说明 |
|------|------|
| `FastStop(v0, a0, max_decel)` | 恒减速快速停车 |
| `SmoothStop(v0, a0, max_decel, max_pos_jerk, max_neg_jerk)` | S 形 jerk 平滑停车 |

S 形停车的运动学特征：
```
Phase 1: jerk = -max_neg_jerk → 加速度从 a0 减小到 max_decel
Phase 2: jerk = 0            → 恒减速
Phase 3: jerk = +max_pos_jerk → 加速度从 max_decel 恢复到 0
```

---

## 关键 FLAGS 参数

| 参数 | 说明 |
|------|------|
| `fallback_time_unit` | Fallback 时间步长 |
| `fallback_total_time` | Fallback 总时长 |
| `slowdown_profile_deceleration` | 恒减速值（负值） |
| `longitudinal_jerk_lower_bound` | 纵向 jerk 下界 |
| `longitudinal_jerk_upper_bound` | 纵向 jerk 上界 |
