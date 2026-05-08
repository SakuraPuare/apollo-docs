---
title: "PiecewiseJerkSpeedNonlinearOptimizer 非线性速度优化器"
---

# PiecewiseJerkSpeedNonlinearOptimizer 非线性速度优化器

> 源码位置：`modules/planning/tasks/piecewise_jerk_speed_nonlinear/`

## 模块定位

PiecewiseJerkSpeedNonlinearOptimizer 是两阶段速度优化器：先用 QP（二次规划）求解线性化问题获得初始解，再用 NLP（IPOPT 非线性规划）精细优化，引入路径曲率约束实现向心加速度限制。

继承关系：
```
Task → SpeedOptimizer → PiecewiseJerkSpeedNonlinearOptimizer
                              ↓ 使用
                        PiecewiseJerkSpeedProblem（QP 阶段）
                        PiecewiseJerkSpeedNonlinearIpoptInterface（NLP 阶段）
```

与 `PiecewiseJerkSpeedOptimizer`（纯 QP）的区别：本优化器额外考虑了**路径曲率对速度的约束**（向心加速度 = v² × κ），能在弯道处更精确地限速。

---

## Process 主流程

```cpp
Status Process(path_data, init_point, speed_data) {
  // 1. 设置状态和边界
  SetUpStatesAndBounds(path_data, *speed_data);

  // 2. QP 阶段：获得初始解
  OptimizeByQP(speed_data, &distance, &velocity, &acceleration);

  // 3. 检查速度限制可行性
  if (CheckSpeedLimitFeasibility()) {
    // 4. 平滑路径曲率
    SmoothPathCurvature(path_data);
    // 5. 平滑速度限制
    SmoothSpeedLimit();
    // 6. NLP 阶段：精细优化
    OptimizeByNLP(&distance, &velocity, &acceleration);
  }

  // 7. 组装 SpeedData 输出
  speed_data = assemble(distance, velocity, acceleration);
}
```

---

## SetUpStatesAndBounds — 状态与边界设置

```cpp
Status SetUpStatesAndBounds(path_data, speed_data) {
  // 初始状态
  s_init_ = 0.0;
  s_dot_init_ = init_point.v();
  s_ddot_init_ = init_point.a();

  // 动力学边界
  s_dot_max_ = max_speed;
  s_ddot_min_ = max_deceleration;
  s_ddot_max_ = max_acceleration;
  s_dddot_min/max_ = jerk_bounds;

  // 安全边界：遍历 ST 边界
  for (t = 0; t < total_time; t += delta_t) {
    for (boundary : st_boundaries) {
      switch (boundary_type) {
        case STOP/YIELD: s_upper = min(s_upper, boundary_s);
        case FOLLOW:     s_upper = min(s_upper, boundary_s);
                         s_soft_upper = s_upper - min(7.0, 2.5*v);
        case OVERTAKE:   s_lower = max(s_lower, boundary_s);
                         s_soft_lower = s_lower + 10.0;
      }
    }
    s_bounds_.push_back({s_lower, s_upper});
    s_soft_bounds_.push_back({s_soft_lower, s_soft_upper});
  }
}
```

---

## OptimizeByQP — QP 阶段

使用 `PiecewiseJerkSpeedProblem`（OSQP 求解器）：

```cpp
Status OptimizeByQP(speed_data, &distance, &velocity, &acceleration) {
  PiecewiseJerkSpeedProblem problem(num_of_knots_, delta_t_, init_states);
  problem.set_dx_bounds(0.0, max_speed);
  problem.set_ddx_bounds(s_ddot_min_, s_ddot_max_);
  problem.set_dddx_bound(s_dddot_min_, s_dddot_max_);
  problem.set_x_bounds(s_bounds_);

  // 权重设置
  problem.set_weight_ddx(config_.acc_weight());
  problem.set_weight_dddx(config_.jerk_weight());

  // 参考：DP 速度曲线
  x_ref = dp_speed_profile;
  problem.set_x_ref(config_.ref_s_weight(), x_ref);

  problem.Optimize();
  // 输出 distance, velocity, acceleration
}
```

---

## SmoothPathCurvature — 路径曲率平滑

将离散的路径曲率拟合为平滑的 PiecewiseJerk 曲线，供 NLP 使用：

```cpp
Status SmoothPathCurvature(path_data) {
  // 采样路径曲率
  for (s = 0; s < path_length; s += delta_s) {
    kappa_ref.push_back(path_data.GetKappa(s));
  }

  // 用 PiecewiseJerkPathProblem 平滑
  PiecewiseJerkPathProblem problem(size, delta_s, {kappa_ref[0], 0, 0});
  problem.set_x_ref(weight, kappa_ref);
  problem.Optimize();

  // 存储为 PiecewiseJerkTrajectory1d
  smoothed_path_curvature_ = result;
}
```

---

## SmoothSpeedLimit — 速度限制平滑

将阶梯状的速度限制平滑为连续曲线：

```cpp
Status SmoothSpeedLimit() {
  // 采样速度限制
  for (s = 0; s < 200m; s += 2.0) {
    speed_ref.push_back(speed_limit_.GetSpeedLimitByS(s));
  }

  // 用 PiecewiseJerkPathProblem 平滑
  PiecewiseJerkPathProblem problem(size, delta_s, {speed_ref[0], 0, 0});
  problem.set_x_ref(10.0, speed_ref);
  problem.Optimize();

  smoothed_speed_limit_ = result;
}
```

---

## OptimizeByNLP — NLP 阶段（IPOPT）

```cpp
Status OptimizeByNLP(&distance, &velocity, &acceleration) {
  auto ptr_interface = new PiecewiseJerkSpeedNonlinearIpoptInterface(
      s_init_, s_dot_init_, s_ddot_init_, delta_t_, num_of_knots_,
      total_length_, s_dot_max_, s_ddot_min_, s_ddot_max_,
      s_dddot_min_, s_dddot_max_);

  // 设置 warm start（QP 结果）
  ptr_interface->set_warm_start({distance, velocity, acceleration});

  // 设置曲率曲线和速度限制曲线
  ptr_interface->set_curvature_curve(smoothed_path_curvature_);
  ptr_interface->set_speed_limit_curve(smoothed_speed_limit_);

  // 设置安全边界
  ptr_interface->set_safety_bounds(s_bounds_);
  ptr_interface->set_soft_safety_bounds(s_soft_bounds_);

  // 设置权重
  ptr_interface->set_w_overall_a(config_.acc_weight());
  ptr_interface->set_w_overall_j(config_.jerk_weight());
  ptr_interface->set_w_overall_centripetal_acc(config_.lat_acc_weight());
  ptr_interface->set_w_reference_speed(config_.ref_v_weight());
  ptr_interface->set_w_reference_spatial_distance(config_.ref_s_weight());

  // IPOPT 求解
  Ipopt::IpoptApplication app;
  app->Options()->SetIntegerValue("max_iter", config_.max_iter());
  app->OptimizeTNLP(ptr_interface);
}
```

---

## IPOPT 接口 — 目标函数

NLP 目标函数包含：

| 项 | 公式 | 权重 |
|----|------|------|
| 加速度平方 | Σ a² | `w_overall_a_` |
| Jerk 平方 | Σ ((a[i+1]-a[i])/Δt)² | `w_overall_j_` |
| 向心加速度 | Σ (v² × κ)² | `w_overall_centripetal_acc_` |
| 参考速度偏差 | Σ (v - v_ref)² | `w_ref_v_` |
| 参考距离偏差 | Σ (s - s_ref)² | `w_ref_s_` |
| 终点状态 | (s-s_t)² + (v-v_t)² + (a-a_t)² | `w_target_*` |
| 软边界松弛 | Σ slack² | `w_soft_s_bound_` |

约束条件：
- 运动学递推：s[i+1] = s[i] + v[i]Δt + 0.5a[i]Δt²
- 速度递推：v[i+1] = v[i] + a[i]Δt
- 边界约束：s_lower ≤ s[i] ≤ s_upper
- 动力学约束：a_min ≤ a[i] ≤ a_max, jerk_min ≤ j ≤ jerk_max

---

## 关键配置参数

| 参数 | 说明 |
|------|------|
| `acc_weight` | 加速度代价权重 |
| `jerk_weight` | Jerk 代价权重 |
| `lat_acc_weight` | 向心加速度代价权重 |
| `ref_v_weight` | 参考速度偏差权重 |
| `ref_s_weight` | 参考距离偏差权重（DP 结果） |
| `max_iter` | IPOPT 最大迭代次数 |
