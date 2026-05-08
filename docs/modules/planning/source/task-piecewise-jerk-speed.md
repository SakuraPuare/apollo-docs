# PiecewiseJerkSpeed 速度优化器

> 源码位置：`modules/planning/tasks/piecewise_jerk_speed/`

## 模块定位

PiecewiseJerkSpeedOptimizer 是 Apollo 默认的纵向速度规划器。它在 ST 图上构建 QP 问题，以最小化加速度和 jerk 为目标，在满足 ST 边界约束和速度限制的条件下求解最优速度剖面。

---

## 类声明

```cpp
class PiecewiseJerkSpeedOptimizer : public SpeedOptimizer {
 public:
  bool Init(const string& config_dir, const string& name,
            const shared_ptr<DependencyInjector>&) override;
 private:
  Status Process(const PathData&, const TrajectoryPoint&, SpeedData*) override;
  void AdjustInitStatus(const vector<pair<double,double>> s_dot_bound,
                        double delta_t, array<double,3>& init_s);
  PiecewiseJerkSpeedOptimizerConfig config_;
};
```

---

## Process() — 主流程

```cpp
Status PiecewiseJerkSpeedOptimizer::Process(
    const PathData& path_data, const TrajectoryPoint& init_point,
    SpeedData* speed_data) {
  // 0. 已到达目的地则跳过
  if (reference_line_info_->ReachedDestination()) return OK;

  // 1. 初始状态 [s0, v0, a0]
  array<double, 3> init_s = {0.0, st_graph_data.init_point().v(),
                              st_graph_data.init_point().a()};
  // 倒车时翻转速度/加速度符号
  if (gear == GEAR_REVERSE) { init_s[1] = max(-v, 0); init_s[2] = -a; }

  // 2. 构建 s 边界（从 ST 边界获取）
  for (int i = 0; i < num_of_knots; ++i) {
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      switch (boundary->boundary_type()) {
        case STOP/YIELD/FOLLOW: s_upper_bound = min(s_upper, ...);
        case OVERTAKE:          s_lower_bound = max(s_lower, ...);
      }
    }
    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }

  // 3. 构建速度边界和参考速度
  for (int i = 0; i < num_of_knots; ++i) {
    v_upper = min(speed_limit.GetSpeedLimitByS(path_s), planning_upper_speed);
    s_dot_bounds.emplace_back(0.0, v_upper);
    dx_ref[i] = min(cruise_speed, v_upper);
    penalty_dx[i] = |kappa| * kappa_penalty_weight;  // 弯道减速
  }

  // 4. 调整初始状态可行性
  AdjustInitStatus(s_dot_bounds, delta_t, init_s);

  // 5. 构建并求解 QP 问题
  PiecewiseJerkSpeedProblem problem(num_of_knots, delta_t, init_s);
  problem.set_weight_ddx(config_.acc_weight());
  problem.set_weight_dddx(config_.jerk_weight());
  problem.set_x_bounds(s_bounds);
  problem.set_dx_bounds(s_dot_bounds);
  problem.set_ddx_bounds(max_deceleration, max_acceleration);
  problem.set_dddx_bound(jerk_lower, jerk_upper);
  problem.set_dx_ref(dx_ref_weight, dx_ref);
  problem.set_x_ref(ref_s_weight, x_ref);
  problem.set_penalty_dx(penalty_dx);
  problem.Optimize();

  // 6. 提取结果
  const auto& s = problem.opt_x();
  const auto& ds = problem.opt_dx();
  const auto& dds = problem.opt_ddx();
  for (int i = 0; i < num_of_knots; ++i)
    speed_data->AppendSpeedPoint(s[i], delta_t*i, ds[i], dds[i], jerk);
}
```

- `piecewise_jerk_speed_optimizer.cc:L55-L234`

---

## ST 边界处理

| 边界类型 | 处理方式 |
|---------|---------|
| STOP | 限制 s 上界（不能超过停车点） |
| YIELD | 限制 s 上界（让行） |
| FOLLOW | 限制 s 上界（跟车） |
| OVERTAKE | 限制 s 下界（必须超过障碍物） |

---

## AdjustInitStatus() — 初始状态修正

```cpp
void AdjustInitStatus(const vector<pair<double,double>> s_dot_bound,
                      double delta_t, array<double,3>& init_s) {
  // 前向模拟：检查初始加速度是否会导致速度越界
  for (size_t i = 1; i < s_dot_bound.size(); i++) {
    a_min += delta_t * jerk_upper;
    a_max += delta_t * jerk_lower;
    v_min += 0.5 * delta_t * (a_min + last_a_min);
    v_max += 0.5 * delta_t * (a_max + last_a_max);
    if (v_min < bound.first || v_max > bound.second) {
      init_s[2] = 0;  // 将初始加速度置零
      return;
    }
  }
}
```

- 防止初始加速度过大导致 QP 无解
- `piecewise_jerk_speed_optimizer.cc:L236-L260`

---

## 优化失败回退

```cpp
if (!problem.Optimize()) {
  // 放松速度约束后重试
  problem.set_dx_bounds(0.0, max(upper_speed_limit, init_v));
  if (!problem.Optimize()) {
    speed_data->clear();
    return PLANNING_ERROR;
  }
}
```

- 第一次失败后放松速度上界约束重试
- 由 `FLAGS_speed_optimize_fail_relax_velocity_constraint` 控制

---

## 关键配置项

| 参数 | 说明 |
|------|------|
| `acc_weight` | 加速度惩罚权重 |
| `jerk_weight` | jerk 惩罚权重 |
| `ref_v_weight` | 参考速度跟踪权重 |
| `ref_s_weight` | 参考位移跟踪权重 |
| `kappa_penalty_weight` | 曲率减速惩罚权重 |

## QP 问题结构

- 决策变量：`s[0..N]`（每个时间步的纵向位移）
- 目标函数：`min Σ(ddx² * w_acc + dddx² * w_jerk + (dx-dx_ref)² * w_ref)`
- 约束：s_bounds、dx_bounds、ddx_bounds、dddx_bounds
- 缩放因子：`{1.0, 10.0, 100.0}`（s, v, a 量级归一化）
