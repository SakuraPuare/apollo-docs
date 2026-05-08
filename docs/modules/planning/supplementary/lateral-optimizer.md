# LateralQPOptimizer 横向 QP 优化器

> 源码位置：`modules/planning/planners/lattice/trajectory_generation/lateral_qp_optimizer.h/.cc`
> `modules/planning/planners/lattice/trajectory_generation/lateral_osqp_optimizer.h/.cc`

## 模块定位

横向 QP 优化器是 Lattice Planner 轨迹生成阶段的横向路径优化组件。在 Lattice Planner 的 Step 5 中，纵向轨迹由采样+评估产生，而横向轨迹则通过 QP（二次规划）求解器在给定的横向边界约束内生成最优横向偏移曲线。

调用链：`LatticePlanner::PlanOnReferenceLine` → `Trajectory1dGenerator` → `LateralOSQPOptimizer::optimize()`

## 1. LateralQPOptimizer — 基类

### 类声明

```cpp
class LateralQPOptimizer {
 public:
  LateralQPOptimizer() = default;
  virtual ~LateralQPOptimizer() = default;

  virtual bool optimize(
      const std::array<double, 3>& d_state, const double delta_s,
      const std::vector<std::pair<double, double>>& d_bounds) = 0;

  virtual PiecewiseJerkTrajectory1d GetOptimalTrajectory() const;
  virtual std::vector<common::FrenetFramePoint> GetFrenetFramePath() const;

 protected:
  double delta_s_ = FLAGS_default_delta_s_lateral_optimization;
  std::vector<double> opt_d_;
  std::vector<double> opt_d_prime_;
  std::vector<double> opt_d_pprime_;
};
```

### GetOptimalTrajectory()

```cpp
PiecewiseJerkTrajectory1d LateralQPOptimizer::GetOptimalTrajectory() const {
  PiecewiseJerkTrajectory1d optimal_trajectory(
      opt_d_.front(), opt_d_prime_.front(), opt_d_pprime_.front());
  for (size_t i = 1; i < opt_d_.size(); ++i) {
    double j = (opt_d_pprime_[i] - opt_d_pprime_[i - 1]) / delta_s_;
    optimal_trajectory.AppendSegment(j, delta_s_);
  }
  return optimal_trajectory;
}
```

- 将离散优化结果 `(d, d', d'')` 转换为连续的 `PiecewiseJerkTrajectory1d`
- 通过相邻二阶导差分计算 jerk（三阶导），逐段拼接
- `lateral_qp_optimizer.cc:L24-L35`

### GetFrenetFramePath()

```cpp
std::vector<common::FrenetFramePoint> LateralQPOptimizer::GetFrenetFramePath() const {
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  for (size_t i = 0; i < opt_d_.size(); ++i) {
    frenet_frame_point.set_s(accumulated_s);
    frenet_frame_point.set_l(opt_d_[i]);
    frenet_frame_point.set_dl(opt_d_prime_[i]);
    frenet_frame_point.set_ddl(opt_d_pprime_[i]);
    frenet_frame_path.push_back(std::move(frenet_frame_point));
    accumulated_s += delta_s_;
  }
  return frenet_frame_path;
}
```

- 将优化结果转换为 Frenet 坐标路径点序列
- 每个点包含 `(s, l, dl/ds, ddl/ds²)`
- `lateral_qp_optimizer.cc:L37-L51`

## 2. LateralOSQPOptimizer — OSQP 实现

### 类声明

```cpp
class LateralOSQPOptimizer : public LateralQPOptimizer {
 public:
  bool optimize(const std::array<double, 3>& d_state, const double delta_s,
                const std::vector<std::pair<double, double>>& d_bounds) override;
 private:
  void CalculateKernel(const std::vector<std::pair<double, double>>& d_bounds,
                       std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);
};
```

### optimize()

**输入参数：**
- `d_state`：初始横向状态 `[d₀, d₀', d₀'']`（横向偏移、一阶导、二阶导）
- `delta_s`：纵向采样间距
- `d_bounds`：每个采样点的横向边界 `[(l_min, l_max), ...]`

**决策变量布局：**
- 变量总数 `kNumParam = 3 * num_var`
- `[0, num_var)`：横向偏移 `d_i`
- `[num_var, 2*num_var)`：一阶导 `d_i'`
- `[2*num_var, 3*num_var)`：二阶导 `d_i''`

**约束构造（`lateral_osqp_optimizer.cc:L48-L100`）：**

1. **Jerk 约束**：`d''_{i+1} - d''_i ∈ [-jerk_max * Δs, jerk_max * Δs]`
2. **一阶导连续性**：`d'_{i+1} - d'_i - 0.5*Δs*(d''_i + d''_{i+1}) = 0`
3. **位置连续性**：`d_{i+1} - d_i - d'_i*Δs - d''_i*Δs²/3 - d''_{i+1}*Δs²/6 = 0`
4. **边界约束**：`d_bounds[i].first ≤ d_i ≤ d_bounds[i].second`
5. **初始状态约束**：`d_0 = d_state[0], d'_0 = d_state[1], d''_0 = d_state[2]`

**目标函数（CalculateKernel）：**

```cpp
void LateralOSQPOptimizer::CalculateKernel(...) {
  for (int i = 0; i < kNumParam; ++i) {
    if (i < d_bounds.size()) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_offset +
                      2.0 * FLAGS_weight_lateral_obstacle_distance;
    } else if (i < 2 * d_bounds.size()) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_derivative;
    } else {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_second_order_derivative;
    }
  }
}
```

- 对角 Hessian 矩阵，三组权重分别惩罚：横向偏移、一阶导（方向变化率）、二阶导（曲率）
- `lateral_osqp_optimizer.cc:L196-L219`

**求解与结果提取（`lateral_osqp_optimizer.cc:L160-L193`）：**

```cpp
osqp_solve(work);
for (int i = 0; i < num_var; ++i) {
  opt_d_.push_back(work->solution->x[i]);
  opt_d_prime_.push_back(work->solution->x[i + prime_offset]);
  opt_d_pprime_.push_back(work->solution->x[i + pprime_offset]);
}
opt_d_pprime_[num_var - 1] = 0.0;  // 末端二阶导置零
```

- 末端 `d''` 强制为 0，确保轨迹末端曲率为零（直行趋势）

## 配置项

| 参数 | 类型 | 说明 |
|------|------|------|
| `FLAGS_weight_lateral_offset` | double | 横向偏移权重 |
| `FLAGS_weight_lateral_obstacle_distance` | double | 障碍物距离权重 |
| `FLAGS_weight_lateral_derivative` | double | 横向一阶导权重 |
| `FLAGS_weight_lateral_second_order_derivative` | double | 横向二阶导权重 |
| `FLAGS_lateral_third_order_derivative_max` | double | 最大横向 jerk |
| `FLAGS_default_delta_s_lateral_optimization` | double | 默认纵向采样间距 |

## 调用关系

```
Trajectory1dGenerator
  └── LateralOSQPOptimizer::optimize()
        ├── CalculateKernel() → P 矩阵（目标函数）
        ├── 构造约束矩阵 A
        ├── osqp_solve()
        └── GetOptimalTrajectory() / GetFrenetFramePath()
```
