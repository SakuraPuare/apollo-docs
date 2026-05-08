# HorizonOptimizer 预测地平线优化器

> 源码位置：`modules/planning/tasks/horizon_optimizer/`

## 模块定位

HorizonOptimizer 是基于预测地平线的多模态速度优化器。它生成多条候选速度曲线（保守/中等/激进），评估每条轨迹与周围障碍物预测轨迹的碰撞风险和舒适度代价，选择最优轨迹并进行交互调整和平滑。

继承关系：
```
Task → SpeedOptimizer → HorizonOptimizer
```

---

## Process 主流程

```cpp
Status HorizonOptimizer::Process(path_data, init_point, speed_data) {
  // 1. 提取初始状态 (s=0, v, a)
  init_s = {0.0, st_graph_data.init_point().v(), init_point.a()};

  // 2. 计算时间参数
  delta_t = config_.delta_t();  // 默认 0.1s
  horizon_length = config_.horizon_length();

  // 3. 更新预测数据
  predictions_ = UpdatePredictions(st_graph_data);

  // 4. 生成候选轨迹集合
  candidates = GenerateCandidateTrajectories(path_data, init_point, num_candidates);

  // 5. 评估每条候选轨迹的代价
  for (candidate : candidates)
    costs.push_back(EvaluateTrajectoryCost(candidate, predictions_, path_data));

  // 6. 选择代价最低的轨迹
  best_idx = argmin(costs);

  // 7. 交互调整
  adjusted = AdjustForInteraction(candidates[best_idx], predictions_);

  // 8. 平滑
  smoothed = SmoothTrajectory(adjusted);

  // 9. 验证可行性
  if (!ValidateTrajectory(smoothed)) return fallback;

  *speed_data = smoothed;
}
```

---

## GenerateCandidateTrajectories — 候选轨迹生成

生成 `num_candidates` 条不同速度目标的轨迹：

```cpp
vector<SpeedData> GenerateCandidateTrajectories(path_data, init_point, N) {
  for (i = 0; i < N; ++i) {
    // 每条轨迹的目标速度递减 15%
    speed_ratio = 1.0 - i * 0.15;
    target_speed = min(cruise_speed * speed_ratio, max_speed);

    // 欧拉积分生成速度曲线
    for (k = 0; k < num_knots; ++k) {
      a = clamp((target_speed - current_v) * 2.0, -a_max, a_max);
      current_s += current_v * dt + 0.5 * a * dt²;
      current_v = max(0, current_v + a * dt);
      candidate.AppendSpeedPoint(s, t, v, a, jerk);
    }
  }
}
```

---

## EvaluateTrajectoryCost — 轨迹代价评估

目标函数：
```
J = w_collision × CollisionRisk
  + w_accel × Σ a²
  + w_speed × Σ (v - v_cruise)²
  + w_time × total_time
```

| 代价项 | 权重 | 说明 |
|--------|------|------|
| 碰撞风险 | 1000.0 | 与预测轨迹的最小距离 |
| 加速度 | 10.0 | 加速度平方和 |
| 速度偏差 | 1.0 | 与巡航速度的偏差 |
| 行程时间 | 0.1 | 总时间（鼓励效率） |

---

## ComputeCollisionRisk — 碰撞风险计算

```cpp
double ComputeCollisionRisk(trajectory, predictions) {
  for (ego_point : trajectory) {
    for (obs_prediction : predictions) {
      for (obs_point : obs_prediction.trajectory) {
        s_distance = |ego_s - obs_s|;
        if (s_distance < safety_distance) {
          risk = max(risk, 1.0 - s_distance / safety_distance);
        }
      }
    }
  }
  return min(1.0, risk);
}
```

---

## AdjustForInteraction — 交互调整

检测轨迹中与障碍物距离过近的区域，进行减速调整：

```cpp
SpeedData AdjustForInteraction(trajectory, predictions) {
  for (point : trajectory) {
    if (obstacle_within_2x_safety_distance) {
      // 降低该点速度
      point.v *= deceleration_factor;
    }
  }
}
```

---

## SplitHorizon — 地平线分割

将总预测时长划分为多个滚动窗口：

```cpp
vector<pair<double,double>> SplitHorizon(total_time, horizon_length) {
  // 例如 total_time=8s, horizon_length=3s
  // → [(0,3), (3,6), (6,8)]
}
```

---

## 关键配置参数

| 参数 | 说明 |
|------|------|
| `delta_t` | 时间步长（默认 0.1s） |
| `horizon_length` | 单个优化窗口时长 |
| `num_candidates` | 候选轨迹数量（默认 3） |
| `safety_distance` | 安全距离阈值（默认 2.0m） |

---

## PredictionData 数据结构

```cpp
struct PredictionData {
  struct ObstaclePrediction {
    int obstacle_id;
    vector<TrajectoryPoint> trajectory;  // 预测轨迹
    vector<double> probabilities;        // 各轨迹概率
    double timestamp;
  };
  vector<ObstaclePrediction> predictions;
  double horizon_duration;
  double prediction_frequency;
};
```
