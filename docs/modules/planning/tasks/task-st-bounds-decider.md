---
title: "STBoundsDecider ST 可行驶边界决策器"
---

# STBoundsDecider ST 可行驶边界决策器

> 源码位置：`modules/planning/tasks/st_bounds_decider/`

## 模块定位

STBoundsDecider 是 `FLAGS_use_st_drivable_boundary` 模式下的 ST 边界生成器。与 SpeedBoundsDecider 不同，它通过扫描线算法逐时间步决策每个障碍物的超车/让行关系，生成连续的 ST 可行驶区域（drivable boundary），而非独立的 ST 边界。

---

## 类结构

```
STBoundsDecider (Decider)
├── STObstaclesProcessor  — 障碍物映射与决策管理
├── STGuideLine           — 引导线（期望 s-t 曲线）
└── STDrivingLimits       — 车辆动力学限制
```

---

## Process() — 主流程

```cpp
Status STBoundsDecider::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 初始化
  InitSTBoundsDecider(*frame, rli);

  // 2. 生成 Regular ST Bound（扫描线 + 决策排序）
  GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound, &st_guide_line);

  // 3. 设置到 StGraphData
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);
}
```

- `st_bounds_decider.cc:L52-L89`

---

## InitSTBoundsDecider()

```cpp
void InitSTBoundsDecider(const Frame& frame, ReferenceLineInfo* rli) {
  // 1. 障碍物映射到 ST 图
  st_obstacles_processor_.Init(path_length, total_time, path_data, path_decision, history);
  st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);

  // 2. 初始化引导线（desired_speed = 15 m/s）
  st_guide_line_.Init(desired_speed);

  // 3. 初始化动力学限制
  st_driving_limits_.Init(max_acc=2.5, max_dec=5.0, max_v=22.5, init_v);
}
```

---

## GenerateRegularSTBound() — 扫描线决策

核心算法：逐时间步（Δt = 0.1s）扫描，在每个时间步：

```cpp
for (size_t i = 0; i < st_bound->size(); ++i) {
  double t = i * 0.1;

  // 1. 车辆动力学约束 → [s_lower, s_upper]
  auto driving_limits = st_driving_limits_.GetVehicleDynamicsLimits(t);
  s_lower = max(s_lower, driving_limits.first);
  s_upper = min(s_upper, driving_limits.second);

  // 2. 障碍物约束 → 多个可选 s 区间
  st_obstacles_processor_.GetSBoundsFromDecisions(t, &available_s_bounds, &decisions);

  // 3. 移除不可行选择
  RemoveInvalidDecisions(driving_limits, &available_choices);

  // 4. 按引导线排序选择最优
  double guide_s = st_guide_line_.GetGuideSFromT(t);
  RankDecisions(guide_s, driving_limits, &available_choices);

  // 5. 选择排名第一的决策
  auto top_choice = available_choices.front();
  s_lower = max(s_lower, top_choice.s_lower);
  s_upper = min(s_upper, top_choice.s_upper);

  // 6. 设置障碍物决策 & 更新限速信息
  st_obstacles_processor_.SetObstacleDecision(top_choice.decisions);
  st_driving_limits_.UpdateBlockingInfo(t, s_lower, v_lower, s_upper, v_upper);
}
```

---

## GenerateFallbackSTBound() — 保守兜底

与 Regular 版本类似，但选择最保守的决策（最小 s_lower）：

```cpp
// Always go for the most conservative option
for (auto& choice : available_choices) {
  if (choice.s_lower < top_choice.s_lower)
    top_choice = choice;
}
```

---

## 辅助类

### STObstaclesProcessor

| 方法 | 功能 |
|------|------|
| `MapObstaclesToSTBoundaries` | 将障碍物轨迹映射为 ST 边界 |
| `GetSBoundsFromDecisions` | 给定 t，返回所有可选 s 区间及对应决策 |
| `SetObstacleDecision` | 确认某时间步的障碍物决策 |
| `GetLimitingSpeedInfo` | 获取限速障碍物的速度信息 |

### STGuideLine

- `Init(desired_speed)` — 以期望速度初始化引导线
- `GetGuideSFromT(t)` — 获取 t 时刻的引导 s 值
- `UpdateBlockingInfo(t, s, is_lower)` — 被阻塞时更新引导线

### STDrivingLimits

- `Init(max_acc, max_dec, max_v, init_v)` — 初始化动力学参数
- `GetVehicleDynamicsLimits(t)` — 获取 t 时刻的 s 可达范围
- `UpdateBlockingInfo(...)` — 被阻塞时更新可达范围

---

## 关键常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `kSTBoundsDeciderResolution` | 0.1s | 时间步长 |
| `kSTPassableThreshold` | 3.0m | 可通过间隙阈值 |
| `desired_speed` | 15.0 m/s | 引导线期望速度 |
| `max_acc` | 2.5 m/s² | 最大加速度 |
| `max_dec` | 5.0 m/s² | 最大减速度 |

---

## 与 SpeedBoundsDecider 的区别

| 特性 | SpeedBoundsDecider | STBoundsDecider |
|------|-------------------|-----------------|
| 决策方式 | 独立 ST 边界 + 类型标签 | 扫描线逐步决策 |
| 输出 | `vector<STBoundary*>` | `STDrivableBoundary` |
| 障碍物关系 | 由 SpeedDecider 后续决定 | 在此任务内决定 |
| 启用条件 | 默认 | `FLAGS_use_st_drivable_boundary` |
