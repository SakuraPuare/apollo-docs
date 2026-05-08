---
title: "SpeedBoundsDecider 速度边界决策器"
---

# SpeedBoundsDecider 速度边界决策器

> 源码位置：`modules/planning/tasks/speed_bounds_decider/`

## 模块定位

SpeedBoundsDecider 是速度规划的前置任务，负责：
1. 将障碍物映射到 ST 图上（STBoundaryMapper）
2. 计算沿路径的速度限制（SpeedLimitDecider）
3. 组装 StGraphData 供下游速度优化器使用

---

## 类结构

```
SpeedBoundsDecider (Decider)
├── STBoundaryMapper     — 障碍物 → ST 边界映射
└── SpeedLimitDecider    — 路径速度限制计算
```

---

## SpeedBoundsDecider::Process()

```cpp
Status SpeedBoundsDecider::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 障碍物映射到 ST 图
  STBoundaryMapper boundary_mapper(config_, reference_line, path_data,
                                   path_length, total_time, injector_);
  boundary_mapper.ComputeSTBoundary(path_decision);

  // 收集所有 ST 边界
  for (auto* obstacle : path_decision->obstacles().Items()) {
    if (!obstacle->path_st_boundary().IsEmpty()) {
      if (type == KEEP_CLEAR) obstacle->SetBlockingObstacle(false);
      else                    obstacle->SetBlockingObstacle(true);
      boundaries.push_back(&st_boundary);
    }
  }

  // 2. 计算速度限制
  SpeedLimitDecider speed_limit_decider(config_, reference_line, path_data);
  speed_limit_decider.GetSpeedLimits(obstacles, &speed_limit);

  // 3. 组装 StGraphData
  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, cruise_speed, path_length, total_time,
                          st_graph_debug);
}
```

- `speed_bounds_decider.cc:L53-L128`

---

## STBoundaryMapper — 障碍物 ST 边界映射

### ComputeSTBoundary()

对每个障碍物调用映射：

```cpp
Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision) const {
  for (auto* obstacle : path_decision->obstacles().Items()) {
    // 有纵向决策的障碍物：按决策类型设置边界
    if (has_longitudinal_decision)
      ComputeSTBoundaryWithDecision(obstacle, decision);
    else
      ComputeSTBoundary(obstacle);  // 无决策：纯几何映射
  }
}
```

### GetOverlapBoundaryPoints()

核心几何计算：将障碍物轨迹投影到 ST 图上

```cpp
bool GetOverlapBoundaryPoints(
    const vector<PathPoint>& path_points, const Obstacle& obstacle,
    vector<STPoint>* upper_points, vector<STPoint>* lower_points) const;
```

- 遍历障碍物预测轨迹的每个时间步
- 对每个时间步，检查 ADC 路径上哪些点与障碍物重叠
- 重叠区间的 s 范围构成该时刻的 ST 边界上下界

### CheckOverlap()

```cpp
bool CheckOverlap(const PathPoint& path_point,
                  const Box2d& obs_box, double l_buffer) const;
```

- 在给定路径点处构建 ADC 包围盒
- 检查是否与障碍物包围盒相交
- `l_buffer` 提供额外横向安全余量

### MapStopDecision()

```cpp
bool MapStopDecision(Obstacle* stop_obstacle,
                     const ObjectDecisionType& decision) const;
```

- 将 STOP 决策映射为 ST 图上的垂直边界（所有时间 t 上 s 不超过停车点）

---

## SpeedLimitDecider — 速度限制计算

```cpp
Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<string, Obstacle>& obstacles,
    SpeedLimit* speed_limit) const;
```

速度限制来源：
1. **地图限速**：`reference_line.GetSpeedLimitFromS(s)`
2. **曲率限速**：`GetCentricAccLimit(kappa)` → `v = sqrt(a_centric / |kappa|)`
3. **nudge 减速**：靠近横向障碍物时降速

### GetCentricAccLimit()

```cpp
double GetCentricAccLimit(double kappa) const {
  // 向心加速度限制 → 最大允许速度
  // v_max = sqrt(a_lateral_max / |kappa|)
}
```

---

## SetSpeedFallbackDistance()

```cpp
double SetSpeedFallbackDistance(PathDecision* path_decision) {
  // 找到最近的阻塞障碍物 s 位置
  // 用于速度规划失败时的兜底停车距离
}
```

---

## 数据流

```
SpeedBoundsDecider
  ├─ STBoundaryMapper::ComputeSTBoundary()
  │    → obstacle->set_path_st_boundary(...)
  ├─ SpeedLimitDecider::GetSpeedLimits()
  │    → SpeedLimit (s → v_max 映射)
  └─ StGraphData::LoadData(boundaries, speed_limit, ...)
       → 供 PiecewiseJerkSpeedOptimizer 使用
```
