# PathTimeHeuristicOptimizer DP 速度启发式优化器

> 源码位置：`modules/planning/tasks/path_time_heuristic/`

## 模块定位

PathTimeHeuristicOptimizer 是基于动态规划（DP）的速度优化器，在 ST 图的网格上搜索最优速度曲线。它通常作为 PiecewiseJerkSpeedOptimizer 的前置步骤，提供 DP 粗解作为 QP 优化的初始参考（warm start）。

继承关系：
```
Task → SpeedOptimizer → PathTimeHeuristicOptimizer
                              ↓ 使用
                        GriddedPathTimeGraph（DP 搜索引擎）
                              ↓ 使用
                        DpStCost（代价函数）
```

---

## 核心流程

```cpp
Status PathTimeHeuristicOptimizer::Process(path_data, init_point, speed_data) {
  // 构建 GriddedPathTimeGraph
  GriddedPathTimeGraph st_graph(st_graph_data, dp_config, obstacles, init_point);
  // DP 搜索
  st_graph.Search(speed_data);
}
```

---

## GriddedPathTimeGraph — DP 搜索引擎

### Search() 主流程

```cpp
Status GriddedPathTimeGraph::Search(SpeedData* speed_data) {
  // 1. 检查初始点是否与障碍物碰撞 → 直接返回停车
  if (init_point_in_collision) return stop_profile;

  // 2. 初始化代价表
  InitCostTable();

  // 3. 初始化速度限制查找表
  InitSpeedLimitLookUp();

  // 4. 计算所有节点的总代价（DP 前向传播）
  CalculateTotalCost();

  // 5. 回溯最优路径
  RetrieveSpeedProfile(speed_data);
}
```

### 网格结构

ST 图被离散化为二维网格 `cost_table_[t][s]`：

- **时间轴（列）**：均匀间隔 `unit_t`（默认 1.0s）
- **空间轴（行）**：双分辨率
  - 近端密集：`dense_unit_s`（默认 0.5m）× `dense_dimension_s` 个点
  - 远端稀疏：`sparse_unit_s`（默认 1.0m）

```
s ↑
  │  ·  ·  ·  ·  ·  ·  ·  (sparse)
  │  ·  ·  ·  ·  ·  ·  ·
  │  ·  ·  ·  ·  ·  ·  ·  (dense)
  │  ·  ·  ·  ·  ·  ·  ·
  │  ·  ·  ·  ·  ·  ·  ·
  └──────────────────────→ t
     0  1  2  3  4  5  6  (unit_t)
```

### CalculateTotalCost() — DP 前向传播

```cpp
for (col = 0; col < dimension_t; ++col) {
  for (row in [lowest_row, highest_row]) {
    // 对每个节点 (t, s)，遍历前一列的可达节点
    CalculateCostAt(col, row);
  }
}
```

`CalculateCostAt(c, r)` 对节点 `(c, r)` 计算最小代价：

```cpp
void CalculateCostAt(col, row) {
  // 遍历前一列所有可达的前驱节点
  for (pre_col = col-1; pre_col >= 0; --pre_col) {
    for (pre_row in reachable_range) {
      cost = pre_node.total_cost
           + edge_cost(pre_node → current_node)
           + obstacle_cost(current_node)
           + spatial_potential_cost(current_node);
      if (cost < current_node.total_cost)
        update current_node;
    }
  }
}
```

### GetRowRange() — 可达性约束

根据当前节点的速度和加速度限制，计算下一时间步可达的 s 范围：

```cpp
void GetRowRange(point, &next_highest_row, &next_lowest_row) {
  double v = (point.s - pre_point.s) / unit_t;
  // 最大加速到达的 s
  s_upper = point.s + v * unit_t + 0.5 * max_acc * unit_t²;
  // 最大减速到达的 s
  s_lower = point.s + v * unit_t + 0.5 * max_dec * unit_t²;
}
```

### 碰撞检测

```cpp
bool CheckOverlapOnDpStGraph(boundaries, p1, p2) {
  // 线性插值检查 p1→p2 线段是否与 ST 边界相交
  for (boundary : boundaries) {
    if (boundary->HasOverlap({p1.point(), p2.point()}))
      return true;
  }
}
```

---

## DpStCost — 代价函数

| 代价项 | 方法 | 说明 |
|--------|------|------|
| 障碍物代价 | `GetObstacleCost` | 与 ST 边界的距离惩罚 |
| 空间势能 | `GetSpatialPotentialCost` | 鼓励前进（惩罚停在原地） |
| 参考代价 | `GetReferenceCost` | 与参考速度曲线的偏差 |
| 速度代价 | `GetSpeedCost` | 偏离限速/巡航速度的惩罚 |
| 加速度代价 | `GetAccelCost` | 加速度平方惩罚 |
| Jerk 代价 | `GetJerkCost` | 加加速度平方惩罚（舒适性） |

### 边代价计算

```cpp
double CalculateEdgeCost(first, second, third, fourth, speed_limit, cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, fourth, speed_limit, cruise_speed)
       + dp_st_cost_.GetAccelCostByThreePoints(second, third, fourth)
       + dp_st_cost_.GetJerkCostByFourPoints(first, second, third, fourth);
}
```

---

## StGraphPoint — 网格节点

```cpp
class StGraphPoint {
  STPoint point_;           // (s, t) 坐标
  const StGraphPoint* pre_point_;  // 最优前驱
  double total_cost_;       // 累计最小代价
  uint32_t index_t_;        // 列索引
  uint32_t index_s_;        // 行索引
};
```

---

## 与 PiecewiseJerkSpeed 的协作

在 PublicRoad Planner 的典型 pipeline 中：

```
SpeedBoundsDecider → PathTimeHeuristicOptimizer → PiecewiseJerkSpeedOptimizer
                          (DP 粗解)                    (QP 精细优化)
```

1. `PathTimeHeuristicOptimizer` 生成 DP 速度曲线
2. DP 结果存入 `SpeedData`
3. `PiecewiseJerkSpeedOptimizer` 以 DP 结果为参考，进行 QP 优化

---

## 关键配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `unit_t` | 1.0s | 时间步长 |
| `dense_unit_s` | 0.5m | 近端空间分辨率 |
| `sparse_unit_s` | 1.0m | 远端空间分辨率 |
| `dense_dimension_s` | 101 | 密集区节点数 |
| `max_acceleration` | 3.0 m/s² | DP 最大加速度 |
| `max_deceleration` | -4.0 m/s² | DP 最大减速度 |
| `obstacle_weight` | 1.0 | 障碍物代价权重 |
| `speed_weight` | 1.0 | 速度代价权重 |
| `accel_weight` | 10.0 | 加速度代价权重 |
| `jerk_weight` | 10.0 | Jerk 代价权重 |
