---
title: "StGraphOptimizer ST 图速度优化器"
---

# StGraphOptimizer ST 图速度优化器

> 源码位置：`modules/planning/tasks/speed_optimizer/st_graph/`

## 模块定位

StGraphOptimizer 是基于 ST 图动态规划的速度优化器，作为 Lattice Planner 和 PublicRoad Planner 的速度规划组件。它在 ST 图（纵轴为路程 S，横轴为时间 T）上搜索最优速度曲线，生成满足运动学约束和障碍物避让的速度剖面。

继承关系：
```
Task → SpeedOptimizer → StGraphOptimizer
                              ↓ 使用
                        GriddedPathTimeGraph（DP 搜索引擎）
```

## StGraphOptimizer 类

### 类声明

```cpp
class StGraphOptimizer : public SpeedOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
 protected:
  Status Process(const PathData& path_data,
                 const TrajectoryPoint& init_point,
                 SpeedData* const speed_data) override;
  virtual bool SearchPathTimeGraph(SpeedData* speed_data) const;
  void RecordDebugInfo(const SpeedData& speed_data);

  TrajectoryPoint init_point_;
  SLBoundary adc_sl_boundary_;
  DpStSpeedOptimizerConfig config_;
};
```

### Init()

```cpp
bool StGraphOptimizer::Init(...) {
  if (!SpeedOptimizer::Init(config_dir, name, injector)) return false;
  return SpeedOptimizer::LoadConfig<DpStSpeedOptimizerConfig>(&config_);
}
```

- 加载 `DpStSpeedOptimizerConfig` 配置
- `st_graph_optimizer.cc:L38-L48`

### Process()

```cpp
Status StGraphOptimizer::Process(const PathData& path_data,
                                 const TrajectoryPoint& init_point,
                                 SpeedData* const speed_data) {
  init_point_ = init_point;
  if (path_data.discretized_path().empty()) return PLANNING_ERROR;
  if (!SearchPathTimeGraph(speed_data)) return PLANNING_ERROR;
  RecordDebugInfo(*speed_data);
  return Status::OK();
}
```

- 验证路径非空 → 搜索 ST 图 → 记录调试信息
- `st_graph_optimizer.cc:L50-L77`

### SearchPathTimeGraph()

```cpp
bool StGraphOptimizer::SearchPathTimeGraph(SpeedData* speed_data) const {
  const auto& dp_config = reference_line_info_->IsChangeLanePath()
      ? config_.lane_change_speed_config()
      : config_.default_speed_config();

  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(),
      dp_config,
      reference_line_info_->path_decision()->obstacles().Items(),
      init_point_);

  return st_graph.Search(speed_data).ok();
}
```

- 变道时使用 `lane_change_speed_config`，正常行驶使用 `default_speed_config`
- 创建 `GriddedPathTimeGraph` 并执行搜索
- `st_graph_optimizer.cc:L79-L101`

## GriddedPathTimeGraph — DP 搜索引擎

### 网格设计

```
S 轴（混合分辨率）:
  ├── 密集区 [0, dense_length]: dense_unit_s 步长
  └── 稀疏区 [dense_length, S_max]: sparse_unit_s 步长

T 轴（均匀分辨率）:
  └── [0, total_length_t]: unit_t 步长
```

- `cost_table_[t][s]`：二维网格，每个点存储 `(total_cost, pre_point, optimal_speed)`

### Search() — 主搜索流程

```cpp
Status GriddedPathTimeGraph::Search(SpeedData* speed_data) {
  // 1. 初始化成本表
  if (!InitCostTable().ok()) return error;
  // 2. 初始化速度限制查找表
  if (!InitSpeedLimitLookUp().ok()) return error;
  // 3. 计算累计成本（DP 核心）
  if (!CalculateTotalCost().ok()) return error;
  // 4. 提取最优速度曲线
  if (!RetrieveSpeedProfile(speed_data).ok()) return error;
  return Status::OK();
}
```

### InitCostTable()

- 根据配置创建 `dimension_t × dimension_s` 的网格
- 设置每个点的 `(s, t)` 坐标
- 密集区和稀疏区使用不同步长

### CalculateTotalCost() — DP 核心

对每一列 `t`，遍历所有行 `s`，计算从前驱列到当前点的最小成本：

```
cost(t, s) = min over all valid (t-1, s') of:
    cost(t-1, s') + obstacle_cost + speed_cost + accel_cost + jerk_cost
```

成本组成（由 `DpStCost` 计算）：
- **障碍物成本**：与 ST 边界的距离惩罚
- **速度成本**：偏离参考速度的惩罚
- **加速度成本**：加速度大小惩罚
- **Jerk 成本**：加速度变化率惩罚

### RetrieveSpeedProfile()

- 从最后一列回溯 `pre_point` 链，提取最优路径
- 将网格点序列转换为 `SpeedData`（s, t, v, a）

## 配置项

| 字段 | 说明 |
|------|------|
| `total_path_length` | ST 图总路程 |
| `total_time` | ST 图总时间 |
| `unit_t` | 时间步长 |
| `dense_unit_s` | 密集区 S 步长 |
| `sparse_unit_s` | 稀疏区 S 步长 |
| `dense_dimension_s` | 密集区 S 维度 |
| `max_acceleration` | 最大加速度 |
| `max_deceleration` | 最大减速度 |
| `default_speed_config` | 正常行驶速度配置 |
| `lane_change_speed_config` | 变道速度配置 |

## 调用关系

```
PlanOnReferenceLine
  └── ExecuteTaskOnReferenceLine
        └── StGraphOptimizer::Process()
              └── SearchPathTimeGraph()
                    └── GriddedPathTimeGraph::Search()
                          ├── InitCostTable()
                          ├── InitSpeedLimitLookUp()
                          ├── CalculateTotalCost()  ← DP 核心
                          │     └── DpStCost::Calculate()
                          └── RetrieveSpeedProfile()
```
