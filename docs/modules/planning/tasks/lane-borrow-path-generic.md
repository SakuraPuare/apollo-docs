---
title: "LaneBorrowPathGeneric 借道路径规划"
---

# LaneBorrowPathGeneric 借道路径规划

> 源码路径：`modules/planning/tasks/lane_borrow_path_generic/`

## 概述

`LaneBorrowPathGeneric` 是规划模块的借道路径生成任务，当自车道被障碍物长时间阻挡时，自动借用相邻车道（左侧或右侧）生成绕行路径。该任务继承 `PathGeneration`，遵循"决策边界 → 优化路径 → 评估选择"三段式流程。支持正向车道和逆向车道借用，并集成 Nudge（微调避障）决策。

## 核心类

### LaneBorrowPathGeneric

```cpp
class LaneBorrowPathGeneric : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool DecidePathBounds(std::vector<PathBoundary>* boundary);
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);
  bool AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path);

  bool GetBoundaryFromNeighborLane(SidePassDirection pass_direction,
                                    PathBoundary* path_bound,
                                    std::string* borrow_lane_type);
  bool GetBoundaryFromNudgeDecision(SidePassDirection pass_direction,
                                     PathBoundary* path_bound);
  bool IsNecessaryToBorrowLane();
  void CheckLaneBorrow(const ReferenceLineInfo& info,
                       bool* left_borrowable, bool* right_borrowable);
  bool CheckLaneBoundaryType(const ReferenceLineInfo& info, double check_s,
                              const SidePassDirection& direction);
  void SetPathInfo(PathData* path_data);
  void GetSLPolygons(std::vector<SLPolygon>* polygons, LaneBorrowInfo info);

  LaneBorrowPathGenericConfig config_;
  std::vector<SidePassDirection> decided_side_pass_direction_;
  int use_self_lane_;
  std::string blocking_obstacle_id_;
  bool is_in_path_lane_borrow_scenario_ = false;
  std::unordered_map<std::string, double> narrowest_width_;
};
```

**源码**：`modules/planning/tasks/lane_borrow_path_generic/lane_borrow_path_generic.h`

### SidePassDirection

```cpp
enum SidePassDirection { LEFT_BORROW = 1, RIGHT_BORROW = 2 };
```

## 核心流程

### Process() — 主入口

```cpp
Status LaneBorrowPathGeneric::Process(Frame* frame,
                                       ReferenceLineInfo* reference_line_info) {
  if (!config_.is_allow_lane_borrowing() || reference_line_info->path_reusable()) {
    return Status::OK();
  }
  if (!IsNecessaryToBorrowLane()) {
    // 清除借道状态
    return Status::OK();
  }

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) { return Status::OK(); }
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) { return Status::OK(); }
  if (!AssessPath(&candidate_path_data, final_path)) { ... }
  return Status::OK();
}
```

**关键步骤**：

1. 检查是否允许借道且路径不可复用
2. `IsNecessaryToBorrowLane()` 判断是否需要借道
3. `DecidePathBounds()` 生成候选路径边界
4. `OptimizePath()` 对每个边界优化路径
5. `AssessPath()` 评估并选择最优路径

### IsNecessaryToBorrowLane() — 借道决策

**职责**：判断当前是否需要借用相邻车道

**已处于借道状态时**的退出条件：

- `use_self_lane_ >= 6`（自车道已可使用 6 个周期以上）
- 上一帧不在借道场景（如刚从换道回来）
- 参考线数量变化

**未处于借道状态时**的进入条件：

- 非换道路径
- 车速在 `lane_borrow_max_speed` 以下
- 阻挡障碍物远离路口
- 阻挡障碍物在目的地范围内
- 存在可借用的相邻车道（非实线边界）

### DecidePathBounds() — 路径边界决策

对每个决定的借道方向（左/右）：

1. `InitPathBoundary()` 初始化无限大边界
2. `GetBoundaryFromNeighborLane()` 根据相邻车道收缩边界
3. `ExtendBoundaryByADC()` 根据自车位置扩展边界（可选）
4. `UpdatePathBoundaryBySLPolygon()` 根据障碍物 SL 多边形收缩边界
5. `GetBoundaryFromNudgeDecision()` 根据 Nudge 信息调整边界
6. `AddExtraPathBound()` 添加额外边界点

```cpp
bool LaneBorrowPathGeneric::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  for (size_t i = 0; i < decided_side_pass_direction_.size(); i++) {
    PathBoundsDeciderUtil::InitPathBoundary(...);
    GetBoundaryFromNeighborLane(decided_side_pass_direction_[i], &path_bound, &borrow_lane_type);
    PathBoundsDeciderUtil::UpdatePathBoundaryBySLPolygon(...);
    GetBoundaryFromNudgeDecision(decided_side_pass_direction_[i], &path_bound);
    PathBoundsDeciderUtil::AddExtraPathBound(...);
    // 更新借道状态中的阻塞时长
  }
}
```

### OptimizePath() — 路径优化

对每个边界使用 Piecewise Jerk 优化器生成平滑路径。

```cpp
bool LaneBorrowPathGeneric::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  for (const auto& path_boundary : path_boundaries) {
    PathOptimizerUtil::OptimizePath(init_sl_state, end_state, ref_l,
                                    weight_ref_l, path_boundary, ...);
    // 转换为 PiecewiseJerkPath 并存入 candidate_path_data
  }
}
```

### AssessPath() — 路径评估与选择

过滤无效路径，使用 `ComparePathData()` 比较候选路径，选择最优路径写入 `final_path`。

**比较策略**（`ComparePathData()` 辅助函数）：

1. **路径长度**：选更长的（容差 25m）
2. **逆向车道占比**：逆向点数少的优先
3. **障碍物位置**：障碍物偏左选右借，偏右选左借
4. **回自车道时间**：越早回自车道越好（容差 20m）
5. **最窄宽度**：最窄处 < 2m 且差异 > 1.5m 时选更宽的
6. **默认**：优先左侧借道

### GetBoundaryFromNeighborLane() — 邻车道边界

遍历路径上的每个 s 点，获取当前车道宽度和相邻车道宽度，组合计算左右边界。支持正向和逆向相邻车道。

```cpp
bool LaneBorrowPathGeneric::GetBoundaryFromNeighborLane(
    const SidePassDirection pass_direction,
    PathBoundary* const path_bound, std::string* borrow_lane_type) {
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 获取当前车道宽度
    reference_line.GetLaneWidth(curr_s, &curr_lane_left_width, &curr_lane_right_width);
    // 获取相邻车道宽度
    reference_line_info_->GetNeighborLaneInfo(..., &curr_neighbor_lane_width);
    // 组合计算边界
    curr_left_bound = curr_lane_left_width + neighbor_width - offset_to_map;
    curr_right_bound = -(curr_lane_right_width + neighbor_width) - offset_to_map;
  }
  *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
}
```

### CheckLaneBorrow() — 车道可借用性检查

向前检查 20m 范围内的车道边界类型。实线（SOLID_YELLOW、SOLID_WHITE、DOUBLE_YELLOW）禁止借用。

## 配置

通过 `LaneBorrowPathGenericConfig` protobuf 加载：

| 字段 | 说明 |
|------|------|
| `is_allow_lane_borrowing` | 是否允许借道 |
| `lane_borrow_max_speed` | 借道最大车速 |
| `enable_extend_boundary_by_adc` | 是否根据自车扩展边界 |
| `enable_ignore_boundary_type` | 是否忽略车道线类型限制 |
| `enable_active_trigger` | 是否主动触发借道 |
| `enable_nudge_destination_threshold` | Nudge 目的地距离阈值 |
| `long_term_blocking_obstacle_cycle_threshold` | 长期阻塞判定周期数 |
| `path_optimizer_config` | 路径优化器配置 |

## 辅助函数

| 函数 | 说明 |
|------|------|
| `ComparePathData()` | 比较两条候选路径，返回较优者 |
| `ContainsOutOnReverseLane()` | 统计路径中逆向车道点的数量 |
| `GetBackToInLaneIndex()` | 获取路径中最后回到自车道的点索引 |

## 调用关系

- **父类**：`PathGeneration`（路径生成任务基类）
- **依赖**：`PathBoundsDeciderUtil`（边界工具）、`PathOptimizerUtil`（优化工具）、`PathAssessmentDeciderUtil`（评估工具）、`ParkDataCenter`（Nudge 信息）
- **被调用**：规划器通过插件机制作为 Task 调用
