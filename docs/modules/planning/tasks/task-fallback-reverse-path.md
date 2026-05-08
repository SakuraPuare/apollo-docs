---
title: "FallbackPath & ReversePath 特殊路径任务"
---

# FallbackPath & ReversePath 特殊路径任务

> 源码位置：`modules/planning/tasks/fallback_path/`、`modules/planning/tasks/reverse_path/`

## 模块定位

这两个任务是特殊用途的路径生成器，均继承自 `PathGeneration` 基类：
- **FallbackPath**：当主路径（LaneFollowPath 等）生成失败时的兜底路径
- **ReversePath**：倒车路径生成（负 delta_s 方向规划）

---

## 一、FallbackPath — 兜底路径

> `fallback_path.cc`

### 类声明

```cpp
class FallbackPath : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
 private:
  Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;
  bool DecidePathBounds(std::vector<PathBoundary>* boundary);
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);
  bool AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path);
  FallbackPathConfig config_;
};
```

### Process()

```cpp
Status FallbackPath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 若已有路径数据或是变道路径，跳过
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->IsChangeLanePath()) {
    return Status::OK();
  }
  GetStartPointSLState();
  DecidePathBounds(&candidate_path_boundaries);
  OptimizePath(candidate_path_boundaries, &candidate_path_data);
  AssessPath(&candidate_path_data, reference_line_info->mutable_path_data());
  return Status::OK();
}
```

- 仅在主路径为空且非变道时触发
- 流程：边界决策 → 路径优化 → 路径评估
- `fallback_path.cc:L43-L65`

### DecidePathBounds()

```cpp
bool FallbackPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  // 1. 初始化无限大边界
  PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_);
  // 2. 基于自车道收缩边界
  PathBoundsDeciderUtil::GetBoundaryFromSelfLane(...);
  // 3. 基于自车位置扩展边界（extend_buffer）
  PathBoundsDeciderUtil::ExtendBoundaryByADC(..., config_.extend_buffer(), ...);
  path_bound.set_label("fallback/self");
}
```

- 只使用自车道边界，不考虑借道
- `extend_buffer` 确保自车当前位置在边界内
- `fallback_path.cc:L67-L92`

### OptimizePath()

```cpp
bool FallbackPath::OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                                 std::vector<PathData>* candidate_path_data) {
  // 目标：l = 0（回到车道中心）
  std::vector<double> ref_l(path_boundary_size, 0);
  // 使用 PiecewiseJerk 优化
  PathOptimizerUtil::OptimizePath(init_sl_state_, end_state, ref_l, ...);
  // 转换为 FrenetPath
  PathOptimizerUtil::ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, ...);
}
```

- 参考线为 `l = 0`（车道中心）
- 使用与 LaneFollowPath 相同的 PiecewiseJerk 优化器
- `fallback_path.cc:L94-L140`

### AssessPath()

```cpp
bool FallbackPath::AssessPath(std::vector<PathData>* candidate_path_data,
                               PathData* final_path) {
  // 检查是否严重偏离参考线
  PathAssessmentDeciderUtil::IsGreatlyOffReferenceLine(curr_path_data);
  // 检查是否严重偏离道路
  PathAssessmentDeciderUtil::IsGreatlyOffRoad(*reference_line_info_, curr_path_data);
}
```

- 简化版评估：只检查是否偏离过大，不做碰撞检测
- `fallback_path.cc:L142-L169`

---

## 二、ReversePath — 倒车路径

> `reverse_path.cc`

### 类声明

```cpp
class ReversePath : public PathGeneration {
 public:
  bool Init(...) override;
 private:
  Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;
  bool DecidePathBounds(PathBoundary* path_boundary, double backward_length);
  bool OptimizePathOsqp(const PathBoundary& path_boundary, PathData* path_data);
  bool InitPathBoundary(PathBoundary* path_bound, SLState init_sl_state,
                        double& reference_line_backward_length);
  bool GetBoundaryFromRoad(PathBoundary* path_bound);
  ReversePathConfig config_;
};
```

### Process()

```cpp
Status ReversePath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
  GetStartPointSLState();
  // 可选：检查是否在广场路口内
  if (config_.is_considered_square_boundary()
      && reference_line_info->GetJunction(init_sl_state_.first[0], &junction_overlap_) == 0) {
    return Status::OK();
  }
  double backward_length = init_sl_state_.first[0] - reference_line_backward_s - kEpsilon;
  DecidePathBounds(&path_boundary, backward_length);
  OptimizePathOsqp(path_boundary, &path_data);
  *reference_line_info->mutable_path_data() = path_data;
}
```

- 计算可用的后向长度（参考线起点到当前位置）
- `reverse_path.cc:L55-L80`

### InitPathBoundary()

```cpp
bool ReversePath::InitPathBoundary(PathBoundary* path_bound, ...) {
  path_bound->set_delta_s(-0.1);  // 负步长 = 倒车方向
  double end_s = init_s - min(max_s_distance, backward_length);
  for (double curr_s = init_s; curr_s > end_s; curr_s += delta_s) {
    path_bound->emplace_back(curr_s, -max_lateral_distance, max_lateral_distance);
  }
}
```

- **关键特征**：`delta_s = -0.1`（负值表示倒车方向）
- 最大倒车距离由 `max_s_distance` 配置
- `reverse_path.cc:L252-L271`

### GetBoundaryFromRoad()

- 使用 `SLPolygon` 获取道路边界
- 将边界限制在 `[-max_lateral_distance, max_lateral_distance]` 内
- `reverse_path.cc:L225-L250`

### OptimizePathOsqp()

- 使用 OSQP 求解器优化倒车路径
- 目标：最小化横向偏移和曲率变化
- 结果转换为 `DiscretizedPath`（需反转点序列）
- `reverse_path.cc:L82-L170`（推断自 Process 调用链）

---

## 调用关系

| 任务 | 触发条件 | 规划方向 | 优化器 |
|------|----------|----------|--------|
| FallbackPath | 主路径为空且非变道 | 前向 | PiecewiseJerk |
| ReversePath | 倒车场景 | 后向（delta_s < 0） | OSQP |

- **上游**：Stage 的 Task 流水线调用
- **依赖**：`PathBoundsDeciderUtil`、`PathOptimizerUtil`、`PathAssessmentDeciderUtil`
