# LaneFollowPath 车道跟随路径

> 源码位置：`modules/planning/tasks/lane_follow_path/`

## 模块定位

LaneFollowPath 是最核心的路径生成任务，负责在当前车道内生成平滑的行驶路径。它继承 PathGeneration 基类，实现三步流水线：边界决策 → 路径优化 → 路径评估。

---

## 类声明

```cpp
class LaneFollowPath : public PathGeneration {
 public:
  bool Init(const string& config_dir, const string& name,
            const shared_ptr<DependencyInjector>&) override;
 private:
  Status Process(Frame*, ReferenceLineInfo*) override;
  bool DecidePathBounds(vector<PathBoundary>* boundary);
  bool OptimizePath(const vector<PathBoundary>&, vector<PathData>*);
  bool AssessPath(vector<PathData>*, PathData* final_path);
  LaneFollowPathConfig config_;
};
```

---

## Process() — 主流程

```cpp
Status LaneFollowPath::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 跳过条件：路径已存在或可复用
  if (!rli->path_data().Empty() || rli->path_reusable()) return OK;

  GetStartPointSLState();                    // 计算起点 SL 坐标
  DecidePathBounds(&candidate_boundaries);   // Step 1: 边界
  OptimizePath(candidate_boundaries, &data); // Step 2: 优化
  AssessPath(&data, rli->mutable_path_data()); // Step 3: 评估
}
```

- `lane_follow_path.cc:L47-L74`

---

## DecidePathBounds() — 路径边界决策

```cpp
bool LaneFollowPath::DecidePathBounds(vector<PathBoundary>* boundary) {
  // 1. 初始化为无穷大边界
  PathBoundsDeciderUtil::InitPathBoundary(*rli, &path_bound, init_sl_state_);

  // 2. 根据自车道信息收缩边界
  PathBoundsDeciderUtil::GetBoundaryFromSelfLane(*rli, init_sl_state_, &path_bound);

  // 2.5 可选：扩展边界以包含 ADC 当前位置
  if (is_include_adc)
    PathBoundsDeciderUtil::ExtendBoundaryByADC(..., &path_bound);

  // 3. 根据静态障碍物进一步收缩边界
  PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
      *rli, &obs_sl_polygons, init_sl_state_, &path_bound,
      &blocking_obstacle_id, &path_narrowest_width);

  // 4. 追加额外尾部点（避免零长度路径）
  while (blocking && path_bound.size() < temp.size() && counter < N)
    path_bound.push_back(temp[path_bound.size()]);

  // 5. 更新 lane_follow_status（阻塞时长统计）
  lane_follow_status->set_block_obstacle_id(blocking_obstacle_id);
  lane_follow_status->set_block_duration(...);
}
```

- 边界格式：`PathBound = vector<PathBoundPoint>`，每个点含 `(s, l_lower, l_upper)`
- 阻塞检测：如果静态障碍物完全堵住通道，记录 `blocking_obstacle_id`
- `lane_follow_path.cc:L76-L181`

---

## OptimizePath() — 路径优化（Piecewise Jerk）

```cpp
bool LaneFollowPath::OptimizePath(const vector<PathBoundary>& boundaries,
                                   vector<PathData>* candidate_path_data) {
  for (const auto& path_boundary : boundaries) {
    // 1. 计算曲率约束（ddl bounds）
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);

    // 2. 估算 jerk 上界
    double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(speed);

    // 3. 计算参考线偏移（ref_l）
    PathOptimizerUtil::UpdatePathRefWithBound(boundary, weight, &ref_l, &weight_ref_l);

    // 4. 调用 QP 优化器求解
    bool res = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l,
        path_boundary, ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);

    // 5. 转换为 FrenetFramePath
    auto frenet_path = PathOptimizerUtil::ToPiecewiseJerkPath(
        opt_l, opt_dl, opt_ddl, delta_s, start_s);

    // 6. 可选：前轴中心 → 后轴中心坐标转换
    if (FLAGS_use_front_axe_center_in_path_planning)
      PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(path_data);
  }
}
```

- 优化目标：最小化 l 偏移 + dl 变化 + ddl 变化 + jerk
- 约束：路径边界、曲率限制、起终点状态
- `lane_follow_path.cc:L183-L245`

---

## AssessPath() — 路径评估

```cpp
bool LaneFollowPath::AssessPath(vector<PathData>* candidates, PathData* final) {
  PathData& curr = candidates->back();
  RecordDebugInfo(curr, curr.path_label(), rli);

  // 1. 验证路径合法性（碰撞检测、曲率检查等）
  if (!PathAssessmentDeciderUtil::IsValidRegularPath(*rli, curr))
    return false;

  // 2. 初始化路径点决策（IN_LANE 标记）
  PathAssessmentDeciderUtil::InitPathPointDecision(
      curr, PathPointType::IN_LANE, &path_decision);
  curr.SetPathPointDecisionGuide(path_decision);

  // 3. 设置最终路径
  *final = curr;
  rli->MutableCandidatePathData()->push_back(*final);
  rli->SetBlockingObstacle(curr.blocking_obstacle_id());
}
```

- `lane_follow_path.cc:L247-L273`

---

## 配置项

| 参数 | 说明 |
|------|------|
| `is_extend_lane_bounds_to_include_adc` | 是否扩展边界包含当前车辆位置 |
| `extend_buffer` | 扩展缓冲距离 |
| `path_optimizer_config` | QP 优化器参数（权重、约束） |
| `path_reference_l_weight` | 参考线偏移权重 |
