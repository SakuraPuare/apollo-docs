# PullOverPath 靠边停车路径

> 源码位置：`modules/planning/tasks/pull_over_path/`

## 模块定位

PullOverPath 是靠边停车场景的路径生成任务。当 Scenario 决定执行靠边停车时，本任务负责搜索合适的停车位置并生成从当前位置到停车点的平滑路径。

继承关系：
```
Task → PathGeneration → PullOverPath
```

---

## Process 主流程

```cpp
Status PullOverPath::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 跳过条件：已有路径 或 换道参考线
  if (!rli->path_data().Empty() || rli->IsChangeLanePath()) return OK;

  GetStartPointSLState();           // 获取起点 SL 状态
  DecidePathBounds(&boundaries);    // 计算路径边界
  OptimizePath(boundaries, &candidates);  // QP 优化路径
  AssessPath(&candidates, final_path);    // 评估选择最优路径
}
```

---

## DecidePathBounds — 路径边界决策

```cpp
bool PullOverPath::DecidePathBounds(vector<PathBoundary>* boundary) {
  // 1. 检查 PlanningContext 中是否标记了 plan_pull_over_path
  if (!plan_pull_over_path) return false;

  // 2. 初始化路径边界为道路宽度
  PathBoundsDeciderUtil::InitPathBoundary(rli, &path_bound, init_sl_state_);
  GetBoundaryFromRoads(rli, &path_bound);

  // 3. 确定靠边方向（左/右）
  if (config == BOTH_SIDE)
    is_pull_over_right = (adc_to_left > adc_to_right);
  else if (config == LEFT_SIDE)
    is_pull_over_right = false;
  else
    is_pull_over_right = true;

  // 4. 用车道边界更新靠边侧的路径边界
  UpdatePullOverBoundaryByLaneBoundary(is_pull_over_right, &path_bound);

  // 5. 考虑静态障碍物收缩边界
  PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(...);

  // 6. 搜索或验证停车位置
  if (pull_over_status.has_position()) {
    // 验证已有位置是否仍在边界内
    curr_idx = IsPointWithinPathBound(x, y, path_bound);
  }
  if (curr_idx < 0) {
    // 搜索新的停车位置
    SearchPullOverPosition(path_bound, &pull_over_configuration);
    // 更新 PlanningContext
    pull_over_status.set_position(x, y);
    pull_over_status.set_theta(theta);
  }

  // 7. 裁剪路径边界到停车点之后
  TrimPathBound(curr_idx + extra_tail_points);
}
```

---

## SearchPullOverPosition — 停车位搜索

搜索策略由配置决定：

| 模式 | 搜索方向 | 目标 s |
|------|----------|--------|
| `NEAREST_POSITION` | 前向搜索 | `adc_s + 2 * min_turn_radius * factor` |
| `DESTINATION` | 后向搜索 | 路由终点 s |

```cpp
bool PullOverPath::SearchPullOverPosition(path_bound, &config) {
  // 确定搜索起点 pull_over_s
  if (NEAREST_POSITION) FindNearestPullOverS(&pull_over_s);
  else if (DESTINATION) FindDestinationPullOverS(path_bound, &pull_over_s);

  // 滑动窗口搜索可行停车区域
  while (idx in valid_range) {
    // 跳过交叉口区域
    if (point_in_junction) continue;

    // 检查窗口内所有点是否满足：
    //   - 靠近道路边缘（距边缘 < pull_over_space_width）
    //   - 窗口长度 >= 1.5 * vehicle_length
    //   - 不在交叉口内
    if (feasible_window_found) {
      // 取窗口中心作为停车点
      pull_over_x, pull_over_y = SLToXY(center_s, center_l);
      pull_over_theta = reference_line.GetHeading(center_s);
      return true;
    }
  }
}
```

### FindNearestPullOverS

```cpp
bool PullOverPath::FindNearestPullOverS(double* pull_over_s) {
  double pull_over_distance = min_turn_radius * 2 * adjust_factor;
  *pull_over_s = adc_end_s + pull_over_distance;
}
```

基于车辆最小转弯半径计算最近可停车距离。

### FindDestinationPullOverS

```cpp
bool PullOverPath::FindDestinationPullOverS(path_bound, double* pull_over_s) {
  // 将路由终点投影到参考线获取 destination_s
  reference_line.XYToSL(routing_end.pose(), &destination_sl);
  // 检查距离 ADC 足够远
  if (destination_s - adc_end_s < buffer) return false;
  // 检查在路径边界范围内
  if (destination_s + buffer >= path_bound.back().s) return false;
  *pull_over_s = destination_s;
}
```

---

## OptimizePath — QP 路径优化

```cpp
bool PullOverPath::OptimizePath(boundaries, &candidate_path_data) {
  for (each path_boundary) {
    // 终点状态：停车位置的横向偏移
    end_state = {pull_over_sl.l(), 0.0, 0.0};

    // 参考线设为停车位置的 l 值，权重使用 pull_over_weight
    ref_l.assign(size, pull_over_sl.l());
    weight_ref_l.assign(size, config_.pull_over_weight());

    // 调用 PiecewiseJerk 路径优化
    PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l,
        path_boundary, ddl_bounds, jerk_bound, config, ...);
  }
}
```

---

## AssessPath — 路径评估

```cpp
bool PullOverPath::AssessPath(candidates, &final_path) {
  // 验证路径合法性（无碰撞、曲率合理等）
  PathAssessmentDeciderUtil::IsValidRegularPath(rli, curr_path);

  // 标记路径点决策类型为 IN_LANE
  PathAssessmentDeciderUtil::InitPathPointDecision(
      curr_path, IN_LANE, &path_decision);

  // 输出 debug 信息（停车位置、朝向、车辆尺寸）
  pull_over_debug->set_position(x, y);
  pull_over_debug->set_theta(theta);
}
```

---

## UpdatePullOverBoundaryByLaneBoundary

根据靠边方向，用车道边界替换路径边界的一侧：

```cpp
void PullOverPath::UpdatePullOverBoundaryByLaneBoundary(
    bool is_pull_over_right, PathBoundary* path_bound) {
  for (each point in path_bound) {
    GetLaneWidth(curr_s, &left_width, &right_width);
    if (is_pull_over_right)
      path_bound[i].l_upper = left_width;   // 左侧用车道边界
    else
      path_bound[i].l_lower = right_width;  // 右侧用车道边界
  }
}
```

- 靠右停车：左边界收紧到车道线（不允许跨车道）
- 靠左停车：右边界收紧到车道线

---

## 关键配置参数

| 参数 | 说明 |
|------|------|
| `pull_over_direction` | 停车方向：RIGHT_SIDE / LEFT_SIDE / BOTH_SIDE |
| `pull_over_position` | 停车位置策略：NEAREST_POSITION / DESTINATION |
| `pull_over_weight` | QP 优化中停车位置参考权重 |
| `pull_over_approach_lon_distance_adjust_factor` | 最近停车距离调整因子 |
| `pull_over_destination_to_adc_buffer` | 目的地到 ADC 最小距离 |
| `pull_over_destination_to_pathend_buffer` | 目的地到路径末端缓冲 |
