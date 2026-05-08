---
title: "PathReferenceDecider 路径参考决策器"
---

# PathReferenceDecider 路径参考决策器

> 源码位置：`modules/planning/tasks/path_reference_decider/`

## 模块定位

PathReferenceDecider 是混合规划模式（Hybrid Model）的桥梁任务。它将学习模型（Learning-based Model）输出的轨迹作为路径参考，验证其是否在规则模型计算的路径边界内。如果有效，则将学习模型输出作为路径优化的参考线；否则回退到纯规则模型。

继承关系：
```
Task → PathReferenceDecider
```

---

## Process 主流程

```cpp
Status PathReferenceDecider::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 跳过条件
  if (skip_in_change_lane && multi_reference_lines) return OK;
  if (skip_in_side_pass && is_lane_borrow) return OK;

  // 2. 获取路径边界
  const auto& path_boundaries = rli->GetCandidatePathBoundaries();
  size_t regular_idx = GetRegularPathBound(path_boundaries);

  // 3. 获取学习模型输出
  const auto& learning_trajectory =
      injector_->learning_based_data()->learning_data_adc_future_trajectory_points();
  if (learning_trajectory.empty()) {
    set_is_valid_path_reference(false);
    return OK;
  }

  // 4. 拼接学习轨迹（对齐当前规划起点）
  rli->AdjustTrajectoryWhichStartsFromCurrentPos(
      start_point, learning_trajectory, &stitched_trajectory);

  // 5. 转换为路径点
  ConvertTrajectoryToPath(stitched_trajectory, &path_reference);

  // 6. 验证路径参考是否在边界内
  if (!IsValidPathReference(rli, path_boundaries[regular_idx], path_reference)) {
    set_is_valid_path_reference(false);
    return OK;
  }

  // 7. 重采样到与路径边界相同的 s 间隔
  EvaluatePathReference(path_boundaries[regular_idx], path_reference,
                        &evaluated_path_reference);

  // 8. 标记为有效路径参考
  ++valid_path_reference_counter_;
  rli->mutable_path_data()->set_is_valid_path_reference(true);
  rli->mutable_path_data()->set_path_reference(evaluated_path_reference);
}
```

---

## IsValidPathReference — 路径参考验证

```cpp
bool PathReferenceDecider::IsValidPathReference(
    rli, path_bound, path_reference) {
  // 逐点检查路径参考是否在路径边界内
  for (path_point : path_reference) {
    int idx = IsPointWithinPathBounds(rli, path_bound, x, y);
    if (idx == -1) return false;  // 点在边界外
  }
  return true;
}
```

### IsPointWithinPathBounds

```cpp
int PathReferenceDecider::IsPointWithinPathBounds(
    rli, path_bound, x, y) {
  // 将 (x, y) 投影到参考线获取 (s, l)
  reference_line.XYToSL({x, y}, &sl_point);

  // 找到对应的 s 索引
  int idx = (sl_point.s() - path_bound.start_s()) / path_bound.delta_s();

  // 检查 l 是否在 [l_lower, l_upper] 范围内
  if (sl_point.l() < path_bound[idx].l_lower ||
      sl_point.l() > path_bound[idx].l_upper)
    return -1;

  return idx;
}
```

---

## IsADCBoxAlongPathReferenceWithinPathBounds

更精确的验证方法——检查车辆包围盒而非单点：

```cpp
bool IsADCBoxAlongPathReferenceWithinPathBounds(
    path_reference, regular_path_bound) {
  // 将路径边界转换为线段
  PathBoundToLineSegments(regular_path_bound, &path_bound_segments);

  // 沿路径参考逐点构建车辆包围盒
  for (point : path_reference) {
    Box2d adc_box(center, heading, length, width);
    // 检查包围盒是否与边界线段相交
    for (segment : path_bound_segments) {
      if (adc_box.HasOverlap(segment)) return false;
    }
  }
  return true;
}
```

---

## GetRegularPathBound

从候选路径边界中找到常规（非 fallback）边界：

```cpp
size_t PathReferenceDecider::GetRegularPathBound(path_bounds) {
  for (i = 0; i < path_bounds.size(); ++i) {
    if (path_bounds[i].label().find("regular") != string::npos)
      return i;
  }
  return path_bounds.size();  // 未找到
}
```

---

## 统计信息

任务维护两个静态计数器用于监控学习模型的使用率：

- `valid_path_reference_counter_`：学习模型输出被采纳的次数
- `total_path_counter_`：总规划次数
- 使用率 = valid / total，输出到 debug 的 `hybrid_model.learning_model_output_usage_ratio`

---

## 关键配置参数

| 参数 | 说明 |
|------|------|
| `skip_path_reference_in_change_lane` | 换道时跳过学习模型参考 |
| `skip_path_reference_in_side_pass` | 借道时跳过学习模型参考 |
