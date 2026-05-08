---
title: "ReusePath 路径复用"
---

# ReusePath 路径复用

> 源码位置：`modules/planning/tasks/reuse_path/`

## 模块定位

ReusePath 是路径复用任务，用于在满足条件时直接复用上一帧的规划路径，避免每帧重新计算路径。这在换道过程中尤为重要——换道路径一旦生成，应保持稳定直到完成，避免频繁重规划导致的路径抖动。

继承关系：
```
Task → PathGeneration → ReusePath
```

---

## Process 主流程

```cpp
Status ReusePath::Process(Frame* frame, ReferenceLineInfo* rli) {
  if (!rli->path_data().Empty()) return OK;  // 已有路径则跳过

  rli->set_path_reusable(false);

  if (!IsPathReusable(frame, rli)) {
    path_reusable_ = false;
    return OK;
  }
  path_reusable_ = true;

  if (!TrimHistoryPath(frame, rli)) {
    path_reusable_ = false;
    return OK;
  }

  rli->set_path_reusable(true);  // 标记路径已复用
}
```

---

## IsPathReusable — 复用条件判断

复用路径需要同时满足以下条件：

```cpp
bool ReusePath::IsPathReusable(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 场景限制：仅在换道中或配置允许车道跟随时启用
  if (!rli->IsChangeLanePath() && !config_.enable_reuse_path_in_lane_follow())
    return false;
  if (rli->IsChangeLanePath() && status != IN_CHANGE_LANE)
    return false;

  // 2. 上一帧速度优化必须成功（非 SPEED_FALLBACK）
  speed_optimization_successful =
      (history_trajectory_type != SPEED_FALLBACK);

  // 3. 上一帧必须有规划路径
  if (history_frame->current_frame_planned_path().empty())
    return false;

  // 4. 状态机逻辑
  if (path_reusable_) {
    // 已在复用状态：检查无重规划、速度优化成功、无碰撞
    return !is_replan && speed_ok && IsCollisionFree(rli);
  } else {
    // 未在复用状态：等待阻塞障碍物消失或远离后启用
    return (counter <= kWaitCycle || IsIgnoredBlockingObstacle(rli))
           && speed_ok && IsCollisionFree(rli);
  }
}
```

### 停止复用的条件

1. 触发了重规划（`is_replan`）
2. 速度优化失败（fallback）
3. 历史路径与当前障碍物碰撞
4. 历史路径为空

---

## IsCollisionFree — 碰撞检测

```cpp
bool ReusePath::IsCollisionFree(ReferenceLineInfo* rli) {
  // 收集所有静态障碍物的 SL 多边形
  for (obstacle : static_obstacles) {
    if (behind_adc || too_small) continue;
    obstacle_polygons.push_back(SL_polygon);
  }

  // 沿历史路径逐点检查车辆包围盒是否与障碍物碰撞
  for (path_point : history_path) {
    if (behind_adc) continue;
    vehicle_box = GetBoundingBox(path_point);
    for (corner : vehicle_box.GetAllCorners()) {
      corner_sl = XYToSL(corner);
      for (obstacle_polygon : obstacle_polygons) {
        if (obstacle_polygon.IsPointIn(corner_sl))
          return false;  // 碰撞
      }
    }
  }
  return true;
}
```

---

## IsIgnoredBlockingObstacle — 阻塞障碍物忽略判断

```cpp
bool ReusePath::IsIgnoredBlockingObstacle(ReferenceLineInfo* rli) {
  double final_s_buffer = max(30.0m, 3s * adc_speed);
  // 如果阻塞障碍物距离 ADC 超过 buffer，认为可忽略
  return (blocking_obstacle_s - adc_s > final_s_buffer);
}
```

---

## TrimHistoryPath — 裁剪历史路径

将上一帧的路径裁剪到当前 ADC 位置，作为本帧路径：

```cpp
bool ReusePath::TrimHistoryPath(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 获取历史路径
  const auto& history_path = history_frame->current_frame_planned_path();

  // 2. 找到当前 ADC 在历史路径上的最近点
  //    从该点开始截取后续路径

  // 3. 重新计算 s 值（从 0 开始累加弧长）
  for (path_point : trimmed_path) {
    updated_s += distance_to_prev;
    path_point.set_s(updated_s);
  }

  // 4. 检查裁剪后路径长度是否足够
  if (!NotShortPath(trimmed_path)) return false;

  // 5. 设置为当前帧路径
  path_data->SetDiscretizedPath(trimmed_path);
}
```

---

## 关键配置参数

| 参数 | 说明 |
|------|------|
| `enable_reuse_path_in_lane_follow` | 是否在车道跟随中也启用路径复用 |
| `short_path_threshold` | 裁剪后路径最小点数阈值 |
