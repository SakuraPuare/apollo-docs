---
title: "Generic 路径任务（借道/换道）"
---

# Generic 路径任务（借道/换道）

> 源码位置：
> - `modules/planning/tasks/lane_borrow_path_generic/`
> - `modules/planning/tasks/lane_change_path_generic/`

## 模块定位

Generic 版本是 `LaneBorrowPath` 和 `LaneChangePath` 的增强实现，集成了 `ParkDataCenter` 数据中心，支持更复杂的园区/泊车场景。核心流程与标准版一致（DecidePathBounds → OptimizePath → AssessPath），但在边界决策和安全检查上有额外逻辑。

---

## LaneBorrowPathGeneric

继承关系：`PathGeneration → LaneBorrowPathGeneric`

### 与标准版的主要差异

| 特性 | 标准版 | Generic 版 |
|------|--------|-----------|
| 数据源 | ReferenceLineInfo | + ParkDataCenter |
| Nudge 决策 | 无 | `GetBoundaryFromNudgeDecision` |
| 借道判断 | 简单计数器 | 多条件综合判断 |
| 路径评估 | 基础评估 | `ComparePathData` 多维度比较 |

### 借道必要性判断

```cpp
bool LaneBorrowPathGeneric::IsNecessaryToBorrowLane() {
  // 1. 必须是单参考线（非换道中）
  if (!HasSingleReferenceLine(frame)) return false;

  // 2. ADC 速度在侧方通过范围内
  if (!IsWithinSidePassingSpeedADC(frame)) return false;

  // 3. 存在长期阻塞障碍物
  if (!IsLongTermBlockingObstacle()) return false;

  // 4. 障碍物可侧方通过
  if (!IsSidePassableObstacle(rli)) return false;

  return true;
}
```

### 车道线类型检查

```cpp
bool CheckLaneBoundaryType(rli, check_s, direction) {
  // 实线不可借道，虚线可借道
  // 检查从 ADC 到阻塞障碍物之间的车道线类型
}
```

### 路径比较

```cpp
bool ComparePathData(rli, lhs, rhs, blocking_obstacle, narrowest_width) {
  // 多维度比较两条候选路径：
  // - 是否包含逆向车道段
  // - 路径宽度
  // - 与阻塞障碍物的距离
  // - 回到本车道的位置
}
```

---

## LaneChangePathGeneric

继承关系：`PathGeneration → LaneChangePathGeneric`

### 与标准版的主要差异

| 特性 | 标准版 | Generic 版 |
|------|--------|-----------|
| 数据源 | ReferenceLineInfo | + ParkDataCenter |
| Nudge 决策 | 无 | `GetBoundaryFromNudgeDecision` |
| 安全检查 | 基础 | `HysteresisFilter` 滞回滤波 |
| 上帧检查 | 无 | `CheckLastFrameSucceed` |

### 换道安全检查

```cpp
bool LaneChangePathGeneric::IsClearToChangeLane(ReferenceLineInfo* rli) {
  // 检查目标车道前后方障碍物是否满足安全距离
  for (obstacle : obstacles) {
    if (!HysteresisFilter(distance, safe_distance, buffer, is_blocking))
      return false;
  }
  return true;
}
```

### 滞回滤波

```cpp
bool HysteresisFilter(obstacle_distance, safe_distance,
                      distance_buffer, is_obstacle_blocking) {
  // 进入阻塞状态需要距离 < safe_distance
  // 退出阻塞状态需要距离 > safe_distance + buffer
  // 避免在临界距离处频繁切换
}
```

### 换道禁止区域

```cpp
void GetBoundaryFromLaneChangeForbiddenZone(PathBoundary* path_bound) {
  // 在换道起点之前，将路径边界限制在当前车道内
  // 换道起点之后，逐渐放开到目标车道
  GetLaneChangeStartPoint(reference_line, adc_s, &start_xy);
  // 在 start_s 之前收紧边界
}
```

---

## 共同的 Process 流程

```cpp
Status Process(Frame* frame, ReferenceLineInfo* rli) {
  if (!rli->path_data().Empty()) return OK;

  GetStartPointSLState();
  DecidePathBounds(&boundaries);
  OptimizePath(boundaries, &candidates);
  AssessPath(&candidates, final_path);
}
```

---

## 关键配置参数

### LaneBorrowPathGeneric

| 参数 | 说明 |
|------|------|
| 借道方向 | LEFT_BORROW / RIGHT_BORROW |
| 阻塞计数阈值 | 判定长期阻塞的帧数 |
| 侧方通过速度上限 | 允许借道的最大速度 |

### LaneChangePathGeneric

| 参数 | 说明 |
|------|------|
| 安全距离 | 换道前后方安全距离 |
| 距离缓冲 | 滞回滤波缓冲区 |
| 换道起点偏移 | 换道开始位置的 s 偏移 |
