---
title: "LaneChangePath 换道路径"
---

# LaneChangePath 换道路径

> 源码位置：`modules/planning/tasks/lane_change_path/`

## 模块定位

LaneChangePath 在 Routing 指示需要换道时，在目标车道的参考线上生成换道路径。它管理换道状态机（IN_CHANGE_LANE / FINISHED / FAILED），并通过安全距离检查确保换道安全。

---

## 类声明

```cpp
class LaneChangePath : public PathGeneration {
 private:
  Status Process(Frame*, ReferenceLineInfo*) override;
  bool DecidePathBounds(vector<PathBoundary>*);
  bool OptimizePath(const vector<PathBoundary>&, vector<PathData>*);
  bool AssessPath(vector<PathData>*, PathData*);
  void UpdateLaneChangeStatus();
  void GetBoundaryFromLaneChangeForbiddenZone(PathBoundary*);
  void GetLaneChangeStartPoint(const ReferenceLine&, double s, Vec2d* start_xy);
  bool IsClearToChangeLane(ReferenceLineInfo*);
  bool HysteresisFilter(double distance, double safe_dist, double buffer, bool blocking);
  bool is_clear_to_change_lane_;
  bool is_exist_lane_change_start_position_;
  Vec2d lane_change_start_xy_;
};
```

---

## Process() — 主流程

```cpp
Status LaneChangePath::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 更新换道状态机
  UpdateLaneChangeStatus();

  // 2. 前置检查
  if (!rli->IsChangeLanePath() || rli->path_reusable()) return OK;
  if (status != ChangeLaneStatus::IN_CHANGE_LANE) return ERROR;

  // 3. 三步流水线
  GetStartPointSLState();
  DecidePathBounds(&candidate_boundaries);
  OptimizePath(candidate_boundaries, &candidate_data);
  AssessPath(&candidate_data, rli->mutable_path_data());
}
```

- `lane_change_path.cc:L58-L96`

---

## UpdateLaneChangeStatus() — 换道状态机

```
状态转换：
  CHANGE_LANE_FINISHED ──(冷却时间到 + 有换道参考线)──→ IN_CHANGE_LANE
  IN_CHANGE_LANE ──(换道成功/参考线消失)──→ CHANGE_LANE_FINISHED
  IN_CHANGE_LANE ──(上帧失败)──→ CHANGE_LANE_FAILED
  CHANGE_LANE_FAILED ──(冷却时间到)──→ IN_CHANGE_LANE
```

关键配置：
- `change_lane_fail_freeze_time`：失败后冷却时间
- `change_lane_success_freeze_time`：成功后冷却时间

- `lane_change_path.cc:L228-L281`

---

## DecidePathBounds() — 换道边界

```cpp
bool LaneChangePath::DecidePathBounds(vector<PathBoundary>* boundary) {
  // 1. 初始化无穷大边界
  PathBoundsDeciderUtil::InitPathBoundary(...);

  // 2. 基于目标车道获取边界
  PathBoundsDeciderUtil::GetBoundaryFromSelfLane(...);
  PathBoundsDeciderUtil::ExtendBoundaryByADC(...);

  // 3. 设置换道禁止区（起点之前不允许偏移）
  GetBoundaryFromLaneChangeForbiddenZone(&path_bound);

  // 4. 静态障碍物收缩
  PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(...);

  path_bound.set_label("regular/lane_change");
}
```

---

## GetBoundaryFromLaneChangeForbiddenZone()

在换道起点之前的区域，将边界收缩为仅允许在当前车道内行驶（禁止提前偏移到目标车道）。

---

## IsClearToChangeLane() — 安全距离检查

```cpp
bool IsClearToChangeLane(ReferenceLineInfo* rli) {
  for (each dynamic obstacle in target lane) {
    // 判断同向/逆向
    same_direction = |heading_diff| < π/2;

    // 计算安全距离
    if (same_direction) {
      forward_safe = max(10m, (ego_v - obs_v) * 3s);
      backward_safe = max(10m, (obs_v - ego_v) * 3s);
    } else {
      forward_safe = max(50m, (ego_v + obs_v) * 5s);
      backward_safe = 1m;
    }

    // 滞后滤波判断
    if (!HysteresisFilter(backward_gap, backward_safe, 0.5, blocking) ||
        !HysteresisFilter(forward_gap, forward_safe, 0.5, blocking))
      → 不安全，标记 obstacle 为 LaneChangeBlocking
  }
}
```

- `lane_change_path.cc:L283-L367`

---

## HysteresisFilter() — 滞后滤波

```cpp
bool HysteresisFilter(double distance, double safe_distance,
                      double buffer, bool is_blocking) {
  if (is_blocking)
    return distance > safe_distance - buffer;  // 已阻塞时放宽条件
  else
    return distance > safe_distance + buffer;  // 未阻塞时收紧条件
}
```

防止在安全距离边界附近频繁切换决策。

---

## 安全距离常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `kSafeTimeOnSameDirection` | 3.0s | 同向安全时间 |
| `kSafeTimeOnOppositeDirection` | 5.0s | 逆向安全时间 |
| `kForwardMinSafeDistanceOnSameDirection` | 10m | 同向前方最小安全距离 |
| `kBackwardMinSafeDistanceOnSameDirection` | 10m | 同向后方最小安全距离 |
| `kForwardMinSafeDistanceOnOppositeDirection` | 50m | 逆向前方最小安全距离 |
