---
title: "LaneBorrowPath 借道路径"
---

# LaneBorrowPath 借道路径

> 源码位置：`modules/planning/tasks/lane_borrow_path/`

## 模块定位

LaneBorrowPath 在前方有静态障碍物阻塞且无法在本车道内绕行时，借用相邻车道生成绕行路径。它与 LaneFollowPath 共享相同的三步流水线架构（边界→优化→评估），但边界计算扩展到相邻车道。

---

## 类声明

```cpp
enum SidePassDirection { LEFT_BORROW = 1, RIGHT_BORROW = 2 };

class LaneBorrowPath : public PathGeneration {
 private:
  Status Process(Frame*, ReferenceLineInfo*) override;
  bool DecidePathBounds(vector<PathBoundary>*);
  bool OptimizePath(const vector<PathBoundary>&, vector<PathData>*);
  bool AssessPath(vector<PathData>*, PathData*);
  bool GetBoundaryFromNeighborLane(SidePassDirection, PathBoundary*, string*);
  bool IsNecessaryToBorrowLane();
  void CheckLaneBorrow(const ReferenceLineInfo&, bool* left, bool* right);
  bool CheckLaneBoundaryType(const ReferenceLineInfo&, double s, const SidePassDirection&);
  vector<SidePassDirection> decided_side_pass_direction_;
};
```

---

## Process() — 主流程

```cpp
Status LaneBorrowPath::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 前置检查
  if (!config_.is_allow_lane_borrowing() || rli->path_reusable()) return OK;
  if (!IsNecessaryToBorrowLane()) return OK;

  GetStartPointSLState();
  DecidePathBounds(&candidate_boundaries);
  OptimizePath(candidate_boundaries, &candidate_data);
  AssessPath(&candidate_data, rli->mutable_path_data());
}
```

- `lane_borrow_path.cc:L57-L85`

---

## IsNecessaryToBorrowLane() — 借道条件判断

必须同时满足：
1. `HasSingleReferenceLine` — 只有一条参考线
2. `IsWithinSidePassingSpeedADC` — 车速低于借道阈值
3. `IsLongTermBlockingObstacle` — 障碍物持续阻塞（计数器 > 阈值）
4. `IsBlockingObstacleWithinDestination` — 障碍物在目的地之前
5. `IsBlockingObstacleFarFromIntersection` — 障碍物远离交叉口（> 20m）
6. `IsSidePassableObstacle` — 障碍物可侧向通过

---

## DecidePathBounds() — 借道边界

```cpp
bool LaneBorrowPath::DecidePathBounds(vector<PathBoundary>* boundary) {
  for (each decided_side_pass_direction) {
    // 1. 初始化无穷大边界
    PathBoundsDeciderUtil::InitPathBoundary(...);

    // 2. 获取相邻车道边界（核心区别）
    GetBoundaryFromNeighborLane(direction, &path_bound, &borrow_lane_type);

    // 3. 静态障碍物收缩
    PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(...);

    // 4. 追加尾部点
    path_bound.set_label("regular/left|right" + borrow_lane_type);
  }
}
```

---

## GetBoundaryFromNeighborLane() — 相邻车道边界获取

```cpp
bool GetBoundaryFromNeighborLane(SidePassDirection direction,
                                  PathBoundary* path_bound, string* type) {
  for (each point in path_bound) {
    // 1. 获取当前车道宽度
    reference_line.GetLaneWidth(s, &left_width, &right_width);

    // 2. 检查边界线类型（实线不可借）
    if (CheckLaneBoundaryType(rli, s, direction)) {
      // 3. 获取相邻车道宽度
      rli->GetNeighborLaneInfo(LeftForward/LeftReverse/..., s, &id, &width);
    }

    // 4. 扩展边界到相邻车道
    if (LEFT_BORROW)
      path_bound[i].l_upper = curr_lane_left_width + neighbor_width;
    if (RIGHT_BORROW)
      path_bound[i].l_lower = -(curr_lane_right_width + neighbor_width);
  }
}
```

- 支持借用同向车道和逆向车道
- 逆向车道借用时标记 `borrowing_reverse_lane`
- `lane_borrow_path.cc:L240-L317`

---

## CheckLaneBorrow() — 可借道检查

```cpp
void CheckLaneBorrow(const ReferenceLineInfo& rli,
                     bool* left_borrowable, bool* right_borrowable) {
  // 检查 ADC 前方一段距离内的车道边界线类型
  // 实线（SOLID_WHITE/SOLID_YELLOW）→ 不可借
  // 虚线（DOTTED_WHITE/DOTTED_YELLOW）→ 可借
}
```

---

## AssessPath() — 路径评估与选择

```cpp
bool AssessPath(vector<PathData>* candidates, PathData* final) {
  // 1. 验证每条路径合法性
  for (auto& path : *candidates) {
    if (IsValidRegularPath(rli, path)) {
      SetPathInfo(&path);           // 标记 IN_LANE / OUT_ON_FORWARD / OUT_ON_REVERSE
      TrimTailingOutLanePoints();   // 裁剪超出车道的尾部
      valid_paths.push_back(path);
    }
  }

  // 2. 多条有效路径时比较选择
  if (valid_paths.size() > 1)
    ComparePathData(path0, path1, blocking_obstacle);  // 选更优的
  else
    *final = valid_paths[0];
}
```

---

## 关键常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `kIntersectionClearanceDist` | 20.0m | 交叉口安全距离 |
| `kJunctionClearanceDist` | 15.0m | 路口安全距离 |
