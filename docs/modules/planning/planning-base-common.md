# Planning Base Common 核心数据结构函数级源码解析

本文聚焦 `modules/planning/planning_base/common/` 目录，按函数级粒度拆解规划模块的 **12 个核心数据结构**：Frame、Obstacle、ReferenceLineInfo、PlanningContext、PathDecision、PathData、SpeedData、STBoundary、DiscretizedTrajectory、EgoInfo、History、SpeedProfileGenerator。

## 1. 模块定位

`planning_base/common/` 是规划模块的**数据层**，定义了规划流水线中流转的所有核心数据结构。这些结构在场景机、任务、控制器之间传递，是理解规划模块的基石。

数据流关系：

```
EgoInfo → Frame → ReferenceLineInfo → [PathDecision, PathData, SpeedData]
                    │                        ↓
                    │              DiscretizedTrajectory
                    ↓
              PlanningContext (跨帧持久化)
              History (历史帧决策)
```

## 2. Frame — 规划帧

```cpp
class Frame {
  // FrameHistory : IndexedQueue<uint32_t, Frame>
};
```

**职责**：封装单次规划周期的所有数据——车辆状态、感知/预测输入、参考线、障碍物、计算结果。

### 2.1 构造与初始化

```cpp
explicit Frame(uint32_t sequence_num);
Frame(uint32_t sequence_num, const LocalView&, const TrajectoryPoint&,
      const VehicleState&, ReferenceLineProvider*);
Status Init(const VehicleStateProvider*, const list<ReferenceLine>&,
            const list<RouteSegments>&, const vector<LaneWaypoint>&, const EgoInfo*);
Status InitForOpenSpace(const VehicleStateProvider*, const EgoInfo*);
```

- `Init`：标准初始化，创建参考线信息列表、障碍物列表
- `InitForOpenSpace`：开放空间专用初始化（简化版，无参考线）

### 2.2 参考线查询

```cpp
const list<ReferenceLineInfo>& reference_line_info() const;
list<ReferenceLineInfo>* mutable_reference_line_info();
const ReferenceLineInfo* FindDriveReferenceLineInfo();
const ReferenceLineInfo* FindTargetReferenceLineInfo();
const ReferenceLineInfo* FindFailedReferenceLineInfo();
const ReferenceLineInfo* DriveReferenceLineInfo() const;
```

- `FindDriveReferenceLineInfo`：找到可驾驶的参考线（已规划成功且代价最低）
- `FindTargetReferenceLineInfo`：找到变道目标参考线
- `FindFailedReferenceLineInfo`：找到规划失败的参考线

### 2.3 障碍物管理

```cpp
Obstacle* Find(const string& id);
vector<const Obstacle*> obstacles() const;
const Obstacle* CreateStopObstacle(ReferenceLineInfo*, const string&, double, double);
const Obstacle* CreateStopObstacle(const string&, const string&, double);
const Obstacle* CreateStaticObstacle(ReferenceLineInfo*, const string&, double, double);
ThreadSafeIndexedObstacles* GetObstacleList();
```

- `CreateStopObstacle`：创建停车墙虚拟障碍物（用于场景机设置停车栅栏）
- `CreateStaticObstacle`：创建静态虚拟障碍物

### 2.4 核心成员

| 成员 | 类型 | 说明 |
|------|------|------|
| `sequence_num_` | uint32_t | 帧序号 |
| `local_view_` | LocalView | 传感器/路由输入视图 |
| `planning_start_point_` | TrajectoryPoint | 规划起始点 |
| `vehicle_state_` | VehicleState | 车辆状态 |
| `reference_line_info_` | list\<ReferenceLineInfo\> | 参考线候选列表 |
| `drive_reference_line_info_` | const ReferenceLineInfo* | 选定的驾驶参考线 |
| `obstacles_` | ThreadSafeIndexedObstacles | 线程安全障碍物列表 |
| `open_space_info_` | OpenSpaceInfo | 开放空间信息 |

### 2.5 FrameHistory

```cpp
class FrameHistory : public IndexedQueue<uint32_t, Frame> {};
```

- 继承 `IndexedQueue`，以帧序号为键存储历史帧
- 供 `History` 类和速度剖面继承使用

## 3. Obstacle — 障碍物

```cpp
class Obstacle {
  using IndexedObstacles = IndexedList<string, Obstacle>;
  using ThreadSafeIndexedObstacles = ThreadSafeIndexedList<string, Obstacle>;
};
```

**职责**：关联感知/预测障碍物与其路径相关属性（s, l）、ST 边界和规划决策。

### 3.1 构造与工厂

```cpp
Obstacle(const string& id, const PerceptionObstacle&, const ObstaclePriority::Priority&, bool is_static);
Obstacle(const string& id, const PerceptionObstacle&, const Trajectory&,
         const ObstaclePriority::Priority&, bool is_static);
static list<unique_ptr<Obstacle>> CreateObstacles(const PredictionObstacles&);
static unique_ptr<Obstacle> CreateStaticVirtualObstacles(const string& id, const Box2d&);
```

- `CreateObstacles`：从预测消息创建障碍物列表，每个预测轨迹一个 Obstacle
- `CreateStaticVirtualObstacles`：创建虚拟障碍物（交通规则模块使用）

### 3.2 几何查询

```cpp
TrajectoryPoint GetPointAtTime(double time) const;
Box2d GetBoundingBox(const TrajectoryPoint&) const;
const Box2d& PerceptionBoundingBox() const;
const Polygon2d& PerceptionPolygon() const;
Polygon2d GetObstacleTrajectoryPolygon(const TrajectoryPoint&) const;
```

- `GetPointAtTime`：获取障碍物在指定时刻的预测位置
- `GetBoundingBox`：获取障碍物在指定位置的 bounding box
- `GetObstacleTrajectoryPolygon`：获取障碍物轨迹多边形

### 3.3 决策管理

```cpp
const ObjectDecisionType& LateralDecision() const;
const ObjectDecisionType& LongitudinalDecision() const;
void AddLongitudinalDecision(const string& tag, const ObjectDecisionType&);
void AddLateralDecision(const string& tag, const ObjectDecisionType&);
bool HasLateralDecision() const;
bool HasLongitudinalDecision() const;
bool HasNonIgnoreDecision() const;
bool IsIgnore() const;
bool IsLongitudinalIgnore() const;
bool IsLateralIgnore() const;
```

- 每个障碍物可累积多个决策（按 decider_tag 标识来源）
- 最终合并为单一的 `LateralDecision` 和 `LongitudinalDecision`
- 决策优先级由静态排序器 `s_lateral_decision_safety_sorter_` / `s_longitudinal_decision_safety_sorter_` 决定

### 3.4 ST 边界

```cpp
const STBoundary& reference_line_st_boundary() const;
const STBoundary& path_st_boundary() const;
void SetReferenceLineStBoundary(const STBoundary&);
void set_path_st_boundary(const STBoundary&);
void BuildReferenceLineStBoundary(const ReferenceLine&, double adc_start_s);
void EraseStBoundary();
```

- `reference_line_st_boundary_`：参考线坐标系下的 ST 边界
- `path_st_boundary_`：路径坐标系下的 ST 边界（更精确）
- `BuildReferenceLineStBoundary`：从障碍物预测轨迹构建参考线 ST 边界

### 3.5 阻塞判断

```cpp
void SetBlockingObstacle(bool);
bool IsBlockingObstacle() const;
bool IsLaneBlocking() const;
void CheckLaneBlocking(const ReferenceLine&);
bool IsLaneChangeBlocking() const;
void SetLaneChangeBlocking(bool);
```

- `IsBlockingObstacle`：是否阻挡自车前进
- `IsLaneBlocking`：是否阻挡当前车道
- `IsLaneChangeBlocking`：是否阻挡变道

## 4. ReferenceLineInfo — 参考线信息

```cpp
class ReferenceLineInfo {
  enum LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  enum OverlapType { CLEAR_AREA, CROSSWALK, OBSTACLE, PNC_JUNCTION, SIGNAL, STOP_SIGN, YIELD_SIGN, JUNCTION };
};
```

**职责**：存储单条参考线候选的所有规划数据——路径决策、路径数据、速度数据、轨迹、代价、调试信息。规划器评估多个 ReferenceLineInfo 并选择最优。

### 4.1 初始化

```cpp
ReferenceLineInfo(const VehicleState&, const TrajectoryPoint&, const ReferenceLine&, const RouteSegments&);
bool Init(const vector<const Obstacle*>&, double target_speed);
bool AddObstacles(const vector<const Obstacle*>&);
Obstacle* AddObstacle(const Obstacle*);
```

- `Init`：初始化参考线信息，添加障碍物，设置目标速度
- `AddObstacles`：将障碍物投影到参考线，计算 SL 边界

### 4.2 核心数据访问

```cpp
PathDecision* path_decision();
const ReferenceLine& reference_line() const;
const PathData& path_data() const;
const SpeedData& speed_data() const;
PathData* mutable_path_data();
SpeedData* mutable_speed_data();
const DiscretizedTrajectory& trajectory() const;
void SetTrajectory(const DiscretizedTrajectory&);
```

### 4.3 代价管理

```cpp
double Cost() const;
void AddCost(double);
void SetCost(double);
double PriorityCost() const;
void SetPriorityCost(double);
```

- `Cost`：总代价（越低越好），由路径和速度任务累加
- `PriorityCost`：参考线优先级代价（变道参考线更高）

### 4.4 速度管理

```cpp
void SetCruiseSpeed(double);
void LimitCruiseSpeed(double);
double GetCruiseSpeed() const;
double GetBaseCruiseSpeed() const;
void SetLatticeCruiseSpeed(double);
void SetLatticeStopPoint(const StopPoint&);
const PlanningTarget& planning_target() const;
```

### 4.5 轨迹组合

```cpp
bool CombinePathAndSpeedProfile(double relative_time, double start_s, DiscretizedTrajectory*);
bool AdjustTrajectoryWhichStartsFromCurrentPos(const TrajectoryPoint&,
    const vector<TrajectoryPoint>&, DiscretizedTrajectory*);
```

- `CombinePathAndSpeedProfile`：将 PathData 和 SpeedData 合并为 DiscretizedTrajectory
- `AdjustTrajectoryWhichStartsFromCurrentPos`：调整轨迹使其从当前位置开始

### 4.6 车道与重叠信息

```cpp
const RouteSegments& Lanes() const;
bool IsChangeLanePath() const;
bool IsNeighborLanePath() const;
const vector<pair<OverlapType, PathOverlap>>& FirstEncounteredOverlaps() const;
int GetPnCJunction(double, PathOverlap*) const;
int GetJunction(double, PathOverlap*) const;
PathOverlap* GetOverlapOnReferenceLine(const string&, const OverlapType&) const;
```

### 4.7 候选路径管理

```cpp
const vector<PathBoundary>& GetCandidatePathBoundaries() const;
void SetCandidatePathBoundaries(vector<PathBoundary>&&);
const vector<PathData>& GetCandidatePathData() const;
vector<PathData>* MutableCandidatePathData();
void SetCandidatePathData(vector<PathData>&&);
```

- 支持多候选路径评估（如正常路径 + 借道路径）

## 5. PlanningContext — 规划上下文

```cpp
class PlanningContext {
  PlanningStatus planning_status_;  // Protobuf
};
```

**职责**：跨帧持久化的运行时上下文，存储场景状态、靠边停车状态、侧方通行状态等。

```cpp
void Clear();
void Init();
const PlanningStatus& planning_status() const;
PlanningStatus* mutable_planning_status();
```

- 唯一数据成员是 `PlanningStatus` Protobuf
- 通过 `mutable_planning_status()` 读写各子状态

## 6. PathDecision — 路径决策

```cpp
class PathDecision {
  IndexedList<string, Obstacle> obstacles_;
  MainStop main_stop_;
  double stop_reference_line_s_;
};
```

**职责**：存储单条参考线路径上的所有障碍物决策，跟踪主停车点。

```cpp
Obstacle* AddObstacle(const Obstacle&);
bool AddLateralDecision(const string& tag, const string& object_id, const ObjectDecisionType&);
bool AddLongitudinalDecision(const string& tag, const string& object_id, const ObjectDecisionType&);
const Obstacle* Find(const string& object_id) const;
void SetSTBoundary(const string& id, const STBoundary&);
void EraseStBoundaries();
MainStop main_stop() const;
double stop_reference_line_s() const;
bool MergeWithMainStop(const ObjectStop&, const string&, const ReferenceLine&, const SLBoundary&);
```

- `MergeWithMainStop`：合并停车决策，保留最近的停车点

## 7. PathData — 路径数据

```cpp
class PathData {
  enum PathPointType { IN_LANE, OUT_ON_FORWARD_LANE, OUT_ON_REVERSE_LANE, OFF_ROAD, UNKNOWN };
};
```

**职责**：存储规划路径的 Cartesian（DiscretizedPath）和 Frenet（FrenetFramePath）双表示。

```cpp
bool SetDiscretizedPath(DiscretizedPath);
bool SetFrenetPath(FrenetFramePath);
void SetReferenceLine(const ReferenceLine*);
bool SetPathPointDecisionGuide(vector<tuple<double, PathPointType, double>>);
PathPoint GetPathPointWithPathS(double s) const;
bool GetPathPointWithRefS(double ref_s, PathPoint*) const;
bool LeftTrimWithRefS(const FrenetFramePoint&);
bool UpdateFrenetFramePath(const ReferenceLine*);
```

- `SetPathPointDecisionGuide`：设置每个路径点的决策引导信息（s, 类型, 到障碍物距离），用于速度边界生成
- `GetPathPointWithRefS`：根据参考线 s 坐标查找路径点
- `PathPointType`：标识路径点在车道内/借道/逆向/路外

### 核心成员

| 成员 | 说明 |
|------|------|
| `discretized_path_` | Cartesian 路径 |
| `frenet_path_` | Frenet 路径 |
| `path_point_decision_guide_` | 决策引导（用于速度边界） |
| `path_label_` | 路径标签（"regular"/"fallback"） |
| `path_reference_` | 学习模型输出的路径参考 |
| `is_reverse_path_` | 是否为倒车路径 |

## 8. SpeedData — 速度数据

```cpp
class SpeedData : public vector<common::SpeedPoint> {};
```

**职责**：速度剖面，有序的 (s, t, v, a, da) 点序列。

```cpp
void AppendSpeedPoint(double s, double time, double v, double a, double da);
bool EvaluateByTime(double time, SpeedPoint*) const;
bool EvaluateByS(double s, SpeedPoint*) const;
double TotalTime() const;
double TotalLength() const;
```

- `EvaluateByTime`：按时间插值获取速度点
- `EvaluateByS`：按纵向距离插值获取速度点（要求 s 单调）

## 9. STBoundary — ST 边界

```cpp
class STBoundary : public common::math::Polygon2d {
  enum BoundaryType { UNKNOWN, STOP, FOLLOW, YIELD, OVERTAKE, KEEP_CLEAR };
};
```

**职责**：障碍物在 ST（空间-时间）图中的占据区域。

### 9.1 工厂方法

```cpp
static STBoundary CreateInstance(const vector<STPoint>& lower, const vector<STPoint>& upper);
static STBoundary CreateInstanceAccurate(const vector<STPoint>& lower, const vector<STPoint>& upper);
```

- `CreateInstance`：去除冗余点后创建
- `CreateInstanceAccurate`：保留所有点（不去除冗余）

### 9.2 查询方法

```cpp
bool GetUnblockSRange(double curr_time, double* s_upper, double* s_lower) const;
bool GetBoundarySRange(double curr_time, double* s_upper, double* s_lower) const;
bool GetBoundarySlopes(double curr_time, double* ds_upper, double* ds_lower) const;
bool IsPointInBoundary(const STPoint&) const;
```

- `GetUnblockSRange`：获取时刻 t 的未阻挡 s 范围
- `GetBoundarySRange`：获取时刻 t 的边界 s 范围
- `GetBoundarySlopes`：获取时刻 t 的 ds/dt 斜率

### 9.3 变换方法

```cpp
STBoundary ExpandByS(double s) const;
STBoundary ExpandByT(double t) const;
STBoundary CutOffByT(double t) const;
```

### 9.4 角点访问（Lattice Planner 专用）

```cpp
STPoint upper_left_point() const;
STPoint upper_right_point() const;
STPoint bottom_left_point() const;
STPoint bottom_right_point() const;
```

## 10. DiscretizedTrajectory — 离散化轨迹

```cpp
class DiscretizedTrajectory : public vector<common::TrajectoryPoint> {};
```

**职责**：车辆将跟随的轨迹点序列。

```cpp
TrajectoryPoint Evaluate(double relative_time) const;
size_t QueryLowerBoundPoint(double relative_time, double epsilon) const;
size_t QueryNearestPoint(const Vec2d& position) const;
size_t QueryNearestPointWithBuffer(const Vec2d&, double buffer) const;
double GetTemporalLength() const;
double GetSpatialLength() const;
void AppendTrajectoryPoint(const TrajectoryPoint&);
void PrependTrajectoryPoints(const vector<TrajectoryPoint>&);
```

- `Evaluate`：按时间插值获取轨迹点
- `QueryLowerBoundPoint`：二分查找时间下界
- `QueryNearestPoint`：查找最近的 2D 位置点

## 11. EgoInfo — 自车信息

```cpp
class EgoInfo {
  TrajectoryPoint start_point_;
  VehicleState vehicle_state_;
  double front_clear_distance_;
  Box2d ego_box_;
};
```

**职责**：每帧更新的自车状态快照。

```cpp
bool Update(const TrajectoryPoint& start_point, const VehicleState& vehicle_state);
void Clear();
TrajectoryPoint start_point() const;
VehicleState vehicle_state() const;
double front_clear_distance() const;
Box2d ego_box() const;
void CalculateFrontObstacleClearDistance(const vector<const Obstacle*>& obstacles);
```

- `Update`：每帧调用，更新起始点（加速度 clamp 到车辆参数限制）
- `CalculateFrontObstacleClearDistance`：计算前方最近障碍物距离

## 12. History — 历史帧

```cpp
class History {
  list<HistoryFrame> history_frames_;
  HistoryStatus history_status_;
};
```

**职责**：维护历史规划帧和跨帧障碍物状态，实现时间一致性。

### 12.1 HistoryFrame

```cpp
class HistoryFrame {
  void Init(const ADCTrajectory&);
  int seq_num() const;
  vector<const HistoryObjectDecision*> GetObjectDecisions() const;
  vector<const HistoryObjectDecision*> GetStopObjectDecisions() const;
  const HistoryObjectDecision* GetObjectDecisionsById(const string&) const;
};
```

### 12.2 HistoryObjectDecision

```cpp
class HistoryObjectDecision {
  void Init(const string& id, const vector<ObjectDecisionType>&);
  const string& id() const;
  vector<const ObjectDecisionType*> GetObjectDecision() const;
};
```

### 12.3 HistoryStatus

```cpp
class HistoryStatus {
  void SetObjectStatus(const string& id, const ObjectStatus&);
  bool GetObjectStatus(const string& id, ObjectStatus*);
};
```

### 12.4 History 主类

```cpp
const HistoryFrame* GetLastFrame() const;
int Add(const ADCTrajectory&);
void Clear();
size_t Size() const;
HistoryStatus* mutable_history_status();
```

## 13. SpeedProfileGenerator — 速度剖面生成器

```cpp
class SpeedProfileGenerator {
 public:
  static void FillEnoughSpeedPoints(SpeedData* speed_data);
  static SpeedData GenerateFixedDistanceCreepProfile(double distance, double max_speed);
};
```

- 纯静态工具类（构造函数已删除）
- `FillEnoughSpeedPoints`：确保速度剖面有足够采样点（填充零速度点）
- `GenerateFixedDistanceCreepProfile`：生成固定距离的蠕行速度剖面

## 14. 其他辅助结构

### 14.1 DecisionData — 决策数据

- 存储规划决策结果（停车点、巡航速度等）

### 14.2 PathBoundary — 路径边界

- 存储路径的左右边界（s, l_lower, l_upper）序列
- 由路径生成任务的 `DecidePathBounds` 产出

### 14.3 StGraphData — ST 图数据

- 存储 ST 图的约束数据（速度限制、ST 边界列表）
- 由 `SpeedBoundsDecider` 产出，供速度优化器使用

### 14.4 SpeedLimit — 速度限制

- 沿参考线的速度限制序列

### 14.5 SLPolygon — SL 多边形

- 障碍物在 SL 坐标系中的多边形表示
- 用于通用借道/变道路径的碰撞检测

### 14.6 LocalView — 本地视图

- 封装所有传感器输入（定位、底盘、预测、交通灯等）
- 作为 `Frame` 的输入

## 15. 数据流全景

```
感知/预测 ──> LocalView ──> Frame.Init()
                              │
                    ┌─────────┼─────────┐
                    ▼         ▼         ▼
              EgoInfo    Obstacle[]  ReferenceLine[]
                    │         │         │
                    └────┬────┘         ▼
                         │      ReferenceLineInfo[]
                         │         │
                    ScenarioManager ──> Scenario
                         │         │
                    ┌────┘    ┌────┘
                    ▼         ▼
              Stage.Process() ──> Task.Execute()
                    │              │
                    │    ┌─────────┼─────────┐
                    │    ▼         ▼         ▼
                    │ PathGeneration  SpeedDecider  SpeedOptimizer
                    │    │              │         │
                    │    ▼              ▼         ▼
                    │ PathData    PathDecision  SpeedData
                    │    │                        │
                    │    └────────┬───────────────┘
                    │             ▼
                    │    CombinePathAndSpeedProfile()
                    │             │
                    │             ▼
                    │    DiscretizedTrajectory
                    │             │
                    └─────> SetTrajectory()
                              │
                              ▼
                         ControlCommand
```