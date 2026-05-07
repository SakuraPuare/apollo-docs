# Planning Tasks 规划任务函数级源码解析

本文聚焦 `modules/planning/tasks/` 目录，按函数级粒度拆解 Apollo 规划模块的 **34 个任务**实现。任务是场景机（Scenario）阶段（Stage）内部调用的最小执行单元，负责路径生成、速度规划、障碍物决策等具体算法。

## 1. 模块定位

任务（Task）是规划流水线的**最底层执行单元**。场景机的每个阶段按配置顺序调用一组任务，任务之间通过 `ReferenceLineInfo` 共享数据。

```
Scenario → Stage → [Task_0, Task_1, ..., Task_N]
                      │
                      ├── 路径生成任务 → PathData
                      ├── 路径决策任务 → PathDecision
                      ├── 速度边界任务 → StGraphData
                      └── 速度规划任务 → SpeedData
```

## 2. 基类层次

| 基类 | 用途 | 核心方法 |
|------|------|---------|
| `Task` | 根基类 | `Execute(Frame*, ReferenceLineInfo*)` |
| `PathGeneration : Task` | 路径生成模板 | `Process()` → `DecidePathBounds` → `OptimizePath` → `AssessPath` |
| `Decider : Task` | 决策/过滤器 | `Process(Frame*, ReferenceLineInfo*)` 或 `Process(Frame*)` |
| `SpeedOptimizer : Task` | 速度优化器 | `Process(PathData&, TrajectoryPoint&, SpeedData*)` |
| `TrajectoryOptimizer : Task` | 开放空间轨迹优化 | `Process()`（无参，使用 frame 级数据） |
| `TrajectoryFallbackTask : Task` | 回退轨迹生成 | `GenerateFallbackSpeed(EgoInfo*, stop_distance)` |

### 2.1 PathGeneration 模板方法

```cpp
class PathGeneration : public Task {
  void Process(Frame*, ReferenceLineInfo*) {
    DecidePathBounds(&path_boundaries);  // 确定路径边界
    OptimizePath(path_boundaries, path_data);  // 优化路径
    AssessPath(path_data);  // 评估路径质量
  }
  virtual void DecidePathBounds(vector<PathBoundary>*) = 0;
  virtual void OptimizePath(...) = 0;
  virtual void AssessPath(...) = 0;
};
```

所有路径生成任务遵循此三步模板：边界确定 → 路径优化 → 路径评估。

## 3. 路径生成任务

### 3.1 LaneFollowPath — 车道跟随路径

```cpp
class LaneFollowPath : public PathGeneration {
  void DecidePathBounds(vector<PathBoundary>*) override;
  void OptimizePath(...) override;
  void AssessPath(...) override;
};
```

- 默认路径生成任务
- `DecidePathBounds`：基于当前车道宽度和障碍物 SL 边界计算路径可行区域
- `OptimizePath`：使用 QP 优化器在边界内生成平滑路径
- `AssessPath`：评估路径的可行性和代价

### 3.2 LaneBorrowPath — 借道路径

```cpp
class LaneBorrowPath : public PathGeneration {
  void DecidePathBounds(vector<PathBoundary>*) override;
  void OptimizePath(...) override;
  void AssessPath(...) override;
  bool IsNecessaryToBorrowLane();
  void GetBoundaryFromNeighborLane(SidePassDirection, PathBoundary*, string*);
  bool CheckLaneBorrow(...);
  bool CheckLaneBoundaryType(...);
};
```

- 低速时借相邻车道绕过阻塞障碍物
- `IsNecessaryToBorrowLane`：判断是否需要借道（障碍物阻挡且无法在本车道内绕过）
- `CheckLaneBoundaryType`：检查相邻车道边界类型（实线不可借道）
- `GetBoundaryFromNeighborLane`：从相邻车道获取扩展边界
- `SidePassDirection`：左借或右借

### 3.3 LaneBorrowPathGeneric — 通用借道路径

```cpp
class LaneBorrowPathGeneric : public PathGeneration {
  void GetBoundaryFromNudgeDecision(...);
  bool IsEnableNudge(...);
  bool IsNudgeFinish();
  void GetSLPolygons(vector<SLPolygon>*, LaneBorrowInfo);
};
```

- 增强版借道，支持基于 nudge 决策的边界计算
- `GetSLPolygons`：生成 SL 多边形用于碰撞检测
- 适用于泊车/开放空间感知场景

### 3.4 LaneChangePath — 变道路径

```cpp
class LaneChangePath : public PathGeneration {
  void DecidePathBounds(vector<PathBoundary>*) override;
  void OptimizePath(...) override;
  void AssessPath(...) override;
  void UpdateLaneChangeStatus();
  void GetBoundaryFromLaneChangeForbiddenZone(PathBoundary*);
  bool IsClearToChangeLane(...);
  void GetLaneChangeStartPoint(...);
};
```

- 生成变道路径
- `UpdateLaneChangeStatus`：更新变道状态机（准备→执行→完成）
- `GetBoundaryFromLaneChangeForbiddenZone`：计算变道禁区边界
- `IsClearToChangeLane`：检查目标车道是否安全（带迟滞滤波）

### 3.5 LaneChangePathGeneric — 通用变道路径

- 增强版变道，增加 nudge 决策边界和 SL 多边形碰撞检测
- 适用于泊车场景

### 3.6 PullOverPath — 靠边停车路径

```cpp
class PullOverPath : public PathGeneration {
  void DecidePathBounds(vector<PathBoundary>*) override;
  void OptimizePath(...) override;
  void AssessPath(...) override;
  void GetBoundaryFromRoads(...);
  void UpdatePullOverBoundaryByLaneBoundary(...);
  void SearchPullOverPosition(...);
  void FindNearestPullOverS(...);
  void FindDestinationPullOverS(...);
};
```

- `SearchPullOverPosition`：搜索可行的靠边停车位置
- `FindNearestPullOverS`：找最近的可停车纵向位置
- `FindDestinationPullOverS`：在目的地附近找停车位置

### 3.7 FallbackPath — 回退路径

```cpp
class FallbackPath : public PathGeneration {
  void DecidePathBounds(vector<PathBoundary>*) override;
  void OptimizePath(...) override;
  void AssessPath(...) override;
};
```

- 主路径规划失败时的紧急回退
- 使用更宽的边界寻找任何可行驶路径

### 3.8 ReusePath — 路径复用

```cpp
class ReusePath : public PathGeneration {
  bool IsPathReusable(...);
  bool IsCollisionFree(...);
  bool NotShortPath(...);
  void TrimHistoryPath(...);
  bool IsIgnoredBlockingObstacle(...);
};
```

- `IsPathReusable`：判断上一帧路径是否可复用
  - `IsCollisionFree`：碰撞检测
  - `NotShortPath`：路径长度检查
  - 停车位置一致性检查
- 避免不必要的重新计算，提升帧率

### 3.9 ReversePath — 倒车路径

```cpp
class ReversePath : public PathGeneration {
  void DecidePathBounds(PathBoundary*, double&) override;
  void OptimizePath(PathBoundary*, PathData*) override;
  void OptimizePathOsqp(...);
  void GetBoundaryFromSquare(...);
};
```

- 生成倒车行驶路径
- `OptimizePathOsqp`：使用 OSQP 求解器优化路径
- `GetBoundaryFromSquare`：从广场/环岛区域获取边界

### 3.10 SquarePath — 广场路径

```cpp
class SquarePath : public PathGeneration {
  void GetBoundaryFromSquare(...);
};
```

- 生成广场/U 型弯路径
- 从方形约束区域计算边界

## 4. 路径决策任务

### 4.1 PathDecider — 路径决策

```cpp
class PathDecider : public Task {
  void Process(const ReferenceLineInfo*, const PathData&, PathDecision*);
  void MakeObjectDecision(...);
  void MakeStaticObstacleDecision(...);
  void IgnoreBackwardObstacle(...);
  void GenerateObjectStopDecision(const Obstacle&);
};
```

- 对静态障碍物做出路径级决策（停车/忽略）
- `MakeStaticObstacleDecision`：对每个静态障碍物判断
  - 在路径内 → 生成停车决策
  - 在路径外 → 忽略
- `IgnoreBackwardObstacle`：忽略自车后方的障碍物

### 4.2 PathReferenceDecider — 路径参考决策

```cpp
class PathReferenceDecider : public Task {
  bool IsValidPathReference(...);
  bool IsADCBoxAlongPathReferenceWithinPathBounds(...);
  void EvaluatePathReference(...);
};
```

- 验证学习模型生成的路径参考是否可用
- `IsValidPathReference`：检查路径参考是否在规则边界内
- `IsADCBoxAlongPathReferenceWithinPathBounds`：检查自车 box 沿路径参考是否在边界内

### 4.3 ObstacleNudgeDecider — 障碍物避让决策

```cpp
class ObstacleNudgeDecider : public Task {
  void Process(Frame*, ReferenceLineInfo*);
};
```

- 计算开放空间/泊车场景中障碍物的横向避让偏移
- 委托 `NudgeCalculation` 辅助类计算具体偏移量

## 5. 速度任务

### 5.1 SpeedDecider — 速度决策

```cpp
class SpeedDecider : public Task {
  void MakeObjectDecision(const SpeedData&, PathDecision*);
  void CreateStopDecision(...);
  void CreateFollowDecision(...);
  void CreateYieldDecision(...);
  void CreateOvertakeDecision(...);
  int GetSTLocation(...);
  bool CheckIsFollow(...);
  bool IsFollowTooClose(...);
  double EstimateProperFollowGap(...);
};
```

- 对每个障碍物做出速度级决策
- `GetSTLocation`：判断障碍物在 ST 图中的位置（前方/后方/上方/下方）
- `CheckIsFollow`：判断是否需要跟车
- `IsFollowTooClose`：判断跟车距离是否过近
- `EstimateProperFollowGap`：估算合适的安全跟车距离

### 5.2 SpeedBoundsDecider — 速度边界决策

```cpp
class SpeedBoundsDecider : public Decider {
  void Process(Frame*, ReferenceLineInfo*);
  void SetSpeedFallbackDistance(PathDecision*);
};
```

- 计算速度限制边界和 ST 边界映射
- 使用 `SpeedLimitDecider` 计算限速
- 使用 `STBoundaryMapper` 将障碍物映射到 ST 空间
- 输出 `StGraphData` 供下游速度优化器使用

### 5.3 STBoundsDecider — ST 边界决策

```cpp
class STBoundsDecider : public Decider {
  void Process(Frame*, ReferenceLineInfo*);
  void GenerateRegularSTBound(...);
  void GenerateFallbackSTBound(...);
  void RemoveInvalidDecisions(...);
  void RankDecisions(...);
  void BackwardFlatten(...);
};
```

- 计算安全 ST 空间边界
- 使用 `STObstaclesProcessor` 处理障碍物
- 使用 `STDrivingLimits` 计算驾驶限制
- 使用 `STGuideLine` 生成引导线
- `RankDecisions`：对决策排序
- `BackwardFlatten`：反向展平确保连续性

### 5.4 PathTimeHeuristicOptimizer — ST 图启发式搜索

```cpp
class PathTimeHeuristicOptimizer : public SpeedOptimizer {
  void Process(const PathData&, const TrajectoryPoint&, SpeedData*);
  void SearchPathTimeGraph(SpeedData*);
};
```

- 使用动态规划在 ST 网格图上搜索初始可行速度剖面
- `GriddedPathTimeGraph`：网格化 ST 图
- `DpStCost`：DP 代价函数
- `STGraphPoint`：ST 图节点

### 5.5 PiecewiseJerkSpeedOptimizer — 分段 Jerk 速度优化

```cpp
class PiecewiseJerkSpeedOptimizer : public SpeedOptimizer {
  void Process(const PathData&, const TrajectoryPoint&, SpeedData*);
  void AdjustInitStatus(...);
};
```

- 使用 QP 优化速度剖面，最小化 jerk
- 约束：速度/加速度边界、ST 边界约束
- 输出平滑的速度曲线

### 5.6 PiecewiseJerkSpeedNonlinearOptimizer — 非线性速度优化

```cpp
class PiecewiseJerkSpeedNonlinearOptimizer : public SpeedOptimizer {
  void Process(const PathData&, const TrajectoryPoint&, SpeedData*);
  void SetUpStatesAndBounds(...);
  void SmoothSpeedLimit();
  void SmoothPathCurvature(...);
  void OptimizeByQP(...);
  void OptimizeByNLP(...);
};
```

- 使用 IPOPT 非线性优化器
- `SmoothSpeedLimit`：平滑速度限制
- `SmoothPathCurvature`：平滑路径曲率
- `OptimizeByQP`：QP 热启动
- `OptimizeByNLP`：NLP 精细优化

## 6. 开放空间任务

### 6.1 ROI 决策

#### OpenSpaceRoiDecider — ROI 决策器

```cpp
class OpenSpaceRoiDecider : public Decider {
  void Process(Frame*);
  void GetParkingSpot(...);
  void GetPullOverSpot(...);
  void GetRoadBoundary(...);
  void GetParkingBoundary(...);
  void FormulateBoundaryConstraints(...);
  void LoadObstacleInVertices(...);
  void LoadObstacleInHyperPlanes(...);
};
```

- 确定开放空间规划的兴趣区域（ROI）
- 从 HD Map 提取目标车位/靠边停车位置
- 计算道路边界
- 将障碍物转换为顶点和超平面（Ax > b）表示

#### OpenSpaceRoiDeciderPark — 泊车 ROI 决策器

- 增强版 ROI，支持大曲率机动（U 型弯）
- `GetLargeCurvatureEndPose`：计算大曲率终端位姿
- `GetParkingSoftBoundary`：计算泊车软边界

### 6.2 预停车/重规划/回退

#### OpenSpacePreStopDecider — 预停车决策

- 在进入开放空间前设置停车栅栏
- `CheckParkingSpotPreStop`：检查车位预停车
- `CheckPullOverPreStop`：检查靠边停车预停车

#### OpenSpaceReplanDecider — 重规划决策

- 判断是否需要重新规划开放空间轨迹
- 基于轨迹偏差和环境变化

#### OpenSpaceFallbackDecider — 回退决策

```cpp
class OpenSpaceFallbackDecider : public Decider {
  void BuildPredictedEnvironment(...);
  bool IsCollisionFreeTrajectory(...);
  bool IsCollisionFreeEgoBox();
};
```

- 检查当前轨迹与预测障碍物的碰撞
- 碰撞时触发回退（停车或重规划）

#### OpenSpaceFallbackDeciderPark — 泊车回退决策

- 区分静态/动态障碍物碰撞
- `CalculateFallbackTrajectory`：计算减速回退轨迹

### 6.3 轨迹生成与优化

#### OpenSpacePathPlanning — 开放空间路径规划

```cpp
class OpenSpacePathPlanning : public TrajectoryOptimizer {
  void Process();
  void PathPlanning();
  void GeneratePathThread();
};
```

- 使用 Hybrid A* 算法在后台线程生成粗路径
- 在 ROI 边界约束内计算无碰撞运动学路径

#### OpenSpaceTrajectoryProvider — 轨迹提供者

```cpp
class OpenSpaceTrajectoryProvider : public TrajectoryOptimizer {
  void Process();
  void GenerateTrajectoryThread();
  bool IsVehicleNearDestination(...);
  bool IsVehicleStopDueToFallBack(...);
  void GenerateStopTrajectory(...);
  void ReuseLastFrameResult(...);
  void Stop();
  void Restart();
};
```

- 编排开放空间轨迹生成
- 后台线程运行 `OpenSpaceTrajectoryOptimizer`
- 管理生命周期（启动/停止/重启）

#### OpenSpaceTrajectoryOptimizerPark — 泊车轨迹优化器

- 泊车专用轨迹优化
- 后台线程运行 `Optimizer` 对象

### 6.4 轨迹后处理

#### OpenSpaceTrajectoryPartition — 轨迹分段

```cpp
class OpenSpaceTrajectoryPartition : public TrajectoryOptimizer {
  void PartitionTrajectory(...);
  void InterpolateTrajectory(...);
  void EncodeTrajectory(...);
  void CheckTrajTraversed(...);
  void InsertGearShiftTrajectory(...);
  void GenerateGearShiftTrajectory(...);
};
```

- 将连续轨迹按档位分段（前进/倒车）
- 跟踪遍历进度
- 处理换挡空闲轨迹
- 轨迹编码用于重规划检测

#### OpenSpaceTrajectoryPostProcess — 轨迹后处理

- 与 `Partition` 类似，增加到达点检查和停车轨迹生成
- 最终管线阶段

## 7. 回退任务

### 7.1 FallbackPath — 回退路径

- 主路径规划失败时生成紧急路径
- 使用更宽的边界

### 7.2 FastStopTrajectoryFallback — 快速停车回退

```cpp
class FastStopTrajectoryFallback : public TrajectoryFallbackTask {
  void GenerateFallbackSpeed(const EgoInfo*, double stop_distance);
  static void GenerateStopProfile(double init_speed, double init_acc);
};
```

- 使用最大减速度尽快停车
- 最小停车距离

### 7.3 SmoothStopTrajectoryFallback — 平滑停车回退

```cpp
class SmoothStopTrajectoryFallback : public TrajectoryFallbackTask {
  void GenerateFallbackSpeed(const EgoInfo*, double stop_distance);
  bool IsCollisionWithSpeedBoundaries(const SpeedData&);
  void GetLowerSTBound(vector<vector<Vec2d>>&);
};
```

- 更舒适的停车轨迹
- `IsCollisionWithSpeedBoundaries`：检查与 ST 边界的碰撞安全性

## 8. 其他任务

### 8.1 RssDecider — RSS 安全决策

```cpp
class RssDecider : public Task {
  void Process(Frame*, ReferenceLineInfo*);
  rss_config_default_dynamics(...);
  rss_create_ego_object(...);
  rss_create_other_object(...);
};
```

- 应用 Intel/Mobileye RSS（责任敏感安全）规则
- 检查与障碍物的纵向/横向安全距离
- 标记安全违规

### 8.2 RuleBasedStopDecider — 规则停车决策

```cpp
class RuleBasedStopDecider : public Decider {
  void Process(Frame*, ReferenceLineInfo*);
  void AddPathEndStop(...);
  void CheckLaneChangeUrgency(...);
  void StopOnSidePass(...);
  void CheckSidePassStop(...);
  void BuildSidePassStopFence(...);
};
```

- 在路径末端设置停车栅栏
- 紧急变道场景的停车决策
- 侧方通行（side-pass）的停车检查

### 8.3 ReverseSpeed — 倒车速度

```cpp
class ReverseSpeed : public Task {
  void Process(const ReferenceLineInfo*, const PathData&, PathDecision*);
  void GetSTboundaries(...);
  void CheckOverlap(...);
};
```

- 计算倒车速度剖面
- 使用多边形重叠检查构建 ST 边界

## 9. 任务执行管线

### 9.1 典型管线配置

**车道跟随管线**：
```
ReusePath → LaneFollowPath → LaneBorrowPath → PathDecider
→ SpeedBoundsDecider → PathTimeHeuristicOptimizer
→ PiecewiseJerkSpeedOptimizer → SpeedDecider
```

**变道管线**：
```
LaneChangePath → PathDecider → SpeedBoundsDecider
→ PathTimeHeuristicOptimizer → PiecewiseJerkSpeedOptimizer
→ SpeedDecider
```

**开放空间管线**：
```
OpenSpaceRoiDecider → OpenSpacePreStopDecider
→ OpenSpacePathPlanning → OpenSpaceTrajectoryProvider
→ OpenSpaceTrajectoryPartition → OpenSpaceFallbackDecider
```

### 9.2 数据流

```
PathGeneration → PathData → PathDecider → PathDecision
                                              ↓
SpeedBoundsDecider → StGraphData → SpeedOptimizer → SpeedData
                                                      ↓
                                              SpeedDecider → 最终决策
```

## 10. 设计模式

### 10.1 模板方法模式

- `PathGeneration` 定义 `DecidePathBounds → OptimizePath → AssessPath` 骨架
- 子类只需实现三个钩子函数

### 10.2 策略模式

- 不同的路径生成任务（LaneFollow/LaneBorrow/LaneChange）可互换
- 不同的速度优化器（Heuristic/PiecewiseJerk/Nonlinear）可互换

### 10.3 后台线程

- 开放空间任务大量使用后台线程（`GeneratePathThread`/`GenerateTrajectoryThread`）
- 通过 `LoadResult`/`ReuseLastFrameResult` 与主线程同步
- 支持 `Stop`/`Restart` 生命周期管理