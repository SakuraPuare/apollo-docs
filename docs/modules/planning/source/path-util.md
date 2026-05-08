# Path Utility 路径工具类

> 源码位置：`modules/planning/planning_interface_base/task_base/common/path_util/`

## 模块定位

路径工具类是所有路径生成任务（`PathGeneration` 子类）的公共基础设施，提供三大能力：

1. **PathBoundsDeciderUtil** — 路径边界计算：初始化边界、根据车道/障碍物收缩边界、SL 多边形处理
2. **PathAssessmentDeciderUtil** — 路径质量评估：验证路径合法性（偏离参考线、偏离道路、碰撞检测）
3. **PathOptimizerUtil** — 路径优化求解：Piecewise Jerk QP 优化器封装、参考线权重更新

三者在 `PathGeneration` 模板方法中的调用顺序：

```
DecidePathBounds() → PathBoundsDeciderUtil
OptimizePath()     → PathOptimizerUtil
AssessPath()       → PathAssessmentDeciderUtil
```

## 1. PathBoundsDeciderUtil — 路径边界决策工具

> 源码：`path_bounds_decider_util.h` / `path_bounds_decider_util.cc`

### 类型定义

```cpp
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;
using SLState = std::pair<std::array<double, 3>, std::array<double, 3>>;

enum class LaneBorrowInfo {
  LEFT_BORROW,
  NO_BORROW,
  RIGHT_BORROW,
};
```

- `ObstacleEdge`：障碍物边缘 (is_start_s, s, l_min, l_max, obstacle_id)
- `SLState`：Frenet 坐标状态 (s, s', s''), (l, l', l'')

### 类声明

```cpp
class PathBoundsDeciderUtil {
 public:
  static bool InitPathBoundary(const ReferenceLineInfo&, PathBoundary*, SLState);
  static void GetStartPoint(TrajectoryPoint, const ReferenceLine&, SLState*);
  static double GetADCLaneWidth(const ReferenceLine&, double adc_frenet_s);
  static bool GetBoundaryFromLanesAndADC(...);
  static bool UpdatePathBoundaryWithBuffer(double left, double right, ...);
  static bool UpdateLeftPathBoundaryWithBuffer(...);
  static bool UpdateRightPathBoundaryWithBuffer(...);
  static void TrimPathBounds(int path_blocked_idx, PathBoundary*);
  static bool IsWithinPathDeciderScopeObstacle(const Obstacle&);
  static void GetSLPolygons(const ReferenceLineInfo&, vector<SLPolygon>*, const SLState&);
  static bool UpdatePathBoundaryBySLPolygon(...);
  static bool AddCornerPoint(...);
  static void AddCornerBounds(const vector<SLPolygon>&, PathBoundary*);
  static void AddAdcVertexBounds(PathBoundary*);
  static bool RelaxBoundaryPoint(...);
  static bool RelaxEgoPathBoundary(PathBoundary*, const SLState&);
  static bool RelaxObsCornerBoundary(PathBoundary*, const SLState&);
  static bool AddExtraPathBound(...);
};
```

### 核心函数

#### InitPathBoundary()

```cpp
bool PathBoundsDeciderUtil::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info,
    PathBoundary* const path_bound, SLState init_sl_state) {
  path_bound->clear();
  path_bound->set_delta_s(FLAGS_path_bounds_decider_resolution);
  const double ego_front_to_center = vehicle_config.vehicle_param().front_edge_to_center();
  for (double curr_s = init_sl_state.first[0];
       curr_s < fmin(init_sl_state.first[0] +
                     fmax(FLAGS_path_bounds_horizon, cruise_speed * FLAGS_trajectory_time_length),
                     reference_line.Length() - ego_front_to_center);
       curr_s += FLAGS_path_bounds_decider_resolution) {
    path_bound->emplace_back(curr_s, lowest, max);  // 初始为无穷大边界
  }
  return !path_bound->empty();
}
```

- 从 ADC 当前 s 位置开始，按 `FLAGS_path_bounds_decider_resolution` 步长采样
- 规划范围取 `path_bounds_horizon` 和 `cruise_speed × trajectory_time_length` 的较大值
- 初始边界设为 ±∞，后续由其他函数收缩
- `path_bounds_decider_util.cc:L40-L71`

#### GetStartPoint()

```cpp
void PathBoundsDeciderUtil::GetStartPoint(
    TrajectoryPoint planning_start_point,
    const ReferenceLine& reference_line, SLState* init_sl_state) {
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point = InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  *init_sl_state = reference_line.ToFrenetFrame(planning_start_point);
}
```

- 将规划起点从笛卡尔坐标转换为 Frenet 坐标
- 支持前轴中心/后轴中心两种参考点模式
- `path_bounds_decider_util.cc:L73-L88`

#### GetBoundaryFromLanesAndADC()

- 根据车道边界和 ADC 当前位置收缩路径边界
- 支持借道模式（`LaneBorrowInfo`）：左借、右借、不借
- 如果 ADC 已经超出车道，会扩展边界以包含当前位置
- 支持变道回退场景（`is_fallback_lanechange`）

#### UpdatePathBoundaryWithBuffer()

```cpp
bool PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
    double left_bound, double right_bound, BoundType left_type,
    BoundType right_type, string left_id, string right_id,
    PathBoundPoint* const bound_point) {
  // 左边界减去半车宽，右边界加上半车宽
  // 如果 l_lower > l_upper 则路径被阻塞，返回 false
}
```

- 更新单个路径边界点，自动扣除半车宽作为安全缓冲
- 检测路径是否被阻塞（左右边界交叉）
- `path_bounds_decider_util.cc:L104-L164`

#### TrimPathBounds()

- 当路径在某个 s 位置被阻塞时，截断该位置之后的所有边界点
- 截断位置需减去 `front_edge_to_center` 以确保车头不超出
- `path_bounds_decider_util.cc:L166-L183`

#### GetSLPolygons()

```cpp
void PathBoundsDeciderUtil::GetSLPolygons(
    const ReferenceLineInfo& reference_line_info,
    vector<SLPolygon>* polygons, const SLState& init_sl_state) {
  for (const auto* obstacle : obstacles.Items()) {
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) continue;
    if (obstacle->PerceptionSLBoundary().end_s() < adc_back_edge_s) continue;
    polygons->emplace_back(obstacle_sl, obstacle->Id());
  }
  sort(polygons->begin(), polygons->end(), [](a, b) { return a.MinS() < b.MinS(); });
}
```

- 将所有在路径决策范围内的障碍物转换为 SL 多边形
- 过滤掉 ADC 后方的障碍物
- 按 s 坐标排序，便于后续顺序处理
- `path_bounds_decider_util.cc:L185-L210`

#### UpdatePathBoundaryBySLPolygon()

- 根据 SL 多边形逐点收缩路径边界
- 对每个边界点，检查所有与之 s 范围重叠的障碍物多边形
- 根据障碍物相对于路径中心的位置决定 nudge 方向（左绕/右绕）
- 记录最窄宽度和阻塞障碍物 ID
- `path_bounds_decider_util.cc:L212-L249`

## 2. PathAssessmentDeciderUtil — 路径评估工具

> 源码：`path_assessment_decider_util.h` / `path_assessment_decider_util.cc`

### 类声明

```cpp
using PathPointDecision = std::tuple<double, PathData::PathPointType, double>;
constexpr double kMinObstacleArea = 1e-4;

class PathAssessmentDeciderUtil {
 public:
  static bool IsValidRegularPath(const ReferenceLineInfo&, const PathData&);
  static bool IsGreatlyOffReferenceLine(const PathData&);
  static bool IsGreatlyOffRoad(const ReferenceLineInfo&, const PathData&);
  static bool IsCollidingWithStaticObstacles(const ReferenceLineInfo&, const PathData&);
  static bool IsStopOnReverseNeighborLane(const ReferenceLineInfo&, const PathData&);
  static void InitPathPointDecision(const PathData&, PathPointType, vector<PathPointDecision>*);
  static void TrimTailingOutLanePoints(PathData*);
};
```

### 核心函数

#### IsValidRegularPath()

```cpp
bool PathAssessmentDeciderUtil::IsValidRegularPath(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  if (path_data.Empty()) return false;
  if (IsGreatlyOffReferenceLine(path_data)) return false;
  if (IsGreatlyOffRoad(reference_line_info, path_data)) return false;
  if (IsStopOnReverseNeighborLane(reference_line_info, path_data)) return false;
  return true;
}
```

- 路径合法性的总入口，依次检查四个条件
- 碰撞检测当前被注释掉（性能考虑）
- `path_assessment_decider_util.cc:L32-L61`

#### IsGreatlyOffReferenceLine()

- 阈值：`kOffReferenceLineThreshold = 20.0` 米
- 遍历 Frenet 路径点，任一点 |l| > 20m 即判定为严重偏离
- `path_assessment_decider_util.cc:L63-L75`

#### IsGreatlyOffRoad()

- 阈值：`kOffRoadThreshold = 10.0` 米
- 检查路径点是否超出道路边界 + 10m 容差
- `path_assessment_decider_util.cc:L77-L95`

#### IsCollidingWithStaticObstacles()

```cpp
bool PathAssessmentDeciderUtil::IsCollidingWithStaticObstacles(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // 过滤小面积障碍物 (< kMinObstacleArea)
  // 对每个路径点构建车辆 BoundingBox
  // 检查车辆四角是否在障碍物多边形内
}
```

- 使用车辆四角点与障碍物多边形的 `IsPointIn` 检测碰撞
- 跳过路径尾部点（距离末端 < 车长 + 额外缓冲）
- `path_assessment_decider_util.cc:L97-L166`

#### IsStopOnReverseNeighborLane()

- 仅对借道路径（label 含 "left"/"right"）生效
- 检查停车点是否落在对向车道上
- 如果停车点的 l 偏移超出本车道宽度，判定为不合法
- `path_assessment_decider_util.cc:L168-L199`

## 3. PathOptimizerUtil — 路径优化工具

> 源码：`path_optimizer_util.h` / `path_optimizer_util.cc`

### 类声明

```cpp
class PathOptimizerUtil {
 public:
  static FrenetFramePath ToPiecewiseJerkPath(const vector<double>& x,
      const vector<double>& dx, const vector<double>& ddx, double delta_s, double start_s);
  static double EstimateJerkBoundary(double vehicle_speed);
  static vector<PathPoint> ConvertPathPointRefFromFrontAxeToRearAxe(const PathData&);
  static void FormulateExtraConstraints(PathBound, const PathBoundary&, ObsCornerConstraints*);
  static bool OptimizePath(const SLState& init_state, const array<double,3>& end_state,
      vector<double> l_ref, vector<double> l_ref_weight, const PathBoundary&,
      const vector<pair<double,double>>& ddl_bounds, double dddl_bound,
      const PiecewiseJerkPathConfig&, vector<double>* x, vector<double>* dx, vector<double>* ddx);
  static bool OptimizePathWithTowingPoints(...);
  static void UpdatePathRefWithBound(const PathBoundary&, double weight, vector<double>* ref_l, vector<double>* weight_ref_l);
  static void UpdatePathRefWithBoundInSidePassDirection(...);
  static void CalculateAccBound(const PathBoundary&, const ReferenceLine&, vector<pair<double,double>>* ddl_bounds);
};
```

### 核心函数

#### ToPiecewiseJerkPath()

```cpp
FrenetFramePath PathOptimizerUtil::ToPiecewiseJerkPath(
    const vector<double>& x, const vector<double>& dx,
    const vector<double>& ddx, const double delta_s, const double start_s) {
  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(), ddx.front());
  for (size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i-1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }
  // 按 FLAGS_trajectory_space_resolution 采样输出 FrenetFramePath
}
```

- 将 QP 优化结果 (l, dl, ddl) 转换为连续的 `FrenetFramePath`
- 通过差分计算 dddl，构建分段 jerk 轨迹
- 按空间分辨率重采样输出
- `path_optimizer_util.cc:L32-L66`

#### EstimateJerkBoundary()

```cpp
double PathOptimizerUtil::EstimateJerkBoundary(const double vehicle_speed) {
  const double axis_distance = veh_param.wheel_base();
  const double max_yaw_rate = max_steer_angle_rate / steer_ratio;
  return max_yaw_rate / axis_distance / vehicle_speed;
}
```

- 基于车辆运动学模型估算横向 jerk 上界
- 公式：`dddl_max = (max_steer_angle_rate / steer_ratio) / wheel_base / v`
- `path_optimizer_util.cc:L68-L75`

#### OptimizePath()

```cpp
bool PathOptimizerUtil::OptimizePath(
    const SLState& init_state, const array<double,3>& end_state,
    vector<double> l_ref, vector<double> l_ref_weight,
    const PathBoundary& path_boundary,
    const vector<pair<double,double>>& ddl_bounds, double dddl_bound,
    const PiecewiseJerkPathConfig& config,
    vector<double>* x, vector<double>* dx, vector<double>* ddx) {
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s, init_state.second);
  piecewise_jerk_problem.set_end_state_ref(end_state_weight, end_state);
  piecewise_jerk_problem.set_x_ref(l_ref_weight, l_ref);
  piecewise_jerk_problem.set_weight_x(config.l_weight());
  piecewise_jerk_problem.set_weight_dx(config.dl_weight());
  piecewise_jerk_problem.set_weight_ddx(config.ddl_weight());
  piecewise_jerk_problem.set_weight_dddx(config.dddl_weight());
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(-lateral_derivative_bound, lateral_derivative_bound);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
  piecewise_jerk_problem.set_dddx_bound(dddl_bound);
  return piecewise_jerk_problem.Optimize(config.max_iteration());
}
```

- 路径优化的核心入口，封装 `PiecewiseJerkPathProblem` QP 求解器
- 优化变量：l（横向偏移）、dl（横向偏移对 s 的导数）、ddl（横向加速度）
- 约束：横向边界、横向速度上限、横向加速度上限、jerk 上限
- 目标函数权重来自 `PiecewiseJerkPathConfig` proto 配置
- scale_factor `{1.0, 10.0, 100.0}` 用于数值稳定性
- 支持额外约束：ADC 顶点约束、障碍物角点约束
- `path_optimizer_util.cc:L96-L210`

#### OptimizePathWithTowingPoints()

- 与 `OptimizePath` 类似，额外支持 towing（牵引）参考点
- 用于需要跟随前方引导点的场景（如泊车引导线）
- 通过 `set_towing_x_ref` 设置牵引参考线及其权重
- `path_optimizer_util.cc:L212-L328`

#### UpdatePathRefWithBound()

```cpp
void PathOptimizerUtil::UpdatePathRefWithBound(
    const PathBoundary& path_boundary, double weight,
    vector<double>* ref_l, vector<double>* weight_ref_l) {
  for (size_t i = 0; i < ref_l->size(); i++) {
    bool is_need_update = (boundary has OBSTACLE type) &&
                          (towing_l is near or outside boundary);
    if (is_need_update) {
      ref_l->at(i) = (l_lower + l_upper) / 2.0;  // 居中
      weight_ref_l->at(i) = weight;
    } else {
      weight_ref_l->at(i) = 0;
    }
  }
}
```

- 当参考线被障碍物边界挤压时，将参考点更新为边界中心
- 仅在障碍物边界点处激活权重，其余点权重为 0
- `path_optimizer_util.cc:L330-L377`

#### CalculateAccBound()

```cpp
void PathOptimizerUtil::CalculateAccBound(
    const PathBoundary& path_boundary, const ReferenceLine& reference_line,
    vector<pair<double,double>>* ddl_bounds) {
  const double lat_acc_bound = tan(max_steer_angle / steer_ratio) / wheel_base;
  for (size_t i = 0; i < path_boundary_size; ++i) {
    double kappa = reference_line.GetNearestReferencePoint(s).kappa();
    ddl_bounds->emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
  }
}
```

- 基于车辆运动学计算每个 s 位置的横向加速度上下界
- 考虑参考线曲率 κ 的影响：`ddl ∈ [-lat_acc_max - κ, lat_acc_max - κ]`
- `path_optimizer_util.cc:L407-L422`

## 调用关系

```
PathGeneration (所有路径生成任务的基类)
├── DecidePathBounds()
│   ├── PathBoundsDeciderUtil::GetStartPoint()
│   ├── PathBoundsDeciderUtil::InitPathBoundary()
│   ├── PathBoundsDeciderUtil::GetBoundaryFromLanesAndADC()
│   ├── PathBoundsDeciderUtil::GetSLPolygons()
│   └── PathBoundsDeciderUtil::UpdatePathBoundaryBySLPolygon()
├── OptimizePath()
│   ├── PathOptimizerUtil::CalculateAccBound()
│   ├── PathOptimizerUtil::UpdatePathRefWithBound()
│   ├── PathOptimizerUtil::OptimizePath()
│   └── PathOptimizerUtil::ToPiecewiseJerkPath()
└── AssessPath()
    ├── PathAssessmentDeciderUtil::IsValidRegularPath()
    └── PathAssessmentDeciderUtil::TrimTailingOutLanePoints()
```
