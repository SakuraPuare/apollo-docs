# Reference Line 参考线模块函数级源码解析

本文聚焦 `modules/planning/planning_base/reference_line/` 目录，按函数级粒度拆解参考线的核心实现：`ReferencePoint`、`ReferenceLine`、`ReferenceLineProvider` 以及三套参考线平滑器（`QpSplineReferenceLineSmoother`、`SpiralReferenceLineSmoother`、`DiscretePointsReferenceLineSmoother`）。

## 1. 模块定位

参考线是 Apollo 规划模块的**空间基准坐标系**。它将高精地图的车道中心线转化为可插值、可查询的连续曲线，为后续的路径规划、速度规划、决策模块统一提供 Frenet 坐标变换与几何查询能力。

核心职责：

- 从 `PncMap`（基于路由的导航地图）获取原始路径点，构造 `ReferenceLine`
- 对原始参考线做平滑处理（QP Spline / Spiral / Discrete Points 三种策略）
- 提供 XY ↔ SL 坐标变换、参考点插值、车道宽度/限速/边界查询
- 支持参考线的拼接（Stitch）、裁剪（Segment）、延伸（Extend）
- 在独立线程中周期性刷新参考线（默认 50 ms），保证规划模块能获取最新参考线

上下游拓扑：

```
Routing ──> PncMap ──> ReferenceLineProvider ──> ReferenceLine ──> Planning
                                                       │
                                               Smoother (3种策略)
```

## 2. 目录结构

```
modules/planning/planning_base/reference_line/
├── reference_point.h / .cc                     # 参考点（带曲率信息的地图点）
├── reference_line.h / .cc                      # 参考线核心类
├── reference_line_provider.h / .cc             # 参考线提供者（生产者-消费者模型）
├── reference_line_smoother.h                   # 平滑器抽象基类
├── qp_spline_reference_line_smoother.h / .cc   # QP 样条平滑器
├── spiral_reference_line_smoother.h / .cc      # 螺旋线平滑器
├── discrete_points_reference_line_smoother.h / .cc  # 离散点平滑器
├── spiral_problem_interface.h / .cc            # 螺旋线 IPOPT 优化接口
├── spiral_smoother_util.cc                     # 螺旋线工具函数
├── smoother_util.cc                            # 平滑器通用工具
├── BUILD                                       # Bazel 构建规则
└── *_test.cc                                   # 各类单元测试
```

## 3. ReferencePoint 参考点

`ReferencePoint` 继承自 `hdmap::MapPathPoint`，在地图路径点的基础上额外携带曲率 `kappa` 和曲率变化率 `dkappa`。它是参考线上的基本采样单元。

### 3.1 类声明

```cpp
class ReferencePoint : public hdmap::MapPathPoint {
 public:
  ReferencePoint() = default;
  ReferencePoint(const MapPathPoint& map_path_point, double kappa, double dkappa);
  common::PathPoint ToPathPoint(double s) const;
  double kappa() const;
  double dkappa() const;
  std::string DebugString() const;
  static void RemoveDuplicates(std::vector<ReferencePoint>* points);
 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};
```

### 3.2 构造函数 `ReferencePoint`

```cpp
ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,
                               const double kappa, const double dkappa)
    : hdmap::MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa) {}
```

- 将传入的 `MapPathPoint`（包含 x, y, heading, lane_waypoints）拷贝到基类
- 额外保存该点处的曲率 `kappa_` 和曲率变化率 `dkappa_`

### 3.3 `ToPathPoint`

```cpp
common::PathPoint ReferencePoint::ToPathPoint(double s) const {
  return common::util::PointFactory::ToPathPoint(x(), y(), 0.0, s, heading(),
                                                 kappa_, dkappa_);
}
```

- 将 `ReferencePoint` 转换为 Protobuf 消息 `common::PathPoint`
- 参数 `s` 为该点在参考线上的纵向距离
- z 坐标固定为 0.0

### 3.4 `kappa()` / `dkappa()`

简单的 getter，返回曲率和曲率变化率。

### 3.5 `DebugString`

```cpp
std::string ReferencePoint::DebugString() const {
  return absl::StrCat("{x: ", x(), ", y: ", y(), ", theta: ", heading(),
                      ", kappa: ", kappa(), ", dkappa: ", dkappa(), "}");
}
```

### 3.6 `RemoveDuplicates`（静态）

```cpp
void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  CHECK_NOTNULL(points);
  int count = 0;
  for (size_t i = 0; i < points->size(); ++i) {
    auto& last_point = (*points)[count - 1];
    const auto& this_point = (*points)[i];
    if (count == 0 ||
        std::abs(last_point.x() - this_point.x()) > kDuplicatedPointsEpsilon ||
        std::abs(last_point.y() - this_point.y()) > kDuplicatedPointsEpsilon) {
      (*points)[count++] = this_point;
    } else {
      last_point.add_lane_waypoints(this_point.lane_waypoints());
    }
  }
  points->resize(count);
}
```

- **算法**：原地去重，使用曼哈顿距离（分量绝对值差）判断，阈值 `kDuplicatedPointsEpsilon = 1e-7`
- **合并策略**：重合点不丢弃其 `lane_waypoints`，而是合并到保留点上
- **时间复杂度**：O(n)，单次遍历

## 4. ReferenceLine 参考线

`ReferenceLine` 是参考线模块的核心类，管理一组有序的 `ReferencePoint` 和底层的 `hdmap::Path`，提供坐标变换、几何查询、限速管理等功能。

### 4.1 数据成员

```cpp
struct SpeedLimit {
  double start_s = 0.0;
  double end_s = 0.0;
  double speed_limit = 0.0;  // m/s
};

std::vector<SpeedLimit> speed_limit_;          // 覆盖限速段
std::vector<ReferencePoint> reference_points_; // 参考点序列
hdmap::Path map_path_;                         // 底层地图路径（提供累积弧长、平滑点）
uint32_t priority_ = 0;                        // 优先级（多参考线选择时使用）
common::math::Vec2d ego_position_;             // 自车位置（SL 边界计算优化用）
```

### 4.2 构造函数

**模板构造函数（迭代器范围）**

```cpp
template <typename Iterator>
ReferenceLine(const Iterator begin, const Iterator end)
    : reference_points_(begin, end),
      map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}
```

- 从迭代器范围 `[begin, end)` 构造参考点列表
- 同时将参考点隐式转换为 `MapPathPoint` 序列构造 `map_path_`

**从 ReferencePoint 向量构造**

```cpp
ReferenceLine::ReferenceLine(const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points),
      map_path_(std::move(std::vector<hdmap::MapPathPoint>(
          reference_points.begin(), reference_points.end()))) {
  CHECK_EQ(static_cast<size_t>(map_path_.num_points()), reference_points_.size());
}
```

- 将 `ReferencePoint` 向量拷贝为成员
- 同时构造 `hdmap::Path`，该路径会计算累积弧长 `accumulated_s`、单位方向向量 `unit_directions` 等几何信息
- 校验点数一致性

**从 hdmap::Path 构造**

```cpp
ReferenceLine::ReferenceLine(const MapPath& hdmap_path) : map_path_(hdmap_path) {
  for (const auto& point : hdmap_path.path_points()) {
    DCHECK(!point.lane_waypoints().empty());
    const auto& lane_waypoint = point.lane_waypoints()[0];
    reference_points_.emplace_back(
        hdmap::MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0);
  }
  CHECK_EQ(static_cast<size_t>(map_path_.num_points()), reference_points_.size());
}
```

- 从已有的 `hdmap::Path` 构造参考线
- 遍历路径点，取每个点的第一个 `lane_waypoint`，构造 `ReferencePoint`（曲率和曲率变化率初始化为 0）
- 这是 `SmoothRouteSegment` → `SmoothReferenceLine` 路径的入口构造方式

### 4.3 `Stitch` — 参考线拼接

```cpp
bool ReferenceLine::Stitch(const ReferenceLine& other);
```

将另一条参考线 `other` 拼接到当前参考线的首尾。拼接策略是**优先保留当前参考线**，仅补充 `other` 中超出当前范围的部分。

**算法流程**：

1. 将当前参考线的首尾点投影到 `other` 上，得到 SL 坐标
2. 判断 `first_join`（首点在 other 内部）和 `last_join`（尾点在 other 内部）
3. 若两者都不满足，返回 false（参考线不相连）
4. 横向拼接误差阈值 `kStitchingError = 0.1 m`，超过则失败
5. 若 `first_join`：将 `other` 中 `s < first_sl.s()` 的参考点插入当前参考线头部
6. 若 `last_join`：将 `other` 中 `s > last_sl.s()` 的参考点追加到当前参考线尾部
7. 重建 `map_path_`

**使用场景**：`ExtendReferenceLine` 中将延伸段拼接到已有参考线上。

### 4.4 `Segment` — 参考线裁剪

```cpp
bool Segment(const common::math::Vec2d& point, double distance_backward, double distance_forward);
bool Segment(double s, double distance_backward, double distance_forward);
```

- 按纵向距离裁剪参考线，仅保留 `[s - look_backward, s + look_forward]` 范围
- 向量版本先通过 `XYToSL` 将 XY 坐标转换为 s，再调用 s 版本
- 裁剪后保留点数 < 2 则返回失败
- 裁剪后重建 `map_path_`

**使用场景**：`Shrink` 方法中裁剪过长的参考线，减少后续计算量。

### 4.5 参考点查询

#### `GetReferencePoint(double s)` — 按纵向距离插值

```cpp
ReferencePoint ReferenceLine::GetReferencePoint(const double s) const;
```

- 核心查询方法，返回参考线上纵向距离 `s` 处的**精确插值**参考点
- 使用 `map_path_.GetIndexFromS(s)` 获取插值索引 `InterpolatedIndex`
- 取相邻两点 `p0`、`p1`，调用 `InterpolateWithMatchedIndex` 做线性插值
- 插值内容包含 x, y, heading, kappa, dkappa
- 边界处理：s 超出范围时 clamp 到首/尾点

#### `GetReferencePoint(double x, double y)` — 按坐标查询

```cpp
ReferencePoint ReferenceLine::GetReferencePoint(const double x, const double y) const;
```

- 先遍历所有参考点找到最近点 `index_min`
- 取其左右邻居 `[index_min-1, index_min+1]`
- 在该区间内使用 **Brent 最小值搜索**（`FindMinDistancePoint`，8 步迭代）找到精确最近点
- 返回插值后的参考点

#### `GetNearestReferencePoint(Vec2d xy)` — 最近参考点（无插值）

```cpp
ReferencePoint ReferenceLine::GetNearestReferencePoint(const common::math::Vec2d& xy) const;
```

- 遍历所有参考点，返回欧氏距离最近的参考点（不做插值）
- O(n) 复杂度，用于不要求精确插值的快速查询

#### `GetNearestReferencePoint(double s)` — 按 s 查找最近参考点

```cpp
ReferencePoint ReferenceLine::GetNearestReferencePoint(const double s) const;
```

- 使用 `std::lower_bound` 在累积弧长数组中二分查找
- 比较相邻两点与 s 的距离，返回更近的那个
- 不做插值，返回离散参考点

#### `GetNearestReferenceIndex(double s)`

```cpp
size_t ReferenceLine::GetNearestReferenceIndex(const double s) const;
```

- 返回累积弧长中 `>= s` 的第一个索引
- 使用 `std::lower_bound` 二分查找

#### `GetReferencePoints(double start_s, double end_s)` — 范围查询

```cpp
std::vector<ReferencePoint> ReferenceLine::GetReferencePoints(double start_s, double end_s) const;
```

- 返回 `[start_s, end_s]` 范围内的参考点子序列
- 先 clamp 到 `[0, Length()]`，再通过 `GetNearestReferenceIndex` 获取首尾索引

### 4.6 坐标变换

#### `SLToXY` — Frenet → Cartesian

```cpp
bool ReferenceLine::SLToXY(const common::SLPoint& sl_point,
                           common::math::Vec2d* const xy_point) const;
```

- 取纵向距离 `sl_point.s()` 处的参考点
- 沿参考点法向偏移 `sl_point.l()` 得到 XY 坐标
- 公式：`x = ref.x - sin(heading) * l`，`y = ref.y + cos(heading) * l`
- 要求参考线至少有 2 个点

#### `XYToSL` — Cartesian → Frenet（4 个重载）

**基础版本**：

```cpp
bool XYToSL(const common::math::Vec2d& xy_point, common::SLPoint* sl_point,
            double warm_start_s = -1.0) const;
```

- `warm_start_s < 0` 时使用 `map_path_.GetProjection()` 全局搜索最近投影
- `warm_start_s >= 0` 时使用 `map_path_.GetProjectionWithWarmStartS()` 从指定 s 附近搜索，加速计算

**带 heading 版本**：

```cpp
bool XYToSL(const double heading, const common::math::Vec2d& xy_point,
            common::SLPoint* sl_point, double warm_start_s = -1.0) const;
```

- 额外传入 `heading` 用于 `GetProjection(heading, ...)` 以解决多解歧义（如环形匝道）

**带启发式范围版本**：

```cpp
bool XYToSL(const common::math::Vec2d& xy_point, common::SLPoint* sl_point,
            double hueristic_start_s, double hueristic_end_s) const;
```

- 使用 `GetProjectionWithHueristicParams` 在 `[hueristic_start_s, hueristic_end_s]` 范围内搜索
- 用于 `GetSLBoundary` 中已知相邻角点 s 范围时的快速投影

**模板版本**：

```cpp
template <class XYPoint>
bool XYToSL(const XYPoint& xy, common::SLPoint* sl_point) const {
  return XYToSL(common::math::Vec2d(xy.x(), xy.y()), sl_point);
}
```

- 适配任意具有 `x()`, `y()` 方法的类型

#### `GetFrenetPoint` — PathPoint → FrenetFramePoint

```cpp
common::FrenetFramePoint ReferenceLine::GetFrenetPoint(
    const common::PathPoint& path_point) const;
```

- 将 Cartesian 空间的 `PathPoint` 转换为 Frenet 坐标系的 `FrenetFramePoint`
- 计算内容：
  - `s`, `l`：通过 `XYToSL` 获取
  - `dl`：横向偏移一阶导数，通过 `CartesianFrenetConverter::CalculateLateralDerivative` 计算
  - `ddl`：横向偏移二阶导数，通过 `CartesianFrenetConverter::CalculateSecondOrderLateralDerivative` 计算

#### `ToFrenetFrame` — TrajectoryPoint → Frenet 条件

```cpp
std::pair<std::array<double, 3>, std::array<double, 3>>
ReferenceLine::ToFrenetFrame(const common::TrajectoryPoint& traj_point) const;
```

- 将 `TrajectoryPoint`（含位置、速度、加速度、曲率）转换为 Frenet 坐标系
- 返回值：`s_condition = [s, ds, dds]`（纵向位移、速度、加速度），`l_condition = [l, dl, ddl]`（横向偏移、一阶导、二阶导）
- 底层调用 `CartesianFrenetConverter::cartesian_to_frenet`
- 这是规划起始点转换的核心方法，后续路径/速度规划都基于此 Frenet 条件

### 4.7 插值辅助函数

#### `Interpolate`（静态）

```cpp
static ReferencePoint Interpolate(const ReferencePoint& p0, double s0,
                                  const ReferencePoint& p1, double s1, double s);
```

- 在两个参考点 `p0`（位于 s0）、`p1`（位于 s1）之间线性插值
- 插值内容：x, y（线性 lerp）、heading（球面线性 slerp）、kappa, dkappa（线性 lerp）
- 同时处理 `lane_waypoints` 的插值：若 `p0` 和 `p1` 在不同车道上，尝试维护两个车道的 waypoint
- 要求 `s0 <= s <= s1`

#### `InterpolateWithMatchedIndex`

```cpp
ReferencePoint InterpolateWithMatchedIndex(
    const ReferencePoint& p0, double s0, const ReferencePoint& p1, double s1,
    const hdmap::InterpolatedIndex& index) const;
```

- 与 `Interpolate` 类似，但利用 `map_path_` 内置的平滑点（`GetSmoothPoint`）获取更精确的 x, y, heading
- kappa 和 dkappa 仍使用线性插值
- `GetReferencePoint(s)` 的内部实现

#### `FindMinDistancePoint`（静态）

```cpp
static double FindMinDistancePoint(const ReferencePoint& p0, double s0,
                                   const ReferencePoint& p1, double s1,
                                   double x, double y);
```

- 在 `[s0, s1]` 区间内寻找距离点 `(x, y)` 最近的插值点对应的 s 值
- 使用 **Boost Brent 最小值搜索**，8 步精度迭代
- 被 `GetReferencePoint(x, y)` 调用

## 5. SL 边界计算

### 5.1 `GetSLBoundary` — Box2d / Polygon2d → SLBoundary

```cpp
bool GetSLBoundary(const common::math::Box2d& box, SLBoundary* sl_boundary,
                   double warm_start_s = -1.0) const;
bool GetSLBoundary(const common::math::Polygon2d& polygon, SLBoundary* sl_boundary,
                   double warm_start_s = -1.0) const;
```

- Box2d 版本先提取 4 个角点，再调用角点版本
- Polygon2d 版本提取多边形顶点，再调用角点版本

### 5.2 `GetSLBoundary` — 角点版本（核心实现）

```cpp
bool GetSLBoundary(const std::vector<common::math::Vec2d>& corners,
                   SLBoundary* sl_boundary, double warm_start_s) const;
```

**算法**：

1. 找到距离自车最近的角点，将其旋转到序列首位（优化 warm_start 效果）
2. 将首角点投影到参考线（使用 warm_start_s 加速）
3. 依次投影后续角点，利用前一角点的 s 值计算启发式搜索范围 `±2*distance`
4. 对每对相邻角点的中点做额外投影，检查是否在多边形外部（叉积判断），若是则加入边界点
5. 最终取所有边界点的 min/max s/l 填充 `SLBoundary`

**设计要点**：中点检测解决了角点投影可能遗漏参考线穿越多边形的情况。

### 5.3 `GetSLBoundary` — hdmap::Polygon 版本

```cpp
bool GetSLBoundary(const hdmap::Polygon& polygon, SLBoundary* sl_boundary) const;
```

- 遍历多边形的所有点，逐个投影到参考线
- 取 min/max s/l 填充边界
- 不使用启发式搜索，适用于地图区域查询

### 5.4 `GetApproximateSLBoundary` — 快速近似

```cpp
bool GetApproximateSLBoundary(const common::math::Box2d& box,
                              double start_s, double end_s,
                              SLBoundary* sl_boundary) const;
```

- 先将 box 中心投影到参考线
- 将 box 旋转到与参考线对齐
- 直接用旋转后的坐标近似为 SL（x → s, y → l）
- **速度快但精度低**，保证返回的边界 >= 真实边界
- 用于对精度要求不高的场景（如障碍物粗筛）

## 6. 车道/道路信息查询

### 6.1 `GetLaneWidth`

```cpp
bool GetLaneWidth(double s, double* lane_left_width, double* lane_right_width) const;
```

- 委托 `map_path_.GetLaneWidth(s, ...)` 查询纵向距离 s 处的车道左右宽度

### 6.2 `GetOffsetToMap`

```cpp
bool GetOffsetToMap(double s, double* l_offset) const;
```

- 获取参考线在 s 处相对车道中心线的横向偏移量
- 通过最近参考点的第一个 `lane_waypoint.l` 获取

### 6.3 `GetRoadWidth`

```cpp
bool GetRoadWidth(double s, double* road_left_width, double* road_right_width) const;
```

- 查询 s 处的道路左右宽度（比车道宽度更宽，包含路肩等）

### 6.4 `GetRoadType`

```cpp
hdmap::Road::Type GetRoadType(double s) const;
```

- 将参考线上的 s 点转换为 XY，然后查询 HDMap 获取道路类型
- 返回 `HIGHWAY`、`CITY_ROAD`、`UNKNOWN` 等

### 6.5 `GetLaneBoundaryType`

```cpp
void GetLaneBoundaryType(double s,
    hdmap::LaneBoundaryType::Type* left_boundary_type,
    hdmap::LaneBoundaryType::Type* right_boundary_type) const;
```

- 查询 s 处左右车道边界的类型（实线、虚线、路缘石等）

### 6.6 `GetLaneFromS`

```cpp
void GetLaneFromS(double s, std::vector<hdmap::LaneInfoConstPtr>* lanes) const;
```

- 获取 s 处参考点所在的所有车道信息（去重）

### 6.7 `GetDrivingWidth`

```cpp
double GetDrivingWidth(const SLBoundary& sl_boundary) const;
```

- 计算给定 SL 边界在车道内的可用行驶宽度
- 公式：`max(lane_left - end_l, lane_right + start_l)`，再与总宽度取 min

### 6.8 `GetLaneSegments`

```cpp
std::vector<hdmap::LaneSegment> GetLaneSegments(double start_s, double end_s) const;
```

- 返回 `[start_s, end_s]` 范围内的车道段序列

## 7. 位置判断

### 7.1 `IsOnLane` — 是否在车道内

```cpp
bool IsOnLane(const common::SLPoint& sl_point) const;
bool IsOnLane(const common::math::Vec2d& vec2d_point) const;
bool IsOnLane(const SLBoundary& sl_boundary) const;
template <class XYPoint> bool IsOnLane(const XYPoint& xy) const;
```

- SLPoint 版本：`s ∈ (0, length]` 且 `l ∈ [-right_width, left_width]`
- Vec2d 版本：先 XYToSL 再判断
- SLBoundary 版本：取 `middle_s` 处宽度，判断边界与车道宽度的关系
- 模板版本：适配任意 XY 类型

### 7.2 `IsOnRoad` — 是否在道路上

```cpp
bool IsOnRoad(const common::SLPoint& sl_point) const;
bool IsOnRoad(const common::math::Vec2d& vec2d_point) const;
bool IsOnRoad(const SLBoundary& sl_boundary) const;
```

- 与 `IsOnLane` 类似，但使用 `GetRoadWidth` 而非 `GetLaneWidth`
- 道路宽度 > 车道宽度，判断更宽松（包含路肩区域）

### 7.3 `IsBlockRoad` — 是否阻挡路面

```cpp
bool IsBlockRoad(const common::math::Box2d& box2d, double gap) const;
```

- 委托 `map_path_.OverlapWith(box2d, gap)` 判断
- `gap` 参数指定剩余空间阈值

### 7.4 `HasOverlap` — 是否与车道有重叠

```cpp
bool HasOverlap(const common::math::Box2d& box) const;
```

1. 将 box 投影为 SLBoundary
2. 检查 `end_s >= 0` 且 `start_s <= Length()`
3. 检查 `start_l * end_l < 0` 时一定无重叠（box 跨越参考线）
4. 否则取中点处车道宽度，判断 box 是否在车道范围内

## 8. 限速管理

### 8.1 `GetSpeedLimitFromS`

```cpp
double ReferenceLine::GetSpeedLimitFromS(const double s) const;
```

**查找优先级**：

1. **覆盖限速**：遍历 `speed_limit_` 列表，若 s 落在某个 `[start_s, end_s]` 内，直接返回
2. **车道限速**：获取参考点处所有车道的 `speed_limit`，取最小值
3. **默认限速**：若无车道信息，根据道路类型使用 `FLAGS_default_city_road_speed_limit` 或 `FLAGS_default_highway_speed_limit`

### 8.2 `AddSpeedLimit`

```cpp
void ReferenceLine::AddSpeedLimit(double start_s, double end_s, double speed_limit);
```

- 在参考线上叠加新的限速段
- **区间合并算法**：处理新旧限速段的重叠，取重叠区间的较小限速值
- 结果按 `(start_s, end_s, speed_limit)` 排序
- 由交通规则模块（如 `SpeedSetting`、`TrafficLight`）调用

## 9. 其他方法

### 9.1 `Length()`

```cpp
double Length() const { return map_path_.length(); }
```

### 9.2 `DebugString()`

- 输出参考点数量和前 `FLAGS_trajectory_point_num_for_debug` 个参考点的调试信息

### 9.3 `GetPriority()` / `SetPriority`

- 多参考线场景下的优先级管理（数值越小优先级越高）
- 导航模式下由 `path_priority` 字段决定

### 9.4 `SetEgoPosition`

```cpp
void SetEgoPosition(common::math::Vec2d ego_pos) { ego_position_ = ego_pos; }
```

- 设置自车位置，用于 `GetSLBoundary` 中优化角点排序（先处理离自车最近的角点）

## 10. ReferenceLineProvider 参考线提供者

`ReferenceLineProvider` 是参考线的**生产者**，负责从路由和地图创建、平滑、缓存参考线。它支持两种运行模式：

- **独立线程模式**（`FLAGS_enable_reference_line_provider_thread = true`）：在后台线程中 50 ms 周期刷新
- **同步模式**：在 `GetReferenceLines` 调用时同步计算

### 10.1 构造函数

```cpp
ReferenceLineProvider(
    const common::VehicleStateProvider* vehicle_state_provider,
    const ReferenceLineConfig* reference_line_config,
    const std::shared_ptr<relative_map::MapMsg>& relative_map = nullptr);
```

**初始化流程**：

1. 加载平滑器配置文件 `FLAGS_smoother_config_filename`
2. 根据配置创建平滑器实例：`QpSplineReferenceLineSmoother` / `SpiralReferenceLineSmoother` / `DiscretePointsReferenceLineSmoother`
3. 加载 PncMap 插件（默认 `LaneFollowMap`），支持多 PncMap 配置
4. 导航模式下保存 `relative_map` 引用

### 10.2 生命周期方法

#### `Start()`

- 非导航模式下，若 `FLAGS_enable_reference_line_provider_thread` 为 true，启动 `GenerateThread` 异步任务

#### `Stop()`

- 设置 `is_stop_ = true`，等待 `GenerateThread` 退出

#### `Reset()`

- 清空路由信息、参考线缓存、历史队列

### 10.3 `UpdatePlanningCommand`

```cpp
bool UpdatePlanningCommand(const planning::PlanningCommand& command);
```

- 遍历 `pnc_map_list_`，找到能处理该命令的 PncMap
- 若 PncMap 判断是新命令，调用 `UpdatePlanningCommand` 更新路由
- 保存命令并标记 `has_planning_command_`

### 10.4 `GetReferenceLines` — 核心接口

```cpp
bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                       std::list<hdmap::RouteSegments>* segments);
```

**三种路径**：

1. **导航模式**：调用 `GetReferenceLinesFromRelativeMap` 从相对地图获取
2. **独立线程模式**：直接从 `reference_lines_` 缓存读取（加锁）
3. **同步模式**：调用 `CreateReferenceLine` 同步创建

**容错**：若当前参考线为空，尝试使用历史队列（最多保存 3 帧）中的最后一帧。

### 10.5 `CreateReferenceLine` — 参考线创建

```cpp
bool CreateReferenceLine(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);
```

1. 获取当前车辆状态和路由命令
2. 调用 `CreateRouteSegments` 从 PncMap 提取路由段
3. 若 `FLAGS_prioritize_change_lane`，将变道路由段移到列表前端
4. **新命令或禁用拼接**：对每个路由段独立平滑 → 投影自车 → 裁剪
5. **拼接模式**：调用 `ExtendReferenceLine` 尝试延伸已有参考线

### 10.6 `ExtendReferenceLine` — 参考线延伸

```cpp
bool ExtendReferenceLine(const common::VehicleState& state,
                         hdmap::RouteSegments* segments,
                         ReferenceLine* reference_line);
```

**算法**：

1. 在已有参考线中找到与当前路由段相连的前一段
2. 计算剩余前瞻距离 `remain_s`
3. 若 `remain_s > look_forward_required_distance`，直接复用已有参考线
4. 否则调用 `current_pnc_map_->ExtendSegments` 向前延伸路由段
5. 对延伸段做平滑（`SmoothPrefixedReferenceLine`，锚定前段端点）
6. 拼接新旧参考线（`Stitch`）
7. 裁剪（`Shrink`）

### 10.7 `GenerateThread` — 后台刷新线程

```cpp
void ReferenceLineProvider::GenerateThread() {
  while (!is_stop_) {
    cyber::SleepFor(std::chrono::milliseconds(50)); // 50 ms 周期
    // ... CreateReferenceLine → UpdateReferenceLine
  }
}
```

- 每 50 ms 执行一次参考线创建和更新
- 更新后维护历史队列（最多 3 帧）

### 10.8 `GetReferenceLinesFromRelativeMap` — 导航模式

从 `relative_map` 的 `navigation_path` 创建参考线：

1. 获取 ADC 所在车道及左右邻居车道
2. 找到优先级高于当前车道的目标车道
3. 确定变道方向（LEFT/RIGHT）
4. 为每条导航路径创建 `ReferenceLine` 和 `RouteSegments`

### 10.9 `SmoothReferenceLine` / `SmoothPrefixedReferenceLine`

```cpp
bool SmoothReferenceLine(const ReferenceLine& raw, ReferenceLine* smoothed);
bool SmoothPrefixedReferenceLine(const ReferenceLine& prefix_ref,
                                  const ReferenceLine& raw_ref,
                                  ReferenceLine* smoothed);
```

- `SmoothReferenceLine`：生成锚点 → 设置到平滑器 → 执行平滑 → 验证平滑误差
- `SmoothPrefixedReferenceLine`：额外将 `prefix_ref` 中的端点设为强制锚点（`enforced = true`），确保延伸段与前段平滑衔接

### 10.10 `GetAnchorPoint` / `GetAnchorPoints`

```cpp
AnchorPoint GetAnchorPoint(const ReferenceLine& ref, double s) const;
void GetAnchorPoints(const ReferenceLine& ref, std::vector<AnchorPoint>* points) const;
```

- `GetAnchorPoint`：计算 s 处的锚点，包含横向约束宽度
  - 获取车道宽度，扣除车身宽度、路缘石偏移、横向缓冲
  - `lateral_bound` 范围：`[min_lateral_boundary_bound, max_lateral_boundary_bound]`
- `GetAnchorPoints`：均匀采样，首尾锚点强制约束（`enforced = true`, `bound = 1e-6`）

## 11. 平滑器

### 11.1 AnchorPoint 锚点

```cpp
struct AnchorPoint {
  common::PathPoint path_point;    // 锚点位置
  double lateral_bound = 0.0;      // 横向约束宽度
  double longitudinal_bound = 0.0; // 纵向约束宽度
  bool enforced = false;           // 是否强制跟随
};
```

### 11.2 ReferenceLineSmoother 基类

```cpp
class ReferenceLineSmoother {
 public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig& config);
  virtual void SetAnchorPoints(const std::vector<AnchorPoint>& anchor_points) = 0;
  virtual bool Smooth(const ReferenceLine&, ReferenceLine* const) = 0;
  virtual ~ReferenceLineSmoother() = default;
 protected:
  ReferenceLineSmootherConfig config_;
};
```

### 11.3 QpSplineReferenceLineSmoother — QP 样条平滑器

- 使用 **二次规划（QP）** 求解 1D 样条曲线拟合参考线
- 优化目标：最小化样条的二阶导数（平滑性）+ 锚点偏离（保真度）
- 约束：锚点的横向边界、首尾点强制约束
- 底层使用 `Spline1dSolver`（OSQP 求解器）
- 适用于常规道路场景，计算效率和质量平衡

### 11.4 SpiralReferenceLineSmoother — 螺旋线平滑器

- 使用 **Euler 螺旋（Clothoid）** 拟合参考线
- 螺旋线的曲率随弧长线性变化，符合公路设计规范
- 通过 IPOPT 非线性优化器求解
- `SpiralProblemInterface` 实现 IPOPT 的 `TNLP` 接口，定义目标函数和约束
- 适用于高速公路等对曲率连续性要求高的场景

### 11.5 DiscretePointsReferenceLineSmoother — 离散点平滑器

- 直接对离散参考点做平滑
- 使用 **FEM 偏差平滑**（FEM Position Deviation Smoother）
- 通过 OSQP 求解带约束的二次规划问题
- 约束包含锚点边界和点间距
- 适用于复杂城市道路（不规则车道线）

### 11.6 平滑验证

```cpp
bool IsReferenceLineSmoothValid(const ReferenceLine& raw,
                                const ReferenceLine& smoothed) const;
```

- 沿平滑后参考线每 10 m 采样，投影到原始参考线
- 检查横向偏差 `|l|` 是否超过 `FLAGS_smoothed_reference_line_max_diff`
- 超过阈值则认为平滑失败，回退到原始参考线
