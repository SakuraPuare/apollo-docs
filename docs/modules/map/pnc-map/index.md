---
title: "PnC Map 规划控制地图"
---

# PnC Map

> 源码路径：`modules/map/pnc_map/`

## 概述

PnC Map（Planning and Control Map）是连接 HDMap 和 Planning 模块的桥梁层。它根据 Routing 结果和车辆当前状态，从高精地图中提取规划所需的路段信息（RouteSegments），为 Planning 模块生成参考线提供数据基础。核心功能包括：路由段提取、车辆投影、前后视距计算、换道动作判断。

## 核心类

### PncMapBase

```cpp
// pnc_map_base.h
class PncMapBase {
 public:
  virtual bool CanProcess(const PlanningCommand& command) const = 0;
  virtual bool UpdatePlanningCommand(const PlanningCommand& command);
  static double LookForwardDistance(double velocity);
  virtual bool GetRouteSegments(
      const common::VehicleState& vehicle_state,
      std::list<RouteSegments>* route_segments) = 0;
  bool IsNewPlanningCommand(const PlanningCommand& command) const;
  virtual bool ExtendSegments(const RouteSegments& segments,
      double start_s, double end_s,
      RouteSegments* truncated_segments) const = 0;
  virtual std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const = 0;
  virtual void GetEndLaneWayPoint(
      std::shared_ptr<routing::LaneWaypoint>& end_point) const = 0;
  virtual hdmap::LaneInfoConstPtr GetLaneById(const hdmap::Id& id) const = 0;
  virtual double GetDistanceToDestination() const = 0;
 protected:
  PlanningCommand last_command_;
  hdmap::LaneWaypoint adc_waypoint_;
};
```

**职责**：规划地图抽象基类，定义路由段获取和命令处理接口

### RouteSegments

```cpp
// route_segments.h
class RouteSegments : public std::vector<LaneSegment> {
 public:
  routing::ChangeLaneType NextAction() const;
  routing::ChangeLaneType PreviousAction() const;
  bool CanExit() const;
  bool GetProjection(const common::PointENU& point_enu,
                     common::SLPoint* sl_point, LaneWaypoint* waypoint) const;
  bool GetProjection(const common::math::Vec2d& point,
                     common::SLPoint* sl_point, LaneWaypoint* waypoint) const;
  bool GetWaypoint(double s, LaneWaypoint* waypoint) const;
  bool CanDriveFrom(const LaneWaypoint& waypoint) const;
  bool Stitch(const RouteSegments& other);
  bool Shrink(const common::math::Vec2d& point,
              double look_backward, double look_forward);
  bool IsOnSegment() const;
  bool IsNeighborSegment() const;
  LaneWaypoint FirstWaypoint() const;
  LaneWaypoint LastWaypoint() const;
  static double Length(const RouteSegments& segments);
};
```

**职责**：表示 Routing 中一个 Passage 对应的车道段集合，是生成参考线的数据源

## 核心函数

### PncMapBase::GetRouteSegments()

**职责**：根据车辆状态提取当前及相邻车道的路由段
**输入**：`VehicleState`（位置、速度、航向）
**输出**：`list<RouteSegments>`（当前段 + 邻居段）
**关键逻辑**：根据速度计算前视距离，裁剪路由段

### PncMapBase::LookForwardDistance()

```cpp
static double LookForwardDistance(const double velocity) {
  // 根据速度在 short_distance 和 long_distance 之间插值
}
```

**职责**：根据车速计算前视距离（速度越快看得越远）

### RouteSegments::GetProjection()

**职责**：将地图点投影到路由段上，返回 SL 坐标和对应 LaneWaypoint
**输入**：ENU 坐标点或 Vec2d 点
**输出**：`SLPoint`（纵向 s、横向 l）+ `LaneWaypoint`

### RouteSegments::Stitch()

**职责**：将两个路由段拼接为一个连续段
**场景**：参考线更新时，将旧段和新段拼接保持连续性

### RouteSegments::Shrink()

**职责**：根据前后视距裁剪路由段
**输入**：参考点、前视距离、后视距离

## 配置（GFlags）

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `look_backward_distance` | double | 后视距离 |
| `look_forward_short_distance` | double | 短前视距离（低速） |
| `look_forward_long_distance` | double | 长前视距离（高速） |

## 调用关系

- **调用者**：Planning 模块的 ReferenceLineProvider 通过 PncMapBase 获取 RouteSegments
- **依赖**：HDMap（车道信息）、Routing 结果（PlanningCommand）
- **下游**：RouteSegments 用于生成 ReferenceLine
