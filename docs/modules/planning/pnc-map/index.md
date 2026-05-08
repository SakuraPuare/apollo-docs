---
title: "PNC Map"
---

# PNC Map

> 源码路径: `modules/planning/pnc_map/`

## 概述

PNC Map（Planning and Control Map）是规划模块与高精地图之间的桥梁层。它负责将 routing 模块下发的路线指令（`PlanningCommand`）转换为规划模块可直接使用的车道级路段（`RouteSegments`），并持续跟踪自车在路由中的位置、管理可行驶车道范围、计算到终点的距离。

PNC Map 采用插件架构，通过 `PncMapBase` 基类定义统一接口，当前唯一的具体实现为 `LaneFollowMap`，专门处理沿车道行驶（`lane_follow_command`）类型的指令。

## 核心类

### LaneFollowMap

继承自 `PncMapBase`，是 PNC Map 的核心实现类。它基于 HD Map 进行路线跟踪和路段管理。

```cpp
class LaneFollowMap : public PncMapBase {
 public:
  LaneFollowMap();
  virtual ~LaneFollowMap() = default;

  bool CanProcess(const planning::PlanningCommand &command) const override;
  bool UpdatePlanningCommand(const planning::PlanningCommand &command) override;
  bool GetRouteSegments(
      const common::VehicleState &vehicle_state,
      std::list<apollo::hdmap::RouteSegments> *const route_segments) override;
  bool ExtendSegments(
      const apollo::hdmap::RouteSegments &segments, double start_s,
      double end_s,
      apollo::hdmap::RouteSegments *const truncated_segments) const override;
  // ...
};
```

通过插件注册宏绑定到 `PncMapBase`：

```cpp
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneFollowMap, PncMapBase)
```

### RouteIndex（内部结构）

用于将 routing 响应中的三维索引 `{road_index, passage_index, lane_index}` 与对应的车道段（`LaneSegment`）关联起来。

```cpp
struct RouteIndex {
  apollo::hdmap::LaneSegment segment;
  std::array<int, 3> index;  // {road_index, passage_index, lane_index}
};
```

### WaypointIndex（内部结构）

用于记录 routing 请求中每个路径点在 `route_indices_` 中对应的索引位置。

```cpp
struct WaypointIndex {
  apollo::hdmap::LaneWaypoint waypoint;
  int index;
};
```

## 核心函数

### GetRouteSegments（public，获取可行驶路段）

**职责**：根据自车状态生成当前可行驶的路由路段列表，是规划模块获取参考线的入口。

**输入**：

- `vehicle_state`：自车当前位置、速度、航向等状态
- `route_segments`（输出）：生成的可行驶路段列表

**输出**：`bool`，是否成功生成路段

**关键步骤**：

1. 调用 `UpdateVehicleState` 更新自车在路由上的位置
2. 根据车速计算前瞻/后视距离（`LookForwardDistance`、`look_backward_distance`）
3. 通过 `GetNeighborPassages` 获取当前道路的所有可达车道（包括换道目标车道）
4. 对每个可达车道执行投影，裁剪并延展到所需 s 范围
5. 设置路段属性（是否可出口、下一步动作、是否为当前所在车道等）

### UpdatePlanningCommand（public，更新路由指令）

**职责**：解析新的 `PlanningCommand`，重建路由索引结构。

**输入**：`command` — 来自 routing 模块的规划指令

**输出**：`bool`，是否更新成功

**关键步骤**：

1. 遍历 `road -> passage -> lane_segment` 三层结构，构建 `route_indices_`
2. 收集所有 `all_lane_ids_` 用于后续最近点搜索
3. 识别终点车道段 `dest_lane_segment_`
4. 构建 `routing_waypoint_index_`，将请求路径点映射到路由索引

### UpdateVehicleState（private，更新自车状态）

**职责**：将自车物理状态映射到路由坐标系，更新内部跟踪变量。

**输入**：`vehicle_state` — 自车状态

**关键步骤**：

1. 检测位置是否发生大幅跳变（超过重规划阈值），若是则重置索引
2. 调用 `GetNearestPointFromRouting` 将自车投影到路由车道上，得到 `adc_waypoint_`
3. 通过 `GetWaypointIndex` 确定自车在 `route_indices_` 中的位置
4. 调用 `UpdateNextRoutingWaypointIndex` 更新下一个路径点索引
5. 调用 `UpdateRoutingRange` 更新当前有效车道 ID 范围

### GetNearestPointFromRouting（private，获取最近路由点）

**职责**：在路由涉及的所有车道中，找到距离自车最近且航向兼容的车道投影点。

**输入**：`state` — 自车状态；`waypoint`（输出）— 最近点

**关键步骤**：

1. 仅在 `range_lane_ids_` 范围内的车道中搜索
2. 对每条候选车道执行 2D 投影，过滤超出车道长度或航向偏差大于 135 度的结果
3. 选取横向偏差 `|l|` 最小的投影点作为结果

### ExtendSegments（private，延展路段）

**职责**：将路由路段向前/后延展，确保参考线覆盖规划所需的视野范围。

**输入**：`segments`（原始路段）、`start_s` / `end_s`（目标 s 范围）、`truncated_segments`（输出）

**关键步骤**：

1. 若 `start_s < 0`，沿 `GetRoutePredecessor` 向前回溯拼接车道
2. 裁剪原始路段到 `[start_s, end_s]` 范围
3. 若接近终点且 `can_exit`，额外延长 `FLAGS_reference_line_endpoint_extend_length`
4. 沿 `GetRouteSuccessor` 向后延伸，直到满足 `end_s` 或遇到环路

### GetNeighborPassages（private，获取相邻车道通道）

**职责**：在同一条道路上查找与当前车道通道相邻的可换道目标通道。

**输入**：`road`（道路段）、`start_passage`（当前通道索引）

**输出**：可达通道索引列表，始终包含自身

**关键步骤**：

1. 若 `change_lane_type` 为 `FORWARD` 或通道可出口，直接返回自身
2. 根据换道方向（`LEFT`/`RIGHT`）收集邻接车道 ID
3. 遍历同道路其他通道，若其包含邻接车道则加入结果

### GetDistanceToDestination（public，获取到终点距离）

**职责**：计算自车当前位置沿路由到目的地的剩余距离。

**关键步骤**：

1. 从 `adc_route_index_` 出发，沿 `can_exit` 的通道累加车道段长度
2. 首段减去自车当前 `adc_waypoint_.s` 的偏移

## 配置

| 配置项（GFlags） | 用途 | 说明 |
|---|---|---|
| `FLAGS_replan_lateral_distance_threshold` | 重规划横向距离阈值 | 自车位置跳变超过此值时重置路由索引 |
| `FLAGS_replan_longitudinal_distance_threshold` | 重规划纵向距离阈值 | 与横向阈值联合判断是否需要重置 |
| `FLAGS_look_backward_distance` | 后视距离 | 获取路段时向后延伸的默认距离 |
| `FLAGS_reference_line_endpoint_extend_length` | 终点延展长度 | 接近目的地时额外延展参考线的长度 |

## 调用关系

```text
PlanningComponent
  └─ PncMapBase (接口)
       └─ LaneFollowMap (实现，插件注册)
            ├─ UpdatePlanningCommand()  ← 收到新 routing 时调用
            │    └─ 构建 route_indices_ / routing_waypoint_index_
            ├─ GetRouteSegments()       ← 每帧规划时调用
            │    ├─ UpdateVehicleState()
            │    │    ├─ GetNearestPointFromRouting()
            │    │    ├─ GetWaypointIndex()
            │    │    │    ├─ SearchForwardWaypointIndex()
            │    │    │    └─ SearchBackwardWaypointIndex()
            │    │    ├─ UpdateNextRoutingWaypointIndex()
            │    │    └─ UpdateRoutingRange()
            │    ├─ GetNeighborPassages()
            │    │    └─ PassageToSegments()
            │    ├─ ExtendSegments()
            │    │    ├─ GetRoutePredecessor()
            │    │    └─ GetRouteSuccessor()
            │    └─ UpdateRouteSegmentsLaneIds()
            ├─ GetDistanceToDestination()
            ├─ FutureRouteWaypoints()
            └─ GetEndLaneWayPoint()
```

`GetRouteSegments` 返回的 `RouteSegments` 列表会被下游的参考线生成器（`ReferenceLineProvider`）进一步处理为平滑的参考线，供轨迹规划使用。
