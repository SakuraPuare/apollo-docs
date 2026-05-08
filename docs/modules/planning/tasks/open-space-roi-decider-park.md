---
title: "泊车开放空间 ROI 决策器"
---

# OpenSpaceRoiDeciderPark

> 源码路径: `modules/planning/tasks/open_space_roi_decider_park/`

## 概述

OpenSpaceRoiDeciderPark 是 Apollo 规划模块中开放空间场景的 ROI (Region of Interest) 决策器，继承自 `Decider` 基类。该模块负责为开放空间路径规划器生成感兴趣区域边界，支持泊车入库 (`PARKING`)、靠边停车 (`PULL_OVER`)、即停即走 (`PARK_AND_GO`) 和大曲率掉头 (`LARGE_CURVATURE`) 四种场景。

核心工作流程为：根据配置的 ROI 类型确定目标泊位/靠边点 -> 设置坐标原点进行归一化 -> 计算终点位姿 -> 生成道路边界 -> 将边界约束转化为线性不等式表示 (Ax > b)。

## 核心类

### ParkingInfo

```cpp
struct ParkingInfo {
  ParkingType parking_type;
  std::string parking_id;
  std::vector<common::math::Vec2d> corner_points;  // left_top, right_top, right_down, left_down
  common::math::Vec2d center_point;
  bool is_on_left = false;
};
```

描述目标泊车位信息，包含泊位类型（垂直/平行）、角点坐标、中心点以及相对于车道的位置。

### OpenSpaceRoiDeciderPark

```cpp
class OpenSpaceRoiDeciderPark : public Decider { ... };
```

通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册为 Task 插件。主要成员变量：

- `target_parking_spot_id_` — 来自 routing 的目标泊位 ID
- `nearby_path_` — 泊位附近参考路径
- `hdmap_` — 高精地图指针
- `vehicle_params_` — 车辆参数
- `config_` — ROI 决策器配置

## 核心函数

### Init

```cpp
bool Init(const std::string &config_dir, const std::string &name,
          const std::shared_ptr<DependencyInjector> &injector) override;
```

初始化高精地图指针、车辆参数，并从配置文件加载 `OpenSpaceRoiDeciderParkConfig`。

### Process

```cpp
apollo::common::Status Process(Frame *frame) override;
```

主处理流程，根据 `config_.roi_type()` 分发到不同场景：

1. **PARKING** — 调用 `GetParkingSpot` 获取泊位信息，`SetOrigin` 设置坐标原点，`SetParkingSpotEndPose` 计算终点位姿，`GetParkingBoundary` 生成边界
2. **PULL_OVER** — 调用 `GetPullOverSpot` 获取靠边停车区域，`SetOrigin` / `SetPullOverSpotEndPose` / `GetPullOverBoundary`
3. **PARK_AND_GO** — 从参考线获取路径，区分车辆在泊位内 (`GetParkingOutBoundary`) 和不在泊位内 (`GetParkAndGoBoundary`) 两种情况
4. **LARGE_CURVATURE** — 调用 `GetUTurnPath` 获取掉头路径，`GetLargeCurvatureBoundary` 生成边界

所有场景最终都调用 `FormulateBoundaryConstraints` 将边界转化为优化约束。

### GetParkingSpot

```cpp
bool GetParkingSpot(Frame *const frame, ParkingInfo *parking_info);
```

根据 `target_parking_spot_id_` 从高精地图查询泊位，确定泊位类型（垂直泊车 `VERTICAL_PARKING` 或平行泊车 `PARALLEL_PARKING`），通过比较边长和航向角差异两种方式判断。

### SetOrigin / SetOriginFromADC

```cpp
void SetOrigin(const ParkingInfo &parking_info, Frame *const frame);
void SetOrigin(Frame *const frame, const std::array<Vec2d, 4> &vertices);
void SetOriginFromADC(Frame *const frame, const hdmap::Path &nearby_path);
```

设置坐标归一化的原点和航向角，写入 `frame->open_space_info()` 的 `origin_point` 和 `origin_heading`。泊车场景以车位左上角为原点，即停即走和大曲率场景以自车位置为原点。

### GetRoadBoundary / GetRoadBoundaryFromMap

```cpp
void GetRoadBoundary(const hdmap::Path &nearby_path, double center_line_s,
                     const Vec2d &origin_point, double origin_heading,
                     std::vector<Vec2d> *left_lane_boundary,
                     std::vector<Vec2d> *right_lane_boundary, ...);
```

沿参考线采样生成左右车道边界点。`GetRoadBoundary` 通过路径宽度和曲率自适应采样，`GetRoadBoundaryFromMap` 从高精地图直接获取道路边界。两种方式均支持通过 `use_road_boundary_from_map` 配置项切换。

### AddBoundaryKeyPoint

```cpp
void AddBoundaryKeyPoint(const hdmap::Path &nearby_path, double check_point_s,
                         double start_s, double end_s, bool is_anchor_point,
                         bool is_left_curb, ...);
```

检查当前采样点是否为边界关键点。关键点包括两类：车道锚点（起止点或曲率突变处）和路缘拐角点（路缘宽度二阶导数超过阈值）。

### GetParkingBoundary

```cpp
bool GetParkingBoundary(const ParkingInfo &parking_info,
                        const hdmap::Path &nearby_path, Frame *const frame,
                        std::vector<std::vector<Vec2d>> *roi_parking_boundary,
                        std::vector<std::vector<Vec2d>> *parking_soft_boundary);
```

泊车场景专用边界生成。根据车位在车道左侧还是右侧（由 `average_l` 判断），将车道边界与泊位边界拼接为闭合多边形，然后调用 `FuseLineSegments` 融合线段为凸约束。

### GetParkAndGoBoundary / GetParkingOutBoundary

即停即走场景边界生成。`GetParkAndGoBoundary` 以车辆初始位置的包围盒为基准获取道路边界；`GetParkingOutBoundary` 处理车辆在泊位内需要驶出的情况，先检测车辆所在的泊位多边形，再拼接道路边界。

### FuseLineSegments

```cpp
bool FuseLineSegments(std::vector<std::vector<Vec2d>> *line_segments_vec);
```

将共线或端点重合的相邻线段融合，减少冗余边界段。通过叉积判断三点是否共线，共线则合并。

### FormulateBoundaryConstraints

```cpp
bool FormulateBoundaryConstraints(
    const std::vector<std::vector<Vec2d>> &roi_parking_boundary, Frame *const frame);
```

依次调用 `LoadObstacleInVertices` 和 `LoadObstacleInHyperPlanes`，将边界和感知障碍物的顶点表示转化为线性不等式约束 (A, b)。

### LoadObstacleInVertices

将 ROI 边界段和感知障碍物（可选，由 `enable_perception_obstacles` 控制）的顶点加载到 `obstacles_vertices_vec`。障碍物通过 `FilterOutObstacle` 过滤虚拟障碍物、动态障碍物和超出 ROI 范围的障碍物。

### GetHyperPlanes

```cpp
bool GetHyperPlanes(const size_t &obstacles_num,
                    const Eigen::MatrixXi &obstacles_edges_num,
                    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
                    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);
```

将每个障碍物的多边形顶点转化为半平面表示 Ax > b。逐边计算超平面方程，处理水平/垂直/一般斜率三种情况。

### SetParkingSpotEndPose / SetPullOverSpotEndPose / SetParkAndGoEndPose / GetLargeCurvatureEndPose

各类场景的终点位姿设置函数。终点位姿以 `(x, y, heading, velocity)` 格式存入 `open_space_end_pose`。泊车终点根据垂直/平行泊车类型和 `parking_inwards` 配置确定位置和朝向；即停即走终点取参考线上距离自车 `end_pose_s_distance` 处的点。

## 配置

配置文件路径: `conf/default_conf.pb.txt`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `roi_type` | RoiType 枚举 | - | ROI 场景类型 |
| `roi_longitudinal_range_start` | double | 10.0 | ROI 纵向起始范围 (m) |
| `roi_longitudinal_range_end` | double | 10.0 | ROI 纵向结束范围 (m) |
| `parking_start_range` | double | 7.0 | 泊位检测范围阈值 (m) |
| `parking_inwards` | bool | false | 垂直泊车时是否车头朝内 |
| `enable_perception_obstacles` | bool | - | 是否考虑感知障碍物 |
| `parking_depth_buffer` | double | 0.1 | 车辆边缘到泊位终点线的缓冲距离 (m) |
| `roi_line_segment_min_angle` | double | 0.3 | 边界线段最小转角 (rad) |
| `roi_line_segment_length` | double | 1.0 | 边界线段采样长度 (m) |
| `roi_line_segment_length_from_map` | double | 10.0 | 地图边界采样长度 (m) |
| `perception_obstacle_filtering_distance` | double | 1000.0 | 障碍物过滤距离 (m) |
| `perception_obstacle_buffer` | double | 0.0 | 感知障碍物扩展缓冲 (m) |
| `curb_heading_tangent_change_upper_limit` | double | 1.0 | 路缘宽度变化容忍阈值 |
| `end_pose_s_distance` | double | 10.0 | 即停即走终点 s 距离 (m) |
| `use_road_boundary_from_map` | bool | false | 是否从地图获取道路边界 |
| `expand_polygon_of_obstacle_by_distance` | bool | false | 是否按距离扩展障碍物多边形 |

RoiType 枚举值: `NOT_DEFINED(0)`, `PARKING(1)`, `PULL_OVER(2)`, `PARK_AND_GO(3)`, `NARROW_STREET_U_TURN(4)`, `DEAD_END(5)`, `LARGE_CURVATURE(6)`。

## 调用关系

```text
Frame
  └── OpenSpaceRoiDeciderPark::Process()
        ├── GetParkingSpot() / GetPullOverSpot() / GetUTurnPath()
        │     └── GetNearbyPath() → GetAllLaneSegments()
        ├── SetOrigin() / SetOriginFromADC()
        ├── SetParkingSpotEndPose() / SetPullOverSpotEndPose()
        │     / SetParkAndGoEndPose() / GetLargeCurvatureEndPose()
        ├── GetParkingBoundary() / GetPullOverBoundary()
        │     / GetParkAndGoBoundary() / GetParkingOutBoundary()
        │     / GetLargeCurvatureBoundary()
        │     ├── GetRoadBoundary() → AddBoundaryKeyPoint()
        │     ├── GetRoadBoundaryFromMap()
        │     └── FuseLineSegments()
        └── FormulateBoundaryConstraints()
              ├── LoadObstacleInVertices() → FilterOutObstacle()
              └── LoadObstacleInHyperPlanes() → GetHyperPlanes()
```
