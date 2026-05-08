---
title: "开放空间感兴趣区域决策器"
---

# 开放空间感兴趣区域决策器

> 源码路径: `modules/planning/tasks/open_space_roi_decider/`

## 概述

`OpenSpaceRoiDecider` 是开放空间规划中的感兴趣区域（ROI）决策器，负责为开放空间路径规划器构建约束边界。该模块根据不同的场景类型（泊车、靠边停车、即停即走）从高精地图中提取道路边界和车位信息，将世界坐标系下的几何数据转换到以车位左上角为原点的局部坐标系中，并最终将障碍物和边界表示为线性不等式约束 `Ax > b`，供后续优化求解器使用。

## 核心类

### ParkingInfo

停车信息结构体，描述目标车位的几何属性：

```cpp
struct ParkingInfo {
  ParkingType parking_type;                // 停车类型（垂直/平行）
  std::string parking_id;                  // 车位 ID
  std::vector<common::math::Vec2d> corner_points;  // 四角点：左上、右上、右下、左下
  common::math::Vec2d center_point;        // 车位中心点
  bool is_on_left = false;                 // 是否在车道左侧
};
```

### OpenSpaceRoiDecider

继承自 `Decider`，是 ROI 决策的核心类。通过 CyberRT 插件机制注册为 `Task` 插件。

**关键成员变量：**

- `target_parking_spot_id_` -- 来自 routing 的目标车位 ID
- `nearby_path_` -- 车位附近的参考路径
- `hdmap_` -- 高精地图指针
- `vehicle_params_` -- 车辆参数
- `config_` -- `OpenSpaceRoiDeciderConfig` 配置对象
- `is_parking_out` -- 是否处于泊出模式

## 核心函数

### Init

```cpp
bool Init(const std::string &config_dir, const std::string &name,
          const std::shared_ptr<DependencyInjector> &injector) override;
```

初始化决策器：加载高精地图指针、车辆参数和 ROI 配置。

### Process

```cpp
apollo::common::Status Process(Frame *frame) override;
```

主处理入口，根据 `roi_type` 配置分三条路径执行：

1. **PARKING（泊车）** -- 调用 `GetParkingSpot` 获取车位信息，`SetOrigin` 设置局部坐标原点，`SetParkingSpotEndPose` 计算终止位姿，`GetParkingBoundary` 构建边界。
2. **PULL_OVER（靠边停车）** -- 调用 `GetPullOverSpot` 获取靠边停车区域，`SetOrigin` 设置原点，`SetPullOverSpotEndPose` 计算终止位姿，`GetPullOverBoundary` 构建边界。
3. **PARK_AND_GO（即停即走）** -- 判断车辆是否在车位内（`is_parking_out`），分别调用 `GetParkingOutBoundary` 或 `GetParkAndGoBoundary` 构建边界，`SetParkAndGoEndPose` 计算终止位姿。

三条路径最终均调用 `FormulateBoundaryConstraints` 将边界转化为优化约束。

### GetRoadBoundary / GetRoadBoundaryFromMap

```cpp
void GetRoadBoundary(const hdmap::Path &nearby_path, double center_line_s,
                     const common::math::Vec2d &origin_point, double origin_heading,
                     /* 左右车道边界、中心线边界、道路宽度等输出参数 */);
```

沿参考路径按步长采样，提取左右车道边界关键点。`GetRoadBoundary` 通过路径曲率和路缘角变化检测关键点；`GetRoadBoundaryFromMap` 直接从地图的道路边界接口获取。两个函数都将边界点变换到局部坐标系。

### AddBoundaryKeyPoint

```cpp
void AddBoundaryKeyPoint(const hdmap::Path &nearby_path, double check_point_s,
                         double start_s, double end_s, bool is_anchor_point,
                         bool is_left_curb,
                         /* 输出参数 */);
```

对单侧路缘检测关键点。当采样点为锚点（路径起止点或高曲率点）或路缘角突变超过阈值时，将对应路缘点加入边界集合。

### SetOrigin（三个重载）

```cpp
void SetOrigin(const ParkingInfo &parking_info, Frame *const frame);
void SetOrigin(Frame *const frame, const std::array<Vec2d, 4> &vertices);
void SetOriginFromADC(Frame *const frame, const hdmap::Path &nearby_path);
```

设置局部坐标系原点和朝向。以车位左上角（或靠边停车区域左上角、ADC 初始位置左上角）为原点，车头方向为 x 轴正方向，构建归一化坐标系。

### GetParkingBoundary / GetPullOverBoundary / GetParkAndGoBoundary / GetParkingOutBoundary

```cpp
bool GetParkingBoundary(const ParkingInfo &parking_info,
                        const hdmap::Path &nearby_path, Frame *const frame,
                        std::vector<std::vector<Vec2d>> *roi_parking_boundary);
```

为各场景构建 ROI 边界线段集合。流程为：获取车位/停车区域角点 -> 获取道路边界 -> 根据车位在车道左/右侧拼接完整闭合边界 -> 调用 `FuseLineSegments` 融合共线段 -> 计算 XY 包围盒并验证车辆是否在边界内。

### FormulateBoundaryConstraints

```cpp
bool FormulateBoundaryConstraints(
    const std::vector<std::vector<Vec2d>> &roi_parking_boundary,
    Frame *const frame);
```

将边界约束转化为优化问题可用的形式。依次调用 `LoadObstacleInVertices`（加载边界和感知障碍物为顶点表示）和 `LoadObstacleInHyperPlanes`（将顶点转化为超平面不等式 `Ax > b`）。

### LoadObstacleInVertices

```cpp
bool LoadObstacleInVertices(
    const std::vector<std::vector<Vec2d>> &roi_parking_boundary,
    Frame *const frame);
```

将 ROI 边界线段和感知障碍物统一加载为顶点向量。对感知障碍物，根据配置选择按距离扩展多边形或扩展边界框，变换到局部坐标后以顺时针顺序存储。

### LoadObstacleInHyperPlanes / GetHyperPlanes

```cpp
bool GetHyperPlanes(const size_t &obstacles_num,
                    const Eigen::MatrixXi &obstacles_edges_num,
                    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
                    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);
```

将每个障碍物的顶点凸包转化为 H-表示（线性不等式），对每条边计算经过两端点的超平面法向量，输出矩阵 `A_all` 和向量 `b_all`。

### FilterOutObstacle

```cpp
bool FilterOutObstacle(const Frame &frame, const Obstacle &obstacle);
```

过滤无关障碍物。跳过虚拟障碍物、动态障碍物、ROI 包围盒外的障碍物，以及距离车辆和终止位姿均超过 `perception_obstacle_filtering_distance` 的障碍物。

### FuseLineSegments

```cpp
bool FuseLineSegments(std::vector<std::vector<Vec2d>> *line_segments_vec);
```

将相邻共线线段融合为更长的凸线段，减少后续约束数量。通过叉积判断相邻线段是否共线并合并。

### 其他辅助函数

- `SetParkingSpotEndPose` / `SetPullOverSpotEndPose` / `SetParkAndGoEndPose` -- 根据场景计算规划终止位姿（x, y, heading, velocity）。
- `GetParkingSpot` -- 从地图查询目标车位并判断停车类型（垂直/平行）和位置（左/右侧）。
- `GetPullOverSpot` -- 从规划状态获取靠边停车区域四角点。
- `SearchTargetParkingSpotOnPath` -- 在参考路径上搜索目标车位。
- `GetNearbyPath` -- 根据 routing 和车位信息构建附近参考路径。
- `IsInParkingLot` / `GetParkSpotFromMap` -- 判断车辆是否在车位内并获取车位角点。
- `GetAllLaneSegments` -- 收集 routing 中的所有车道段。
- `AdjustPointsOrderToClockwise` -- 将多边形调整为顺时针顺序。

## 配置

配置项通过 `OpenSpaceRoiDeciderConfig` protobuf 消息定义，包含以下参数：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `roi_type` | RoiType | - | ROI 场景类型：PARKING / PULL_OVER / PARK_AND_GO |
| `roi_longitudinal_range_start` | double | 10.0 | 沿路径前方的 ROI 纵向范围（米） |
| `roi_longitudinal_range_end` | double | 10.0 | 沿路径后方的 ROI 纵向范围（米） |
| `parking_start_range` | double | 7.0 | 车位检测起始范围（米） |
| `parking_inwards` | bool | false | 是否车头朝内泊入（垂直车位） |
| `enable_perception_obstacles` | bool | false | 是否将感知障碍物纳入约束 |
| `parking_depth_buffer` | double | 0.1 | 车辆边缘到车位终止线的缓冲距离（米） |
| `roi_line_segment_min_angle` | double | 0.3 | 分割新线段的最小角度差（弧度） |
| `roi_line_segment_length` | double | 1.0 | ROI 采样步长（米） |
| `roi_line_segment_length_from_map` | double | 10.0 | 从地图获取边界时的采样步长（米） |
| `perception_obstacle_filtering_distance` | double | 1000.0 | 过滤远处障碍物的距离阈值（米） |
| `perception_obstacle_buffer` | double | - | 感知障碍物膨胀缓冲距离（米） |
| `curb_heading_tangent_change_upper_limit` | double | 1.0 | 路缘角突变检测阈值 |
| `end_pose_s_distance` | double | 10.0 | 即停即走终止位姿沿路径的距离（米） |
| `parallel_park_end_x_buffer` | double | 0.2 | 平行泊车终止位姿 x 方向缓冲（米） |
| `use_road_boundary_from_map` | bool | false | 是否从地图直接获取道路边界 |
| `expand_polygon_of_obstacle_by_distance` | bool | false | 是否按距离扩展障碍物多边形（否则扩展边界框） |

## 调用关系

```text
Frame::ExecuteTask
  └─ OpenSpaceRoiDecider::Process
       ├─ [PARKING]
       │    ├─ GetParkingSpot            -- 从地图获取车位信息
       │    ├─ SetOrigin                 -- 设置局部坐标原点
       │    ├─ SetParkingSpotEndPose     -- 计算终止位姿
       │    └─ GetParkingBoundary        -- 构建泊车 ROI 边界
       │         ├─ GetRoadBoundary      -- 提取道路边界
       │         │    └─ AddBoundaryKeyPoint  -- 检测路缘关键点
       │         └─ FuseLineSegments     -- 融合共线线段
       ├─ [PULL_OVER]
       │    ├─ GetPullOverSpot           -- 获取靠边停车区域
       │    ├─ SetOrigin                 -- 设置局部坐标原点
       │    ├─ SetPullOverSpotEndPose    -- 计算终止位姿
       │    └─ GetPullOverBoundary       -- 构建靠边停车 ROI 边界
       │         ├─ GetRoadBoundary
       │         └─ FuseLineSegments
       ├─ [PARK_AND_GO]
       │    ├─ SetOriginFromADC          -- 以车辆位置设置原点
       │    ├─ GetParkingOutBoundary     -- 泊出场景边界
       │    │    或 GetParkAndGoBoundary -- 即停即走场景边界
       │    │         ├─ GetRoadBoundary / GetRoadBoundaryFromMap
       │    │         └─ FuseLineSegments
       │    └─ SetParkAndGoEndPose       -- 计算终止位姿
       └─ FormulateBoundaryConstraints   -- 将边界转为优化约束
            ├─ LoadObstacleInVertices    -- 加载顶点表示
            │    └─ FilterOutObstacle    -- 过滤无关障碍物
            └─ LoadObstacleInHyperPlanes -- 转化为 Ax > b
                 └─ GetHyperPlanes       -- 计算超平面法向量
```
