# Map 高精地图模块

## 模块职责

Map 模块是 Apollo 自动驾驶平台的高精地图基础设施层，负责高精地图数据的加载、解析、存储和查询。它为 Planning、Routing、Perception 等上层模块提供统一的地图数据访问接口。

模块由三个核心子模块组成：

- **hdmap**：高精地图引擎，负责 OpenDRIVE XML / Protobuf 格式地图的加载与空间查询
- **pnc_map**：规划与控制地图层，在 hdmap 之上为 Planning 模块提供路径段（RouteSegments）和参考线（Path）抽象
- **relative_map**：相对地图组件，在无高精地图场景下基于感知车道线和导航线实时生成局部地图

## 核心类/接口

### hdmap 子模块

| 类名 | 文件 | 职责 |
|------|------|------|
| `HDMap` | `hdmap/hdmap.h` | 对外统一门面（Facade），委托 `HDMapImpl` 实现所有操作 |
| `HDMapImpl` | `hdmap/hdmap_impl.h` | 核心实现：地图加载、空间索引构建、各类元素查询 |
| `HDMapUtil` | `hdmap/hdmap_util.h` | 静态工具类，管理全局单例 base_map / sim_map 的懒加载与线程安全访问 |
| `LaneInfo` | `hdmap/hdmap_common.h` | 车道信息封装，包含中心线段、边界采样宽度、KD-Tree 索引 |
| `JunctionInfo` | `hdmap/hdmap_common.h` | 路口信息封装，持有多边形几何与 overlap 关联 |
| `SignalInfo` | `hdmap/hdmap_common.h` | 交通信号灯信息，包含停止线段集合 |
| `CrosswalkInfo` | `hdmap/hdmap_common.h` | 人行横道信息，持有多边形几何 |
| `StopSignInfo` | `hdmap/hdmap_common.h` | 停车标志信息，关联停止线与所属路口 |
| `RoadInfo` | `hdmap/hdmap_common.h` | 道路信息，包含 RoadSection 列表与道路边界 |
| `OverlapInfo` | `hdmap/hdmap_common.h` | 重叠区域信息，描述任意两个地图元素的空间重叠关系 |
| `ObjectWithAABox<T>` | `hdmap/hdmap_common.h` | 模板包装器，将地图对象与轴对齐包围盒绑定以支持 KD-Tree 查询 |
| `OpendriveAdapter` | `hdmap/adapter/opendrive_adapter.h` | OpenDRIVE XML 解析入口，协调各 XML Parser 完成格式转换 |
| `ProtoOrganizer` | `hdmap/adapter/proto_organizer.h` | 将解析后的中间数据结构组织为最终 Protobuf `Map` 消息 |

### pnc_map 子模块

| 类名 | 文件 | 职责 |
|------|------|------|
| `PncMapBase` | `pnc_map/pnc_map_base.h` | 规划地图抽象基类，定义 `GetRouteSegments` / `ExtendSegments` 等纯虚接口 |
| `RouteSegments` | `pnc_map/route_segments.h` | 路由通道段，继承自 `vector<LaneSegment>`，携带换道动作、目的地停车等属性 |
| `Path` | `pnc_map/path.h` | 参考路径，由有序 `MapPathPoint` 构成，提供 s-l 坐标投影与 overlap 查询 |
| `LaneWaypoint` | `pnc_map/path.h` | 车道路点 (lane, s, l) 三元组 |
| `MapPathPoint` | `pnc_map/path.h` | 路径点，包含 (x, y) 坐标、航向角及所属车道信息 |

### relative_map 子模块

| 类名 | 文件 | 职责 |
|------|------|------|
| `RelativeMapComponent` | `relative_map/relative_map_component.h` | Cyber RT 定时器组件，周期 100ms 触发地图生成 |
| `RelativeMap` | `relative_map/relative_map.h` | 相对地图核心逻辑，融合感知/定位/导航数据生成 `MapMsg` |
| `NavigationLane` | `relative_map/navigation_lane.h` | 导航车道生成器，基于导航线与感知车道标记生成实时车道 |

## 算法概述

### 高精地图数据结构

Apollo 采用基于 OpenDRIVE 标准扩展的高精地图格式，通过 Protobuf 定义数据模型。顶层消息 `Map`（定义于 `map.proto`）聚合了所有地图元素：

```protobuf
message Map {
  optional Header header = 1;       // 地图头信息（版本、投影、边界）
  repeated Lane lane = 4;           // 车道
  repeated Road road = 11;          // 道路
  repeated Junction junction = 3;   // 路口
  repeated Signal signal = 6;       // 交通信号灯
  repeated StopSign stop_sign = 5;  // 停车标志
  repeated Crosswalk crosswalk = 2; // 人行横道
  repeated Overlap overlap = 8;     // 元素重叠关系
  repeated ClearArea clear_area = 9;
  repeated SpeedBump speed_bump = 10;
  repeated ParkingSpace parking_space = 12;
  repeated PNCJunction pnc_junction = 13;
  repeated RSU rsu = 14;            // 路侧单元
  repeated Area ad_area = 15;
  repeated BarrierGate barrier_gate = 16;
}
```

#### 核心元素说明

**Lane（车道）** 是地图最基本的元素：
- `central_curve`：中心参考线，由 `CurveSegment`（折线段）序列组成
- `left_boundary` / `right_boundary`：车道边界线，携带边界类型（实线/虚线/路缘）
- `left_sample` / `right_sample`：中心点到边界的宽度采样关联
- `predecessor_id` / `successor_id`：前驱/后继车道拓扑
- `left_neighbor_forward_lane_id` / `right_neighbor_forward_lane_id`：同向相邻车道
- `type`：车道类型（CITY_DRIVING / BIKING / SIDEWALK / PARKING 等）
- `turn`：转弯类型（NO_TURN / LEFT_TURN / RIGHT_TURN / U_TURN）

**Road（道路）** 是车道的容器：
- 包含有序的 `RoadSection`，每个 Section 引用一组并行车道
- 关联 `junction_id` 表示是否位于路口内
- 类型分为 HIGHWAY / CITY_ROAD / PARK

**Junction（路口）** 表示道路交汇区域：
- 以 `Polygon` 描述路口边界
- 类型包括 IN_ROAD / CROSS_ROAD / FORK_ROAD / MAIN_SIDE / DEAD_END

**Overlap（重叠）** 描述任意两个地图元素的空间重叠关系：
- 通过 `ObjectOverlapInfo` 的 oneof 字段区分不同元素类型的重叠信息
- `LaneOverlapInfo` 携带 start_s / end_s 表示重叠在车道上的纵向范围

**几何基础类型**：
- `Curve` → `CurveSegment` → `LineSegment` → `PointENU`：曲线由折线段序列表示
- `Polygon`：由 `PointENU` 点序列构成的多边形

#### 坐标系与投影

地图使用 ENU（East-North-Up）坐标系。`Header.projection` 中存储 PROJ.4 投影参数字符串，`CoordinateConvertTool` 使用 proj4 库将 WGS84 经纬度转换为 UTM 平面坐标。

### 地图加载流程

`HDMapImpl::LoadMapFromFile` 根据文件扩展名选择加载策略：

1. **`.xml` 文件（OpenDRIVE 格式）**：调用 `OpendriveAdapter::LoadData`
   - 使用 tinyxml2 解析 XML DOM
   - 依次调用各专用解析器：
     - `HeaderXmlParser`：解析地图头信息（投影参数、版本等）
     - `RoadsXmlParser`：解析道路、车道、车道边界、速度限制
     - `LanesXmlParser`：解析车道详细信息（中心线、边界类型、overlap 关联）
     - `JunctionsXmlParser`：解析路口几何与关联道路
     - `SignalsXmlParser`：解析交通灯、停车标志、让行标志
     - `ObjectsXmlParser`：解析 RSU 等路侧对象
   - `ProtoOrganizer` 将中间数据结构（`RoadInternal`、`JunctionInternal` 等）组织为 Protobuf `Map` 消息，并计算 Overlap 关系

2. **`.bin` / `.txt` 文件（Protobuf 格式）**：直接通过 `GetProtoFromFile` 反序列化

加载完成后调用 `LoadMapFromProto`，遍历 `Map` 中所有元素，构建：
- 各类型的 `unordered_map` 哈希表（如 `lane_table_`、`junction_table_`），支持 O(1) ID 查询
- 各类型的 KD-Tree 空间索引（如 `lane_segment_kdtree_`、`junction_polygon_kdtree_`），支持高效近邻查询

### 空间查询算法

`HDMapImpl` 的空间查询基于 **AABox KD-Tree**（轴对齐包围盒 KD 树）：

1. **索引构建**：每个地图元素被包装为 `ObjectWithAABox<T>`，将几何对象（线段/多边形）与其轴对齐包围盒绑定
   - 车道被分割为线段（`LaneSegmentBox`），每段对应中心线的一个 `LineSegment2d`
   - 路口、人行横道等面状元素使用 `Polygon2d` 的包围盒

2. **查询过程**（以 `GetLanes(point, distance)` 为例）：
   - 以查询点为中心、distance 为半径构建搜索区域
   - KD-Tree 快速剪枝排除不相交的包围盒
   - 对候选对象计算精确距离，筛选满足条件的结果

3. **前向搜索**（如 `GetForwardNearestSignalsOnLane`）：
   - 从查询点所在车道出发，沿车道拓扑（successor）向前搜索
   - 通过 Overlap 关系找到车道上的信号灯/停车标志等
   - 搜索范围由 distance 参数控制

### PNC Map 路径构建

`PncMapBase` 为 Planning 模块提供路由段抽象：

- `GetRouteSegments`：根据车辆状态和路由结果，提取前后一定距离内的 `RouteSegments`
- `LookForwardDistance`：根据车速动态计算前视距离（低速 180m，高速 250m）
- `RouteSegments` 携带换道指令（`NextAction` / `PreviousAction`）和目的地停车标记

`Path` 类提供参考路径的几何计算：
- 将 `MapPathPoint` 序列转换为分段线段表示
- 支持 s-l 坐标系投影（`GetProjection`）和最近点查询（`GetNearestPoint`）
- 沿路径采样车道宽度和道路宽度
- 收集路径上的各类 Overlap（信号灯、停车标志、人行横道等）

### 相对地图生成

`NavigationLane` 在无高精地图场景下实时生成车道：

1. 接收导航线（`NavigationInfo`）和感知车道标记（`PerceptionObstacles`）
2. 将导航线投影到车辆坐标系，按曲率自适应采样
3. 融合感知车道宽度与导航线信息，生成左右边界
4. 输出标准 `MapMsg` 格式，供 Planning 模块透明使用

## 数据流

### hdmap 数据流（离线高精地图模式）

```
地图文件 (.xml / .bin / .txt)
        │
        ▼
  HDMapImpl::LoadMapFromFile
        │
        ├── .xml ──► OpendriveAdapter::LoadData
        │               ├── HeaderXmlParser
        │               ├── RoadsXmlParser
        │               ├── LanesXmlParser
        │               ├── JunctionsXmlParser
        │               ├── SignalsXmlParser
        │               ├── ObjectsXmlParser
        │               └── ProtoOrganizer ──► Map (protobuf)
        │
        └── .bin/.txt ──► GetProtoFromFile ──► Map (protobuf)
                                │
                                ▼
                    HDMapImpl::LoadMapFromProto
                        ├── 构建哈希表 (lane_table_, junction_table_, ...)
                        ├── PostProcess (关联 Overlap)
                        └── 构建 KD-Tree 空间索引
                                │
                                ▼
                    HDMap / HDMapUtil (对外查询接口)
                        │
                        ▼
            Planning / Routing / Perception 等模块
```

### relative_map 数据流（导航模式）

```
/apollo/perception_obstacles ──► RelativeMapComponent
/apollo/canbus/chassis       ──►       │
/apollo/localization/pose    ──►       │
/apollo/navigation           ──►       │
                                       ▼
                               RelativeMap::Process
                                       │
                                       ▼
                              NavigationLane::GeneratePath
                              NavigationLane::CreateMap
                                       │
                                       ▼
                                    MapMsg
                                       │
                                       ▼
                          /apollo/relative_map (Cyber RT topic)
                                       │
                                       ▼
                          HDMapUtil::BaseMapPtr(MapMsg)
                                       │
                                       ▼
                              Planning 模块透明使用
```

### pnc_map 数据流

```
HDMap (hdmap 查询接口)  +  RoutingResponse (路由结果)
              │                        │
              ▼                        ▼
         PncMapBase::GetRouteSegments
              │
              ▼
      RouteSegments (车道段列表 + 换道指令)
              │
              ▼
         Path (参考路径)
              │
              ├── s-l 坐标投影
              ├── 车道/道路宽度采样
              └── Overlap 收集 (信号灯/停车标志/人行横道...)
              │
              ▼
      Planning 模块 (参考线生成)
```

## 配置方式

### 全局地图配置（gflags）

地图相关的全局配置定义在 `modules/common/configs/config_gflags.cc` 中：

| Flag | 默认值 | 说明 |
|------|--------|------|
| `map_dir` | `/apollo/modules/map/data/sunnyvale_loop` | 地图数据目录 |
| `base_map_filename` | `base_map.bin\|base_map.xml\|base_map.txt` | 基础地图文件名（按优先级尝试） |
| `sim_map_filename` | `sim_map.bin\|sim_map.txt` | 仿真地图文件名 |
| `routing_map_filename` | `routing_map.bin\|routing_map.txt` | 路由地图文件名 |
| `end_way_point_filename` | `default_end_way_point.txt` | 默认终点文件 |
| `use_navigation_mode` | `false` | 是否启用导航模式（使用 relative_map） |
| `local_utm_zone_id` | `10` | UTM 区域 ID |
| `half_vehicle_width` | `1.05` | 半车宽（用于车道宽度校验） |

`HDMapUtil` 通过 `BaseMapFile()` 函数在 `map_dir` 下按 `base_map_filename` 中的优先级列表查找第一个存在的文件进行加载。

### relative_map 配置

配置文件：`modules/map/relative_map/conf/relative_map.conf`

```
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--use_navigation_mode
--relative_map_config_filename=/apollo/modules/map/relative_map/conf/relative_map_config.pb.txt
--enable_cyclic_rerouting=1
```

Protobuf 配置（`RelativeMapConfig`）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_param.default_left_width` | 1.75m | 默认车道左半宽 |
| `map_param.default_right_width` | 1.75m | 默认车道右半宽 |
| `map_param.default_speed_limit` | 29.06 m/s (65mph) | 默认限速 |
| `navigation_lane.min_lane_marker_quality` | 0.5 | 车道线检测最低质量阈值 |
| `navigation_lane.max_len_from_navigation_line` | 250m | 导航线最大截取长度 |
| `navigation_lane.min_len_for_navigation_lane` | 150m | 生成车道最小长度 |
| `navigation_lane.max_distance_to_navigation_line` | 15m | 车辆到导航线最大距离 |
| `navigation_lane.lane_marker_weight` | 0.1 | 感知车道线融合权重 |

### DAG 配置

`relative_map` 作为 Cyber RT 定时器组件运行：

```
# modules/map/relative_map/dag/relative_map.dag
module_config {
    module_library : "modules/map/librelative_map_component.so"
    timer_components {
        class_name : "RelativeMapComponent"
        config {
            name: "relative_map"
            flag_file_path: "/apollo/modules/map/relative_map/conf/relative_map.conf"
            interval: 100   # 100ms 周期
        }
    }
}
```

### 地图数据目录结构

```
modules/map/data/<map_name>/
├── base_map.bin          # 二进制 Protobuf 格式基础地图（优先加载）
├── base_map.xml          # OpenDRIVE XML 格式基础地图
├── base_map.txt          # 文本 Protobuf 格式基础地图
├── sim_map.bin           # 仿真用简化地图
├── routing_map.bin       # 路由用拓扑地图
├── default_end_way_point.txt  # 默认终点
└── speed_control.pb.txt  # 速度控制区域
```

### 地图工具

模块提供以下命令行工具：

| 工具 | 源文件 | 功能 |
|------|--------|------|
| `bin_map_generator` | `tools/bin_map_generator.cc` | 将 .txt 格式地图转换为 .bin 二进制格式 |
| `proto_map_generator` | `tools/proto_map_generator.cc` | Protobuf 地图生成工具 |
| `sim_map_generator` | `tools/sim_map_generator.cc` | 生成仿真用简化地图 |
| `map_tool` | `tools/map_tool.cc` | 地图信息查询工具 |
| `map_xysl` | `tools/map_xysl.cc` | XY 坐标与 SL 坐标互转工具 |
| `map_datachecker` | `tools/map_datachecker/` | 地图数据质量检查（C/S 架构） |
