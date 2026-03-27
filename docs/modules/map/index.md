# Map 地图模块

Apollo 地图模块（`modules/map`）是自动驾驶系统的基础设施层，负责高精地图（HD Map）的加载、解析、存储和查询。它为感知、规划、路由等上层模块提供统一的地图数据访问接口，是整个自动驾驶软件栈中不可或缺的底层依赖。

## 模块职责

Map 模块承担以下核心职责：

1. **高精地图数据管理**：加载和解析 OpenDRIVE XML 格式或 Protobuf 二进制格式的地图文件，将其转换为统一的内部数据结构。
2. **空间索引与高效查询**：基于 AABox KD-Tree 空间索引，提供按位置、距离、方向等条件的高效地图元素检索能力。
3. **地图元素抽象**：将车道（Lane）、路口（Junction）、信号灯（Signal）、停车标志（StopSign）、人行横道（Crosswalk）等地图元素封装为带有几何计算能力的 C++ 对象。
4. **规划导航地图（PnC Map）**：为 Planning 模块提供基于路由结果的路段切分、路径生成等高层抽象。
5. **相对地图（Relative Map）**：在无高精地图的导航模式下，基于感知车道线和导航线实时生成局部地图。

模块整体分为三个子系统：

| 子系统 | 目录 | 说明 |
|--------|------|------|
| HDMap | `hdmap/` | 高精地图核心，负责地图加载、存储、空间索引和查询 |
| PnC Map | `pnc_map/` | 规划导航地图，为 Planning 提供路段和路径抽象 |
| Relative Map | `relative_map/` | 相对地图，导航模式下实时生成局部地图 |

## 核心类与接口

### HDMap 层

```
HDMap (hdmap.h)                    -- 对外统一接口，Facade 模式
  └── HDMapImpl (hdmap_impl.h)     -- 实际实现，管理所有地图元素表和 KD-Tree
        ├── LaneTable              -- unordered_map<string, shared_ptr<LaneInfo>>
        ├── JunctionTable          -- unordered_map<string, shared_ptr<JunctionInfo>>
        ├── SignalTable            -- unordered_map<string, shared_ptr<SignalInfo>>
        ├── CrosswalkTable
        ├── StopSignTable / YieldSignTable
        ├── ClearAreaTable / SpeedBumpTable
        ├── ParkingSpaceTable / RoadTable
        ├── OverlapTable / PNCJunctionTable
        ├── RSUTable / AreaTable / BarrierGateTable
        └── 各类 KD-Tree 空间索引
```

**`HDMap`**（`hdmap/hdmap.h`）是对外的统一门面类，采用 Facade 模式将所有查询委托给内部的 `HDMapImpl`。主要接口：

- **按 ID 查询**：`GetLaneById`、`GetJunctionById`、`GetSignalById`、`GetStopSignById`、`GetCrosswalkById`、`GetRoadById`、`GetOverlapById` 等，通过哈希表 O(1) 查找
- **按空间范围查询**：`GetLanes(point, distance)`、`GetJunctions(point, distance)`、`GetSignals(point, distance)` 等，基于 KD-Tree 空间检索
- **沿车道前向查询**：`GetForwardNearestSignalsOnLane`、`GetForwardNearestBarriersOnLane`，沿车道拓扑方向搜索前方元素
- **关联查询**：`GetStopSignAssociatedStopSigns`、`GetStopSignAssociatedLanes`，查询同一路口内关联的交通标志
- **局部地图提取**：`GetLocalMap(point, range, local_map)` 提取指定区域的地图子集

**`HDMapImpl`**（`hdmap/hdmap_impl.h`）是核心实现类，内部维护：
- 各类地图元素的 `unordered_map` 哈希表（以元素 ID 字符串为 key），支持 O(1) 查找
- 各类地图元素的 `AABoxKDTree2d` 空间索引树，支持高效近邻查询

**`HDMapUtil`**（`hdmap/hdmap_util.h`）是静态工具类，提供全局单例地图访问：

```cpp
static const HDMap* HDMapUtil::BaseMapPtr();                    // 懒加载 base_map（线程安全）
static const HDMap* HDMapUtil::BaseMapPtr(const MapMsg& msg);   // 从 RelativeMap 消息创建
static const HDMap* HDMapUtil::SimMapPtr();                     // 获取仿真地图
static bool HDMapUtil::ReloadMaps();                            // 重新加载所有地图
```

地图文件路径通过 gflags 配置：`BaseMapFile()`、`SimMapFile()`、`RoutingMapFile()` 分别从 `FLAGS_map_dir` 和对应文件名 flag 拼接路径。

### 地图元素信息类

所有地图元素都有对应的 `XxxInfo` 封装类（定义在 `hdmap/hdmap_common.h`），在 Protobuf 原始数据基础上增加了几何计算能力：

| 类名 | Protobuf 类型 | 几何表示 | 说明 |
|------|--------------|---------|------|
| `LaneInfo` | `Lane` | 中心线点序列 + `LineSegment2d` 分段 | 车道，含宽度采样、累积弧长、方向向量 |
| `JunctionInfo` | `Junction` | `Polygon2d` | 路口区域 |
| `SignalInfo` | `Signal` | `LineSegment2d`（停止线） | 交通信号灯 |
| `CrosswalkInfo` | `Crosswalk` | `Polygon2d` | 人行横道 |
| `StopSignInfo` | `StopSign` | `LineSegment2d`（停止线） | 停车标志 |
| `YieldSignInfo` | `YieldSign` | `LineSegment2d` | 让行标志 |
| `ClearAreaInfo` | `ClearArea` | `Polygon2d` | 禁停区 |
| `SpeedBumpInfo` | `SpeedBump` | `LineSegment2d` | 减速带 |
| `ParkingSpaceInfo` | `ParkingSpace` | `Polygon2d` | 停车位 |
| `RoadInfo` | `Road` | 路段边界 | 道路，含 `RoadSection` 列表 |
| `OverlapInfo` | `Overlap` | -- | 元素间重叠关系 |
| `PNCJunctionInfo` | `PNCJunction` | `Polygon2d` | 规划用路口 |
| `RSUInfo` | `RSU` | -- | 路侧单元 |
| `AreaInfo` | `Area` | `Polygon2d` | 区域 |
| `BarrierGateInfo` | `BarrierGate` | `LineSegment2d` | 道闸 |

每个 Info 类都通过 `ObjectWithAABox` 模板包装为带轴对齐包围盒（AABB）的对象，用于构建 KD-Tree：

```cpp
template <class Object, class GeoObject>
class ObjectWithAABox {
  AABox2d aabox_;           // 轴对齐包围盒
  const Object* object_;    // 地图元素指针
  const GeoObject* geo_object_;  // 几何对象指针（LineSegment2d 或 Polygon2d）
  int id_;                  // 对象标识符
  // 提供 DistanceTo / DistanceSquareTo 用于 KD-Tree 查询
};
```

`LaneInfo` 是最核心的元素类，内部维护丰富的预计算数据：

```cpp
class LaneInfo {
  const Lane& lane_;                          // Protobuf 原始数据
  std::vector<Vec2d> points_;                 // 中心线离散点
  std::vector<Vec2d> unit_directions_;        // 各段单位方向向量
  std::vector<double> headings_;              // 各点航向角
  std::vector<LineSegment2d> segments_;       // 中心线分段
  std::vector<double> accumulated_s_;         // 累积弧长
  std::vector<OverlapInfoConstPtr> signals_;  // 关联的信号灯 overlap
  std::vector<OverlapInfoConstPtr> stop_signs_;  // 关联的停车标志 overlap
  std::vector<SampledWidth> sampled_left_width_;   // 左侧宽度采样
  std::vector<SampledWidth> sampled_right_width_;  // 右侧宽度采样
  // ...
};
```

### PnC Map 层

**`PncMapBase`**（`pnc_map/pnc_map_base.h`）是规划导航地图的抽象基类，位于 `apollo::planning` 命名空间：

```cpp
class PncMapBase {
  virtual bool GetRouteSegments(
      const VehicleState& vehicle_state,
      std::list<RouteSegments>* route_segments) = 0;
  virtual bool ExtendSegments(...) = 0;
  virtual std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const = 0;
  virtual hdmap::LaneInfoConstPtr GetLaneById(const Id& id) const = 0;
  virtual double GetDistanceToDestination() const = 0;
};
```

**`RouteSegments`**（`pnc_map/route_segments.h`）继承自 `std::vector<LaneSegment>`，表示路由中的一个 Passage 区域，附加了变道动作（`NextAction` / `PreviousAction`）、是否可退出（`can_exit_`）、是否为当前段（`is_on_segment_`）、目的地停车标记等属性。

**`Path`**（`pnc_map/path.h`）表示一条由多个 `MapPathPoint` 组成的路径，提供：
- 路径上任意弧长处的插值（位置、方向、宽度）
- 路径与各类地图元素的重叠区间（`PathOverlap`），涵盖信号灯、停车标志、人行横道、减速带、路口等
- 路径近似（`PathApproximation`）用于加速最近点查询
- 左右车道宽度和道路宽度的采样

关键数据结构：

```cpp
struct LaneWaypoint {
  LaneInfoConstPtr lane;  // 所在车道
  double s;               // 沿车道的弧长
  double l;               // 横向偏移
};

struct LaneSegment {
  LaneInfoConstPtr lane;
  double start_s, end_s;  // 车道上的起止弧长
};

class MapPathPoint : public Vec2d {
  double heading_;                              // 航向角（通过 heading() 访问）
  std::vector<LaneWaypoint> lane_waypoints_;    // 该点对应的车道位置（通过 lane_waypoints() 访问）
  // 成员变量通过 getter/setter 方法访问
};
```

### Relative Map 层

**`RelativeMapComponent`**（`relative_map/relative_map_component.h`）是 Cyber RT 定时器组件，每 100ms 触发一次，订阅感知、底盘、定位和导航信息，输出实时生成的 `MapMsg`。

**`RelativeMap`**（`relative_map/relative_map.h`）是核心逻辑类，接收多路输入并调用 `NavigationLane` 生成地图。

**`NavigationLane`**（`relative_map/navigation_lane.h`）负责：
- 融合导航线和感知车道线，生成虚拟车道的中心线和左右边界
- 管理多条导航路径的排列（从左到右按行驶方向排序）
- 处理路径拼接（`StitchIndexPair`）和车辆投影（`ProjIndexPair`）
- 支持两种车道来源：`PERCEPTION`（感知车道标记）和 `OFFLINE_GENERATED`（离线导航线）

## 高精地图数据结构（OpenDRIVE 等详细说明）

### 地图格式支持

Apollo 地图模块支持两种地图格式：

1. **OpenDRIVE XML 格式**（`.xml`）：国际通用的高精地图标准格式
2. **Apollo Protobuf 格式**（`.bin` / `.txt`）：Apollo 自定义的 Protobuf 序列化格式

加载逻辑在 `HDMapImpl::LoadMapFromFile` 中根据文件扩展名自动选择解析器：

```cpp
int HDMapImpl::LoadMapFromFile(const std::string& map_filename) {
  if (absl::EndsWith(map_filename, ".xml")) {
    adapter::OpendriveAdapter::LoadData(map_filename, &map_);  // OpenDRIVE
  } else {
    cyber::common::GetProtoFromFile(map_filename, &map_);       // Protobuf
  }
  return LoadMapFromProto(map_);
}
```

### OpenDRIVE 适配器

`OpendriveAdapter`（`hdmap/adapter/opendrive_adapter.h`）是 OpenDRIVE XML 到 Apollo Protobuf 的转换桥梁。解析流程：

```
OpenDRIVE XML 文件
  │  tinyxml2 解析
  ▼
XML DOM 树
  │  各专用 Parser 分别解析
  ├── HeaderXmlParser    → PbHeader（版本、投影参数）
  ├── RoadsXmlParser     → vector<RoadInternal>（道路、车道、边界）
  ├── LanesXmlParser     → LaneInternal（中心线、边界类型、overlap）
  ├── JunctionsXmlParser → vector<JunctionInternal>（路口几何）
  ├── SignalsXmlParser   → TrafficLightInternal / StopSignInternal / YieldSignInternal
  └── ObjectsXmlParser   → ObjectInternal（RSU 等路侧对象）
  │
  ▼  ProtoOrganizer 组织
中间数据结构（RoadInternal, JunctionInternal 等）
  │  计算 Overlap 关系
  ▼
apollo::hdmap::Map（Protobuf 消息）
```

中间数据结构定义在 `hdmap/adapter/xml_parser/common_define.h`：

```cpp
struct RoadInternal {
  std::string id;
  std::string type;
  PbRoad road;
  bool in_junction;
  std::string junction_id;
  std::vector<RoadSectionInternal> sections;
  std::vector<TrafficLightInternal> traffic_lights;
  std::vector<StopSignInternal> stop_signs;
  std::vector<YieldSignInternal> yield_signs;
  std::vector<StopLineInternal> stop_lines;
  std::vector<PbCrosswalk> crosswalks;
  std::vector<PbClearArea> clear_areas;
  std::vector<PbSpeedBump> speed_bumps;
  std::vector<PbParkingSpace> parking_spaces;
  std::vector<PbPNCJunction> pnc_junctions;
};
```

`ProtoOrganizer` 负责将这些中间结构组织为最终的 Protobuf `Map` 消息，关键步骤包括：
- `GetRoadElements`：提取道路和车道的 Protobuf 表示
- `GetJunctionElements`：提取路口信息
- `GetOverlapElements`：计算车道与信号灯、车道与车道、车道与路口等的空间重叠关系
- `GetObjectElements`：提取 RSU 等路侧对象
- `OutputData`：将所有元素写入 `Map` 消息

### Protobuf 数据模型

顶层消息 `Map`（定义于 `map.proto`）聚合了所有地图元素：

```protobuf
message Map {
  optional Header header = 1;       // 地图头信息（版本、投影、边界）
  repeated Crosswalk crosswalk = 2; // 人行横道
  repeated Junction junction = 3;   // 路口
  repeated Lane lane = 4;           // 车道
  repeated StopSign stop_sign = 5;  // 停车标志
  repeated Signal signal = 6;       // 交通信号灯
  repeated YieldSign yield = 7;     // 让行标志
  repeated Overlap overlap = 8;     // 元素重叠关系
  repeated ClearArea clear_area = 9;
  repeated SpeedBump speed_bump = 10;
  repeated Road road = 11;          // 道路
  repeated ParkingSpace parking_space = 12;
  repeated PNCJunction pnc_junction = 13;
  repeated RSU rsu = 14;            // 路侧单元
  repeated Area ad_area = 15;
  repeated BarrierGate barrier_gate = 16;
}
```

### 核心元素详解

**Lane（车道）** 是地图最基本的元素：
- `central_curve`：中心参考线，由 `CurveSegment`（折线段）序列组成，每个折线段包含有序的 `PointENU` 点
- `left_boundary` / `right_boundary`：车道边界线，携带边界类型（实线 `SOLID` / 虚线 `DOTTED` / 路缘 `CURB`）
- `left_sample` / `right_sample`：沿弧长采样的中心线到边界的宽度值
- `predecessor_id` / `successor_id`：前驱/后继车道 ID，构成车道拓扑图
- `left_neighbor_forward_lane_id` / `right_neighbor_forward_lane_id`：同向相邻车道
- `type`：车道类型（`CITY_DRIVING` / `BIKING` / `SIDEWALK` / `PARKING` 等）
- `turn`：转弯类型（`NO_TURN` / `LEFT_TURN` / `RIGHT_TURN` / `U_TURN`）
- `speed_limit`：车道限速（m/s）
- `overlap_id`：与该车道有空间重叠的其他元素 ID 列表

**Road（道路）** 是车道的容器：
- 包含有序的 `RoadSection`，每个 Section 引用一组并行车道
- 关联 `junction_id` 表示是否位于路口内
- 类型分为 `HIGHWAY` / `CITY_ROAD` / `PARK`
- 通过 `BoundaryPolygon` 和 `BoundaryEdge` 描述道路边界

**Junction（路口）** 表示道路交汇区域：
- 以 `Polygon` 描述路口边界
- 类型包括 `IN_ROAD` / `CROSS_ROAD` / `FORK_ROAD` / `MAIN_SIDE` / `DEAD_END`

**Overlap（重叠）** 描述任意两个地图元素的空间重叠关系：
- 通过 `ObjectOverlapInfo` 的 oneof 字段区分不同元素类型
- `LaneOverlapInfo` 携带 `start_s` / `end_s` 表示重叠在车道上的纵向范围
- 这是连接车道与交通标志、信号灯等元素的关键纽带

### 坐标系与投影

地图使用 ENU（East-North-Up）坐标系。`Header.projection` 中存储 PROJ.4 投影参数字符串，例如：

```
+proj=tmerc +lat_0={37.413082} +lon_0={-122.013332} +k={0.9999999996} +ellps=WGS84 +no_defs
```

`CoordinateConvertTool`（`xml_parser/coordinate_convert_tool.h`）使用 proj4 库将 WGS84 经纬度转换为 UTM 平面坐标，在 OpenDRIVE 解析过程中完成坐标转换。

### 几何基础类型

```
Curve → CurveSegment → LineSegment → PointENU(x, y, z)
Polygon → PointENU 点序列
```

所有几何计算依赖 `modules/common/math` 库：`Vec2d`、`LineSegment2d`、`AABox2d`、`Polygon2d`、`AABoxKDTree2d` 等。

## 地图加载与查询

### 加载流程

地图加载由 `HDMapImpl::LoadMapFromProto` 完成，无论原始格式是 OpenDRIVE 还是 Protobuf，最终都会走到这个方法。加载过程分为三个阶段：

**阶段一：构建哈希表**

遍历 `Map` 消息中的所有元素，为每种类型创建 `XxxInfo` 对象并存入对应的 `unordered_map`：

```cpp
for (const auto& lane : map_.lane()) {
  lane_table_[lane.id().id()].reset(new LaneInfo(lane));
}
for (const auto& junction : map_.junction()) {
  junction_table_[junction.id().id()].reset(new JunctionInfo(junction));
}
// ... signal, crosswalk, stop_sign, yield_sign, clear_area,
//     speed_bump, parking_space, pnc_junction, rsu, overlap, road, area, barrier_gate
```

**阶段二：后处理与关联**

- 将 `Road` 中的 `RoadSection` 与 `Lane` 关联，设置每条车道的 `road_id` 和 `section_id`
- 调用各元素的 `PostProcess` 方法，建立 Overlap 关联（如车道关联其上的信号灯、停车标志等）

```cpp
for (const auto& road_ptr_pair : road_table_) {
  for (const auto& section : road_ptr_pair.second->sections()) {
    for (const auto& lane_id : section.lane_id()) {
      lane_table_[lane_id.id()]->set_road_id(road_id);
    }
  }
}
for (const auto& lane_ptr_pair : lane_table_) {
  lane_ptr_pair.second->PostProcess(*this);  // 建立 overlap 关联
}
```

**阶段三：构建 KD-Tree 空间索引**

为每种需要空间查询的元素类型构建独立的 KD-Tree：

```cpp
BuildLaneSegmentKDTree();          // 车道线段索引
BuildJunctionPolygonKDTree();      // 路口多边形索引
BuildSignalSegmentKDTree();        // 信号灯索引
BuildCrosswalkPolygonKDTree();     // 人行横道索引
BuildStopSignSegmentKDTree();      // 停车标志索引
BuildYieldSignSegmentKDTree();     // 让行标志索引
BuildClearAreaPolygonKDTree();     // 禁停区索引
BuildSpeedBumpSegmentKDTree();     // 减速带索引
BuildParkingSpacePolygonKDTree();  // 停车位索引
BuildPNCJunctionPolygonKDTree();   // PNC 路口索引
BuildAreaPolygonKDTree();          // 区域索引
BuildBarrierGateSegmentKDTree();   // 道闸索引
```

### 空间查询算法

`HDMapImpl` 的空间查询基于 **AABox KD-Tree**（轴对齐包围盒 KD 树）：

1. **索引构建**：每个地图元素被包装为 `ObjectWithAABox<T>`，将几何对象（线段/多边形）与其轴对齐包围盒绑定
   - 车道被分割为线段（`LaneSegmentBox`），每段对应中心线的一个 `LineSegment2d`
   - 路口、人行横道等面状元素使用 `Polygon2d` 的包围盒

2. **范围查询**（以 `GetLanes(point, distance)` 为例）：
   - 以查询点为中心、distance 为半径构建搜索区域
   - KD-Tree 快速剪枝排除不相交的包围盒
   - 对候选对象计算精确距离，筛选满足条件的结果

3. **前向搜索**（如 `GetForwardNearestSignalsOnLane`）：
   - 先通过 `GetLanes` 找到查询点附近的车道
   - 确定查询点在车道上的投影位置（s 值）
   - 沿车道拓扑（successor）向前搜索，搜索范围由 `kLanesSearchRange`（默认 10m）和 distance 参数控制
   - 通过 Overlap 关系找到车道上的信号灯/停车标志等
   - 支持 `kBackwardDistance`（默认 4m）的回溯搜索

4. **最近车道查询**（`GetNearestLaneWithHeading`）：
   - 在 KD-Tree 中搜索距离查询点最近的车道段
   - 额外过滤航向角差异，确保返回的车道方向与查询方向一致

### 全局地图访问

`HDMapUtil` 提供线程安全的全局单例访问模式：

```cpp
const HDMap* HDMapUtil::BaseMapPtr() {
  if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double-check locking
      base_map_ = CreateMap(BaseMapFile());
    }
  }
  return base_map_.get();
}
```

支持两种地图来源：
- **文件加载**：从 `FLAGS_map_dir` 指定的目录加载 `base_map.bin` 或 `base_map.xml`
- **消息加载**：从 `RelativeMap` 组件发布的 `MapMsg` 消息中加载（导航模式）

### PnC Map 路径构建

`PncMapBase` 为 Planning 模块提供路由段抽象：

- `GetRouteSegments`：根据车辆状态和路由结果，提取前后一定距离内的 `RouteSegments`
- `LookForwardDistance`：根据车速动态计算前视距离
- `Path` 类将 `MapPathPoint` 序列转换为分段线段表示，支持 s-l 坐标系投影和最近点查询，沿路径采样车道宽度和道路宽度，收集路径上的各类 Overlap

## 数据流

### 静态地图数据流（HDMap 模式）

```
地图文件（.xml / .bin / .txt）
  │
  ▼  HDMapImpl::LoadMapFromFile
  ├── .xml → OpendriveAdapter::LoadData
  │           ├── HeaderXmlParser
  │           ├── RoadsXmlParser + LanesXmlParser
  │           ├── JunctionsXmlParser
  │           ├── SignalsXmlParser
  │           ├── ObjectsXmlParser
  │           └── ProtoOrganizer → Map (Protobuf)
  │
  └── .bin/.txt → GetProtoFromFile → Map (Protobuf)
  │
  ▼  HDMapImpl::LoadMapFromProto
  ├── 构建各类型 unordered_map 哈希表
  ├── PostProcess 建立 Overlap 关联
  └── Build*KDTree 构建空间索引
  │
  ▼  HDMap / HDMapUtil 对外提供查询
  ├── Planning 模块 → GetLanes / GetSignals / GetNearestLane ...
  ├── Routing 模块 → GetLaneById / GetRoadById ...
  ├── Perception 模块 → GetJunctions / GetCrosswalks ...
  └── 其他模块 → GetLocalMap / GetRoi ...
```

### 动态地图数据流（Relative Map 模式）

在导航模式（`FLAGS_use_navigation_mode = true`）下，`RelativeMapComponent` 作为 Cyber RT 定时器组件运行：

```
输入 Cyber 话题：
  /apollo/perception/obstacles  ──┐
  /apollo/canbus/chassis        ──┤
  /apollo/localization/pose     ──┤  RelativeMapComponent (100ms 周期)
  /apollo/navigation            ──┘
                                   │
                                   ▼  RelativeMap::Process
                                   NavigationLane
                                   ├── 融合导航线与感知车道标记
                                   ├── 生成虚拟车道中心线和边界
                                   └── 构建 MapMsg
                                   │
                                   ▼
输出 Cyber 话题：
  /apollo/relative_map  ──→  Planning 模块（通过 HDMapUtil::BaseMapPtr(MapMsg) 访问）
```

`RelativeMapComponent` 的输入输出：

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| 输入 | `/apollo/perception/obstacles` | `PerceptionObstacles` | 感知障碍物及车道标记 |
| 输入 | `/apollo/canbus/chassis` | `Chassis` | 底盘信息（车速等） |
| 输入 | `/apollo/localization/pose` | `LocalizationEstimate` | 车辆定位 |
| 输入 | `/apollo/navigation` | `NavigationInfo` | 导航路线信息 |
| 输出 | `/apollo/relative_map` | `MapMsg` | 实时生成的局部地图 |

### PnC Map 数据流

```
Routing 模块输出
  └── PlanningCommand (含 RoutingResponse)
        │
        ▼  PncMapBase::UpdatePlanningCommand
        PncMapBase
        │
        ▼  PncMapBase::GetRouteSegments(vehicle_state)
        ├── 根据车辆位置确定当前所在车道
        ├── 按前视/后视距离截取 RouteSegments
        └── 标记换道动作和目的地停车
        │
        ▼
        std::list<RouteSegments>
        │
        ▼  Path 构建
        Path（参考路径）
        ├── MapPathPoint 序列
        ├── 各类 PathOverlap
        └── 车道/道路宽度采样
        │
        ▼
        Planning 模块使用
```

## 配置方式

### 全局地图配置（gflags）

地图模块的核心配置通过 gflags 全局标志完成，主要标志定义在 `modules/common/configs/config_gflags.cc`：

| Flag | 默认值 | 说明 |
|------|--------|------|
| `FLAGS_map_dir` | -- | 地图数据目录路径 |
| `FLAGS_base_map_filename` | `base_map.bin\|base_map.xml\|base_map.txt` | 基础地图文件名（支持 `\|` 分隔的候选列表） |
| `FLAGS_sim_map_filename` | `sim_map.bin\|sim_map.txt` | 仿真地图文件名 |
| `FLAGS_routing_map_filename` | `routing_map.bin\|routing_map.txt` | 路由地图文件名 |
| `FLAGS_end_way_point_filename` | `default_end_way_point.txt` | 默认终点文件 |
| `FLAGS_use_navigation_mode` | `false` | 是否启用导航模式（使用 Relative Map） |
| `FLAGS_test_base_map_filename` | `""` | 测试用地图文件名（非空时覆盖 base_map_filename） |

文件路径解析逻辑（`hdmap_util.cc`）：
- `BaseMapFile()` 在 `FLAGS_map_dir` 下按 `FLAGS_base_map_filename` 中 `|` 分隔的候选文件名依次查找，返回第一个存在的文件路径
- 导航模式下不使用 base_map 文件，而是从 Relative Map 话题获取地图数据

### Relative Map 配置

Relative Map 组件的配置分为三层：

**1. DAG 配置**（`relative_map/dag/relative_map.dag`）：

```
module_config {
    module_library : "modules/map/librelative_map_component.so"
    timer_components {
        class_name : "RelativeMapComponent"
        config {
            name: "relative_map"
            flag_file_path: "/apollo/modules/map/relative_map/conf/relative_map.conf"
            interval: 100   # 100ms 触发周期
        }
    }
}
```

**2. Flag 配置**（`relative_map/conf/relative_map.conf`）：

```
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--use_navigation_mode
--relative_map_config_filename=/apollo/modules/map/relative_map/conf/relative_map_config.pb.txt
--enable_cyclic_rerouting=1
```

**3. Protobuf 配置**（`relative_map/conf/relative_map_config.pb.txt`）：

```protobuf
// RelativeMapConfig 消息结构
message RelativeMapConfig {
  optional MapGenerationParam map_param = 1;
  optional NavigationLaneConfig navigation_lane = 2;
}
```

关键配置参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_param.default_left_width` | 1.875m | 默认车道左侧宽度 |
| `map_param.default_right_width` | 1.875m | 默认车道右侧宽度 |
| `map_param.default_speed_limit` | 29.06 m/s (65mph) | 默认限速 |
| `navigation_lane.min_lane_marker_quality` | 0.49 | 车道标记最低质量阈值 |
| `navigation_lane.lane_source` | `OFFLINE_GENERATED` | 车道来源（感知/离线） |
| `navigation_lane.max_len_from_navigation_line` | 250m | 导航线最大使用长度 |
| `navigation_lane.min_len_for_navigation_lane` | 150m | 生成车道最小长度 |
| `navigation_lane.max_len_for_navigation_lane` | 250m | 生成车道最大长度 |
| `navigation_lane.max_distance_to_navigation_line` | 15m | 车辆到导航线最大距离 |
| `navigation_lane.lane_marker_weight` | 0.1 | 车道标记融合权重 |

**4. Launch 配置**（`relative_map/launch/relative_map.launch`）：

```xml
<cyber>
    <module>
        <name>relative_map</name>
        <dag_conf>/apollo/modules/map/relative_map/dag/relative_map.dag</dag_conf>
        <process_name>relative_map</process_name>
    </module>
</cyber>
```

### 地图数据目录结构

```
modules/map/data/<map_name>/
├── base_map.bin          # 二进制 Protobuf 格式基础地图（优先加载）
├── base_map.xml          # OpenDRIVE XML 格式基础地图
├── base_map.txt          # 文本 Protobuf 格式基础地图
├── sim_map.bin           # 仿真用简化地图
├── routing_map.bin       # 路由用拓扑地图
└── default_end_way_point.txt  # 默认终点
```

### 地图工具

模块提供以下命令行工具：

| 工具 | 源文件 | 功能 |
|------|--------|------|
| `bin_map_generator` | `tools/bin_map_generator.cc` | 将 `.txt` 格式地图转换为 `.bin` 二进制格式 |
| `map_datachecker` | `tools/map_datachecker/` | 地图数据质量检查（C/S 架构） |