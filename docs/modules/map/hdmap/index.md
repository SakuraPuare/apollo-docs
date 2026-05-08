---
title: "高精地图"
---

# 高精地图

> 源码路径: `modules/map/hdmap/`

## 概述

HDMap（High-Definition Map）是 Apollo 自动驾驶平台的高精地图模块，提供地图数据的加载、存储与查询能力。模块采用**门面 + 实现分离**架构：

- `HDMap` 是对外暴露的公共接口（门面类），所有方法直接委托给内部成员 `HDMapImpl`
- `HDMapImpl` 负责实际的地图加载、索引构建与空间查询
- `HDMapUtil` 提供全局地图单例管理（`BaseMap` / `SimMap`），支持线程安全的懒加载与热重载
- `hdmap_common.h` 定义了所有地图要素的信息类（`LaneInfo`、`JunctionInfo` 等）及 KD 树索引类型

地图数据来源支持两种格式：Protobuf 二进制文件（`.bin`）和 OpenDRIVE XML 文件（`.xml`），通过适配器 `OpendriveAdapter` 统一转换为 `Map` protobuf 对象。

空间查询基于 **AABoxKDTree2d**（轴对齐包围盒 KD 树）实现，对线段型要素（车道、信号灯、停止标志等）和多边形型要素（路口、人行横道、停车区等）分别构建不同的 KD 树，支持按半径/朝向/前方距离等多种方式检索。

## 核心类

### HDMap

公共接口类，持有 `HDMapImpl` 实例，将所有查询操作透传至实现层。

```cpp
class HDMap {
 public:
  int LoadMapFromFile(const std::string& map_filename);
  int LoadMapFromProto(const Map& map_proto);

  // 按 ID 查询各类地图要素
  LaneInfoConstPtr GetLaneById(const Id& id) const;
  JunctionInfoConstPtr GetJunctionById(const Id& id) const;
  SignalInfoConstPtr GetSignalById(const Id& id) const;
  // ... 其余 Get*ById 方法同理

  // 空间范围查询
  int GetLanes(const apollo::common::PointENU& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  int GetNearestLane(const apollo::common::PointENU& point,
                     LaneInfoConstPtr* nearest_lane,
                     double* nearest_s, double* nearest_l) const;
  int GetNearestLaneWithHeading(const apollo::common::PointENU& point,
                                double distance, double central_heading,
                                double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  int GetRoadBoundaries(const apollo::common::PointENU& point, double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const;
  int GetLocalMap(const apollo::common::PointENU& point,
                  const std::pair<double, double>& range, Map* local_map) const;

 private:
  HDMapImpl impl_;
};
```

### HDMapImpl

地图的实际实现类，负责加载数据、构建哈希表索引和 KD 树。

```cpp
class HDMapImpl {
 public:
  using LaneTable = std::unordered_map<std::string, std::shared_ptr<LaneInfo>>;
  using JunctionTable = std::unordered_map<std::string, std::shared_ptr<JunctionInfo>>;
  // ... 其余 Table 类型定义

  int LoadMapFromFile(const std::string& map_filename);
  int LoadMapFromProto(const Map& map_proto);
  // 与 HDMap 完全对应的查询接口（含 PointENU 和 Vec2d 两个重载版本）

 private:
  Map map_;
  LaneTable lane_table_;
  JunctionTable junction_table_;
  // ... 其余要素哈希表

  std::vector<LaneSegmentBox> lane_segment_boxes_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;
  // ... 其余 KD 树

  void BuildLaneSegmentKDTree();
  void BuildJunctionPolygonKDTree();
  // ... 其余 Build* 方法

  template <class KDTree>
  static int SearchObjects(const apollo::common::math::Vec2d& center,
                           double radius, const KDTree& kdtree,
                           std::vector<std::string>* results);
};
```

### HDMapUtil

全局地图单例管理工具类（不可实例化），线程安全，采用双重检查锁定（Double-Checked Locking）懒加载模式。

```cpp
class HDMapUtil {
 public:
  static const HDMap* BaseMapPtr();                              // 获取基准地图指针
  static const HDMap* BaseMapPtr(const relative_map::MapMsg& map_msg);  // 相对地图
  static const HDMap& BaseMap();                                 // 获取基准地图引用（失败则 fatal）
  static const HDMap* SimMapPtr();                               // 获取仿真地图指针
  static const HDMap& SimMap();                                  // 获取仿真地图引用
  static bool ReloadMaps();                                      // 重载全部地图
  static bool ReloadBaseMap();                                   // 仅重载基准地图

 private:
  static std::unique_ptr<HDMap> base_map_;
  static std::mutex base_map_mutex_;
  static std::unique_ptr<HDMap> sim_map_;
  static std::mutex sim_map_mutex_;
};
```

### LaneInfo

车道信息类，存储车道中心线的采样点、方向、曲率、累积弧长 s 值以及与各类地图要素的重叠关系。内部构建 `LaneSegmentKDTree` 以支持高效的空间查询。

```cpp
class LaneInfo {
 public:
  explicit LaneInfo(const Lane& lane);
  const Id& id() const;
  double Heading(double s) const;
  double Curvature(double s) const;
  double total_length() const;
  void GetWidth(double s, double* left_width, double* right_width) const;
  bool IsOnLane(const apollo::common::math::Vec2d& point) const;
  apollo::common::PointENU GetSmoothPoint(double s) const;
  bool GetProjection(const apollo::common::math::Vec2d& point,
                     double* accumulate_s, double* lateral) const;
};
```

### ObjectWithAABox

通用模板包装类，为地图要素附加轴对齐包围盒（AABox2d），用于 KD 树索引。

```cpp
template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const apollo::common::math::AABox2d& aabox,
                  const Object* object, const GeoObject* geo_object, int id);
  double DistanceTo(const apollo::common::math::Vec2d& point) const;
};
```

常用别名：

- `LaneSegmentBox` = `ObjectWithAABox<LaneInfo, LineSegment2d>`
- `JunctionPolygonBox` = `ObjectWithAABox<JunctionInfo, Polygon2d>`
- `SignalSegmentBox` = `ObjectWithAABox<SignalInfo, LineSegment2d>`

## 核心函数

### 地图加载

| 函数 | 说明 |
|------|------|
| `HDMap::LoadMapFromFile` | 从文件加载地图，支持 `.bin`（protobuf）和 `.xml`（OpenDRIVE）格式 |
| `HDMap::LoadMapFromProto` | 从 protobuf `Map` 对象加载地图 |
| `HDMapImpl::LoadMapFromProto` | 遍历 `Map` 中所有要素，构建哈希表索引，然后为每种要素类型构建对应的 KD 树 |

### 按 ID 查询

通过哈希表 O(1) 查找，支持 15 种要素类型：

| 函数 | 返回类型 |
|------|---------|
| `GetLaneById` | `LaneInfoConstPtr` |
| `GetJunctionById` | `JunctionInfoConstPtr` |
| `GetSignalById` | `SignalInfoConstPtr` |
| `GetCrosswalkById` | `CrosswalkInfoConstPtr` |
| `GetStopSignById` | `StopSignInfoConstPtr` |
| `GetYieldSignById` | `YieldSignInfoConstPtr` |
| `GetClearAreaById` | `ClearAreaInfoConstPtr` |
| `GetSpeedBumpById` | `SpeedBumpInfoConstPtr` |
| `GetOverlapById` | `OverlapInfoConstPtr` |
| `GetRoadById` | `RoadInfoConstPtr` |
| `GetParkingSpaceById` | `ParkingSpaceInfoConstPtr` |
| `GetPNCJunctionById` | `PNCJunctionInfoConstPtr` |
| `GetRSUById` | `RSUInfoConstPtr` |
| `GetAreaById` | `AreaInfoConstPtr` |
| `GetBarrierGateById` | `BarrierGateInfoConstPtr` |

### 空间范围查询

基于 KD 树的近邻搜索，核心模板函数 `SearchObjects` 在 KD 树中以给定中心点和半径检索命中要素 ID，再通过哈希表还原为 `InfoConstPtr`：

```cpp
template <class KDTree>
static int SearchObjects(const Vec2d& center, double radius,
                         const KDTree& kdtree,
                         std::vector<std::string>* results);
```

### 车道最近点查询

| 函数 | 说明 |
|------|------|
| `GetNearestLane` | 获取最近车道（无距离限制） |
| `GetNearestLaneWithDistance` | 获取搜索半径内的最近车道 |
| `GetNearestLaneWithHeading` | 按位姿（位置 + 航向角）获取最近车道 |
| `GetLanesWithHeading` | 按位姿获取所有匹配车道 |

返回的 `nearest_s` 为沿车道中心线的纵向偏移，`nearest_l` 为横向偏移。

### 特殊查询

| 函数 | 说明 |
|------|------|
| `GetForwardNearestSignalsOnLane` | 沿车道前方搜索最近信号灯 |
| `GetForwardNearestBarriersOnLane` | 沿车道前方搜索最近道闸 |
| `GetStopSignAssociatedStopSigns` | 获取同路口关联的其他停车标志 |
| `GetStopSignAssociatedLanes` | 获取停车标志关联的车道 |
| `GetRoadBoundaries` | 获取搜索范围内的道路边界和路口 |
| `GetRoi` | 获取感兴趣区域（道路 + 多边形） |
| `GetLocalMap` | 获取指定区域的裁剪子地图 |

### HDMapUtil 工具函数

| 函数 | 说明 |
|------|------|
| `BaseMapFile` | 获取基准地图文件路径（优先使用 `FLAGS_test_base_map_filename`） |
| `SimMapFile` | 获取仿真地图文件路径 |
| `RoutingMapFile` | 获取路由地图文件路径 |
| `CreateMap` | 创建 `HDMap` 实例并加载地图文件 |
| `MakeMapId` | 字符串转 `Map Id` 对象 |

## 配置

地图相关配置通过 gflags 全局参数控制：

| Flag | 说明 |
|------|------|
| `FLAGS_map_dir` | 地图文件所在目录 |
| `FLAGS_base_map_filename` | 基准地图文件名（支持 `\|` 分隔多个候选） |
| `FLAGS_test_base_map_filename` | 测试用基准地图文件名（为空时使用 `base_map_filename`） |
| `FLAGS_sim_map_filename` | 仿真地图文件名 |
| `FLAGS_routing_map_filename` | 路由地图文件名 |
| `FLAGS_use_navigation_mode` | 是否使用导航模式（影响地图加载策略） |
| `FLAGS_end_way_point_filename` | 终点航点文件名 |
| `FLAGS_default_routing_filename` | 默认路由文件名 |
| `FLAGS_park_go_routing_filename` | 泊车路由文件名 |

文件查找支持 `|` 分隔的多候选机制，`FindFirstExist` 会返回目录中第一个存在的文件。

## 调用关系

```text
规划 / 控制 / 感知模块
        │
        ▼
    HDMapUtil (全局单例管理)
        │
        ├── BaseMapPtr() / SimMapPtr()  ←─ 懒加载，双重检查锁
        │       │
        │       ▼
        │    CreateMap()
        │       │
        │       ▼
        │    HDMap::LoadMapFromFile / LoadMapFromProto
        │       │
        │       ▼
        │    HDMapImpl::LoadMapFromProto
        │       ├── 构建哈希表索引（lane_table_ 等 15 张表）
        │       └── 构建 KD 树索引（Build* 方法族）
        │
        ▼
    HDMap (门面)
        │  所有 Get* / 查询方法
        ▼
    HDMapImpl (实现)
        │
        ├── Get*ById → 哈希表查找
        ├── GetLanes / GetJunctions / ... → KD 树半径搜索 → SearchObjects
        ├── GetNearestLane* → KD 树 + 距离/朝向过滤
        └── GetForwardNearest* → 前方车道追踪 + 要素匹配

    hdmap_common (数据结构层)
        ├── *Info 类 → 解析 protobuf 并缓存几何信息
        ├── ObjectWithAABox → 要素 + 包围盒包装
        └── *KDTree 类型别名 → AABoxKDTree2d 特化
```
