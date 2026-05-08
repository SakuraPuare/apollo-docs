---
title: "相对地图"
---

# 相对地图模块 (Relative Map)

> 源码路径: `modules/map/relative_map/`

## 概述

相对地图模块是 Apollo 导航模式下的地图生成组件。它不依赖高精地图文件，而是根据导航路径信息、感知车道线和车辆定位数据，实时生成车辆前方约 250 米的局部地图。生成的地图以 `MapMsg` 消息发布，供下游规划模块使用。

模块采用 Cyber RT 的 `TimerComponent` 驱动，按周期调用 `Proc()` 完成一次地图刷新。数据流为：接收导航路径 / 感知障碍物 / 底盘 / 定位四路消息，经过坐标变换和路径融合后输出包含车道、车道边界和导航路径的相对地图。

## 核心类

### `RelativeMapComponent`

继承 `cyber::TimerComponent`，是模块的 Cyber RT 入口。负责初始化 Reader/Writer 并在 `Proc()` 中调用 `RelativeMap::Process()` 生成地图并发布。

```cpp
class RelativeMapComponent final : public ::apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
};
```

**订阅话题：**

| 话题类型 | 说明 |
|---|---|
| `PerceptionObstacles` | 感知障碍物（含车道线） |
| `Chassis` | 底盘信息 |
| `LocalizationEstimate` | 定位信息 |
| `NavigationInfo` | 导航路径 |

**发布话题：** `MapMsg`（相对地图）

### `RelativeMap`

模块核心逻辑类。持有 `NavigationLane` 实例，负责接收上游数据并通过 `CreateMapFromNavigationLane()` 生成地图。所有数据回调均通过 `navigation_lane_mutex_` 保证线程安全。

```cpp
class RelativeMap {
 public:
  bool Process(MapMsg* const map_msg);
  void OnPerception(const perception::PerceptionObstacles& perception_obstacles);
  void OnChassis(const canbus::Chassis& chassis);
  void OnLocalization(const localization::LocalizationEstimate& localization);
  void OnNavigationInfo(const NavigationInfo& navigation_info);
};
```

### `NavigationLane`

导航车道生成类，是路径融合和地图构建的核心。处理流程：

1. 接收 `NavigationInfo` 中的多条导航路径（ENU 坐标系）
2. 截取车辆前方约 250 m 的路径段，转换为 FLU（车辆本体）坐标系
3. 将当前车道导航路径与感知车道中心线融合
4. 根据导航路径和感知车道宽度生成 HD Map 格式的车道
5. 输出 `MapMsg`

```cpp
class NavigationLane {
 public:
  void UpdateNavigationInfo(const NavigationInfo& navigation_info);
  bool GeneratePath();
  bool CreateMap(const MapGenerationParam& map_config, MapMsg* const map_msg) const;
};
```

**关键类型定义：**

```cpp
// (导航线索引, 左半宽, 右半宽, 导航路径指针)
typedef std::tuple<int, double, double, std::shared_ptr<NavigationPath>> NaviPathTuple;
typedef std::pair<int, int> StitchIndexPair;   // 循环路径拼接索引对
typedef std::pair<int, double> ProjIndexPair;  // 投影索引与距离
```

## 核心函数

### `RelativeMap::Process`

```cpp
bool RelativeMap::Process(MapMsg* const map_msg);
```

定时回调主函数。加锁后调用 `CreateMapFromNavigationLane()`，更新车辆状态、感知信息，生成导航路径并构建地图。

### `RelativeMap::CreateMapFromNavigationLane`

```cpp
bool RelativeMap::CreateMapFromNavigationLane(MapMsg* map_msg);
```

地图生成管线：更新车辆状态 -> 更新感知信息到 `NavigationLane` -> `GeneratePath()` 生成路径 -> `CreateMap()` 构建地图。

### `NavigationLane::GeneratePath`

```cpp
bool NavigationLane::GeneratePath();
```

路径生成核心。根据 `lane_source` 配置选择数据源：

- **OFFLINE_GENERATED**：从 `NavigationInfo` 的多条导航线生成多条路径，按 y 坐标从左到右排序，选择距车辆最近的路径作为当前车道，与感知车道线融合，并计算相邻路径的车道宽度
- **感知模式**：仅基于感知车道线（三次多项式拟合）生成一条路径

优先级：融合 > 导航线 > 感知车道线。

### `NavigationLane::ConvertNavigationLineToPath`

```cpp
bool NavigationLane::ConvertNavigationLineToPath(const int line_index,
                                                  common::Path* const path);
```

将 ENU 坐标系下的导航路径截取前方 250 m 段并转换为 FLU 坐标系。支持循环路径拼接（`FLAGS_enable_cyclic_rerouting`）。

### `NavigationLane::ConvertLaneMarkerToPath`

```cpp
void NavigationLane::ConvertLaneMarkerToPath(const perception::LaneMarkers& lane_marker,
                                              common::Path* const path);
```

取左右车道线三次多项式系数的均值生成车道中心线路径，同时更新感知车道宽度。路径长度根据车速动态调整（`ratio_navigation_lane_len_to_speed`）。

### `NavigationLane::MergeNavigationLineAndLaneMarker`

```cpp
void NavigationLane::MergeNavigationLineAndLaneMarker(const int line_index,
                                                       common::Path* const path);
```

将导航路径与感知车道中心线按权重（`lane_marker_weight`）加权融合。超出感知范围的首尾部分仅融合 y 坐标。

### `NavigationLane::CreateMap`

```cpp
bool NavigationLane::CreateMap(const MapGenerationParam& map_config,
                                MapMsg* const map_msg) const;
```

基于 `NaviPathTuple` 列表构建 HD Map 格式的车道（中心线、左右边界、车道采样宽度），设置道路边界和车道邻接关系。内部通过 `CreateSingleLaneMap()` 辅助函数为每条路径创建单个车道。

## 配置

配置文件由 `FLAGS_relative_map_config_filename` 指定，对应 proto 定义为 `RelativeMapConfig`：

```protobuf
message RelativeMapConfig {
  optional MapGenerationParam map_param = 1;
  optional NavigationLaneConfig navigation_lane = 2;
}
```

### `MapGenerationParam`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `default_left_width` | 1.75 m | 默认车道左半宽 |
| `default_right_width` | 1.75 m | 默认车道右半宽 |
| `default_speed_limit` | 29.06 m/s (65 mph) | 默认限速 |

### `NavigationLaneConfig`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `lane_source` | OFFLINE_GENERATED | 车道数据源：`PERCEPTION` 或 `OFFLINE_GENERATED` |
| `max_len_from_navigation_line` | 250 m | 导航线截取最大长度 |
| `min_len_for_navigation_lane` | 150 m | 生成导航车道最小长度 |
| `max_len_for_navigation_lane` | 250 m | 生成导航车道最大长度 |
| `ratio_navigation_lane_len_to_speed` | 8.0 | 车道长度与车速之比 |
| `max_distance_to_navigation_line` | 15 m | 车辆到导航线最大匹配距离 |
| `min_view_range_to_use_lane_marker` | 0.5 m | 使用车道线的最小视距 |
| `min_lane_half_width` | 1.5 m | 最小车道半宽 |
| `max_lane_half_width` | 2.0 m | 最大车道半宽 |
| `lane_marker_weight` | 0.1 | 感知车道线融合权重 |

### 关键 gflags

| Flag | 说明 |
|---|---|
| `FLAGS_use_navigation_mode` | 是否启用导航模式（必须为 true） |
| `FLAGS_enable_cyclic_rerouting` | 是否启用循环路径拼接 |
| `FLAGS_relative_map_generate_left_boundray` | 是否生成左侧车道边界 |

## 调用关系

```text
RelativeMapComponent::Init()
  └─ RelativeMap::Init()
       └─ NavigationLane::SetConfig() / SetVehicleStateProvider() / SetDefaultWidth()

RelativeMapComponent::Proc()  [定时触发]
  └─ RelativeMap::Process()
       └─ CreateMapFromNavigationLane()
            ├─ VehicleStateProvider::Update(localization, chassis)
            ├─ NavigationLane::UpdatePerception(perception)
            ├─ NavigationLane::GeneratePath()
            │    ├─ ConvertNavigationLineToPath()  [OFFLINE_GENERATED 模式]
            │    │    └─ UpdateProjectionIndex() / ENU→FLU 坐标转换
            │    ├─ MergeNavigationLineAndLaneMarker()  [融合当前车道]
            │    └─ ConvertLaneMarkerToPath()  [纯感知模式]
            └─ NavigationLane::CreateMap()
                 └─ CreateSingleLaneMap()  [为每条路径构建车道]

[数据回调 - 由 Cyber RT Reader 触发]
  OnNavigationInfo() → NavigationLane::UpdateNavigationInfo()
  OnPerception()     → 本地缓存 + NavigationLane::UpdatePerception()
  OnChassis()        → 本地缓存
  OnLocalization()   → 本地缓存
```
