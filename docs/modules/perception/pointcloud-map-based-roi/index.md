---
title: "基于地图的点云 ROI"
---

# 基于地图的点云 ROI

> 源码路径: `modules/perception/pointcloud_map_based_roi/`

## 概述

本模块基于高精地图（HD Map）对激光雷达点云进行 ROI（Region of Interest）过滤，将点云中处于道路区域内的点标记出来，剔除道路外的无关点。模块接收上游 `LidarFrameMessage`，通过 `MapManager` 从高精地图获取道路多边形结构，再由 `BaseROIFilter` 的具体实现完成点云过滤，最终将结果写入输出通道。

核心处理流程为：组件初始化 -> 接收 LidarFrame -> 更新地图结构 -> 执行 ROI 过滤 -> 发布结果。

## 核心类

### PointCloudMapROIComponent

Cyber Component，是整个模块的入口。继承 `cyber::Component<LidarFrameMessage>`，负责初始化地图管理器和 ROI 过滤器插件，并在每帧调用中串联地图更新与点云过滤。

```cpp
class PointCloudMapROIComponent final
    : public cyber::Component<LidarFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;
 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& message);
  std::shared_ptr<cyber::Writer<LidarFrameMessage>> writer_;
  std::string output_channel_name_;
  bool use_map_manager_;
  MapManager map_manager_;
  std::shared_ptr<BaseROIFilter> roi_filter_;
};
```

### MapManager

管理高精地图数据的获取与更新。通过 `HDMapInput` 查询车辆周围指定距离内的道路多边形（道路、交叉口、空洞区域、道路边界），写入 `LidarFrame::hdmap_struct`。

```cpp
class MapManager {
 public:
  bool Init(const MapManagerInitOptions& options = MapManagerInitOptions());
  bool Update(const MapManagerOptions& options, LidarFrame* frame);
  bool QueryPose(Eigen::Affine3d* sensor2world_pose) const;
  std::string Name() const { return "MapManager"; }
 private:
  LidarFrame* cached_frame_ = nullptr;
  map::HDMapInput* hdmap_input_ = nullptr;
  bool update_pose_ = false;
  double roi_search_distance_ = 80.0;
};
```

### BaseROIFilter

ROI 过滤器的抽象基类，通过 Cyber 插件机制注册。定义了 `Init`、`Filter`、`Name` 三个纯虚函数，具体实现由子类提供。

```cpp
class BaseROIFilter {
 public:
  virtual bool Init(const ROIFilterInitOptions& options = ROIFilterInitOptions()) = 0;
  virtual bool Filter(const ROIFilterOptions& options, LidarFrame* frame) = 0;
  virtual std::string Name() const = 0;
};
```

### HdmapROIFilter

基于高精地图多边形的 ROI 过滤器。将世界坐标系下的道路/交叉口多边形转换到传感器局部坐标系，使用 `Bitmap2D` 栅格化后逐点判断是否在 ROI 内。支持将结果写入 `ROIService` 供下游模块使用。

```cpp
class HdmapROIFilter : public BaseROIFilter {
 public:
  bool Init(const ROIFilterInitOptions& options) override;
  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;
  std::string Name() const override { return "HdmapROIFilter"; }
 private:
  void TransformFrame(...);
  bool FilterWithPolygonMask(...);
  bool Bitmap2dFilter(...);
  double range_ = 120.0;
  double cell_size_ = 0.25;
  double extend_dist_ = 0.0;
  bool no_edge_table_ = false;
  Bitmap2D bitmap_;
};
```

### ROIServiceFilter

从 `SceneManager` 获取已有的 `ROIService` 内容（由 `HdmapROIFilter` 写入），直接用服务中的位图数据逐点判断 ROI。适用于不需要重新计算地图多边形、仅复用已有 ROI 结果的场景。

```cpp
class ROIServiceFilter : public BaseROIFilter {
 public:
  bool Init(const ROIFilterInitOptions& options) override;
  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;
  std::string Name() const override { return "ROIServiceFilter"; }
 private:
  ROIServicePtr roi_service_ = nullptr;
  ROIServiceContent roi_service_content_;
};
```

### Bitmap2D

二维位图栅格结构，用于高效地表示和查询 ROI 区域。支持单点 Set/Check 和区间 Set/Reset，内部使用 `uint64_t` 位运算加速。

### PolygonScanCvter

多边形扫描线转换器，将多边形沿主方向进行扫描线分割，输出每条扫描线上的有效区间，供 `Bitmap2D` 填充使用。支持边表（ET）和活动边表（AET）优化。

## 核心函数

### PointCloudMapROIComponent::Init

```cpp
bool PointCloudMapROIComponent::Init();
```

读取 `PointCloudMapROIComponentConfig` 配置，创建输出 Writer，初始化 `SceneManager`，按配置初始化 `MapManager`，并通过插件机制创建 `BaseROIFilter` 实例。

### PointCloudMapROIComponent::InternalProc

```cpp
bool PointCloudMapROIComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& message);
```

每帧核心处理逻辑：先调用 `MapManager::Update` 更新地图结构，再调用 `roi_filter_->Filter` 过滤点云。若过滤失败，将全部点云索引作为 ROI 结果（即不过滤）。

### MapManager::Update

```cpp
bool MapManager::Update(const MapManagerOptions& options, LidarFrame* frame);
```

从 `HDMapInput` 查询车辆位置周围 `roi_search_distance_` 范围内的 ROI 多边形，填充到 `frame->hdmap_struct` 中（包括 `road_polygons`、`junction_polygons`、`hole_polygons`、`road_boundary`）。

### HdmapROIFilter::Filter

```cpp
bool HdmapROIFilter::Filter(const ROIFilterOptions& options, LidarFrame* frame);
```

将 `hdmap_struct` 中的道路和交叉口多边形转换到局部坐标系，通过 `PolygonMask` 将多边形栅格化到 `Bitmap2D`，再用 `Bitmap2dFilter` 逐点查询，输出 ROI 索引并将对应点标记为 `LidarPointLabel::ROI`。若 `set_roi_service_` 为 true，还会将位图写入 `ROIService`。

### Bitmap2dFilter

```cpp
bool HdmapROIFilter::Bitmap2dFilter(
    const base::PointFCloudPtr& in_cloud,
    const Bitmap2D& bitmap, base::PointIndices* roi_indices);
```

首先检查车辆原点是否在 ROI 内，然后遍历局部坐标系下的点云，通过 `Bitmap2D::Check` 判断每个点是否位于 ROI 内，收集索引。

## 配置

### PointCloudMapROIComponentConfig

| 字段 | 类型 | 说明 |
|------|------|------|
| `output_channel_name` | string | 输出通道名称 |
| `use_map_manager` | bool | 是否使用地图管理器 |
| `enable_hdmap` | bool | 是否启用高精地图 |
| `plugin_param` | PluginParam | ROI 过滤器插件参数（名称、配置路径） |
| `map_manager_config_path` | string | 地图管理器配置路径 |
| `map_manager_config_file` | string | 地图管理器配置文件名 |

### MapManagerConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `update_pose` | bool | false | 是否更新位姿 |
| `roi_search_distance` | double | 80.0 | ROI 搜索半径（米） |

### HDMapRoiFilterConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `range` | double | 120.0 | 过滤范围（米），Bitmap2D 覆盖 [-range, range] |
| `cell_size` | double | 0.25 | 栅格单元大小（米） |
| `extend_dist` | double | 0.0 | 多边形扩展距离（米） |
| `no_edge_table` | bool | false | 是否禁用边表优化 |
| `set_roi_service` | bool | false | 是否将结果写入 ROIService |

## 调用关系

```text
PointCloudMapROIComponent::Init()
  ├── SceneManager::Init()
  ├── MapManager::Init()
  │     └── HDMapInput::Init()
  └── PluginManager::CreateInstance<BaseROIFilter>()
        ├── HdmapROIFilter::Init()
        │     └── Bitmap2D::Init()
        └── ROIServiceFilter::Init()
              └── SceneManager::Service("ROIService")

PointCloudMapROIComponent::Proc()
  └── InternalProc()
        ├── MapManager::Update()
        │     └── HDMapInput::GetRoiHDMapStruct()
        └── roi_filter_->Filter()
              ├── HdmapROIFilter::Filter()
              │     ├── TransformFrame()  — 世界坐标转局部坐标
              │     ├── FilterWithPolygonMask()
              │     │     ├── DrawPolygonsMask()  — 多边形栅格化
              │     │     │     └── PolygonScanCvter::ScansCvt()
              │     │     └── Bitmap2dFilter()  — 逐点 ROI 查询
              │     └── ROIService::UpdateServiceContent()
              └── ROIServiceFilter::Filter()
                    └── ROIService::QueryIsPointInROI()
```
