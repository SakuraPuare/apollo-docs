---
title: "点云地面检测"
---

# 点云地面检测

> 源码路径: `modules/perception/pointcloud_ground_detection/`

## 概述

点云地面检测模块负责从激光雷达点云中识别地面点与非地面点。模块采用插件化架构，通过 `BaseGroundDetector` 基类定义统一接口，支持两种检测器实现：`GroundServiceDetector` 基于场景管理器的地表服务进行查询，适用于已有地面模型的场景；`SpatioTemporalGroundDetector` 基于平面拟合算法进行检测，支持 ROI 过滤、语义信息融合和时域地面高度平均。检测结果写入 `LidarFrame` 的 `non_ground_indices` 和 `points_height` 字段，供下游感知模块使用。

## 核心类

### BaseGroundDetector

抽象基类，定义地面检测器的统一接口，位于 `interface/base_ground_detector.h`。

```cpp
class BaseGroundDetector {
 public:
  virtual bool Init(const GroundDetectorInitOptions& options =
                        GroundDetectorInitOptions()) = 0;
  virtual bool Detect(const GroundDetectorOptions& options,
                      LidarFrame* frame) = 0;
  virtual std::string Name() const = 0;
};
```

通过 `PERCEPTION_REGISTER_REGISTERER` 宏注册工厂，子类使用 `PERCEPTION_REGISTER_GROUNDDETECTOR` 宏完成注册。

### PointCloudGroundDetectComponent

Cyber RT 组件，封装地面检测流程，位于 `pointcloud_ground_detection_component.h`。继承 `cyber::Component<LidarFrameMessage>`，接收激光雷达帧消息，调用插件化的地面检测器，并将结果通过 `Writer` 发送到输出 channel。

```cpp
class PointCloudGroundDetectComponent
    : public cyber::Component<LidarFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;
 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& message);
  std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;
  std::string output_channel_name_;
  BaseGroundDetector* ground_detector_;
};
```

### GroundServiceDetector

基于场景管理器地表服务的检测器，位于 `ground_detector/ground_service_detector/`。从 `SceneManager` 获取 `GroundService`，对每个点查询到地面距离，距离大于阈值则归类为非地面点。

```cpp
class GroundServiceDetector : public BaseGroundDetector {
 private:
  GroundServicePtr ground_service_ = nullptr;
  GroundServiceContent ground_service_content_;
  double ground_threshold_ = 0.25;
};
```

### SpatioTemporalGroundDetector

基于平面拟合的时空地面检测器，位于 `ground_detector/spatio_temporal_ground_detector/`。使用 `PlaneFitGroundDetector` 进行网格化平面拟合，支持近/中/远距离差异化阈值、语义地面信息融合、单帧/世界坐标系两种检测模式，以及时域地面高度滑动平均。

```cpp
class SpatioTemporalGroundDetector : public BaseGroundDetector {
 private:
  algorithm::PlaneFitGroundDetectorParam* param_ = nullptr;
  algorithm::PlaneFitGroundDetector* pfdetector_ = nullptr;
  bool use_roi_ = true;
  bool use_ground_service_ = false;
  bool use_semantic_ground_ = false;
  float ground_thres_ = 0.25f;
  int grid_size_ = 256;
  std::list<float> origin_ground_z_array_;
  const size_t ground_z_average_frame = 5;
};
```

## 核心函数

### PointCloudGroundDetectComponent::Init

从配置文件读取 `PointCloudGroundDetectComponentConfig`，创建输出 `Writer`，通过工厂注册器实例化指定的地面检测器插件并调用其 `Init`。

### PointCloudGroundDetectComponent::Proc

接收 `LidarFrameMessage`，调用 `InternalProc` 执行地面检测，成功后将消息写入输出 channel。

### GroundServiceDetector::Detect

遍历世界坐标系点云，通过 `GroundService::QueryPointToGroundDistance` 获取每个点到地面的距离，距离超过 `ground_threshold_` 的点加入 `non_ground_indices`，其余标记为 `GROUND`。

### SpatioTemporalGroundDetector::Detect

核心检测流程：

1. 根据 `use_roi_` 决定使用 ROI 索引还是全量点云
2. 拷贝点坐标到内部缓冲区，同时统计近距离语义地面点的平均高度
3. 调用 `PlaneFitGroundDetector::Detect` 进行平面拟合
4. 遍历所有点，根据拟合结果和分段阈值（近/中/远距离）判定地面/非地面
5. 计算时域滑动平均的原始地面高度 `original_ground_z`
6. 若启用 `use_ground_service_`，将检测到的地面平面参数更新到 `GroundService`

## 配置

### 组件配置

`PointCloudGroundDetectComponentConfig`（proto 定义）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `output_channel_name` | string | 输出 channel 名称 |
| `plugin_param` | PluginParam | 插件参数，包含检测器名称、配置路径和配置文件 |

### GroundServiceDetector 配置

`GroundServiceDetectorConfig`：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ground_threshold` | double | 0.25 | 地面距离阈值，超过此值判定为非地面 |

### SpatioTemporalGroundDetector 配置

`SpatioTemporalGroundDetectorConfig` 主要字段：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `big_grid_size` | uint32 | 256 | 粗网格尺寸 |
| `small_grid_size` | uint32 | 64 | 细网格尺寸 |
| `roi_rad_x` / `roi_rad_y` / `roi_rad_z` | float | 120.0 / 120.0 / 100.0 | ROI 区域半径 |
| `near_range` | float | 32.0 | 近距离范围 |
| `near_z_min` / `near_z_max` | float | -3.0 / -1.0 | 采样 Z 范围 |
| `ground_thres` | float | 0.20 | 通用地面判定阈值 |
| `near_range_dist` / `near_range_ground_thres` | float | 10.0 / 0.10 | 近距离范围及其阈值 |
| `middle_range_dist` / `middle_range_ground_thres` | float | 3.0 / 0.05 | 中距离范围及其阈值 |
| `use_roi` | bool | true | 是否使用 ROI 过滤 |
| `use_semantic_ground` | bool | false | 是否融合语义地面信息 |
| `single_ground_detect` | bool | false | 是否使用单帧坐标系检测 |
| `use_ground_service` | bool | true | 是否更新地面服务 |
| `planefit_dist_thres_near` / `planefit_dist_thres_far` | float | 0.10 / 0.20 | 平面拟合近/远距离阈值 |
| `nr_smooth_iter` | uint32 | 5 | 平滑迭代次数 |
| `inliers_min_threshold` | uint32 | 4 | 最小内点数 |
| `debug_output` | bool | false | 是否输出调试文件 |

## 调用关系

```text
PointCloudGroundDetectComponent::Proc
  -> InternalProc
    -> BaseGroundDetector::Detect (多态调用)
        |
        +-- GroundServiceDetector::Detect
        |     -> GroundService::QueryPointToGroundDistance
        |
        +-- SpatioTemporalGroundDetector::Detect
              -> PlaneFitGroundDetector::Detect (平面拟合)
              -> 阈值分段判定 (近/中/远距离)
              -> GroundService::UpdateServiceContent (可选)
```

`PointCloudGroundDetectComponent` 通过 Cyber RT 消息机制接收上游 `LidarFrameMessage`，经地面检测后将结果发送到输出 channel，供下游模块（如点云分割、目标检测）使用。
