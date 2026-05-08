---
title: "Camera Tracking 相机目标跟踪"
---

# Camera Tracking 相机目标跟踪

> 源码路径：`modules/perception/camera_tracking/`

## 概述

`camera_tracking` 模块负责对相机检测到的障碍物进行多帧跟踪，为每个障碍物分配唯一的 Track ID 并维护其生命周期。接收上游 `camera_detection` 输出的 `CameraFrame`（含检测结果和特征向量），通过 `OMTObstacleTracker` 执行多目标跟踪，输出带 Track ID 的 `SensorFrameMessage`。

## 架构

```text
CameraFrame (检测结果 + 特征向量)
    │
    ▼
CameraTrackingComponent::Proc()
    │
    ├── 构建 CameraTrackingFrame
    │     ├── detected_objects (检测框)
    │     ├── feature_blob (外观特征)
    │     └── camera2world_pose
    │
    ├── BaseObstacleTracker::Process()
    │     └── OMTObstacleTracker (多目标跟踪)
    │           ├── 特征匹配
    │           ├── 卡尔曼滤波
    │           └── Track 生命周期管理
    │
    └── 输出 SensorFrameMessage (带 Track ID 的障碍物)
```

## 核心类

### CameraTrackingComponent

CyberRT 组件，驱动相机跟踪流程。

```cpp
class CameraTrackingComponent : public cyber::Component<onboard::CameraFrame> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<onboard::CameraFrame>& message) override;

 private:
  bool InternalProc(
      const std::shared_ptr<const onboard::CameraFrame>& in_message,
      std::shared_ptr<onboard::SensorFrameMessage> out_message);

  std::string output_channel_name_;
  std::shared_ptr<BaseObstacleTracker> camera_obstacle_tracker_;
  std::shared_ptr<cyber::Writer<onboard::SensorFrameMessage>> writer_;
  algorithm::SensorManager* sensor_manager_;
  OmtParam camera_tracker_config_;
};

CYBER_REGISTER_COMPONENT(CameraTrackingComponent);
```

**源码**：`modules/perception/camera_tracking/camera_tracking_component.h`

### BaseObstacleTracker

障碍物跟踪器抽象基类（定义在 `interface/` 目录）。

```cpp
class BaseObstacleTracker {
 public:
  virtual bool Init(const ObstacleTrackerInitOptions& options) = 0;
  virtual bool Process(CameraTrackingFrame* frame) = 0;
};
```

### OMTObstacleTracker

`BaseObstacleTracker` 的唯一实现——OMT（Online Multi-object Tracker）多目标跟踪器。

**源码**：`modules/perception/camera_tracking/tracking/omt_obstacle_tracker.h`

## 核心函数

### CameraTrackingComponent::Init()

```cpp
bool CameraTrackingComponent::Init() {
  // 1. 加载配置
  CameraTrackingComponentConfig config;
  GetProtoConfig(&config);

  // 2. 创建输出 Writer
  writer_ = node_->CreateWriter<SensorFrameMessage>(output_channel_name_);

  // 3. 通过插件机制创建跟踪器
  std::string tracker_name = plugin_param.name();
  camera_obstacle_tracker_.reset(
      BaseObstacleTrackerRegisterer::GetInstanceByName(tracker_name));

  // 4. 初始化跟踪器
  ObstacleTrackerInitOptions tracker_init_options;
  tracker_init_options.image_width = config.image_width();
  tracker_init_options.image_height = config.image_height();
  tracker_init_options.gpu_id = config.gpu_id();
  camera_obstacle_tracker_->Init(tracker_init_options);
}
```

### CameraTrackingComponent::InternalProc()

```cpp
bool CameraTrackingComponent::InternalProc(
    const std::shared_ptr<const onboard::CameraFrame>& in_message,
    std::shared_ptr<onboard::SensorFrameMessage> out_message) {
  // 1. 构建跟踪帧
  auto tracking_frame = std::make_shared<CameraTrackingFrame>();
  tracking_frame->detected_objects = in_message->detected_objects;
  tracking_frame->feature_blob = in_message->feature_blob;
  tracking_frame->camera2world_pose = in_message->camera2world_pose;

  // 2. 执行跟踪
  camera_obstacle_tracker_->Process(tracking_frame);

  // 3. 构建输出帧
  out_message->frame_->objects = tracking_frame->tracked_objects;
  out_message->frame_->sensor2world_pose = tracking_frame->camera2world_pose;
}
```

**职责**：将检测结果转换为跟踪帧，调用跟踪器处理，输出带 Track ID 的障碍物

**关键步骤**：

1. 从 `CameraFrame` 提取检测对象和特征向量
2. 调用 `OMTObstacleTracker::Process()` 执行多目标跟踪
3. 将跟踪结果写入 `SensorFrameMessage` 的 `Frame::objects`

## 子模块

| 子目录 | 说明 |
|--------|------|
| `tracking/` | OMT 多目标跟踪器实现 |
| `feature_extract/` | 跟踪特征提取 |
| `base/` | 跟踪基础数据结构 |
| `common/` | 跟踪通用工具 |

## 配置

通过 `CameraTrackingComponentConfig` protobuf 加载：

| 字段 | 类型 | 说明 |
|------|------|------|
| output_channel_name | string | 输出通道名 |
| plugin_param.name | string | 跟踪器插件名 |
| plugin_param.config_path | string | 跟踪器配置路径 |
| plugin_param.config_file | string | 跟踪器配置文件 |
| image_width | int | 图像宽度 |
| image_height | int | 图像高度 |
| gpu_id | int | GPU 设备 ID |

## 调用关系

- **上游**：`camera_detection` 模块输出 `CameraFrame`（含检测框和特征向量）
- **下游**：跟踪结果发布为 `SensorFrameMessage`，由多传感器融合模块消费
- **依赖**：`OMTObstacleTracker`、`SensorManager`、`CameraTrackingFrame`
