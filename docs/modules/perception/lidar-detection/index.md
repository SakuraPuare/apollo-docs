---
title: "Lidar Detection 激光雷达检测"
---

# Lidar Detection 激光雷达检测

> 源码路径：`modules/perception/lidar_detection/`

## 概述

`lidar_detection` 模块是 Apollo 感知系统的激光雷达 3D 目标检测子模块，负责从点云数据中检测前景障碍物。采用插件架构，通过 `BaseLidarDetector` 接口抽象检测算法，支持多种深度学习检测器实现。由 `LidarDetectionComponent` 驱动，接收预处理后的 `LidarFrameMessage`，输出检测结果。

## 架构

```text
LidarFrameMessage (预处理后的点云帧)
    │
    ▼
LidarDetectionComponent::Proc()
    │
    ├── BaseLidarDetector::Detect()     ← 插件检测器
    │     ├── PointPillarsDetection
    │     ├── CenterPointDetection
    │     ├── MaskPillarsDetection
    │     └── CNNSegmentation
    │
    ├── ObjectBuilder::Build()          ← 构建 Object 属性
    │
    └── Writer->Write(LidarFrameMessage)  ← 发布结果
```

## 核心类

### LidarDetectionComponent

CyberRT 组件，驱动激光雷达检测流程。

```cpp
class LidarDetectionComponent : public cyber::Component<LidarFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& in_message);

  std::string sensor_name_;
  bool use_object_builder_ = true;
  std::shared_ptr<BaseLidarDetector> detector_;
  ObjectBuilder builder_;
  std::shared_ptr<cyber::Writer<onboard::LidarFrameMessage>> writer_;
};

CYBER_REGISTER_COMPONENT(LidarDetectionComponent);
```

**源码**：`modules/perception/lidar_detection/lidar_detection_component.h`

### BaseLidarDetector

激光雷达检测器抽象基类，所有检测器插件实现此接口。

```cpp
class BaseLidarDetector {
 public:
  virtual ~BaseLidarDetector() = default;
  virtual bool Init(const LidarDetectorInitOptions& options) = 0;
  virtual bool Detect(const LidarDetectorOptions& options,
                      LidarFrame* frame) = 0;
  virtual std::string Name() const = 0;
};

PERCEPTION_REGISTER_REGISTERER(BaseLidarDetector);
```

**源码**：`modules/perception/lidar_detection/interface/base_lidar_detector.h`

**插件注册宏**：`PERCEPTION_REGISTER_LIDARDETECTOR(name)`

## 核心函数

### LidarDetectionComponent::Init()

```cpp
bool LidarDetectionComponent::Init() {
  // 1. 加载配置
  LidarDetectionComponentConfig comp_config;
  GetProtoConfig(&comp_config);
  sensor_name_ = comp_config.sensor_name();
  use_object_builder_ = comp_config.use_object_builder();

  // 2. 创建输出 Writer
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  // 3. 通过插件机制创建检测器
  std::string detector_name = plugin_param.name();
  BaseLidarDetector* detector =
      BaseLidarDetectorRegisterer::GetInstanceByName(detector_name);
  detector_.reset(detector);
  detector_->Init(detection_init_options);

  // 4. 初始化 ObjectBuilder（可选）
  if (use_object_builder_) {
    builder_.Init(builder_init_options);
  }
}
```

### LidarDetectionComponent::InternalProc()

```cpp
bool LidarDetectionComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& in_message) {
  // 1. 执行检测
  LidarDetectorOptions detection_options;
  detector_->Detect(detection_options, in_message->lidar_frame_.get());

  // 2. 构建 Object 属性（包围盒、形状等）
  if (use_object_builder_) {
    ObjectBuilderOptions builder_options;
    builder_.Build(builder_options, in_message->lidar_frame_.get());
  }
  return true;
}
```

**职责**：调用检测器检测点云中的障碍物，然后通过 `ObjectBuilder` 补充 Object 属性。

## 检测器实现

| 子目录 | 说明 |
|--------|------|
| `point_pillars_detection/` | PointPillars 算法，将点云编码为柱状特征后用 2D CNN 检测 |
| `center_point_detection/` | CenterPoint 算法，基于中心点的 3D 目标检测 |
| `mask_pillars_detection/` | MaskPillars 算法，PointPillars 的变体 |
| `cnn_segmentation/` | CNN 语义分割，将点云投影为鸟瞰图后做语义分割 |

## 配置

通过 `LidarDetectionComponentConfig` protobuf 加载：

| 字段 | 类型 | 说明 |
|------|------|------|
| sensor_name | string | 传感器名称（如 velodyne64） |
| output_channel_name | string | 输出通道名 |
| use_object_builder | bool | 是否使用 ObjectBuilder |
| plugin_param.name | string | 检测器插件名 |
| plugin_param.config_path | string | 检测器配置文件路径 |
| plugin_param.config_file | string | 检测器配置文件名 |

## 调用关系

- **上游**：点云预处理模块发布 `LidarFrameMessage`
- **下游**：检测结果发布到 `output_channel_name`，由 lidar_tracking 等模块消费
- **依赖**：`ObjectBuilder`（Object 属性构建）、`LidarFrame`（点云帧数据结构）
