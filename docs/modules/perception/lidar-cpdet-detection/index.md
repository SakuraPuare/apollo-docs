---
title: "激光雷达候选点检测"
---

# 激光雷达候选点检测模块 (Lidar CPDet Detection)

> 源码路径: `modules/perception/lidar_cpdet_detection/`

## 概述

激光雷达候选点检测模块负责从点云帧中检测前景候选点（Candidate Points），为下游跟踪与融合提供初始目标。该模块基于 Apollo 感知插件框架设计，提供抽象基类 `BaseCPDetector` 供具体算法实现，并通过 Cyber 组件 `LidarCPDetectionComponent` 集成到在线流水线中。同时提供离线工具 `OfflineLidarCPDetection`，可直接读取 PCD 文件进行批量检测与结果输出。

整体流程为：接收 `LidarFrameMessage` -> 候选点检测 -> 目标属性构建（ObjectBuilder）-> 输出结果。

## 核心类

### BaseCPDetector

候选点检测器的抽象基类，定义了检测器的标准接口。所有具体检测算法（如 CNN 分割）需继承此类并通过注册宏注册。

```cpp
class BaseCPDetector {
 public:
  virtual bool Init(const CPDetectorInitOptions& options = CPDetectorInitOptions()) = 0;
  virtual bool Detect(const CPDetectorOptions& options, LidarFrame* frame) = 0;
  virtual std::string Name() const = 0;
};
```

通过宏 `PERCEPTION_REGISTER_LIDARDETECTOR(name)` 注册具体检测器实现类，运行时通过 `BaseCPDetectorRegisterer::GetInstanceByName` 动态获取实例。

### LidarCPDetectionComponent

Cyber 组件，继承自 `cyber::Component<LidarFrameMessage>`，封装检测器为在线可部署的组件。

```cpp
class LidarCPDetectionComponent : public cyber::Component<LidarFrameMessage> {
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  std::string sensor_name_;
  bool use_object_builder_ = true;
  std::shared_ptr<BaseCPDetector> detector_;
  ObjectBuilder builder_;
  std::string output_channel_name_;
  std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;
};
```

核心成员：

- `detector_`：通过插件机制加载的具体候选点检测器
- `builder_`：目标属性构建器，用于补充检测结果的几何属性
- `writer_`：将检测结果写入下游通道

### OfflineLidarCPDetection

离线检测工具类，用于从磁盘加载 PCD 点云文件执行检测并输出结果到文本文件。

```cpp
class OfflineLidarCPDetection {
  bool Init();
  bool TransformCloud(const base::PointFCloudPtr& local_cloud,
                      const Eigen::Affine3d& pose,
                      base::PointDCloudPtr world_cloud) const;
  bool Run();
  bool WriteObjectsForNewBenchmark(size_t frame_id, LidarFrame* frame,
                                   const std::string& path);
};
```

## 核心函数

### LidarCPDetectionComponent::Init

初始化组件，从配置文件加载参数，创建检测器实例并初始化，创建输出通道 Writer。若配置 `use_object_builder` 为 true，则同时初始化 ObjectBuilder。

```cpp
bool LidarCPDetectionComponent::Init() {
  // 读取 protobuf 配置
  LidarCPDetectionComponentConfig comp_config;
  GetProtoConfig(&comp_config);
  // 通过插件机制创建检测器
  BaseCPDetector* detector = BaseCPDetectorRegisterer::GetInstanceByName(detector_name);
  detector_->Init(detection_init_options);
  // 初始化 ObjectBuilder
  builder_.Init(builder_init_options);
}
```

### LidarCPDetectionComponent::Proc

消息处理入口，调用 `InternalProc` 执行实际检测逻辑，成功后通过 Writer 发送结果。

### LidarCPDetectionComponent::InternalProc

核心处理函数，依次执行候选点检测和目标属性构建。带有性能埋点（PERF_BLOCK）。

```cpp
bool LidarCPDetectionComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& in_message) {
  // 候选点检测
  detector_->Detect(detection_options, in_message->lidar_frame_.get());
  // 目标属性构建
  if (use_object_builder_) {
    builder_.Build(builder_options, in_message->lidar_frame_.get());
  }
}
```

### OfflineLidarCPDetection::Run

离线批量执行入口。读取指定目录下所有 `.pcd` 文件，按文件名排序后逐帧执行检测与属性构建，结果写入输出目录。

### OfflineLidarCPDetection::WriteObjectsForNewBenchmark

将检测结果以文本格式写入文件，每行包含目标的类型、位置、尺寸、朝向、速度及点云数据和多边形轮廓。

## 配置

组件配置通过 protobuf 文件 `lidar_cpdet_detection_component_config.pb.txt` 加载，主要字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `sensor_name` | string | 传感器名称，默认 `velodyne64` |
| `use_object_builder` | bool | 是否启用目标属性构建 |
| `output_channel_name` | string | 输出 Cyber 通道名 |
| `plugin_param.name` | string | 检测器插件名称（如 `CNNSegmentation`） |
| `plugin_param.config_path` | string | 检测器配置路径 |
| `plugin_param.config_file` | string | 检测器配置文件名 |

离线工具通过 gflags 配置，主要标志：

| 标志 | 默认值 | 说明 |
|------|--------|------|
| `--pcd_path` | `./pcd/` | 输入 PCD 文件目录 |
| `--output_path` | `./output/` | 输出结果目录 |
| `--detector_name` | `CNNSegmentation` | 检测器类名 |
| `--config_path` | `perception/lidar_detection/data` | 配置路径 |
| `--config_file` | `cnnseg16_param.pb.txt` | 配置文件名 |
| `--sensor_name` | `velodyne64` | 传感器名称 |

## 调用关系

```text
LidarCPDetectionComponent (Cyber 组件)
  |
  +-- Init()
  |     +-- GetProtoConfig()          // 加载 protobuf 配置
  |     +-- BaseCPDetectorRegisterer::GetInstanceByName()  // 插件创建检测器
  |     +-- detector_->Init()         // 初始化检测器
  |     +-- builder_.Init()           // 初始化 ObjectBuilder
  |     +-- node_->CreateWriter()     // 创建输出通道
  |
  +-- Proc()
        +-- InternalProc()
              +-- detector_->Detect() // 候选点检测
              +-- builder_.Build()    // 目标属性构建
        +-- writer_->Write()          // 发送检测结果
```

```text
OfflineLidarCPDetection (离线工具)
  |
  +-- Init()
  |     +-- BaseCPDetectorRegisterer::GetInstanceByName()
  |     +-- detector_->Init()
  |     +-- builder_->Init()
  |
  +-- Run()
        +-- LoadPCLPCD()              // 加载点云
        +-- TransformCloud()          // 坐标变换
        +-- detector_->Detect()       // 候选点检测
        +-- builder_->Build()         // 目标属性构建
        +-- WriteObjectsForNewBenchmark() // 写入结果
```
