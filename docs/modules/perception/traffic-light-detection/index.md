---
title: "交通灯检测"
---

# Traffic Light Detection

> 源码路径: `modules/perception/traffic_light_detection/`

## 概述

交通灯检测模块负责从摄像头图像中检测交通信号灯的状态（颜色和形状），并将检测结果传递给下游规划模块。该模块采用 CyberRT Component 架构，通过插件机制支持多种检测算法（如 Caffe、YOLOX），实现了检测器与组件框架的解耦。模块接收 `TrafficDetectMessage` 消息，内部调用已注册的检测器插件完成推理，再将结果通过 CyberRT Writer 发布到输出通道。

## 核心类

### TrafficLightDetectComponent

继承自 `cyber::Component<TrafficDetectMessage>`，是交通灯检测的 CyberRT 组件入口。负责初始化配置和算法插件，并在每次消息到达时触发检测流程。

```cpp
class TrafficLightDetectComponent : public cyber::Component<TrafficDetectMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<TrafficDetectMessage>& message) override;

 private:
  int InitConfig();
  bool InitAlgorithmPlugin();
  bool InternalProc(const std::shared_ptr<TrafficDetectMessage const>& message);

  trafficlight::TrafficLightDetectorInitOptions detector_init_options_;
  std::string tl_detector_name_;
  int gpu_id_;
  std::string config_path_;
  std::string config_file_;
  std::shared_ptr<trafficlight::BaseTrafficLightDetector> detector_;
  std::shared_ptr<apollo::cyber::Writer<TrafficDetectMessage>> writer_;
};
```

### BaseTrafficLightDetector

交通灯检测器的抽象基类，定义了插件接口。所有具体检测算法（如 Caffe、YOLOX）通过 `REGISTER_TRAFFIC_LIGHT_DETECTOR` 宏注册为插件，由组件在运行时动态加载。

```cpp
class BaseTrafficLightDetector {
 public:
  virtual bool Init(const TrafficLightDetectorInitOptions& options) = 0;
  virtual bool Detect(camera::TrafficLightFrame* frame) = 0;
};
```

### TrafficLightDetectorInitOptions

检测器初始化选项结构体，继承自 `BaseInitOptions`，额外包含相机模型信息。

```cpp
struct TrafficLightDetectorInitOptions : public BaseInitOptions {
  std::shared_ptr<base::BaseCameraModel> base_camera_model = nullptr;
};
```

## 核心函数

### TrafficLightDetectComponent::Init

组件初始化入口，依次完成配置加载和算法插件初始化。任一步骤失败则返回 `false`。

### TrafficLightDetectComponent::Proc

消息处理回调。收到 `TrafficDetectMessage` 后调用 `InternalProc` 执行检测，成功后通过 `writer_` 将消息发布到输出通道。

### TrafficLightDetectComponent::InitConfig

从 proto 配置文件加载 `TrafficLightParam`，提取插件名称（`tl_detector_name_`）、配置路径、GPU ID，并创建输出 Writer。

### TrafficLightDetectComponent::InitAlgorithmPlugin

通过 `BaseTrafficLightDetectorRegisterer` 按名称查找并实例化检测器插件，使用配置参数初始化检测器。

### TrafficLightDetectComponent::InternalProc

实际执行检测的内部方法。从消息中获取 `TrafficLightFrame`，调用 `detector_->Detect()` 完成推理。使用 `PERF_BLOCK` 进行性能剖析。

## 配置

模块通过 `TrafficLightParam` protobuf 配置，主要字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `plugin_param.name` | `string` | 检测器插件名称 |
| `plugin_param.config_path` | `string` | 插件配置文件目录 |
| `plugin_param.config_file` | `string` | 插件配置文件名 |
| `gpu_id` | `int32` | GPU 设备编号 |
| `detection_output_channel_name` | `string` | 输出通道名称 |

各检测器插件有独立的模型参数 proto，位于子目录 `detector/<plugin>/proto/model_param.proto`。

## 调用关系

```text
TrafficDetectMessage (输入)
  |
  v
TrafficLightDetectComponent::Proc()
  |
  +-- InternalProc()
  |     |
  |     v
  |   BaseTrafficLightDetector::Detect(TrafficLightFrame*)
  |     |
  |     +-- [Caffe 检测器] detector/caffe_detection/
  |     +-- [YOLOX 检测器] detector/yolox_detection/
  |
  +-- Writer::Write() --> 输出通道
```
