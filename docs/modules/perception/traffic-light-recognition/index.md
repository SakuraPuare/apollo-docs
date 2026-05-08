---
title: "交通灯识别"
---

# Traffic Light Recognition

> 源码路径: `modules/perception/traffic_light_recognition/`

## 概述

交通灯识别模块负责对摄像头检测到的交通灯候选区域进行分类识别。模块基于 CyberRT 的 Component 框架，通过订阅上游 `TrafficDetectMessage` 消息获取交通灯帧数据，调用可插拔的识别算法插件（`BaseTrafficLightRecognitor`）对交通灯状态进行分类，识别完成后将结果写入输出通道供下游模块消费。

整体流程为：接收消息 -> 加载配置 -> 初始化识别插件 -> 对每帧执行 `Detect` -> 发布结果。

## 核心类

### TrafficLightRecognComponent

继承自 `cyber::Component<TrafficDetectMessage>`，是交通灯识别的 CyberRT 组件入口。

```cpp
class TrafficLightRecognComponent : public cyber::Component<TrafficDetectMessage> {
 public:
    bool Init() override;
    bool Proc(const std::shared_ptr<TrafficDetectMessage>& message) override;

 private:
    int InitConfig();
    bool InitAlgorithmPlugin();
    bool InternalProc(const std::shared_ptr<TrafficDetectMessage const>& message);

    TrafficLightRecognitorInitOptions recognitor_init_options_;
    std::string tl_recognitor_name_;
    int gpu_id_;
    std::string config_path_;
    std::string config_file_;
    std::shared_ptr<BaseTrafficLightRecognitor> recognitor_;
    std::shared_ptr<apollo::cyber::Writer<TrafficDetectMessage>> writer_;
};
```

**关键成员**：

| 成员 | 类型 | 说明 |
|------|------|------|
| `recognitor_` | `shared_ptr<BaseTrafficLightRecognitor>` | 识别算法插件实例 |
| `writer_` | `shared_ptr<Writer<TrafficDetectMessage>>` | 输出通道写入器 |
| `tl_recognitor_name_` | `string` | 识别插件名称，用于插件工厂注册 |
| `gpu_id_` | `int` | GPU 设备 ID |

### BaseTrafficLightRecognitor

识别算法的基类接口，定义了插件化识别算法的标准规范。

```cpp
class BaseTrafficLightRecognitor {
 public:
    virtual bool Init(const TrafficLightRecognitorInitOptions& options) = 0;
    virtual bool Detect(camera::TrafficLightFrame* frame) = 0;
};
```

通过 `REGISTER_TRAFFIC_LIGHT_DETECTOR(name)` 宏注册具体实现到插件工厂。

### TrafficLightRecognitorInitOptions

继承自 `BaseInitOptions`，用于初始化识别插件：

```cpp
struct TrafficLightRecognitorInitOptions : public BaseInitOptions {
    std::shared_ptr<base::BaseCameraModel> base_camera_model = nullptr;
};
```

## 核心函数

### TrafficLightRecognComponent::Init

组件初始化入口，依次完成配置加载和算法插件初始化：

```cpp
bool TrafficLightRecognComponent::Init() {
    if (InitConfig() != cyber::SUCC) { return false; }
    if (InitAlgorithmPlugin() != cyber::SUCC) { return false; }
    return true;
}
```

### TrafficLightRecognComponent::Proc

消息处理回调，每收到一条 `TrafficDetectMessage` 触发一次识别流程：

```cpp
bool TrafficLightRecognComponent::Proc(
    const std::shared_ptr<TrafficDetectMessage>& message) {
    bool status = InternalProc(message);
    if (status) {
        writer_->Write(message);
    }
    return status;
}
```

识别成功后将原始消息写入输出通道，供下游决策模块使用。

### TrafficLightRecognComponent::InitConfig

从 protobuf 配置文件读取组件参数，包括插件名称、配置路径、GPU ID 和输出通道名，并创建 CyberRT Writer。

### TrafficLightRecognComponent::InitAlgorithmPlugin

通过插件工厂 `BaseTrafficLightRecognitorRegisterer` 实例化识别算法，使用配置参数调用 `Init` 完成插件初始化。

### TrafficLightRecognComponent::InternalProc

核心处理逻辑，调用识别插件的 `Detect` 方法对交通灯帧进行分类：

```cpp
bool TrafficLightRecognComponent::InternalProc(
    const std::shared_ptr<TrafficDetectMessage const>& message) {
    bool status = recognitor_->Detect(message->traffic_light_frame_.get());
    return true;
}
```

## 配置

组件使用 protobuf 格式配置文件，定义在 `RecognitionParam` 消息中：

```protobuf
message RecognitionParam {
  optional int32 gpu_id = 1;
  optional perception.PluginParam plugin_param = 2;
  optional string recognition_output_channel_name = 3;
}
```

| 字段 | 说明 |
|------|------|
| `gpu_id` | 推理使用的 GPU 设备编号 |
| `plugin_param` | 插件参数，包含插件名称（`name`）、配置路径（`config_path`）和配置文件（`config_file`） |
| `recognition_output_channel_name` | 识别结果输出的 CyberRT 通道名称 |

## 调用关系

```text
上游 TrafficDetectMessage
        |
        v
TrafficLightRecognComponent::Proc
        |
        v
InternalProc
        |
        v
BaseTrafficLightRecognitor::Detect  (插件化实现)
        |
        v
writer_->Write (输出识别结果)
```

上游模块发送包含交通灯帧数据的 `TrafficDetectMessage`，组件通过 `Proc` 接收后委托给 `InternalProc`，内部调用已注册的识别插件执行分类推理，最终将结果通过 CyberRT Writer 发布到输出通道。
