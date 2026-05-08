---
title: "激光雷达检测滤波器"
---

# Lidar Detection Filter

> 源码路径: `modules/perception/lidar_detection_filter/`

## 概述

激光雷达检测滤波器模块负责对激光雷达感知管线中的检测结果进行后处理过滤。该模块接收上游传来的 `LidarFrameMessage`，通过可插拔的对象滤波器组（`ObjectFilterBank`）对帧内的目标物体进行过滤，移除误检或不符合条件的物体，然后将过滤后的结果发布到下游通道。

模块作为 Cyber RT 组件运行，支持通过配置开关控制是否启用滤波器组，并通过插件机制动态加载具体的滤波策略。

## 核心类

### LidarDetectionFilterComponent

继承自 `cyber::Component<LidarFrameMessage>`，是模块的 Cyber RT 组件入口。通过 `CYBER_REGISTER_COMPONENT` 宏注册到框架。

```cpp
class LidarDetectionFilterComponent final
    : public cyber::Component<LidarFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& in_message);
  std::shared_ptr<cyber::Writer<LidarFrameMessage>> writer_;
  std::string output_channel_name_;
  bool use_object_filter_bank_;
  ObjectFilterBank filter_bank_;
};
```

### ObjectFilterBank

对象滤波器容器，内部维护一个 `std::vector<std::shared_ptr<BaseObjectFilter>>` 列表。初始化时通过插件机制加载具体滤波器，`Filter` 方法依次调用每个滤波器的 `Filter` 对帧内目标进行过滤。

```cpp
class ObjectFilterBank {
 public:
  bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions());
  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame);
  std::string Name() const;
  std::size_t Size() const;

 private:
  std::vector<std::shared_ptr<BaseObjectFilter>> filter_bank_;
};
```

### BaseObjectFilter

所有对象滤波器的基类接口，定义了 `Init`、`Filter` 和 `Name` 三个纯虚函数。已知的实现包括：

- `BackgroundFilter` -- 背景过滤器
- `ROIBoundaryFilter` -- 感兴趣区域边界过滤器
- `StrategyFilter` -- 策略过滤器

```cpp
class BaseObjectFilter {
 public:
  virtual bool Init(const ObjectFilterInitOptions& options) = 0;
  virtual bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) = 0;
  virtual std::string Name() const = 0;
};
```

## 核心函数

### LidarDetectionFilterComponent::Init

```cpp
bool LidarDetectionFilterComponent::Init();
```

组件初始化函数。从 Proto 配置文件加载 `LidarDetectionFilterComponentConfig`，创建输出通道的 `Writer`，并根据 `use_object_filter_bank` 开关决定是否初始化 `ObjectFilterBank`。初始化滤波器组时，将配置中的 `plugin_param`（包含 `config_path` 和 `config_file`）传入。

### LidarDetectionFilterComponent::Proc

```cpp
bool LidarDetectionFilterComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& message);
```

消息处理入口函数，带有 `PERF_FUNCTION` 性能标记。调用 `InternalProc` 执行实际过滤逻辑，成功后通过 `writer_` 将消息发布到输出通道。

### LidarDetectionFilterComponent::InternalProc

```cpp
bool LidarDetectionFilterComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& in_message);
```

内部处理函数。当 `use_object_filter_bank_` 为 `true` 时，调用 `filter_bank_.Filter()` 对帧内的物体进行过滤，带有 `PERF_BLOCK("filter_bank")` 性能标记。

### ObjectFilterBank::Init

```cpp
bool ObjectFilterBank::Init(const ObjectFilterInitOptions& options);
```

通过插件机制加载配置文件中指定的所有滤波器实例，填充内部 `filter_bank_` 向量。

### ObjectFilterBank::Filter

```cpp
bool ObjectFilterBank::Filter(const ObjectFilterOptions& options, LidarFrame* frame);
```

遍历 `filter_bank_` 中所有已加载的滤波器，依次对 `LidarFrame` 执行过滤操作。

## 配置

模块使用 Protobuf 配置 `LidarDetectionFilterComponentConfig`：

```protobuf
message LidarDetectionFilterComponentConfig {
  optional string output_channel_name = 1;
  optional bool use_object_filter_bank = 2;
  optional PluginParam plugin_param = 3;
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `output_channel_name` | `string` | 过滤结果的输出通道名称 |
| `use_object_filter_bank` | `bool` | 是否启用对象滤波器组 |
| `plugin_param` | `PluginParam` | 插件参数，包含 `config_path` 和 `config_file`，用于加载滤波器组的具体配置 |

## 调用关系

```text
Cyber RT 框架
  └── LidarDetectionFilterComponent::Init()
  │     ├── GetProtoConfig() — 加载配置
  │     ├── CreateWriter() — 创建输出通道
  │     └── ObjectFilterBank::Init() — 加载滤波器插件
  │           └── 插件管理器加载 BaseObjectFilter 实现
  └── LidarDetectionFilterComponent::Proc()
        └── InternalProc()
        │     └── ObjectFilterBank::Filter()
        │           ├── BackgroundFilter::Filter()
        │           ├── ROIBoundaryFilter::Filter()
        │           └── StrategyFilter::Filter()
        └── Writer::Write() — 发布过滤结果
```
