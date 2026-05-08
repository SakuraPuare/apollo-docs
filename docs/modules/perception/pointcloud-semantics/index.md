---
title: "点云语义"
---

# 点云语义模块

> 源码路径: `modules/perception/pointcloud_semantics/`

## 概述

点云语义模块（PointCloud Semantics）负责对激光雷达点云进行语义解析与标注。该模块以 CyberRT Component 形式运行，订阅 `LidarFrameMessage` 输入消息，通过可插拔的 `BasePointCloudParser` 解析器对点云帧执行语义分割处理，再将结果写入输出通道供下游模块消费。

整个模块采用插件架构，解析器的具体实现通过 `BasePointCloudParserRegisterer` 动态注册与加载，便于扩展不同的语义解析算法。

## 核心类

### PointCloudSemanticComponent

```cpp
class PointCloudSemanticComponent final
    : public cyber::Component<LidarFrameMessage>
```

继承自 CyberRT `Component<LidarFrameMessage>`，是本模块的核心组件。

**命名空间**: `apollo::perception::lidar`

**成员变量**:

| 成员 | 类型 | 说明 |
|------|------|------|
| `output_channel_name_` | `std::string` | 输出通道名称，从配置读取 |
| `writer_` | `std::shared_ptr<cyber::Writer<LidarFrameMessage>>` | 结果消息写入器 |
| `parser_` | `std::shared_ptr<BasePointCloudParser>` | 点云语义解析器（插件） |

组件通过 `CYBER_REGISTER_COMPONENT(PointCloudSemanticComponent)` 宏完成注册。

## 核心函数

### Init

```cpp
bool Init() override;
```

组件初始化函数，执行以下步骤：

1. 调用 `GetProtoConfig` 加载 `PointCloudSemanticComponentConfig` 配置
2. 根据 `output_channel_name` 创建 `Writer` 写入器
3. 从配置的 `plugin_param` 中获取解析器名称，通过 `BasePointCloudParserRegisterer::GetInstanceByName` 实例化解析器插件
4. 使用 `config_path` 和 `config_file` 初始化解析器

初始化失败时返回 `false`。

### Proc

```cpp
bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;
```

消息处理回调，由 CyberRT 框架在收到 `LidarFrameMessage` 时触发：

1. 调用 `InternalProc` 执行实际语义解析
2. 解析成功后通过 `writer_->Write` 将结果发送到输出通道
3. 处理过程中带有 `PERF_FUNCTION()` 性能埋点

### InternalProc

```cpp
bool InternalProc(const std::shared_ptr<LidarFrameMessage>& in_message);
```

内部处理函数，负责调用解析器：

```cpp
PointCloudParserOptions parser_options;
if (!parser_->Parse(parser_options, in_message->lidar_frame_.get())) {
    AERROR << "PointCloud parser error!";
    return false;
}
```

通过 `parser_->Parse` 对 `lidar_frame_` 执行语义解析，带有 `PERF_BLOCK("pointcloud_parser")` 性能监控。

## 配置

模块使用 Protobuf 配置 `PointCloudSemanticComponentConfig`：

| 字段 | 说明 |
|------|------|
| `output_channel_name` | 语义结果输出通道名称 |
| `plugin_param.name` | 解析器插件名称 |
| `plugin_param.config_path` | 解析器配置文件目录 |
| `plugin_param.config_file` | 解析器配置文件名 |

配置通过 CyberRT 组件机制自动加载。

## 调用关系

```text
CyberRT 框架
  └─ PointCloudSemanticComponent::Proc(LidarFrameMessage)
       ├─ InternalProc(LidarFrameMessage)
       │    └─ BasePointCloudParser::Parse(options, lidar_frame)
       └─ Writer::Write(LidarFrameMessage)  // 发送到输出通道
```

模块整体数据流如下：

- **输入**: 订阅激光雷达组件发布的 `LidarFrameMessage`（包含 `LidarFrame` 数据）
- **处理**: 通过插件式解析器对点云帧执行语义解析，结果直接写回 `lidar_frame`
- **输出**: 将处理后的 `LidarFrameMessage` 发布到配置的输出通道，供下游规划或可视化模块使用
