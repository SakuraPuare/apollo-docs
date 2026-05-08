---
title: "Pointcloud Motion 点云运动补偿"
---

# Pointcloud Motion

> 源码路径：`modules/perception/pointcloud_motion/`

## 概述

Pointcloud Motion 组件负责对预处理后的 LiDAR 点云进行运动补偿（motion compensation）。由于 LiDAR 扫描一圈需要时间，车辆在此期间的运动会导致点云畸变，本模块通过 MotionParser 插件对点云进行去畸变处理，使所有点对齐到同一时刻。是 LiDAR 感知流水线中预处理之后、检测之前的环节。

## 核心类

### PointCloudMotionComponent

```cpp
// pointcloud_motion_component.h
class PointCloudMotionComponent final
    : public cyber::Component<LidarFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;
 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& in_message);
  std::shared_ptr<BaseMotionParser> parser_;
  std::shared_ptr<cyber::Writer<LidarFrameMessage>> writer_;
};
```

## 核心函数

### Init()

**职责**：加载配置，初始化运动补偿解析器
**关键步骤**：

1. 加载 `PointCloudMotionComponentConfig` 配置
2. 创建输出 channel writer
3. 通过注册器实例化 `BaseMotionParser` 插件并初始化

### InternalProc()

```cpp
bool PointCloudMotionComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& in_message) {
  PointCloudParserOptions parser_options;
  parser_->Parse(parser_options, in_message->lidar_frame_.get());
  return true;
}
```

**职责**：对 LidarFrame 中的点云执行运动补偿
**输入**：包含原始点云和位姿的 `LidarFrameMessage`
**输出**：原地修改点云，去除运动畸变
**关键逻辑**：`parser_->Parse()` 利用帧间位姿差异对每个点进行时间插值补偿

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `output_channel_name` | string | 输出 channel |
| `plugin_param.name` | string | MotionParser 插件名称 |
| `plugin_param.config_path` | string | 插件配置路径 |
| `plugin_param.config_file` | string | 插件配置文件 |

## 调用关系

- **上游**：`PointCloudPreprocess` 输出 `LidarFrameMessage`
- **下游**：输出运动补偿后的 `LidarFrameMessage` 供 `LidarDetection` 使用
