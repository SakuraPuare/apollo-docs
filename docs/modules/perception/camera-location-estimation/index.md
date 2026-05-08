---
title: "Camera Location Estimation 相机位置估计"
---

# Camera Location Estimation

> 源码路径：`modules/perception/camera_location_estimation/`

## 概述

Camera Location Estimation 组件负责将相机检测到的 2D 障碍物转换为 3D 空间位置。接收上游检测组件输出的 `CameraFrame`（包含 2D 检测框和相机内外参），通过 Transformer 插件估计障碍物的 3D 位置（深度、中心点），输出带有 3D 位置信息的 `CameraFrame`。

## 核心类

### CameraLocationEstimationComponent

```cpp
// camera_location_estimation_component.h
class CameraLocationEstimationComponent final
    : public cyber::Component<onboard::CameraFrame> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<onboard::CameraFrame>& msg) override;
 private:
  void InitTransformer(const CameraLocationEstimation& location_estimation_param);
  std::shared_ptr<BaseTransformer> transformer_;
  std::shared_ptr<cyber::Writer<onboard::CameraFrame>> writer_;
};
```

## 核心函数

### Init()

**职责**：加载配置并初始化 Transformer 插件
**关键步骤**：

1. 加载 `CameraLocationEstimation` 配置
2. 通过插件注册器实例化 `BaseTransformer`
3. 创建输出 channel writer

### Proc()

```cpp
bool CameraLocationEstimationComponent::Proc(
    const std::shared_ptr<onboard::CameraFrame>& msg) {
  // 复制输入帧数据
  out_message->frame_id = msg->frame_id;
  out_message->timestamp = msg->timestamp;
  out_message->detected_objects = msg->detected_objects;
  out_message->camera_k_matrix = msg->camera_k_matrix;
  out_message->camera2world_pose = msg->camera2world_pose;

  // 执行 2D→3D 位置变换
  transformer_->Transform(out_message.get());

  writer_->Write(out_message);
  return true;
}
```

**职责**：对每帧检测结果执行 2D 到 3D 位置估计
**输入**：包含 2D 检测结果和相机参数的 CameraFrame
**输出**：填充了 3D 位置信息的 CameraFrame
**关键逻辑**：调用 `transformer_->Transform()` 利用相机内参和几何约束估计深度

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `plugin_param.name` | string | Transformer 插件名称 |
| `plugin_param.config_path` | string | 插件配置路径 |
| `plugin_param.config_file` | string | 插件配置文件 |
| `channel.output_obstacles_channel_name` | string | 输出 channel |

## 调用关系

- **上游**：`CameraDetectionSingleStage` 或 `CameraDetectionMultiStage` 输出 `CameraFrame`
- **下游**：输出 `CameraFrame` 供 `CameraLocationRefinement` 或 `CameraTracking` 使用
