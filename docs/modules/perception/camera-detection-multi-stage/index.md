---
title: "Camera Detection Multi Stage 多阶段相机检测"
---

# Camera Detection Multi Stage

> 源码路径：`modules/perception/camera_detection_multi_stage/`

## 概述

多阶段相机检测组件使用两阶段（two-stage）检测器从单目相机图像中检测 2D/3D 障碍物。与单阶段检测器相比，多阶段检测器先生成候选区域（proposals），再对候选区域进行精细分类和回归，精度更高但速度稍慢。组件输入单路相机图像，输出包含检测结果的 `CameraFrame`。

## 核心类

### CameraDetectionMultiStageComponent

```cpp
// camera_detection_multi_stage_component.h
class CameraDetectionMultiStageComponent final
    : public cyber::Component<drivers::Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::Image>& msg) override;
 private:
  bool InitCameraFrame(const CameraDetectionMultiStage& detection_param);
  bool InitObstacleDetector(const CameraDetectionMultiStage& detection_param);
  bool InitTransformWrapper(const CameraDetectionMultiStage& detection_param);
  bool InternalProc(const std::shared_ptr<apollo::drivers::Image>& msg,
                    const std::shared_ptr<onboard::CameraFrame>& out_message);
};
```

## 核心函数

### Init()

**职责**：初始化检测器、数据提供器和坐标变换
**关键步骤**：

1. 加载 `CameraDetectionMultiStage` 配置
2. `InitObstacleDetector()`：从 SensorManager 获取相机内参，通过插件注册器实例化多阶段检测器
3. `InitCameraFrame()`：初始化 DataProvider
4. `InitTransformWrapper()`：初始化 TF2 坐标变换
5. 创建输出 channel writer

### InternalProc()

```cpp
bool CameraDetectionMultiStageComponent::InternalProc(
    const std::shared_ptr<apollo::drivers::Image>& msg,
    const std::shared_ptr<onboard::CameraFrame>& out_message) {
  // 填充图像数据
  out_message->data_provider->FillImageData(
      image_height_, image_width_,
      reinterpret_cast<const uint8_t*>(msg->data().data()), msg->encoding());
  out_message->camera_k_matrix = camera_k_matrix_;
  out_message->timestamp = msg->measurement_time() + timestamp_offset_;

  // 获取相机到世界坐标系变换
  Eigen::Affine3d camera2world;
  trans_wrapper_->GetSensor2worldTrans(msg_timestamp, &camera2world);
  out_message->camera2world_pose = camera2world;

  // 执行多阶段检测
  detector_->Detect(out_message.get());
  return true;
}
```

**职责**：单帧图像的完整处理流程
**关键步骤**：

1. 填充图像数据到 DataProvider
2. 设置相机内参矩阵和时间戳
3. 通过 TF 获取 camera2world 变换
4. 调用 `detector_->Detect()` 执行多阶段检测

**注意**：与单阶段检测不同，多阶段组件不在组件内做 `CameraToWorldCoor` 坐标转换，检测结果保持在相机坐标系，由下游模块处理。

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `camera_name` | string | 相机名称 |
| `gpu_id` | int | GPU 设备 ID |
| `enable_undistortion` | bool | 是否启用去畸变 |
| `timestamp_offset` | double | 时间戳偏移量（秒） |
| `plugin_param.name` | string | 检测器插件名称 |
| `plugin_param.config_path` | string | 检测器配置路径 |
| `plugin_param.config_file` | string | 检测器配置文件 |
| `channel.output_obstacles_channel_name` | string | 输出 channel |

## 调用关系

- **上游**：相机驱动发布 `drivers::Image`
- **依赖**：SensorManager（相机内参）、TransformWrapper（TF2）、DataProvider（图像预处理）
- **下游**：输出 `onboard::CameraFrame` 供位置估计和跟踪模块使用
