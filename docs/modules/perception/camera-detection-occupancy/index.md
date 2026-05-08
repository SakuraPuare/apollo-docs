---
title: "Camera Detection Occupancy 占据网格检测"
---

# Camera Detection Occupancy

> 源码路径：`modules/perception/camera_detection_occupancy/`

## 概述

Camera Detection Occupancy 组件基于多相机输入生成占据网格（Occupancy Grid）和伪点云，同时检测 3D 障碍物。与 BEV 检测不同，本模块额外输出占据网格的伪点云表示（pseudo pointcloud），可供下游 LiDAR 融合模块使用。支持多路相机输入，通过 lidar2img 变换矩阵将多视角图像映射到统一 3D 空间。

## 核心类

### CameraFrame

```cpp
// camera_frame.h
struct CameraFrame {
  std::uint64_t frame_id;
  double timestamp;
  std::vector<std::shared_ptr<DataProvider>> data_provider;
  std::vector<base::ObjectPtr> detected_objects;
  std::vector<float> k_lidar2img;  // lidar到图像的变换矩阵
  std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud;  // 伪点云
};
```

### CameraDetectionOccComponent

```cpp
// camera_detection_occupancy_component.h
class CameraDetectionOccComponent final : public cyber::Component<> {
 public:
  bool Init() override;
 private:
  bool InitTransformWrapper(const CameraDetectionBEV& detection_param);
  bool InitCameraFrame(const CameraDetectionBEV& detection_param);
  bool InitListeners(const CameraDetectionBEV& detection_param);
  bool InitDetector(const CameraDetectionBEV& detection_param);
  bool OnReceiveImage(const std::shared_ptr<drivers::Image>& msg);
  void CameraToWorldCoor(const Eigen::Affine3d& camera2world,
                         std::vector<base::ObjectPtr>* objs);
  bool LoadCameraExtrinsic(const std::string& file_path,
                           Eigen::Affine3d* camera_extrinsic);
  bool LoadCameraExtrinsicNus(const std::string& camera_extrinsic_file_path,
                              const std::string& lidar_extrinsic_file_path,
                              Eigen::Affine3d* camera2lidar_rt);
};
```

**注意**：继承 `cyber::Component<>`，手动创建多路 Reader。

## 核心函数

### InitCameraFrame()

**职责**：初始化多路相机 DataProvider 并计算 lidar2img 变换矩阵
**关键步骤**：

1. 从配置读取 `bev_camera_name` 列表（多路相机名称）
2. 为每路相机创建 DataProvider
3. 加载每路相机的外参（camera2lidar 变换）
4. 计算 lidar2img = intrinsic × camera2lidar_rt⁻¹，存入 `k_lidar2img`

### OnReceiveImage()

**职责**：接收多路图像，执行占据网格检测
**关键步骤**：

1. 将图像数据填充到对应 DataProvider
2. 获取 lidar2world 变换
3. 调用 `detector_->Detect(frame_ptr_)` 执行检测（同时生成伪点云）
4. `CameraToWorldCoor()` 转换检测结果到世界坐标系
5. 构建 `PerceptionObstacles` 消息并发布

### LoadCameraExtrinsicNus()

**职责**：加载 nuScenes 格式的相机和 LiDAR 外参，计算 camera2lidar 变换
**逻辑**：camera2lidar = lidar_extrinsic⁻¹ × camera_extrinsic

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `camera_name` | string | 主相机名称 |
| `gpu_id` | int | GPU 设备 ID |
| `enable_undistortion` | bool | 是否启用去畸变 |
| `sensors.bev_camera_name` | repeated string | 多路相机名称列表 |
| `detector_plugin_param.name` | string | 占据检测器插件名称 |
| `channel.input_camera_channel_name` | repeated string | 多路相机输入 channel |
| `channel.output_obstacles_channel_name` | string | 输出 channel |

## 与 BEV 检测的区别

| 特性 | BEV | Occupancy |
| --- | --- | --- |
| 输出 | 仅 3D 目标 | 3D 目标 + 伪点云 |
| lidar2img | 无 | 有（用于体素投影） |
| Tracker | 无 | 预留接口（当前注释） |

## 调用关系

- **上游**：多路相机驱动发布 `drivers::Image`
- **依赖**：SensorManager（相机内参）、TransformWrapper（TF2）、相机外参文件
- **下游**：输出 `PerceptionObstacles` 供融合和 Planning 使用
