---
title: "Pointcloud Preprocess 点云预处理"
---

# Pointcloud Preprocess

> 源码路径：`modules/perception/pointcloud_preprocess/`

## 概述

Pointcloud Preprocess 组件负责对 LiDAR 驱动发布的原始点云进行预处理，包括坐标变换查询、点云过滤（NaN 去除、范围过滤等），并将处理后的点云封装为 `LidarFrameMessage` 供下游检测模块使用。是 LiDAR 感知流水线的第一个环节。

## 核心类

### PointCloudPreprocessComponent

```cpp
// pointcloud_preprocess_component.h
class PointCloudPreprocessComponent
    : public cyber::Component<drivers::PointCloud> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::PointCloud>& message) override;
 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(
      const std::shared_ptr<const drivers::PointCloud>& in_message,
      const std::shared_ptr<onboard::LidarFrameMessage>& out_message);

  onboard::TransformWrapper lidar2world_trans_;
  BasePointCloudPreprocessor* cloud_preprocessor_;
  std::shared_ptr<cyber::Writer<onboard::LidarFrameMessage>> writer_;
};
```

## 核心函数

### Init()

**职责**：加载配置，初始化预处理器插件和坐标变换
**关键步骤**：

1. 加载 `PointCloudPreprocessComponentConfig` 配置
2. 通过注册器实例化 `BasePointCloudPreprocessor` 插件
3. 创建输出 channel writer
4. `InitAlgorithmPlugin()`：获取传感器信息，初始化预处理器，初始化 TF

### InternalProc()

```cpp
bool PointCloudPreprocessComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<onboard::LidarFrameMessage>& out_message) {
  // 从对象池获取 LidarFrame
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();

  // 查询 lidar2world 和 novatel2world 变换
  lidar2world_trans_.GetSensor2worldTrans(timestamp, &pose, &pose_novatel);
  frame->lidar2world_pose = pose;

  // 获取 sensor2novatel 外参
  lidar2world_trans_.GetExtrinsics(&preprocessor_option.sensor2novatel_extrinsics);

  // 执行点云预处理
  cloud_preprocessor_->Preprocess(preprocessor_option, in_message, frame.get());
  return true;
}
```

**职责**：单帧点云的完整预处理流程
**输入**：原始 `drivers::PointCloud`
**输出**：填充了点云和位姿的 `LidarFrameMessage`
**关键步骤**：

1. 从对象池获取 LidarFrame 和 PointCloud 对象（避免频繁分配）
2. 通过 TF 查询 lidar→world 和 novatel→world 位姿
3. 获取 lidar→novatel 外参
4. 调用预处理器执行点云过滤和转换

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `sensor_name` | string | LiDAR 传感器名称 |
| `output_channel_name` | string | 输出 channel |
| `lidar_query_tf_offset` | double | TF 查询时间偏移（毫秒） |
| `plugin_param.name` | string | 预处理器插件名称 |
| `plugin_param.config_path` | string | 预处理器配置路径 |
| `plugin_param.config_file` | string | 预处理器配置文件 |

## 调用关系

- **上游**：LiDAR 驱动发布 `drivers::PointCloud`
- **依赖**：TransformWrapper（TF2）、SensorManager（传感器信息）、LidarFramePool（对象池）
- **下游**：输出 `LidarFrameMessage` 供 `LidarDetection` 或 `PointcloudMotion` 使用
