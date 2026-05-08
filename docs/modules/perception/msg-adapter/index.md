---
title: "消息适配器"
---

# Msg Adapter

> 源码路径: `modules/perception/msg_adapter/`

## 概述

消息适配器（Msg Adapter）是 Apollo 感知模块中的一个 CyberRT 组件，负责将感知模块内部使用的消息格式转换为标准的 `PerceptionObstacles` protobuf 消息并发布到统一的输出 topic。该模块实现了发布-订阅模式下的消息格式桥接，支持三种输入源的转换：

- **相机帧**（CameraFrame）-> PerceptionObstacles
- **传感器帧消息**（SensorFrameMessage）-> PerceptionObstacles
- **激光雷达帧消息**（LidarFrameMessage）-> PerceptionObstacles

每种转换的输入/输出 topic 均通过 gflags 配置，默认输出统一到 `/apollo/perception/obstacles`。

## 核心类

### MsgAdapterComponent

继承自 `apollo::cyber::Component<>`，是该模块的 CyberRT 组件入口。通过 `CYBER_REGISTER_COMPONENT` 宏注册到 CyberRT 框架。

```cpp
class MsgAdapterComponent final : public apollo::cyber::Component<> {
 public:
  bool Init() override;

 private:
  std::shared_ptr<MsgConverter> msg_converter_;
};
```

`Init()` 方法中创建 `MsgConverter` 实例，并通过 `Add()` 注册三条消息转换通道。

### MsgConverter

通用消息转换器，封装了 CyberRT 的 Reader/Writer 创建逻辑。通过模板方法 `Add<From, To>()` 将源 topic 订阅、消息转换、目标 topic 发布串联为一个完整的转换管道。

```cpp
class MsgConverter {
 public:
  template <class U, class V>
  using Callback = bool(*)(const std::shared_ptr<U> &, V *);

  template <class From, class To>
  bool Add(const std::string &from_topic, const std::string &to_topic,
           Callback<From, To> convert);

 private:
  std::vector<std::shared_ptr<cyber::ReaderBase>> readers_;
  std::vector<std::shared_ptr<cyber::WriterBase>> writers_;
  std::shared_ptr<cyber::Node> node_;
};
```

内部持有所有已创建的 Reader 和 Writer 的共享指针，确保生命周期与组件一致。

## 核心函数

### ConvertObjectToPb

```cpp
bool ConvertObjectToPb(const base::ObjectPtr &object_ptr,
                       PerceptionObstacle *pb_msg);
```

定义在 `convert/common.h` 中。将感知内部的 `base::Object` 逐字段映射到 `PerceptionObstacle` protobuf 消息，包括：

- 基本属性：id、theta、position、velocity、acceleration、尺寸
- 多边形轮廓（polygon）和点云数据
- 锚点（anchor_point）和 2D 边界框（bbox2d）
- 协方差矩阵：position / velocity / acceleration covariance
- 跟踪时间和类型信息：type、sub_type、semantic_type、motion_type
- 车辆灯光状态（仅当 object 类型为 VEHICLE 时）
- 融合传感器量测（当 fusion_supplement.on_use 为 true 时）

### ConvertCameraFrame2Obstacles

```cpp
bool ConvertCameraFrame2Obstacles(
    const std::shared_ptr<onboard::CameraFrame> &frame,
    PerceptionObstacles *obstacles);
```

将相机检测帧转换为 `PerceptionObstacles`。设置 header 中的 module_name 为 `perception_camera`，遍历 `frame->detected_objects` 逐一调用 `ConvertObjectToPb` 完成转换。

### ConvertSensorFrameMessage2Obstacles

```cpp
bool ConvertSensorFrameMessage2Obstacles(
    const std::shared_ptr<onboard::SensorFrameMessage> &msg,
    PerceptionObstacles *obstacles);
```

将通用传感器帧消息转换为 `PerceptionObstacles`。设置 header 中的 module_name 为 `perception_obstacle`，使用消息自带的 `seq_num_` 和 `lidar_timestamp_` 填充头部字段。

### ConvertLidarFrameMessage2Obstacles

```cpp
bool ConvertLidarFrameMessage2Obstacles(
    const std::shared_ptr<onboard::LidarFrameMessage> &msg,
    PerceptionObstacles *obstacles);
```

将激光雷达帧消息转换为 `PerceptionObstacles`。与其他转换函数不同，该函数在转换前会执行坐标变换：利用 `lidar2world_pose` 矩阵将目标的中心点、多边形、点云和锚点从激光雷达坐标系变换到世界坐标系，然后重新分配 track_id。

## 配置

通过 gflags 定义输入输出 topic，默认值如下：

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `cameraframe_to_obstacles_in` | `/fake_topic` | 相机帧订阅 topic |
| `cameraframe_to_obstacles_out` | `/apollo/perception/obstacles` | 相机帧输出 topic |
| `sensorframe_message_to_obstacles_in` | `/fake_topic` | 传感器帧订阅 topic |
| `sensorframe_message_to_obstacles_out` | `/apollo/perception/obstacles` | 传感器帧输出 topic |
| `lidarframe_to_obstacles_in` | `/fake_topic` | 激光雷达帧订阅 topic |
| `lidarframe_to_obstacles_out` | `/apollo/perception/obstacles` | 激光雷达帧输出 topic |

实际部署时需要通过 dag 文件或命令行参数将 `_in` topic 设置为真实的上游输出 topic。

## 调用关系

```text
CyberRT 调度
  └── MsgAdapterComponent::Init()
        ├── 创建 MsgConverter(node_)
        ├── MsgConverter::Add<CameraFrame, PerceptionObstacles>()
        │     └── 订阅 FLAGS_cameraframe_to_obstacles_in
        │           └── 回调: ConvertCameraFrame2Obstacles()
        │                 └── ConvertObjectToPb()  (逐目标)
        │                       └── 发布到 FLAGS_cameraframe_to_obstacles_out
        ├── MsgConverter::Add<SensorFrameMessage, PerceptionObstacles>()
        │     └── 订阅 FLAGS_sensorframe_message_to_obstacles_in
        │           └── 回调: ConvertSensorFrameMessage2Obstacles()
        │                 └── ConvertObjectToPb()  (逐目标)
        │                       └── 发布到 FLAGS_sensorframe_message_to_obstacles_out
        └── MsgConverter::Add<LidarFrameMessage, PerceptionObstacles>()
              └── 订阅 FLAGS_lidarframe_to_obstacles_in
                    └── 回调: ConvertLidarFrameMessage2Obstacles()
                          ├── 坐标变换 (lidar -> world)
                          ├── ConvertObjectToPb()  (逐目标)
                          └── 发布到 FLAGS_lidarframe_to_obstacles_out
```

所有转换路径最终汇聚到统一的 `/apollo/perception/obstacles` topic（默认值），供下游规划控制模块消费。
