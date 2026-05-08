---
title: "Adapters 适配器配置"
---

# Adapters 适配器配置

> 源码路径：`modules/common/adapters/`

## 概述

`adapters` 模块定义了 Apollo 各模块通信所使用的话题（Topic）名称和消息类型配置。它不包含业务逻辑，而是作为全局话题注册中心，通过 gflags 定义默认话题名，通过 protobuf 定义适配器配置结构。各模块在启动时读取 `AdapterManagerConfig` 来确定自己需要订阅和发布哪些话题。

## 数据结构

### AdapterConfig

单个适配器的配置，描述一个话题的类型、方向和参数。

```protobuf
message AdapterConfig {
  enum MessageType {
    POINT_CLOUD = 1;
    GPS = 2;
    IMU = 3;
    CHASSIS = 4;
    LOCALIZATION = 5;
    PLANNING_TRAJECTORY = 6;
    PREDICTION = 10;
    PERCEPTION_OBSTACLES = 11;
    ROUTING_RESPONSE = 17;
    // ... 共 70+ 种消息类型
  }

  enum Mode {
    RECEIVE_ONLY = 0;   // 仅接收
    PUBLISH_ONLY = 1;   // 仅发送
    DUPLEX = 2;         // 双向
  }

  optional MessageType type = 1;
  optional Mode mode = 2;
  optional int32 message_history_limit = 3 [default = 10];
  optional bool latch = 4 [default = false];
  optional string topic = 5;
}
```

**源码**：`modules/common/adapters/proto/adapter_config.proto`

### AdapterManagerConfig

模块级别的适配器配置，包含该模块需要的所有 `AdapterConfig`。

```protobuf
message AdapterManagerConfig {
  repeated AdapterConfig config = 1;
  optional bool is_ros = 2;  // 是否来自 ROS
}
```

## 话题注册

所有默认话题名通过 gflags 在 `adapter_gflags.h` 中声明，覆盖 Apollo 系统的全部通信话题。按功能分类：

### 传感器话题

| gflags | 默认话题 | 说明 |
|--------|----------|------|
| `gps_topic` | `/apollo/sensor/gnss/best_pose` | GPS 定位 |
| `imu_topic` | `/apollo/sensor/gnss/imu` | IMU 数据 |
| `pointcloud_topic` | `/apollo/sensor/velodyne64/pointcloud` | 64 线激光雷达 |
| `pointcloud_128_topic` | `/apollo/sensor/velodyne128/pointcloud` | 128 线激光雷达 |
| `image_front_topic` | `/apollo/sensor/camera/front_6mm/image` | 前视相机 |
| `conti_radar_topic` | `/apollo/sensor/radar/front` | 前向毫米波雷达 |

### 定位话题

| gflags | 说明 |
|--------|------|
| `localization_topic` | 融合定位输出 |
| `localization_gnss_topic` | GNSS 定位 |
| `localization_lidar_topic` | 激光雷达定位 |

### 规划控制话题

| gflags | 说明 |
|--------|------|
| `planning_trajectory_topic` | 规划轨迹 |
| `control_command_topic` | 控制指令 |
| `chassis_topic` | 底盘状态 |
| `routing_response_topic` | 路由响应 |

### 感知话题

| gflags | 说明 |
|--------|------|
| `perception_obstacle_topic` | 感知障碍物 |
| `traffic_light_detection_topic` | 交通灯检测 |
| `prediction_topic` | 预测结果 |

### 其他话题

| gflags | 说明 |
|--------|------|
| `storytelling_topic` | 场景叙述 |
| `guardian_topic` | Guardian 安全模块 |
| `tf_topic` / `tf_static_topic` | 坐标变换 |
| `latency_recording_topic` | 延迟记录 |

**源码**：`modules/common/adapters/adapter_gflags.h`

## 配置方式

各模块通过 YAML 配置文件指定 `AdapterManagerConfig`，例如：

```yaml
adapter_config:
  config:
    - type: POINT_CLOUD
      mode: RECEIVE_ONLY
      topic: "/apollo/sensor/velodyne64/pointcloud"
    - type: PERCEPTION_OBSTACLES
      mode: PUBLISH_ONLY
      topic: "/apollo/perception/obstacles"
```

## 调用关系

- **被依赖**：所有 Apollo 模块在初始化时读取 `AdapterManagerConfig` 配置话题订阅/发布
- **关联**：`AdapterManager` 类（位于 `cyber/common/` 或各模块内部）使用此配置创建 Reader/Writer
