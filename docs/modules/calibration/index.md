---
title: Calibration 标定模块
description: Apollo 自动驾驶平台传感器标定数据管理模块，负责存储和组织各车型的传感器外参、内参、车辆参数等标定配置。
---

# Calibration 标定模块

## 模块职责

Calibration 模块是 Apollo 自动驾驶平台的**传感器标定数据管理中心**。它并非一个运行时服务，而是一个以数据为核心的配置模块，负责：

- 按车型组织和存储所有传感器的标定参数（外参、内参）
- 管理车辆物理参数（尺寸、转向比、轮距等）
- 提供 GNSS/IMU 天线杠杆臂参数
- 维护传感器元信息注册表（`sensor_meta`）
- 为 Perception、Localization、Planning、Control 等下游模块提供统一的标定数据源

模块通过 Bazel `filegroup` 将 `data/` 目录下的所有标定文件打包为 `calibrated_vehicles` 目标，供其他模块依赖引用。

## 车型命名规则

车型目录遵循 `{车辆名称}_{LiDAR数量}{Camera数量}{Radar数量}` 的命名约定：

| 车型目录 | 车辆 | LiDAR | Camera | Radar | 用途 |
|---|---|---|---|---|---|
| `kitti_140` | KITTI 数据集 | 1 | 4 | 0 | KITTI 2011_09_26 数据集回放 |
| `mkz_121` | Lincoln MKZ | 1 | 2 | 1 | 实车测试 |
| `mkz_example` | Lincoln MKZ | 2 | 3 | 2 | 示例配置，不遵循命名约定 |
| `mkz_lgsvl_321` | Lincoln MKZ | 3 | 2 | 1 | LGSVL 仿真器 |
| `nuscenes_165` | nuScenes 数据集 | 1 | 6 | 5 | nuScenes 数据集回放 |

## 目录结构

每个车型目录下按传感器类型组织标定文件：

```
modules/calibration/
├── BUILD                          # Bazel 构建：filegroup("calibrated_vehicles")
├── cyberfile.xml                  # 包元信息
├── data/
│   └── <vehicle_model>/
│       ├── sensor_meta.pb.txt     # 传感器元信息注册表
│       ├── vehicle_param.pb.txt   # 车辆物理参数
│       ├── vehicle_info.pb.txt    # 车辆身份信息（可选）
│       ├── novatel_localization_extrinsics.yaml  # NovAtel 定位外参（可选）
│       ├── camera_params/         # 相机标定参数
│       │   ├── *_extrinsics.yaml  #   外参（位姿变换）
│       │   └── *_intrinsics.yaml  #   内参（焦距、畸变等）
│       ├── lidar_params/          # LiDAR 标定参数（部分车型使用 velodyne_params/）
│       │   ├── *_novatel_extrinsics.yaml  # LiDAR→NovAtel 外参
│       │   └── *_height.yaml      #   LiDAR 安装高度
│       ├── radar_params/          # 毫米波雷达外参
│       │   └── *_extrinsics.yaml
│       ├── gnss_params/           # GNSS 天线参数
│       │   └── ant_imu_leverarm.yaml  # 天线-IMU 杠杆臂
│       ├── vehicle_params/        # 车辆-IMU 外参
│       │   └── vehicle_imu_extrinsics.yaml
│       ├── transform_conf/        # 静态 TF 变换配置（并非所有车型都有）
│       │   └── static_transform_conf.pb.txt
│       ├── camera_conf/           # 相机驱动配置（可选）
│       ├── velodyne_conf/         # Velodyne 驱动配置（可选）
│       ├── radar_conf/            # 雷达驱动配置（可选）
│       ├── gnss_conf/             # GNSS 驱动配置（可选）
│       └── video_conf/            # 视频流配置（可选）
└── README.md
```

## 核心数据结构与 Proto 定义

### SensorMeta — 传感器元信息

定义于 `modules/perception/common/proto/sensor_meta_schema.proto`，每个车型的 `sensor_meta.pb.txt` 是 `MultiSensorMeta` 的文本格式实例。

```protobuf
message SensorMeta {
  optional string name = 1;                    // 传感器名称，如 "velodyne128"
  optional SensorType type = 2;                // 传感器类型枚举
  optional SensorOrientation orientation = 3;  // 安装朝向
  optional bool is_main_sensor = 4;            // 是否为主传感器
}
```

支持的传感器类型：

| 类型 | 说明 |
|---|---|
| `VELODYNE_128` / `64` / `32` / `16` | Velodyne 系列 LiDAR |
| `LDLIDAR_4` / `LDLIDAR_1` | LD 系列 LiDAR |
| `MONOCULAR_CAMERA` | 单目相机 |
| `STEREO_CAMERA` | 双目相机 |
| `LONG_RANGE_RADAR` / `SHORT_RANGE_RADAR` | 毫米波雷达 |
| `ULTRASONIC` | 超声波传感器 |

安装朝向枚举：`FRONT`、`LEFT_FORWARD`、`LEFT`、`LEFT_BACKWARD`、`REAR`、`RIGHT_BACKWARD`、`RIGHT`、`RIGHT_FORWARD`、`PANORAMIC`。

### VehicleParam — 车辆物理参数

定义于 `modules/common_msgs/config_msgs/vehicle_config.proto`，`vehicle_param.pb.txt` 是其文本格式实例。

```protobuf
message VehicleParam {
  optional VehicleBrand brand = 1;
  optional VehicleID vehicle_id = 2;
  optional double front_edge_to_center = 3;   // 前沿到后轴中心距离
  optional double back_edge_to_center = 4;
  optional double length = 7;
  optional double width = 8;
  optional double height = 9;
  optional double max_steer_angle = 13;       // 最大转向角
  optional double steer_ratio = 16;           // 方向盘转角比
  optional double wheel_base = 17;            // 轴距
  optional double wheel_rolling_radius = 18;  // 车轮滚动半径
  // ... 制动/油门死区、延迟参数等
}
```

### ExtrinsicFile — 静态 TF 变换配置

定义于 `modules/transform/proto/static_transform_conf.proto`，用于 `static_transform_conf.pb.txt`。

```protobuf
message ExtrinsicFile {
  optional string frame_id = 1;        // 父坐标系
  optional string child_frame_id = 2;  // 子坐标系
  optional string file_path = 3;       // 外参 YAML 文件路径
  optional bool enable = 4;            // 是否启用
}
```

## 标定参数文件格式

### 外参文件（Extrinsics）

描述两个坐标系之间的刚体变换（平移 + 四元数旋转）：

```yaml
header:
  frame_id: novatel          # 父坐标系
child_frame_id: velodyne64   # 子坐标系
transform:
  translation:
    x: 0.307
    y: 0.811
    z: 0.803
  rotation:                  # 四元数 (x, y, z, w)
    x: -0.00596
    y: -0.00452
    z: 0.70736
    w: 0.70681
```

### 内参文件（Intrinsics）

描述相机的内部光学参数，遵循 ROS `CameraInfo` 格式：

```yaml
header:
  frame_id: velodyne64       # 参考坐标系
height: 512.0                # 图像高度（像素）
width: 1392.0                # 图像宽度（像素）
distortion_model: plumb_bob  # 畸变模型
D: [k1, k2, p1, p2, k3, ...]  # 畸变系数
K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]  # 3x3 内参矩阵
```

### GNSS 天线杠杆臂（Lever Arm）

描述 GNSS 天线相对于 IMU 的偏移量及不确定度：

```yaml
leverarm:
  primary:
    offset: { x: 0.0, y: -0.1, z: 0.60 }
    uncertainty: { x: 0.05, y: 0.05, z: 0.08 }
  secondary:   # 双天线配置
    offset: { x: 0.0, y: 0.98, z: 0.60 }
    uncertainty: { x: 0.05, y: 0.05, z: 0.08 }
```

### LiDAR 安装高度

```yaml
vehicle:
  parameters:
    height: 1.48        # LiDAR 距地面高度（米）
    height_var: 0.0047  # 高度方差
```

## 坐标系与数据流

### 坐标系变换链

标定模块定义了传感器之间的坐标系变换关系，典型的 TF 树结构如下：

```
world
  └── localization
        └── novatel (IMU/GNSS)
              ├── velodyne128 (主 LiDAR)
              │     ├── camera_front_6mm
              │     ├── camera_front_12mm
              │     └── ...
              ├── velodyne16_front_center
              ├── radar_front
              └── ...
```

### 数据流向

```
calibration/data/<vehicle>/
        │
        ├──→ Transform 模块        读取 static_transform_conf.pb.txt
        │    发布静态 TF 变换        加载各 *_extrinsics.yaml
        │
        ├──→ Perception 模块       读取 sensor_meta.pb.txt
        │    传感器融合/检测          读取 camera intrinsics/extrinsics
        │
        ├──→ Localization 模块     读取 novatel_localization_extrinsics.yaml
        │    多传感器融合定位          读取 gnss_params/ant_imu_leverarm.yaml
        │
        ├──→ Drivers 模块          读取 camera_conf/, velodyne_conf/ 等
        │    传感器驱动配置
        │
        └──→ Planning/Control      读取 vehicle_param.pb.txt
             规划与控制              车辆动力学参数
```

## 配置方式

### 添加新车型

1. 在 `modules/calibration/data/` 下创建新目录，遵循命名规则 `{name}_{L}{C}{R}`
2. 创建 `sensor_meta.pb.txt`，注册所有传感器及其类型、朝向，指定主传感器
3. 为每个传感器提供外参文件（`*_extrinsics.yaml`），相机还需提供内参文件（`*_intrinsics.yaml`）
4. 配置 `gnss_params/ant_imu_leverarm.yaml`，填写天线-IMU 杠杆臂测量值
5. 填写 `vehicle_param.pb.txt`，包含车辆尺寸、转向参数、制动/油门死区等
6. 创建 `transform_conf/static_transform_conf.pb.txt`，声明所有外参文件路径及坐标系关系
7. 在 HMI 中选择新车型即可加载对应标定数据

### 修改标定参数

- 外参调整：直接编辑对应的 `*_extrinsics.yaml`，修改 `translation` 和 `rotation` 字段
- 内参调整：编辑 `*_intrinsics.yaml`，更新 `K`（内参矩阵）和 `D`（畸变系数）
- 车辆参数：编辑 `vehicle_param.pb.txt`，注意字段值需符合 `VehicleParam` proto 定义

### 标定工具

Apollo 提供了配套的标定工具链（位于独立仓库或工具包中）：

- **Camera 内参标定**：基于棋盘格图案，使用 OpenCV 标定流程计算焦距、主点、畸变系数
- **Camera-LiDAR 外参标定**：通过点云与图像的特征对应关系，求解相机与 LiDAR 之间的刚体变换
- **LiDAR-GNSS/IMU 外参标定**：利用手推标定法或基于运动的标定方法，确定 LiDAR 与 IMU 之间的位姿关系
- **Radar 外参标定**：通过目标匹配确定雷达相对于主传感器的安装位姿
- **车辆动力学标定**：`modules/tools/vehicle_calibration/` 下提供了标定表可视化工具（`plot_calibration_table.py`），用于分析速度-加速度-控制指令的映射关系

::: tip 标定数据验证
修改标定参数后，建议通过回放已有 record 数据，在 Dreamview 中检查点云与图像的对齐效果、定位精度等，确认标定结果的正确性。
:::
