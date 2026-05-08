---
title: "多传感器融合定位"
---

# MSF 模块

> 源码路径: `modules/localization/msf/`

## 概述

MSF (Multi-Sensor Fusion) 是 Apollo 的多传感器融合定位模块，融合 IMU、GNSS 和 LiDAR 三种传感器数据输出高精度定位结果。该模块以 CyberRT Component 形式运行，IMU 消息作为主触发源（10ms 定时器周期），GNSS 和 LiDAR 作为辅助观测参与融合。

模块核心数据流如下：

```text
IMU (主触发) ──→ SINS 惯导推算 ──→ 融合定位输出
GNSS (辅助) ──→ GNSS 位置/航向观测 ──→ EKF 融合修正
LiDAR (辅助) ──→ 点云匹配定位 ──→ EKF 融合修正
```

子目录结构：

| 目录 | 功能 |
|------|------|
| `local_integ/` | 融合核心实现，包含 IMU/GNSS/LiDAR 处理管线与集成接口 |
| `local_map/` | 传统地图结构（无损、有损、NDT 地图） |
| `local_pyramid_map/` | 金字塔多分辨率地图结构，用于 LiDAR 定位匹配 |
| `local_tool/` | 离线工具：数据提取、可视化、地图构建 |
| `params/` | 标定参数文件（外参、天线杆臂等） |
| `common/` | IO 与通用工具 |

## 核心类

### MSFLocalization

融合定位主类，管理传感器回调、参数初始化与定位发布。内部持有 `LocalizationInteg` 实例完成实际融合计算。

```cpp
// modules/localization/msf/msf_localization.h
namespace apollo::localization {
class MSFLocalization {
 public:
  apollo::common::Status Init();
  void OnPointCloud(const std::shared_ptr<drivers::PointCloud> &message);
  void OnRawImu(const std::shared_ptr<drivers::gnss::Imu> &imu_msg);
  void OnGnssRtkObs(const std::shared_ptr<drivers::gnss::EpochObservation> &raw_obs_msg);
  void OnGnssBestPose(const std::shared_ptr<drivers::gnss::GnssBestPose> &bestgnsspos_msg);
  void OnGnssHeading(const std::shared_ptr<drivers::gnss::Heading> &gnss_heading_msg);
  void OnLocalizationTimer();  // 10ms 周期定时器回调

 private:
  msf::LocalizationInteg localization_integ_;
  msf::LocalizationIntegParam localization_param_;
  Eigen::Quaternion<double> imu_vehicle_quat_;   // IMU-车辆坐标系旋转
  Eigen::Vector3d imu_vehicle_translation_;       // IMU-车辆平移补偿
};
}
```

### MSFLocalizationComponent

CyberRT 组件入口，继承自 `cyber::Component<drivers::gnss::Imu>`，以 IMU 消息作为 `Proc` 触发源。负责创建 Reader 订阅传感器话题并将消息路由到 `MSFLocalization`。

```cpp
// modules/localization/msf/msf_localization_component.h
class MSFLocalizationComponent final : public cyber::Component<drivers::gnss::Imu> {
  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& imu_msg) override;

 private:
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_listener_;
  std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>> bestgnsspos_listener_;
  std::shared_ptr<cyber::Reader<drivers::gnss::Heading>> gnss_heading_listener_;
  MSFLocalization localization_;
  std::shared_ptr<LocalizationMsgPublisher> publisher_;
};
```

### LocalizationMsgPublisher

定位结果发布器，创建 CyberRT Writer 发布融合定位、LiDAR 定位、GNSS 定位与定位状态到各自话题，同时通过 `TransformBroadcaster` 发布 TF 变换。

```cpp
class LocalizationMsgPublisher {
  void PublishPoseBroadcastTF(const LocalizationEstimate& localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate& localization);
  void PublishLocalizationMsfGnss(const LocalizationEstimate& localization);
  void PublishLocalizationMsfLidar(const LocalizationEstimate& localization);
  void PublishLocalizationStatus(const LocalizationStatus& localization_status);
};
```

### LocalizationInteg / LocalizationIntegImpl

融合接口与实现（Pimpl 模式）。`LocalizationInteg` 对外暴露传感器处理入口，内部委托给 `LocalizationIntegImpl`，后者协调四个子处理器：

- **MeasureRepublishProcess** -- 观测数据重发布
- **LocalizationIntegProcess** -- SINS + EKF 融合核心
- **LocalizationGnssProcess** -- GNSS 原始观测解析与处理
- **LocalizationLidarProcess** -- LiDAR 点云匹配定位

### LocalizationLidar

LiDAR 定位核心类，使用金字塔多分辨率地图（`PyramidMap`）进行点云匹配，输出车辆在地图坐标系中的位姿估计及协方差。

### LocalizationIntegParam

融合参数结构体，涵盖 INS 对齐、GNSS 模式、LiDAR 匹配参数、IMU 延迟阈值等配置项。

## 核心函数

### MSFLocalization::Init

初始化入口。调用 `InitParams` 从 gflags 加载全部融合参数（GNSS 模式、LiDAR 地图路径、IMU-车辆外参、天线杆臂等），然后调用 `LocalizationInteg::Init` 完成融合引擎初始化。同时启动 10ms 周期定时器驱动 IMU 处理。

### MSFLocalization::OnRawImu

IMU 处理主函数。将 IMU 数据送入 `LocalizationInteg::RawImuProcessFlu/Rfu` 进行惯导推算与融合，获取融合结果后执行以下操作：

1. 构造并发布 `LocalizationStatus`
2. 调用 `CompensateImuVehicleExtrinsic` 将 IMU 坐标系结果补偿到车辆坐标系
3. 发布 TF 变换与融合定位话题

### MSFLocalization::OnPointCloud

LiDAR 点云回调。按 `FLAGS_point_cloud_step` 间隔采样，将点云送入 `LocalizationInteg::PcdProcess` 进行匹配定位，发布 LiDAR 定位结果。

### MSFLocalization::OnGnssBestPose / OnGnssRtkObs / OnGnssHeading

GNSS 回调函数组。将 GNSS 最佳位置、RTK 原始观测、航向角分别送入 `LocalizationInteg` 对应接口。当 GNSS 仅用于初始化（`FLAGS_gnss_only_init`）且定位已稳定时跳过处理。

### MSFLocalization::CompensateImuVehicleExtrinsic

将 IMU 坐标系下的定位结果补偿到车辆坐标系：旋转通过四元数乘法 `quat_vehicle_world = imu_quat * imu_vehicle_quat` 转换，位置通过旋转矩阵补偿平移偏移，最终更新 heading 与欧拉角。

### MSFLocalization::OnLocalizationTimer

10ms 定时器回调，从缓存中取出最新 IMU 消息调用 `OnRawImu`，驱动融合循环。

## 配置

### gflags 命令行参数

模块通过 gflags 管理运行时配置，主要参数类别：

| 类别 | 参数示例 | 说明 |
|------|---------|------|
| 融合模式 | `FLAGS_integ_ins_can_self_align`, `FLAGS_gnss_mode` | INS 自对齐、GNSS 模式选择 |
| LiDAR | `FLAGS_lidar_localization_mode`, `FLAGS_lidar_filter_size`, `FLAGS_map_dir` | LiDAR 定位模式、滤波尺寸、地图路径 |
| IMU 外参 | `FLAGS_imu_vehicle_qx/qy/qz/qw`, `FLAGS_if_vehicle_imu_from_file` | IMU-车辆旋转四元数，支持文件加载 |
| GNSS 杆臂 | `FLAGS_imu_to_ant_offset_x/y/z`, `FLAGS_ant_imu_leverarm_file` | IMU 到 GNSS 天线的偏移量 |
| 阈值 | `FLAGS_imu_delay_time_threshold_*`, `FLAGS_localization_std_*_threshold_*` | IMU 延迟与定位精度阈值 |
| TF | `FLAGS_broadcast_tf_frame_id`, `FLAGS_broadcast_tf_child_frame_id` | TF 广播坐标系名称 |

### YAML 标定文件（`params/`）

| 文件 | 用途 |
|------|------|
| `imu_localization_extrinsics.yaml` | IMU 到 localization 坐标系的外参（旋转+平移） |
| `novatel_localization_extrinsics.yaml` | NovAtel GNSS 设备外参 |
| `gnss_params/ant_imu_leverarm.yaml` | GNSS 天线到 IMU 的杆臂参数（offset + uncertainty） |
| `vehicle_params/vehicle_imu_extrinsics.yaml` | 车辆到 IMU 的外参 |
| `velodyne_params/*.yaml` | Velodyne 雷达标定与安装高度 |

## 调用关系

```text
MSFLocalizationComponent::Init()
  ├── InitConfig() → 读取 gflags topic 配置
  ├── MSFLocalization::Init()
  │     ├── InitParams() → 加载 gflags 参数到 LocalizationIntegParam
  │     │     ├── LoadGnssAntennaExtrinsic() → 从 YAML 加载杆臂
  │     │     ├── LoadImuVehicleExtrinsic() → 从 YAML 加载 IMU 外参
  │     │     └── LoadZoneIdFromFolder() → 从地图目录读取 UTM Zone
  │     └── LocalizationInteg::Init()
  │           └── LocalizationIntegImpl::Init()
  │                 ├── LocalizationGnssProcess::Init()
  │                 ├── LocalizationLidarProcess::Init()
  │                 │     └── LocalizationLidar::Init() → 加载金字塔地图
  │                 └── LocalizationIntegProcess::Init()
  └── InitIO() → 创建 Reader/Writer，绑定回调

MSFLocalizationComponent::Proc(imu_msg)
  └── MSFLocalization::OnRawImuCache() → 缓存最新 IMU

OnLocalizationTimer() [10ms]
  └── MSFLocalization::OnRawImu()
        ├── LocalizationInteg::RawImuProcessFlu/Rfu()
        │     └── LocalizationIntegImpl::ImuProcessImpl()
        │           ├── LocalizationIntegProcess (SINS + EKF 融合)
        │           └── MeasureRepublishProcess
        ├── CompensateImuVehicleExtrinsic() → IMU→车辆坐标系补偿
        └── LocalizationMsgPublisher::Publish*() → 发布 TF + 话题

OnPointCloud(pointcloud)          → LocalizationInteg::PcdProcess() → LiDAR 匹配
OnGnssBestPose(bestpose)          → LocalizationInteg::GnssBestPoseProcess()
OnGnssRtkObs(epoch_obs)           → LocalizationInteg::RawObservationProcess()
OnGnssHeading(heading)            → LocalizationInteg::GnssHeadingProcess()
```
