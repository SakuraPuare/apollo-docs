---
title: "RTK 定位模块"
---

# RTK 定位模块

> 源码路径：`modules/localization/rtk/`

## 概述

RTK 定位模块是 Apollo 最基础的定位方案，直接融合 GNSS（GPS）和 IMU 数据输出定位结果。GPS 提供绝对位置和姿态，IMU 提供线加速度和角速度，两者通过时间戳对齐和插值后组合成完整的 `LocalizationEstimate`。该模块适用于开阔道路等 GNSS 信号良好的场景，不依赖高精地图或激光雷达。

## 架构

```text
GPS 消息 ──→ RTKLocalizationComponent::Proc()
                    │
                    ├── RTKLocalization::GpsCallback()
                    │       ├── FindMatchingIMU()  ← IMU 缓冲区
                    │       ├── ComposeLocalizationMsg()  ← GPS + IMU 融合
                    │       ├── FillLocalizationStatusMsg()  ← GPS 状态
                    │       └── RunWatchDog()  ← 延迟监控
                    │
                    ├── PublishPoseBroadcastTopic()  ← /apollo/localization/pose
                    ├── PublishPoseBroadcastTF()      ← TF 广播
                    └── PublishLocalizationStatus()    ← /apollo/localization/msf_status

IMU 消息 ──→ RTKLocalization::ImuCallback()  → imu_list_
GPS 状态 ──→ RTKLocalization::GpsStatusCallback()  → gps_status_list_
```

## 核心类

### RTKLocalization

定位核心类，负责 GPS/IMU 数据的缓存、匹配、插值和融合。

```cpp
class RTKLocalization {
 public:
  void InitConfig(const rtk_config::Config &config);

  void GpsCallback(const std::shared_ptr<localization::Gps> &gps_msg);
  void GpsStatusCallback(const std::shared_ptr<drivers::gnss::InsStat> &status_msg);
  void ImuCallback(const std::shared_ptr<localization::CorrectedImu> &imu_msg);

  bool IsServiceStarted();
  void GetLocalization(LocalizationEstimate *localization);
  void GetLocalizationStatus(LocalizationStatus *localization_status);

 private:
  void RunWatchDog(double gps_timestamp);
  void PrepareLocalizationMsg(const localization::Gps &gps_msg,
                              LocalizationEstimate *localization,
                              LocalizationStatus *localization_status);
  void ComposeLocalizationMsg(const localization::Gps &gps,
                              const localization::CorrectedImu &imu,
                              LocalizationEstimate *localization);
  bool FindMatchingIMU(double gps_timestamp_sec, CorrectedImu *imu_msg);
  bool InterpolateIMU(const CorrectedImu &imu1, const CorrectedImu &imu2,
                      double timestamp_sec, CorrectedImu *imu_msg);
  template <class T>
  T InterpolateXYZ(const T &p1, const T &p2, double frac1);
  bool FindNearestGpsStatus(double gps_timestamp_sec, drivers::gnss::InsStat *status);

  std::list<localization::CorrectedImu> imu_list_;       // IMU 缓冲区（最大 50）
  std::list<drivers::gnss::InsStat> gps_status_list_;    // GPS 状态缓冲区（最大 10）
  std::vector<double> map_offset_;                        // 地图偏移 [x, y, z]
};
```

**源码**：`modules/localization/rtk/rtk_localization.h`、`modules/localization/rtk/rtk_localization.cc`

### RTKLocalizationComponent

CyberRT 组件，继承 `cyber::Component<Gps>`，驱动整个 RTK 定位流程。

```cpp
class RTKLocalizationComponent final : public cyber::Component<localization::Gps> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<localization::Gps> &gps_msg) override;

 private:
  bool InitConfig();
  bool InitIO();
  bool GetLocalizationToImuTF();
  void PublishPoseBroadcastTF(const LocalizationEstimate &localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate &localization);
  void PublishLocalizationStatus(const LocalizationStatus &localization_status);

  std::unique_ptr<RTKLocalization> localization_;
  std::unique_ptr<apollo::transform::TransformBroadcaster> tf2_broadcaster_;
};

CYBER_REGISTER_COMPONENT(RTKLocalizationComponent);
```

**源码**：`modules/localization/rtk/rtk_localization_component.h`

## 核心函数

### RTKLocalization::GpsCallback()

GPS 消息到达时的主回调，触发定位融合流程。

```cpp
void RTKLocalization::GpsCallback(const std::shared_ptr<localization::Gps> &gps_msg) {
  // 检查 GPS 消息延迟
  double time_delay = Clock::NowInSeconds() - last_received_timestamp_sec_;
  if (time_delay > gps_time_delay_tolerance_) { monitor_logger_.WARN(...); }

  // 检查 IMU 和 GPS 状态缓冲区非空
  if (imu_list_.empty()) { return; }
  if (gps_status_list_.empty()) { return; }

  // 融合定位
  PrepareLocalizationMsg(*gps_msg, &last_localization_result_,
                         &last_localization_status_result_);
  service_started_ = true;

  // 看门狗
  RunWatchDog(gps_msg->header().timestamp_sec());
}
```

### RTKLocalization::ComposeLocalizationMsg()

GPS 和 IMU 数据融合的核心方法：

```cpp
void RTKLocalization::ComposeLocalizationMsg(
    const localization::Gps &gps_msg,
    const localization::CorrectedImu &imu_msg,
    LocalizationEstimate *localization) {
  // 位置：GPS 位置减去地图偏移
  mutable_pose->mutable_position()->set_x(pose.position().x() - map_offset_[0]);
  mutable_pose->mutable_position()->set_y(pose.position().y() - map_offset_[1]);
  mutable_pose->mutable_position()->set_z(pose.position().z() - map_offset_[2]);

  // 姿态：直接使用 GPS 四元数
  mutable_pose->mutable_orientation()->CopyFrom(pose.orientation());
  mutable_pose->set_heading(QuaternionToHeading(...));

  // 线速度：直接使用 GPS
  mutable_pose->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());

  // 线加速度：IMU 加速度从车体坐标旋转到地图坐标
  Vector3d vec = QuaternionRotate(localization->pose().orientation(), orig);

  // 角速度：同样从车体坐标旋转到地图坐标
  Vector3d vec = QuaternionRotate(localization->pose().orientation(), orig);

  // 欧拉角：直接使用 IMU
  mutable_pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
}
```

**数据来源**：

| 字段 | 来源 | 坐标变换 |
|------|------|----------|
| position | GPS | 减去 map_offset |
| orientation / heading | GPS | 四元数转航向角 |
| linear_velocity | GPS | 无 |
| linear_acceleration | IMU | 车体→地图（四元数旋转） |
| angular_velocity | IMU | 车体→地图（四元数旋转） |
| euler_angles | IMU | 无 |

### RTKLocalization::FindMatchingIMU()

在 IMU 缓冲区中查找与 GPS 时间戳匹配的 IMU 消息，通过线性插值对齐时间。

```cpp
bool RTKLocalization::FindMatchingIMU(const double gps_timestamp_sec,
                                      CorrectedImu *imu_msg) {
  // 遍历 imu_list_，找到第一个时间戳 > GPS 时间戳的 IMU
  // 使用前一条和当前一条 IMU 进行线性插值
  // 若所有 IMU 都早于 GPS，使用最新一条（无外推）
  // 若时间差 > gps_imu_time_diff_threshold_，报错
}
```

### RTKLocalization::InterpolateIMU()

在两条 IMU 消息之间按时间戳线性插值，包括角速度、线加速度和欧拉角。

```cpp
bool RTKLocalization::InterpolateIMU(const CorrectedImu &imu1,
                                     const CorrectedImu &imu2,
                                     const double timestamp_sec,
                                     CorrectedImu *imu_msg) {
  double frac1 = (timestamp_sec - imu1.header().timestamp_sec()) / time_diff;
  // 对 angular_velocity, linear_acceleration, euler_angles 分别插值
}
```

### RTKLocalization::FillLocalizationStatusMsg()

根据 GNSS 解算类型（`InsStat.pos_type`）判断定位质量：

| GNSS 解算类型 | 定位状态 | 消息 |
|---------------|----------|------|
| NARROW_INT / INS_RTKFIXED | OK | 空 |
| NARROW_FLOAT / INS_RTKFLOAT | WARNING | "Unstable" |
| 其他 | ERROR | "Very Unstable" |

### RTKLocalization::RunWatchDog()

监控 GPS 和 IMU 消息延迟，当延迟超过 `report_threshold_err_num_` 个周期时通过 `monitor_logger_` 报警。

## 配置

通过 `rtk_config::Config` protobuf 加载：

| 字段 | 类型 | 说明 |
|------|------|------|
| imu_list_max_size | int | IMU 缓冲区最大长度 |
| gps_imu_time_diff_threshold | double | GPS-IMU 时间差阈值（秒） |
| map_offset_x / y / z | double | 地图坐标偏移量 |
| localization_publish_freq | int | 定位发布频率（Hz） |

## 调用关系

- **上游**：GPS 驱动模块（`gnss` 驱动）发布 `Gps` 消息；IMU 驱动发布 `CorrectedImu` 消息；GNSS 板卡发布 `InsStat`
- **下游**：发布 `LocalizationEstimate` 到 `/apollo/localization/pose`；广播 TF 变换
- **被依赖**：`Localization` 模块（MSF 融合定位）可使用 RTK 结果作为 GNSS 子模块输入
