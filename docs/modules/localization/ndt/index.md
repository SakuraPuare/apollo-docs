---
title: "NDT 定位"
---

# NDT 定位

> 源码路径: `modules/localization/ndt/`

## 概述

NDT（Normal Distributions Transform）定位模块基于正态分布变换算法，利用激光雷达点云与预建地图进行匹配，输出高精度车辆位姿。模块作为 Cyber 组件运行，以 GPS/INS 里程计消息为主触发源，以激光雷达点云为匹配源，将 NDT 匹配结果与里程计进行融合，最终发布 `LocalizationEstimate` 和 `LocalizationStatus` 消息并广播 TF 变换。

模块的核心工作流程：

1. `Proc()` 接收里程计消息，调用 `NDTLocalization::OdometryCallback()` 将里程计位姿存入环形缓冲区，并通过 `LocalizationPoseBuffer` 融合输出定位结果
2. `LidarCallback()` 接收点云消息，从里程计缓冲区或 TF 查询预测位姿，执行 NDT 匹配，更新位姿缓冲区
3. NDT 匹配通过 `LidarLocatorNdt` 实现（位于 `ndt_locator` 子模块），输出位姿和匹配分数

## 核心类

### NDTLocalizationComponent

```cpp
class NDTLocalizationComponent final
    : public cyber::Component<localization::Gps> {
  bool Init() override;
  bool Proc(const std::shared_ptr<localization::Gps> &odometry_msg) override;
};
```

Cyber 组件入口，继承自 `cyber::Component<localization::Gps>`，以 GPS 里程计消息为主输入。负责：

- `InitConfig()` — 从 gflags 读取 topic 名称、地图路径等配置
- `InitIO()` — 创建 Reader（激光雷达、INS 状态）和 Writer（定位结果、定位状态）
- `Proc()` — 里程计主回调，驱动定位更新并发布结果
- `PublishPoseBroadcastTF()` / `PublishPoseBroadcastTopic()` — 发布 TF 变换和定位话题

### NDTLocalization

```cpp
class NDTLocalization {
  void Init();
  void OdometryCallback(const std::shared_ptr<localization::Gps> &odometry_msg);
  void LidarCallback(const std::shared_ptr<drivers::PointCloud> &lidar_msg);
  bool IsServiceStarted();
  void GetLocalization(LocalizationEstimate *localization) const;
  void GetLidarLocalization(LocalizationEstimate *lidar_localization) const;
  void GetLocalizationStatus(LocalizationStatus *localization_status) const;
};
```

NDT 定位核心逻辑类。管理里程计缓冲区、NDT 匹配器、位姿缓冲区和定位状态。内部持有 `LidarLocatorNdt` 实例执行点云匹配，持有 `LocalizationPoseBuffer` 实例进行位姿融合。

### LocalizationPoseBuffer

```cpp
class LocalizationPoseBuffer {
  void UpdateLidarPose(double timestamp,
                       const Eigen::Affine3d &locator_pose,
                       const Eigen::Affine3d &novatel_pose);
  Eigen::Affine3d UpdateOdometryPose(double timestamp,
                                     const Eigen::Affine3d &novatel_pose);
};
```

环形缓冲区（大小 20），存储激光雷达位姿与对应里程计位姿的配对记录。通过历史偏移量加权平均预测当前里程计对应的定位位姿。

### 辅助结构体

```cpp
struct LidarHeight {
  double height;
  double height_var;
};

struct TimeStampPose {
  double timestamp;
  Eigen::Affine3d pose;
};

struct LocalizationStampedPosePair {
  double timestamp;
  Eigen::Affine3d novatel_pose;
  Eigen::Affine3d locator_pose;
};
```

## 核心函数

### NDTLocalization::Init()

初始化流程：加载 TF Buffer、读取 gflags 配置（UTM 区号、NDT 分辨率、分数阈值）、从 YAML 文件加载激光雷达外参和高度参数、从地图文件夹读取 UTM 区号、配置 `LidarLocatorNdt` 的地图路径和外参。

### NDTLocalization::OdometryCallback()

里程计消息处理：

1. 解析 GPS 消息中的位置和四元数，构建 `Eigen::Affine3d` 位姿
2. 过滤零位姿（`ZeroOdometry()`，平移范数小于 0.01）
3. 存入 `odometry_buffer_` 环形缓冲区（最大 100 条，互斥锁保护）
4. 若 NDT 已初始化，调用 `pose_buffer_.UpdateOdometryPose()` 融合输出
5. 组装 `LocalizationEstimate` 和 `LocalizationStatus`

### NDTLocalization::LidarCallback()

激光雷达点云处理：

1. `LidarMsgTransfer()` 将 protobuf 点云转为 `LidarFrame`（过滤 NaN 和超高度点）
2. 从里程计缓冲区插值获取预测位姿（`QueryPoseFromBuffer()`），失败则查询 TF（`QueryPoseFromTF()`）
3. 首次调用 `lidar_locator_.Init()` 初始化 NDT 匹配器
4. 后续调用 `lidar_locator_.Update()` 执行 NDT 匹配
5. 将匹配位姿写入 `pose_buffer_` 更新融合基准
6. 监控 NDT 匹配分数，累计低分帧数用于状态评估

### NDTLocalization::QueryPoseFromBuffer()

从里程计缓冲区中反向查找时间戳小于目标时间的两个相邻位姿，进行线性插值。平移部分直接线性插值，旋转部分分解为欧拉角后分别插值再合成四元数。

### NDTLocalization::ComposeLocalizationStatus()

根据 NDT 匹配分数和 INS 解状态判定定位质量：

- **OK** — NDT 分数低于警告阈值 且 INS 为 RTK Fixed
- **WARNING** — 介于 OK 与 ERROR 之间
- **ERROR** — 累计低分帧数超阈值、或 NDT 分数超错误阈值、或 INS 非 RTK 状态

### LocalizationPoseBuffer::UpdateOdometryPose()

融合算法：遍历缓冲区中所有历史激光雷达-里程计位姿对，计算偏移量 `locator_pose - novatel_pose`，叠加到当前里程计位姿上，对平移和偏航角分别做加权平均，最终合成融合位姿。

## 配置

模块通过 gflags 配置（定义于 `localization_gflags.h`）：

| 配置项 | 说明 |
|--------|------|
| `localization_topic` | 融合定位结果发布 topic |
| `localization_ndt_topic` | 纯 NDT 激光雷达定位结果 topic |
| `localization_msf_status` | 定位状态 topic |
| `lidar_topic` | 激光雷达点云输入 topic |
| `ins_stat_topic` | INS 状态输入 topic |
| `broadcast_tf_frame_id` | TF 父坐标系 ID |
| `broadcast_tf_child_frame_id` | TF 子坐标系 ID |
| `map_dir` / `ndt_map_dir` / `local_map_name` | NDT 地图路径 |
| `local_utm_zone_id` | UTM 区号 |
| `if_utm_zone_id_from_folder` | 是否从地图文件夹读取 UTM 区号 |
| `online_resolution` | NDT 在线匹配分辨率 |
| `lidar_extrinsics_file` | 激光雷达外参文件路径（YAML） |
| `lidar_height_file` | 激光雷达高度文件路径（YAML） |
| `lidar_height_default` | 默认激光雷达高度 |
| `ndt_bad_score_count_threshold` | 累计低分帧数阈值 |
| `ndt_warnning_ndt_score` | NDT 匹配分数警告阈值 |
| `ndt_error_ndt_score` | NDT 匹配分数错误阈值 |
| `ndt_debug_log_flag` | 是否输出调试日志 |

激光雷达外参从 YAML 文件加载，格式为 `transform.translation.{x,y,z}` 和 `transform.rotation.{x,y,z,w}`。

## 调用关系

```text
NDTLocalizationComponent::Proc (GPS 里程计触发)
  -> NDTLocalization::OdometryCallback
       -> LocalizationPoseBuffer::UpdateOdometryPose (位姿融合)
       -> ComposeLocalizationEstimate (组装定位结果)
       -> ComposeLocalizationStatus (评估定位状态)
  -> PublishPoseBroadcastTopic / PublishPoseBroadcastTF (发布结果)

NDTLocalizationComponent::LidarCallback (激光雷达触发)
  -> NDTLocalization::LidarCallback
       -> LidarMsgTransfer (点云格式转换)
       -> QueryPoseFromBuffer / QueryPoseFromTF (获取预测位姿)
       -> LidarLocatorNdt::Init / Update (NDT 匹配)
       -> LocalizationPoseBuffer::UpdateLidarPose (更新融合基准)
  -> PublishLidarPoseBroadcastTopic (发布激光雷达定位)
```
