---
title: "多传感器融合"
---

# 多传感器融合

> 源码路径: `modules/perception/multi_sensor_fusion/`

## 概述

多传感器融合模块负责将来自激光雷达、毫米波雷达和摄像头等不同传感器的检测结果进行概率融合，输出统一的障碍物列表。模块以 Cyber RT Component 形式运行，接收 `SensorFrameMessage` 消息，经融合处理后发布 `PerceptionObstacles` 到下游规划模块。

融合算法采用概率融合（ProbabilisticFusion）框架，核心流程包括：数据关联、航迹更新、航迹维护和输出门控。模块支持前景目标与背景目标分别维护航迹，并通过高精地图 ROI 校验过滤无效目标。

## 核心类

### MultiSensorFusionComponent

Cyber RT 组件，继承自 `cyber::Component<SensorFrameMessage>`，是融合模块的入口。

```cpp
class MultiSensorFusionComponent : public cyber::Component<SensorFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;
 private:
  bool InitAlgorithmPlugin(const FusionComponentConfig &config);
  bool InternalProc(const std::shared_ptr<SensorFrameMessage const>& in_message,
                    std::shared_ptr<PerceptionObstacles> out_message,
                    std::shared_ptr<SensorFrameMessage> viz_message);
  std::unique_ptr<fusion::BaseFusionSystem> fusion_;
  map::HDMapInput* hdmap_input_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> inner_writer_;
};
```

### BaseFusionSystem

融合系统抽象基类，定义融合接口。通过注册器机制实现插件化。

```cpp
class BaseFusionSystem {
 public:
  virtual bool Init(const FusionInitOptions& options) = 0;
  virtual bool Fuse(const base::FrameConstPtr& sensor_frame,
                    std::vector<base::ObjectPtr>* fused_objects) = 0;
  virtual std::string Name() const = 0;
};
```

### ProbabilisticFusion

`BaseFusionSystem` 的默认实现，基于概率框架的多传感器融合系统。内部组合了 `BaseDataAssociation`（数据关联）、`BaseTracker`（航迹跟踪）和 `BaseGatekeeper`（发布门控）三个子模块。

```cpp
class ProbabilisticFusion : public BaseFusionSystem {
 private:
  ScenePtr scenes_;
  std::vector<std::shared_ptr<BaseTracker>> trackers_;
  std::unique_ptr<BaseDataAssociation> matcher_;
  std::unique_ptr<BaseGatekeeper> gate_keeper_;
  FusionParams params_;
};
```

### Track

融合航迹类，维护单个目标在各传感器上的观测记录及融合状态。存储激光雷达、毫米波雷达、摄像头三种传感器的 `SensorObjectMap`，并管理存在概率（existence_prob）和跟踪概率（toic_prob）。

### Scene

场景容器，分别维护前景航迹（`foreground_tracks_`）和背景航迹（`background_tracks_`）列表。

### BaseTracker

航迹跟踪器抽象基类，定义 `UpdateWithMeasurement` 和 `UpdateWithoutMeasurement` 两个核心更新接口。默认实现为 `PbfTracker`。

### BaseDataAssociation

数据关联抽象基类，默认实现为 `HmTracksObjectsMatch`，采用匈牙利匹配算法完成目标与航迹的关联。

### BaseGatekeeper

发布门控抽象基类，默认实现为 `PbfGatekeeper`，根据航迹质量决定是否对外发布融合目标。

## 核心函数

### MultiSensorFusionComponent::Init

初始化融合组件：加载 protobuf 配置，实例化融合算法插件，创建 Cyber RT Writer 通道，初始化传感器管理器和高精地图。

### MultiSensorFusionComponent::Proc

消息处理入口。过滤已融合阶段的消息，调用 `InternalProc` 执行融合，仅主传感器的结果发布到输出通道。

### MultiSensorFusionComponent::InternalProc

融合主逻辑。递增序列号，调用 `fusion_->Fuse()` 获取融合结果，执行 ROI 校验，序列化为 `PerceptionObstacles` protobuf 消息并发布。

### ProbabilisticFusion::Init

初始化概率融合系统：解析配置，设置各传感器最大不可见周期，初始化数据关联器、门控器和跟踪器。

### ProbabilisticFusion::Fuse

融合单帧传感器数据，主要步骤：

1. **保存帧数据** — 根据传感器类型过滤后存入 `SensorDataManager`
2. **查询关联帧** — 获取当前融合时刻所有传感器的最新帧
3. **逐帧融合** — 对每个帧调用 `FuseFrame`
4. **收集融合结果** — 通过 `CollectFusedObjects` 输出发布级目标

### ProbabilisticFusion::FuseFrame

单帧融合处理：依次执行前景航迹融合（`FuseForegroundTrack`）、背景航迹融合（`FusebackgroundTrack`）和丢失航迹清除（`RemoveLostTrack`）。

### ProbabilisticFusion::FuseForegroundTrack

前景目标融合流程：

1. 调用 `matcher_->Associate()` 执行数据关联
2. `UpdateAssignedTracks` — 用匹配的测量更新已有航迹
3. `UpdateUnassignedTracks` — 对未匹配航迹执行无测量更新
4. `CreateNewTracks` — 为未匹配的测量创建新航迹

### ProbabilisticFusion::CollectFusedObjects

遍历前景和背景航迹，通过 `gate_keeper_->AbleToPublish()` 过滤后，将合格目标序列化为 `base::ObjectPtr` 输出。每个目标包含融合后的位姿、速度及各传感器的原始测量信息。

## 配置

### FusionComponentConfig

组件级配置，定义在 `fusion_component_config.proto` 中：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `fusion_param` | PluginParam | - | 融合算法插件参数（名称、配置路径） |
| `object_in_roi_check` | bool | false | 是否启用高精地图 ROI 校验 |
| `radius_for_roi_object_check` | double | 0 | ROI 校验半径（米） |
| `output_obstacles_channel_name` | string | `/perception/vehicle/obstacles` | 输出障碍物通道名 |
| `output_viz_fused_content_channel_name` | string | `/perception/inner/visualization/FusedObjects` | 可视化通道名 |

### ProbabilisticFusionConfig

概率融合算法配置，定义在 `probabilistic_fusion_config.proto` 中：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_lidar` | bool | true | 是否使用激光雷达 |
| `use_radar` | bool | true | 是否使用毫米波雷达 |
| `use_camera` | bool | true | 是否使用摄像头 |
| `tracker_param` | PluginParam | - | 跟踪器插件参数 |
| `data_association_param` | PluginParam | - | 数据关联插件参数 |
| `gatekeeper_param` | PluginParam | - | 门控器插件参数 |
| `prohibition_sensors` | repeated string | - | 禁止创建新航迹的传感器列表 |
| `max_lidar_invisible_period` | double | 0.25 | 激光雷达最大不可见周期（秒） |
| `max_radar_invisible_period` | double | 0.50 | 毫米波雷达最大不可见周期（秒） |
| `max_camera_invisible_period` | double | 0.75 | 摄像头最大不可见周期（秒） |
| `max_cached_frame_num` | int64 | 50 | 传感器最大缓存帧数 |

### 运行时配置文件

`conf/multi_sensor_fusion_config.pb.txt` 使用 `ProbabilisticFusion` 插件，启用 ROI 校验（半径 120 米），输出通道为 `/apollo/perception/obstacles`。

## 调用关系

```text
Cyber RT 调度
  |
  v
MultiSensorFusionComponent::Proc(SensorFrameMessage)
  |
  +---> Init() [初始化时]
  |       |
  |       +---> InitAlgorithmPlugin()
  |       |       |
  |       |       +---> BaseFusionSystemRegisterer::GetInstanceByName("ProbabilisticFusion")
  |       |       +---> ProbabilisticFusion::Init()
  |       |       |       +---> HmTracksObjectsMatch::Init()    [数据关联]
  |       |       |       +---> PbfGatekeeper::Init()           [门控]
  |       |       |       +---> PbfTracker::InitParams()        [跟踪器]
  |       |       +---> HDMapInput::Init()                      [高精地图]
  |       |
  |       +---> SensorManager::Init()
  |
  +---> InternalProc(message, out_message, viz_message)
          |
          +---> ProbabilisticFusion::Fuse(frame, fused_objects)
          |       |
          |       +---> SensorDataManager::AddSensorMeasurements() [缓存帧数据]
          |       +---> SensorDataManager::GetLatestFrames()       [查询关联帧]
          |       +---> FuseFrame(frame)                           [逐帧融合]
          |       |       |
          |       |       +---> FuseForegroundTrack()
          |       |       |       +---> HmTracksObjectsMatch::Associate()  [匈牙利匹配]
          |       |       |       +---> UpdateAssignedTracks()            [更新已匹配航迹]
          |       |       |       +---> UpdateUnassignedTracks()          [更新未匹配航迹]
          |       |       |       +---> CreateNewTracks()                 [创建新航迹]
          |       |       |
          |       |       +---> FusebackgroundTrack()             [背景航迹融合]
          |       |       +---> RemoveLostTrack()                 [清除丢失航迹]
          |       |
          |       +---> CollectFusedObjects()
          |               +---> PbfGatekeeper::AbleToPublish()   [门控判断]
          |               +---> CollectObjectsByTrack()           [序列化融合目标]
          |
          +---> MsgSerializer::SerializeMsg()     [序列化为 protobuf]
          +---> writer_->Write(out_message)       [发布融合结果]
```
