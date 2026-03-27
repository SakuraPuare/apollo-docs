---
title: Perception 感知模块
description: Apollo 感知模块架构分析 — 涵盖 Camera、LiDAR、Radar、Fusion、Traffic Light 等子系统的组件设计、DAG 数据流与配置方式
---

# Perception 感知模块

## 模块职责概述

Perception 模块是 Apollo 自动驾驶系统的"眼睛"，负责从多种传感器（相机、LiDAR、毫米波雷达、4D 雷达）获取原始数据，经过预处理、检测、跟踪、融合等流水线，最终输出结构化的障碍物列表（`PerceptionObstacles`）和交通信号灯状态，供下游 Prediction、Planning 模块使用。

核心能力包括：

- 基于相机的 2D/3D 目标检测、位置估计与跟踪
- 基于 LiDAR 点云的预处理、地面检测、ROI 过滤、3D 目标检测与跟踪
- 基于毫米波雷达 / 4D 雷达的目标检测
- 多传感器融合（Multi-Sensor Fusion）
- 交通信号灯检测、识别与跟踪
- 车道线检测
- 障碍物（护栏等）识别

## 目录结构说明

```
modules/perception/
├── common/                          # 公共库：基础数据结构、算法、推理引擎、接口定义
├── data/                            # 模型文件、配置参数、flag 文件
├── launch/                          # CyberRT launch 文件（按传感器组合启动）
├── tools/                           # 辅助工具
├── perception_plugin/               # 插件扩展（下采样、过滤器等）
│
│   ── Camera 子系统 ──
├── camera_detection_multi_stage/    # 相机多阶段检测（YOLO / YOLOx3D）
├── camera_detection_single_stage/   # 相机单阶段检测（SMOKE / CADDN）
├── camera_detection_bev/            # 相机 BEV 检测（PETR）
├── camera_detection_occupancy/      # 相机占据网格检测（BEVFormer）
├── camera_location_estimation/      # 2D→3D 位置估计
├── camera_location_refinement/      # 3D 位置精修
├── camera_tracking/                 # 相机目标跟踪（OMT）
│
│   ── LiDAR 子系统 ──
├── pointcloud_preprocess/           # 点云预处理（运动补偿、过滤）
├── pointcloud_map_based_roi/        # 基于高精地图的 ROI 过滤
├── pointcloud_ground_detection/     # 地面点检测与分离
├── pointcloud_motion/               # 点云运动估计
├── pointcloud_semantics/            # 点云语义分割
├── lidar_detection/                 # LiDAR 3D 目标检测（CenterPoint / PointPillars / CNNSeg）
├── lidar_cpdet_detection/           # LiDAR CPDet 检测
├── lidar_detection_filter/          # 检测结果过滤（背景过滤、ROI 边界过滤）
├── lidar_segmentation/              # LiDAR 语义分割
├── lidar_tracking/                  # LiDAR 目标跟踪（多 LiDAR 融合跟踪）
├── lidar_output/                    # LiDAR 结果输出 / 融合输出
│
│   ── Radar 子系统 ──
├── radar_detection/                 # 传统毫米波雷达检测（Continental）
├── radar4d_detection/               # 4D 雷达检测（Oculii，含 PointPillars）
│
│   ── Fusion ──
├── multi_sensor_fusion/             # 多传感器融合（概率融合 PBF）
│
│   ── Traffic Light 子系统 ──
├── traffic_light_region_proposal/   # 信号灯 ROI 提取
├── traffic_light_detection/         # 信号灯检测
├── traffic_light_recognition/       # 信号灯颜色识别
├── traffic_light_tracking/          # 信号灯状态跟踪
│
│   ── 其他 ──
├── lane_detection/                  # 车道线检测（DarkSCNN / DenseLine）
├── barrier_recognition/             # 护栏识别
├── motion_service/                  # 车辆运动状态服务
└── msg_adapter/                     # 消息适配器
```

## 核心组件与接口

### 组件注册机制

所有感知组件均基于 CyberRT 的 `Component` 模板类实现，通过 `CYBER_REGISTER_COMPONENT` 宏注册。算法插件则使用自定义的工厂注册机制：

```cpp
// 组件注册 — 每个子模块的 component.h 末尾
CYBER_REGISTER_COMPONENT(LidarDetectionComponent);

// 算法插件注册 — 通过 registerer.h 中的宏
PERCEPTION_REGISTER_REGISTERER(BaseLidarDetector);
#define PERCEPTION_REGISTER_LIDARDETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseLidarDetector, name)
```

`PERCEPTION_REGISTER_CLASS` 利用 `__attribute__((constructor))` 在动态库加载时自动将实现类注册到全局工厂 `GlobalFactoryMap` 中，运行时通过名称字符串查找并实例化。

### 核心基类 / 接口

| 基类 | 命名空间 | 职责 | 注册宏 |
|------|----------|------|--------|
| `BaseObstacleDetector` | `camera` | 相机障碍物检测接口 | `REGISTER_OBSTACLE_DETECTOR` |
| `BaseLidarDetector` | `lidar` | LiDAR 障碍物检测接口 | `PERCEPTION_REGISTER_LIDARDETECTOR` |
| `BaseRadarObstaclePerception` | `radar` | 雷达障碍物感知接口 | — |
| `BaseFusionSystem` | `fusion` | 多传感器融合接口 | `FUSION_REGISTER_FUSIONSYSTEM` |
| `BaseTransformer` | `camera` | 2D→3D 位置变换接口 | — |
| `BaseObstacleTracker` | `camera` | 相机目标跟踪接口 | — |
| `BaseFeatureExtractor` | `camera` | 跟踪特征提取接口 | — |
| `BasePointCloudPreprocessor` | `lidar` | 点云预处理接口 | — |
| `BaseBarrierRecognizer` | `perception` | 护栏识别接口 | — |

### 核心数据结构

`base::Object` 是贯穿整个感知流水线的核心数据结构，包含：

```cpp
struct Object {
  int id;                          // 帧内 ID
  PointCloud<PointD> polygon;      // 凸包
  Eigen::Vector3f direction;       // 主方向
  float theta;                     // 偏航角
  Eigen::Vector3d center;          // 包围盒中心
  Eigen::Vector3f size;            // [length, width, height]
  ObjectType type;                 // 目标类型
  int track_id;                    // 跟踪 ID
  Eigen::Vector3f velocity;        // 速度
  Eigen::Vector3f acceleration;    // 加速度
  // 各传感器补充信息
  LidarObjectSupplement lidar_supplement;
  RadarObjectSupplement radar_supplement;
  CameraObjectSupplement camera_supplement;
  FusionObjectSupplement fusion_supplement;
};
```

`base::Frame` 封装单帧传感器数据：

```cpp
struct Frame {
  SensorInfo sensor_info;
  double timestamp;
  std::vector<std::shared_ptr<Object>> objects;
  Eigen::Affine3d sensor2world_pose;
  // 各传感器帧补充
  LidarFrameSupplement lidar_frame_supplement;
  RadarFrameSupplement radar_frame_supplement;
  CameraFrameSupplement camera_frame_supplement;
};
```

## 子模块划分与职责

### Camera 子系统

相机感知采用流水线架构，各阶段通过 CyberRT channel 串联：

| 组件 | 类名 | 输入 channel | 输出 channel | 说明 |
|------|------|-------------|-------------|------|
| 多阶段检测 | `CameraDetectionMultiStageComponent` | `/apollo/sensor/camera/*/image` | `/perception/inner/Detection` | YOLO / YOLOx3D 2D 检测 |
| 单阶段检测 | `CameraDetectionSingleStageComponent` | `/apollo/sensor/camera/*/image` | `/perception/inner/Detection` | SMOKE / CADDN 直接 3D 检测 |
| BEV 检测 | `CameraDetectionBevComponent` | 多路相机图像 | `/perception/inner/Detection` | PETR 多视角 BEV 检测 |
| 占据网格检测 | `CameraDetectionOccComponent` | 多路相机图像 | `/perception/inner/Detection` | BEVFormer 占据网格 |
| 位置估计 | `CameraLocationEstimationComponent` | `/perception/inner/Detection` | `/perception/inner/location_estimation` | 2D→3D 位置变换（MultiCue） |
| 位置精修 | `CameraLocationRefinementComponent` | `/perception/inner/location_estimation` | `/perception/inner/location_refinement` | 利用地面约束精修 3D 位置 |
| 目标跟踪 | `CameraTrackingComponent` | `/perception/inner/location_refinement` | `/perception/inner/PrefusedObjects` | OMT 多目标跟踪 |

支持的检测模型：

- YOLO — 经典多阶段检测器
- YOLOx3D — 基于 YOLOX 的 3D 检测
- SMOKE — 单阶段单目 3D 检测
- CADDN — 基于深度分布的单目 3D 检测
- PETR — 基于 Transformer 的多视角 BEV 检测
- BEVFormer — 基于时序 BEV 的占据网格检测

### LiDAR 子系统

LiDAR 感知是一条严格的串行流水线：

| 阶段 | 组件类名 | 输入 channel | 输出 channel |
|------|---------|-------------|-------------|
| 1. 点云预处理 | `PointCloudPreprocessComponent` | `/apollo/sensor/velodyne64/compensator/PointCloud2` | `/perception/lidar/pointcloud_preprocess` |
| 2. 地图 ROI 过滤 | `PointCloudMapROIComponent` | `/perception/lidar/pointcloud_preprocess` | `/perception/lidar/pointcloud_map_based_roi` |
| 3. 地面检测 | `PointCloudGroundDetectComponent` | `/perception/lidar/pointcloud_map_based_roi` | `/perception/lidar/pointcloud_ground_detection` |
| 4. 目标检测 | `LidarDetectionComponent` | `/perception/lidar/pointcloud_ground_detection` | `/perception/lidar/detection` |
| 5. 检测过滤 | `LidarDetectionFilterComponent` | `/perception/lidar/detection` | `/perception/lidar/detection_filter` |
| 6. 目标跟踪 | `LidarTrackingComponent` | `/perception/lidar/detection_filter` | `/perception/inner/PrefusedObjects` |
| 7. 结果输出 | `LidarOutputComponent` | `/perception/inner/PrefusedObjects` | 最终输出 |

支持的检测模型：

- CenterPoint — 基于中心点的 3D 检测
- PointPillars — 基于柱状体素的 3D 检测
- CNNSeg — 基于 CNN 的点云分割检测（16/64/128 线）
- MaskPillars — 带掩码的 PointPillars 变体
- CPDet — 自定义 3D 检测器

### Radar 子系统

| 组件 | 类名 | 输入 channel | 说明 |
|------|------|-------------|------|
| 传统雷达检测 | `RadarDetectionComponent` | `/apollo/sensor/radar/front`、`/rear` | Continental 毫米波雷达，前后双雷达 |
| 4D 雷达检测 | `Radar4dDetectionComponent` | `/apollo/sensor/oculii/PointCloud2` | Oculii 4D 雷达，使用 PointPillars 模型 |

`RadarDetectionComponent` 接收 `ContiRadar` 消息，结合定位信息和高精地图进行预处理与检测，输出 `SensorFrameMessage` 送入融合模块。

### Multi-Sensor Fusion（多传感器融合）

`MultiSensorFusionComponent` 是感知模块的汇聚点，订阅 `/perception/inner/PrefusedObjects` channel，接收来自 Camera、LiDAR、Radar 各子系统的 `SensorFrameMessage`，通过概率融合算法（PBF — Probabilistic Fusion）输出最终的 `PerceptionObstacles`。

核心接口：

```cpp
class BaseFusionSystem {
  virtual bool Init(const FusionInitOptions& options) = 0;
  virtual bool Fuse(const base::FrameConstPtr& sensor_frame,
                    std::vector<base::ObjectPtr>* fused_objects) = 0;
  virtual std::string Name() const = 0;
};
```

融合输出 channel：`/perception/vehicle/obstacles`

### Traffic Light 子系统

交通信号灯感知分为四个串行阶段：

| 阶段 | 组件类名 | 输入 channel | 输出 channel |
|------|---------|-------------|-------------|
| 1. ROI 提取 | `TrafficLightsPerceptionComponent` | 定时触发 | `/perception/inner/Detection` |
| 2. 信号灯检测 | `TrafficLightDetectComponent` | `/perception/inner/Detection` | `/perception/inner/Retection` |
| 3. 颜色识别 | `TrafficLightRecognComponent` | `/perception/inner/Retection` | `/perception/inner/Tracking` |
| 4. 状态跟踪 | `TrafficLightTrackComponent` | `/perception/inner/Tracking` | 最终输出 |

### 其他子模块

| 子模块 | 组件类名 | 说明 |
|--------|---------|------|
| `lane_detection` | `LaneDetectionComponent` | 车道线检测，支持 DarkSCNN 和 DenseLine 模型 |
| `barrier_recognition` | `BarrierRecognitionComponent` | 护栏识别（直杆识别器 + 状态跟踪） |
| `motion_service` | `MotionServiceComponent` | 提供车辆运动状态服务，供其他组件查询 |
| `msg_adapter` | `MsgAdapterComponent` | 消息格式适配，桥接不同传感器消息格式 |

## 数据流概述

感知模块的整体数据流可以分为四条并行流水线，最终汇聚到多传感器融合节点：

```
                        ┌─────────────────────────────────────────────────────────┐
                        │                    Camera Pipeline                      │
  /apollo/sensor/       │  Detection ──► LocationEstimation ──► LocationRefinement│
  camera/*/image  ────► │  (YOLO/SMOKE/    (2D→3D)              (地面约束精修)     │
                        │   PETR/BEVFormer)                                       │
                        │       ──► CameraTracking (OMT) ──────────────────────┐  │
                        └──────────────────────────────────────────────────────┼──┘
                                                                               │
                        ┌──────────────────────────────────────────────────────┼──┐
                        │                    LiDAR Pipeline                    │  │
  /apollo/sensor/       │  Preprocess ──► MapROI ──► GroundDetect ──►         │  │
  velodyne64/     ────► │                            Detection ──►            │  │
  PointCloud2           │                            Filter ──► Tracking ──┐  │  │
                        └──────────────────────────────────────────────────┼──┼──┘
                                                                           │  │
                                                                           ▼  ▼
                        ┌──────────────────────────────────────────────────────┐
                        │              /perception/inner/PrefusedObjects       │
                        │                         │                            │
                        │              MultiSensorFusionComponent              │
                        │              (Probabilistic Fusion)                  │
                        │                         │                            │
                        │              /perception/vehicle/obstacles           │
                        └──────────────────────────────────────────────────────┘
                                                                           ▲  ▲
                        ┌──────────────────────────────────────────────────┼──┼──┐
  /apollo/sensor/       │                   Radar Pipeline                 │  │  │
  radar/front     ────► │  RadarDetectionComponent ────────────────────────┘  │  │
  radar/rear            │                                                     │  │
                        └─────────────────────────────────────────────────────┼──┘
                        ┌─────────────────────────────────────────────────────┼──┐
  /apollo/sensor/       │                  4D Radar Pipeline                  │  │
  oculii/PointCloud2 ─► │  Radar4dDetectionComponent ─────────────────────────┘  │
                        └────────────────────────────────────────────────────────┘


                        ┌────────────────────────────────────────────────────────┐
                        │              Traffic Light Pipeline                    │
                        │  RegionProposal ──► Detection ──► Recognition ──►     │
                        │                                    Tracking           │
                        └────────────────────────────────────────────────────────┘
```

关键 channel 汇总：

| Channel | 数据类型 | 说明 |
|---------|---------|------|
| `/apollo/sensor/camera/*/image` | `drivers::Image` | 原始相机图像 |
| `/apollo/sensor/velodyne64/compensator/PointCloud2` | `drivers::PointCloud` | 运动补偿后的点云 |
| `/apollo/sensor/radar/front` | `drivers::ContiRadar` | Continental 前雷达原始数据 |
| `/apollo/sensor/oculii/PointCloud2` | `drivers::PointCloud` | 4D 雷达点云 |
| `/perception/inner/Detection` | `CameraFrame` | 相机检测结果（内部） |
| `/perception/inner/location_estimation` | `CameraFrame` | 位置估计结果（内部） |
| `/perception/inner/location_refinement` | `CameraFrame` | 位置精修结果（内部） |
| `/perception/inner/PrefusedObjects` | `SensorFrameMessage` | 融合前各传感器结果 |
| `/perception/vehicle/obstacles` | `PerceptionObstacles` | 最终融合障碍物输出 |
| `/perception/lidar/pointcloud_preprocess` | `LidarFrameMessage` | LiDAR 预处理结果（内部） |
| `/perception/lidar/detection` | `LidarFrameMessage` | LiDAR 检测结果（内部） |
| `/perception/lidar/detection_filter` | `LidarFrameMessage` | LiDAR 过滤结果（内部） |

## DAG 配置说明

每个子模块在 `dag/` 目录下提供 `.dag` 文件，描述 CyberRT 组件的加载方式。DAG 文件使用 protobuf text 格式，结构如下：

```protobuf
module_config {
  module_library : "modules/perception/lidar_detection/liblidar_detection_component.so"
  components {
    class_name : "LidarDetectionComponent"
    config {
      name : "LidarDetection"
      config_file_path : "/apollo/modules/perception/lidar_detection/conf/lidar_detection_config.pb.txt"
      flag_file_path: "/apollo/modules/perception/data/flag/perception_common.flag"
      readers {
        channel: "/perception/lidar/pointcloud_ground_detection"
      }
    }
  }
}
```

DAG 文件核心字段：

| 字段 | 说明 |
|------|------|
| `module_library` | 组件动态库路径（`.so` 文件） |
| `class_name` | 通过 `CYBER_REGISTER_COMPONENT` 注册的组件类名 |
| `config.name` | 组件实例名称 |
| `config.config_file_path` | 组件配置文件路径（`.pb.txt`） |
| `config.flag_file_path` | gflag 文件路径 |
| `config.readers.channel` | 订阅的输入 channel |

一个 DAG 文件可以包含多个 `module_config` 块，将多个组件串联成完整流水线。例如 `lidar_output.dag` 将 LiDAR 全流水线（预处理 → ROI → 地面检测 → 检测 → 过滤 → 跟踪 → 输出）的 7 个组件定义在同一个文件中。

### Launch 文件

Launch 文件（`.launch`）是 CyberRT 的启动入口，以 XML 格式组织多个 DAG 文件：

```xml
<cyber>
    <module>
        <name>lidar</name>
        <dag_conf>/apollo/modules/perception/pointcloud_preprocess/dag/pointcloud_preprocess.dag</dag_conf>
        <dag_conf>/apollo/modules/perception/pointcloud_map_based_roi/dag/pointcloud_map_based_roi.dag</dag_conf>
        <dag_conf>/apollo/modules/perception/pointcloud_ground_detection/dag/pointcloud_ground_detection.dag</dag_conf>
        <dag_conf>/apollo/modules/perception/lidar_detection/dag/lidar_detection.dag</dag_conf>
        <dag_conf>/apollo/modules/perception/lidar_detection_filter/dag/lidar_detection_filter.dag</dag_conf>
        <dag_conf>/apollo/modules/perception/lidar_tracking/dag/lidar_tracking.dag</dag_conf>
        <process_name>lidar</process_name>
    </module>
</cyber>
```

可用的 launch 配置：

| Launch 文件 | 说明 |
|------------|------|
| `perception_all.launch` | 全量启动（Camera + LiDAR + Radar + Fusion） |
| `perception_lidar.launch` | 仅 LiDAR 流水线 |
| `perception_camera_multi_stage.launch` | 相机多阶段检测流水线 |
| `perception_camera_single_stage.launch` | 相机单阶段检测流水线 |
| `perception_radar.launch` | 仅 Radar 流水线 |
| `perception_radar4d.launch` | 仅 4D Radar 流水线 |
| `perception_trafficlight.launch` | 交通信号灯流水线 |
| `perception_lane.launch` | 车道线检测 |

## Proto 消息定义

感知模块使用大量 protobuf 消息进行配置和数据传输。按用途分类：

### 组件配置 Proto

每个子模块在 `proto/` 目录下定义自己的组件配置消息：

| Proto 文件 | 消息 | 用途 |
|-----------|------|------|
| `lidar_detection_component_config.proto` | `LidarDetectionComponentConfig` | LiDAR 检测组件配置 |
| `pointcloud_preprocess_component_config.proto` | `PointCloudPreprocessComponentConfig` | 点云预处理配置 |
| `camera_detection_multi_stage.proto` | `CameraDetectionMultiStage` | 相机多阶段检测配置 |
| `fusion_component_config.proto` | `FusionComponentConfig` | 融合组件配置 |
| `radar_component_config.proto` | `RadarComponentConfig` | 雷达组件配置 |
| `radar4d_component_config.proto` | `Radar4dComponentConfig` | 4D 雷达组件配置 |
| `lane_perception_component.proto` | `LanePerceptionComponentConfig` | 车道线检测配置 |

### 模型参数 Proto

各检测器在 `detector/*/proto/` 下定义模型参数：

| Proto 文件 | 对应模型 |
|-----------|---------|
| `camera_detection_multi_stage/detector/yolo/proto/model_param.proto` | YOLO 模型参数 |
| `camera_detection_multi_stage/detector/yolox3d/proto/model_param.proto` | YOLOx3D 模型参数 |
| `camera_detection_single_stage/detector/smoke/proto/model_param.proto` | SMOKE 模型参数 |
| `camera_detection_single_stage/detector/caddn/proto/model_param.proto` | CADDN 模型参数 |
| `camera_detection_bev/detector/petr/proto/model_param.proto` | PETR 模型参数 |
| `lidar_detection/detector/center_point_detection/proto/model_param.proto` | CenterPoint 模型参数 |
| `lidar_detection/detector/point_pillars_detection/proto/model_param.proto` | PointPillars 模型参数 |
| `lidar_detection/detector/cnn_segmentation/proto/model_param.proto` | CNNSeg 模型参数 |

### 公共 Proto

| Proto 文件 | 说明 |
|-----------|------|
| `common/proto/plugin_param.proto` | 插件参数（name, config_path, config_file） |
| `common/proto/model_info.proto` | 模型信息定义 |
| `common/proto/perception_config_schema.proto` | 通用配置 schema（KV 参数） |
| `common/proto/sensor_meta_schema.proto` | 传感器元信息 |

## 配置方式

感知模块采用三层配置体系：

### 1. 组件配置（`.pb.txt`）

每个组件在 `conf/` 目录下有对应的 protobuf text 格式配置文件，由 DAG 文件中的 `config_file_path` 指定。例如：

```
modules/perception/lidar_detection/conf/lidar_detection_config.pb.txt
modules/perception/camera_detection_multi_stage/conf/camera_detection_multi_stage_config.pb.txt
modules/perception/multi_sensor_fusion/conf/multi_sensor_fusion_config.pb.txt
```

### 2. 算法 / 模型配置（`data/`）

模型权重和算法参数存放在 `data/` 目录下：

```
modules/perception/data/models/
├── center_point_paddle/       # CenterPoint 模型
├── point_pillars_torch/       # PointPillars 模型
├── cnnseg64_caffe/            # CNNSeg 64线模型
├── smoke_torch/               # SMOKE 模型
├── yolox3d_onnx/              # YOLOx3D 模型
├── tl_detection_caffe/        # 信号灯检测模型
└── ...
```

各检测器在 `data/` 子目录下也有自己的 `.pb.txt` 配置，如：

```
camera_detection_multi_stage/data/yolo.pb.txt
camera_detection_multi_stage/data/yolox3d.pb.txt
camera_detection_occupancy/data/occ_det_nus.pb.txt
```

### 3. 全局 Flag 文件

通过 gflag 机制提供全局参数：

```
modules/perception/data/flag/perception_common.flag
```

几乎所有组件的 DAG 配置中都引用了此文件，用于设置日志级别、传感器类型、模型路径等全局参数。

### 4. 插件配置（`PluginParam`）

融合等模块通过 `PluginParam` proto 实现算法的可插拔配置：

```protobuf
message PluginParam {
  optional string name = 1;         // 插件名称（用于工厂查找）
  optional string config_path = 2;  // 配置目录
  optional string config_file = 3;  // 配置文件名
}
```

### 子模块标准目录结构

每个子模块遵循统一的目录约定：

```
<submodule>/
├── <submodule>_component.h/.cc    # CyberRT 组件入口
├── conf/                          # 组件配置（.pb.txt）
├── dag/                           # DAG 文件
├── launch/                        # Launch 文件（部分子模块）
├── proto/                         # Proto 定义
├── data/                          # 算法配置 / 模型参数
├── interface/                     # 基类 / 接口定义
├── detector/ | tracker/ | ...     # 算法实现
├── cyberfile.xml                  # 包管理描述
├── BUILD                          # Bazel 构建文件
└── README.md                      # 说明文档
```

