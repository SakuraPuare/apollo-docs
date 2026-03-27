---
title: Apollo 感知模块核心算法详解
description: 深入分析 Apollo 自动驾驶感知模块的检测、跟踪、融合算法及其工程实现
---

# Apollo 感知模块核心算法详解

## 算法概述

Apollo 感知模块（`modules/perception/`）是自动驾驶系统的"眼睛"，负责从多种传感器数据中检测、跟踪和识别周围环境中的目标。整体架构采用模块化插件设计，按传感器类型和功能阶段拆分为独立组件，通过 Cyber RT 消息机制串联成完整的感知流水线。

核心能力包括：

| 能力 | 传感器 | 关键算法 |
|------|--------|----------|
| 目标检测 | Camera | PETR、BEVFormer、YOLO/YOLOx3D、SMOKE、CADDN |
| 目标检测 | LiDAR | CenterPoint、PointPillars、MaskPillars、CNN Segmentation |
| 目标检测 | Radar | ContiARS（传统雷达）、PointPillars（4D 雷达） |
| 目标跟踪 | Camera | OMT（Online Multi-Target）、自适应卡尔曼滤波 |
| 目标跟踪 | LiDAR | MLF（Multi-LiDAR Fusion）引擎 |
| 目标跟踪 | Radar | 自适应卡尔曼滤波 + 匈牙利匹配 |
| 多传感器融合 | 全部 | 概率融合（PBF）+ D-S 证据理论 |
| 车道线检测 | Camera | DarkSCNN、DenseLine |
| 交通灯感知 | Camera | YOLOX 检测 + EfficientNet 识别 + 语义跟踪 |

## 检测算法详解

### Camera 检测

Apollo 提供了多种相机检测方案，覆盖从单目 2D 到多相机 BEV 3D 检测的完整技术栈。

#### BEV 检测方案

**PETR（Position Embedding Transformation）**

- 源码位置：`camera_detection_bev/detector/petr/`
- 核心类：`BEVObstacleDetector`
- 输入：多相机图像（支持 6 路 nuScenes 相机布局）
- 处理流程：
  1. 图像预处理：缩放、归一化（`ImagePreprocess`）
  2. 外参预处理：计算 img2lidar 变换矩阵（`ImageExtrinsicPreprocess`）
  3. 模型推理：通过 TensorRT/Paddle 推理引擎执行 PETR 网络
  4. 后处理：解析 bbox、label、score 输出 blob，按置信度阈值过滤
  5. 坐标转换：nuScenes 坐标系到 Apollo 坐标系（绕 Z 轴逆时针旋转 90°）

```cpp
// 坐标系转换核心逻辑
bool BEVObstacleDetector::Nuscenes2Apollo(std::vector<base::ObjectPtr> *objects) {
  for (auto &obj : *objects) {
    obj->theta -= M_PI / 2;
    Eigen::AngleAxisd rotation_vector(-M_PI / 2, Eigen::Vector3d(0, 0, 1));
    obj->center = rotation_vector.matrix() * obj->center;
  }
}
```

**BEVFormer（Occupancy 检测）**

- 源码位置：`camera_detection_occupancy/detector/bevformer/`
- 核心类：`BEVFORMERObstacleDetector`
- 特点：同时输出 3D 目标检测和占用栅格（Occupancy Grid）
- 关键参数：
  - 体素大小：0.2m，检测范围 ±20m（x/y/z）
  - 占用阈值：0.25
  - 支持 CAN Bus 信息融合（`FillCanBus`）
  - 支持时序 BEV 特征（`use_prev_bev_`）
  - GPU 加速预处理（`ImagePreprocessGPU`）

#### 多阶段检测方案

**YOLO 检测器**

- 源码位置：`camera_detection_multi_stage/detector/yolo/`
- 核心类：`YoloObstacleDetector`
- 特点：经典 anchor-based 2D 检测
- 支持功能：3D 框估计（`with_box3d_`）、前后比例（`with_frbox_`）、车灯状态（`with_lights_`）、区域 ID（`with_area_id_`）
- 后处理：CUDA 加速 NMS，使用 `overlapped_` 和 `idx_sm_` blob 进行 GPU 端 NMS

**YOLOx3D 检测器**

- 源码位置：`camera_detection_multi_stage/detector/yolox3d/`
- 核心类：`Yolox3DObstacleDetector`
- 特点：anchor-free 设计，支持 2D 检测 + 独立 3D 网络推理
- 推理流程：先执行 2D 检测，再通过 `Init3DNetwork` 初始化的独立 3D 网络进行深度估计

#### 单阶段检测方案

**SMOKE（Single-stage Monocular 3D Object Detection）**

- 源码位置：`camera_detection_single_stage/detector/smoke/`
- 核心类：`SmokeObstacleDetector`
- 特点：单阶段单目 3D 检测，直接从图像回归 3D 框参数
- 输出：中心点、深度、尺寸、朝向角

**CADDN（Categorical Depth Distribution Network）**

- 源码位置：`camera_detection_single_stage/detector/caddn/`
- 核心类：`CaddnObstacleDetector`
- 特点：利用深度分布预测将 2D 特征提升到 3D 空间
- 内置 lidar-to-camera 标定矩阵用于深度监督

### LiDAR 检测

#### CenterPoint

- 源码位置：`lidar_detection/detector/center_point_detection/`
- 核心类：`CenterPointDetection`
- 推理框架：Paddle Inference
- 算法原理：anchor-free 的中心点检测方法，将 3D 目标检测建模为关键点检测问题
- 处理流程：
  1. 点云下采样（可配置 `BaseDownSample` 插件）
  2. 多帧点云融合（时序融合）
  3. 点云体素化 → Pillar 特征编码
  4. Backbone + Neck 特征提取
  5. 多头输出：中心点热力图、偏移量、深度、尺寸、朝向、速度
  6. 后处理：NMS（支持类间 NMS 和策略化 NMS）
- 分类阈值独立配置：

```
cone_score_threshold_ = 0.40
ped_score_threshold_ = 0.40
cyc_score_threshold_ = 0.40
small_mot_score_threshold_ = 0.40
big_mot_score_threshold_ = 0.40
```

#### PointPillars

- 源码位置：`lidar_detection/detector/point_pillars_detection/`
- 核心类：`PointPillars`
- 推理框架：TensorRT / LibTorch
- 算法原理：将点云组织为垂直柱体（Pillar），通过 PointNet 编码后在伪图像上进行 2D 检测
- CUDA 加速模块：
  - `PreprocessPointsCuda`：点云预处理与 Pillar 生成
  - `PfeCuda`：Pillar Feature Encoding
  - `ScatterCuda`：Pillar 特征散射到 BEV 伪图像
  - `PostprocessCuda`：后处理与 NMS
  - `AnchorMaskCuda`：Anchor 掩码生成
- 关键参数（`Params` 类）：
  - Pillar 尺寸：0.32m x 0.32m x 6.0m
  - 检测范围：[-74.88, 74.88]m (x/y), [-2.0, 4.0]m (z)
  - 网格大小：468 x 468
  - 每 Pillar 最多 20 个点，最多 32000 个 Pillar
  - 每网格 6 个 Anchor（3 类 x 2 方向）
  - 点特征维度：5（x, y, z, intensity, delta_time）
  - 可配置 score_threshold、nms_overlap_threshold、reproduce_result_mode

#### MaskPillars

- 源码位置：`lidar_detection/detector/mask_pillars_detection/`
- 核心类：`MaskPillarsDetection`
- 特点：PointPillars 的改进版本，复用 `PointPillars` 推理核心
- 增强功能：多帧点云融合（`prev_world_clouds_`）、可配置下采样

#### CPDet（CenterPoint Detection 变体）

- 源码位置：`lidar_cpdet_detection/detector/cpdet/`
- 核心类：`CPDetection`
- 特点：CenterPoint 的增强实现，支持更丰富的后处理策略
- 增强功能：
  - 语义地面移除（`remove_semantic_ground_`）
  - 点唯一性过滤（`point_unique_`）
  - 类间 NMS（`inter_class_nms_`）
  - 语义类型过滤（`filter_by_semantic_type_`）
  - 多任务头输出（`num_classes_in_task_`）
  - GPU 加速目标提取和点分配

#### CNN Segmentation

- 源码位置：`lidar_detection/detector/cnn_segmentation/`
- 核心类：`CNNSegmentation`
- 算法原理：将点云投影到 BEV 特征图，通过 CNN 进行逐像素语义分割
- 处理流程：
  1. 点云到网格映射（`MapPointToGrid`）
  2. 特征生成（`FeatureGenerator`）：支持 GPU/CPU 双路径，生成 8 通道 BEV 特征图
     - `max_height_data_`：最大高度
     - `mean_height_data_`：平均高度
     - `count_data_`：点计数（log 归一化）
     - `direction_data_`：方向特征
     - `top_intensity_data_`：顶部强度
     - `mean_intensity_data_`：平均强度
     - `distance_data_`：距离特征
     - `nonempty_data_`：非空标记
  3. CNN 推理：输出 instance、category、confidence、classify、heading、height
  4. SPP 引擎聚类（`SppEngine`）：连通域分析 + 目标提取
- SPP 引擎子模块：
  - `SppCCDetector`：2D 连通域分割
  - `SppLabelImage`：标签图像处理
  - `SppClusterList`：聚类列表管理

### Radar 检测

#### 传统毫米波雷达（ContiARS）

- 源码位置：`radar_detection/lib/detector/conti_ars_detector/`
- 核心类：`ContiArsDetector`
- 输入：Continental ARS 系列雷达原始数据（`drivers::ContiRadar`）
- 处理流程：
  1. 预处理（`ContiArsPreprocessor`）：数据校验、坐标转换
  2. 检测（`RawObs2Frame`）：将雷达原始观测转换为标准检测帧
  3. ROI 过滤（`HdmapRadarRoiFilter`）：基于高精地图的感兴趣区域过滤
  4. 跟踪（`ContiArsTracker`）：自适应卡尔曼滤波 + 匈牙利匹配

#### 4D 毫米波雷达

- 源码位置：`radar4d_detection/`
- 核心类：`Radar4dDetection`
- 算法：PointPillars（与 LiDAR 检测共享算法框架）
- 特点：4D 雷达提供高度信息，可使用点云检测方法
- 处理流程：
  1. 预处理（`RadarPreprocessor`）
  2. 多帧点云融合（`FuseCloud`）
  3. PointPillars 推理
  4. 目标构建（`ObjectBuilder`）
  5. 速度计算（`CalObjectVelocity`）
  6. 分类融合（`FusedClassifier` + `CcrfTypeFusion`）
  7. 多雷达融合跟踪（`MrfEngine`）

### 交通灯检测与识别

交通灯感知采用四阶段流水线设计：

```
Region Proposal → Detection → Recognition → Tracking
```

#### 区域提议（Region Proposal）

- 源码位置：`traffic_light_region_proposal/`
- 核心类：`TLPreprocessor`、`MultiCameraProjection`
- 功能：利用高精地图中交通灯的 3D 位置，投影到图像平面生成 ROI 候选区域
- 支持多相机选择，自动选取最佳视角

#### 检测（Detection）

- 源码位置：`traffic_light_detection/`
- 检测器：
  - `TrafficLightTLDetectorYolox`：基于 YOLOX 的交通灯检测（主力方案）
  - `Detection`（Caffe）：传统 Caffe 模型检测（兼容方案）
- YOLOX 检测器特点：
  - 支持三种灯形分类：竖向灯、方形灯、横向灯（`tl_shape_classes_ = 3`）
  - Pad-Resize 预处理保持宽高比
  - 多 ROI 批量推理（`max_batch_roi_ = 3000`）
  - 多级 NMS：标准 NMS + 重叠 NMS

#### 识别（Recognition）

- 源码位置：`traffic_light_recognition/`
- 识别器：
  - `EfficientNetRecognition`：基于 EfficientNet 的颜色/状态识别
  - `TrafficLightRecognition`（Caffe）：传统分类器
- EfficientNet 识别器：
  - 双头输出：颜色分类（`outputs_cls_`）+ 状态分类（`outputs_status_`）
  - Zero-Padding 预处理保持灯框宽高比
  - 概率到颜色映射（`Prob2Color`）

#### 跟踪（Tracking）

- 源码位置：`traffic_light_tracking/`
- 核心类：`SemanticReviser`
- 跟踪策略：
  - 语义修正（`ReviseBySemantic`）：基于交通灯语义规则修正颜色判断
  - 时序修正（`ReviseByTimeSeries`）：利用时间序列平滑闪烁检测
  - 滞后窗口（`HystereticWindow`）：防止颜色状态频繁跳变
  - 历史语义表（`SemanticTable`）：维护每个交通灯的历史状态

### 车道线检测

#### DarkSCNN

- 源码位置：`lane_detection/lib/detector/darkSCNN/`
- 核心类：`DarkSCNNLaneDetector`
- 推理框架：TensorRT（NVIDIA）/ MIGraphX（AMD）
- 特点：基于空间 CNN 的语义分割方法，逐像素预测车道线
- 后处理：`DarkSCNNLanePostprocessor` 将分割结果转换为车道线实例

#### DenseLine

- 源码位置：`lane_detection/lib/detector/denseline/`
- 核心类：`DenselineLaneDetector`
- 特点：密集车道线检测，输出更精细的车道线表示
- 后处理：`DenselineLanePostprocessor`

## 跟踪算法详解

### Camera 跟踪

#### OMT（Online Multi-Target Tracker）

- 源码位置：`camera_tracking/tracking/`
- 核心类：`OMTObstacleTracker`
- 算法框架：基于假设的多目标跟踪

跟踪流程：

1. **特征提取**（`FeatureExtract`）：通过 `TrackingFeatExtractor` 或 `ExternalFeatureExtractor` 提取目标外观特征
2. **预测**（`Predict`）：基于运动模型预测目标在新帧中的位置
3. **2D 关联**（`Associate2D`）：利用多维相似度进行目标关联
4. **3D 关联**（`Associate3D`）：结合 3D 信息进一步优化关联

相似度计算采用多维度融合：

- `ScoreAppearance`：外观特征相似度（深度学习特征）
- `ScoreMotion`：运动一致性评分
- `ScoreShape`：形状相似度
- `ScoreOverlap`：IoU 重叠度

关联策略：
- 生成假设（`GenerateHypothesis`）：为每个目标-检测对生成假设
- 假设评分排序后贪心分配
- 未匹配检测创建新目标（`CreateNewTarget`）
- 重复目标合并（`CombineDuplicateTargets`）

#### Camera 卡尔曼滤波

源码位置：`camera_tracking/common/kalman_filter.h`

提供多种滤波器实现：

| 滤波器 | 状态向量 | 适用场景 |
|--------|----------|----------|
| `KalmanFilterConstVelocity` | (x, y, vx, vy) | 匀速运动目标 |
| `KalmanFilterConstState` | N 维可配置 | 静态/缓变属性 |
| `ExtendedKalmanFilter` | (x, y, v, θ) | 非线性运动目标 |
| `FirstOrderRCLowPassFilter` | 任意维度 | 平滑噪声 |
| `MeanFilter` / `MaxNMeanFilter` | 任意维度 | 统计平滑 |

#### BEV Occupancy 跟踪

- 源码位置：`camera_detection_occupancy/tracker/`
- 核心类：`CameraTracker`
- 滤波器：`AdaptiveKalmanFilter`（4 维状态：位置 + 速度）
- 匹配器：`HmMatcher`（匈牙利匹配）
- 轨迹管理：`CameraTrackManager`

### LiDAR 跟踪

#### MLF（Multi-LiDAR Fusion）引擎

- 源码位置：`lidar_tracking/tracker/multi_lidar_fusion/`
- 核心类：`MlfEngine`

MLF 是 Apollo LiDAR 跟踪的核心引擎，处理流程：

```
输入帧 → 前景/背景分离 → 目标匹配 → 状态滤波 → 结果收集
```

1. **前景/背景分离**（`SplitAndTransformToTrackedObjects`）：将检测目标分为前景（动态目标）和背景（静态目标）
2. **目标匹配**（`TrackObjectMatchAndAssign`）：使用 `MlfTrackObjectMatcher` 进行轨迹-检测关联
3. **状态滤波**（`TrackStateFilter`）：通过 `MlfTracker` 更新轨迹状态
4. **过期清理**（`RemoveStaleTrackData`）：移除超时轨迹（`reserved_invisible_time_ = 0.3s`）

#### MLF 运动滤波器

- 核心类：`MlfMotionFilter`
- 运动模型：恒加速度模型（Constant Acceleration）
- 观测量：仅速度（部分观测卡尔曼滤波）

关键策略：
- **自适应增益调整**（`StateGainAdjustment`）：根据轨迹历史动态调整卡尔曼增益
- **收敛性估计与加速**（`ConvergenceEstimationAndBoostUp`）：评估滤波器收敛状态，对已收敛轨迹加速状态更新
- **在线协方差估计**（`OnlineCovarianceEstimation`）：基于历史测量动态估计过程噪声
- **状态裁剪**（`ClipingState`）：将噪声级别以下的状态置零

默认参数：

```
init_velocity_variance_ = 5.0
init_acceleration_variance_ = 10.0
measured_velocity_variance_ = 0.4
predict_variance_per_sqrsec_ = 50.0
noise_maximum_ = 0.1
```

#### MLF 辅助滤波器

- `MlfShapeFilter`：形状滤波，平滑目标尺寸变化
- `MlfDirectionFilter`：方向滤波，平滑朝向角
- `MlfTypeFilter`：类型滤波，基于 CCRF 的类型融合

### Radar 跟踪

#### 自适应卡尔曼滤波

- 源码位置：`radar_detection/lib/tracker/filter/adaptive_kalman_filter.h`
- 核心类：`AdaptiveKalmanFilter`
- 状态向量：4 维（x, y, vx, vy）
- 矩阵定义：
  - A：状态转移矩阵
  - C：观测矩阵
  - Q：过程噪声协方差（可通过 `s_q_matrix_ratio_` 动态调整）
  - R：观测噪声协方差
  - K：最优卡尔曼增益

#### 匈牙利匹配

- 源码位置：`radar_detection/lib/tracker/matcher/hm_matcher.h`
- 核心类：`HmMatcher`
- 匹配策略：基于距离的匈牙利二部图匹配

## 融合策略详解

### 概率融合框架（ProbabilisticFusion）

- 源码位置：`multi_sensor_fusion/fusion/fusion_system/probabilistic_fusion/`
- 核心类：`ProbabilisticFusion`

这是 Apollo 多传感器融合的顶层框架，协调所有传感器数据的融合过程。

融合流程：

```
传感器数据 → 数据关联 → 轨迹更新/创建 → 门控过滤 → 输出融合结果
```

1. **数据接收**（`Fuse`）：接收来自 LiDAR、Camera、Radar 的检测结果
2. **前景融合**（`FuseForegroundTrack`）：
   - 数据关联（`HMTrackersObjectsAssociation`）
   - 已匹配轨迹更新（`UpdateAssignedTracks`）
   - 未匹配轨迹预测（`UpdateUnassignedTracks`）
   - 新轨迹创建（`CreateNewTracks`）
3. **背景融合**（`FusebackgroundTrack`）
4. **过期清理**（`RemoveLostTrack`）
5. **门控输出**（`PbfGatekeeper`）：决定哪些轨迹可以发布

配置参数（`FusionParams`）：

```protobuf
message ProbabilisticFusionConfig {
  optional bool use_lidar = 1 [default = true];
  optional bool use_radar = 2 [default = true];
  optional bool use_camera = 3 [default = true];
  // tracker、data_association、gatekeeper 均为可插拔插件
}
```

### 数据关联（HM Data Association）

- 核心类：`HMTrackersObjectsAssociation`
- 算法：门控匈牙利匹配（`GatedHungarianMatcher`）

关联流程：
1. **ID 关联**（`IdAssign`）：优先使用传感器 ID 进行快速关联
2. **距离矩阵计算**（`ComputeAssociationDistanceMat`）：计算轨迹-检测距离矩阵
3. **匈牙利求解**（`MinimizeAssignment`）：最小化总关联代价
4. **后处理 ID 关联**（`PostIdAssign`）：对未匹配项进行二次关联

距离计算（`TrackObjectDistance`）综合考虑：
- 位置距离
- 速度一致性
- 形状相似度
- 卡方检验（使用预计算的卡方分布表 `chi_squared_cdf_*.h`）

### PBF Tracker

- 核心类：`PbfTracker`
- 功能：管理单个融合轨迹的多维状态更新

PBF Tracker 内部集成四个子融合模块：

| 子模块 | 实现类 | 功能 |
|--------|--------|------|
| 类型融合 | `DstTypeFusion` | D-S 证据理论融合目标类型 |
| 运动融合 | `KalmanMotionFusion` | 卡尔曼滤波融合位置和速度 |
| 形状融合 | `PbfShapeFusion` | 融合目标尺寸和形状 |
| 存在性融合 | `DstExistenceFusion` | D-S 证据理论判断目标是否真实存在 |

### D-S 证据理论（Dempster-Shafer Theory）

- 源码位置：`multi_sensor_fusion/common/dst_evidence.h`
- 核心类：`Dst`、`DstManager`

D-S 证据理论用于处理不确定性推理，在 Apollo 中用于类型融合和存在性融合。

核心概念：
- **辨识框架（FOD）**：所有可能假设的集合
- **基本概率分配（BBA）**：每个假设子集的置信度
- **支持度（Support）**：假设的下界概率
- **似然度（Plausibility）**：假设的上界概率
- **不确定度（Uncertainty）**：似然度与支持度之差

**类型融合**（`DstTypeFusion`）的假设空间：

```cpp
enum {
  PEDESTRIAN = (1 << 0),
  BICYCLE = (1 << 1),
  VEHICLE = (1 << 2),
  OTHERS_MOVABLE = (1 << 3),
  OTHERS_UNMOVABLE = (1 << 4)
};
```

传感器可靠度配置（影响 BBA 分配权重）：

| 传感器 | 可靠度 | Unknown 可靠度 |
|--------|--------|----------------|
| velodyne64 | 0.5 | 0.5 |
| camera_front | 0.95 | 0.2 |
| camera_rear | 0.95 | 0.2 |

**存在性融合**（`DstExistenceFusion`）：
- TOIC（Target of Interest in Camera）评分：判断 LiDAR 目标是否在相机视野内被确认
- 存在性概率：综合多传感器证据判断目标是否真实存在
- 距离衰减（`ComputeDistDecay`）：远距离目标的存在性证据权重降低

### 卡尔曼运动融合

- 核心类：`KalmanMotionFusion`
- 状态向量：6 维（位置 x/y/z + 速度 vx/vy/vz）

关键机制：
- **伪测量计算**：根据传感器类型生成伪测量值
  - `ComputePseudoLidarMeasurement`：LiDAR 伪测量
  - `ComputePseudoCameraMeasurement`：Camera 伪测量
  - `ComputePseudoRadarMeasurement`：Radar 伪测量
- **R 矩阵自适应**（`RewardRMatrix`）：根据传感器类型和收敛状态动态调整观测噪声
- **加速度估计**（`ComputeAccelerationMeasurement`）：利用历史传感器信息计算加速度
- **历史管理**：维护速度、时间戳、传感器类型的历史队列

### 门控策略（PBF Gatekeeper）

- 核心类：`PbfGatekeeper`
- 功能：决定融合轨迹是否满足发布条件

门控条件（可配置）：

```protobuf
message PbfGatekeeperConfig {
  optional bool publish_if_has_lidar = 1 [default = true];
  optional bool publish_if_has_radar = 2 [default = true];
  optional bool publish_if_has_camera = 3 [default = true];
  optional double min_radar_confident_distance = 5;  // 默认 40m
  optional double max_radar_confident_angle = 6;     // 默认 20°
  optional double invisible_period_threshold = 8;    // 默认 0.001s
  optional double toic_threshold = 9;                // 默认 0.8
  optional double existence_threshold = 12;          // 默认 0.7
  optional double radar_existence_threshold = 13;    // 默认 0.9
  optional double min_camera_publish_distance = 14;  // 默认 50m
}
```

## Proto 消息定义

Apollo 感知模块使用 Protocol Buffers 定义配置和数据结构。以下是按功能分类的关键 Proto 文件：

### 检测模型配置

| Proto 文件 | 用途 |
|------------|------|
| `camera_detection_bev/detector/petr/proto/model_param.proto` | PETR BEV 检测模型参数 |
| `camera_detection_occupancy/detector/bevformer/proto/model_param.proto` | BEVFormer 模型参数 |
| `camera_detection_multi_stage/detector/yolo/proto/model_param.proto` | YOLO 检测模型参数 |
| `camera_detection_multi_stage/detector/yolox3d/proto/model_param.proto` | YOLOx3D 模型参数 |
| `camera_detection_single_stage/detector/smoke/proto/model_param.proto` | SMOKE 模型参数 |
| `camera_detection_single_stage/detector/caddn/proto/model_param.proto` | CADDN 模型参数 |
| `lidar_detection/detector/center_point_detection/proto/model_param.proto` | CenterPoint 模型参数 |
| `lidar_detection/detector/point_pillars_detection/proto/model_param.proto` | PointPillars 模型参数 |
| `radar4d_detection/lib/detector/point_pillars_detection/proto/model_param.proto` | 4D 雷达 PointPillars 参数 |

### 组件配置

| Proto 文件 | 用途 |
|------------|------|
| `camera_detection_bev/proto/camera_detection_bev.proto` | BEV 检测组件配置 |
| `camera_detection_occupancy/proto/camera_detection_occupancy.proto` | Occupancy 检测组件配置 |
| `camera_tracking/proto/camera_tracking_component.proto` | Camera 跟踪组件配置 |
| `camera_tracking/proto/omt.proto` | OMT 跟踪器参数 |
| `lidar_tracking/proto/lidar_tracking_component_config.proto` | LiDAR 跟踪组件配置 |
| `radar_detection/proto/radar_component_config.proto` | Radar 检测组件配置 |
| `radar4d_detection/proto/radar4d_component_config.proto` | 4D Radar 组件配置 |

### 融合配置

| Proto 文件 | 用途 |
|------------|------|
| `multi_sensor_fusion/proto/fusion_component_config.proto` | 融合组件配置 |
| `multi_sensor_fusion/proto/probabilistic_fusion_config.proto` | 概率融合参数 |
| `multi_sensor_fusion/proto/pbf_tracker_config.proto` | PBF Tracker 配置 |
| `multi_sensor_fusion/proto/pbf_gatekeeper_config.proto` | 门控策略配置 |
| `multi_sensor_fusion/proto/dst_type_fusion_config.proto` | D-S 类型融合配置 |
| `multi_sensor_fusion/proto/dst_existence_fusion_config.proto` | D-S 存在性融合配置 |

### 交通灯与车道线

| Proto 文件 | 用途 |
|------------|------|
| `traffic_light_detection/proto/traffic_light_detection_component.proto` | 交通灯检测组件 |
| `traffic_light_recognition/proto/traffic_light_recognition_component.proto` | 交通灯识别组件 |
| `traffic_light_tracking/proto/traffic_light_tracking_component.proto` | 交通灯跟踪组件 |
| `traffic_light_tracking/tracker/proto/semantic.proto` | 语义修正配置 |
| `lane_detection/proto/darkSCNN.proto` | DarkSCNN 车道线检测参数 |
| `lane_detection/proto/denseline.proto` | DenseLine 车道线检测参数 |

### 通用配置

| Proto 文件 | 用途 |
|------------|------|
| `common/proto/model_info.proto` | 模型信息定义 |
| `common/proto/model_process.proto` | 模型处理配置 |
| `common/proto/plugin_param.proto` | 插件参数定义 |
| `common/proto/sensor_meta_schema.proto` | 传感器元数据 |
| `common/proto/rt.proto` | 推理运行时配置 |

### 核心输出消息（`common_msgs/perception_msgs/`）

**`PerceptionObstacle`**（`perception_obstacle.proto`）-- 感知模块的主要输出：

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | int32 | 目标唯一 ID |
| `position` | Point3D | 世界坐标系下的位置 |
| `theta` | double | 航向角 |
| `velocity` | Point3D | 速度向量 |
| `length` / `width` / `height` | double | 包围盒尺寸 |
| `polygon_point` | Point3D[] | 凸包角点 |
| `tracking_time` | double | 跟踪持续时间 |
| `type` | Type enum | UNKNOWN / VEHICLE / PEDESTRIAN / BICYCLE / UNKNOWN_MOVABLE / UNKNOWN_UNMOVABLE |
| `sub_type` | SubType enum | CAR / TRUCK / BUS / CYCLIST / MOTORCYCLIST / TRICYCLIST / PEDESTRIAN / TRAFFICCONE 等 |
| `confidence` | double | 置信度 |
| `confidence_type` | ConfidenceType | 置信度类型 |
| `acceleration` | Point3D | 加速度 |
| `anchor_point` | Point3D | 锚点 |
| `bbox2d` | BBox2D | 2D 检测框（像素坐标） |
| `measurements` | SensorMeasurement[] | 各传感器的原始测量值 |

**`TrafficLight`**（`traffic_light_detection.proto`）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `color` | Color enum | UNKNOWN / RED / YELLOW / GREEN / BLACK |
| `id` | string | 地图中的信号灯 ID |
| `confidence` | double | 识别置信度 |
| `tracking_time` | double | 跟踪持续时间 |
| `blink` | bool | 是否闪烁 |
| `remaining_time` | double | V2X 剩余时间 |

## 模型配置与部署

### 推理框架支持

Apollo 感知模块支持多种推理后端，通过统一的 `inference::Inference` 接口抽象：

| 推理框架 | 适用平台 | 使用模块 |
|----------|----------|----------|
| TensorRT | NVIDIA GPU | PointPillars、YOLO、DarkSCNN、PETR |
| Paddle Inference | NVIDIA/通用 | CenterPoint、CPDet |
| LibTorch | 通用 | PointPillars（备选）、YOLOx3D |
| MIGraphX | AMD GPU | DarkSCNN、DenseLine（AMD 适配） |

### 模型加载流程

所有检测器遵循统一的初始化模式：

```cpp
bool Detector::Init(const DetectorInitOptions &options) {
  // 1. 读取 Proto 配置文件
  GetProtoFromFile(config_file, &model_param_);
  // 2. 初始化图像/点云尺寸参数
  InitImageSize(model_param_);
  // 3. 初始化目标类型映射
  InitTypes(model_param_);
  // 4. 创建推理引擎（通过 InferenceFactory）
  net_ = inference::InferenceFactory::CreateInferenceByName(...);
  // 5. 初始化输入/输出 Blob
  net_->Init(...);
}
```

### GPU 加速

- CUDA Stream 管理：每个检测器维护独立的 `cudaStream_t`，支持异步推理
- GPU 预处理：BEVFormer 支持 `ImagePreprocessGPU` 在 GPU 端完成图像归一化
- CUDA 核函数：PointPillars 的预处理、特征编码、后处理均有 CUDA 实现
- CPDet 支持 GPU 端目标提取（`use_cpu_get_objects_ = false`）和点分配（`use_cpu_assign_points_ = false`）

### 配置文件组织

每个检测模块的配置遵循统一目录结构：

```
module_name/
├── conf/          # 组件配置（Cyber RT 组件参数）
├── dag/           # DAG 调度图
├── data/          # 模型参数配置（Proto 文本格式）
├── detector/      # 检测器实现
│   └── xxx/
│       └── proto/ # 模型参数 Proto 定义
├── interface/     # 基类接口定义
├── launch/        # 启动文件
└── proto/         # 组件级 Proto 定义
```

## 算法流水线（Pipeline）

### 完整感知流水线

Apollo 感知系统的完整数据流如下：

```
┌─────────────────────────────────────────────────────────────┐
│                     传感器数据输入                             │
├──────────┬──────────┬──────────┬────────────┬───────────────┤
│  Camera  │  LiDAR   │  Radar   │  4D Radar  │  HD Map       │
└────┬─────┴────┬─────┴────┬─────┴─────┬──────┴───────┬───────┘
     │          │          │           │              │
     ▼          ▼          ▼           ▼              │
┌─────────┐┌────────┐┌─────────┐┌──────────┐         │
│ Camera   ││ Point  ││ Radar   ││ Radar4D  │         │
│Detection ││ Cloud  ││Detection││Detection │         │
│(BEV/YOLO ││Preproc ││(ContiARS││(Point    │         │
│/SMOKE/   ││+ Ground││)        ││Pillars)  │         │
│CADDN)    ││+ ROI   ││         ││          │         │
└────┬─────┘└───┬────┘└────┬────┘└────┬─────┘         │
     │          │          │          │               │
     │          ▼          │          │               │
     │   ┌───────────┐    │          │               │
     │   │  LiDAR    │    │          │               │
     │   │ Detection │    │          │               │
     │   │(CenterPt/ │    │          │               │
     │   │PointPillar│    │          │               │
     │   │/CNN Seg)  │    │          │               │
     │   └─────┬─────┘    │          │               │
     │         │           │          │               │
     ▼         ▼           ▼          ▼               │
┌─────────┐┌────────┐┌─────────┐┌──────────┐         │
│ Camera  ││ LiDAR  ││ Radar   ││ Radar4D  │         │
│Tracking ││Tracking││Tracking ││ Tracking │         │
│ (OMT)   ││ (MLF)  ││(AKF+HM) ││(MRF)    │         │
└────┬─────┘└───┬────┘└────┬────┘└────┬─────┘         │
     │          │          │          │               │
     └──────────┴──────────┴──────────┘               │
                      │                               │
                      ▼                               │
              ┌───────────────┐                       │
              │ Multi-Sensor  │◄──────────────────────┘
              │   Fusion      │
              │(Probabilistic │
              │  Fusion)      │
              └───────┬───────┘
                      │
                      ▼
              ┌───────────────┐
              │  Perception   │
              │   Output      │
              │ (Obstacles)   │
              └───────────────┘
```

### LiDAR 处理子流水线

```
原始点云 → 预处理(pointcloud_preprocess)
         → 地面检测(pointcloud_ground_detection)
         → ROI 过滤(pointcloud_map_based_roi)
         → 语义分割(pointcloud_semantics) [可选]
         → 运动估计(pointcloud_motion) [可选]
         → 目标检测(lidar_detection / lidar_cpdet_detection)
         → 检测过滤(lidar_detection_filter)
         → 目标跟踪(lidar_tracking)
         → 输出(lidar_output)
```

### 交通灯处理子流水线

```
相机图像 + HD Map → 区域提议(traffic_light_region_proposal)
                  → 交通灯检测(traffic_light_detection)
                  → 颜色识别(traffic_light_recognition)
                  → 语义跟踪(traffic_light_tracking)
```

### 关键设计模式

1. **插件化架构**：所有算法通过 `REGISTER_*` 宏注册，运行时通过配置文件选择具体实现
2. **基类接口**：每个功能模块定义 `Base*` 抽象基类（如 `BaseLidarDetector`、`BaseObstacleDetector`）
3. **组件化部署**：每个处理阶段封装为独立的 Cyber RT Component，通过 DAG 文件编排
4. **配置驱动**：模型参数、阈值、开关均通过 Proto 配置文件管理，无需重新编译
