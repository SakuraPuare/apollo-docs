---
title: "激光雷达分割"
---

# 激光雷达分割 (Lidar Segmentation)

> 源码路径: `modules/perception/lidar_segmentation/`

## 概述

激光雷达分割模块负责将激光雷达点云数据分割为独立的物体实例，并为每个物体赋予语义标签和运动状态。模块通过 Cyber RT 组件框架运行，接收 `LidarFrameMessage` 输入，经过分割检测、语义构建两阶段处理后输出带有类别和运动信息的物体集合。

模块包含一个核心 Cyber 组件和三个公共工具类/函数，以及两种分割算法实现（`graph_segmentation` 和 `ncut_segmentation`）。

## 核心类

### LidarSegmentationComponent

Cyber RT 组件，是模块的入口，继承自 `cyber::Component<LidarFrameMessage>`。

```cpp
class LidarSegmentationComponent : public cyber::Component<onboard::LidarFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<onboard::LidarFrameMessage>& message) override;

 private:
  bool InternalProc(const std::shared_ptr<onboard::LidarFrameMessage>& message);

  std::string output_channel_name_;
  std::string segmentor_name_;
  std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;
  std::shared_ptr<BaseLidarDetector> segmentor_;
  SemanticBuilder semantic_builder_;
};
```

- `Init()`: 从 protobuf 配置加载参数，创建输出 channel writer，通过插件机制实例化分割器（`BaseLidarDetector`），初始化 `SemanticBuilder`。
- `Proc()`: 调用 `InternalProc()` 完成分割和语义构建，成功后将结果通过 writer 发送到下游。

### SemanticBuilder

为分割后的物体计算语义类型（`ObjectSemanticType`）和运动状态（`MotionState`）。

```cpp
class SemanticBuilder {
 public:
  bool Init(const SemanticBuilderInitOptions& options = SemanticBuilderInitOptions());
  bool Build(const SemanticBuilderOptions& options, LidarFrame* frame);

 private:
  void GetObjSemanticAndMotionType(ObjectPtr object);
};
```

- `Build()`: 遍历帧中所有分割物体，对每个物体调用 `GetObjSemanticAndMotionType()`。
- `GetObjSemanticAndMotionType()`: 统计点云中各语义标签和运动标签的出现频次，取众数作为物体的语义类型和运动状态。

## 核心函数

### BgObjectBuilder

构建背景物体的几何属性（2D 多边形、方向、尺寸、中心、高度等）。

```cpp
bool BgObjectBuilder(std::vector<base::ObjectPtr>* objects,
                     Eigen::Affine3d& lidar2novatel_pose);
```

对每个物体依次调用 `ObjectBuilder` 的 `ComputePolygon2D`、`ComputePolygonDirection`、`ComputePolygonSizeCenter`、`ComputeOtherObjectInformation`、`ComputeHeightAboveGround`。当 `FLAGS_need_judge_front_critical` 为 true 时，还会判断是否为前方关键物体。

### SplitObject

沿物体主方向将过大的物体拆分为多个子物体。

```cpp
void SplitObject(const base::ObjectPtr& obj,
                 std::vector<base::ObjectPtr>* split_objs,
                 float split_distance);
```

算法流程：若物体最大边长小于 `split_distance` 则不拆分；否则通过 `LineFit2D` 拟合主方向，将点云投影到主方向轴上，按 `split_distance` 分段，每段生成一个子物体（至少 4 个点）。

### LineFit2D

基于 PCA 的 2D 线拟合，返回直线参数 `[nx, ny, d]`（法向量 + 偏移）。

```cpp
bool LineFit2D(const base::PointFCloud& cloud, Eigen::Vector3f* params);
```

计算点云的 2x2 协方差矩阵，取最小特征值对应的特征向量作为直线法向量。

## 配置

配置通过 protobuf 定义，字段如下：

| 字段 | 类型 | 说明 |
|------|------|------|
| `output_channel_name` | `string` | 输出 Channel 名称 |
| `segmentor` | `string` | 分割算法名称 |
| `plugin_param` | `PluginParam` | 插件参数（name、config_path、config_file） |

分割算法可选实现：

- `graph_segmentation`: 基于图的分割算法，配置文件 `data/graph_segmentation.pb.txt`
- `ncut_segmentation`: 归一化切割分割算法，配置文件 `data/ncut_param.pb.txt`

## 调用关系

```text
LidarSegmentationComponent::Proc()
  |
  +-- InternalProc()
  |     |
  |     +-- BaseLidarDetector::Detect()       // 点云分割检测
  |     |
  |     +-- SemanticBuilder::Build()           // 语义与运动状态构建
  |           |
  |           +-- GetObjSemanticAndMotionType() // 逐物体语义投票
  |
  +-- Writer::Write()                          // 输出到下游 Channel
```

公共工具函数的典型调用场景：

```text
BgObjectBuilder()
  +-- ObjectBuilder::ComputePolygon2D()
  +-- ComputePolygonDirection()
  +-- ObjectBuilder::ComputePolygonSizeCenter()
  +-- ObjectBuilder::ComputeOtherObjectInformation()
  +-- ObjectBuilder::ComputeHeightAboveGround()
  +-- ObjectBuilder::JudgeFrontCritical()  // 可选

SplitObject()
  +-- LineFit2D()                           // PCA 方向拟合
  +-- ObjectPool::BatchGet()               // 批量分配子物体
```
