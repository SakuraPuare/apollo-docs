---
title: "感知插件"
---

# Perception Plugin

> 源码路径: `modules/perception/perception_plugin/`

## 概述

感知插件模块提供可热插拔的感知子功能，通过 Apollo 插件管理器 (`CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN`) 注册，按需加载。模块包含三大子系统：

- **ONNX 多批次推理** (`inference/`) -- 基于 TensorRT 的 ONNX 模型推理引擎，支持动态 batch 和 FP16 加速
- **GPU 体素降采样** (`lidar_detection/`) -- 利用 CUDA 对激光雷达点云进行体素网格降采样
- **检测目标滤波器** (`lidar_detection_filter/`) -- 四种后处理滤波器，对激光雷达检测结果进行精化

所有插件均位于 `apollo::perception` 命名空间下，遵循统一的 `Init` / `Process` 或 `Init` / `Filter` 生命周期接口。

## 核心类

### MultiBatchInference

- **继承**: `Inference`
- **路径**: `inference/onnx_multi_batch/`
- **功能**: 加载 ONNX 模型并转换为 TensorRT 引擎执行推理。支持序列化引擎缓存（`.trt.engine`），避免重复编译。通过 `SetOnnxOptimizationProfile` 配置动态维度的 min/opt/max profile。

```cpp
class MultiBatchInference : public Inference {
 public:
  bool OnnxToTRTModel(const std::string &model_file);
  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
  void Infer() override;
  base::BlobPtr<float> get_blob(const std::string &name) override;
};
```

### GpuDownSample

- **继承**: `BaseDownSample`
- **路径**: `lidar_detection/gpu_down_sample/`
- **功能**: 基于 CUDA 的体素降采样，将点云数据拷贝至 pinned memory 后调用 `VoxelDownSampleCuda` 执行 GPU 端降采样，支持质心模式和随机保留模式。

```cpp
class GpuDownSample : public BaseDownSample {
 public:
  bool Init(const DownSampleInitOptions &options) override;
  bool Process(const DownSampleOptions &options, base::PointFCloudPtr &cloud_ptr) override;
};
```

### VoxelDownSampleCuda

- **路径**: `lidar_detection/gpu_down_sample/voxel_downsample.h`
- **功能**: CUDA 核函数封装类，提供网格参数计算 (`cudaCalculateGridParams`)、网格哈希 (`cudaCalculateGrid`)、降采样 (`cudaDownSample`) 和 GPU 预热 (`cudaWarmUpGPU`) 接口。

### BigmotRefineFilter

- **继承**: `BaseObjectFilter`
- **路径**: `lidar_detection_filter/bigmot_refine_filter/`
- **功能**: 大型移动目标（车辆）精化滤波器。遍历 VEHICLE 类型目标，计算行人/骑行者扩展包围盒，剔除落入该包围盒内的车辆点云，并重新计算凸包和形状。

### MarkAreaFilter

- **继承**: `BaseObjectFilter`
- **路径**: `lidar_detection_filter/mark_area_filter/`
- **功能**: 标记区域滤波器。根据配置的多边形区域过滤障碍物，支持三种模式：过滤全部目标 (`type=0`)、仅过滤背景目标 (`type=1`)、按前景类型过滤 (`type=2`)。仅当标记区域距离车辆小于 `filter_ratio_` 时才生效。

### ObjectSplitFilter

- **继承**: `BaseObjectFilter`
- **路径**: `lidar_detection_filter/object_split_filter/`
- **功能**: 目标分裂滤波器。检测与自车包围盒交叉的目标，将其按配置间隔分裂为多个子目标，并重新设定 ID 和类型属性。

### SemanticFilter

- **继承**: `BaseObjectFilter`
- **路径**: `lidar_detection_filter/semantic_filter/`
- **功能**: 语义类型滤波器。根据语义标签过滤障碍物，包含三阶段：语义类型过滤、地面点概率过滤、前向关键目标过滤（支持 ignore/过滤/高度阈值三种策略）。

## 核心函数

### MultiBatchInference

| 函数 | 说明 |
| --- | --- |
| `OnnxToTRTModel(model_file)` | 解析 ONNX 模型并构建 TensorRT 引擎，支持 FP16 和引擎序列化缓存 |
| `SetOnnxOptimizationProfile(profile, shapes)` | 为动态输入维度设置 min/opt/max 优化 profile |
| `Init(shapes)` | 初始化 GPU 设备、CUDA 流，创建 TensorRT 引擎和执行上下文，分配输入输出 Blob |
| `Infer()` | 绑定输入输出 buffer，通过 `enqueueV2` 异步执行推理并同步结果 |

### GpuDownSample

| 函数 | 说明 |
| --- | --- |
| `Init(options)` | 从 protobuf 配置文件加载体素尺寸（x/y/z）和质心降采样开关 |
| `Process(options, cloud_ptr)` | 将点云拷贝至 pinned memory，执行 GPU 降采样后回写结果 |
| `downSample(...)` | 内部方法：计算网格参数、分配 GPU 哈希表和体素、执行 CUDA 降采样 |

### 滤波器通用接口

所有滤波器共享 `BaseObjectFilter` 接口：

| 函数 | 说明 |
| --- | --- |
| `Init(options)` | 加载对应 protobuf 配置并初始化内部状态 |
| `Filter(options, frame)` | 对 `LidarFrame` 中的 `segmented_objects` 执行过滤，原地移除不合格目标 |

### BigmotRefineFilter 特有函数

| 函数 | 说明 |
| --- | --- |
| `RefineBigMotObjects(frame)` | 对车辆目标剔除落入行人/骑行者扩展包围盒内的点，重建凸包和形状 |
| `GetExpandBBox(object, bbox, expand)` | 根据目标朝向和尺寸 + expand 生成扩展四边形包围盒 |

### MarkAreaFilter 特有函数

| 函数 | 说明 |
| --- | --- |
| `ComputeAreaDistance(frame)` | 计算各标记区域到车辆的距离，筛选小于 `filter_ratio_` 的区域 |
| `FilterAreaObstacles(frame)` | 按区域类型分派：全部过滤、仅背景、按前景类型过滤 |
| `IsPointInPolygon(frame, obj, area_polygen)` | 判断目标中心点（世界坐标）是否在多边形区域内 |

### ObjectSplitFilter 特有函数

| 函数 | 说明 |
| --- | --- |
| `IsCrossWithEgo(object)` | 判断目标多边形是否与自车包围盒交叉（角点包含 + 边线相交） |
| `CheckForegroundBackground(object)` | 根据配置决定是否对前景目标执行分裂 |
| `SetSplitObjectsValue(base_object, split_objects)` | 为分裂后的子目标设置 ID、类型和检测属性 |

### SemanticFilter 特有函数

| 函数 | 说明 |
| --- | --- |
| `FillFilterFlags(frame)` | 按语义类型标记需过滤的目标 |
| `FillFilterFlagsOfGround(frame)` | 根据地面点概率和高度差标记地面/路缘类目标 |
| `FillFilterFlagsOfFrontCritical(frame)` | 对前向关键目标按策略过滤：直接过滤、高度阈值过滤、不过滤 |

## 配置

所有插件通过 protobuf 配置文件初始化，配置文件路径由 `GetConfigFile(config_path, config_file)` 确定：

| 插件 | 配置消息类型 | 关键参数 |
| --- | --- | --- |
| `MultiBatchInference` | 通过基类 `Inference` 配置 | `model_file`, `max_batch_size`, `gpu_id` |
| `GpuDownSample` | `GpuDownSampleConfig` | `downsample_voxel_size_x/y/z`, `downsample_use_centroid` |
| `BigmotRefineFilter` | `BigmotRefineFilterConfig` | 无额外可配参数 |
| `MarkAreaFilter` | `MarkAreaFilterConfig` | `area[]`（多边形区域 + filter_type + filter_foreground_type）, `filter_ratio` |
| `ObjectSplitFilter` | `ObjectSplitFilterConfig` | `forward_length`, `left_width`, `right_width`, `backard_length`, `split_interval`, `is_split_foreground_object`, `start_id` |
| `SemanticFilter` | `SemanticFilterConfig` | `filter_type[]`, `ground_prob_thr`, `ground_height_thr`, `is_filter_cone`, `only_filter_cluster`, `front_critical_filter[]` |

## 调用关系

```text
Inference (基类)
  └── MultiBatchInference
        ├── OnnxToTRTModel()  -->  TensorRT Builder/Parser/Engine
        └── Infer()  -->  context->enqueueV2() (CUDA 异步推理)

BaseDownSample (基类)
  └── GpuDownSample
        └── Process()  -->  downSample()  -->  VoxelDownSampleCuda (CUDA kernel)

BaseObjectFilter (基类)
  ├── BigmotRefineFilter
  │     └── Filter()  -->  RefineBigMotObjects()  -->  GetExpandBBox() + ConvexHull2D
  ├── MarkAreaFilter
  │     └── Filter()  -->  ComputeAreaDistance()  -->  FilterAreaObstacles()
  ├── ObjectSplitFilter
  │     └── Filter()  -->  IsCrossWithEgo()  -->  SplitObject() + BgObjectBuilder()
  └── SemanticFilter
        └── Filter()  -->  FillFilterFlags() + FillFilterFlagsOfGround()
                          + FillFilterFlagsOfFrontCritical()
```
