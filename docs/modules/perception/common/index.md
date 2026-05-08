---
title: "Perception Common 感知公共库"
---

# Perception Common 感知公共库

> 源码路径：`modules/perception/common/`

## 概述

`perception/common` 是感知模块的公共基础设施库，定义了感知子系统共享的数据结构、算法接口、相机模型、高精地图工具和推理引擎封装。所有感知子模块（lidar、camera、radar 等）都依赖此库。

## 子模块结构

| 子目录 | 说明 |
|--------|------|
| `base/` | 基础数据结构：Object、Frame、BBox、Camera、Image 等 |
| `algorithm/` | 通用算法：点云处理、几何计算、IoU、NMS 等 |
| `camera/` | 相机模型与坐标变换工具 |
| `hdmap/` | 高精地图查询工具 |
| `inference/` | 推理引擎封装（TensorRT、Paddle 等） |
| `interface/` | 检测器基类接口 |
| `lib/` | 工具库：配置加载、线程池、计时器等 |

## 核心数据结构

### Frame

感知帧，包含一帧传感器数据和检测结果。

```cpp
struct Frame {
  SensorInfo sensor_info;
  double timestamp = 0.0;
  std::vector<std::shared_ptr<Object>> objects;
  Eigen::Affine3d sensor2world_pose;

  // 传感器特定补充数据
  LidarFrameSupplement lidar_frame_supplement;
  RadarFrameSupplement radar_frame_supplement;
  CameraFrameSupplement camera_frame_supplement;
  UltrasonicFrameSupplement ultrasonic_frame_supplement;
};
```

**源码**：`modules/perception/common/base/frame.h`

### Rect\<T\> / BBox2D\<T\>

2D 检测框的两种表示：

```cpp
// 左上角 + 宽高
template <typename T>
struct Rect {
  T x = 0, y = 0;      // 左上角
  T width = 0, height = 0;
  Point2D<T> Center() const;
  T Area() const;
  // 交集 & 并集 | 运算符
};

// 两点式
template <typename T>
struct BBox2D {
  T xmin = 0, ymin = 0;  // 左上角
  T xmax = 0, ymax = 0;  // 右下角
  Point2D<T> Center() const;
  T Area() const;
};
```

**源码**：`modules/perception/common/base/box.h`

**类型别名**：`RectI`/`RectF`/`RectD`、`BBox2DI`/`BBox2DF`/`BBox2DD`

### Object

感知障碍物对象（定义在 `base/object.h`），包含位置、尺寸、类型、速度、置信度等信息。

### Blob\<T\>

多维张量容器，用于推理引擎的数据传输。

**源码**：`modules/perception/common/base/blob.h`

### Camera

相机内参模型，支持畸变校正。

**源码**：`modules/perception/common/base/camera.h`、`modules/perception/common/base/distortion_model.h`

### Image / Image8U

图像数据容器，支持不同像素格式。

**源码**：`modules/perception/common/base/image.h`、`modules/perception/common/base/image_8u.h`

## 关键子目录

### algorithm/

通用算法实现：

- 点云下采样（`base_down_sample.h`）
- 基础障碍物检测器（`base_obstacle_detector.h`）
- IoU 计算、NMS 等

### inference/

推理引擎封装，支持多种后端：

- TensorRT
- PaddlePaddle
- 统一的 `Inference` 接口

### camera/

相机工具：

- 坐标系变换（图像坐标 ↔ 世界坐标）
- 畸变校正

### hdmap/

高精地图查询工具，提供车道线、路口等查询接口。

### interface/

检测器基类接口，定义统一的 `BaseObstacleDetector` 接口。

**源码**：`modules/perception/common/interface/base_obstacle_detector.h`

```cpp
class BaseObstacleDetector {
 public:
  virtual bool Init(const DetectorInitOptions& options = {}) = 0;
  virtual bool Detect(const DetectorOptions& options,
                      PerceptionObstacles* obstacles) = 0;
};
```

### lib/

工具库：

- 配置文件加载器
- 线程池
- 性能计时器
- 并发对象池（`concurrent_object_pool.h`）

## 调用关系

- **被依赖**：所有感知子模块（lidar_detection、camera_detection、radar_detection 等）
- **依赖**：Eigen、Protobuf、OpenCV（部分）
