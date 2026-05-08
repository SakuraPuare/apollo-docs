---
title: "车道线检测"
---

# 车道线检测

> 源码路径: `modules/perception/lane_detection/`

## 概述

车道线检测模块负责从摄像头图像中识别车道线并输出车道信息。采用插件化架构，支持 DarkSCNN 和 Denseline 两种深度学习检测方案，均基于 GPU 推理（TensorRT / MIGraphX）。

Cyber RT 组件接收双目摄像头图像（默认 `front_6mm` + `front_12mm`），经 `LaneCameraPerception` 管线依次执行车道检测、2D 后处理、标定服务更新和 3D 后处理，最终以 `PerceptionLanes` 消息发布结果。

## 核心类

### LaneDetectionComponent

Cyber RT 组件入口，继承 `apollo::cyber::Component<>`，管理摄像头监听、运动信息融合和管线生命周期。

```cpp
class LaneDetectionComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
 private:
  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>&, const std::string&);
  void OnMotionService(const MotionServiceMsgType&);
  int InternalProc(const std::shared_ptr<apollo::drivers::Image const>&, const std::string&,
                   apollo::common::ErrorCode*, SensorFrameMessage*, PerceptionLanes*);
  int InitConfig();
  int InitSensorInfo();
  int InitCameraFrames();
  int InitProjectMatrix();
  int InitMotionService();
  int InitCameraListeners();
  void SetCameraHeightAndPitch();
  int MakeProtobufMsg(double, const std::string&, const camera::CameraFrame&, PerceptionLanes*);
};
```

### LaneCameraPerception

车道感知管线核心，实现 `BaseCameraPerception`，组合检测器、后处理器和标定服务。

```cpp
class LaneCameraPerception final : public BaseCameraPerception {
 public:
  bool Init(const CameraPerceptionInitOptions& options) override;
  bool Perception(const CameraPerceptionOptions& options, CameraFrame* frame) override;
  void InitLane(base::BaseCameraModelPtr& model,
                const app::PerceptionParam& perception_param);
  void InitCalibrationService(const base::BaseCameraModelPtr model,
                              const app::PerceptionParam& perception_param);
  void SetIm2CarHomography(const Eigen::Matrix3d& homography_im2car);
};
```

### BaseLaneDetector / BaseLanePostprocessor

检测器与后处理器抽象基类，通过注册机制实现插件化。

```cpp
class BaseLaneDetector {
 public:
  virtual bool Init(const LaneDetectorInitOptions& options) = 0;
  virtual bool Detect(const LaneDetectorOptions& options, CameraFrame* frame) = 0;
};

class BaseLanePostprocessor {
 public:
  virtual bool Init(const LanePostprocessorInitOptions& options) = 0;
  virtual bool Process2D(const LanePostprocessorOptions& options, CameraFrame* frame) = 0;
  virtual bool Process3D(const LanePostprocessorOptions& options, CameraFrame* frame) = 0;
  virtual void SetIm2CarHomography(const Eigen::Matrix3d& homography_im2car) {}
};
```

**检测器实现:** `DarkSCNNLaneDetector`（多车道，13 类）、`DenselineLaneDetector`

**后处理器实现:** `DarkSCNNLanePostprocessor`（单应性矩阵转换 + 多项式拟合）、`DenselineLanePostprocessor`（连通域分析 + 得分图提取）

### CommonFunctions

通用算法工具，位于 `common/common_functions.h`，提供多项式拟合、RANSAC、连通域和坐标转换。

```cpp
struct LanePointInfo { int type; float score; float x; float y; };

// 多项式拟合（QR 分解，最高 3 阶）
template <typename Dtype>
bool PolyFit(const EigenVector<Eigen::Matrix<Dtype, 2, 1>>& pos_vec,
             const int& order,
             Eigen::Matrix<Dtype, max_poly_order + 1, 1>* coeff,
             const bool& is_x_axis = true);

// RANSAC 拟合
template <typename Dtype>
bool RansacFitting(const EigenVector<Eigen::Matrix<Dtype, 2, 1>>& pos_vec,
                   EigenVector<Eigen::Matrix<Dtype, 2, 1>>* selected_points,
                   Eigen::Matrix<Dtype, 4, 1>* coeff, /* ... */);

// 连通域检测与坐标转换
class DisjointSet { /* 并查集 */ };
class ConnectedComponent { /* 连通域 */ };
bool FindCC(/* ... */);
bool ImagePoint2Camera(/* ... */);
```

## 核心函数

### LaneDetectionComponent::Init

组件初始化入口，依次执行配置加载、子模块初始化（`EXEC_ALL_FUNS` 宏遍历 `init_func_arry_` 数组）和可视化设置。

### LaneDetectionComponent::InternalProc

核心处理函数，在 `OnReceiveImage` 回调中调用。流程：获取 sensor-to-world 位姿 → 填充 `CameraFrame` → 调用 `LaneCameraPerception::Perception()` → 序列化为 Protobuf → 发布到 `/perception/lanes` 通道。

### LaneCameraPerception::Perception

感知管线主流程，仅在标定工作传感器（默认 `front_6mm`）上运行检测。

```cpp
bool LaneCameraPerception::Perception(const CameraPerceptionOptions& options,
                                      CameraFrame* frame) {
  if (lane_calibration_working_sensor_name_ == frame->data_provider->sensor_name()) {
    lane_detector_->Detect(...);                  // 车道检测
    lane_postprocessor_->Process2D(...);          // 2D 后处理
    frame->calibration_service->Update(frame);    // 更新标定
    lane_postprocessor_->Process3D(...);          // 3D 后处理
  } else {
    frame->calibration_service->Update(frame);    // 非工作传感器仅更新标定
  }
}
```

### LaneCameraPerception::InitLane

通过注册机制动态创建检测器和后处理器插件：`BaseLaneDetectorRegisterer::GetInstanceByName()` 创建检测器，`BaseLanePostprocessorRegisterer::GetInstanceByName()` 创建后处理器，随后分别调用 `Init()` 完成初始化。

## 配置

### 组件参数（`conf/lane_detection_config.pb.txt`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_names` | `front_6mm,front_12mm` | 摄像头名称（固定 2 个） |
| `input_camera_channel_names` | `/apollo/sensor/camera/.../image` | 图像输入通道 |
| `frame_capacity` | `20` | CameraFrame 环形缓冲区大小 |
| `enable_undistortion` | `false` | 是否启用图像去畸变 |
| `output_lanes_channel_name` | `/perception/lanes` | 车道线输出通道 |
| `default_camera_pitch` | `0.0` | 默认相机俯仰角（度） |
| `default_camera_height` | `1.5` | 默认相机离地高度（米） |
| `lane_calibration_working_sensor_name` | `front_6mm` | 标定工作传感器 |
| `ts_diff` | `0.1` | 时间戳差异阈值（秒） |

### 管线配置（`PerceptionParam` proto）

| 字段 | 说明 |
|------|------|
| `gpu_id` | GPU 设备 ID |
| `lane_param` | 车道参数列表，含 `lane_detector_param` 和 `lane_postprocessor_param` 插件配置 |
| `calibration_service_param` | 标定服务插件配置 |
| `debug_param` | 调试输出目录配置 |

## 调用关系

```text
Cyber RT Mainboard
  └─ LaneDetectionComponent::Init()
       ├─ InitConfig()                          // 从 pb.txt 加载参数
       ├─ InitSensorInfo()                      // 获取传感器信息和 TF
       ├─ InitCameraFrames()                    // 初始化 DataProvider、内外参
       ├─ InitProjectMatrix()                   // 计算窄角-广角投影矩阵
       ├─ InitMotionService()                   // 订阅运动服务
       └─ InitCameraListeners()                 // 注册图像回调
  ├─ [图像] OnReceiveImage() → InternalProc()
  │    └─ LaneCameraPerception::Perception()
  │         ├─ BaseLaneDetector::Detect()         // DarkSCNN / Denseline
  │         ├─ BaseLanePostprocessor::Process2D() // 2D 点集提取
  │         ├─ BaseCalibrationService::Update()   // 标定更新
  │         └─ BaseLanePostprocessor::Process3D() // 3D 拟合
  ├─ [运动] OnMotionService() → MotionBuffer
  └─ MakeProtobufMsg() → Writer::Write(PerceptionLanes)
```
