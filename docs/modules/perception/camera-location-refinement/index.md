---
title: "相机位置精修"
---

# 相机位置精修（Camera Location Refinement）

> 源码路径: `modules/perception/camera_location_refinement/`

## 概述

相机位置精修模块负责对摄像头检测到的障碍物进行 3D 位置后处理优化。该模块接收来自上游相机检测组件的 `CameraFrame` 消息，利用标定服务（`CalibrationService`）提供的地面平面模型，对每个障碍物的 3D 中心坐标、尺寸和旋转角进行精修，使检测结果在物理空间中更加准确。

模块采用 Cyber RT Component 架构，通过插件注册机制加载后处理器（`Postprocessor`）和标定服务（`CalibrationService`）。核心精修逻辑基于地面-3D 包围盒一致性约束和地面线段边界约束两种策略，在保证投影 IoU 不显著下降的前提下，优化物体的深度和横向位置。

## 核心类

### CameraLocationRefinementComponent

Cyber RT 组件，是模块的入口。继承自 `cyber::Component<onboard::CameraFrame>`，负责加载配置、初始化标定服务和后处理器，并在每帧处理中调用后处理器完成位置精修。

```cpp
class CameraLocationRefinementComponent final
    : public cyber::Component<onboard::CameraFrame> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<onboard::CameraFrame>& msg) override;
 private:
  std::shared_ptr<BasePostprocessor> postprocessor_;
  std::shared_ptr<BaseCalibrationService> calibration_service_;
  std::shared_ptr<cyber::Writer<onboard::CameraFrame>> writer_;
  apollo::common::EigenMap<std::string, Eigen::Matrix3f> name_intrinsic_map_;
};
```

### BasePostprocessor

后处理器抽象基类，定义了初始化和处理接口，通过 `PERCEPTION_REGISTER_REGISTERER` 宏支持插件注册。

```cpp
class BasePostprocessor {
 public:
  virtual bool Init(const PostprocessorInitOptions& options) = 0;
  virtual bool Process(const PostprocessorOptions& options,
                       onboard::CameraFrame* frame) = 0;
  virtual std::string Name() const = 0;
};
```

### LocationRefinerPostprocessor

`BasePostprocessor` 的具体实现，负责遍历帧中的所有检测物体，根据距离和 ROI 条件筛选后，调用 `ObjPostProcessor` 进行逐物体精修。

```cpp
class LocationRefinerPostprocessor : public BasePostprocessor {
 public:
  bool Init(const PostprocessorInitOptions& options) override;
  bool Process(const PostprocessorOptions& options,
               onboard::CameraFrame* frame) override;
 private:
  std::unique_ptr<ObjPostProcessor> postprocessor_;
  std::shared_ptr<BaseCalibrationService> calibration_service_;
  location_refiner::LocationRefinerParam location_refiner_param_;
};
```

### ObjPostProcessor

单物体级别的后处理器，实现基于地面平面的 3D 位置精修算法。包含软约束（`AdjustCenterWithGround`）和硬约束（`PostRefineCenterWithGroundBoundary`）两阶段优化。

```cpp
class ObjPostProcessor {
 public:
  void Init(const float* k_mat, int width, int height);
  bool PostProcessObjWithGround(const ObjPostProcessorOptions& options,
                                float center[3], float hwl[3], float* ry);
  bool PostProcessObjWithDispmap(const ObjPostProcessorOptions& options,
                                 float center[3], float hwl[3], float* ry);
};
```

### ObjPostProcessorParams

精修算法的超参数集合，控制迭代次数、学习率、IoU 权重等。

```cpp
struct ObjPostProcessorParams {
  int max_nr_iter = 5;
  float sampling_ratio_low = 0.1f;
  float weight_iou = 3.0f;
  float learning_r = 0.2f;
  float learning_r_decay = 0.9f;
  float dist_far = 15.0f;
  float shrink_ratio_iou = 0.9f;
  float iou_good = 0.5f;
};
```

## 核心函数

### CameraLocationRefinementComponent::Init

从 protobuf 配置文件加载参数，依次初始化标定服务和后处理器，创建输出 Cyber RT Writer 通道。

```cpp
bool CameraLocationRefinementComponent::Init() {
  CameraLocationRefinement location_refinement_param;
  if (!GetProtoConfig(&location_refinement_param)) { return false; }
  InitPostprocessor(location_refinement_param);
  writer_ = node_->CreateWriter<onboard::CameraFrame>(
      location_refinement_param.channel().output_obstacles_channel_name());
  return true;
}
```

### CameraLocationRefinementComponent::Proc

每帧处理入口。构造 `PostprocessorOptions`，拷贝输入帧数据到输出消息，调用后处理器执行精修，结果通过 Writer 发布。

```cpp
bool CameraLocationRefinementComponent::Proc(
    const std::shared_ptr<onboard::CameraFrame>& msg) {
  // 构造选项并拷贝帧数据
  // 调用 postprocessor_->Process()
  // writer_->Write(out_message)
}
```

### LocationRefinerPostprocessor::Process

遍历帧中所有检测物体，根据距离阈值（`min_dist_to_camera`）和 ROI 判定（`is_in_roi`）筛选目标，提取 2D 框、3D 尺寸、旋转角等信息，调用 `ObjPostProcessor::PostProcessObjWithGround` 完成精修，并将结果写回物体的 `local_center` 和 `center` 字段。

```cpp
bool LocationRefinerPostprocessor::Process(
    const PostprocessorOptions& options, onboard::CameraFrame* frame);
```

### ObjPostProcessor::PostProcessObjWithGround

两阶段精修主函数。先执行软约束调整（基于投影 IoU 的梯度下降），再执行硬约束调整（基于地面线段边界），合并两步结果。当物体深度超过 `dist_far`（默认 15m）时仅执行软约束。

```cpp
bool ObjPostProcessor::PostProcessObjWithGround(
    const ObjPostProcessorOptions& options,
    float center[3], float hwl[3], float* ry);
```

### ObjPostProcessor::AdjustCenterWithGround

软约束精修：通过迭代优化使 3D 包围盒在图像上的投影 IoU 与 2D 框保持一致。每轮将物体中心投影到地面平面上，根据 IoU 变化和位移计算代价函数，使用衰减学习率更新中心坐标。

```cpp
bool ObjPostProcessor::AdjustCenterWithGround(
    const float* bbox, const float* hwl, float ry,
    const float* plane, float* center) const;
```

### ObjPostProcessor::PostRefineCenterWithGroundBoundary

硬约束精修：利用地面线段边界（如车道线底部线段）对物体中心的深度和横向位置进行约束校正。通过 `GetDxDzForCenterFromGroundLineSeg` 计算校正量，并在 IoU 退化过大时回滚。

```cpp
bool ObjPostProcessor::PostRefineCenterWithGroundBoundary(
    const float* bbox, const float* hwl, float ry, const float* plane,
    const std::vector<LineSegment2D<float>>& line_seg_limits,
    float* center, bool check_lowerbound) const;
```

### LocationRefinerPostprocessor::is_in_roi

判定物体底部中心是否位于图像的感兴趣区域内，采用以光心为中心的梯形 ROI 过滤远处或偏离中心的物体。

```cpp
bool is_in_roi(const float pt[2], float img_w, float img_h,
               float v, float h_down) const;
```

## 配置

模块配置通过 protobuf 定义，包含相机名称、后处理器插件、标定服务插件和通信通道。

### 组件配置（`CameraLocationRefinement`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `camera_name` | string | 相机名称，默认 `front_6mm` |
| `postprocessor_param` | PostprocessorParam | 后处理器插件参数 |
| `calibration_service_param` | CalibrationServiceParam | 标定服务插件参数 |
| `channel` | RefinementChannel | 输入输出通信通道 |

### 精修参数（`LocationRefinerParam`）

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `min_dist_to_camera` | 30.0 | 仅对距离小于此值的物体执行精修（单位：米） |
| `roi_h2bottom_scale` | 0.5 | ROI 底部延伸比例 |

### 算法超参数（`ObjPostProcessorParams`）

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `max_nr_iter` | 5 | 软约束最大迭代次数 |
| `sampling_ratio_low` | 0.1 | 采样比率下限 |
| `weight_iou` | 3.0 | IoU 项在代价函数中的权重 |
| `learning_r` | 0.2 | 初始学习率 |
| `learning_r_decay` | 0.9 | 学习率衰减系数 |
| `dist_far` | 15.0 | 仅执行软约束的距离阈值（米） |
| `shrink_ratio_iou` | 0.9 | IoU 退化容忍比率，低于此值则回滚 |
| `iou_good` | 0.5 | 初始 IoU 低于此值则跳过精修 |

### 默认配置文件（`camera_location_refinement_config.pb.txt`）

```text
camera_name: "front_6mm"
postprocessor_param {
  plugin_param {
    name: "LocationRefinerPostprocessor"
    config_path: "perception/camera_location_refinement/data"
    config_file: "location_refiner.pb.txt"
  }
}
calibration_service_param {
  calibrator_method: "LaneLineCalibrator"
  plugin_param {
    name: "OnlineCalibrationService"
  }
}
channel {
  output_obstacles_channel_name: "/perception/inner/location_refinement"
}
```

## 调用关系

```text
CameraLocationRefinementComponent::Init()
  -> InitPostprocessor()
    -> InitCalibrationService()
      -> SensorManager::GetUndistortCameraModel()  // 获取相机内参
      -> BaseCalibrationService::Init()             // 初始化标定服务
    -> BasePostprocessor::Init()                    // 加载 LocationRefinerPostprocessor
      -> GetProtoFromFile()                         // 读取 location_refiner.pb.txt
  -> Cyber Writer 创建输出通道

CameraLocationRefinementComponent::Proc()
  -> 构造 PostprocessorOptions
  -> 拷贝 CameraFrame 数据到输出消息
  -> LocationRefinerPostprocessor::Process()
    -> calibration_service_->QueryGroundPlaneInCameraFrame()  // 查询地面平面
    -> 遍历 detected_objects:
      -> is_in_roi()                          // ROI 过滤
      -> 计算 rotation_y = theta_ray + alpha  // 旋转角计算
      -> ObjPostProcessor::PostProcessObjWithGround()
        -> AdjustCenterWithGround()           // 软约束：IoU 梯度下降
        -> PostRefineCenterWithGroundBoundary() // 硬约束：地面线段校正
      -> 更新 local_center 和 center（世界坐标）
  -> writer_->Write(out_message)              // 发布精修结果
```
