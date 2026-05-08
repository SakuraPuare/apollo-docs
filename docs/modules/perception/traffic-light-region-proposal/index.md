---
title: "交通灯区域提议"
---

# Traffic Light Region Proposal

> 源码路径: `modules/perception/traffic_light_region_proposal/`

## 概述

交通灯区域提议是感知模块中负责交通灯区域定位与投影的子系统。它从高精地图获取车辆前方的交通灯信号，结合车辆位姿（TF2 坐标变换）将三维边界点投影到相机图像平面上生成二维 ROI，并通过 Cyber RT 通道发布给下游检测模块。

该模块支持多相机场景，按焦距降序排列相机，优先选择长焦相机；投影超出视野时自动回退到短焦相机。同时监控相机工作状态，标记无图像输入的相机为不可用。

## 核心类

### TrafficLightsPerceptionComponent

继承自 `apollo::cyber::Component<>`，是 Cyber RT 组件入口，负责整体流程编排。

```cpp
class TrafficLightsPerceptionComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
 private:
  int InitConfig();
  int InitAlgorithmPlugin();
  int InitCameraListeners();
  int InitCameraFrame();
  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image> image,
                      const std::string& camera_name);
  bool QueryPoseAndSignals(const double ts, camera::CarPose* pose,
                           std::vector<apollo::hdmap::Signal>* signals);
  bool VerifyLightsProjection(const double& ts,
                              const trafficlight::TLPreprocessorOption& option,
                              const std::string& camera_name,
                              std::shared_ptr<camera::TrafficLightFrame> image_lights,
                              std::shared_ptr<TrafficDetectMessage> msg);
  bool UpdateCameraSelection(double timestamp,
                             const trafficlight::TLPreprocessorOption& option,
                             std::shared_ptr<camera::TrafficLightFrame> frame);
  void GenerateTrafficLights(const std::vector<apollo::hdmap::Signal>& signals,
                             std::vector<base::TrafficLightPtr>* traffic_lights);
  std::shared_ptr<trafficlight::BaseTLPreprocessor> preprocessor_;
  apollo::perception::map::HDMapInput* hd_map_;
  Buffer* tf2_buffer_;
  std::shared_ptr<apollo::cyber::Writer<TrafficDetectMessage>> traffic_detect_writer_;
};
```

### BaseTLPreprocessor

交通灯预处理器抽象基类，定义相机选择、时间同步、投影更新接口。通过 `PERCEPTION_REGISTER_REGISTERER` 宏支持插件注册。

```cpp
class BaseTLPreprocessor {
 public:
  virtual bool Init(const TrafficLightPreprocessorInitOptions& options) = 0;
  virtual std::string Name() const = 0;
  virtual bool UpdateCameraSelection(const camera::CarPose& pose,
      const TLPreprocessorOption& option,
      std::vector<base::TrafficLightPtr>* lights) = 0;
  virtual bool SyncInformation(const double ts, const std::string& camera_name) = 0;
  virtual bool UpdateLightsProjection(const camera::CarPose& pose,
      const TLPreprocessorOption& option, const std::string& camera_name,
      std::vector<base::TrafficLightPtr>* lights) = 0;
  virtual bool SetCameraWorkingFlag(const std::string& camera_name, bool is_working) = 0;
  virtual bool GetCameraWorkingFlag(const std::string& camera_name,
                                    bool* is_working) const = 0;
  virtual const std::vector<std::string>& GetCameraNamesByDescendingFocalLen() const = 0;
};
```

### TLPreprocessor

`BaseTLPreprocessor` 的具体实现，内部持有 `MultiCamerasProjection`，按焦距降序遍历相机选择最佳投影相机。

```cpp
class TLPreprocessor : public BaseTLPreprocessor {
 private:
  MultiCamerasProjection projection_;
  std::pair<double, std::string> selected_camera_name_;
  std::map<std::string, bool> camera_is_working_flags_;
  std::vector<base::TrafficLightPtrs> lights_on_image_array_;
  std::vector<base::TrafficLightPtrs> lights_outside_image_array_;
};
```

### MultiCamerasProjection

多相机投影工具类，通过 `SensorManager` 加载 Brown 畸变模型，按焦距降序排列，提供 3D 到 2D 投影能力。

```cpp
class MultiCamerasProjection {
 public:
  bool Init(const MultiCamerasInitOption& options);
  bool Project(const camera::CarPose& pose, const ProjectOption& option,
               base::TrafficLight* light) const;
  bool HasCamera(const std::string& camera_name) const;
 private:
  bool BoundaryBasedProject(const base::BrownCameraDistortionModelPtr camera_model,
      const Eigen::Matrix4d& c2w_pose,
      const std::vector<base::PointXYZID>& point, base::TrafficLight* light) const;
  std::vector<std::string> camera_names_;
  std::map<std::string, base::BrownCameraDistortionModelPtr> camera_models_;
};
```

## 核心函数

### TrafficLightsPerceptionComponent::OnReceiveImage

图像接收回调（主处理入口）。检查时间戳合法性，按频率节流跳帧，同步投影信息，填充相机帧数据，验证投影结果，发布 `TrafficDetectMessage`。

### TrafficLightsPerceptionComponent::QueryPoseAndSignals

从 TF2 获取车辆位姿，从高精地图查询前方 150m 范围内的交通灯信号。查询失败时回退使用缓存信号（有效间隔 1.5s）。

### TrafficLightsPerceptionComponent::GenerateTrafficLights

将高精地图 `Signal` 转换为 `TrafficLight` 对象，提取信号灯 ID、边界点坐标和停车线信息。

### TLPreprocessor::SelectCamera

从焦距最大相机开始遍历，检查工作状态和投影边界条件，选择第一个满足条件的相机。最小焦距相机不做边界检查。

### TLPreprocessor::ProjectLightsAndSelectCamera

将交通灯投影到所有相机图像平面，分类为图像内/外灯光列表，调用 `SelectCamera` 选择最佳相机。

### MultiCamerasProjection::BoundaryBasedProject

核心投影：3D 边界点经相机逆变换到相机坐标系，通过 Brown 畸变模型投影到 2D 平面，计算最小外接矩形作为 ROI。投影点在相机后方或 ROI 超出边界则返回失败。

## 配置

模块通过 Protobuf 配置驱动，定义在 `trafficlights_proposal_component.proto` 中：

```protobuf
message TrafficLight {
  optional string tl_tf2_frame_id = 1 [default = "world"];
  optional string tl_tf2_child_frame_id = 2
      [default = "perception_localization_100hz"];
  optional double tf2_timeout_second = 3 [default = 0.01];
  optional string camera_names = 4 [default = "front_6mm,front_12mm"];
  optional string camera_channel_names = 5
      [default = "/apollo/sensor/camera/front_6mm,/apollo/sensor/camera/front_12mm"];
  optional double tl_image_timestamp_offset = 6 [default = 0.0];
  optional int32 max_process_image_fps = 7 [default = 8];
  optional double query_tf_interval_seconds = 8 [default = 0.3];
  optional double valid_hdmap_interval = 9 [default = 1.5];
  optional double image_sys_ts_diff_threshold = 10 [default = 0.5];
  optional double sync_interval_seconds = 11 [default = 0.5];
  optional int32 default_image_border_size = 12 [default = 100];
  optional perception.PluginParam plugin_param = 13;
  optional int32 gpu_id = 14 [default = 0];
  optional string proposal_output_channel_name = 15
      [default = "/perception/inner/Detection"];
}
```

| 配置项 | 默认值 | 说明 |
|---|---|---|
| `camera_names` | `front_6mm,front_12mm` | 相机名称列表，按焦距自动排序 |
| `max_process_image_fps` | `8` | 最大处理帧率，控制跳帧频率 |
| `query_tf_interval_seconds` | `0.3` | TF 查询最小间隔（秒） |
| `valid_hdmap_interval` | `1.5` | 高精地图信号缓存有效时长（秒） |
| `sync_interval_seconds` | `0.5` | 投影与图像的时间同步间隔（秒） |
| `default_image_border_size` | `100` | 图像边界区域像素大小 |

## 调用关系

```text
Init()
  ├── InitConfig()           -- 加载 Protobuf 配置，创建 Writer
  ├── InitAlgorithmPlugin()  -- 实例化 TLPreprocessor 插件，初始化 HDMap
  ├── InitCameraFrame()      -- 初始化各相机 DataProvider
  └── InitCameraListeners()  -- 注册 OnReceiveImage 回调

OnReceiveImage(image, camera_name)
  ├── CheckCameraImageStatus()           -- 检测相机工作状态
  ├── UpdateCameraSelection()            -- 定时查询位姿与信号
  │     ├── QueryPoseAndSignals()        -- TF2 位姿 + HDMap 信号
  │     ├── GenerateTrafficLights()      -- Signal -> TrafficLight
  │     └── preprocessor_->UpdateCameraSelection()
  │           ├── ProjectLightsAndSelectCamera()
  │           │     ├── ProjectLights()
  │           │     │     └── MultiCamerasProjection::Project()
  │           │     │           └── BoundaryBasedProject()  -- 3D->2D
  │           │     └── SelectCamera()   -- 按焦距选最佳相机
  │           └── GetMaxFocalLenWorkingCameraName()
  ├── SyncInformation()                  -- 时间戳同步校验
  ├── VerifyLightsProjection()           -- 验证当前帧投影
  │     ├── QueryPoseAndSignals()
  │     ├── GenerateTrafficLights()
  │     └── preprocessor_->UpdateLightsProjection()
  └── traffic_detect_writer_->Write()    -- 发布 TrafficDetectMessage
```
