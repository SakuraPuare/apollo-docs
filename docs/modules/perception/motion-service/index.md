---
title: "运动服务"
---

# Motion Service

> 源码路径: `modules/perception/motion_service/`

## 概述

Motion Service 是感知模块中的运动信息服务组件，负责融合相机图像时间戳与定位（Localization）数据，将高频定位信息对齐到相机帧时间戳上，计算帧间车辆平面运动矩阵，并通过 Cyber RT 通道发布 `MotionService` 消息供下游感知模块（如车道检测、目标追踪）使用。

组件通过订阅相机图像通道记录最新相机时间戳，订阅定位通道获取车辆速度和角速度，利用 `PlaneMotion` 在定位回调中累积运动并按相机时间戳分段推送，最终将运动缓冲区序列化为 Protobuf 消息发布。

## 核心类

### MotionServiceComponent

继承自 `apollo::cyber::Component<>`，是该模块的 Cyber RT 组件入口。

```cpp
class MotionServiceComponent final : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  bool GetMotionInformation(double timestamp, base::VehicleStatus *vs);
  base::MotionBuffer GetMotionBuffer();
  double GetLatestTimestamp();

 private:
  void OnLocalization(const LocalizationMsgType &localization);
  void OnReceiveImage(const ImageMsgType &message, const std::string &camera_name);
  void PublishEvent(const double timestamp);
  void ConvertVehicleMotionToMsgOut(base::VehicleStatus vs,
                                     apollo::perception::VehicleStatus *v_status_msg);

  PlaneMotion *vehicle_planemotion_;
  std::string device_id_;
  double pre_azimuth;
  double pre_timestamp_;
  double pre_camera_timestamp_;
  double camera_timestamp_;
  bool start_flag_;
  const int motion_buffer_size_ = 100;
  double timestamp_offset_ = 0.0;
  std::vector<std::string> camera_names_;
  std::vector<std::string> input_camera_channel_names_;
  std::mutex mutex_;
  std::mutex motion_mutex_;
  std::shared_ptr<apollo::cyber::Writer<apollo::perception::MotionService>> writer_;
};
```

### PlaneMotion

管理运动矩阵的累积与缓冲区，维护一个 `VehicleStatus` 的环形缓冲区，支持按时间戳查询历史运动。

```cpp
class PlaneMotion {
 public:
  enum { ACCUM_MOTION = 0, ACCUM_PUSH_MOTION, PUSH_ACCUM_MOTION, RESET };

  explicit PlaneMotion(int s);
  void add_new_motion(double pre_image_timestamp, double image_timestamp,
                      int motion_operation_flag, base::VehicleStatus *vehicledata);
  base::MotionBuffer get_buffer();
  bool find_motion_with_timestamp(double timestamp, base::VehicleStatus *vs);
  void cleanbuffer();
  void set_buffer_size(int s);
};
```

## 核心函数

### Init

组件初始化函数，完成以下工作：

1. 创建 `PlaneMotion` 实例（缓冲区大小 100）
2. 从 Protobuf 配置文件读取相机名称和通道名
3. 订阅第一个相机图像通道，注册 `OnReceiveImage` 回调
4. 订阅定位通道，注册 `OnLocalization` 回调
5. 创建 `MotionService` 消息写入器

### OnReceiveImage

接收相机图像消息，仅记录当前相机帧的 `measurement_time`，用于后续定位回调中对齐时间戳。

```cpp
void MotionServiceComponent::OnReceiveImage(const ImageMsgType &message,
                                            const std::string &camera_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double curr_timestamp = message->measurement_time() + timestamp_offset_;
  camera_timestamp_ = curr_timestamp;
}
```

### OnLocalization

核心处理函数。每次收到定位消息时：

1. 从定位消息中提取线速度（x/y/z）和角速度（roll/pitch/yaw rate），构建 `VehicleStatus`
2. 比较当前相机时间戳与上一次相机时间戳，决定运动操作类型：
   - **ACCUM_MOTION**：相机时间戳未变化，继续累积运动
   - **ACCUM_PUSH_MOTION**：相机时间戳前进了，累积后推送并发布
   - **RESET**：相机时间戳回退，重置运动状态
3. 调用 `PlaneMotion::add_new_motion` 更新运动缓冲区
4. 当发生 `ACCUM_PUSH_MOTION` 时调用 `PublishEvent` 发布消息

### PublishEvent

将运动缓冲区序列化为 `MotionService` Protobuf 消息并通过 Cyber RT Writer 发布。消息包含：

- Header（时间戳、模块名）
- `VehicleStatus` 列表（按时间倒序，包含速度、角速度、4x4 运动矩阵）

### GetMotionInformation / GetMotionBuffer / GetLatestTimestamp

供其他模块调用的接口：

- `GetMotionInformation`：按时间戳查找最近的车辆运动状态
- `GetMotionBuffer`：获取完整的运动缓冲区
- `GetLatestTimestamp`：获取最新处理的相机时间戳

## 配置

配置文件路径: `modules/perception/motion_service/conf/motion_service_config.pb.txt`

```text
camera_names: "front_6mm"
input_camera_channel_names: "/apollo/sensor/camera/front_6mm/image"
input_localization_channel_name: "/apollo/localization/pose"
output_topic_channel_name: "/apollo/perception/motion_service"
```

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `camera_names` | 相机传感器名称（逗号分隔） | `front_6mm` |
| `input_camera_channel_names` | 相机图像输入通道（逗号分隔，与 camera_names 一一对应） | `/apollo/sensor/camera/front_6mm/image` |
| `input_localization_channel_name` | 定位数据输入通道 | `/apollo/localization/pose` |
| `output_topic_channel_name` | 运动服务输出通道 | `/apollo/perception/motion_service` |

对应 Protobuf 定义位于 `modules/perception/motion_service/proto/motion_service_component.proto`。

## 调用关系

```text
Localization 通道 (/apollo/localization/pose)
        |
        v
  OnLocalization()
        |
        +-- 构建 VehicleStatus (速度 + 角速度)
        +-- PlaneMotion::add_new_motion()
        |       |
        |       +-- ACCUM_MOTION       → 累积运动矩阵
        |       +-- ACCUM_PUSH_MOTION  → 累积并推送至缓冲区
        |       +-- RESET              → 重置运动状态
        |
        +-- PublishEvent()
                |
                +-- GetMotionBuffer() → 读取运动缓冲区
                +-- ConvertVehicleMotionToMsgOut() → 序列化 4x4 运动矩阵
                +-- writer_->Write()  → 发布至 MotionService 通道

相机图像通道 (/apollo/sensor/camera/*/image)
        |
        v
  OnReceiveImage()
        |
        +-- 记录 camera_timestamp_ (供 OnLocalization 对齐时间)
```
