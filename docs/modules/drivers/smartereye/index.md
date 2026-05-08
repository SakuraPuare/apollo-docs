---
title: "SmartEye 驱动"
---

# SmartEye 驱动

> 源码路径: `modules/drivers/smartereye/`

## 概述

SmartEye 驱动模块负责与 SmartEye 双目立体摄像头通信，采集图像帧数据并解析障碍物检测、车道线检测等扩展信息，最终通过 Cyber RT 通道发布。模块还提供可选的 JPEG 压缩组件，用于压缩原始图像后输出。

整体数据流：`SmartereyeDevice` 通过 TCP 连接到摄像头（`192.168.1.251`），启用障碍物检测和显示任务后，周期性调用 `poll()` 请求帧；`SmartereyeHandler` 作为 SDK 回调接收原始帧，转交给 `SmartereyeComponent` 解析并发布 `Image`、`SmartereyeObstacles`、`SmartereyeLanemark` 三种消息。`CompressComponent` 独立订阅 `Image` 通道，输出 JPEG 压缩后的 `CompressedImage`。

## 核心类

### SmartereyeComponent

主组件，继承自 `cyber::Component<>`。负责初始化设备、注册回调、异步轮询帧数据并解析发布。

```cpp
class SmartereyeComponent : public Component<> {
 public:
  bool Init() override;
  ~SmartereyeComponent();
 protected:
  void run();
  bool SetCallback();
  bool Callback(RawImageFrame *rawFrame);
  void processFrame(int frameId, char *image, char *extended,
                    int64_t time, int width, int height);
  void processFrame(int frameId, char *image, uint32_t dataSize,
                    int width, int height, int frameFormat);
 private:
  std::shared_ptr<Writer<Image>> writer_;
  std::shared_ptr<Writer<SmartereyeObstacles>> SmartereyeObstacles_writer_;
  std::shared_ptr<Writer<SmartereyeLanemark>> SmartereyeLanemark_writer_;
  std::unique_ptr<SmartereyeDevice> camera_device_;
  std::shared_ptr<Config> camera_config_;
  uint32_t spin_rate_;
  uint32_t device_wait_;
  std::future<void> async_result_;
  std::atomic<bool> running_;
  bool b_ispolling_;
};
```

### SmartereyeDevice

设备管理类，封装 `StereoCamera` SDK 的连接、断开和帧请求操作。

```cpp
class SmartereyeDevice {
 public:
  bool init(const std::shared_ptr<Config> &camera_config);
  bool is_capturing();
  bool wait_for_device();
  int uninit();
  bool SetCallback(CallbackFunc ptr);
  int poll();
 private:
  bool is_capturing_;
  bool inited_;
  std::shared_ptr<Config> config_;
  StereoCamera *pcamera_;
  SmartereyeHandler *pcameraHandler_;
};
```

### SmartereyeHandler

继承自 SDK 的 `CameraHandler`，将 `handleRawFrame` 接收到的原始帧通过回调函数转发给上层组件。

```cpp
class SmartereyeHandler : public CameraHandler {
 public:
  explicit SmartereyeHandler(std::string name);
  void handleRawFrame(const RawImageFrame *rawFrame);
  bool SetCallback(CallbackFunc ptr);
};
```

### CompressComponent

继承自 `cyber::Component<Image>`，订阅原始 `Image` 通道，使用 OpenCV 将 RGB 图像编码为 JPEG 并发布 `CompressedImage`。

```cpp
class CompressComponent : public Component<Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Image>& image) override;
 private:
  std::shared_ptr<CCObjectPool<CompressedImage>> image_pool_;
  std::shared_ptr<Writer<CompressedImage>> writer_;
  Config config_;
};
```

## 核心函数

### SmartereyeComponent::Init

从配置文件加载 `Config`，初始化 `SmartereyeDevice`，计算轮询间隔（`spin_rate` 转微秒），创建三个 Writer（Image / Obstacles / Lanemark），然后异步启动 `run()` 循环。

### SmartereyeComponent::run

异步线程入口，循环调用 `camera_device_->poll()` 请求帧并按 `spin_rate` 间隔休眠，直到 Cyber RT 关闭。

### SmartereyeComponent::Callback

SDK 帧回调入口。根据 `frameId` 分流：

- `FrameId::Compound` -- 同时包含图像和障碍物数据，调用 `processFrame` (6 参数版本) 解析 `OutputObstacles`，绘制障碍物标注后发布 `Image` 和 `SmartereyeObstacles`。
- `FrameId::LaneExt` -- 包含图像和车道线数据，解析 `LdwDataPack`，绘制车道线标注后发布 `Image` 和 `SmartereyeLanemark`。
- 其它帧（`Lane`、`Obstacle`、`Disparity`、`CalibLeftCamera`）-- 调用 `processFrame` (5 参数版本) 仅记录日志。

### SmartereyeDevice::init

通过 `StereoCamera::connect` 连接到摄像头（默认地址 `192.168.1.251`），创建 `SmartereyeHandler`，启用 `ObstacleTask` 和 `DisplayTask`。

### SmartereyeDevice::poll

调用 `pcamera_->requestFrame` 请求 `FrameId::Compound` 帧。

### CompressComponent::Proc

从对象池获取 `CompressedImage`，将 RGB 图像转为 BGR 后用 OpenCV `imencode` 编码为 JPEG（质量 95），写入压缩通道。

## 配置

配置通过 protobuf 定义（`proto/config.proto`），主要字段：

| 字段 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `channel_name_image` | string | 原始图像输出通道 | -- |
| `device_wait_ms` | uint32 | 设备等待超时（毫秒） | 2000 |
| `spin_rate` | uint32 | 轮询频率（Hz） | 200 |
| `width` / `height` | uint32 | 图像分辨率 | -- |
| `pixel_format` | string | 像素格式 | `"yuyv"` |
| `io_method` | IOMethod | IO 方式（mmap/userptr/read） | -- |
| `output_type` | OutputType | 输出类型（YUYV/RGB） | -- |
| `compress_conf.output_channel` | string | 压缩图像输出通道 | -- |
| `compress_conf.image_pool_size` | uint32 | 压缩图像对象池大小 | 20 |

示例配置文件位于 `conf/smartereye.pb.txt`。

## 调用关系

```text
SmartereyeComponent::Init
  +-- 读取 Config (GetProtoFromFile)
  +-- SmartereyeDevice::init
  |     +-- StereoCamera::connect("192.168.1.251")
  |     +-- new SmartereyeHandler
  |     +-- enableTasks(ObstacleTask | DisplayTask)
  +-- SmartereyeDevice::SetCallback -> SmartereyeHandler::SetCallback
  +-- CreateWriter<Image / SmartereyeObstacles / SmartereyeLanemark>
  +-- cyber::Async(run)

SmartereyeComponent::run  [异步线程]
  +-- loop: SmartereyeDevice::poll
        +-- StereoCamera::requestFrame -> SmartereyeHandler::handleRawFrame
              +-- CallbackFunc -> SmartereyeComponent::Callback
                    +-- processFrame (Compound/LaneExt -> 发布消息)
                    +-- processFrame (其它帧 -> 日志)

CompressComponent::Init  [独立组件]
  +-- GetProtoConfig
  +-- CCObjectPool<CompressedImage>
  +-- CreateWriter<CompressedImage>

CompressComponent::Proc  [Image 通道回调]
  +-- cv::cvtColor + cv::imencode (JPEG)
  +-- Writer::Write(CompressedImage)
```
