---
title: "相机驱动"
---

# 相机驱动

> 源码路径: `modules/drivers/camera/`

## 概述

相机驱动模块负责通过 V4L2 (Video4Linux2) 接口采集 USB 相机的原始图像帧，并发布到 CyberRT 通道供下游感知模块消费。模块由两个 CyberRT Component 组成：`CameraComponent` 负责采集原始图像并发布 YUYV/RGB 格式消息；`CompressComponent` 订阅原始图像后执行 JPEG 压缩并发布压缩图像。底层 `UsbCam` 类封装了 V4L2 设备的打开、初始化、帧采集、像素格式转换（YUYV 到 RGB，x86 使用 AVX2 SIMD，aarch64 使用 NEON 或 CUDA）以及设备重连逻辑。支持 mjpeg、yuyv、uyvy、rgb24、yuvmono10 等像素格式，兼容 x86_64 与 aarch64 两种平台。

## 核心类

### CameraComponent

继承自 `apollo::cyber::Component<>`，是相机采集的主组件。初始化时读取 protobuf 配置、创建 `UsbCam` 实例、分配原始图像缓冲区（aarch64 平台可选 CUDA pinned memory），然后异步启动采集循环。采集循环中不断调用 `UsbCam::poll` 获取帧数据，填充 `apollo::drivers::Image` 消息并通过 `writer_`（RGB/YUYV 输出）和 `raw_writer_`（传感器原始输出）两个 Writer 发布。

```cpp
class CameraComponent : public apollo::cyber::Component<> {
 public:
    bool Init() override;
    ~CameraComponent();
 private:
    void run();
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Image>> writer_;
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Image>> raw_writer_;
    std::unique_ptr<UsbCam> camera_device_;
    // ...
};
```

### CompressComponent

继承自 `apollo::cyber::Component<apollo::drivers::Image>`，订阅原始图像通道，执行 JPEG 压缩后发布 `CompressedImage`。在 aarch64 平台使用 NVIDIA NvJPEGEncoder 硬件加速；在 x86 平台使用 OpenCV `cv::imencode` 实现软件压缩。支持 YUV422 到 YUV420 的下采样（aarch64 NEON 加速）。

```cpp
class CompressComponent : public apollo::cyber::Component<apollo::drivers::Image> {
 public:
    bool Init() override;
    bool Proc(const std::shared_ptr<apollo::drivers::Image>& image) override;
    // ...
};
```

### UsbCam

封装 V4L2 摄像头操作的核心类。负责设备文件打开、V4L2 格式协商、内存映射（mmap / read / userptr）初始化、流启停、帧读取、像素格式转换以及设备异常重连。

```cpp
class UsbCam {
 public:
    bool init(const std::shared_ptr<Config>& camera_config);
    CameraDeviceState poll(const CameraImagePtr& raw_image,
                           const CameraImagePtr& sensor_raw_image);
    bool wait_for_device();
    // ...
};
```

### CameraDeviceState

设备状态枚举，用于标识每帧采集的结果：

```cpp
enum CameraDeviceState : std::uint8_t {
    STATE_SUCCESS = 0,
    STATE_DROP,       // 帧率过高，丢弃
    STATE_INTR,       // 被信号中断
    STATE_DEVICE_ERROR,
    STATE_OTHER,
};
```

## 核心函数

### CameraComponent::Init()

读取 protobuf 配置文件，初始化 `UsbCam` 并设置像素格式，根据输出类型（YUYV/RGB）分配原始图像缓冲区（aarch64 可选 `cudaMallocHost`），创建 Writer 通道，异步启动 `run()` 采集线程。

### CameraComponent::run()

采集主循环。反复调用 `UsbCam::wait_for_device()` 确保设备就绪，调用 `poll()` 获取帧数据，将原始图像转换为 `apollo::drivers::Image` protobuf 消息并写入 `writer_` 和 `raw_writer_`。x86 平台在每帧间通过 `cyber::SleepFor` 控制帧率。

### UsbCam::init()

根据配置的像素格式字符串（yuyv/uyvy/mjpeg/rgb24/yuvmono10）映射为 V4L2 格式常量。若为 mjpeg 则初始化 FFmpeg 解码器。设置帧率告警与丢帧间隔阈值。aarch64 平台额外初始化 CUDA 色彩转换处理器。

### UsbCam::poll()

单次帧采集。通过 `select()` 等待设备可读，超时或异常时触发重连。调用 `read_frame()` 从 V4L2 缓冲区读取数据，根据 io_method（mmap/read/userptr）分发处理，并进行时间戳补偿和帧率控制（丢弃过密帧）。

### UsbCam::process_image()

像素格式转换核心。根据像素格式和输出类型选择转换路径：YUYV 到 RGB 在 x86 使用 `yuyv2rgb_avx()`（AVX2 SIMD 加速），aarch64 使用 CUDA 或 NEON。支持 UYVY 到 YUYV 的字节序翻转。

### CompressComponent::Proc()

接收原始图像，校验分辨率后执行 JPEG 压缩。aarch64 平台走 YUV422->YUV420 下采样 + NvJPEG 编码路径；x86 平台走 OpenCV RGB2BGR 转换 + `cv::imencode` 路径。压缩后通过 Writer 发布。

### UsbCam::wait_for_device()

设备就绪检查。依次尝试 `open_device()`、`init_device()`、`start_capturing()`，任一步骤失败则清理资源返回 false。x86 平台额外尝试配置硬件触发模式。

## 配置

配置通过 protobuf 文件 `modules/drivers/camera/proto/config.pb.h` 定义，主要字段包括：

| 字段 | 说明 |
|------|------|
| `camera_dev` | 设备路径，如 `/dev/video0` |
| `pixel_format` | 像素格式：yuyv / uyvy / mjpeg / yuvmono10 / rgb24 |
| `width` / `height` | 图像分辨率 |
| `frame_rate` | 目标帧率 |
| `output_type` | 输出格式：YUYV 或 RGB |
| `channel_name` | CyberRT 输出通道名 |
| `raw_channel_name` | 原始图像输出通道名 |
| `io_method` | V4L2 I/O 方式：MMAP / READ / USERPTR |
| `spin_rate` | 采集循环频率控制 |
| `device_wait_ms` | 设备未就绪时的等待时间（毫秒） |
| `arm_gpu_acceleration` | aarch64 平台是否启用 GPU 加速（CUDA） |
| `hardware_trigger` | 是否使用硬件触发模式 |
| `brightness` / `contrast` / `saturation` / `sharpness` / `gain` | V4L2 图像参数 |
| `auto_white_balance` / `white_balance` | 自动/手动白平衡 |
| `auto_exposure` / `exposure` | 自动/手动曝光 |
| `auto_focus` / `focus` | 自动/手动对焦 |
| `compress_conf` | JPEG 压缩配置（CompressComponent 使用） |

## 调用关系

```text
CameraComponent::Init()
  +-> UsbCam::init()                          // 初始化设备与像素格式
  +-> cyber::Async(CameraComponent::run)      // 异步启动采集线程

CameraComponent::run()                        // 采集主循环
  +-> UsbCam::wait_for_device()
  |     +-> open_device()                     // 打开 /dev/videoN
  |     +-> init_device()                     // V4L2 格式协商与缓冲区初始化
  |     +-> start_capturing()                 // VIDIOC_STREAMON
  +-> UsbCam::poll()
  |     +-> select()                          // 等待帧就绪
  |     +-> read_frame()                      // VIDIOC_DQBUF + 帧率控制
  |           +-> process_image()             // YUYV->RGB 转换
  |                 +-> yuyv2rgb_avx()        // x86 AVX2 SIMD
  |                 +-> CudaConvertHandler    // aarch64 CUDA
  +-> Writer::Write(Image)                    // 发布到 CyberRT 通道

CompressComponent::Proc(Image)                // 订阅回调
  +-> YUV422ToYUV420()                        // aarch64 NEON 下采样
  +-> NvJPEGEncoder::encodeFromBuffer()       // aarch64 硬件 JPEG
  +-> cv::imencode()                          // x86 软件 JPEG
  +-> Writer::Write(CompressedImage)          // 发布压缩图像
```
