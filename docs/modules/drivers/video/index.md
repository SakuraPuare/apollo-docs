---
title: "视频驱动"
---

# Video 模块

> 源码路径: `modules/drivers/video/`

## 概述

视频驱动模块负责通过 UDP Socket 接收来自车载摄像头的 H.265 编码视频流，将其解析为 `CompressedImage` Protobuf 消息并发布到 Cyber RT 通道。模块采用 RTP 协议传输，使用自定义帧头（`FrameHeader` + `RtpHeader`）标识帧边界与元数据。整体架构分为三层：Cyber RT 组件层、摄像头驱动层和 Socket 输入层。

## 核心类

### `CompCameraH265Compressed`

继承自 `cyber::Component<>`，是视频驱动的 Cyber RT 组件入口。负责初始化配置、创建 Writer 通道、启动轮询线程并发布压缩图像。

```cpp
class CompCameraH265Compressed : public Component<> {
 public:
  bool Init();
 private:
  void VideoPoll();
  std::shared_ptr<apollo::cyber::Writer<CompressedImage>> writer_;
  std::shared_ptr<std::thread> video_thread_;
  volatile bool runing_;
  std::unique_ptr<CameraDriver> camera_deivce_;
  std::string record_folder_;
  std::shared_ptr<CompressedImage> pb_image_;
};
```

### `CameraDriver`

摄像头驱动核心类，封装 Socket 输入和帧轮询逻辑。持有 `CameraH265Config` 配置和 `SocketInput` 实例。

```cpp
class CameraDriver {
 public:
  explicit CameraDriver(const CameraH265Config *h265_cfg);
  bool Poll(std::shared_ptr<CompressedImage> h265);
  void Init();
  int Port();
  int Record();
 protected:
  CameraH265Config config_;
  std::shared_ptr<SocketInput> input_;
  bool PollByFrame(std::shared_ptr<CompressedImage> h265);
};
```

### `SocketInput`

UDP Socket 输入类，负责绑定端口、接收 RTP 分片数据并重组为完整 H.265 帧。

```cpp
class SocketInput {
 public:
  void Init(uint32_t port);
  int GetFramePacket(std::shared_ptr<CompressedImage> h265);
 private:
  int sockfd_;
  int port_;
  uint8_t *buf_;   // 帧缓冲区，4MB
  uint8_t *pdu_;   // PDU 缓冲区，1500B
  int pkg_num_;
  int bytes_num_;
  uint32_t frame_id_;
  bool InputAvailable(int timeout);
};
```

### `RtpHeader` / `FrameHeader` / `HwPduPacket`

C 结构体，定义在 `input.h` 中，描述 RTP 包头和自定义帧头格式：

```c
// RtpHeader: version(2b), padding(1b), extension(1b), csrc_count(4b),
//   marker(1b), payloadtype(7b), seq(u16), timestamp(u32), ssrc(u32)

typedef struct FrameHeader {
  uint32_t magic0;      // 0xBBAABBAA
  uint32_t magic1;      // 0xAABBAABB
  uint8_t  PhyNo;
  uint8_t  frame_type;  // IDR: 1, other: 0
  uint8_t  error;       // error: 1, other: 0
  uint8_t  resv;
  uint32_t frame_size;
  uint32_t frame_id;
  uint32_t ts_sec;      // 秒
  uint32_t ts_usec;     // 微秒
  uint16_t height;      // 1920
  uint16_t width;       // 1080
  uint32_t format;      // H265
} FrameHeader;
```

## 核心函数

### `CompCameraH265Compressed::Init`

组件初始化入口。从配置文件加载 `CameraH265Config`，创建 `CameraDriver` 并初始化。若开启录制则准备存储目录。创建 Cyber RT Writer 并启动独立的 `VideoPoll` 线程。

```cpp
bool CompCameraH265Compressed::Init() {
  CameraH265Config video_config;
  GetProtoConfig(&video_config);
  camera_deivce_.reset(new CameraDriver(&video_config));
  camera_deivce_->Init();
  writer_ = node_->CreateWriter<CompressedImage>(
      video_config.compress_conf().output_channel());
  video_thread_ = std::make_shared<std::thread>(
      std::bind(&CompCameraH265Compressed::VideoPoll, this));
  // ...
}
```

### `CompCameraH265Compressed::VideoPoll`

轮询主循环，在独立线程中运行。持续调用 `CameraDriver::Poll` 获取帧数据，设置时间戳后通过 Writer 发布。支持录制模式将原始 H.265 数据写入文件。连续失败 256 次后退出循环。

```cpp
void CompCameraH265Compressed::VideoPoll() {
  while (!apollo::cyber::IsShutdown()) {
    if (!camera_deivce_->Poll(pb_image_)) {
      if (++poll_failure_number > 256) break;
      continue;
    }
    pb_image_->mutable_header()->set_timestamp_sec(
        cyber::Time::Now().ToSecond());
    writer_->Write(pb_image_);
  }
}
```

### `CameraDriver::Poll` / `CameraDriver::PollByFrame`

`Poll` 直接委托给 `PollByFrame`。`PollByFrame` 调用 `SocketInput::GetFramePacket` 接收一帧数据，设置 `frame_id` 并记录相机与主机时间差。

### `SocketInput::Init`

创建 UDP Socket，绑定指定端口，设置非阻塞模式，配置 4MB 接收缓冲区和地址复用。分配帧缓冲区（`H265_FRAME_PACKAGE_SIZE` = 4MB）和 PDU 缓冲区（`H265_PDU_SIZE` = 1500B）。

### `SocketInput::GetFramePacket`

从 UDP Socket 接收 RTP 分片并重组为完整 H.265 帧。通过魔数（`HW_CAMERA_MAGIC0`/`HW_CAMERA_MAGIC1`）识别帧头，逐包拼接帧数据直至完整。校验 RTP 序列号连续性和帧 ID 连续性，异常时输出错误日志。

### `SocketInput::InputAvailable`

使用 `poll()` 检测 Socket 是否有可读数据，超时时间为 `POLL_TIMEOUT`（1000ms）。避免 `recvfrom` 阻塞。

## 配置

配置通过 `CameraH265Config` Protobuf 消息加载，定义在 `proto/video_h265cfg.proto` 中：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `udp_port` | uint32 | 必填 | UDP 接收端口 |
| `frame_id` | string | 必填 | 摄像头帧标识 |
| `pixel_format` | string | `"yuyv"` | 像素格式 |
| `record` | uint32 | 必填 | 是否录制原始 H.265 流 |
| `width` / `height` | uint32 | 必填 | 图像宽高 |
| `frame_rate` | uint32 | 必填 | 帧率 |
| `monochrome` | bool | false | 是否灰度模式 |
| `brightness` / `contrast` / `saturation` / `sharpness` / `gain` | int32 | -1 | 图像调节参数 |
| `auto_focus` / `focus` | bool / int32 | false / -1 | 对焦控制 |
| `auto_exposure` / `exposure` | bool / int32 | true / 100 | 曝光控制 |
| `auto_white_balance` / `white_balance` | bool / int32 | true / 4000 | 白平衡控制 |
| `bytes_per_pixel` | uint32 | 3 | 每像素字节数 |
| `compress_conf.output_channel` | string | - | 输出 Cyber RT 通道名 |
| `compress_conf.image_pool_size` | uint32 | 20 | 图像对象池大小 |

录制模式下，文件保存路径由环境变量 `H265_SAVE_FOLDER` 决定，默认为当前目录。

## 调用关系

```text
CompCameraH265Compressed::Init()
  |-- GetProtoConfig()          // 加载 CameraH265Config
  |-- CameraDriver::Init()
  |     |-- SocketInput::Init() // 创建 UDP Socket、绑定端口
  |-- CreateWriter()            // 创建 Cyber RT Writer
  |-- VideoPoll() [新线程]
        |-- CameraDriver::Poll()
        |     |-- PollByFrame()
        |           |-- SocketInput::GetFramePacket()
        |                 |-- InputAvailable()    // poll() 检测可读
        |                 |-- recvfrom()           // 接收 UDP 分片
        |                 |-- 解析 RtpHeader + FrameHeader
        |                 |-- 拼接帧数据
        |-- Writer::Write()     // 发布 CompressedImage
        |-- [可选] 写入 H265 录制文件
```
