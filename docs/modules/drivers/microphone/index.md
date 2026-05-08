---
title: "麦克风驱动"
---

# Microphone

> 源码路径: `modules/drivers/microphone/`

## 概述

麦克风驱动模块基于 PortAudio 库实现音频采集，通过 Cyber RT 框架将多通道音频数据发布到消息总线。当前仅支持 ReSpeaker 硬件设备，模块自动扫描系统音频设备列表并匹配名称中包含 "ReSpeaker" 的输入设备。采集完成后，原始交错格式的 PCM 数据被拆分为独立通道，封装为 `AudioData` protobuf 消息发布。

## 核心类

### Stream

PortAudio 流的轻量封装，负责打开音频输入流并读取原始 PCM 帧数据。

```cpp
class Stream {
 private:
  PaStream *pastream_ptr_;
  PaStreamParameters *input_parameters_ptr_;

 public:
  ~Stream();
  void init_stream(int rate, int channels, int chunk,
                   int input_device_index, PaSampleFormat format);
  void read_stream(int n_frames, char *buffer) const;
};
```

- `init_stream` 设置输入参数（设备索引、通道数、采样格式、延迟），调用 `Pa_OpenStream` / `Pa_StartStream` 启动采集。
- `read_stream` 调用 `Pa_ReadStream` 读取指定帧数，读取失败时抛出 `std::runtime_error`。

### Respeaker

ReSpeaker 设备管理器，封装 PortAudio 初始化、设备发现和流的生命周期。

```cpp
class Respeaker {
 private:
  std::unique_ptr<Stream> stream_ptr_;
  const PaDeviceIndex get_respeaker_index() const;
  const PaSampleFormat get_format_from_width(int width,
                                             bool is_unsigned = true) const;

 public:
  void init(const std::shared_ptr<const MicrophoneConfig> &microphone_config);
  void read_stream(int n_frames, char *buffer) const;
};
```

### MicrophoneComponent

Cyber RT 组件入口，继承自 `Component<>`，通过 `CYBER_REGISTER_COMPONENT` 注册。负责从配置文件加载参数、分配缓冲区、异步驱动采集循环并将分通道音频数据写入 Cyber Writer。

```cpp
class MicrophoneComponent : public Component<> {
 public:
  bool Init() override;
  ~MicrophoneComponent();

 private:
  void run();
  void fill_channel_data(int chunk_i);
};
```

## 核心函数

### MicrophoneComponent::Init

组件初始化入口，完成以下工作：

1. 从 `config_file_path_` 加载 `MicrophoneConfig` protobuf 配置。
2. 创建 `Respeaker` 实例并调用 `init` 打开音频设备。
3. 根据配置计算缓冲区大小：`chunk_size_ = chunk_ * n_channels_ * sample_width_`，`n_chunk_ = ceil(sample_rate * record_seconds / chunk_)`。
4. 构建 `AudioData` 消息结构，为每个通道预分配 `n_chunk_ * chunk_ * sample_width_` 字节的空间。
5. 创建 Cyber Writer 发布到配置指定的 channel。
6. 通过 `cyber::Async` 启动异步采集线程。

### MicrophoneComponent::run

采集主循环，在 `cyber::IsShutdown()` 之前持续运行：

```cpp
void MicrophoneComponent::run() {
  int chunk_i;
  while (!cyber::IsShutdown()) {
    for (chunk_i = 0; chunk_i < n_chunk_; ++chunk_i) {
      microphone_device_ptr_->read_stream(chunk_, buffer_);
      fill_channel_data(chunk_i);
    }
    FillHeader(node_->Name(), audio_data_ptr_.get());
    writer_ptr_->Write(audio_data_ptr_);
  }
}
```

每次循环读取 `n_chunk_` 个 chunk，每个 chunk 读完后立即分通道写入 `AudioData`，全部 chunk 读完后发布一条完整消息。

### MicrophoneComponent::fill_channel_data

将交错（interleaved）PCM 数据拆分为独立通道：

```cpp
void MicrophoneComponent::fill_channel_data(int chunk_i) {
  int pos = chunk_i * chunk_ * sample_width_;
  for (int buff_i = 0; buff_i < chunk_size_; pos += sample_width_) {
    for (int channel_i = 0; channel_i < n_channels_; ++channel_i) {
      for (int di = 0; di < sample_width_; ++di) {
        (*channel_data_ptrs_[channel_i])[pos + di] = buffer_[buff_i++];
      }
    }
  }
}
```

内层循环按 sample_width 逐字节拷贝，将 `buffer_` 中 LRLRLR... 交错排列的数据分离到各通道的独立缓冲区。

### Respeaker::init

调用 `Pa_Initialize` 初始化 PortAudio，通过 `get_respeaker_index` 遍历设备列表查找 ReSpeaker，随后创建 `Stream` 并以配置参数启动采集流。

### Respeaker::get_respeaker_index

遍历默认 Host API 下的所有设备，查找设备名称包含 "ReSpeaker" 的设备索引。

### Respeaker::get_format_from_width

根据采样位宽（字节数）映射 PortAudio 采样格式：1 字节对应 `paUInt8`/`paInt8`，2 字节对应 `paInt16`，3 字节对应 `paInt24`，4 字节对应 `paFloat32`。

## 配置

配置文件为 protobuf text 格式，路径通过 Cyber RT 组件的 `config_file_path` 参数指定。默认配置 `conf/respeaker.pb.txt` 示例：

```text
microphone_model: RESPEAKER
chunk: 8192
sample_rate: 48000
record_seconds: 0.17
sample_width: 2
channel_name: "/apollo/sensor/microphone"
frame_id: "respeaker"
mic_distance: 0.065
channel_type: ASR
channel_type: RAW
channel_type: RAW
channel_type: RAW
channel_type: RAW
channel_type: PLAYBACK
```

主要字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `microphone_model` | enum | 设备型号，当前仅支持 `RESPEAKER` |
| `chunk` | int32 | 每次读取的帧数 |
| `sample_rate` | float | 采样率（Hz），如 48000 |
| `record_seconds` | float | 单条消息覆盖的录音时长（秒） |
| `sample_width` | int32 | 每采样的字节数 |
| `channel_name` | string | Cyber RT 发布通道名 |
| `channel_type` | repeated | 通道用途，取值 `ASR` / `RAW` / `PLAYBACK` |

## 调用关系

```text
Cyber RT 启动
  -> MicrophoneComponent::Init()
       -> GetProtoFromFile() 加载 MicrophoneConfig
       -> Respeaker::init()
            -> Pa_Initialize()
            -> get_respeaker_index() 查找设备
            -> Stream::init_stream()
                 -> Pa_OpenStream() + Pa_StartStream()
       -> 分配 buffer_ 和 AudioData 通道内存
       -> CreateWriter<AudioData>()
       -> cyber::Async(run)

采集线程 (run)
  -> loop:
       -> Respeaker::read_stream()
            -> Stream::read_stream() -> Pa_ReadStream()
       -> fill_channel_data() 交错数据拆分到各通道
       -> FillHeader()
       -> writer_ptr_->Write(AudioData)

MicrophoneComponent::~MicrophoneComponent()
  -> free(buffer_)
  -> 等待异步线程结束
Respeaker::~Respeaker()
  -> Pa_Terminate()
Stream::~Stream()
  -> Pa_CloseStream()
```
