---
title: "激光雷达输出"
---

# 激光雷达输出

> 源码路径: `modules/perception/lidar_output/`

## 概述

激光雷达输出模块（lidar_output）是感知流水线的最终环节，负责将上游跟踪模块输出的 `SensorFrameMessage` 序列化为 `PerceptionObstacles` 消息并发布到 Cyber Channel。同时支持可选的基准帧（benchmark frame）保存功能，用于端到端评估。

模块还提供独立的 `LidarFrameMessage` 订阅通道，可将激光雷达检测/分割阶段的原始帧以 benchmark 格式落盘，供模型精度离线评测使用。基准帧的序列化与写入通过独立后台线程异步执行，避免阻塞主处理流程。

## 核心类

### LidarOutputComponent

Cyber 组件入口，继承 `cyber::Component<SensorFrameMessage>`。负责加载配置、创建输出 Writer 和可选的基准帧 Reader，接收上游帧消息并执行序列化与发布。

```cpp
class LidarOutputComponent : public cyber::Component<SensorFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;
  bool SaveBenchmarkFrame(const std::shared_ptr<SensorFrameMessage>& message);
  bool SaveBenchmarkLidarFrame(const std::shared_ptr<LidarFrameMessage>& message);
  void BenchmarkThreadFunc();
 private:
  std::string output_channel_name_;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  bool save_benchmark_frame_ = false;
  std::string benchmark_frame_output_dir_;
  std::mutex benchmark_mutex_;
  std::atomic<bool> is_terminate_;
  std::queue<std::shared_ptr<SensorFrameMessage>> message_buffer_;
  std::unique_ptr<std::thread> benchmark_thread_;
  std::string lidar_frame_output_dir_;
  std::shared_ptr<apollo::cyber::Reader<LidarFrameMessage>> lidar_frame_reader_;
  bool use_lidar_cooridinate_ = false;
  bool timestamp_two_decimal_format_ = false;
};
```

## 核心函数

### LidarOutputComponent::Init

初始化组件：加载 `LidarOutputComponentConfig` protobuf 配置，创建输出 Channel Writer。若启用基准帧保存，启动后台线程；若启用 LidarFrame 保存，创建 Reader 并绑定回调。

```cpp
bool LidarOutputComponent::Init() {
  LidarOutputComponentConfig comp_config;
  GetProtoConfig(&comp_config);
  output_channel_name_ = comp_config.output_channel_name();
  writer_ = node_->CreateWriter<PerceptionObstacles>(output_channel_name_);
  // 若 save_benchmark_frame 为 true，启动 BenchmarkThreadFunc 线程
  // 若 save_lidar_frame_message 为 true，创建 LidarFrameMessage Reader
}
```

### LidarOutputComponent::Proc

主处理回调：从 `SensorFrameMessage` 提取有效目标对象，调用 `MsgSerializer::SerializeMsg` 生成 `PerceptionObstacles` 并写入输出 Channel。若启用基准帧保存，同时将消息推入缓冲队列。

```cpp
bool LidarOutputComponent::Proc(
    const std::shared_ptr<SensorFrameMessage>& in_message) {
  // 提取 frame_->objects 中的有效目标
  // 若 save_benchmark_frame_，加入 message_buffer_
  // MsgSerializer::SerializeMsg 序列化为 PerceptionObstacles
  writer_->Write(out_message);
}
```

### LidarOutputComponent::SaveBenchmarkFrame

将 `SensorFrameMessage` 序列化为 `PerceptionBenchmarkFrame` 并以 protobuf ASCII 格式写入磁盘。文件名使用时间戳（支持两位小数格式），包含传感器 ID、时间戳和 sensor2world 位姿矩阵。

### LidarOutputComponent::SaveBenchmarkLidarFrame

类似 `SaveBenchmarkFrame`，但处理 `LidarFrameMessage`。从 `lidar_frame_->segmented_objects` 提取目标，通过 `MsgSerializer::SerializeLidarFrameMsg` 序列化，支持在激光雷达坐标系下输出（由 `use_lidar_cooridinate` 控制）。

### LidarOutputComponent::BenchmarkThreadFunc

后台线程函数：循环从 `message_buffer_` 取出消息，调用 `SaveBenchmarkFrame` 写盘。以 1ms 间隔轮询，收到终止信号后退出。

```cpp
void LidarOutputComponent::BenchmarkThreadFunc() {
  while (true) {
    // 加锁从 message_buffer_ 取消息
    if (is_terminate_.load()) break;
    if (message != nullptr) SaveBenchmarkFrame(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
```

## 配置

### 组件配置

Protobuf 定义位于 `proto/lidar_output_component_config.proto`：

```protobuf
message LidarOutputComponentConfig {
  optional string output_channel_name = 1;
  optional bool save_benchmark_frame = 2 [default = false];
  optional string benchmark_frame_output_dir = 3 [default = ""];
  optional bool save_lidar_frame_message = 4 [default = false];
  optional string lidar_frame_output_dir = 5 [default = ""];
  optional string lidar_frame_channel_name = 6 [default = "/perception/lidar/detection"];
  optional bool use_lidar_cooridinate = 7 [default = true];
  optional bool timestamp_two_decimal_format = 8 [default = false];
}
```

### 组件配置文件

`conf/lidar_output_config.pb.txt`：

```text
output_channel_name: "/apollo/perception/obstacles"
save_benchmark_frame: false
benchmark_frame_output_dir: "/apollo/data/benchmark/dt"
save_lidar_frame_message: false
lidar_frame_output_dir: "/apollo/data/benchmark/model_dt"
lidar_frame_channel_name: "/perception/lidar/detection"
use_lidar_cooridinate: true
timestamp_two_decimal_format: false
```

| 参数 | 含义 |
|------|------|
| `output_channel_name` | PerceptionObstacles 输出 Channel 名称 |
| `save_benchmark_frame` | 是否启用跟踪结果基准帧保存 |
| `benchmark_frame_output_dir` | 跟踪基准帧输出目录 |
| `save_lidar_frame_message` | 是否启用 LidarFrame 基准帧保存 |
| `lidar_frame_output_dir` | LidarFrame 基准帧输出目录 |
| `lidar_frame_channel_name` | LidarFrameMessage 订阅 Channel |
| `use_lidar_cooridinate` | LidarFrame 基准帧是否使用激光雷达坐标系 |
| `timestamp_two_decimal_format` | 基准帧文件名是否使用两位小数时间戳格式 |

## 调用关系

```text
LidarOutputComponent::Proc
  |
  +-- MsgSerializer::SerializeMsg (序列化 PerceptionObstacles)
  +-- Writer::Write (发布到 output_channel_name)
  |
  +-- [若 save_benchmark_frame]
       +-- message_buffer_.push (入队)
       +-- BenchmarkThreadFunc (后台线程)
             +-- SaveBenchmarkFrame
                   +-- MsgSerializer::SerializeBenchmarkMsg
                   +-- SetProtoToASCIIFile (落盘 .pb)

LidarFrameMessage Reader 回调 [若 save_lidar_frame_message]
  |
  +-- SaveBenchmarkLidarFrame
        +-- MsgSerializer::SerializeLidarFrameMsg
        +-- SetProtoToASCIIFile (落盘 .pb)
```

**Channel 关系：**

- 输入：`SensorFrameMessage`（来自 `/perception/inner/PrefusedObjects`，上游为 lidar_tracking）
- 输出：`/apollo/perception/obstacles`（`PerceptionObstacles`，供下游 planning 使用）
- 可选输入：`/perception/lidar/detection`（`LidarFrameMessage`，用于基准帧保存）
