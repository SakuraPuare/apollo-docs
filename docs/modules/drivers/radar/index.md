---
title: "雷达驱动"
---

# 雷达驱动

> 源码路径：`modules/drivers/radar/`

## 概述

`radar` 驱动模块负责对接 Apollo 中所有毫米波雷达和超声波雷达传感器，通过 CAN 总线或 UDP 网络接口接收原始数据并解析为 protobuf 消息。模块包含 5 种雷达驱动子模块，分别适配大陆（Continental）ARS408、赛恩领动（Racobit）、纳瓦（Nano）、傲酷（Oculii）4D 雷达和超声波雷达。

CAN 类雷达（conti / racobit / nano / ultrasonic）共享 `drivers/canbus` 基础设施，使用 `CanClient` + `CanReceiver` + `MessageManager` 模板框架；Oculii 雷达通过 UDP socket 异步接收点云帧数据。

## 核心类

### ContiRadarCanbusComponent

大陆 ARS408 毫米波雷达驱动组件，继承 `cyber::Component<>`。除基本 CAN 收发外，额外订阅定位话题以获取自车速度和横摆角速度，通过 `MotionInputSpeed300` / `MotionInputYawRate301` 报文馈入雷达做运动补偿。

```cpp
class ContiRadarCanbusComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;

 private:
  bool Start();
  void Stop();
  ErrorCode ConfigureRadar();
  void PoseCallback(const std::shared_ptr<LocalizationEstimate>& pose);

  ContiRadarConf conti_radar_conf_;
  std::shared_ptr<CanClient> can_client_;
  CanReceiver<ContiRadar> can_receiver_;
  std::unique_ptr<ContiRadarMessageManager> sensor_message_manager_;
  std::shared_ptr<cyber::Writer<ContiRadar>> conti_radar_writer_;
  std::shared_ptr<cyber::Reader<LocalizationEstimate>> pose_reader_;
};
```

**源码**：`modules/drivers/radar/conti_radar/conti_radar_canbus_component.h`

### ContiRadarMessageManager

大陆雷达消息管理器，继承 `MessageManager<ContiRadar>`。维护 CAN ID 到协议解析器的映射，在 `Parse()` 中按 ID 调用对应的 `ProtocolData<ContiRadar>` 解码并组装完整的 `ContiRadar` protobuf 消息，通过 Writer 发布。

```cpp
class ContiRadarMessageManager
    : public apollo::drivers::canbus::MessageManager<ContiRadar> {
 public:
  explicit ContiRadarMessageManager(
      const std::shared_ptr<cyber::Writer<ContiRadar>>& writer);
  void set_radar_conf(RadarConf radar_conf);
  ProtocolData<ContiRadar>* GetMutableProtocolDataById(uint32_t message_id);
  void Parse(uint32_t message_id, const uint8_t* data, int32_t length);
  void set_can_client(std::shared_ptr<CanClient> can_client);
};
```

**源码**：`modules/drivers/radar/conti_radar/conti_radar_message_manager.h`

### NanoRadarCanbusComponent

纳瓦毫米波雷达驱动组件，与 `ContiRadarCanbusComponent` 结构基本相同，但额外支持 `ConfigureRadarRegion()` 配置雷达检测区域。不订阅定位话题（不做运动补偿输入）。

```cpp
class NanoRadarCanbusComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;

 private:
  ErrorCode ConfigureRadar();
  ErrorCode ConfigureRadarRegion();

  NanoRadarConf nano_radar_conf_;
  std::shared_ptr<CanClient> can_client_;
  CanReceiver<NanoRadar> can_receiver_;
  std::unique_ptr<NanoRadarMessageManager> sensor_message_manager_;
  std::shared_ptr<cyber::Writer<NanoRadar>> nano_radar_writer_;
};
```

**源码**：`modules/drivers/radar/nano_radar/nano_radar_canbus_component.h`

### RacobitRadarCanbusComponent

赛恩领动毫米波雷达驱动组件，结构与大陆雷达类似，通过 CAN 总线收发。使用独立的 `RacobitRadar` protobuf 和 CAN 协议解析器。

**源码**：`modules/drivers/radar/racobit_radar/racobit_radar_canbus_component.h`

### OculiiRadarComponent

傲酷 4D 毫米波雷达驱动组件，通过 UDP socket 接收点云帧数据（非 CAN 总线）。`Init()` 中创建 UDP 解析器并启动异步解析线程 `run()`，循环从解析器获取 `OculiiPointCloud` 并发布。

```cpp
class OculiiRadarComponent : public Component<> {
 public:
  bool Init() override;

 private:
  void run();

  std::shared_ptr<Writer<OculiiPointCloud>> writer_;
  std::unique_ptr<OculiiRadarUdpParser> parser_;
  std::shared_ptr<OculiiRadarConf> config_;
  std::atomic<bool> running_ = {false};
  float frame_drop_interval_;
};
```

**源码**：`modules/drivers/radar/oculii_radar/oculii_radar_component.h`

### OculiiRadarUdpParser

Oculii 雷达 UDP 数据包解析器。在独立线程中异步接收 UDP 数据包入队，`Parse()` 方法从队列中按握手帧 -> 数据帧 -> 尾帧的顺序组装完整帧，再解析 Header、Detection、Track、Footer 四段数据，将原始极坐标检测点转换为笛卡尔坐标点云并解析跟踪目标。

```cpp
class OculiiRadarUdpParser {
 public:
  bool Init(uint16_t port);
  int Parse(OculiiPointCloud& oculii_output);

 private:
  void AsyncUdpInput();
  int ParseHeader(OculiiHeader& header, void* buffer);
  int ParseDection(OculiiDetection& dection, void* buffer);
  int ParseTrack(OculiiTrack& track, void* buffer);
  int ParseFooter(OculiiFooter& footer, void* buffer);
  bool IsMultiCastMode(float hr, float hd, float ha, float he,
                       float fr, float fd, float fa, float fe);
};
```

**源码**：`modules/drivers/radar/oculii_radar/parser/oculii_radar_udp_parser.h`

### UltrasonicRadarCanbusComponent / UltrasonicRadarCanbus

超声波雷达驱动，采用两层设计：`UltrasonicRadarCanbusComponent`（CyberRT 组件壳）委托 `UltrasonicRadarCanbus`（实际业务逻辑）完成初始化和启动。

**源码**：`modules/drivers/radar/ultrasonic_radar/ultrasonic_radar_canbus_component.h`、`ultrasonic_radar_canbus.h`

## 核心函数

### CAN 类雷达 Init 流程

以下以大陆雷达为代表，其余 CAN 类雷达流程相同。

```cpp
// ContiRadarCanbusComponent::Init()
bool ContiRadarCanbusComponent::Init() {
  // 1. 加载 protobuf 配置
  GetProtoConfig(&conti_radar_conf_);

  // 2. 创建 CAN 客户端
  auto can_factory = CanClientFactory::Instance();
  can_client_ = can_factory->CreateCANClient(
      conti_radar_conf_.can_conf().can_card_parameter());

  // 3. 创建消息管理器并关联 Writer
  sensor_message_manager_.reset(new ContiRadarMessageManager(conti_radar_writer_));
  sensor_message_manager_->set_radar_conf(conti_radar_conf_.radar_conf());
  sensor_message_manager_->set_can_client(can_client_);

  // 4. 初始化 CAN 接收器
  can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                     conti_radar_conf_.can_conf().enable_receiver_log());

  // 5. 启动
  return Start();
}
```

### ContiRadarCanbusComponent::PoseCallback()

大陆雷达特有：订阅定位消息，提取自车速度和横摆角速度，按配置的发送间隔通过 CAN 发送给雷达用于运动补偿。

```cpp
void ContiRadarCanbusComponent::PoseCallback(
    const std::shared_ptr<LocalizationEstimate>& pose_msg) {
  // 限频
  if ((now_nsec - last_nsec_) < send_interval) return;

  // 从四元数计算旋转矩阵，提取纵向速度
  Eigen::Quaterniond orientation(...);
  Eigen::Matrix3d rotation_matrix = orientation.toRotationMatrix().inverse();
  float speed = (rotation_matrix * speed_v).y();
  float yaw_rate = pose_msg->pose().angular_velocity().z() * 180.0f / M_PI;

  // 发送运动输入报文
  MotionInputSpeed300 input_speed;   // CAN ID 0x300
  MotionInputYawRate301 input_yawrate;  // CAN ID 0x301
  can_client_->SendSingleFrame(...);
}
```

### OculiiRadarUdpParser::Parse()

Oculii 雷达核心解析函数，从 UDP 队列中按帧格式组装数据，逐字节解析检测点和跟踪目标。

```cpp
int OculiiRadarUdpParser::Parse(OculiiPointCloud& oculii_output) {
  // 1. 从队列取握手帧，获得数据总长度
  queue_->WaitDequeue(&handshake);
  uint32_t data_length = handshake.data_length;

  // 2. 循环取数据帧拼接完整数据包
  while (data_length > 0) { queue_->WaitDequeue(&packet); }

  // 3. 解析 Header -> Detection[] -> Track[] -> Footer
  ParseHeader(header, buffer);
  for (i = 0; i < header.dections_number; i++) ParseDection(...);
  for (i = 0; i < header.tracks_number; i++) ParseTrack(...);
  ParseFooter(footer, buffer);

  // 4. 极坐标转笛卡尔坐标（含 radar->lidar 坐标变换）
  // 5. 填充 OculiiPointCloud protobuf
  return 0;
}
```

## 配置

### CAN 类雷达

通过各自 protobuf 配置文件加载（如 `ContiRadarConf`、`NanoRadarConf`）：

| 字段路径 | 类型 | 说明 |
|----------|------|------|
| `can_conf.can_card_parameter` | CANCardParameter | CAN 卡硬件参数 |
| `can_conf.enable_receiver_log` | bool | 是否启用接收日志 |
| `radar_conf` | RadarConf | 雷达参数配置（速度分辨率、检测区域等） |
| `radar_channel` / gflags topic | string | 发布通道名 |

大陆雷达额外配置：

| 字段 | 说明 |
|------|------|
| `radar_conf.input_send_interval` | 运动输入报文发送间隔（纳秒） |

纳瓦雷达额外配置：

| 字段 | 说明 |
|------|------|
| `radar_conf` 中区域字段 | 检测区域参数，通过 `RegionConfig401` 报文配置 |

### Oculii 雷达

通过 `OculiiRadarConf` protobuf 加载：

| 字段 | 类型 | 说明 |
|------|------|------|
| `port` | uint16 | UDP 监听端口 |
| `channel_name` | string | 点云发布通道 |
| `frame_id` | string | 坐标系 ID |
| `frame_rate` | float | 帧率，用于计算丢帧间隔 |

### 超声波雷达

通过 `UltrasonicRadarConf` protobuf 加载：

| 字段 | 类型 | 说明 |
|------|------|------|
| `can_conf` | SensorCanbusConf | CAN 配置 |
| `entrance_num` | int | 探头数量 |

## CAN 协议

### 大陆 / 赛恩领动

| CAN ID | 报文 | 方向 | 说明 |
|--------|------|------|------|
| 0x200 | RadarConfig200 | 发送 | 雷达参数配置 |
| 0x201 | RadarState201 | 接收 | 雷达状态反馈 |
| 0x300 | MotionInputSpeed300 | 发送 | 自车速度馈入 |
| 0x301 | MotionInputYawRate301 | 发送 | 横摆角速度馈入 |
| 0x600 | ClusterListStatus600 | 接收 | 目标簇列表状态 |
| 0x60A | ObjectListStatus60A | 接收 | 目标列表状态 |
| 0x60B | ObjectGeneralInfo60B | 接收 | 目标通用信息 |
| 0x60C | ObjectQualityInfo60C | 接收 | 目标质量信息 |
| 0x60D | ObjectExtendedInfo60D | 接收 | 目标扩展信息 |
| 0x701 | ClusterGeneralInfo701 | 接收 | 目标簇通用信息 |
| 0x702 | ClusterQualityInfo702 | 接收 | 目标簇质量信息 |

### 纳瓦

在大陆协议基础上增加：

| CAN ID | 报文 | 方向 | 说明 |
|--------|------|------|------|
| 0x401 | RegionConfig401 | 发送 | 检测区域配置 |
| 0x402 | RegionState402 | 接收 | 区域状态反馈 |
| 0x700 | SoftwareVersion700 | 接收 | 软件版本信息 |

## 调用关系

- **上游**：CAN 总线硬件（CAN 类雷达）、UDP 网络接口（Oculii 雷达）、定位模块（大陆雷达运动补偿）
- **下游**：发布 `ContiRadar` / `NanoRadar` / `RacobitRadar` / `OculiiPointCloud` / `Ultrasonic` protobuf 消息到 CyberRT 通道
- **依赖**：`drivers/canbus`（CAN 基础设施）、`drivers/canbus/can_client`（CAN 卡硬件抽象）、CyberRT 组件框架
- **被依赖**：感知模块中的雷达检测和融合组件订阅雷达输出话题
