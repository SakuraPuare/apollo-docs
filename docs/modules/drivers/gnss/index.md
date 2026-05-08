---
title: "GNSS 驱动"
---

# GNSS 驱动

> 源码路径: `modules/drivers/gnss/`

## 概述

GNSS 驱动模块负责从 GNSS 接收机读取原始卫星导航数据，并将解析后的定位信息发布到 CyberRT 通信总线。该模块作为 CyberRT Component 运行，支持多种 GNSS 接收机协议（Novatel、uBlox、Huace、Asensing、BroadGNSS）及多种连接方式（串口、TCP、UDP、NTRIP、CAN）。同时支持 RTK 差分修正数据的收发，可对接 NTRIP 服务获取实时差分改正数，实现厘米级定位精度。

## 核心类

### GnssDriverComponent

继承自 `apollo::cyber::Component<>`，是 GNSS 驱动的 CyberRT 入口组件。负责加载配置文件并启动 `RawStream` 实例。

```cpp
class GnssDriverComponent : public Component<> {
 public:
  GnssDriverComponent();
  bool Init() override;

 private:
  std::unique_ptr<RawStream> raw_stream_ = nullptr;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(GnssDriverComponent)
```

| 成员 | 说明 |
|------|------|
| `raw_stream_` | GNSS 原始数据流管理器，封装了所有流连接和数据解析逻辑 |
| `monitor_logger_buffer_` | 监控日志缓冲器，类别为 `MonitorMessageItem::GNSS` |

### RawStream

GNSS 数据流核心管理类，负责建立与 GNSS 接收机的连接、接收原始数据、发送 RTK 修正数据，并协调多个解析器完成数据解析与发布。

```cpp
class RawStream {
 public:
  RawStream(const config::Config& config,
            const std::shared_ptr<apollo::cyber::Node>& node);
  ~RawStream();
  bool Init();
  void Start();

 private:
  void DataSpin();
  void RtkSpin();
  bool Connect();
  bool Disconnect();
  bool Login();
  bool Logout();
  // ...
};
```

| 成员 | 说明 |
|------|------|
| `data_stream_` / `command_stream_` | 数据流和命令流连接 |
| `in_rtk_stream_` / `out_rtk_stream_` | RTK 差分数据的输入/输出流 |
| `data_parser_ptr_` | GNSS 原始数据解析器 |
| `rtcm_parser_ptr_` | RTCM 差分修正数据解析器 |
| `data_thread_ptr_` / `rtk_thread_ptr_` | 数据接收和 RTK 处理的工作线程 |
| `raw_writer_` / `rtcm_writer_` | 原始数据和 RTCM 数据的 CyberRT Writer |
| `stream_writer_` | 流状态信息的 CyberRT Writer |

## 核心函数

### GnssDriverComponent::Init()

组件初始化入口。从配置文件加载 `config::Config`，创建 `RawStream` 并调用其 `Init()` 和 `Start()` 方法。

```cpp
bool GnssDriverComponent::Init() {
  config::Config gnss_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &gnss_config)) {
    monitor_logger_buffer_.ERROR("Unable to load gnss conf file: " +
                                 config_file_path_);
    return false;
  }
  raw_stream_.reset(new RawStream(gnss_config, node_));
  if (!raw_stream_->Init()) {
    monitor_logger_buffer_.ERROR("Init gnss stream failed");
    return false;
  }
  raw_stream_->Start();
  return true;
}
```

### RawStream::Init()

初始化所有通信流（数据流、命令流、RTK 流），创建 `DataParser` 和 `RtcmParser` 实例，建立 CyberRT Writer/Reader 通道。

### RawStream::Start()

启动数据接收线程 `DataSpin` 和 RTK 处理线程 `RtkSpin`，开始持续读取和处理 GNSS 数据。

### RawStream::DataSpin() / RtkSpin()

分别在独立线程中运行：`DataSpin` 持续从数据流读取原始字节并交给 `DataParser` 解析发布；`RtkSpin` 从 NTRIP 或其他来源接收 RTK 差分修正数据并转发给接收机。

### RawStream::Connect() / Login()

建立与 GNSS 接收机的物理连接（串口/TCP/UDP），并发送登录指令序列完成接收机初始化配置。

## 配置

GNSS 驱动通过 protobuf 配置文件（`config::Config`）进行参数设定，主要配置项如下：

| 字段 | 类型 | 说明 |
|------|------|------|
| `data` | `Stream` | **必填**，数据读取端口，指定接收机连接方式和数据格式 |
| `command` | `Stream` | 命令发送端口，不配置时使用 `data` 端口 |
| `rtk_from` | `Stream` | RTK 差分数据来源，通常为 NTRIP 服务 |
| `rtk_to` | `Stream` | RTK 差分数据发送端口，不配置时使用 `data` 端口 |
| `login_commands` | `repeated bytes` | 登录命令序列 |
| `logout_commands` | `repeated bytes` | 登出命令序列 |
| `rtk_solution_type` | `RtkSolutionType` | RTK 解算方式：接收机解算 或 软件解算 |
| `imu_type` | `ImuType` | IMU 类型（如 STIM300、ISA100、ADIS16488 等） |
| `proj4_text` | `string` | 坐标投影参数（Proj4 格式） |
| `tf` | `TF` | TF 坐标变换配置 |
| `use_gnss_time` | `bool` | 是否使用 GNSS 时间戳，默认 `true` |
| `gpsbin_folder` | `string` | GPS 原始二进制数据存储目录 |

`Stream` 支持 `Serial`（串口）、`Tcp`、`Udp`、`Ntrip`、`CAN` 五种连接类型，并通过 `format` 字段指定数据格式（NMEA、Novatel Binary、uBlox Binary 等）。

## 调用关系

```text
CyberRT 框架
  └── GnssDriverComponent::Init()
        ├── GetProtoFromFile() — 加载 GNSS 配置
        ├── RawStream::Init() — 初始化流连接与解析器
        │     ├── Connect() — 建立数据/命令/RTK 流连接
        │     ├── Login() — 发送登录指令
        │     ├── DataParser 创建 — 原始数据解析器
        │     └── RtcmParser 创建 — RTCM 差分数据解析器
        └── RawStream::Start() — 启动数据处理
              ├── DataSpin 线程 — 持续读取数据并解析发布
              └── RtkSpin 线程 — 接收并转发 RTK 差分数据
```
