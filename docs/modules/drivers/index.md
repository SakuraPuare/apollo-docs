# Drivers 驱动模块

## 模块职责

`modules/drivers/` 是 Apollo 自动驾驶平台的传感器驱动层，负责与各类硬件传感器通信，将原始硬件数据采集、解析并转换为 Cyber RT 标准消息，供感知、定位等上层模块消费。该模块覆盖了自动驾驶所需的全部传感器类型，包括相机、激光雷达、毫米波雷达、GNSS/IMU、CAN 总线、麦克风、视频流等。

## 目录结构总览

```
modules/drivers/
├── camera/                          # USB 相机驱动
├── canbus/                          # CAN 总线通信抽象层
├── gnss/                            # GNSS/IMU 定位设备驱动
├── lidar/                           # 激光雷达驱动（多品牌）
│   ├── common/                      #   公共基类与工厂
│   ├── compensator/                 #   运动补偿
│   ├── fusion/                      #   多雷达点云融合
│   ├── hslidar/                     #   禾赛 (Hesai) 驱动
│   ├── livox/                       #   览沃 (Livox) 驱动
│   ├── lslidar/                     #   镭神 (Leishen) 驱动
│   ├── rslidar/                     #   速腾 (RoboSense) 驱动
│   ├── seyond/                      #   Seyond (原 Innovusion) 驱动
│   ├── vanjeelidar/                 #   万集 (Vanjee) 驱动
│   └── velodyne/                    #   Velodyne 驱动
├── lidar_fusion_and_compensator/    # 激光雷达融合+补偿一体组件
├── microphone/                      # 麦克风驱动 (ReSpeaker)
├── radar/                           # 毫米波雷达驱动
│   ├── conti_radar/                 #   Continental ARS408
│   ├── nano_radar/                  #   Nano Radar
│   ├── oculii_radar/                #   Oculii 4D 成像雷达
│   ├── racobit_radar/               #   Racobit Radar
│   └── ultrasonic_radar/            #   超声波雷达
├── smartereye/                      # SmarterEye 双目相机驱动
├── tools/                           # 工具组件（图像解压等）
└── video/                           # H.265 视频流驱动
```

## 核心设计模式

### Cyber RT Component 模式

所有驱动均以 Cyber RT Component 形式实现，继承 `apollo::cyber::Component<>` 基类，通过 `CYBER_REGISTER_COMPONENT` 宏注册为可动态加载的共享库。每个 Component 实现 `Init()` 方法完成初始化，并在内部启动异步线程持续采集数据，通过 `cyber::Writer` 将消息发布到对应 Channel。

典型的 Component 生命周期：

```
Init() → 加载 proto 配置 → 初始化硬件设备 → 创建 Writer → 启动异步采集线程 → 循环 poll/read → 填充 protobuf 消息 → Writer::Write() 发布
```

### 工厂模式 (Factory Pattern)

模块中有两处典型的工厂模式应用：

**CAN 客户端工厂** (`canbus/can_client/can_client_factory.h`)：
- `CanClientFactory` 继承 `apollo::common::util::Factory`，以 `CANCardParameter::CANCardBrand` 为键
- 支持的 CAN 卡品牌：ESD CAN、Hermes CAN、Socket CAN、Fake CAN（测试用）
- 通过 `RegisterCanClients()` 注册所有品牌，`CreateCANClient()` 按配置创建实例

**LiDAR 驱动工厂** (`lidar/common/driver_factory/lidar_driver_factory.h`)：
- `LidarDriverFactory` 继承 `apollo::common::util::Factory`，以 `LidarParameter::LidarBrand` 为键
- 通过 `RegisterLidarClients()` 注册所有品牌，`CreateLidarDriver()` 按配置创建实例

### 抽象基类体系

#### CAN 总线抽象层

```
CanClient (抽象基类)
├── Init(CANCardParameter) → bool
├── Start() → ErrorCode
├── Stop()
├── Send(vector<CanFrame>, frame_num) → ErrorCode
├── Receive(vector<CanFrame>*, frame_num) → ErrorCode
└── GetErrorString(status) → string

实现类：
├── EsdCanClient          # ESD CAN 卡
├── HermesCanClient       # Hermes CAN 卡
├── SocketCanClientRaw    # Linux SocketCAN
└── FakeCanClient         # 测试用假客户端
```

`ProtocolData<SensorType>` 是 CAN 协议数据的基类模板，定义了 `Parse()`（解析接收数据）和 `UpdateData()`（更新发送数据）等虚方法。

`MessageManager<SensorType>` 管理所有协议数据实例，根据 CAN 帧 ID 分发到对应的 `ProtocolData` 进行解析，并维护聚合后的传感器数据对象。

`CanReceiver<SensorType>` 在独立线程中循环调用 `CanClient::Receive()`，将收到的帧交给 `MessageManager` 解析。

`CanSender<SensorType>` 在独立线程中按周期调用 `CanClient::Send()`，发送控制帧。

`SensorCanbus<SensorType>` 是基于 CAN 总线的传感器 Component 模板基类，仅封装了 `CanClient`、`CanReceiver`、`MessageManager` 的完整初始化和运行流程，不包含 `CanSender`。`CanSender` 是独立组件，用于车辆控制等其他场景（如 `ContiRadarCanbusComponent` 直接使用）。

#### LiDAR 驱动抽象层

```
LidarDriver (抽象基类)
└── Init() → bool (纯虚)

LidarComponentBaseImpl<ScanType, ComponentType>
├── InitConverter()       # 初始化点云 Writer
├── InitPacket()          # 初始化扫描包 Reader/Writer
├── ReadScanCallback()    # 扫描包回调（纯虚）
├── WriteScan()           # 发布扫描包
├── AllocatePointCloud()  # 分配点云内存
└── WritePointCloud()     # 发布点云消息

LidarComponentBase<ScanType>
└── 继承 LidarComponentBaseImpl，提供 InitBase() 默认实现
```

各品牌 LiDAR 驱动（Hesai、Velodyne、Livox、Leishen、RoboSense 等）继承上述基类，实现各自的数据包解析和点云转换逻辑。

#### GNSS 抽象层

```
Stream (抽象基类) — 通信流
├── Connect() / Disconnect()
├── read(buffer, max_length) → size_t
├── write(buffer, length) → size_t
│
├── TcpStream
├── UdpStream
├── SerialStream
├── NtripStream
└── CanStream

Parser (抽象基类) — 数据解析器
├── Update(data, length)
├── GetMessage(message_ptr) → MessageType
├── CreateParser(config) — 静态工厂方法
│   ├── CreateNovatel()
│   ├── CreateHuaCeText()
│   ├── CreateAsensingBinary()
│   ├── CreateAsensingCan()
│   └── CreateBroadGnssText()
└── CreateRtcmV3()
```

## 各传感器驱动详解

### Camera 相机驱动

**核心文件：** `camera/camera_component.h`, `camera/usb_cam.h`, `camera/compress_component.h`

**组件：**
- `CameraComponent` — 主采集组件，通过 V4L2 接口从 USB 摄像头采集图像
- `CompressComponent` — 图像压缩组件，订阅原始图像并输出 JPEG 压缩图像

**数据流：**
```
USB 摄像头 (V4L2)
  → UsbCam::poll() 获取原始帧
    → CameraComponent::run() 异步线程循环采集
      → 填充 apollo::drivers::Image protobuf
        → writer_->Write() 发布到 channel (如 /apollo/sensor/camera/front_6mm/image)
        → raw_writer_->Write() 发布原始格式图像
          → CompressComponent::Proc() 订阅并压缩
            → 发布 CompressedImage
```

`UsbCam` 类封装了 Linux V4L2 设备操作，支持 YUYV、MJPEG、RGB24 等像素格式，内部使用 FFmpeg (libavcodec/libswscale) 进行 MJPEG 解码和颜色空间转换。在 ARM (aarch64) 平台上支持 CUDA 加速和 NvJPEG 硬件编码。

### LiDAR 激光雷达驱动

**核心文件：** `lidar/lidar_driver_component.h`, `lidar/common/lidar_component_base.h`, `lidar/common/driver_factory/`

**支持品牌：**

| 子目录 | 品牌 | 型号 |
|--------|------|------|
| `hslidar/` | 禾赛 (Hesai) | Pandar 系列 |
| `velodyne/` | Velodyne | VLP-16, VLP-32, HDL-64E, VLS-128 |
| `livox/` | 览沃 (Livox) | Mid-40/70, Horizon 等 |
| `lslidar/` | 镭神 (Leishen) | CH16/32/64/120/128, LS128S2, CXV4 等 |
| `rslidar/` | 速腾 (RoboSense) | RS 系列 |
| `seyond/` | Seyond (原 Innovusion) | Seyond 系列 |
| `vanjeelidar/` | 万集 (Vanjee) | Vanjee 系列 |

**数据流：**
```
LiDAR 硬件 (UDP 数据包)
  → 品牌驱动 (如 HesaiComponent) 接收 UDP 包
    → 解析为 ScanType (扫描包 protobuf)
      → WriteScan() 发布扫描包
    → 解析为 PointCloud (点云 protobuf)
      → WritePointCloud() 发布点云
```

**辅助组件：**
- `Compensator` — 运动补偿组件，利用 TF2 变换对点云进行运动畸变校正
- `PriSecFusionComponent` — 多 LiDAR 点云融合组件，将副雷达点云变换到主雷达坐标系后合并
- `FusionAndCompensatorComponent` — 融合+补偿一体化组件

### Radar 毫米波雷达驱动

**核心文件：** `radar/conti_radar/conti_radar_canbus_component.h`, `radar/oculii_radar/oculii_radar_component.h`

**支持型号：**

| 子目录 | 型号 | 通信方式 |
|--------|------|----------|
| `conti_radar/` | Continental ARS408 | CAN 总线 |
| `nano_radar/` | Nano Radar | CAN 总线 |
| `racobit_radar/` | Racobit Radar | CAN 总线 |
| `ultrasonic_radar/` | 超声波雷达 | CAN 总线 |
| `oculii_radar/` | Oculii 4D 成像雷达 | UDP |

**CAN 总线雷达数据流（以 Continental 为例）：**
```
雷达硬件 → CAN 总线
  → CanClient::Receive() 接收 CAN 帧
    → ContiRadarMessageManager 按帧 ID 分发
      → 各 ProtocolData 子类解析 (cluster_info, object_info 等)
        → 聚合为 ContiRadar protobuf
          → conti_radar_writer_->Write() 发布
```

Continental 雷达同时订阅车辆定位信息 (`LocalizationEstimate`)，通过 CAN 发送车速和横摆角速度给雷达，用于雷达内部的运动补偿。

**UDP 雷达数据流（以 Oculii 为例）：**
```
Oculii 雷达 → UDP 数据包
  → OculiiRadarUdpParser 解析
    → OculiiPointCloud protobuf
      → writer_->Write() 发布
```

### GNSS/IMU 定位设备驱动

**核心文件：** `gnss/gnss_component.h`, `gnss/stream/raw_stream.h`, `gnss/parser/parser.h`, `gnss/parser/data_parser.h`

**支持设备协议：**
- NovAtel (SPAN 系列，二进制协议)
- 华测 (HuaCe，文本协议)
- 导远 (Asensing，二进制/CAN 协议)
- 华大北斗 (BroadGNSS，文本协议)
- RTCM v3 差分数据

**数据流：**
```
GNSS/IMU 硬件
  → Stream (Serial/TCP/UDP/NTRIP/CAN) 读取原始字节流
    → RawStream 管理多路数据流 (data/command/rtk_from/rtk_to)
      → DataParser 调用 Parser 解析原始数据
        → 按消息类型分发并发布：
           ├── Gps (定位)        → /apollo/sensor/gnss/odometry
           ├── Imu (原始IMU)     → /apollo/sensor/gnss/imu
           ├── CorrectedImu      → /apollo/sensor/gnss/corrected_imu
           ├── InsStat (INS状态) → /apollo/sensor/gnss/ins_stat
           ├── GnssBestPose      → /apollo/sensor/gnss/best_pose
           ├── Heading (航向)    → /apollo/sensor/gnss/heading
           ├── GnssEphemeris     → /apollo/sensor/gnss/rtk_eph
           └── EpochObservation  → /apollo/sensor/gnss/rtk_obs
      → RtcmParser 解析 RTCM 差分数据并转发
```

`RawStream` 是 GNSS 驱动的核心编排类，管理四路 Stream：
- `data_stream_` — 主数据流，读取 GNSS/IMU 观测数据
- `command_stream_` — 命令流，发送配置指令
- `in_rtk_stream_` — RTK 输入流，从 NTRIP Caster 获取差分改正数据
- `out_rtk_stream_` — RTK 输出流，将差分数据转发给接收机

`DataParser` 负责将解析后的 protobuf 消息发布到对应的 Cyber RT Channel，同时维护 GNSS 和 INS 的状态监控，并通过 `TransformBroadcaster` 发布 TF 变换。

### CAN 总线通信层

**核心文件：** `canbus/can_client/can_client.h`, `canbus/can_comm/message_manager.h`, `canbus/sensor_canbus.h`

CAN 总线模块不是一个独立的传感器驱动，而是为所有基于 CAN 通信的设备（雷达、超声波等）提供的公共通信抽象层。

**核心类关系：**
```
CanClientFactory (单例工厂)
  → 创建 CanClient 实例

CanClient (通信层)
  ↕ CAN 帧收发
CanReceiver (接收线程)
  → MessageManager (协议分发)
    → ProtocolData (协议解析)

CanSender (发送线程，独立组件，不属于 SensorCanbus)
  ← MessageManager (协议组装)
    ← ProtocolData (数据更新)

SensorCanbus<SensorType> (Component 模板)
  组合 CanClient、CanReceiver、MessageManager，提供 CAN 传感器驱动框架（不含 CanSender）
```

`CanFrame` 结构体定义了标准 CAN 帧格式：ID (32位)、长度 (8位)、数据 (8字节)、时间戳。

### Microphone 麦克风驱动

**核心文件：** `microphone/microphone_component.h`, `microphone/respeaker.h`

驱动 ReSpeaker 麦克风阵列，采集多通道音频数据，发布 `AudioData` protobuf 消息。采集线程循环读取音频 chunk，按通道拆分后填充到 protobuf 并发布。

### SmarterEye 双目相机驱动

**核心文件：** `smartereye/smartereye_component.h`, `smartereye/smartereye_device.h`

驱动 SmarterEye 双目立体相机，除了输出图像 (`Image`) 外，还输出相机内置算法的障碍物检测结果 (`SmartereyeObstacles`) 和车道线检测结果 (`SmartereyeLanemark`)。同时提供 `CompressComponent` 用于图像压缩。

### Video 视频流驱动

**核心文件：** `video/video_driver_component.h`, `video/driver.h`, `video/socket_input.h`

通过 Socket 接收 H.265 编码的视频流，解码后发布 `CompressedImage` 消息。`tools/decode_video/` 子目录提供离线 H.265 解码工具，可将视频文件转换为 JPEG 图片序列。

### Tools 工具组件

**核心文件：** `tools/image_decompress/image_decompress.h`

`ImageDecompressComponent` 订阅 `CompressedImage` 消息，解压后发布 `Image` 消息，是 `CompressComponent` 的逆操作。

## 配置方式

### 三层配置体系

Apollo 驱动模块采用三层配置体系：

**1. Launch 文件 (XML)**

定义进程级别的模块加载，指定 DAG 文件路径和进程名：

```xml
<!-- camera/launch/camera.launch -->
<cyber>
    <module>
        <name>camera</name>
        <dag_conf>/apollo/modules/drivers/camera/dag/camera.dag</dag_conf>
        <process_name>usb_cam</process_name>
    </module>
</cyber>
```

**2. DAG 文件 (protobuf text)**

定义 Component 的加载方式，指定共享库路径、类名和配置文件：

```protobuf
# camera/dag/camera.dag
module_config {
    module_library : "modules/drivers/camera/libcamera_component.so"
    components {
      class_name : "CameraComponent"
      config {
        name : "camera_front_6mm"
        config_file_path : "/apollo/modules/drivers/camera/conf/camera_front_6mm.pb.txt"
      }
    }
}
```

同一个 DAG 文件中可以定义多个 Component 实例（如多个摄像头），共享同一个进程。

**3. Proto 配置文件 (.proto / .pb.txt)**

定义传感器的具体参数，每种传感器有独立的 proto 定义：

| 传感器 | Proto 定义文件 | 关键配置项 |
|--------|---------------|-----------|
| Camera | `camera/proto/config.proto` | 设备路径、分辨率、像素格式、帧率、曝光、Channel 名 |
| LiDAR | `lidar/proto/config.proto` | 品牌选择、品牌专属配置 (Hesai/Velodyne) |
| LiDAR (通用) | `lidar/common/proto/lidar_config_base.proto` | 数据源类型、扫描包/点云 Channel、frame_id |
| GNSS | `gnss/proto/config.proto` | 数据流配置 (Serial/TCP/UDP/NTRIP)、协议格式、IMU 类型、RTK 配置 |
| Conti Radar | `radar/conti_radar/proto/conti_radar_conf.proto` | CAN 卡参数、Channel 名 |
| Oculii Radar | `radar/oculii_radar/proto/oculii_radar_conf.proto` | UDP 地址端口、Channel 名 |
| Microphone | `microphone/proto/microphone_config.proto` | 采样率、通道数、chunk 大小 |
| Video | `video/proto/video_h265cfg.proto` | Socket 地址、编码参数 |

### GNSS 配置示例

GNSS 的配置最为复杂，支持多路数据流的灵活组合：

```protobuf
# gnss/proto/config.proto 中的 Config 消息
message Config {
  optional Stream data = 1;           # 主数据流 (必选)
  optional Stream command = 2;        # 命令流 (可选，默认复用 data)
  optional Stream rtk_from = 3;       # RTK 差分输入 (通常为 NTRIP)
  optional Stream rtk_to = 4;         # RTK 差分输出 (可选)
  repeated bytes login_commands = 5;  # 登录命令序列
  repeated bytes logout_commands = 6; # 登出命令序列
  oneof device_config {               # 设备配置
    NovatelConfig novatel_config = 7;
    UbloxConfig ublox_config = 8;
  }
  optional RtkSolutionType rtk_solution_type = 9; # RTK 方案类型
  optional ImuType imu_type = 10;     # IMU 型号
  optional string proj4_text = 11;    # 坐标投影参数
  optional TF tf = 12;               # TF 变换配置
  optional string wheel_parameters = 13;  # 轮速参数
  optional string gpsbin_folder = 14;     # GPS bin 文件夹
  optional bool use_gnss_time = 15;       # 是否使用 GNSS 时间
  optional bool auto_fill_gps_msg = 16;   # 是否自动填充 GPS 消息
}
```

每路 Stream 支持 Serial、TCP、UDP、NTRIP、CAN 五种传输方式，通过 `oneof type` 选择。

## 关键 Protobuf 消息类型

驱动模块输出的核心消息定义在 `modules/common_msgs/sensor_msgs/` 中：

| 消息类型 | 说明 | 典型 Channel |
|---------|------|-------------|
| `Image` | 原始图像 | `/apollo/sensor/camera/*/image` |
| `CompressedImage` | 压缩图像 | `/apollo/sensor/camera/*/image/compressed` |
| `PointCloud` | 3D 点云 | `/apollo/sensor/lidar/*/pointcloud` |
| `ContiRadar` | Continental 雷达目标 | `/apollo/sensor/conti_radar` |
| `OculiiPointCloud` | Oculii 4D 雷达点云 | `/apollo/sensor/oculii_radar` |
| `Gnss` | GNSS 定位 | `/apollo/sensor/gnss/odometry` |
| `Imu` | IMU 数据 | `/apollo/sensor/gnss/imu` |
| `CorrectedImu` | 校正后 IMU | `/apollo/sensor/gnss/corrected_imu` |
| `AudioData` | 音频数据 | `/apollo/sensor/microphone` |
