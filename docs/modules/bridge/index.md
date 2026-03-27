# Bridge 模块

## 模块职责

Bridge 模块负责 Apollo 自动驾驶系统与外部进程之间的网络通信桥接。它通过 UDP Socket 实现 Apollo 内部 Cyber RT 消息与外部系统之间的双向数据传输，使得 Apollo 之外的程序（如远程调试工具、外部仿真器、第三方控制系统等）能够与 Apollo 各模块进行数据交互。

模块位于 `modules/bridge/`，命名空间为 `apollo::bridge`。

## 目录结构

```
modules/bridge/
├── BUILD                                    # Bazel 构建文件
├── cyberfile.xml                            # 包描述文件
├── README.md                                # 原始英文文档
├── udp_bridge_sender_component.h/.cc        # UDP 发送组件
├── udp_bridge_receiver_component.h/.cc      # UDP 接收组件（模板化，单消息类型）
├── udp_bridge_multi_receiver_component.h/.cc # UDP 多消息类型接收组件
├── udp_bridge_component_test.cc             # 组件单元测试
├── common/                                  # 公共基础库
│   ├── bridge_header.h/.cc                  # 桥接协议头定义与序列化
│   ├── bridge_header_item.h                 # 协议头字段项模板
│   ├── bridge_proto_serialized_buf.h        # Protobuf 序列化缓冲区（发送端）
│   ├── bridge_proto_diserialized_buf.h      # Protobuf 反序列化缓冲区（接收端）
│   ├── bridge_proto_diser_buf_factory.h     # 反序列化缓冲区工厂（多接收器用）
│   ├── bridge_buffer.h/.cc                  # 通用缓冲区封装
│   ├── bridge_gflags.h/.cc                  # GFlags 参数定义
│   ├── udp_listener.h                       # UDP 监听器（epoll 模型）
│   ├── macro.h                              # 宏定义（FRAME_SIZE、内存释放）
│   └── util.h/.cc                           # 工具函数
├── conf/                                    # 配置文件
│   ├── bridge.conf                          # 全局 GFlags 配置
│   ├── udp_bridge_sender_adctrajectory.pb.txt
│   ├── udp_bridge_sender_localization.pb.txt
│   └── udp_bridge_receiver_chassis.pb.txt
├── dag/                                     # DAG 调度文件
│   ├── bridge_sender.dag
│   ├── bridge_receiver.dag
│   └── bridge_multi_receiver.dag
├── launch/                                  # 启动文件
│   ├── bridge_sender.launch
│   ├── bridge_receiver.launch
│   └── bridge_multi_receiver.launch
├── proto/                                   # Protobuf 定义
│   └── udp_bridge_remote_info.proto
└── test/                                    # 集成测试
    ├── bridge_sender_test.cc
    ├── bridge_receiver_test.cc
    └── BUILD
```

## 桥接协议

### 通信方式

Bridge 模块唯一使用的传输协议是 **UDP（User Datagram Protocol）**。选择 UDP 的原因是其低延迟、无连接的特性，适合自动驾驶场景中对实时性要求较高的数据传输。

发送端使用 `SOCK_DGRAM | SOCK_NONBLOCK` 创建非阻塞 UDP Socket，接收端通过 Linux `epoll` 机制实现高效的事件驱动监听。

### 自定义分帧协议

由于 UDP 单包大小有限，Bridge 模块实现了一套自定义的分帧传输协议，将大型 Protobuf 消息拆分为多个 UDP 帧进行传输。

每个 UDP 帧的结构如下：

```
+---------------------+-------------------+------------------+
|   BridgeHeader      |   Header Body     |   Payload Data   |
+---------------------+-------------------+------------------+
```

**帧头标识**：每个帧以固定字符串 `"ApolloBridgeHeader"` 开头，用于校验帧的合法性。

**Header Body** 包含以下字段（定义于 `HType` 枚举）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `Header_Ver` | `uint32_t` | 协议版本号 |
| `Msg_Name` | `string` | Protobuf 消息名称（如 `"ADCTrajectory"`） |
| `Msg_ID` | `uint32_t` | 消息序列号（来自 `header().sequence_num()`） |
| `Msg_Size` | `uint32_t` | 完整消息的总字节数 |
| `Msg_Frames` | `uint32_t` | 消息被拆分的总帧数 |
| `Frame_Size` | `uint32_t` | 当前帧的数据载荷大小 |
| `Frame_Pos` | `uint32_t` | 当前帧数据在完整消息中的偏移位置 |
| `Frame_Index` | `uint32_t` | 当前帧的索引号 |
| `Time_Stamp` | `double` | 消息时间戳 |

每帧的最大数据载荷为 `FRAME_SIZE = 1024` 字节。发送端将 Protobuf 消息序列化后按 1024 字节切片，每片附加独立的 BridgeHeader 后通过 UDP 发送。

## 核心类与接口

### 发送端

#### `UDPBridgeSenderComponent<T>`

模板化的 Cyber RT Component，继承自 `cyber::Component<T>`。订阅指定的 Cyber 通道消息，通过 UDP 发送到远端。

- `Init()`：从 Protobuf 配置文件读取 `remote_ip`、`remote_port`、`proto_name`
- `Proc(const std::shared_ptr<T> &pb_msg)`：每次收到消息时触发，创建 UDP Socket，将消息序列化并分帧发送

已注册的消息类型：
- `planning::ADCTrajectory` — 规划轨迹
- `localization::LocalizationEstimate` — 定位估计

#### `BridgeProtoSerializedBuf<T>`

发送端的序列化缓冲区。`Serialize()` 方法将 Protobuf 消息序列化为字节数组，按 `FRAME_SIZE` 切分为多帧，每帧附加 `BridgeHeader`。

### 接收端

#### `UDPBridgeReceiverComponent<T>`

模板化的 Cyber RT Component，继承自 `cyber::Component<>`（无输入通道）。监听指定 UDP 端口，接收数据后反序列化为 Protobuf 消息并发布到 Cyber 通道。

- `Init()`：从配置读取 `bind_port`、`proto_name`、`topic_name`、`enable_timeout`，创建 Cyber Writer，初始化 UDP 监听
- `MsgHandle(int fd)`：处理收到的 UDP 数据包，解析 BridgeHeader，将帧数据写入对应的反序列化缓冲区
- `IsTimeout(double time_stamp)`：基于 `FLAGS_timeout` 判断消息是否超时

已注册的消息类型：
- `canbus::Chassis` — 底盘信息

#### `UDPBridgeMultiReceiverComponent`

非模板化的接收组件，支持在同一端口接收多种不同类型的 Protobuf 消息。通过 `ProtoDiserializedBufBaseFactory` 工厂类根据消息名称动态创建对应的反序列化缓冲区。

当前工厂支持的消息类型：
- `"Chassis"` → `canbus::Chassis`（发布到 `FLAGS_chassis_topic` 通道）

#### `BridgeProtoDiserializedBuf<T>`

接收端的反序列化缓冲区。维护一个与完整消息等大的 `char*` 缓冲区和一个位图状态列表 `status_list_`，用于跟踪各帧的接收状态。当所有帧都收齐后（`IsReadyDiserialize()` 返回 `true`），调用 `Diserialized()` 将字节数组解析为 Protobuf 对象。

### 网络层

#### `UDPListener<T>`

基于 Linux epoll 的 UDP 监听器模板类。

- `Initialize()`：创建 UDP Socket，绑定端口，设置非阻塞模式，注册到 epoll
- `Listen()`：进入 epoll 事件循环，每当有数据到达时创建 detached pthread 调用注册的回调函数处理消息

#### `BridgeHeader`

封装帧头的序列化与反序列化逻辑。内部使用 `HeaderItem<HType, T>` 模板存储各字段，支持类型安全的二进制序列化。

## 数据流

### 发送流程

```
Apollo 内部模块
    │
    ▼ (Cyber RT Channel)
UDPBridgeSenderComponent<T>::Proc()
    │
    ├─ 序列化 Protobuf 为字节数组
    ├─ 按 FRAME_SIZE(1024B) 分帧
    ├─ 每帧附加 BridgeHeader
    │
    ▼ (UDP Socket send)
外部系统
```

具体步骤：
1. Sender 组件订阅 Cyber RT 通道（如 `/apollo/planning`）
2. 收到消息后，`BridgeProtoSerializedBuf<T>::Serialize()` 将 Protobuf 序列化为二进制
3. 按 1024 字节切分，为每帧构造 `BridgeHeader`（包含消息名、序列号、帧索引等）
4. 创建 UDP Socket，连接到配置的远端 IP:Port
5. 逐帧通过 `send()` 发送

### 接收流程

```
外部系统
    │
    ▼ (UDP Socket recvfrom)
UDPListener::Listen() [epoll 事件循环]
    │
    ▼ (pthread 回调)
UDPBridgeReceiverComponent<T>::MsgHandle()
    │
    ├─ 校验 BridgeHeader 标识
    ├─ 反序列化帧头
    ├─ 将帧数据写入 BridgeProtoDiserializedBuf
    ├─ 更新帧接收状态位图
    ├─ 所有帧收齐后反序列化为 Protobuf
    │
    ▼ (Cyber RT Writer)
Apollo 内部模块
```

具体步骤：
1. `UDPListener` 通过 epoll 监听绑定端口
2. 收到 UDP 包后，在新线程中调用 `MsgHandle()`
3. 校验帧头标识 `"ApolloBridgeHeader"`，解析 `BridgeHeader`
4. 根据消息名和序列号查找或创建 `BridgeProtoDiserializedBuf`
5. 将帧数据 `memcpy` 到缓冲区的对应偏移位置
6. 更新位图状态，当所有帧收齐后调用 `ParseFromArray()` 反序列化
7. 通过 `cyber::Writer` 将消息发布到 Cyber RT 通道

### 默认数据流实例

| 方向 | 消息类型 | Cyber 通道 | 远端地址 |
|------|----------|-----------|---------|
| 发送 | `planning::ADCTrajectory` | `/apollo/planning` | `127.0.0.1:8900` |
| 发送 | `localization::LocalizationEstimate` | `/apollo/localization/pose` | `127.0.0.1:8901` |
| 接收 | `canbus::Chassis` | `/apollo/canbus/Chassis` | 绑定端口 `8900` |

## 配置方式

Bridge 模块的配置分为三层：GFlags 全局参数、Protobuf 组件配置、DAG/Launch 启动配置。

### GFlags 全局参数

文件：`conf/bridge.conf`

```
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--timeout=10000.0
```

- `bridge_module_name`：模块名称，默认 `"Bridge"`
- `timeout`：消息超时时间（秒），默认 `1.0`，配置文件中覆盖为 `10000.0`

### Protobuf 组件配置

配置消息定义于 `proto/udp_bridge_remote_info.proto`：

**发送端配置** (`UDPBridgeSenderRemoteInfo`)：

```protobuf
message UDPBridgeSenderRemoteInfo {
  optional string remote_ip = 1 [default = "127.0.0.1"];
  optional int32 remote_port = 2 [default = 8900];
  optional string proto_name = 3 [default = "ProtoMsgName"];
}
```

示例（`conf/udp_bridge_sender_adctrajectory.pb.txt`）：

```
remote_ip: "127.0.0.1"
remote_port: 8900
proto_name: "ADCTrajectory"
```

**接收端配置** (`UDPBridgeReceiverRemoteInfo`)：

```protobuf
message UDPBridgeReceiverRemoteInfo {
  optional string topic_name = 1 [default = ""];
  optional int32 bind_port = 2 [default = 8500];
  optional string proto_name = 3 [default = "ProtoMsgName"];
  optional bool enable_timeout = 4 [default = true];
}
```

示例（`conf/udp_bridge_receiver_chassis.pb.txt`）：

```
topic_name: "/apollo/canbus/Chassis"
bind_port: 8900
proto_name: "Chassis"
enable_timeout: false
```

### DAG 调度配置

DAG 文件定义组件的加载方式和参数来源。

**发送端** (`dag/bridge_sender.dag`)：加载 `libudp_bridge_sender_component.so`，包含两个组件实例：
- `UDPBridgeSenderComponent<planning::ADCTrajectory>`，订阅 `/apollo/planning`
- `UDPBridgeSenderComponent<localization::LocalizationEstimate>`，订阅 `/apollo/localization/pose`

**接收端** (`dag/bridge_receiver.dag`)：加载 `libudp_bridge_receiver_component.so`，包含：
- `UDPBridgeReceiverComponent<canbus::Chassis>`

**多消息接收端** (`dag/bridge_multi_receiver.dag`)：加载 `libudp_bridge_multi_receiver_component.so`

### 启动命令

```bash
# 启动发送端
cyber_launch start /apollo/modules/bridge/launch/bridge_sender.launch

# 启动接收端
cyber_launch start /apollo/modules/bridge/launch/bridge_receiver.launch

# 启动多消息接收端
cyber_launch start /apollo/modules/bridge/launch/bridge_multi_receiver.launch
```

## 扩展新消息类型

如需通过 Bridge 发送/接收新的 Protobuf 消息类型，需要：

1. 在 `udp_bridge_sender_component.h` 中添加宏注册：
   ```cpp
   BRIDGE_COMPONENT_REGISTER(your_namespace::YourProtoMsg)
   ```

2. 在 `udp_bridge_sender_component.cc` 中添加模板实例化：
   ```cpp
   BRIDGE_IMPL(your_namespace::YourProtoMsg);
   ```

3. 在 `udp_bridge_receiver_component.h` 中添加接收端注册：
   ```cpp
   RECEIVER_BRIDGE_COMPONENT_REGISTER(your_namespace::YourProtoMsg)
   ```

4. 在 `udp_bridge_receiver_component.cc` 中添加模板实例化：
   ```cpp
   BRIDGE_RECV_IMPL(your_namespace::YourProtoMsg);
   ```

5. 如使用 `UDPBridgeMultiReceiverComponent`，还需在 `bridge_proto_diser_buf_factory.h` 的工厂方法中添加对应的消息名称映射。

6. 添加对应的 `.pb.txt` 配置文件、DAG 文件和 Launch 文件。

7. 重新编译代码后启动。
