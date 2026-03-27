---
title: Node 模块
---

# Node 模块

## 模块职责概述

Node 是 Cyber RT 框架中最基础的通信单元。每个功能模块都通过 Node 与外部进行数据交换。一个 Node 可以同时拥有多种通信方式：

- **Channel 通信**：通过 Reader/Writer 实现发布-订阅模式
- **Service 通信**：通过 Service/Client 实现请求-响应模式

Node 在拓扑网络中必须具有唯一名称，不允许重名。

## 核心类与接口

### Node

`Node` 是面向用户的顶层类，定义于 `cyber/node/node.h`。内部将 Channel 通信委托给 `NodeChannelImpl`，Service 通信委托给 `NodeServiceImpl`。

```cpp
class Node {
public:
  // 获取节点名称（拓扑中唯一）
  const std::string& Name() const;

  // 创建 Writer（通过 RoleAttributes 或 channel 名称）
  template <typename MessageT>
  auto CreateWriter(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Writer<MessageT>>;

  template <typename MessageT>
  auto CreateWriter(const std::string& channel_name)
      -> std::shared_ptr<Writer<MessageT>>;

  // 创建 Reader（多种重载）
  template <typename MessageT>
  auto CreateReader(const std::string& channel_name,
                    const CallbackFunc<MessageT>& reader_func = nullptr)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const ReaderConfig& config,
                    const CallbackFunc<MessageT>& reader_func = nullptr)
      -> std::shared_ptr<Reader<MessageT>>;

  // 创建 Service / Client
  template <typename Request, typename Response>
  auto CreateService(const std::string& service_name,
                     const typename Service<Request, Response>::ServiceCallback&
                         service_callback)
      -> std::shared_ptr<Service<Request, Response>>;

  template <typename Request, typename Response>
  auto CreateClient(const std::string& service_name)
      -> std::shared_ptr<Client<Request, Response>>;

  // 触发所有 Reader 的 Observe，将消息从 PublishQueue 移至 ObserveQueue
  void Observe();

  // 清除所有 Reader 缓存的数据
  void ClearData();

  // 删除指定 Reader
  bool DeleteReader(const std::string& channel_name);
};
```

Node 不能直接构造，需通过框架提供的工厂函数创建：

```cpp
// 创建节点，name_space 可选
std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space = "");
```

构造时会自动初始化 `NodeChannelImpl` 和 `NodeServiceImpl` 两个内部实现对象。

### NodeChannelImpl

`NodeChannelImpl` 负责 Channel 通信对象的创建与属性填充，定义于 `cyber/node/node_channel_impl.h`。

```cpp
class NodeChannelImpl {
public:
  explicit NodeChannelImpl(const std::string& node_name);

  // 创建 Writer
  template <typename MessageT>
  auto CreateWriter(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Writer<MessageT>>;

  // 创建 Reader（支持 RoleAttributes 或 ReaderConfig）
  template <typename MessageT>
  auto CreateReader(const proto::RoleAttributes& role_attr,
                    const CallbackFunc<MessageT>& reader_func)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const ReaderConfig& config,
                    const CallbackFunc<MessageT>& reader_func)
      -> std::shared_ptr<Reader<MessageT>>;
};
```

关键行为：
- 根据运行模式（`MODE_REALITY` / `MODE_SIMULATION`）决定创建标准 Reader/Writer 还是进程内（Intra）版本
- 自动填充 `RoleAttributes` 中缺失的字段（host_name、process_id、node_id、channel_id、message_type、proto_desc、qos_profile）
- 创建后将 Writer/Reader 注册到拓扑管理器（`TopologyManager`）

### ReaderConfig

Reader 的配置结构体，定义于 `cyber/node/node_channel_impl.h`：

```cpp
struct ReaderConfig {
  std::string channel_name;              // 订阅的 channel 名称
  proto::QosProfile qos_profile;         // QoS 配置
  uint32_t pending_queue_size;           // 未处理消息队列容量（默认 1）
};
```

默认 QoS 配置：
- `history`: `HISTORY_KEEP_LAST`
- `depth`: 1
- `mps`: 0（不限速）
- `reliability`: `RELIABILITY_RELIABLE`
- `durability`: `DURABILITY_VOLATILE`

### WriterBase

Writer 的抽象基类，定义于 `cyber/node/writer_base.h`：

```cpp
class WriterBase {
public:
  explicit WriterBase(const proto::RoleAttributes& role_attr);

  virtual bool Init() = 0;
  virtual void Shutdown() = 0;
  virtual bool HasReader();
  virtual void GetReaders(std::vector<proto::RoleAttributes>* readers);

  const std::string& GetChannelName() const;
  const uint64_t GetChannelId() const;
  bool IsInit() const;

protected:
  proto::RoleAttributes role_attr_;
};
```

### Writer\<MessageT\>

模板类，继承自 `WriterBase`，定义于 `cyber/node/writer.h`。通过 Transport 层的 `Transmitter` 发送消息。

```cpp
template <typename MessageT>
class Writer : public WriterBase {
public:
  explicit Writer(const proto::RoleAttributes& role_attr);

  bool Init() override;
  void Shutdown() override;

  // 发送消息（值语义或智能指针语义）
  virtual bool Write(const MessageT& msg);
  virtual bool Write(const std::shared_ptr<MessageT>& msg_ptr);

  // 查询是否有 Reader 订阅了当前 channel
  bool HasReader() override;
  void GetReaders(std::vector<proto::RoleAttributes>* readers) override;
};
```

Init 流程：
1. 通过 `Transport::Instance()->CreateTransmitter<MessageT>()` 创建 Transmitter
2. 将自身注册到 `ChannelManager`（加入拓扑）
3. 监听拓扑变化事件

### ReaderBase

Reader 的抽象基类，定义于 `cyber/node/reader_base.h`：

```cpp
class ReaderBase {
public:
  explicit ReaderBase(const proto::RoleAttributes& role_attr);

  virtual bool Init() = 0;
  virtual void Shutdown() = 0;
  virtual void ClearData() = 0;
  virtual void Observe() = 0;
  virtual bool Empty() const = 0;
  virtual bool HasWriter() = 0;
  virtual void GetWriters(std::vector<proto::RoleAttributes>* writers) = 0;

  const std::string& GetChannelName() const;
  uint64_t ChannelId() const;
  uint32_t PendingQueueSize() const;
};
```

### Reader\<MessageT\>

模板类，继承自 `ReaderBase`，定义于 `cyber/node/reader.h`。支持两种消息消费方式：

1. **回调模式**：注册 `CallbackFunc`，消息到达时自动触发
2. **轮询模式**：调用 `Observe()` 后通过迭代器访问消息

```cpp
template <typename MessageT>
class Reader : public ReaderBase {
public:
  Reader(const proto::RoleAttributes& role_attr,
         const CallbackFunc<MessageT>& reader_func = nullptr,
         uint32_t pending_queue_size = DEFAULT_PENDING_QUEUE_SIZE);

  bool Init() override;
  void Shutdown() override;
  void Observe() override;

  // 获取最新消息
  std::shared_ptr<MessageT> GetLatestObserved() const;

  // 获取最旧消息
  std::shared_ptr<MessageT> GetOldestObserved() const;

  // 迭代器访问 ObserveQueue
  Iterator Begin() const;
  Iterator End() const;

  // 历史深度控制
  void SetHistoryDepth(const uint32_t& depth);
  uint32_t GetHistoryDepth() const;

  // 查询对端 Writer
  bool HasWriter() override;
  void GetWriters(std::vector<proto::RoleAttributes>* writers) override;
};
```

Init 流程：
1. 通过 `ReceiverManager` 获取或创建 `Receiver`（同一 channel 共享一个 Receiver）
2. 创建 `DataVisitor` 和 `ChannelBuffer` 用于数据缓存
3. 如果设置了回调函数，通过 `RoutineFactory` 创建协程任务并注册到调度器
4. 创建 `Blocker` 用于 Observe 模式的消息缓存
5. 将自身注册到 `ChannelManager`（加入拓扑）
6. 监听拓扑变化事件

### ReceiverManager\<MessageT\>

单例模板类，管理每个 channel 的 `Receiver` 实例，确保同一 channel 只创建一个 Receiver。Receiver 收到消息后通过 `DataDispatcher` 分发给所有订阅者。

```cpp
template <typename MessageT>
class ReceiverManager {
public:
  auto GetReceiver(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<transport::Receiver<MessageT>>;
};
```

### NodeServiceImpl

负责 Service/Client 通信对象的创建，定义于 `cyber/node/node_service_impl.h`：

```cpp
class NodeServiceImpl {
public:
  explicit NodeServiceImpl(const std::string& node_name);

  template <typename Request, typename Response>
  auto CreateService(const std::string& service_name,
                     const typename Service<Request, Response>::ServiceCallback&
                         service_callback)
      -> std::shared_ptr<Service<Request, Response>>;

  template <typename Request, typename Response>
  auto CreateClient(const std::string& service_name)
      -> std::shared_ptr<Client<Request, Response>>;
};
```

## 数据流描述

### 发布流程（Writer）

```
用户代码 -> Writer::Write(msg)
         -> Transmitter::Transmit(msg)
         -> Transport 层（SHM / RTPS / Intra）
         -> 网络 / 共享内存
```

### 订阅流程（Reader）

```
Transport 层接收消息
  -> Receiver 回调
  -> DataDispatcher::Dispatch()
  -> DataVisitor / ChannelBuffer 缓存
  -> 调度器触发协程
  -> 用户回调函数 CallbackFunc
```

### Observe 模式

```
消息到达 -> Blocker::PublishQueue 自动入队
用户调用 Reader::Observe()
  -> Blocker 将 PublishQueue 内容移至 ObserveQueue
  -> 用户通过 Begin()/End() 迭代访问
```

## 配置方式

### Proto 定义

Node 通信依赖以下 proto 消息（定义于 `cyber/proto/`）：

**RoleAttributes**（`role_attributes.proto`）：标识通信端点的完整属性。

```protobuf
message RoleAttributes {
  optional string host_name = 1;
  optional string host_ip = 2;
  optional int32 process_id = 3;
  optional string node_name = 4;
  optional uint64 node_id = 5;
  optional string channel_name = 6;
  optional uint64 channel_id = 7;
  optional string message_type = 8;
  optional bytes proto_desc = 9;
  optional uint64 id = 10;
  optional QosProfile qos_profile = 11;
  optional SocketAddr socket_addr = 12;
  optional string service_name = 13;
  optional uint64 service_id = 14;
}
```

**QosProfile**（`qos_profile.proto`）：服务质量配置。

```protobuf
message QosProfile {
  optional QosHistoryPolicy history = 1;     // KEEP_LAST / KEEP_ALL
  optional uint32 depth = 2;                 // 历史消息容量
  optional uint32 mps = 3;                   // 每秒消息数限制（0 = 不限）
  optional QosReliabilityPolicy reliability = 4;  // RELIABLE / BEST_EFFORT
  optional QosDurabilityPolicy durability = 5;    // TRANSIENT_LOCAL / VOLATILE
}
```

### 代码中配置 Reader

```cpp
// 方式一：简单用法，仅指定 channel 名称
auto reader = node->CreateReader<MessageType>("channel_name", callback);

// 方式二：通过 ReaderConfig 精细控制
ReaderConfig config;
config.channel_name = "channel_name";
config.pending_queue_size = 10;
config.qos_profile.set_depth(5);
auto reader = node->CreateReader<MessageType>(config, callback);

// 方式三：通过 RoleAttributes 完全控制
proto::RoleAttributes attr;
attr.set_channel_name("channel_name");
attr.mutable_qos_profile()->set_depth(5);
auto reader = node->CreateReader<MessageType>(attr, callback);
```

## 与其他模块的关系

| 依赖模块 | 关系说明 |
|---------|---------|
| **Transport** | Writer 通过 `Transmitter` 发送消息，Reader 通过 `Receiver` 接收消息 |
| **Data** | Reader 使用 `DataVisitor`、`DataDispatcher`、`ChannelBuffer` 进行数据缓存与分发 |
| **Scheduler** | Reader 的回调通过协程任务（`CRoutine`）被调度器调度执行 |
| **Service Discovery** | Writer/Reader 创建时注册到 `TopologyManager`，实现拓扑感知 |
| **Blocker** | Reader 使用 `Blocker` 实现 Observe 模式；仿真模式下使用 `IntraReader`/`IntraWriter` |
| **Component** | `Component` 和 `TimerComponent` 内部持有 `Node` 实例，通过 Node 创建 Reader |
| **Proto** | `RoleAttributes`、`QosProfile` 等 proto 消息定义了通信端点的元数据 |
