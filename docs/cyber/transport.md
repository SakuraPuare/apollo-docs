---
title: Transport 模块
---

# Transport 模块

## 模块职责概述

Transport 模块是 Apollo Cyber RT 的传输层，负责在不同进程、不同主机之间高效传递消息。它实现了三种传输机制 -- Intra-process（进程内）、SHM（共享内存）、RTPS（跨主机网络） -- 并通过 Hybrid 模式根据通信双方的拓扑关系自动选择最优传输方式。

## 架构总览

```
                        Transport (单例入口)
                       /        |          \
              Transmitter    Dispatcher    Receiver
              /   |   \      /   |   \     /   |   \
          Intra  SHM  RTPS Intra SHM RTPS Intra SHM RTPS
              \   |   /
            HybridTransmitter / HybridReceiver
```

## 核心类/接口说明

### Transport

单例类，传输层的统一入口。负责创建 Transmitter 和 Receiver，管理 RTPS Participant 和各 Dispatcher 的生命周期。

```cpp
class Transport {
 public:
  template <typename M>
  auto CreateTransmitter(const RoleAttributes& attr,
                         const OptionalMode& mode = OptionalMode::HYBRID)
      -> std::shared_ptr<Transmitter<M>>;

  template <typename M>
  auto CreateReceiver(const RoleAttributes& attr,
                      const typename Receiver<M>::MessageListener& msg_listener,
                      const OptionalMode& mode = OptionalMode::HYBRID)
      -> std::shared_ptr<Receiver<M>>;

  ParticipantPtr participant() const;
  void Shutdown();
};
```

构造时初始化所有子系统：

```cpp
Transport::Transport() {
  CreateParticipant();                          // RTPS Participant
  notifier_ = NotifierFactory::CreateNotifier(); // SHM 通知器
  intra_dispatcher_ = IntraDispatcher::Instance();
  shm_dispatcher_ = ShmDispatcher::Instance();
  rtps_dispatcher_ = RtpsDispatcher::Instance();
  rtps_dispatcher_->set_participant(participant_);
}
```

### Endpoint

所有 Transmitter 和 Receiver 的公共基类：

```cpp
class Endpoint {
 protected:
  bool enabled_;
  Identity id_;          // 8 字节唯一标识
  RoleAttributes attr_;  // 角色属性（channel、node、QoS 等）
};
```

### Identity

8 字节的唯一标识符，构造时自动生成，用于区分不同的 Transmitter/Receiver 实例：

```cpp
class Identity {
 public:
  explicit Identity(bool need_generate = true);
  std::string ToString() const;
  uint64_t HashValue() const;
  const char* data() const;
  void set_data(const char* data);
 private:
  char data_[ID_SIZE];  // ID_SIZE = 8
  uint64_t hash_value_;
};
```

### MessageInfo

消息传输的元数据，随消息一起在传输层流转：

```cpp
class MessageInfo {
 public:
  const Identity& sender_id() const;
  void set_sender_id(const Identity& sender_id);
  uint64_t seq_num() const;
  void set_seq_num(uint64_t seq_num);
  const Identity& spare_id() const;
  int32_t msg_seq_num() const;
  uint64_t send_time() const;

  bool SerializeTo(std::string* dst) const;
  bool SerializeTo(char* dst, std::size_t len) const;
  bool DeserializeFrom(const char* src, std::size_t len);

  static const std::size_t kSize;
};
```

### Transmitter\<M\>

发送端抽象基类，定义了消息发送的统一接口：

```cpp
template <typename M>
class Transmitter : public Endpoint {
 public:
  using MessagePtr = std::shared_ptr<M>;

  virtual void Enable() = 0;
  virtual void Disable() = 0;
  virtual bool AcquireMessage(std::shared_ptr<M>& msg) = 0;

  virtual void Enable(const RoleAttributes& opposite_attr);
  virtual void Disable(const RoleAttributes& opposite_attr);

  virtual bool Transmit(const MessagePtr& msg);
  virtual bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) = 0;

  uint64_t NextSeqNum();
};
```

`Transmit(msg)` 的默认实现会自动递增序列号、记录发送时间、写入性能事件，然后调用子类的 `Transmit(msg, msg_info)`。

### Receiver\<M\>

接收端抽象基类：

```cpp
template <typename M>
class Receiver : public Endpoint {
 public:
  using MessageListener = std::function<void(
      const MessagePtr&, const MessageInfo&, const RoleAttributes&)>;

  Receiver(const RoleAttributes& attr, const MessageListener& msg_listener);

  virtual void Enable() = 0;
  virtual void Disable() = 0;
  virtual void Enable(const RoleAttributes& opposite_attr) = 0;
  virtual void Disable(const RoleAttributes& opposite_attr) = 0;

 protected:
  void OnNewMessage(const MessagePtr& msg, const MessageInfo& msg_info);
  MessageListener msg_listener_;
};
```

### Dispatcher

消息分发器基类，维护 channel_id 到 `ListenerHandler` 的映射：

```cpp
class Dispatcher {
 public:
  template <typename MessageT>
  void AddListener(const RoleAttributes& self_attr,
                   const MessageListener<MessageT>& listener);

  template <typename MessageT>
  void AddListener(const RoleAttributes& self_attr,
                   const RoleAttributes& opposite_attr,
                   const MessageListener<MessageT>& listener);

  template <typename MessageT>
  void RemoveListener(const RoleAttributes& self_attr);

  bool HasChannel(uint64_t channel_id);

 protected:
  AtomicHashMap<uint64_t, ListenerHandlerBasePtr> msg_listeners_;
};
```

内部使用 `AtomicHashMap` 实现无锁读的高性能查找。`ListenerHandler<MessageT>` 基于 Signal/Slot 模式，支持按 `self_id` 和 `oppo_id` 两种粒度连接回调。

### History\<M\>

消息历史缓存，用于 `DURABILITY_TRANSIENT_LOCAL` QoS 策略下的历史消息重发：

```cpp
template <typename MessageT>
class History {
 public:
  struct CachedMessage {
    MessagePtr msg;
    MessageInfo msg_info;
  };

  explicit History(const HistoryAttributes& attr);
  void Enable();
  void Add(const MessagePtr& msg, const MessageInfo& msg_info);
  void GetCachedMessage(std::vector<CachedMessage>* msgs) const;
  size_t GetSize() const;
};
```

缓存深度由 QoS `depth` 参数控制，上限受 `TransportConf.resource_limit.max_history_depth` 约束（默认 1000）。

---

## Intra-process 传输

进程内传输是最轻量的方式，消息以 `std::shared_ptr` 直接在同进程的 Writer 和 Reader 之间传递，零拷贝、零序列化。

### IntraTransmitter

```cpp
template <typename M>
class IntraTransmitter : public Transmitter<M> {
  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override {
    dispatcher_->OnMessage(channel_id_, msg, msg_info);
    return true;
  }
};
```

直接调用 `IntraDispatcher::OnMessage()`，将消息指针传递给所有注册的 listener。

### IntraDispatcher

进程内分发器，核心特性是支持跨消息类型的分发。内部维护 `ChannelChain` 对象：

- 同类型消息：直接传递 `shared_ptr`，零拷贝
- 不同类型消息（如 Writer 发 `SensorMsg`，Reader 订阅 `RawMessage`）：先序列化为 HC 格式字符串，再通过 `RunFromString` 反序列化为目标类型

```cpp
// ChannelChain::Run 核心逻辑
if (message_type == ele.first) {
  // 类型匹配，直接传递指针
  handler->Run(message, message_info);
} else {
  // 类型不匹配，序列化后转发
  message::SerializeToHC(*message, msg.data(), msg_size);
  handler_base->RunFromString(msg, message_info);
}
```

---

## SHM（共享内存）传输

共享内存传输用于同一主机不同进程间的高效通信，避免了网络协议栈的开销。

### 核心组件

#### Segment

共享内存段的抽象，每个 channel 对应一个 Segment：

```cpp
class Segment {
 public:
  explicit Segment(uint64_t channel_id);

  bool AcquireBlockToWrite(std::size_t msg_size, WritableBlock* writable_block);
  void ReleaseWrittenBlock(const WritableBlock& writable_block);
  bool AcquireBlockToRead(ReadableBlock* readable_block);
  void ReleaseReadBlock(const ReadableBlock& readable_block);

  // Arena 模式
  bool AcquireArenaBlockToWrite(std::size_t msg_size, WritableBlock* writable_block);
  void ReleaseArenaWrittenBlock(const WritableBlock& writable_block);
};
```

`WritableBlock` / `ReadableBlock` 结构：

```cpp
struct WritableBlock {
  uint32_t index = 0;   // block 索引
  Block* block = nullptr; // block 元数据
  uint8_t* buf = nullptr; // 数据缓冲区指针
};
```

Segment 有两种实现：
- `PosixSegment`：基于 POSIX 共享内存（`shm_open`/`mmap`）
- `XsiSegment`：基于 System V 共享内存（`shmget`/`shmat`）

由 `SegmentFactory::CreateSegment()` 根据配置创建。

#### Block

共享内存中的数据块，使用原子变量实现读写锁：

```cpp
class Block {
  std::atomic<int32_t> lock_num_;  // 0=free, -1=写独占, >0=读者计数
  uint64_t msg_size_;
  uint64_t msg_info_size_;
};
```

#### ShmConf

根据消息大小自动计算共享内存配置：

| 消息大小 | Block 数量 | 缓冲区大小 |
|----------|-----------|-----------|
| 0 - 10K | 512 | 16K |
| 10K - 100K | 128 | 128K |
| 100K - 1M | 64 | 1M |
| 1M - 6M | 32 | 8M |
| 6M - 10M | 16 | 16M |
| 10M+ | 8 | 动态 |

#### NotifierBase 通知机制

写入共享内存后需要通知读者，有两种通知器实现：

```cpp
class NotifierBase {
 public:
  virtual bool Notify(const ReadableInfo& info) = 0;
  virtual bool Listen(int timeout_ms, ReadableInfo* info) = 0;
  virtual void Shutdown() = 0;
};
```

- `ConditionNotifier`：基于共享内存的环形缓冲区（4096 槽位），通过原子序列号实现无锁通知，适用于单机场景
- `MulticastNotifier`：基于 UDP 组播，通过网络 socket 发送/接收 `ReadableInfo`

`NotifierFactory` 根据配置 `shm_conf.notifier_type` 选择实现。

#### ReadableInfo

通知消息体，告知读者哪个 channel 的哪个 block 有新数据：

```cpp
class ReadableInfo {
  uint64_t host_id_;
  int32_t block_index_;        // 普通序列化 block 索引
  int32_t arena_block_index_;  // Arena block 索引
  uint64_t channel_id_;
};
```

### ShmTransmitter

SHM 发送端的核心发送流程：

```
1. AcquireBlockToWrite(msg_size) → 获取可写 block
2. SerializeToArray(msg, block.buf) → 序列化消息到共享内存
3. msg_info.SerializeTo(block.buf + msg_size) → 写入 MessageInfo
4. ReleaseWrittenBlock() → 释放写锁
5. notifier_->Notify(readable_info) → 通知读者
```

ShmTransmitter 还支持 Arena 模式（零拷贝 protobuf）：当 channel 配置了 Arena SHM 且消息类型不是 RawMessage/PyMessageWrap 时，会同时写入 arena block 和普通 block（兼容不同类型的 receiver）。

### ShmDispatcher

SHM 接收端分发器，运行独立线程轮询通知：

```
ThreadFunc 循环:
  notifier_->Listen(timeout, &readable_info)
    → ReadMessage(channel_id, block_index)
      → segment->AcquireBlockToRead()
      → 构造 ReadableBlock
      → OnMessage(channel_id, rb, msg_info)
        → ListenerHandler::Run()
          → 反序列化 + 回调用户 listener
```

对于 Arena 模式，`ShmDispatcher::AddListener` 会注册特殊的 listener adapter，通过 `ProtobufArenaManager` 直接从共享内存获取 protobuf 消息指针，实现零拷贝读取。

### ProtobufArenaManager

基于 `google::protobuf::Arena` 的共享内存消息管理器，实现跨进程零拷贝：

```cpp
class ProtobufArenaManager : public message::ArenaManagerBase {
 public:
  bool Enable();
  bool EnableSegment(uint64_t channel_id);

  void* SetMessage(ArenaMessageWrapper* wrapper, const void* message) override;
  void* GetMessage(ArenaMessageWrapper* wrapper) override;

  std::shared_ptr<ArenaSegment> GetSegment(uint64_t channel_id);

  template <typename M>
  void AcquireArenaMessage(uint64_t channel_id, std::shared_ptr<M>& ret_msg);
};
```

`ArenaSegment` 管理每个 channel 的 arena 内存区域，包含：
- `ArenaSegmentState`：引用计数、消息大小、block 数量等原子状态
- `ArenaSegmentBlock[]`：每个 block 的读写锁和大小信息
- `google::protobuf::Arena[]`：每个 block 对应一个 protobuf Arena 实例

`AcquireArenaMessage` 在 arena 上直接分配 protobuf 消息对象，避免堆分配和拷贝。

---

## RTPS 传输

RTPS（Real-Time Publish-Subscribe）传输基于 eProsima Fast-RTPS 实现，用于跨主机通信。

### Participant

RTPS 域参与者的封装：

```cpp
class Participant {
 public:
  Participant(const std::string& name, int send_port,
              eprosima::fastrtps::ParticipantListener* listener = nullptr);
  eprosima::fastrtps::Participant* fastrtps_participant();
  void Shutdown();
};
```

参与者名称格式为 `hostname+pid`，端口默认 11512。

### RtpsTransmitter

RTPS 发送端，通过 Fast-RTPS Publisher 发送消息：

```cpp
template <typename M>
bool RtpsTransmitter<M>::Transmit(const M& msg, const MessageInfo& msg_info) {
  UnderlayMessage m;
  message::SerializeToString(msg, &m.data());  // 序列化为字符串
  m.timestamp(send_time);
  m.seq(msg_info.msg_seq_num());

  // 将 sender_id、spare_id、seq_num 编码到 RTPS WriteParams
  eprosima::fastrtps::rtps::WriteParams wparams;
  memcpy(ptr, msg_info.sender_id().data(), ID_SIZE);
  memcpy(ptr + ID_SIZE, msg_info.spare_id().data(), ID_SIZE);

  return publisher_->write(&m, wparams);
}
```

消息先序列化为 string，封装到 `UnderlayMessage`（自定义 RTPS 数据类型），MessageInfo 编码到 RTPS 的 `WriteParams.related_sample_identity` 中。

### RtpsDispatcher

RTPS 接收端分发器，为每个 channel 创建 Fast-RTPS Subscriber：

```cpp
template <typename MessageT>
void RtpsDispatcher::AddListener(const RoleAttributes& self_attr,
                                 const MessageListener<MessageT>& listener) {
  auto listener_adapter = [listener](const std::shared_ptr<std::string>& msg_str,
                                     const MessageInfo& msg_info) {
    auto msg = std::make_shared<MessageT>();
    message::ParseFromString(*msg_str, msg.get());  // 反序列化
    listener(msg, msg_info);
  };
  Dispatcher::AddListener<std::string>(self_attr, listener_adapter);
  AddSubscriber(self_attr);  // 创建 RTPS Subscriber
}
```

`SubListener` 接收 RTPS 数据后，从 `WriteParams` 中恢复 MessageInfo，然后调用 `OnMessage` 分发。

---

## Hybrid 模式

Hybrid 是默认的传输模式，根据通信双方的拓扑关系自动选择传输方式。

### 关系判定

```cpp
Relation HybridTransmitter::GetRelation(const RoleAttributes& opposite_attr) {
  if (opposite_attr.channel_name() != this->attr_.channel_name())
    return NO_RELATION;
  if (opposite_attr.host_ip() != this->attr_.host_ip())
    return DIFF_HOST;
  if (opposite_attr.process_id() != this->attr_.process_id())
    return DIFF_PROC;
  return SAME_PROC;
}
```

### 默认映射

| 关系 | 默认传输方式 |
|------|------------|
| `SAME_PROC`（同进程） | INTRA |
| `DIFF_PROC`（同主机不同进程） | SHM |
| `DIFF_HOST`（不同主机） | RTPS |

### HybridTransmitter

内部持有所有需要的子 Transmitter，发送时向所有活跃的子 Transmitter 广播：

```cpp
template <typename M>
bool HybridTransmitter<M>::Transmit(const MessagePtr& msg,
                                    const MessageInfo& msg_info) {
  history_->Add(msg, msg_info);
  for (auto& item : transmitters_) {
    item.second->Transmit(msg, msg_info);
  }
  return true;
}
```

当新的 Receiver 上线时（`Enable(opposite_attr)`），根据 `GetRelation` 判定关系，激活对应的子 Transmitter。如果 QoS 配置了 `DURABILITY_TRANSIENT_LOCAL`，还会异步发送历史缓存消息。

### HybridReceiver

内部持有所有需要的子 Receiver，根据配置初始化：

```cpp
void HybridReceiver<M>::InitReceivers() {
  std::set<OptionalMode> modes;
  modes.insert(mode_->same_proc());   // 默认 INTRA
  modes.insert(mode_->diff_proc());   // 默认 SHM
  modes.insert(mode_->diff_host());   // 默认 RTPS
  for (auto& mode : modes) {
    switch (mode) {
      case OptionalMode::INTRA:
        receivers_[mode] = std::make_shared<IntraReceiver<M>>(attr_, listener);
        break;
      case OptionalMode::SHM:
        receivers_[mode] = std::make_shared<ShmReceiver<M>>(attr_, listener);
        break;
      default:
        receivers_[mode] = std::make_shared<RtpsReceiver<M>>(attr_, listener);
        break;
    }
  }
}
```

---

## QoS 配置

QoS（Quality of Service）通过 `qos_profile.proto` 定义：

```protobuf
message QosProfile {
  optional QosHistoryPolicy history = 1 [default = HISTORY_KEEP_LAST];
  optional uint32 depth = 2 [default = 1];
  optional uint32 mps = 3 [default = 0];  // messages per second
  optional QosReliabilityPolicy reliability = 4 [default = RELIABILITY_RELIABLE];
  optional QosDurabilityPolicy durability = 5 [default = DURABILITY_VOLATILE];
}
```

| 策略 | 选项 | 说明 |
|------|------|------|
| History | `KEEP_LAST` / `KEEP_ALL` | 历史消息保留策略 |
| Reliability | `RELIABLE` / `BEST_EFFORT` | 可靠性策略 |
| Durability | `VOLATILE` / `TRANSIENT_LOCAL` | 持久性策略，`TRANSIENT_LOCAL` 会缓存历史消息供后来的订阅者获取 |

预定义 QoS Profile：

```cpp
QOS_PROFILE_DEFAULT        // 默认配置
QOS_PROFILE_SENSOR_DATA    // 传感器数据（高频、允许丢失）
QOS_PROFILE_PARAMETERS     // 参数服务
QOS_PROFILE_SERVICES_DEFAULT // 服务默认
QOS_PROFILE_TF_STATIC      // 静态 TF（TRANSIENT_LOCAL）
QOS_PROFILE_TOPO_CHANGE    // 拓扑变更
```

## 数据流描述

### 完整发送流程（Hybrid 模式）

```
Writer::Write(msg)
  → Transmitter::Transmit(msg)
    → 设置 seq_num, send_time, 记录 PerfEvent
    → HybridTransmitter::Transmit(msg, msg_info)
      → history_->Add(msg, msg_info)
      → IntraTransmitter::Transmit()
        → IntraDispatcher::OnMessage() → 直接传递 shared_ptr
      → ShmTransmitter::Transmit()
        → Segment::AcquireBlockToWrite()
        → SerializeToArray() → 写入共享内存
        → Notifier::Notify() → 通知读者进程
      → RtpsTransmitter::Transmit()
        → SerializeToString() → 封装 UnderlayMessage
        → Publisher::write() → Fast-RTPS 网络发送
```

### 完整接收流程（Hybrid 模式）

```
HybridReceiver 持有三个子 Receiver:

Intra 路径:
  IntraDispatcher::OnMessage()
    → ListenerHandler::Run() → 回调

SHM 路径:
  ShmDispatcher::ThreadFunc() 轮询
    → Notifier::Listen()
    → Segment::AcquireBlockToRead()
    → ParseFromArray() → 反序列化
    → ListenerHandler::Run() → 回调

RTPS 路径:
  SubListener::onNewDataMessage()
    → RtpsDispatcher::OnMessage()
    → ParseFromString() → 反序列化
    → ListenerHandler::Run() → 回调

所有路径最终:
  → Receiver::OnNewMessage()
    → msg_listener_(msg, msg_info, attr_)
      → Reader 的回调处理
```

## 配置方式

Transport 通过 `transport_conf.proto` 配置：

```protobuf
message TransportConf {
  optional ShmConf shm_conf = 1;
  optional RtpsParticipantAttr participant_attr = 2;
  optional CommunicationMode communication_mode = 3;
  optional ResourceLimit resource_limit = 4;
}
```

### 通信模式配置

```protobuf
message CommunicationMode {
  optional OptionalMode same_proc = 1 [default = INTRA];
  optional OptionalMode diff_proc = 2 [default = SHM];
  optional OptionalMode diff_host = 3 [default = RTPS];
}

enum OptionalMode {
  HYBRID = 0;
  INTRA = 1;
  SHM = 2;
  RTPS = 3;
}
```

可以覆盖默认映射，例如强制同进程也走 SHM。

### SHM 配置

```protobuf
message ShmConf {
  optional string notifier_type = 1;   // "condition" 或 "multicast"
  optional string shm_type = 2;        // "posix" 或 "xsi"
  optional ShmMulticastLocator shm_locator = 3;
  optional ArenaShmConf arena_shm_conf = 4;
}

message ArenaChannelConf {
  optional string channel_name = 1;
  optional uint64 max_msg_size = 2 [default = 33554432];   // 32MB
  optional uint64 max_pool_size = 3 [default = 32];
  optional uint64 shared_buffer_size = 4 [default = 0];
}
```

### RTPS 配置

```protobuf
message RtpsParticipantAttr {
  optional int32 lease_duration = 1 [default = 12];
  optional int32 announcement_period = 2 [default = 3];
  optional uint32 domain_id_gain = 3 [default = 200];
  optional uint32 port_base = 4 [default = 10000];
}
```

## 与其他模块的关系

| 模块 | 关系 |
|------|------|
| Message | Transport 依赖 Message 模块的 traits 体系完成序列化/反序列化 |
| Node/Reader/Writer | 上层通过 `Transport::CreateTransmitter/CreateReceiver` 创建传输端点 |
| Service Discovery | 拓扑变更时通知 HybridTransmitter/HybridReceiver 动态 Enable/Disable 子传输通道 |
| Scheduler | 消息到达后触发协程调度 |
| proto | `transport_conf.proto`、`qos_profile.proto`、`role_attributes.proto` 定义了传输层的配置和属性结构 |
