---
title: "Bridge 公共库"
---

# Bridge 公共库

> 源码路径: `modules/bridge/common/`

## 概述

Bridge 公共库提供了跨主机 Protobuf 消息传输所需的基础设施，包括消息帧头定义、序列化/反序列化、缓冲区管理和 UDP 网络监听。消息在发送端被分帧（每帧 1024 字节），通过 UDP 传输后在接收端重组并发布到 CyberRT 话题。

## 核心类

### BridgeHeader

消息帧头，包含消息名称、序列号、时间戳、总帧数、当前帧索引等元信息。帧头以 `"ApolloBridgeHeader"` 标志开头。

```cpp
class BridgeHeader {
 public:
  bool Serialize(char *buf, size_t size);
  bool Diserialize(const char *buf, size_t buf_size);
  bool IsAvailable(const char *buf);

  std::string GetMsgName() const;
  uint32_t GetMsgID() const;
  uint32_t GetTotalFrames() const;
  uint32_t GetIndex() const;
  double GetTimeStamp() const;
  bsize GetMsgSize() const;
  bsize GetFrameSize() const;
  bsize GetFramePos() const;
  hsize GetHeaderSize() const;

  void SetHeaderVer(uint32_t header_ver);
  void SetMsgName(const std::string &msg_name);
  void SetMsgID(uint32_t msg_id);
  void SetTotalFrames(uint32_t total_frames);
  void SetFrameSize(bsize frame_size);
  void SetFramePos(bsize frame_pos);
  void SetIndex(uint32_t index);
  void SetTimeStamp(double time_stamp);
  void SetMsgSize(bsize msg_size);
};
```

### BridgeProtoSerializedBuf\<T\>

将 Protobuf 消息序列化并按 `FRAME_SIZE`（1024 字节）分帧，每帧前附加 `BridgeHeader`。

```cpp
template <typename T>
class BridgeProtoSerializedBuf {
 public:
  bool Serialize(const std::shared_ptr<T> &proto, const std::string &msg_name);
  const char *GetSerializedBuf(size_t index) const;
  size_t GetSerializedBufCount() const;
  size_t GetSerializedBufSize(size_t index) const;
};
```

### BridgeProtoDiserializedBuf\<T\>

接收端反序列化缓冲区，收集所有帧后重组为完整 Protobuf 消息并通过 CyberRT Writer 发布。

```cpp
template <typename T>
class BridgeProtoDiserializedBuf : public ProtoDiserializedBufBase {
 public:
  bool Initialize(const BridgeHeader &header, std::shared_ptr<cyber::Node> node);
  bool DiserializedAndPub();
  bool IsReadyDiserialize() const;
  void UpdateStatus(uint32_t frame_index);
  bool IsTheProto(const BridgeHeader &header);
  char *GetBuf(size_t offset);
};
```

### BridgeBuffer\<T\>

线程安全的通用缓冲区，支持按偏移写入数据。

```cpp
template <typename T>
class BridgeBuffer {
 public:
  void reset(size_t size);
  size_t size() const;
  size_t capacity() const;
  void write(size_t index, const T *data, size_t size);
};
```

### UDPListener\<T\>

基于 epoll 的 UDP 监听器，接收到数据后通过 pthread 分发给回调函数处理。

```cpp
template <typename T>
class UDPListener {
 public:
  bool Initialize(T *receiver, func msg_handle, uint16_t port);
  bool Listen();
};
```

### ProtoDiserializedBufBaseFactory

根据 `BridgeHeader` 中的消息名称创建对应的反序列化缓冲区实例（目前支持 `Chassis`）。

```cpp
class ProtoDiserializedBufBaseFactory {
 public:
  static std::shared_ptr<ProtoDiserializedBufBase> CreateObj(
      const BridgeHeader &header);
};
```

## 核心函数

### 工具函数（util.h）

```cpp
// 将 Protobuf 消息写入 BridgeBuffer（含长度前缀）
template <typename T>
void WriteToBuffer(BridgeBuffer<char> *buf, const std::shared_ptr<T> &pb_msg);

// 从列表中移除并释放指定元素
template <typename T>
bool RemoveItem(std::vector<T *> *list, const T *t);

// 从缓冲区头部读取 proto 消息大小
int GetProtoSize(const char *buf, size_t size);
```

### 配置标志（bridge_gflags）

| 标志 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bridge_module_name` | string | `"Bridge"` | 模块名称 |
| `timeout` | double | `1.0` | 收发消息超时时间（秒） |

### 宏定义（macro.h）

| 宏 | 说明 |
|------|------|
| `FRAME_SIZE` | 单帧最大数据量，1024 字节 |
| `FREE_ARRY(arry)` | 安全释放数组内存 |
| `FREE_POINTER(p)` | 安全释放指针内存 |

## 调用关系

```text
发送端:
  Protobuf Message
    -> BridgeProtoSerializedBuf::Serialize()
      -> BridgeHeader (附加帧头)
      -> 按 FRAME_SIZE 分帧
        -> UDP 发送

接收端:
  UDPListener::Listen()
    -> epoll 接收 UDP 数据
      -> BridgeHeader::Diserialize() 解析帧头
      -> ProtoDiserializedBufBaseFactory::CreateObj() 创建缓冲区
      -> BridgeProtoDiserializedBuf::UpdateStatus() 记录帧状态
      -> IsReadyDiserialize() 判断是否收齐
        -> DiserializedAndPub() 重组并发布到 CyberRT
```
