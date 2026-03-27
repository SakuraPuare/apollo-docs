---
title: Message 模块
---

# Message 模块

## 模块职责概述

Message 模块是 Apollo Cyber RT 的消息抽象层，负责定义统一的消息类型系统、序列化/反序列化接口以及消息元数据管理。它通过 C++ 模板元编程（SFINAE）构建了一套 traits 体系，使得 protobuf 消息、原始字节消息、Python 消息等不同类型都能在同一套传输框架下无缝工作。

## 核心类/接口说明

### MessageTraits 类型萃取体系

`message_traits.h` 是整个模块的核心，通过 `DEFINE_TYPE_TRAIT` 宏定义了一系列编译期类型检测 trait：

```cpp
DEFINE_TYPE_TRAIT(HasByteSize, ByteSizeLong)
DEFINE_TYPE_TRAIT(HasType, TypeName)
DEFINE_TYPE_TRAIT(HasSerializer, SerializeToString)
DEFINE_TYPE_TRAIT(HasParseFromArray, ParseFromArray)
DEFINE_TYPE_TRAIT(HasSerializeToArenaMessageWrapper, SerializeToArenaMessageWrapper)
// ... 等
```

`HasSerializer<T>` 聚合判断一个类型是否同时具备四个序列化方法：

```cpp
template <typename T>
class HasSerializer {
 public:
  static constexpr bool value =
      HasSerializeToString<T>::value && HasParseFromString<T>::value &&
      HasSerializeToArray<T>::value && HasParseFromArray<T>::value;
};
```

基于这些 trait，模块为以下自由函数提供了多个 SFINAE 重载版本：

| 函数 | 功能 |
|------|------|
| `MessageType<T>()` / `MessageType(msg)` | 获取消息类型名称 |
| `ByteSize(msg)` | 获取序列化后字节大小 |
| `SerializeToArray(msg, buf, size)` | 序列化到字节数组 |
| `ParseFromArray(data, size, msg)` | 从字节数组反序列化 |
| `SerializeToString(msg, str)` | 序列化到 string |
| `ParseFromString(str, msg)` | 从 string 反序列化 |
| `ParseFromHC(data, size, msg)` | 带 MessageHeader 的反序列化 |
| `GetMessageName<T>()` | 获取消息全限定名 |
| `GetDescriptorString(msg, desc_str)` | 获取 protobuf descriptor 序列化字符串 |

分发逻辑优先级：
1. `google::protobuf::Message` 子类 -- 走 `protobuf_traits.h` 特化
2. 具有 `TypeName` 静态/成员方法的自定义类型 -- 走通用 trait 分发
3. `RawMessage` -- 走 `raw_message_traits.h` 特化
4. `PyMessageWrap` -- 走 `py_message_traits.h` 特化
5. 其他类型 -- fallback 到 `typeid(T).name()`

### ProtobufFactory

单例类，管理 protobuf 消息的动态注册与创建。

```cpp
class ProtobufFactory {
 public:
  // 注册消息（支持 Message 对象、Descriptor、FileDescriptorProto、ProtoDesc）
  bool RegisterMessage(const google::protobuf::Message& message);
  bool RegisterMessage(const Descriptor& desc);
  bool RegisterMessage(const FileDescriptorProto& file_desc_proto);
  bool RegisterMessage(const std::string& proto_desc_str);

  // 获取 descriptor 序列化字符串
  static void GetDescriptorString(const google::protobuf::Message& message,
                                  std::string* desc_str);
  void GetDescriptorString(const std::string& type, std::string* desc_str);

  // 根据类型名动态创建消息实例
  google::protobuf::Message* GenerateMessageByType(const std::string& type) const;

  // 查找
  const Descriptor* FindMessageTypeByName(const std::string& type) const;
};
```

内部维护独立的 `DescriptorPool` 和 `DynamicMessageFactory`，支持运行时动态注册 proto 文件描述符并递归处理依赖关系。`RegisterMessage(const ProtoDesc&)` 会先注册所有 `dependencies`，再注册自身。

### MessageHeader

固定布局的消息头结构，用于带头部的序列化格式（HC 格式）：

```cpp
class MessageHeader {
  char magic_num_[8];     // "BDACBDAC"
  char seq_[8];           // 序列号
  char timestamp_ns_[8];  // 纳秒时间戳
  char src_id_[8];        // 源 ID
  char dst_id_[8];        // 目标 ID
  char msg_type_[129];    // 消息类型名
  char res_[19];          // 保留字段
  char content_size_[4];  // 内容大小
};
```

所有多字节字段使用网络字节序（big-endian）存储，通过 `htonl`/`ntohl` 转换。总大小为 192 字节。

### RawMessage

轻量级原始字节消息，不依赖 protobuf：

```cpp
struct RawMessage {
  std::string message;
  uint64_t timestamp;

  bool SerializeToArray(void* data, int size) const;
  bool SerializeToString(std::string* str) const;
  bool ParseFromArray(const void* data, int size);
  bool ParseFromString(const std::string& str);
  int ByteSize() const;
  static std::string TypeName();  // "apollo.cyber.message.RawMessage"
};
```

适用于不需要结构化解析的场景，如录制回放、消息转发等。

### PyMessageWrap

Python 消息包装器，为 Python 接口提供与 C++ 消息一致的序列化接口：

```cpp
class PyMessageWrap {
  std::string data_;
  std::string type_name_;

 public:
  bool SerializeToArray(void* data, int size) const;
  bool SerializeToString(std::string* output) const;
  bool ParseFromArray(const void* data, int size);
  bool ParseFromString(const std::string& msgstr);
  int ByteSize() const;
  static std::string TypeName();  // "apollo.cyber.message.PyMessage"
};
```

### ArenaMessageWrapper

Arena 内存管理的消息包装器，用于共享内存场景下的零拷贝传输：

```cpp
class ArenaMessageWrapper {
 public:
  template <typename MessageT>
  MessageT* GetMessage();       // 从 arena 获取消息指针

  template <typename MessageT>
  MessageT* SetMessage(const MessageT& message);  // 写入消息到 arena

  void* GetData();              // 获取原始数据指针
  bool FillMeta(void* meta, uint64_t size);        // 填充元数据（最大 128 字节）
  bool FillExtended(void* extended, uint64_t size); // 填充扩展数据（最大 256 字节）
};
```

内部数据结构 `ArenaMessageWrapperDataStruct` 总大小 1024 字节，包含 128 字节 meta 区和 256 字节 extended 区。通过 `ArenaManagerBase` 接口与底层 arena 内存管理器交互。

### ArenaManagerBase

Arena 内存管理器的抽象基类：

```cpp
class ArenaManagerBase {
 public:
  virtual uint64_t GetBaseAddress(const ArenaMessageWrapper* wrapper);
  virtual void* SetMessage(ArenaMessageWrapper* wrapper, const void* message) = 0;
  virtual void* GetMessage(ArenaMessageWrapper* wrapper) = 0;

  std::shared_ptr<ArenaMessageWrapper> CreateMessageWrapper();

  template <typename MessageT>
  MessageT* SetMessage(ArenaMessageWrapper* wrapper, const MessageT& message);
  template <typename MessageT>
  MessageT* GetMessage(ArenaMessageWrapper* wrapper);
};
```

## 数据流描述

```
发送端:
  用户消息(protobuf/RawMessage/PyMessage)
    → MessageTraits 编译期分发
    → SerializeToString / SerializeToArray / SerializeToArenaMessageWrapper
    → 传输层 (Transport)

接收端:
  传输层原始数据
    → ParseFromArray / ParseFromString / ParseFromHC / ParseFromArenaMessageWrapper
    → MessageTraits 编译期分发
    → 用户消息对象
```

`ParseFromHC` 流程会先解析 `MessageHeader`（校验 magic number、提取 content_size），再对 payload 部分调用 `ParseFromArray`。

## 配置方式

Message 模块本身无独立配置文件。消息类型通过以下方式注册：

1. protobuf 消息在编译时自动注册到全局 `DescriptorPool`
2. 运行时可通过 `ProtobufFactory::RegisterMessage()` 动态注册
3. `proto_desc.proto` 定义了 `ProtoDesc` 消息，用于序列化传输 proto 描述符及其依赖链：

```protobuf
// cyber/proto/proto_desc.proto
message ProtoDesc {
  optional string desc = 1;
  repeated ProtoDesc dependencies = 2;
}
```

## 与其他模块的关系

| 模块 | 关系 |
|------|------|
| Transport | 传输层通过 `message_traits.h` 中的自由函数完成消息的序列化/反序列化，是 Message 模块的主要消费者 |
| SHM Transport | 使用 `ArenaMessageWrapper` + `ArenaManagerBase` 实现共享内存零拷贝传输 |
| Node/Reader/Writer | 上层 API 通过模板参数传入消息类型，由 MessageTraits 自动适配 |
| Service Discovery | 使用 `ProtobufFactory::GetDescriptorString()` 获取消息描述符，用于拓扑发现时的类型匹配 |
| Record | 录制回放使用 `RawMessage` 作为通用消息容器 |
| Python API | 通过 `PyMessageWrap` 桥接 Python 与 C++ 消息系统 |
