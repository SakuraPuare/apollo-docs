---
title: Record - 数据录制与回放
---

# Record - 数据录制与回放

## 模块职责概述

Record 模块负责 Apollo Cyber 框架中 channel 消息的录制（序列化到文件）和回放（从文件反序列化）。它提供了一套完整的 API，用于将运行时的 protobuf 消息持久化为 `.record` 文件，并支持按时间范围、channel 过滤等方式进行回放。该模块是离线数据分析、算法调试和仿真回放的基础设施。

## 核心类/接口说明

### RecordBase

所有 Reader/Writer 的抽象基类，定义了统一的元数据查询接口。

```cpp
class RecordBase {
 public:
  virtual uint64_t GetMessageNumber(const std::string& channel_name) const = 0;
  virtual const std::string& GetMessageType(const std::string& channel_name) const = 0;
  virtual const std::string& GetProtoDesc(const std::string& channel_name) const = 0;
  virtual std::set<std::string> GetChannelList() const = 0;
  const proto::Header& GetHeader() const;
  const std::string GetFile() const;

 protected:
  std::string file_;
  proto::Header header_;
  bool is_opened_ = false;
};
```

### RecordWriter

数据录制器，将 channel 消息写入 record 文件。支持文件分段（segment）和 chunk 分块。

```cpp
class RecordWriter : public RecordBase {
 public:
  RecordWriter();
  explicit RecordWriter(const proto::Header& header);

  bool Open(const std::string& file);
  void Close();

  // 写入 channel 元信息
  bool WriteChannel(const std::string& name,
                    const std::string& type,
                    const std::string& proto_desc);

  // 写入原始字符串消息
  bool WriteMessage(const std::string& channel_name,
                    const std::string& content,
                    const uint64_t time_nanosec);

  // 写入类型化消息（模板方法）
  template <typename MessageT>
  bool WriteMessage(const std::string& channel_name,
                    const MessageT& message,
                    const uint64_t time_nanosec,
                    const std::string& proto_desc = "");

  bool SetSizeOfFileSegmentation(uint64_t size_kilobytes);
  bool SetIntervalOfFileSegmentation(uint64_t time_sec);
};
```

关键行为：
- `Open()` 时根据 Header 中的 `segment_interval` / `segment_raw_size` 决定是否启用文件分段，分段文件名格式为 `原始文件名.00000.YYYYMMDDHHmmss`
- `WriteMessage()` 内部检查是否需要分段（`SplitOutfile()`），基于时间间隔或文件大小触发
- 模板版本的 `WriteMessage()` 会自动序列化 protobuf 消息并校验消息类型一致性

### RecordReader

数据回放读取器，从 record 文件中按 chunk 顺序读取消息。

```cpp
class RecordReader : public RecordBase {
 public:
  explicit RecordReader(const std::string& file);

  bool IsValid() const;
  bool ReadMessage(RecordMessage* message,
                   uint64_t begin_time = 0,
                   uint64_t end_time = UINT64_MAX);
  void Reset();
};
```

关键行为：
- 构造时读取文件 Header 和 Index，建立 channel 信息映射
- `ReadMessage()` 按 chunk 顺序逐条读取，支持时间范围过滤
- 内部通过 `ReadNextChunk()` 逐段读取 ChunkHeader 和 ChunkBody，可跳过不在时间范围内的 chunk

### RecordViewer

多文件聚合查看器，支持跨多个 RecordReader 按时间排序遍历消息，提供 STL 风格的迭代器接口。

```cpp
class RecordViewer {
 public:
  RecordViewer(const RecordReaderPtr& reader,
               uint64_t begin_time = 0,
               uint64_t end_time = UINT64_MAX,
               const std::set<std::string>& channels = {});

  RecordViewer(const std::vector<RecordReaderPtr>& readers,
               uint64_t begin_time = 0,
               uint64_t end_time = UINT64_MAX,
               const std::set<std::string>& channels = {});

  Iterator begin();
  Iterator end();
  bool IsValid() const;
  std::set<std::string> GetChannelList() const;
};
```

使用示例：

```cpp
auto reader = std::make_shared<RecordReader>("data.record");
RecordViewer viewer(reader, 0, UINT64_MAX, {"/apollo/perception"});
for (auto& msg : viewer) {
  // msg.channel_name, msg.content, msg.time
}
```

### RecordMessage

消息的基础数据结构：

```cpp
struct RecordMessage {
  std::string channel_name;  // channel 名称
  std::string content;       // 序列化后的消息内容
  uint64_t time;             // 纳秒级时间戳
};
```

### HeaderBuilder

Record 文件头构建器，提供默认参数和自定义参数两种构建方式。

```cpp
class HeaderBuilder {
 public:
  static proto::Header GetHeader();
  static proto::Header GetHeaderWithSegmentParams(
      uint64_t segment_interval, uint64_t segment_raw_size);
  static proto::Header GetHeaderWithChunkParams(
      uint64_t chunk_interval, uint64_t chunk_raw_size);
};
```

默认参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CHUNK_INTERVAL` | 20s | 单个 chunk 的最大时间跨度 |
| `SEGMENT_INTERVAL` | 60s | 单个文件段的最大时间跨度 |
| `CHUNK_RAW_SIZE` | 16 MB | 单个 chunk 的最大原始数据量 |
| `SEGMENT_RAW_SIZE` | 2 GB | 单个文件段的最大原始数据量 |
| `COMPRESS_TYPE` | COMPRESS_NONE | 压缩方式（支持 BZ2、LZ4） |

## 数据流描述

### 录制流程

```
应用层 WriteMessage()
  → RecordWriter 检查 channel 注册 / 消息类型校验
  → 序列化为 SingleMessage
  → RecordFileWriter 将消息追加到当前 Chunk
  → Chunk 满（时间/大小）时 flush：写入 ChunkHeader + ChunkBody
  → Segment 满时 SplitOutfile()：关闭当前文件，创建新文件
  → Close() 时写入 Index 段并更新 Header
```

### 回放流程

```
RecordReader 构造
  → 打开文件，读取 Header
  → 读取 Index，建立 channel_info_ 映射
  → Reset() 回到数据起始位置

ReadMessage() 循环
  → 从当前 ChunkBody 中按序读取 SingleMessage
  → 当前 Chunk 读完后 ReadNextChunk()
  → 遇到 SECTION_INDEX 表示文件结束
```

### 文件格式

Record 文件由多个 Section 顺序排列组成，每个 Section 包含一个固定大小的 Section 头（type + size）和对应的 protobuf 序列化数据：

```
[Header (固定 2048 字节)]
[Channel Section]*
[ChunkHeader Section + ChunkBody Section]*
[Index Section]
```

Section 类型定义（`proto::SectionType`）：

| 类型 | 值 | 说明 |
|------|----|------|
| `SECTION_HEADER` | 0 | 文件头，固定占 2048 字节 |
| `SECTION_CHUNK_HEADER` | 1 | Chunk 元信息（时间范围、消息数、原始大小） |
| `SECTION_CHUNK_BODY` | 2 | Chunk 数据体（包含多条 SingleMessage） |
| `SECTION_INDEX` | 3 | 索引段，位于文件末尾 |
| `SECTION_CHANNEL` | 4 | Channel 定义（名称、消息类型、proto 描述） |

## 配置方式

Record 模块通过 `proto::Header` 进行配置，主要参数：

- `chunk_interval` / `chunk_raw_size`：控制 chunk 的切分粒度
- `segment_interval` / `segment_raw_size`：控制文件分段策略，设为 0 则不分段
- `compress`：压缩类型（`COMPRESS_NONE` / `COMPRESS_BZ2` / `COMPRESS_LZ4`）

可通过 `HeaderBuilder` 的静态方法快速构建，也可直接构造 `proto::Header` 对象传入 `RecordWriter`。

## 与其他模块的关系

- **proto 模块**：Record 文件格式完全由 `cyber/proto/record.proto` 定义，包括 Header、Channel、ChunkHeader、ChunkBody、SingleMessage、Index 等消息类型
- **message 模块**：`RecordWriter` 使用 `message::GetMessageName<T>()` 获取消息类型名，使用 protobuf 的 `SerializeToString` 进行序列化
- **cyber_recorder 工具**：基于 Record 模块实现的命令行录制/回放工具
- **Transport 层**：录制时通常配合 Reader 订阅 channel 消息，回放时通过 Writer 发布消息
