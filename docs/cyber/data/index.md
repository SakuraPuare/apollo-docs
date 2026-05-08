---
title: "Data 数据分发与融合"
---

# Data

> 源码路径：`cyber/data/`

## 概述

Data 模块是 CyberRT 的消息缓存与分发层，负责将 transport 层收到的消息写入环形缓冲区，并通知对应的协程取数据。支持单通道读取和多通道融合（AllLatest 策略），是连接 transport 与 scheduler 的桥梁。

## 架构

```text
Transport (Reader 收到消息)
    │
    ▼
DataDispatcher<T>::Dispatch(channel_id, msg)
    │
    ├── 写入所有注册的 CacheBuffer
    │
    └── DataNotifier::Notify(channel_id)
            │
            ▼
        Notifier::callback() → 唤醒 CRoutine
            │
            ▼
DataVisitor::TryFetch() → 从 ChannelBuffer 读取
    │
    └── (多通道) DataFusion::Fusion() → AllLatest 策略
```

## 核心类

### CacheBuffer\<T\>

> 源码文件：`cyber/data/cache_buffer.h`

固定容量的环形缓冲区：

```cpp
template <typename T>
class CacheBuffer {
 public:
  explicit CacheBuffer(uint64_t size);

  T& operator[](const uint64_t& pos);
  const T& at(const uint64_t& pos) const;

  uint64_t Head() const;
  uint64_t Tail() const;
  uint64_t Size() const;
  bool Empty() const;
  bool Full() const;

  void Fill(const T& value);
  void SetFusionCallback(const FusionCallback& callback);
  std::mutex& Mutex();

 private:
  uint64_t GetIndex(const uint64_t& pos) const { return pos % capacity_; }
  uint64_t head_ = 0;
  uint64_t tail_ = 0;
  std::vector<T> buffer_;
  FusionCallback fusion_callback_;
};
```

`Fill()` 在缓冲区满时覆盖最旧数据。若设置了 `fusion_callback_`，则由回调决定如何处理新数据（用于多通道融合场景）。

### ChannelBuffer\<T\>

> 源码文件：`cyber/data/channel_buffer.h`

绑定 channel_id 与 CacheBuffer，提供按索引读取接口：

```cpp
template <typename T>
class ChannelBuffer {
 public:
  bool Fetch(uint64_t* index, std::shared_ptr<T>& m);
  bool Latest(std::shared_ptr<T>& m);
  bool FetchMulti(uint64_t fetch_size, std::vector<std::shared_ptr<T>>* vec);
  uint64_t channel_id() const;
};
```

`Fetch()` 按 index 顺序读取，检测溢出时跳到最新并打印警告。

### DataDispatcher\<T\>

> 源码文件：`cyber/data/data_dispatcher.h`

单例，负责将消息分发到所有注册的 CacheBuffer：

```cpp
template <typename T>
class DataDispatcher {
 public:
  void AddBuffer(const ChannelBuffer<T>& channel_buffer);
  bool Dispatch(const uint64_t channel_id, const std::shared_ptr<T>& msg);
};
```

### DataNotifier

> 源码文件：`cyber/data/data_notifier.h`

单例，管理 channel_id → Notifier 回调的映射：

```cpp
struct Notifier {
  std::function<void()> callback;
};

class DataNotifier {
 public:
  void AddNotifier(uint64_t channel_id, const std::shared_ptr<Notifier>& notifier);
  bool Notify(const uint64_t channel_id);
};
```

`Notify()` 遍历该 channel 的所有 Notifier 并调用回调（通常是唤醒协程）。

### DataVisitorBase

> 源码文件：`cyber/data/data_visitor_base.h`

```cpp
class DataVisitorBase {
 public:
  DataVisitorBase() : notifier_(new Notifier()) {}
  void RegisterNotifyCallback(std::function<void()>&& callback);
 protected:
  uint64_t next_msg_index_ = 0;
  DataNotifier* data_notifier_ = DataNotifier::Instance();
  std::shared_ptr<Notifier> notifier_;
};
```

### DataVisitor\<M0, M1, M2, M3\>

> 源码文件：`cyber/data/data_visitor.h`

模板特化支持 1~4 路消息通道：

```cpp
// 单通道特化
template <typename M0>
class DataVisitor<M0, NullType, NullType, NullType> : public DataVisitorBase {
 public:
  explicit DataVisitor(const VisitorConfig& configs);
  bool TryFetch(std::shared_ptr<M0>& m0);
};

// 双通道特化（使用 AllLatest 融合）
template <typename M0, typename M1>
class DataVisitor<M0, M1, NullType, NullType> : public DataVisitorBase {
 public:
  explicit DataVisitor(const std::vector<VisitorConfig>& configs);
  bool TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1);
};
```

## 核心函数

### DataDispatcher\<T\>::Dispatch()

```cpp
template <typename T>
bool DataDispatcher<T>::Dispatch(const uint64_t channel_id,
                                 const std::shared_ptr<T>& msg) {
  BufferVector* buffers = nullptr;
  if (apollo::cyber::IsShutdown()) { return false; }
  if (buffers_map_.Get(channel_id, &buffers)) {
    for (auto& buffer_wptr : *buffers) {
      if (auto buffer = buffer_wptr.lock()) {
        std::lock_guard<std::mutex> lock(buffer->Mutex());
        buffer->Fill(msg);
      }
    }
  } else { return false; }
  return notifier_->Notify(channel_id);
}
```

**职责**：将消息写入所有订阅该 channel 的缓冲区，然后触发通知。

### CacheBuffer\<T\>::Fill()

```cpp
void Fill(const T& value) {
  if (fusion_callback_) {
    fusion_callback_(value);
  } else {
    if (Full()) {
      buffer_[GetIndex(head_)] = value;
      ++head_; ++tail_;
    } else {
      buffer_[GetIndex(tail_ + 1)] = value;
      ++tail_;
    }
  }
}
```

**职责**：环形写入，满时覆盖最旧元素。

### ChannelBuffer\<T\>::Fetch()

```cpp
template <typename T>
bool ChannelBuffer<T>::Fetch(uint64_t* index, std::shared_ptr<T>& m) {
  std::lock_guard<std::mutex> lock(buffer_->Mutex());
  if (buffer_->Empty()) { return false; }
  if (*index == 0) { *index = buffer_->Tail(); }
  else if (*index == buffer_->Tail() + 1) { return false; }
  else if (*index < buffer_->Head()) { *index = buffer_->Tail(); }
  m = buffer_->at(*index);
  return true;
}
```

**职责**：按序号读取消息，处理溢出（跳到最新）和无新数据的情况。

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `VisitorConfig::channel_id` | uint64 | 通道哈希 ID |
| `VisitorConfig::queue_size` | uint32 | 缓冲区容量 |

## 调用关系

- **上游**：`Transport` 层的 Reader 收到消息后调用 `DataDispatcher::Dispatch`
- **下游**：`CRoutine`（协程）通过 `DataVisitor::TryFetch` 获取数据
- **依赖**：`CacheBuffer`、`DataNotifier`、`fusion/AllLatest`
- **被依赖**：`RoutineFactory` 使用 DataVisitor 构建数据驱动协程
