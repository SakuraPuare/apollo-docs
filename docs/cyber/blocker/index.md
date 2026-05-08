---
title: "Blocker 进程内消息缓存"
---

# Blocker

> 源码路径：`cyber/blocker/`

## 概述

Blocker 模块提供进程内（intra-process）消息发布/订阅与缓存机制。当 Writer 和 Reader 位于同一进程时，消息无需经过共享内存或网络传输，而是通过 Blocker 在内存中直接传递，同时保留一定深度的历史消息队列供 Reader 按需观测。

## 架构

```text
IntraWriter ──Publish──► BlockerManager ──► Blocker<T> ──Notify──► IntraReader
                              │                  │
                              │  blockers_ map   │  published_msg_queue_
                              │  (channel→blocker)│  observed_msg_queue_
                              ▼                  ▼
                         Observe()/Reset()   Subscribe()/Unsubscribe()
```

核心组件：

| 类 | 职责 |
| --- | --- |
| `BlockerBase` | 抽象基类，定义消息队列操作接口 |
| `BlockerAttr` | 配置结构体（capacity + channel_name） |
| `Blocker<T>` | 模板类，管理单个 channel 的消息队列与回调 |
| `BlockerManager` | 单例，管理所有 channel 的 Blocker 实例 |
| `IntraReader<T>` | 进程内 Reader，通过 BlockerManager 订阅消息 |
| `IntraWriter<T>` | 进程内 Writer，通过 BlockerManager 发布消息 |

## 核心类

### BlockerAttr

```cpp
struct BlockerAttr {
  BlockerAttr() : capacity(10), channel_name("") {}
  explicit BlockerAttr(const std::string& channel)
      : capacity(10), channel_name(channel) {}
  BlockerAttr(size_t cap, const std::string& channel)
      : capacity(cap), channel_name(channel) {}

  size_t capacity;
  std::string channel_name;
};
```

默认消息队列容量为 10。

### Blocker\<T\>

> 源码文件：`cyber/blocker/blocker.h`

```cpp
template <typename T>
class Blocker : public BlockerBase {
 public:
  using MessagePtr = std::shared_ptr<T>;
  using MessageQueue = std::list<MessagePtr>;
  using Callback = std::function<void(const MessagePtr&)>;

  explicit Blocker(const BlockerAttr& attr);

  void Publish(const MessageType& msg);
  void Publish(const MessagePtr& msg);

  void Observe() override;
  void ClearObserved() override;
  void ClearPublished() override;
  bool IsObservedEmpty() const override;
  bool IsPublishedEmpty() const override;

  bool Subscribe(const std::string& callback_id, const Callback& callback);
  bool Unsubscribe(const std::string& callback_id) override;

  const MessageType& GetLatestObserved() const;
  const MessagePtr GetLatestObservedPtr() const;
  const MessagePtr GetOldestObservedPtr() const;
  const MessagePtr GetLatestPublishedPtr() const;

  Iterator ObservedBegin() const;
  Iterator ObservedEnd() const;

  size_t capacity() const override;
  void set_capacity(size_t capacity) override;
};
```

## 核心函数

### Blocker\<T\>::Publish()

```cpp
template <typename T>
void Blocker<T>::Publish(const MessagePtr& msg) {
  Enqueue(msg);
  Notify(msg);
}
```

**职责**：将消息入队并通知所有订阅者。

**关键步骤**：

1. `Enqueue` — 将消息压入 `published_msg_queue_` 头部，超出 capacity 时弹出尾部
2. `Notify` — 遍历 `published_callbacks_` 调用所有已注册回调

### Blocker\<T\>::Observe()

```cpp
template <typename T>
void Blocker<T>::Observe() {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  observed_msg_queue_ = published_msg_queue_;
}
```

**职责**：将当前已发布队列快照到观测队列，供 Reader 安全遍历。

### Blocker\<T\>::Enqueue()

```cpp
template <typename T>
void Blocker<T>::Enqueue(const MessagePtr& msg) {
  if (attr_.capacity == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(msg_mutex_);
  published_msg_queue_.push_front(msg);
  while (published_msg_queue_.size() > attr_.capacity) {
    published_msg_queue_.pop_back();
  }
}
```

**职责**：维护固定容量的消息队列（FIFO，最新在前）。

### BlockerManager

> 源码文件：`cyber/blocker/blocker_manager.h`、`cyber/blocker/blocker_manager.cc`

```cpp
class BlockerManager {
 public:
  static const std::shared_ptr<BlockerManager>& Instance();

  template <typename T>
  bool Publish(const std::string& channel_name, const typename Blocker<T>::MessagePtr& msg);

  template <typename T>
  bool Subscribe(const std::string& channel_name, size_t capacity,
                 const std::string& callback_id,
                 const typename Blocker<T>::Callback& callback);

  template <typename T>
  bool Unsubscribe(const std::string& channel_name, const std::string& callback_id);

  template <typename T>
  std::shared_ptr<Blocker<T>> GetBlocker(const std::string& channel_name);

  template <typename T>
  std::shared_ptr<Blocker<T>> GetOrCreateBlocker(const BlockerAttr& attr);

  void Observe();
  void Reset();
};
```

**职责**：全局单例，按 channel_name 管理所有 Blocker 实例。

### BlockerManager::GetOrCreateBlocker()

```cpp
template <typename T>
std::shared_ptr<Blocker<T>> BlockerManager::GetOrCreateBlocker(const BlockerAttr& attr) {
  std::lock_guard<std::mutex> lock(blocker_mutex_);
  auto search = blockers_.find(attr.channel_name);
  if (search != blockers_.end()) {
    return std::dynamic_pointer_cast<Blocker<T>>(search->second);
  }
  auto blocker = std::make_shared<Blocker<T>>(attr);
  blockers_[attr.channel_name] = blocker;
  return blocker;
}
```

**职责**：懒创建 — 若 channel 对应的 Blocker 已存在则返回，否则新建并注册。

### IntraReader\<T\>

> 源码文件：`cyber/blocker/intra_reader.h`

继承 `cyber::Reader<T>`，通过 BlockerManager 订阅消息。

```cpp
template <typename MessageT>
class IntraReader : public Reader<MessageT> {
 public:
  bool Init() override;
  void Shutdown() override;
  void Observe() override;
  void Enqueue(const std::shared_ptr<MessageT>& msg) override;
  std::shared_ptr<MessageT> GetLatestObserved() const override;
  std::shared_ptr<MessageT> GetOldestObserved() const override;
};
```

`Init()` 调用 `BlockerManager::Subscribe` 注册回调；`Shutdown()` 调用 `Unsubscribe` 取消订阅。

### IntraWriter\<T\>

> 源码文件：`cyber/blocker/intra_writer.h`

继承 `cyber::Writer<T>`，通过 BlockerManager 发布消息。

```cpp
template <typename MessageT>
class IntraWriter : public Writer<MessageT> {
 public:
  bool Init() override;
  void Shutdown() override;
  bool Write(const MessageT& msg) override;
  bool Write(const MessagePtr& msg_ptr) override;
};
```

`Init()` 获取 BlockerManager 单例并创建对应 channel 的 Blocker；`Write()` 调用 `BlockerManager::Publish`。

## 调用关系

- **被调用方**：`cyber/node/` 中的 Node 在检测到同进程通信时创建 IntraWriter/IntraReader
- **依赖**：`cyber/node/reader.h`、`cyber/node/writer.h`（基类）
- **调用**：BlockerManager 被 IntraReader/IntraWriter 通过单例访问
