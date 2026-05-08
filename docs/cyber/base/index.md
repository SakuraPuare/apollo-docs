---
title: "基础库 (Base)"
---

# Base

> 源码路径：`cyber/base/`

## 概述

`cyber/base` 是 CyberRT 的底层基础设施库，提供高性能、无锁的并发数据结构和工具类。这些组件被调度器、传输层、数据缓存等上层模块广泛使用，是整个框架的性能基石。

## 架构

| 组件 | 文件 | 职责 |
| --- | --- | --- |
| BoundedQueue | `bounded_queue.h` | 无锁有界队列（MPSC） |
| UnboundedQueue | `unbounded_queue.h` | 无锁无界队列 |
| AtomicHashMap | `atomic_hash_map.h` | 无锁定长哈希表 |
| ObjectPool | `object_pool.h` | 预分配对象池 |
| ThreadPool | `thread_pool.h` | 线程池 |
| Signal/Slot | `signal.h` | 信号槽机制 |
| WaitStrategy | `wait_strategy.h` | 等待策略抽象 |
| Macros | `macros.h` | 分支预测、缓存行对齐等宏 |

## 核心类

### BoundedQueue\<T\>

```cpp
template <typename T>
class BoundedQueue {
 public:
  bool Init(uint64_t size);
  bool Init(uint64_t size, WaitStrategy* strategy);
  bool Enqueue(const T& element);
  bool Enqueue(T&& element);
  bool WaitEnqueue(const T& element);
  bool WaitEnqueue(T&& element);
  bool Dequeue(T* element);
  bool WaitDequeue(T* element);
  uint64_t Size();
  bool Empty();
  void SetWaitStrategy(WaitStrategy* strategy);
  void BreakAllWait();
};
```

**职责**：无锁有界环形队列，支持多生产者并发入队。

**实现要点**：

- 使用 `head_`、`tail_`、`commit_` 三个原子变量实现无锁协议
- `tail_` 通过 CAS 竞争入队位置，`commit_` 保证写入顺序可见性
- 内存按 `CACHELINE_SIZE`（64字节）对齐，避免 false sharing
- 索引计算使用除法替代取模（`num - (num / pool_size_) * pool_size_`）

### AtomicHashMap\<K, V, TableSize\>

```cpp
template <typename K, typename V, std::size_t TableSize = 128>
class AtomicHashMap {
 public:
  bool Has(K key);
  bool Get(K key, V** value);
  bool Get(K key, V* value);
  void Set(K key);
  void Set(K key, const V& value);
  void Set(K key, V&& value);
};
```

**职责**：无锁定长哈希表，key 必须为整数类型，TableSize 必须为 2 的幂。

**实现要点**：

- 使用位与（`key & (TableSize - 1)`）代替取模做哈希
- 每个 bucket 内部是链表，通过 CAS 实现无锁插入
- 适用于 key 为 ID 类型的高频查找场景

### ObjectPool\<T\>

```cpp
template <typename T>
class ObjectPool : public std::enable_shared_from_this<ObjectPool<T>> {
 public:
  explicit ObjectPool(uint32_t num_objects, Args&&... args);
  ObjectPool(uint32_t num_objects, InitFunc f, Args&&... args);
  std::shared_ptr<T> GetObject();
};
```

**职责**：预分配固定数量对象，通过 `shared_ptr` 自定义 deleter 实现自动回收。

**实现要点**：

- 构造时一次性 `calloc` 分配所有对象内存（arena 模式）
- 空闲链表管理可用对象
- `GetObject()` 返回的 `shared_ptr` 析构时自动归还到池中

### ThreadPool

```cpp
class ThreadPool {
 public:
  explicit ThreadPool(std::size_t thread_num, std::size_t max_task_num = 1000);
  template <typename F, typename... Args>
  auto Enqueue(F&& f, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type>;
};
```

**职责**：固定大小线程池，内部使用 `BoundedQueue` + `BlockWaitStrategy` 作为任务队列。

**关键逻辑**：

- 工作线程循环调用 `task_queue_.WaitDequeue()` 阻塞等待任务
- `Enqueue` 将 callable 包装为 `packaged_task`，返回 `std::future`
- 析构时设置 `stop_` 标志并调用 `BreakAllWait()` 唤醒所有线程

### Signal\<Args...\>

```cpp
template <typename... Args>
class Signal {
 public:
  using Callback = std::function<void(Args...)>;
  Connection<Args...> Connect(const Callback& cb);
  bool Disconnect(const Connection<Args...>& conn);
  void DisconnectAllSlots();
  void operator()(Args... args);
};
```

**职责**：线程安全的信号槽机制，用于组件间解耦通信。

**实现要点**：

- `Connect()` 创建 `Slot` 并返回 `Connection` 句柄
- `operator()` 触发时复制 slot 列表到局部变量，避免持锁回调
- 延迟清理已断开的 slot（`ClearDisconnectedSlots`）

## WaitStrategy 体系

```cpp
class WaitStrategy {
 public:
  virtual void NotifyOne() {}
  virtual void BreakAllWait() {}
  virtual bool EmptyWait() = 0;
};
```

| 策略 | 行为 | 适用场景 |
| --- | --- | --- |
| BlockWaitStrategy | condition_variable 阻塞等待 | 低 CPU、高延迟容忍 |
| SleepWaitStrategy | sleep 固定微秒（默认 10ms） | 中等延迟 |
| YieldWaitStrategy | `std::this_thread::yield()` | 低延迟、中 CPU |
| BusySpinWaitStrategy | 空循环 | 最低延迟、最高 CPU |
| TimeoutBlockWaitStrategy | 带超时的 condition_variable | 需要超时退出 |

## 工具宏

源码文件：`macros.h`

| 宏 | 作用 |
| --- | --- |
| `cyber_likely(x)` | 分支预测提示（期望为真） |
| `cyber_unlikely(x)` | 分支预测提示（期望为假） |
| `CACHELINE_SIZE` | 缓存行大小，64 字节 |
| `DEFINE_TYPE_TRAIT` | 编译期检测类是否有某成员函数 |
| `cpu_relax()` | 自旋等待时让出 CPU 流水线（x86: `rep; nop`，ARM: `yield`） |

## 调用关系

- **被调用方**：`cyber/scheduler`（调度队列）、`cyber/transport`（消息缓冲）、`cyber/data`（数据缓存）、`cyber/croutine`（协程池）等几乎所有上层模块
- **依赖**：仅依赖 C++ 标准库和 POSIX 线程原语，无外部依赖
