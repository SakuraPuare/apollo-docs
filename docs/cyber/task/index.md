---
title: "Task"
---

# Task

> 源码路径: `cyber/task/`

## 概述

Task 模块为 Cyber RT 提供异步任务投递与协程感知的线程控制能力。模块分为两层：

- **TaskManager** — 单例任务管理器，内部维护一个有界队列（`BoundedQueue`），启动时根据调度器的 `TaskPoolSize` 创建等量协程，从队列中循环取任务执行。
- **task.h 便捷函数** — 提供 `Async`、`Yield`、`SleepFor`、`USleep` 等全局入口，自动判断当前上下文是协程还是普通线程，选择对应的行为。

整体设计使上层模块无需关心底层是协程调度还是 `std::async`，只需调用 `Async()` 即可投递任务。

## 核心类

### TaskManager

```cpp
class TaskManager {
 public:
  virtual ~TaskManager();
  void Shutdown();

  template <typename F, typename... Args>
  auto Enqueue(F&& func, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type>;

 private:
  uint32_t num_threads_ = 0;
  uint32_t task_queue_size_ = 1000;
  std::atomic<bool> stop_ = {false};
  std::vector<uint64_t> tasks_;
  std::shared_ptr<base::BoundedQueue<std::function<void()>>> task_queue_;
  DECLARE_SINGLETON(TaskManager);
};
```

- 通过 `DECLARE_SINGLETON` 宏声明为全局单例
- 构造函数中初始化容量为 1000 的有界队列（`BlockWaitStrategy` 阻塞等待策略），然后按 `TaskPoolSize` 创建协程消费任务
- 每个协程执行一个循环：从队列 Dequeue，若无任务则 `HangUp` 挂起等待唤醒

## 核心函数

### Async

```cpp
template <typename F, typename... Args>
static auto Async(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;
```

**职责：** 异步投递任务，返回 `std::future` 用于获取结果。

**输入：** 可调用对象 `f` 及其参数 `args...`。

**输出：** `std::future<result_type>`。

**关键步骤：**

1. 检查 `GlobalData::IsRealityMode()` — 仿真模式下走 `TaskManager::Enqueue`，由协程调度器执行
2. 非仿真模式下退化为 `std::async(std::launch::async, ...)`，直接在独立线程中执行

### Yield

```cpp
static inline void Yield();
```

**职责：** 让出当前执行权。

**关键步骤：**

1. 若当前在协程中（`GetCurrentRoutine()` 非空），调用 `CRoutine::Yield()` 让出协程
2. 否则调用 `std::this_thread::yield()` 让出操作系统线程

### SleepFor

```cpp
template <typename Rep, typename Period>
static void SleepFor(const std::chrono::duration<Rep, Period>& sleep_duration);
```

**职责：** 按指定时长休眠，协程感知。

**输入：** `sleep_duration` — `std::chrono::duration` 类型的休眠时长。

**关键步骤：**

1. 获取当前协程指针
2. 若不在协程中，使用 `std::this_thread::sleep_for`
3. 若在协程中，使用 `CRoutine::Sleep`，避免阻塞底层线程

### USleep

```cpp
static inline void USleep(useconds_t usec);
```

**职责：** 以微秒为单位休眠，协程感知。

**输入：** `usec` — 微秒数。

**关键步骤：** 逻辑与 `SleepFor` 一致，将微秒转为 `Duration` 后委托给协程或线程的休眠接口。

### TaskManager::Enqueue

```cpp
template <typename F, typename... Args>
auto Enqueue(F&& func, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;
```

**职责：** 将任务封装后入队，并通知调度器唤醒消费协程。

**输入：** 可调用对象 `func` 及其参数 `args...`。

**输出：** `std::future<result_type>`。

**关键步骤：**

1. 用 `std::packaged_task` 包装任务
2. 若管理器未停止（`!stop_`），将 lambda 入队 `task_queue_->Enqueue`
3. 遍历所有消费协程 ID，逐一调用 `scheduler::Instance()->NotifyTask` 唤醒
4. 返回 `task->get_future()` 供调用方获取结果

### TaskManager::Shutdown

```cpp
void Shutdown();
```

**职责：** 优雅关闭任务管理器。

**关键步骤：**

1. `stop_.exchange(true)` 原子置停止标志，防止重复关闭
2. 遍历所有消费协程，调用 `scheduler::Instance()->RemoveTask` 移除

## 配置项

| 配置项 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `TaskPoolSize` | `uint32_t` | 由调度器决定 | 消费者协程数量，取自 `scheduler::Instance()->TaskPoolSize()` |
| `task_queue_size_` | `uint32_t` | `1000` | 有界队列最大容量 |
| 等待策略 | — | `BlockWaitStrategy` | 队列为空时消费者阻塞等待 |

## 调用关系

```text
Async()
  ├─ [RealityMode] → TaskManager::Enqueue() → BoundedQueue → 消费协程执行
  └─ [非 RealityMode] → std::async()

TaskManager 构造
  → BoundedQueue::Init()
  → scheduler::CreateTask() × num_threads_   // 创建消费协程
      → 循环: BoundedQueue::Dequeue → 执行任务
              无任务时 CRoutine::HangUp

TaskManager::Enqueue()
  → BoundedQueue::Enqueue()
  → scheduler::NotifyTask() × num_threads_   // 唤醒协程

Yield() / SleepFor() / USleep()
  ├─ [协程上下文] → CRoutine::Yield() / CRoutine::Sleep()
  └─ [线程上下文] → std::this_thread::yield() / sleep_for()

TaskManager::Shutdown()
  → scheduler::RemoveTask() × num_threads_
```
