---
title: "CRoutine 协程"
---

# CRoutine

> 源码路径：`cyber/croutine/`

## 概述

CRoutine 是 CyberRT 的用户态协程实现，为调度器提供轻量级执行单元。每个协程拥有独立的 2MB 栈空间，通过汇编级上下文切换（`ctx_swap`）实现协作式调度，避免内核线程切换开销。协程与 DataVisitor 配合，实现数据驱动的消息处理循环。

## 架构

```text
Scheduler
  └── Processor (线程)
        └── CRoutine (协程，N:1 映射到 Processor)
              ├── RoutineContext (独立栈 + 寄存器)
              └── RoutineFunc (用户回调)

RoutineFactory ── 创建 ──► CRoutine
     └── DataVisitor (数据触发)
```

## 核心类

### CRoutine

> 源码文件：`cyber/croutine/croutine.h`、`cyber/croutine/croutine.cc`

```cpp
class CRoutine {
 public:
  explicit CRoutine(const RoutineFunc &func);

  static void Yield();
  static void Yield(const RoutineState &state);
  static CRoutine *GetCurrentRoutine();

  bool Acquire();
  void Release();
  void SetUpdateFlag();

  RoutineState Resume();
  RoutineState UpdateState();

  void Run();
  void Stop();
  void Wake();
  void HangUp();
  void Sleep(const Duration &sleep_duration);

  // getter/setter
  RoutineState state() const;
  uint64_t id() const;
  const std::string &name() const;
  int processor_id() const;
  uint32_t priority() const;
};
```

**状态机**：

| 状态 | 含义 |
| --- | --- |
| `READY` | 可被调度执行 |
| `FINISHED` | 执行完毕 |
| `SLEEP` | 定时休眠，到期自动唤醒 |
| `IO_WAIT` | 等待 IO 事件 |
| `DATA_WAIT` | 等待数据到达 |

### RoutineContext

> 源码文件：`cyber/croutine/detail/routine_context.h`

```cpp
constexpr size_t STACK_SIZE = 2 * 1024 * 1024;

struct RoutineContext {
  char stack[STACK_SIZE];
  char* sp = nullptr;
};

void MakeContext(const func& f1, const void* arg, RoutineContext* ctx);
inline void SwapContext(char** src_sp, char** dest_sp);
```

每个协程分配 2MB 栈。`SwapContext` 调用汇编函数 `ctx_swap`（x86_64 或 aarch64）完成寄存器保存/恢复和栈指针切换。

### RoutineFactory

> 源码文件：`cyber/croutine/routine_factory.h`

```cpp
class RoutineFactory {
 public:
  CreateRoutineFunc create_routine;
  std::shared_ptr<data::DataVisitorBase> GetDataVisitor() const;
  void SetDataVisitor(const std::shared_ptr<data::DataVisitorBase>& dv);
};

template <typename M0, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0>>& dv);
```

**职责**：将用户回调 + DataVisitor 封装为协程工厂。生成的协程函数内部是无限循环：等待数据 → 调用回调 → Yield。支持 1~4 路消息融合。

## 核心函数

### CRoutine::CRoutine()

```cpp
CRoutine::CRoutine(const std::function<void()> &func) : func_(func) {
  std::call_once(pool_init_flag, [&]() {
    uint32_t routine_num = common::GlobalData::Instance()->ComponentNums();
    context_pool.reset(new base::CCObjectPool<RoutineContext>(routine_num));
  });
  context_ = context_pool->GetObject();
  if (context_ == nullptr) {
    context_.reset(new RoutineContext());
  }
  MakeContext(CRoutineEntry, this, context_.get());
  state_ = RoutineState::READY;
}
```

**职责**：从对象池获取 RoutineContext，初始化栈帧使首次 Resume 时跳转到 `CRoutineEntry`。

### CRoutine::Resume()

```cpp
RoutineState CRoutine::Resume() {
  if (cyber_unlikely(force_stop_)) {
    state_ = RoutineState::FINISHED;
    return state_;
  }
  if (cyber_unlikely(state_ != RoutineState::READY)) {
    AERROR << "Invalid Routine State!";
    return state_;
  }
  current_routine_ = this;
  SwapContext(GetMainStack(), GetStack());
  current_routine_ = nullptr;
  return state_;
}
```

**职责**：从主线程栈切换到协程栈执行，协程 Yield 后返回。
**前置条件**：状态必须为 READY。

### CRoutine::Yield()

```cpp
inline void CRoutine::Yield(const RoutineState &state) {
  auto routine = GetCurrentRoutine();
  routine->set_state(state);
  SwapContext(GetCurrentRoutine()->GetStack(), GetMainStack());
}
```

**职责**：协程主动让出 CPU，切换回主线程栈（Processor 调度循环）。

### CRoutine::UpdateState()

```cpp
inline RoutineState CRoutine::UpdateState() {
  if (state_ == RoutineState::SLEEP &&
      std::chrono::steady_clock::now() > wake_time_) {
    state_ = RoutineState::READY;
    return state_;
  }
  if (!updated_.test_and_set(std::memory_order_release)) {
    if (state_ == RoutineState::DATA_WAIT || state_ == RoutineState::IO_WAIT) {
      state_ = RoutineState::READY;
    }
  }
  return state_;
}
```

**职责**：调度器轮询时调用，检查是否满足唤醒条件（定时到期或异步事件通知）。

### CRoutine::Acquire() / Release()

```cpp
inline bool CRoutine::Acquire() {
  return !lock_.test_and_set(std::memory_order_acquire);
}
inline void CRoutine::Release() {
  lock_.clear(std::memory_order_release);
}
```

**职责**：自旋锁语义，防止多个 Processor 同时 Resume 同一协程（work-stealing 场景）。

## 配置

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `scheduler_conf.routine_num` | uint32 | 协程上下文对象池大小 |

## 调用关系

- **被调用方**：`Scheduler` / `Processor` 创建和调度 CRoutine
- **依赖**：`RoutineContext`（栈管理）、`ctx_swap`（汇编上下文切换）、`CCObjectPool`（对象池）
- **RoutineFactory** 被 `Component` 框架调用，将消息回调包装为协程
