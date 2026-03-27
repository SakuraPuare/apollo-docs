---
title: Scheduler 调度器
---

# Scheduler 调度器

## 模块职责概述

Scheduler 是 Apollo CyberRT 的协程调度核心，负责将用户创建的 CRoutine（协程）分配到 Processor（处理线程）上执行。它提供两种调度策略：

- **Classic**：基于优先级的多组线程池调度，协程按优先级排队，同组内的多个 Processor 竞争执行。
- **Choreography**：编排式调度，允许将协程绑定到指定 Processor，实现确定性的执行顺序，未绑定的协程回退到 Classic 线程池。

调度器通过工厂单例模式创建，根据配置文件中的 `policy` 字段自动选择策略。

## 核心类与接口

### CRoutine — 协程

协程是调度的最小执行单元，基于用户态栈切换实现（非内核线程），每个协程拥有独立的 2MB 栈空间。

```cpp
// cyber/croutine/croutine.h
class CRoutine {
 public:
  explicit CRoutine(const RoutineFunc& func);

  // 让出 CPU，切换回主栈
  static void Yield();
  static void Yield(const RoutineState& state);

  // 恢复执行（由 Processor 调用）
  RoutineState Resume();

  // 状态更新：检查 SLEEP 超时或异步事件通知
  RoutineState UpdateState();

  // 无锁自旋获取/释放（防止多 Processor 同时 Resume）
  bool Acquire();
  void Release();

  // 协程挂起
  void HangUp();              // 进入 DATA_WAIT
  void Sleep(const Duration&); // 进入 SLEEP

  // 属性
  uint64_t id() const;
  uint32_t priority() const;
  int processor_id() const;
  const std::string& group_name();
};
```

协程状态机：

| 状态 | 含义 |
|------|------|
| `READY` | 就绪，可被 Processor 调度执行 |
| `FINISHED` | 执行完毕 |
| `SLEEP` | 定时休眠，到达 `wake_time_` 后自动转为 READY |
| `DATA_WAIT` | 等待数据到达（通过 `SetUpdateFlag` 唤醒） |
| `IO_WAIT` | 等待 IO 完成 |

栈切换通过汇编实现（`swap_x86_64.S` / `swap_aarch64.S`），`MakeContext` 在协程栈顶布局寄存器和返回地址，`SwapContext` 调用 `ctx_swap` 完成栈指针交换。

### RoutineFactory — 协程工厂

`RoutineFactory` 将 DataVisitor 的数据拉取逻辑与用户回调封装为协程函数。支持 1~4 路消息融合：

```cpp
// cyber/croutine/routine_factory.h
template <typename M0, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0>>& dv);
```

生成的协程函数内部循环：设置 `DATA_WAIT` -> 尝试 `TryFetch` -> 成功则调用回调并 `Yield(READY)` -> 失败则 `Yield()` 等待通知。

### Scheduler — 调度器基类

```cpp
// cyber/scheduler/scheduler.h
class Scheduler {
 public:
  // 创建任务（从 RoutineFactory 或裸函数）
  bool CreateTask(const RoutineFactory& factory, const std::string& name);
  bool CreateTask(std::function<void()>&& func, const std::string& name,
                  std::shared_ptr<DataVisitorBase> visitor = nullptr);

  // 通知协程有新数据
  bool NotifyTask(uint64_t crid);

  void Shutdown();

  // 子类必须实现
  virtual bool DispatchTask(const std::shared_ptr<CRoutine>&) = 0;
  virtual bool NotifyProcessor(uint64_t crid) = 0;
  virtual bool RemoveCRoutine(uint64_t crid) = 0;

 protected:
  std::unordered_map<uint64_t, std::shared_ptr<CRoutine>> id_cr_;
  std::vector<std::shared_ptr<ProcessorContext>> pctxs_;
  std::vector<std::shared_ptr<Processor>> processors_;
};
```

`CreateTask` 的流程：
1. 注册任务名，生成 hash id
2. 创建 CRoutine 并设置 id/name
3. 调用子类 `DispatchTask` 将协程放入对应队列
4. 若有 DataVisitor，注册通知回调 -> `NotifyProcessor`

### Processor — 处理线程

每个 Processor 对应一个 OS 线程，绑定一个 `ProcessorContext`，在循环中不断获取下一个就绪协程并执行：

```cpp
// cyber/scheduler/processor.cc — 核心循环
void Processor::Run() {
  while (running_) {
    auto croutine = context_->NextRoutine();
    if (croutine) {
      croutine->Resume();   // 切换到协程栈执行
      croutine->Release();  // 释放自旋锁
    } else {
      context_->Wait();     // 无就绪协程，条件变量等待
    }
  }
}
```

`Snapshot` 结构记录当前正在执行的协程名和开始时间，用于 `CheckSchedStatus` 监控。

### ProcessorContext — 处理上下文（抽象基类）

```cpp
// cyber/scheduler/processor_context.h
class ProcessorContext {
 public:
  virtual std::shared_ptr<CRoutine> NextRoutine() = 0;
  virtual void Wait() = 0;
  virtual void Shutdown();
};
```

两种策略分别实现了不同的 Context。

## Classic 调度策略

### 架构

Classic 策略将 Processor 分为多个 **SchedGroup**（调度组），每组拥有独立的多优先级队列和一组 Processor。同组内所有 Processor 共享同一个 `ClassicContext`（静态数据结构），竞争获取就绪协程。

```
SchedGroup "group1"          SchedGroup "group2"
┌─────────────────────┐      ┌─────────────────────┐
│ Prio 19: [cr, cr]   │      │ Prio 19: [cr]       │
│ Prio 18: [cr]       │      │ Prio 18: []         │
│ ...                  │      │ ...                  │
│ Prio  0: [cr, cr]   │      │ Prio  0: [cr]       │
├─────────────────────┤      ├─────────────────────┤
│ Processor 0..N-1    │      │ Processor 0..M-1    │
└─────────────────────┘      └─────────────────────┘
```

### ClassicContext

```cpp
// cyber/scheduler/policy/classic_context.h
static constexpr uint32_t MAX_PRIO = 20;

using MULTI_PRIO_QUEUE = std::array<CROUTINE_QUEUE, MAX_PRIO>;
using CR_GROUP = std::unordered_map<std::string, MULTI_PRIO_QUEUE>;

class ClassicContext : public ProcessorContext {
 public:
  std::shared_ptr<CRoutine> NextRoutine() override;
  void Wait() override;
  static void Notify(const std::string& group_name);
  static bool RemoveCRoutine(const std::shared_ptr<CRoutine>& cr);

  // 全局静态数据，所有 ClassicContext 实例共享
  static CR_GROUP cr_group_;       // group_name -> 20 级优先级队列
  static RQ_LOCK_GROUP rq_locks_;  // 每级队列的读写锁
  static GRP_WQ_CV cv_wq_;         // 每组的条件变量
  static NOTIFY_GRP notify_grp_;   // 每组的通知计数
};
```

`NextRoutine()` 从高优先级到低优先级遍历队列，对每个协程尝试 `Acquire()` + `UpdateState()`，返回第一个 READY 的协程。这是一种非抢占式优先级调度。

### SchedulerClassic

```cpp
// cyber/scheduler/policy/scheduler_classic.h
class SchedulerClassic : public Scheduler {
  bool DispatchTask(const std::shared_ptr<CRoutine>&) override;
  bool NotifyProcessor(uint64_t crid) override;
  void CreateProcessor();
};
```

- **CreateProcessor**：遍历配置中的每个 `SchedGroup`，为每组创建 `proc_num` 个 Processor，每个绑定同名 `ClassicContext`，并设置 CPU 亲和性和调度策略。
- **DispatchTask**：根据配置为协程设置优先级和组名（未配置的协程分配到第一个组），然后插入对应组的优先级队列。
- **NotifyProcessor**：设置协程的 update flag，然后通过 `ClassicContext::Notify` 唤醒对应组的一个等待 Processor。

## Choreography 调度策略

### 架构

Choreography 策略将 Processor 分为两个池：

1. **Choreography Processor 池**：每个 Processor 拥有独立的 `ChoreographyContext`，协程可绑定到指定 Processor，实现确定性调度。
2. **Pool Processor 池**：使用 `ClassicContext`（默认组），处理未绑定的协程，行为与 Classic 策略相同。

```
Choreography Processors          Pool Processors (Classic)
┌──────────────┐                  ┌─────────────────────┐
│ Proc 0       │                  │ default_grp         │
│  └ cr_queue_ │ (按优先级排序)    │  Prio 0..19 队列    │
│ Proc 1       │                  │  Processor 0..M-1   │
│  └ cr_queue_ │                  └─────────────────────┘
│ ...          │
│ Proc N-1     │
└──────────────┘
```

### ChoreographyContext

```cpp
// cyber/scheduler/policy/choreography_context.h
class ChoreographyContext : public ProcessorContext {
 public:
  bool RemoveCRoutine(uint64_t crid);
  std::shared_ptr<CRoutine> NextRoutine() override;
  bool Enqueue(const std::shared_ptr<CRoutine>&);
  void Notify();
  void Wait() override;

 private:
  std::multimap<uint32_t, std::shared_ptr<CRoutine>, std::greater<uint32_t>>
      cr_queue_;  // 按优先级降序排列
};
```

每个 ChoreographyContext 是独立实例（非静态共享），拥有自己的 `cr_queue_`。`NextRoutine()` 遍历队列，返回第一个 READY 的协程。由于每个 Processor 独占一个 Context，不存在跨线程竞争队列的问题。

### SchedulerChoreography

```cpp
// cyber/scheduler/policy/scheduler_choreography.h
class SchedulerChoreography : public Scheduler {
  bool DispatchTask(const std::shared_ptr<CRoutine>&) override;
  bool NotifyProcessor(uint64_t crid) override;
  void CreateProcessor();
};
```

- **CreateProcessor**：先创建 `proc_num_` 个 Choreography Processor（各自绑定独立 `ChoreographyContext`），再创建 `task_pool_size_` 个 Pool Processor（绑定共享的 `ClassicContext`）。
- **DispatchTask**：
  1. 查找配置中的任务，设置优先级和 `processor_id`
  2. 若 `processor_id < proc_num_`，将协程入队到对应 Choreography Processor 的 `ChoreographyContext`
  3. 否则，将协程放入 Classic 线程池的 `default_grp` 队列
- **NotifyProcessor**：根据协程的 `processor_id` 决定通知 Choreography Context 还是 Classic Context。

## 数据流

```
用户代码 / Component
       │
       ▼
Scheduler::CreateTask()
       │
       ├─ 创建 CRoutine
       ├─ DispatchTask() ──► 放入对应队列
       └─ DataVisitor 注册回调 ──► NotifyProcessor()
                                        │
                                        ▼
                                  设置 UpdateFlag
                                  Notify 条件变量
                                        │
                                        ▼
                              Processor::Run() 循环
                                        │
                              NextRoutine() 取就绪协程
                                        │
                                        ▼
                              CRoutine::Resume()
                              (栈切换，执行用户回调)
                                        │
                              CRoutine::Yield()
                              (切回 Processor 主栈)
```

## 配置方式

### 配置文件加载

调度器在 `SchedulerFactory::Instance()` 中根据进程组名加载配置：

```cpp
// 配置文件路径: conf/{ProcessGroup}.conf
// 例如: conf/compute_sched_classic.conf
```

配置文件使用 protobuf text format，顶层结构为 `CyberConfig`，调度相关字段在 `scheduler_conf` 中。

### Proto 定义

```protobuf
// cyber/proto/scheduler_conf.proto
message SchedulerConf {
  optional string policy = 1;              // "classic" 或 "choreography"
  optional uint32 routine_num = 2;         // 协程上下文池大小
  optional uint32 default_proc_num = 3;    // 默认 Processor 数量
  optional string process_level_cpuset = 4; // 进程级 CPU 集合
  repeated InnerThread threads = 5;        // 内部线程配置（如 shm, async_log）
  optional ClassicConf classic_conf = 6;
  optional ChoreographyConf choreography_conf = 7;
}

message InnerThread {
  optional string name = 1;
  optional string cpuset = 2;
  optional string policy = 3;    // SCHED_OTHER / SCHED_RR / SCHED_FIFO
  optional uint32 prio = 4;
}
```

### Classic 配置示例

```protobuf
// conf/example_sched_classic.conf
scheduler_conf {
    policy: "classic"
    process_level_cpuset: "0-7,16-23"
    threads: [
        { name: "async_log"  cpuset: "1"  policy: "SCHED_OTHER"  prio: 0 },
        { name: "shm"        cpuset: "2"  policy: "SCHED_FIFO"   prio: 10 }
    ]
    classic_conf {
        groups: [
            {
                name: "group1"
                processor_num: 16
                affinity: "range"       // "range" 或 "1to1"
                cpuset: "0-7,16-23"
                processor_policy: "SCHED_OTHER"
                processor_prio: 0
                tasks: [
                    { name: "E"  prio: 0 }
                ]
            },
            {
                name: "group2"
                processor_num: 16
                affinity: "1to1"
                cpuset: "8-15,24-31"
                processor_policy: "SCHED_OTHER"
                processor_prio: 0
                tasks: [
                    { name: "A"  prio: 0 },
                    { name: "B"  prio: 1 },
                    { name: "C"  prio: 2 },
                    { name: "D"  prio: 3 }
                ]
            }
        ]
    }
}
```

配置要点：
- `affinity: "range"` — 所有 Processor 线程可在 cpuset 范围内的任意 CPU 上运行
- `affinity: "1to1"` — 第 i 个 Processor 绑定到 cpuset 中的第 i 个 CPU
- `tasks` 中未列出的协程会被分配到第一个组，优先级为默认值

### Choreography 配置示例

```protobuf
// conf/example_sched_choreography.conf
scheduler_conf {
    policy: "choreography"
    process_level_cpuset: "0-7,16-23"
    choreography_conf {
        choreography_processor_num: 8
        choreography_affinity: "range"
        choreography_cpuset: "0-7"
        choreography_processor_policy: "SCHED_FIFO"
        choreography_processor_prio: 10

        pool_processor_num: 8
        pool_affinity: "range"
        pool_cpuset: "16-23"
        pool_processor_policy: "SCHED_OTHER"
        pool_processor_prio: 0

        tasks: [
            { name: "A"  processor: 0  prio: 1 },
            { name: "B"  processor: 0  prio: 2 },
            { name: "C"  processor: 1  prio: 1 },
            { name: "D"  processor: 1  prio: 2 },
            { name: "E" }  // 未指定 processor，进入 Pool
        ]
    }
}
```

配置要点：
- `processor` 字段指定协程绑定的 Choreography Processor 编号（0 ~ N-1）
- 未指定 `processor` 的任务（如 "E"）会进入 Pool Processor 池
- Choreography Processor 通常使用 `SCHED_FIFO` 实时调度策略以获得更低延迟

### CPU 亲和性与调度策略

`pin_thread.cc` 提供底层工具函数：

- `ParseCpuset("0-7,16-23")` — 解析 cpuset 字符串为 CPU 编号列表
- `SetSchedAffinity` — 设置线程的 CPU 亲和性（`range` 模式设置整个集合，`1to1` 模式绑定单个 CPU）
- `SetSchedPolicy` — 设置线程调度策略（`SCHED_FIFO`/`SCHED_RR` 使用 `pthread_setschedparam`，`SCHED_OTHER` 使用 `setpriority`）

## 与其他模块的关系

| 模块 | 关系 |
|------|------|
| `cyber/croutine/` | 提供协程原语（CRoutine、RoutineFactory），是调度的执行单元 |
| `cyber/data/` | DataVisitor 在数据到达时通过回调触发 `NotifyProcessor`，驱动协程唤醒 |
| `cyber/component/` | Component 的 `Process` 方法被封装为 CRoutine，由 Scheduler 调度 |
| `cyber/timer/` | TimingWheel 的 tick 线程通过 `SetInnerThreadAttr` 由 Scheduler 配置 CPU 亲和性 |
| `cyber/proto/` | `scheduler_conf.proto`、`classic_conf.proto`、`choreography_conf.proto` 定义配置结构 |
| `cyber/conf/` | 存放各进程的调度配置文件（protobuf text format） |