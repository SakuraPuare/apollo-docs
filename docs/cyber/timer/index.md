---
title: Timer 定时器
---

# Timer 定时器

## 模块职责概述

Timer 模块为 Apollo CyberRT 提供高效的定时任务调度能力。它基于**分层时间轮**（Hierarchical Timing Wheel）实现，支持单次（oneshot）和周期（periodic）两种定时模式。周期定时器内置累积误差补偿机制，确保长时间运行下的时间精度。

Timer 模块是 `TimerComponent` 的底层驱动——用户通过继承 `TimerComponent` 并配置 `interval`，即可获得周期性调用的 `Proc()` 方法。

## 核心类与接口

### TimerTask — 定时任务

```cpp
// cyber/timer/timer_task.h
struct TimerTask {
  explicit TimerTask(uint64_t timer_id);

  uint64_t timer_id_ = 0;
  std::function<void()> callback;          // 到期回调
  uint64_t interval_ms = 0;               // 周期（毫秒）
  uint64_t remainder_interval_ms = 0;     // 溢出到辅助轮后的余数槽位
  uint64_t next_fire_duration_ms = 0;     // 下次触发的延迟
  int64_t accumulated_error_ns = 0;       // 累积误差（纳秒）
  uint64_t last_execute_time_ns = 0;      // 上次执行时间戳
  std::mutex mutex;                        // 保护回调执行与 Timer::Stop 的并发
};
```

`TimerTask` 是时间轮中流转的基本单元。对于周期定时器，每次回调执行完毕后会重新计算 `next_fire_duration_ms` 并将自身重新插入时间轮。

### TimerBucket — 时间槽

```cpp
// cyber/timer/timer_bucket.h
class TimerBucket {
 public:
  void AddTask(const std::shared_ptr<TimerTask>& task);
  std::mutex& mutex();
  std::list<std::weak_ptr<TimerTask>>& task_list();

 private:
  std::mutex mutex_;
  std::list<std::weak_ptr<TimerTask>> task_list_;
};
```

每个 bucket 是时间轮上的一个槽位，内部维护一个 `weak_ptr` 链表。使用 `weak_ptr` 的设计使得当 `Timer::Stop()` 释放 `shared_ptr<TimerTask>` 后，时间轮在 Tick 时自动跳过已失效的任务，无需显式删除。

### TimingWheel — 分层时间轮

```cpp
// cyber/timer/timing_wheel.h
static const uint64_t WORK_WHEEL_SIZE = 512;
static const uint64_t ASSISTANT_WHEEL_SIZE = 64;
static const uint64_t TIMER_RESOLUTION_MS = 2;
static const uint64_t TIMER_MAX_INTERVAL_MS =
    WORK_WHEEL_SIZE * ASSISTANT_WHEEL_SIZE * TIMER_RESOLUTION_MS;  // 65536ms

class TimingWheel {
 public:
  void Start();
  void Shutdown();
  void Tick();
  void AddTask(const std::shared_ptr<TimerTask>& task);
  void Cascade(const uint64_t assistant_wheel_index);
  void TickFunc();

 private:
  TimerBucket work_wheel_[WORK_WHEEL_SIZE];       // 工作轮：512 槽
  TimerBucket assistant_wheel_[ASSISTANT_WHEEL_SIZE]; // 辅助轮：64 槽
  uint64_t current_work_wheel_index_ = 0;
  uint64_t current_assistant_wheel_index_ = 0;
  std::thread tick_thread_;
};
```

`TimingWheel` 是全局单例（`DECLARE_SINGLETON`），首次 `AddTask` 时自动启动。

### Timer — 用户接口

```cpp
// cyber/timer/timer.h
struct TimerOption {
  uint32_t period = 0;                // 周期，单位毫秒（范围 1 ~ 65535）
  std::function<void()> callback;     // 回调函数
  bool oneshot;                       // true: 单次触发  false: 周期触发
};

class Timer {
 public:
  Timer();
  explicit Timer(TimerOption opt);
  Timer(uint32_t period, std::function<void()> callback, bool oneshot);

  void SetTimerOption(TimerOption opt);
  void Start();
  void Stop();

 private:
  bool InitTimerTask();
  uint64_t timer_id_;
  TimerOption timer_opt_;
  TimingWheel* timing_wheel_;
  std::shared_ptr<TimerTask> task_;
  std::atomic<bool> started_ = {false};
};
```

使用示例：

```cpp
// 创建一个 100ms 周期定时器
cyber::Timer timer(100, []() {
    AINFO << "timer callback fired";
}, false);
timer.Start();

// 停止定时器
timer.Stop();
```

### TimerComponent — 定时组件

```cpp
// cyber/component/timer_component.h
class TimerComponent : public ComponentBase {
 public:
  bool Initialize(const TimerComponentConfig& config) override;
  void Clear() override;
  bool Process();
  uint32_t GetInterval() const;

 private:
  virtual bool Proc() = 0;  // 用户实现
  uint32_t interval_ = 0;
  std::unique_ptr<Timer> timer_;
};
```

`TimerComponent` 在 `Initialize` 中创建一个周期 Timer，回调函数调用 `Process()` -> `Proc()`。用户只需继承并实现 `Init()` 和 `Proc()`：

```cpp
// 示例：cyber/examples/timer_component_example/
class TimerComponentSample : public TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
};
CYBER_REGISTER_COMPONENT(TimerComponentSample)
```

对应的 DAG 配置：

```protobuf
// cyber/proto/component_conf.proto
message TimerComponentConfig {
  optional string name = 1;
  optional string config_file_path = 2;
  optional string flag_file_path = 3;
  optional uint32 interval = 4;  // 周期，单位毫秒
}
```

## 数据流

### 分层时间轮工作原理

时间轮采用两层结构：

```
辅助轮 (Assistant Wheel)          工作轮 (Work Wheel)
64 槽，每槽 = 512 × 2ms = 1024ms   512 槽，每槽 = 2ms
┌───┬───┬───┬─── ··· ───┐         ┌───┬───┬───┬─── ··· ───┐
│ 0 │ 1 │ 2 │           │         │ 0 │ 1 │ 2 │           │
└───┴───┴───┴─── ··· ───┘         └───┴───┴───┴─── ··· ───┘
  ▲                                  ▲
  current_assistant_wheel_index_     current_work_wheel_index_
```

- 时间分辨率：`TIMER_RESOLUTION_MS = 2ms`
- 工作轮覆盖范围：512 × 2ms = 1024ms
- 辅助轮覆盖范围：64 × 1024ms = 65536ms（约 65.5 秒）
- 最大定时间隔：`TIMER_MAX_INTERVAL_MS = 65536ms`

### AddTask 流程

```
AddTask(task)
    │
    ├─ 计算 work_wheel_index = current_index + ceil(duration / 2ms)
    │
    ├─ 若 index < 512（在工作轮范围内）
    │   └─ 直接插入 work_wheel_[index]
    │
    └─ 若 index >= 512（超出工作轮范围）
        ├─ 计算 assistant_ticks = index / 512
        ├─ 计算 remainder = index % 512
        ├─ task->remainder_interval_ms = remainder
        │
        ├─ 若 assistant_ticks == 1 且 remainder < current_index
        │   └─ 直接插入 work_wheel_[remainder]（下一轮即到期）
        │
        └─ 否则
            └─ 插入 assistant_wheel_[current + assistant_ticks]
```

### Tick 流程

```
TickFunc() — 每 2ms 执行一次
    │
    ├─ Tick()
    │   └─ 遍历 work_wheel_[current_index] 中的所有任务
    │       └─ 对每个有效任务，通过 cyber::Async 异步执行回调
    │
    ├─ tick_count_++
    ├─ current_work_wheel_index_ = (current + 1) % 512
    │
    └─ 若 current_work_wheel_index_ == 0（工作轮转完一圈）
        ├─ current_assistant_wheel_index_++
        └─ Cascade(current_assistant_wheel_index_)
            └─ 将辅助轮当前槽的所有任务
               按 remainder_interval_ms 重新分配到工作轮
```

### 周期定时器的误差补偿

对于周期定时器，回调执行完毕后会重新计算下次触发时间：

```
callback 执行完毕
    │
    ├─ 记录执行耗时 execute_time_ms
    ├─ 计算累积误差 accumulated_error_ns += (实际间隔 - 期望间隔)
    │
    ├─ 若 execute_time_ms >= interval_ms
    │   └─ next_fire = TIMER_RESOLUTION_MS（尽快触发）
    │
    └─ 否则
        └─ next_fire = interval_ms - execute_time_ms - accumulated_error_ms
           （补偿累积误差）
    │
    └─ 重新 AddTask 到时间轮
```

这种机制确保即使单次回调执行时间波动，长期平均频率仍然接近配置值。

## 配置方式

### Timer 直接使用

Timer 本身不需要配置文件，通过代码直接创建：

```cpp
cyber::Timer timer;
timer.SetTimerOption({100, callback, false});  // 100ms 周期
timer.Start();
```

### TimerComponent 配置

通过 DAG 文件中的 `TimerComponentConfig` 配置：

```protobuf
timer_components {
  class_name: "TimerComponentSample"
  config {
    name: "timer_sample"
    config_file_path: "/path/to/config"
    interval: 100  # 毫秒
  }
}
```

### 时间轮线程配置

TimingWheel 的 tick 线程可通过 Scheduler 的 `InnerThread` 配置设置 CPU 亲和性：

```protobuf
scheduler_conf {
    threads: [
        {
            name: "timer"
            cpuset: "0"
            policy: "SCHED_FIFO"
            prio: 10
        }
    ]
}
```

时间轮启动时调用 `scheduler::Instance()->SetInnerThreadAttr("timer", &tick_thread_)` 应用此配置。

## 与其他模块的关系

| 模块 | 关系 |
|------|------|
| `cyber/scheduler/` | TimingWheel 通过 `SetInnerThreadAttr` 获取 tick 线程的 CPU/调度配置；回调通过 `cyber::Async` 提交到调度器执行 |
| `cyber/component/` | `TimerComponent` 内部持有 `Timer` 实例，将 `Proc()` 封装为周期回调 |
| `cyber/time/` | 使用 `Rate` 控制 tick 线程的睡眠精度；使用 `Time::MonoTime` 计算执行耗时和误差 |
| `cyber/task/` | `cyber::Async` 将定时回调提交到 CyberRT 的异步任务池执行，而非在 tick 线程中同步执行 |
| `cyber/proto/` | `component_conf.proto` 定义 `TimerComponentConfig`（含 `interval` 字段） |
