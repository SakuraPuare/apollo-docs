---
title: "Event 性能事件"
---

# Event 性能事件

> 源码路径：`cyber/event/`

## 概述

Event 模块是 CyberRT 的性能事件记录子系统，用于采集调度（Sched）和传输（Transport）两个维度的时间戳事件，并异步写入文件。通过全局配置 `perf_conf` 控制开关和采集类型。各模块在关键路径上调用 `PerfEventCache` 的接口记录事件，离线分析工具读取 `.data` 文件进行性能分析。

## 核心类

### EventBase

事件基类，定义所有事件的公共字段（事件类型、事件 ID、时间戳）和序列化接口。

```cpp
class EventBase {
 public:
  virtual std::string SerializeToString() = 0;
  void set_eid(int eid);
  void set_etype(int etype);
  void set_stamp(uint64_t stamp);

  virtual void set_cr_id(uint64_t cr_id);
  virtual void set_cr_state(int cr_state);
  virtual void set_proc_id(int proc_id);
  virtual void set_msg_seq(uint64_t msg_seq);
  virtual void set_channel_id(uint64_t channel_id);
  virtual void set_adder(const std::string& adder);

 protected:
  int etype_;      // 事件类型（SCHED_EVENT / TRANS_EVENT）
  int eid_;        // 事件 ID（对应 SchedPerf / TransPerf 枚举值）
  uint64_t stamp_; // 时间戳（纳秒）
};
```

**源码**：`cyber/event/perf_event.h`

### SchedEvent

调度事件，记录协程的 swap_in、swap_out、notify_in、next_routine 等操作。

```cpp
class SchedEvent : public EventBase {
 public:
  SchedEvent();
  std::string SerializeToString() override;
  void set_cr_id(uint64_t cr_id) override;
  void set_cr_state(int cr_state) override;
  void set_proc_id(int proc_id) override;

 private:
  int cr_state_ = 1;
  int proc_id_ = 0;
  uint64_t cr_id_ = 0;
};
```

**序列化格式**：`etype \t eid \t task_name \t proc_id \t cr_state \t stamp`

### TransportEvent

传输事件，记录消息从发送到接收的全链路时间戳。

```cpp
class TransportEvent : public EventBase {
 public:
  TransportEvent();
  std::string SerializeToString() override;
  void set_msg_seq(uint64_t msg_seq) override;
  void set_channel_id(uint64_t channel_id) override;
  void set_adder(const std::string& adder) override;
  static std::string ShowTransPerf(TransPerf type);

 private:
  std::string adder_;
  uint64_t msg_seq_ = 0;
  uint64_t channel_id_;
};
```

**序列化格式**：`etype \t eid \t channel_name \t msg_seq \t stamp \t adder`

### PerfEventCache

事件缓存（单例），内部维护一个有界队列，由独立 IO 线程消费事件并写入文件。

```cpp
class PerfEventCache {
 public:
  void AddSchedEvent(SchedPerf event_id, uint64_t cr_id,
                     int proc_id, int cr_state = -1);
  void AddTransportEvent(TransPerf event_id, uint64_t channel_id,
                         uint64_t msg_seq, uint64_t stamp = 0,
                         const std::string& adder = "-");
  std::string PerfFile();
  void Shutdown();

 private:
  void Start();
  void Run();

  base::BoundedQueue<EventBasePtr> event_queue_;  // 容量 8192
  std::thread io_thread_;
  std::ofstream of_;
  proto::PerfConf perf_conf_;
  bool enable_ = false;
  DECLARE_SINGLETON(PerfEventCache)
};
```

**源码**：`cyber/event/perf_event_cache.h`、`cyber/event/perf_event_cache.cc`

## 核心函数

### PerfEventCache::AddSchedEvent()

```cpp
void PerfEventCache::AddSchedEvent(const SchedPerf event_id,
                                   const uint64_t cr_id,
                                   const int proc_id,
                                   const int cr_state) {
  if (!enable_) return;
  if (perf_conf_.type() != PerfType::SCHED &&
      perf_conf_.type() != PerfType::ALL) return;
  EventBasePtr e = std::make_shared<SchedEvent>();
  e->set_eid(static_cast<int>(event_id));
  e->set_stamp(Time::Now().ToNanosecond());
  e->set_cr_state(cr_state);
  e->set_cr_id(cr_id);
  e->set_proc_id(proc_id);
  event_queue_.Enqueue(e);
}
```

**职责**：记录一条调度事件到队列。仅在 `perf_conf` 类型为 `SCHED` 或 `ALL` 时生效。

### PerfEventCache::AddTransportEvent()

```cpp
void PerfEventCache::AddTransportEvent(const TransPerf event_id,
                                       const uint64_t channel_id,
                                       const uint64_t msg_seq,
                                       const uint64_t stamp,
                                       const std::string& adder) {
  if (!enable_) return;
  if (perf_conf_.type() != PerfType::TRANSPORT &&
      perf_conf_.type() != PerfType::ALL) return;
  EventBasePtr e = std::make_shared<TransportEvent>();
  e->set_eid(static_cast<int>(event_id));
  e->set_channel_id(channel_id);
  e->set_msg_seq(msg_seq);
  e->set_adder(adder);
  e->set_stamp(stamp == 0 ? Time::Now().ToNanosecond() : stamp);
  event_queue_.Enqueue(e);
}
```

**职责**：记录一条传输事件到队列。仅在 `perf_conf` 类型为 `TRANSPORT` 或 `ALL` 时生效。

### PerfEventCache::Run()

IO 线程主循环，从队列中取出事件并序列化写入文件，每 512 条 flush 一次。

```cpp
void PerfEventCache::Run() {
  EventBasePtr event;
  int buf_size = 0;
  while (!shutdown_ && !apollo::cyber::IsShutdown()) {
    if (event_queue_.WaitDequeue(&event)) {
      of_ << event->SerializeToString() << std::endl;
      buf_size++;
      if (buf_size >= kFlushSize) {
        of_.flush();
        buf_size = 0;
      }
    }
  }
}
```

### PerfEventCache::Start()

创建输出文件（格式 `cyber_perf_<时间戳>.data`），写入起始时间戳，启动 IO 线程。

## 事件枚举

### SchedPerf

| 枚举值 | 数值 | 说明 |
|--------|------|------|
| SWAP_IN | 1 | 协程换入 |
| SWAP_OUT | 2 | 协程换出 |
| NOTIFY_IN | 3 | 通知换入 |
| NEXT_RT | 4 | 下一个就绪协程 |
| RT_CREATE | 5 | 协程创建 |

### TransPerf

| 枚举值 | 数值 | 说明 |
|--------|------|------|
| TRANSMIT_BEGIN | 0 | 传输开始 |
| SERIALIZE | 1 | 序列化 |
| SEND | 2 | 发送 |
| MESSAGE_ARRIVE | 3 | 消息到达 |
| OBTAIN | 4 | 获取（仅共享内存） |
| DESERIALIZE | 5 | 反序列化 |
| DISPATCH | 6 | 分发 |
| NOTIFY | 7 | 通知 |
| FETCH | 8 | 拉取 |
| CALLBACK | 9 | 回调 |

## 配置

通过 `cyber.proto.PerfConf` 控制：

| 字段 | 类型 | 说明 |
|------|------|------|
| enable | bool | 是否启用性能采集 |
| type | PerfType | 采集类型：`SCHED`、`TRANSPORT`、`ALL` |

## 调用关系

- **上游**：Scheduler 调用 `AddSchedEvent()`；Transport 层调用 `AddTransportEvent()`
- **下游**：事件写入 `.data` 文件，供离线分析工具（`cyber/monitor/` 等）读取
- **依赖**：`cyber/base/bounded_queue.h`（有界队列）、`cyber/common/global_data.h`（全局配置）
