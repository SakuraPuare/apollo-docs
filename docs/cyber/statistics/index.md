---
title: "Statistics"
---

# Statistics

> 源码路径: `cyber/statistics/`

## 概述

`statistics` 模块为 Cyber RT 的消息通道提供运行时性能统计能力。它基于百度 brpc 的 `bvar` 库，围绕每个 Reader/Writer 的 `RoleAttributes` 自动注册三类延迟记录器（proc / tran / cyber）、状态变量和累加器，并支持自定义 Span 时间段测量。整个模块通过 `DECLARE_SINGLETON` 宏以单例模式运行，上层通道在创建时调用 `RegisterChanVar` 完成指标注册，消息处理过程中通过模板采样函数写入实时数据。

## 核心类

### SpanHandler

用于记录一段自定义时间区间的耗时信息。

```cpp
struct SpanHandler {
  std::string name;
  uint64_t start_time;
  uint64_t end_time;
  std::shared_ptr<::bvar::LatencyRecorder> span_trace_node;
  uint64_t min_ns = 0;
};
```

- `name` — Span 标识名，需全局唯一
- `start_time` / `end_time` — 微秒级时间戳
- `span_trace_node` — 底层 `bvar::LatencyRecorder`，用于对外暴露延迟分布
- `min_ns` — 最小有效间隔（纳秒），低于此值的测量会被丢弃并打印警告

### Statistics

模块核心单例类，负责所有通道级统计变量的生命周期管理。

```cpp
class Statistics {
 public:
  bool RegisterChanVar(const proto::RoleAttributes& role_attr);
  bool CreateSpan(std::string name, uint64_t min_ns = 0);
  bool StartSpan(std::string name);
  bool EndSpan(std::string name);
  void DisableChanVar();

  template <typename SampleT>
  std::shared_ptr<::bvar::Adder<SampleT>> CreateAdder(
      const proto::RoleAttributes& role_attr);

  template <typename SampleT>
  bool SamplingProcLatency(const proto::RoleAttributes& role_attr, SampleT sample);

  template <typename SampleT>
  bool SamplingTranLatency(const proto::RoleAttributes& role_attr, SampleT sample);

  template <typename SampleT>
  bool SamplingCyberLatency(const proto::RoleAttributes& role_attr, SampleT sample);

  bool SetProcStatus(const proto::RoleAttributes& role_attr, uint64_t val);
  bool GetProcStatus(const proto::RoleAttributes& role_attr, uint64_t* val);
  bool SetTotalMsgsStatus(const proto::RoleAttributes& role_attr, int32_t val);
  bool AddRecvCount(const proto::RoleAttributes& role_attr, int total_msg_val);

 private:
  // 三类延迟变量 + 状态变量 + 累加器的 map
  std::unordered_map<std::string, LatencyVarPtr> latency_map_;
  std::unordered_map<std::string, StatusVarPtr> status_map_;
  std::unordered_map<std::string, AdderVarPtr> adder_map_;
  std::unordered_map<std::string, std::shared_ptr<SpanHandler>> span_handlers_;
  bool first_recv_ = true;
  bool disable_chan_var_ = false;

  DECLARE_SINGLETON(Statistics)
};
```

类型别名：

```cpp
using LatencyVarPtr = std::shared_ptr<::bvar::LatencyRecorder>;
using StatusVarPtr  = std::shared_ptr<::bvar::Status<uint64_t>>;
using AdderVarPtr   = std::shared_ptr<::bvar::Adder<int32_t>>;
```

## 核心函数

### RegisterChanVar

注册一条通道的全部统计变量。

```cpp
bool Statistics::RegisterChanVar(const proto::RoleAttributes& role_attr);
```

- **职责** — 根据 `role_attr` 中的 `node_name` + `channel_name` 创建对应的 bvar 变量并存入 map
- **输入** — `role_attr`：通道角色属性，包含节点名和通道名
- **输出** — `true` 注册成功；`false` 表示同名变量已存在
- **关键步骤**：
  1. 检查 proc / tran / cyber 三个 latency key 是否已存在，存在则报错返回
  2. 创建 proc 类型的 `LatencyRecorder`
  3. 若通道不是 `_timer_component`，额外创建 tran / cyber 的 `LatencyRecorder`、process-status 的 `Status`、total-msgs 的 `Status` 和 recv-msgs 的 `Adder`

### SamplingProcLatency / SamplingTranLatency / SamplingCyberLatency

向对应的延迟记录器写入一次采样值。

```cpp
template <typename SampleT>
bool SamplingProcLatency(const proto::RoleAttributes& role_attr, SampleT sample);
```

- **职责** — 将一次处理 / 传输 / 端到端延迟写入对应的 `LatencyRecorder`
- **输入** — `role_attr` 定位通道，`sample` 为延迟值（通常为微秒）
- **输出** — `true` 写入成功；`false` 表示变量未注册
- **关键步骤**：若 `disable_chan_var_` 为 true 则直接返回；`_timer_component` 通道在 tran / cyber 采样时被跳过

### SetProcStatus / GetProcStatus

读写通道的进程状态值。

```cpp
bool SetProcStatus(const proto::RoleAttributes& role_attr, uint64_t val);
bool GetProcStatus(const proto::RoleAttributes& role_attr, uint64_t* val);
```

- **职责** — 通过 `bvar::Status<uint64_t>` 暴露通道的运行状态
- **输入 / 输出** — `val` 为状态值指针（Get 时为输出参数）

### AddRecvCount

累加消息接收计数。

```cpp
bool AddRecvCount(const proto::RoleAttributes& role_attr, int total_msg_val);
```

- **职责** — 通过 `bvar::Adder<int32_t>` 统计接收消息总数
- **关键步骤** — 首次调用时写入 `total_msg_val`（补偿历史消息），后续每次写入 `1`

### CreateSpan / StartSpan / EndSpan

自定义时间段耗时测量三件套。

```cpp
bool CreateSpan(std::string name, uint64_t min_ns = 0);
bool StartSpan(std::string name);
bool EndSpan(std::string name);
```

- **CreateSpan** — 创建一个 `SpanHandler`，名称需全局唯一，可设置最小有效间隔 `min_ns`
- **StartSpan** — 记录当前微秒时间戳到 `start_time`
- **EndSpan** — 记录 `end_time`，计算差值 `diff`；若 `diff < min_ns` 则丢弃并警告；若 `diff > INT32_MAX` 则丢弃；否则写入 `LatencyRecorder`

### CreateAdder

为指定通道创建一个自定义累加器。

```cpp
template <typename SampleT>
std::shared_ptr<::bvar::Adder<SampleT>> CreateAdder(
    const proto::RoleAttributes& role_attr);
```

- **职责** — 创建以 `node_name-channel_name` 为暴露名的 `bvar::Adder`
- **输出** — 返回累加器的 shared_ptr，由调用方持有

## 配置

| 配置项 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `disable_chan_var_` | `bool` | `false` | 为 `true` 时所有通道级采样 / 状态读写均跳过，用于运行时关闭统计 |
| `TIMER_COMPONENT_CHAN_NAME` | `const std::string` | `"_timer_component"` | 定时器组件专用通道名，该通道仅注册 proc 延迟，不注册 tran / cyber / status / adder |
| `min_ns` (Span) | `uint64_t` | `0` | Span 最小有效耗时阈值（纳秒），低于此值的测量被丢弃 |

## 调用关系

```text
Reader / Writer 创建
  └── RegisterChanVar(role_attr)
        ├── latency_map_[proc]  ← bvar::LatencyRecorder
        ├── latency_map_[tran]  ← bvar::LatencyRecorder  (非 timer_component)
        ├── latency_map_[cyber] ← bvar::LatencyRecorder  (非 timer_component)
        ├── status_map_[process-status]   ← bvar::Status  (非 timer_component)
        ├── status_map_[total-msgs]       ← bvar::Status  (非 timer_component)
        └── adder_map_[recv-msgs]         ← bvar::Adder   (非 timer_component)

消息处理流程
  ├── SamplingProcLatency(sample)   ← 处理完成后调用
  ├── SamplingTranLatency(sample)   ← 传输延迟采样
  ├── SamplingCyberLatency(sample)  ← 端到端延迟采样
  ├── SetProcStatus(val)            ← 更新通道状态
  ├── SetTotalMsgsStatus(val)       ← 更新总消息数
  └── AddRecvCount(val)             ← 累加接收计数

自定义 Span 测量
  ├── CreateSpan(name, min_ns)
  ├── StartSpan(name)
  └── EndSpan(name)  → LatencyRecorder << diff
```
