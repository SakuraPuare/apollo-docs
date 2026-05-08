---
title: "监控日志"
---

# Monitor Log

> 源码路径: `modules/common/monitor_log/`

## 概述

monitor_log 模块为 Apollo 各模块提供统一的运行时监控日志采集与发布机制。模块通过 Cyber RT 的消息通道将监控信息发布到 `/apollo/monitor` 主题，供诊断系统和可视化工具消费。

模块采用 **Buffer + Logger** 两层设计：

- `MonitorLogBuffer` 负责在调用侧收集日志条目，析构时自动发布
- `MonitorLogger`（单例）负责创建 Cyber RT 节点并执行实际的消息写入

## 核心类

### MonitorLogger

单例类，封装 Cyber RT 节点和 `MonitorMessage` 写入器。

```cpp
class MonitorLogger {
 public:
  virtual ~MonitorLogger() = default;
  virtual void Publish(const MonitorMessageItem::MessageSource &source,
                       const std::vector<MessageItem> &messages) const;

 private:
  virtual void DoPublish(MonitorMessage *message) const;
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Writer<MonitorMessage>> monitor_msg_writer_;
  DECLARE_SINGLETON(MonitorLogger)
};
```

构造时创建以 `monitor_logger` + 当前时间戳命名的 Cyber RT 节点，并建立 `/apollo/monitor` 通道的 Writer。

### MonitorLogBuffer

面向使用者的日志缓冲类，将多条 `MessageItem` 收集后批量发布。

```cpp
class MonitorLogBuffer {
 public:
  explicit MonitorLogBuffer(const MonitorMessageItem::MessageSource &source);
  virtual ~MonitorLogBuffer();

  MonitorLogBuffer &INFO(const std::string &msg);
  MonitorLogBuffer &WARN(const std::string &msg);
  MonitorLogBuffer &ERROR(const std::string &msg);
  MonitorLogBuffer &FATAL(const std::string &msg);

  void AddMonitorMsgItem(const MonitorMessageItem::LogLevel log_level,
                         const std::string &msg);
  void Publish();

 private:
  MonitorLogger *logger_ = MonitorLogger::Instance();
  std::vector<MessageItem> monitor_msg_items_;
  MonitorMessageItem::MessageSource source_;
};
```

`REG_MSG_TYPE` 宏为每个日志级别生成两个重载：带消息参数时立即添加并发布；无参数时仅设置当前级别，支持链式调用。

### MessageItem

```cpp
using MessageItem = std::pair<MonitorMessageItem::LogLevel, std::string>;
```

将日志级别与消息文本打包为一对，由 `MonitorLogger::Publish` 遍历组装成 `MonitorMessage` protobuf。

## 核心函数

### MonitorLogger::Publish

```cpp
void MonitorLogger::Publish(const MonitorMessageItem::MessageSource &source,
                            const std::vector<MessageItem> &messages) const;
```

遍历 `messages`，为每条构造 `MonitorMessageItem` 并填充到 `MonitorMessage`，然后调用 `DoPublish` 发布。空消息列表直接返回。

### MonitorLogger::DoPublish

```cpp
void MonitorLogger::DoPublish(MonitorMessage *message) const;
```

填充消息头（`FillHeader`），通过 `monitor_msg_writer_->Write` 写入通道。若 writer 为空则直接返回。

### MonitorLogBuffer::AddMonitorMsgItem

```cpp
void MonitorLogBuffer::AddMonitorMsgItem(
    const MonitorMessageItem::LogLevel log_level, const std::string &msg);
```

设置当前日志级别并将 (level, msg) 对追加到内部缓冲区。

### MonitorLogBuffer::Publish

```cpp
void MonitorLogBuffer::Publish();
```

若缓冲区非空，委托 `MonitorLogger::Publish` 发布所有已收集的条目，随后清空缓冲区并将级别重置为 INFO。析构函数中自动调用此方法，确保未显式发布的日志不会丢失。

## 配置

- **发布通道**: `/apollo/monitor`（硬编码于 `MonitorLogger` 构造函数）
- **日志级别**: INFO / WARN / ERROR / FATAL，定义于 `MonitorMessageItem::LogLevel` 枚举（`monitor_log.pb.h`）
- **消息来源**: `MonitorMessageItem::MessageSource` 枚举标识日志所属模块

## 调用关系

```text
模块代码
  │
  ├─ 构造 MonitorLogBuffer(source)
  │     │
  │     ├─ .INFO(msg) / .WARN(msg) / .ERROR(msg) / .FATAL(msg)
  │     │     └─ AddMonitorMsgItem() → 追加到 buffer
  │     │
  │     ├─ 显式调用 Publish()
  │     └─ 析构时自动调用 Publish()
  │           │
  │           └─ MonitorLogger::Publish(source, items)
  │                 ├─ 组装 MonitorMessage protobuf
  │                 └─ DoPublish()
  │                       └─ FillHeader + Writer::Write
  │                             ↓
  │                     /apollo/monitor 通道
  │
  └─ 也可通过 MONITOR(level, msg) 宏直接使用 MonitorLogger
```
