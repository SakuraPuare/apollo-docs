---
title: "延迟记录器"
---

# Latency Recorder

> 源码路径: `modules/common/latency_recorder/`

## 概述

LatencyRecorder 是 Apollo 的模块间延迟记录工具，用于测量消息从产生到被处理的时间差。各模块通过该类记录消息 ID 与起止时间，积累到一定周期后批量发布到 Cyber RT 通道，供监控和诊断系统消费。

## 核心类

### LatencyRecorder

记录模块延迟的主类，维护当前模块的名称、待发布记录集合及 Cyber Writer。线程安全，内部通过 `std::mutex` 保护共享状态。

```cpp
class LatencyRecorder {
 public:
  explicit LatencyRecorder(const std::string& module_name);
  void AppendLatencyRecord(const uint64_t message_id,
                           const apollo::cyber::Time& begin_time,
                           const apollo::cyber::Time& end_time);
};
```

### Protobuf 消息

延迟记录通过以下 protobuf 消息序列化：

```protobuf
message LatencyRecord {
  optional uint64 begin_time = 1;
  optional uint64 end_time = 2;
  optional uint64 message_id = 3;
};

message LatencyRecordMap {
  optional apollo.common.Header header = 1;
  optional string module_name = 2;
  repeated LatencyRecord latency_records = 3;
};

message LatencyStat {
  optional uint64 min_duration = 1;
  optional uint64 max_duration = 2;
  optional uint64 aver_duration = 3;
  optional uint32 sample_size = 4;
};

message LatencyTrack {
  message LatencyTrackMessage {
    optional string latency_name = 1;
    optional LatencyStat latency_stat = 2;
  }
  repeated LatencyTrackMessage latency_track = 1;
};

message LatencyReport {
  optional apollo.common.Header header = 1;
  optional LatencyTrack e2es_latency = 2;
  optional LatencyTrack modules_latency = 3;
};
```

## 核心函数

### LatencyRecorder::LatencyRecorder

构造函数，初始化模块名称并创建空的 `LatencyRecordMap`。

```cpp
LatencyRecorder::LatencyRecorder(const std::string& module_name)
    : module_name_(module_name) {
  records_.reset(new LatencyRecordMap);
}
```

### AppendLatencyRecord

记录一次延迟。校验 `begin_time < end_time` 后，将记录追加到 `records_` 中。当距上次发布时间超过 3 秒时，自动调用 `PublishLatencyRecords` 批量发布。

- 仿真模式下 `begin_time >= end_time` 时降低日志频率（每 1000 次报一次），避免日志洪泛。
- `message_id` 为 0 时直接返回，不记录。

```cpp
void LatencyRecorder::AppendLatencyRecord(const uint64_t message_id,
                                          const Time& begin_time,
                                          const Time& end_time);
```

### CreateWriter

懒初始化 Cyber RT Writer。首次调用时创建 Cyber 节点（名称格式：`latency_recorder<module_name><timestamp>`），然后在 `FLAGS_latency_recording_topic` 通道上创建 Writer。使用 `static` 局部变量保证只创建一次。

```cpp
std::shared_ptr<apollo::cyber::Writer<LatencyRecordMap>>
LatencyRecorder::CreateWriter();
```

### PublishLatencyRecords

将已累积的延迟记录发布到 Cyber 通道。发布前填充模块名称和消息头，发布后重置 `records_` 为空的 `LatencyRecordMap`。

```cpp
void LatencyRecorder::PublishLatencyRecords(
    const std::shared_ptr<apollo::cyber::Writer<LatencyRecordMap>>& writer);
```

## 配置

- `FLAGS_latency_recording_topic`：延迟记录发布的 Cyber RT 通道名称，定义于 `modules/common/adapters/adapter_gflags.h`。

## 调用关系

```text
AppendLatencyRecord
  |-- 校验 begin_time < end_time
  |-- CreateWriter() [static, 首次调用时创建]
  |     |-- cyber::CreateNode()
  |     |-- node_->CreateWriter<LatencyRecordMap>()
  |-- records_->add_latency_records()
  |-- PublishLatencyRecords() [每 3 秒触发]
        |-- FillHeader()
        |-- writer->Write()
        |-- records_.reset()
```
