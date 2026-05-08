---
title: "性能基准测试 (Benchmark)"
---

# Benchmark

> 源码路径：`cyber/benchmark/`

## 概述

`cyber/benchmark` 提供 CyberRT 通信层的性能基准测试工具，用于测量不同消息大小、发送频率、QoS 策略下的吞吐量和延迟表现。包含一个 writer（发送端）和一个 reader（接收端），支持 gperftools CPU/内存 profiling。

## 组件

| 文件 | 职责 |
| --- | --- |
| `cyber_benchmark_writer.cc` | 按指定频率和消息大小发送测试数据 |
| `cyber_benchmark_reader.cc` | 创建多个 reader 订阅并接收测试数据 |
| `benchmark_msg.proto` | 测试消息定义 |

## 消息定义

```protobuf
message BenchmarkMsg {
  repeated uint32 data = 1;
  optional bytes data_bytes = 2;
}
```

支持两种数据模式：`data_bytes`（原始字节块）和 `data`（repeated uint32 字段）。

## Writer

**通道**：`/apollo/cyber/benchmark`

**命令行参数**：

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| `-s` | 消息大小（支持 K/M 后缀） | 必填 |
| `-t` | 发送频率（Hz） | 必填 |
| `-q` | QoS 策略：0=Reliable, 1=BestEffort | 0 |
| `-d` | 数据类型：0=bytes, 1=repeated field | 0 |
| `-T` | 运行时长（秒） | 10 |
| `-c` | 启用 CPU profiling | 关闭 |
| `-H` | 启用 Heap profiling | 关闭 |

**核心逻辑**（`main` 函数）：

```cpp
auto node = apollo::cyber::CreateNode("cyber_benchmark_writer");
auto writer = node->CreateWriter<BenchmarkMsg>(writer_config);

while (!apollo::cyber::IsShutdown() && send_msg < total_msg) {
  auto trans_unit = std::make_shared<BenchmarkMsg>();
  if (data_type == 0) {
    trans_unit->set_data_bytes(data, message_size);
  } else {
    for (int i = 0; i < num_of_instance; i++) {
      trans_unit->add_data(trans_vec[i]);
    }
  }
  writer->Write(trans_unit);
  rate_ctl.Sleep();
}
```

使用 `apollo::cyber::Rate` 控制发送频率，总发送量 = 频率 × 运行时长。

## Reader

**命令行参数**：

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| `-n` | reader 数量 | 1 |
| `-c` | 启用 CPU profiling | 关闭 |
| `-H` | 启用 Heap profiling | 关闭 |

**核心逻辑**：

```cpp
for (int i = 0; i < nums_of_reader; i++) {
  auto node = apollo::cyber::CreateNode(node_name);
  vec.push_back(node->CreateReader<BenchmarkMsg>(
    reader_config, [](const std::shared_ptr<BenchmarkMsg> m){}));
}
apollo::cyber::WaitForShutdown();
```

每个 reader 创建独立 Node，回调为空（纯测量通信开销）。

## 使用示例

```bash
# 发送端：64KB 消息，10Hz，运行 10 秒
cyber_benchmark_writer -s 64K -t 10 -T 10

# 接收端：10 个 reader 并发
cyber_benchmark_reader -n 10

# 带 CPU profiling
cyber_benchmark_writer -s 1M -t 5 -c -o writer.prof
```

## 调用关系

- **依赖**：`cyber/cyber.h`（Node、Reader、Writer、Rate）、protobuf 消息
- **可选依赖**：gperftools（CPU/Heap profiling）
- **被调用**：手动执行，用于性能回归测试
