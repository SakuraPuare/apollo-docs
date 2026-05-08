---
title: "路由调试工具"
---

# Routing Tools

> 源码路径：`modules/routing/tools/`

## 概述

提供两个命令行调试工具：`routing_dump` 用于从 Planning 轨迹中提取并保存路由信息，`routing_cast` 用于将保存的路由响应重新发布到 CyberRT 通道。

## routing_dump

源文件：`routing_dump.cc`

**功能**：订阅 Planning 轨迹话题，当轨迹中包含路由调试数据时，将其导出为 protobuf 文本文件后退出。

```cpp
void MessageCallback(
    const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory) {
  if (trajectory->debug().planning_data().has_routing()) {
    std::ofstream dump_file(FLAGS_routing_dump_file);
    dump_file << trajectory->debug().planning_data().routing().DebugString();
    dump_file.close();
    exit(0);
  }
}
```

**参数**：

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `--routing_dump_file` | `/tmp/routing.pb.txt` | 输出文件路径 |

## routing_cast

源文件：`routing_cast.cc`

**功能**：从文件加载 `RoutingResponse` protobuf，以 1Hz 频率持续发布到 `/apollo/raw_routing_response` 通道，用于离线回放或调试。

```cpp
int main(int argc, char *argv[]) {
  apollo::routing::RoutingResponse routing_response;
  apollo::cyber::common::GetProtoFromFile(FLAGS_routing_dump_file,
                                          &routing_response);
  auto cast_writer = cast_node->CreateWriter<apollo::routing::RoutingResponse>(
      "/apollo/raw_routing_response");
  Rate rate(1.0);
  while (apollo::cyber::OK()) {
    cast_writer->Write(routing_response);
    rate.Sleep();
  }
}
```

**参数**：

| 参数 | 默认值 | 说明 |
| ---- | ------ | ---- |
| `--routing_dump_file` | `/tmp/routing.pb.txt` | 输入文件路径 |

## 典型用法

```bash
# 1. 先抓取当前路由数据
./routing_dump --routing_dump_file=/tmp/routing.pb.txt

# 2. 修改后重新发布
./routing_cast --routing_dump_file=/tmp/routing.pb.txt
```

## 调用关系

- **依赖**：CyberRT 通信框架、`adapter_gflags`（话题名定义）
- **数据来源**：`routing_dump` 订阅 Planning 轨迹话题
- **数据输出**：`routing_cast` 发布到 `/apollo/raw_routing_response`
