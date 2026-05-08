---
title: "V2X 公共配置"
---

# V2X 公共配置

> 源码路径: `modules/v2x/common/`

## 概述

本模块定义了 V2X（Vehicle-to-Everything）代理系统的全局配置参数。通过 Google gflags 框架声明和定义运行时可配置的标志变量，涵盖 gRPC 通信地址、信号灯处理参数、OBU 消息超时以及 RSU 白名单管理等配置项。

## 核心文件

### v2x_proxy_gflags.h / v2x_proxy_gflags.cc

声明并定义 V2X 代理模块使用的所有 gflags 参数。

```cpp
namespace apollo {
namespace v2x {

// gRPC 通信地址配置
DECLARE_string(grpc_client_host);   // 客户端 IP，默认 "127.0.0.1"
DECLARE_string(grpc_server_host);   // 服务端 IP，默认 "127.0.0.1"
DECLARE_string(grpc_client_port);   // 客户端端口，默认 "50100"
DECLARE_string(grpc_server_port);   // 服务端端口，默认 "50101"
DECLARE_string(grpc_debug_server_port); // 调试端口，默认 "50102"

// 定时器与阈值配置
DECLARE_int64(x2v_traffic_light_timer_frequency); // 信号灯定时频率，默认 10
DECLARE_int64(v2x_car_status_timer_frequency);    // 车辆状态定时频率，默认 10
DECLARE_double(traffic_light_distance);           // 信号灯检测距离，默认 250.0 m
DECLARE_double(heading_difference);               // 最大航向角差，默认 30/180 rad
DECLARE_int64(msg_timeout);                       // OBU 消息超时，默认 250 ms
DECLARE_int64(spat_period);                       // SPAT 消息周期，默认 150 ms
DECLARE_int64(rsu_whitelist_period);              // RSU 白名单刷新周期，默认 3000 ms
DECLARE_string(rsu_whitelist_name);               // RSU 白名单文件路径

}  // namespace v2x
}  // namespace apollo
```

## 参数分类

| 类别 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| gRPC 地址 | `grpc_client_host` | `127.0.0.1` | gRPC 客户端主机 |
| gRPC 地址 | `grpc_server_host` | `127.0.0.1` | gRPC 服务端主机 |
| gRPC 端口 | `grpc_client_port` | `50100` | 客户端端口 |
| gRPC 端口 | `grpc_server_port` | `50101` | 服务端端口 |
| gRPC 端口 | `grpc_debug_server_port` | `50102` | 调试端口 |
| 定时器 | `x2v_traffic_light_timer_frequency` | `10` | X2V 信号灯定时频率 |
| 定时器 | `v2x_car_status_timer_frequency` | `10` | 车辆状态上报频率 |
| 距离阈值 | `traffic_light_distance` | `250.0` | 信号灯关联距离 (m) |
| 角度阈值 | `heading_difference` | `30/180` | 最大航向角差 (rad) |
| 缓存 | `list_size` | `6` | 信号灯数据缓存列表大小 |
| 超时 | `msg_timeout` | `250` | OBU 消息超时 (ms) |
| 仿真 | `sim_sending_num` | `10` | 最大仿真发送次数 |
| 功能开关 | `use_nearest_flag` | `true` | 使用最近信号灯接口 |
| SPAT | `spat_period` | `150` | SPAT 消息周期 (ms) |
| SPAT | `check_time` | `0.5` | SPAT 检查周期 (s) |
| RSU | `rsu_whitelist_period` | `3000` | 白名单刷新周期 (ms) |
| RSU | `rsu_whitelist_name` | 见源码 | RSU 白名单文件路径 |

## 调用关系

本模块作为 V2X 子系统的基础配置层，被以下模块引用：

- `v2x_proxy` — 代理主程序读取 gRPC 地址和超时参数
- `v2x_fusion` — 融合组件读取信号灯距离和航向角阈值
