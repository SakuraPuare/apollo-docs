---
title: "路由公共配置"
---

# Routing Common

> 源码路径：`modules/routing/common/`

## 概述

Common 模块定义了 Routing 模块的全局 gflags 配置参数，供 Navigator、AStarStrategy 等核心组件使用。

## 配置参数

源文件：`routing_gflags.h` / `routing_gflags.cc`

| 参数名 | 类型 | 默认值 | 说明 |
| ------ | ---- | ------ | ---- |
| `routing_conf_file` | string | `/apollo/modules/routing/conf/routing_config.pb.txt` | 路由配置 protobuf 文件路径 |
| `routing_node_name` | string | `"routing"` | CyberRT 节点名称 |
| `min_length_for_lane_change` | double | `1.0` | 换道前最小行驶距离（米），参考 Oregon 交通法规 |
| `enable_change_lane_in_result` | bool | `true` | 路由结果中是否包含换道操作 |
| `routing_response_history_interval_ms` | uint32 | `1000` | 路由响应发布间隔（毫秒） |

## 调用关系

- **被调用方**：`AStarStrategy::Search()` 使用 `FLAGS_min_length_for_lane_change` 判断换道空间
- **被调用方**：`RoutingComponent` 使用 `FLAGS_routing_conf_file` 和 `FLAGS_routing_node_name` 初始化
- **被调用方**：`ResultGenerator` 使用 `FLAGS_enable_change_lane_in_result` 决定输出格式
