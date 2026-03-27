---
title: Cyber 中间件框架
---

# Cyber 中间件框架

Cyber RT 是 Apollo 自研的高性能实时通信中间件，替代了早期版本中的 ROS。它为自动驾驶系统提供了一套完整的进程间/跨主机通信、任务调度、服务发现和数据管理基础设施。

## 架构概览

Cyber RT 采用分层架构设计：

- **应用层**：开发者通过 `Component` / `TimerComponent` 构建功能模块，由 `Mainboard` 加载和管理
- **通信层**：`Node` 提供 Channel（发布-订阅）和 Service（请求-响应）两种通信模式
- **传输层**：`Transport` 根据通信双方的拓扑关系自动选择 Intra-process / SHM / RTPS 传输方式
- **调度层**：`Scheduler` 基于用户态协程（CRoutine）实现高效任务调度，支持 Classic 和 Choreography 两种策略
- **基础设施层**：服务发现、参数服务、日志、定时器、数据录制回放等

## 核心特性

- 三级传输自动选择：进程内零拷贝、共享内存跨进程、RTPS 跨主机
- 基于协程的非抢占式调度，支持 CPU 亲和性绑定和实时调度策略
- 组件化开发模型，支持 1~4 路消息融合触发和定时触发
- 去中心化服务发现，基于 Fast-RTPS 的 Participant 自动感知
- 分层时间轮定时器，2ms 精度，内置累积误差补偿
- 高性能异步日志，按模块名分文件输出
- Record 文件录制回放，支持 chunk 分段和 BZ2/LZ4 压缩
- 动态类加载与插件管理，支持 XML 描述的懒加载机制

## 子模块文档

| 模块 | 职责 | 文档 |
|------|------|------|
| Node | 通信节点抽象，提供 Reader/Writer/Service/Client 创建接口 | [node](./node.md) |
| Component | 组件基类，支持消息驱动（1~4路）和定时驱动两种模型 | [component](./component.md) |
| Message | 消息类型系统，基于 SFINAE 的编译期 trait 分发和 protobuf 序列化 | [message](./message.md) |
| Transport | 传输层，Intra/SHM/RTPS 三种机制 + Hybrid 自动选择 | [transport](./transport.md) |
| Scheduler | 协程调度器，Classic（多组优先级队列）和 Choreography（编排式）策略 | [scheduler](./scheduler.md) |
| Timer | 分层时间轮定时器，驱动 TimerComponent 的周期执行 | [timer](./timer.md) |
| Service Discovery | 去中心化拓扑发现，TopologyManager + NodeManager/ChannelManager/ServiceManager | [service-discovery](./service-discovery.md) |
| Parameter | 参数服务，基于 Service/Client 模式的分布式参数读写 | [parameter](./parameter.md) |
| Record | 数据录制回放，chunk 分段 + 文件分段 + 压缩 | [record](./record.md) |
| Logger | 异步日志系统，按模块名分文件 + 双缓冲异步写入 | [logger](./logger.md) |
| Class Loader | 动态类加载与插件管理，dlopen + 工厂模式 + XML 插件描述 | [class-loader](./class-loader.md) |
| Mainboard | 主程序入口，DAG 文件解析 + 组件加载 + 生命周期管理 | [mainboard](./mainboard.md) |

## 典型数据流

```
应用层 Component::Proc()
  ↑ 调度器触发协程
  ↑ DataVisitor 检测数据就绪
  ↑ DataDispatcher 分发消息
  ↑ Receiver 接收消息
  ↑ Transport 层（Intra / SHM / RTPS）
  ↑ Transmitter 发送消息
  ↑ Writer::Write()
```
