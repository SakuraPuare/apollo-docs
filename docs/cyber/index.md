# Cyber 中间件框架

Cyber 是 Apollo 自研的高性能实时通信中间件，替代了早期版本中的 ROS。

## 核心特性

- 高性能共享内存通信
- 基于协程的任务调度
- 组件化开发模型（Component）
- 服务发现与参数服务
- 数据录制与回放（Record）

## 子模块

| 模块 | 职责 |
|------|------|
| Node | 通信节点抽象 |
| Component | 组件基类，支持消息驱动 |
| Message | 消息定义与序列化 |
| Transport | 传输层（SHM / RTPS） |
| Scheduler | 协程调度器 |
| Service Discovery | 服务注册与发现 |
| Parameter | 参数管理 |
| Record | 数据录制回放 |
| Logger | 日志系统 |
| Timer | 定时器 |
| Class Loader | 插件加载 |
| Mainboard | 主执行引擎 |
