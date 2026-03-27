# 架构概览

Apollo 采用分层架构，自底向上分为：

## 中间件层 — Cyber RT

Cyber 是 Apollo 的实时通信框架，提供：
- 基于 DAG 的任务调度
- 高性能共享内存通信
- 组件化开发模型
- 服务发现与参数管理

## 应用层 — 模块

### 感知决策链路

```
Perception → Prediction → Planning → Control
```

### 定位导航链路

```
Localization + Map + Routing → Planning
```

### 硬件抽象

Drivers / Canbus 模块封装传感器和车辆底盘接口。

### 可视化与监控

Dreamview 提供 Web 可视化界面，Monitor / Guardian 负责系统健康监控和安全守护。
