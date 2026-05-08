---
title: "模块总览"
order: 1
---

# 模块总览

> 源码路径：`modules/`

## 概述

Apollo 自动驾驶平台采用模块化架构，各功能模块以 CyberRT Component 形式独立运行，通过 channel 通信协作。本节汇总所有模块及其职责。

## 模块列表

| 模块 | 说明 |
| --- | --- |
| [planning](./planning/) | 行为决策与轨迹规划 |
| [control](./control/) | 轨迹跟踪控制（横向/纵向） |
| [perception](./perception/) | 环境感知（LiDAR、相机、雷达） |
| [prediction](./prediction/) | 障碍物轨迹预测 |
| [localization](./localization/) | 高精度定位（RTK/MSF/NDT） |
| [routing](./routing/) | 全局路径规划 |
| [map](./map/) | 高精地图服务 |
| [canbus](./canbus/) | CAN 总线驱动与车辆控制 |
| [drivers](./drivers/) | 传感器驱动（相机、LiDAR、雷达、GNSS） |
| [transform](./transform/) | 坐标变换（TF2） |
| [guardian](./guardian/) | 安全守护（紧急制动） |
| [monitor](./monitor/) | 系统健康监控 |
| [dreamview](./dreamview/) | 可视化调试平台 |
| [dreamview-plus](./dreamview-plus/) | 增强版可视化平台 |
| [audio](./audio/) | 音频感知（紧急车辆检测） |
| [bridge](./bridge/) | 跨主机通信桥接 |
| [calibration](./calibration/) | 传感器标定工具 |
| [data](./data/) | 数据采集与管理 |
| [external-command](./external-command/) | 外部命令接口 |
| [task-manager](./task-manager/) | 任务管理器 |
| [storytelling](./storytelling/) | 场景状态广播 |
| [v2x](./v2x/) | 车路协同通信 |
| [third-party-perception](./third-party-perception/) | 第三方感知设备接入 |
| [common](./common/) | 公共库（数学、状态、工具） |

## 数据流

```text
Drivers → Perception → Prediction → Planning → Control → Canbus → 车辆
            ↑                          ↑
        Localization              Routing/Map
```

## 模块间通信

所有模块通过 [CyberRT](../cyber/) 框架的 channel 机制通信，使用 Protobuf 序列化消息。每个模块以 DAG 文件声明输入输出 channel，由 CyberRT 调度器统一管理。
