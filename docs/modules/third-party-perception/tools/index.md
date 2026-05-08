---
title: "第三方感知转换工具"
---

# Third Party Perception Tools

> 源码路径：`modules/third_party_perception/tools/`

## 概述

第三方感知数据转换工具集，将 Mobileye 摄像头、Delphi/Continental 雷达、SmarterEye 双目等第三方传感器的原始数据转换为 Apollo 统一的 `PerceptionObstacles` 格式。

## 转换器

| 文件 | 命名空间 | 职责 |
| ---- | -------- | ---- |
| `conversion_mobileye.h/cc` | `conversion_mobileye` | Mobileye 数据转感知障碍物 |
| `conversion_radar.h/cc` | `conversion_radar` | 雷达数据转感知障碍物 |
| `conversion_smartereye.h/cc` | `conversion_smartereye` | SmarterEye 双目数据转感知障碍物 |

## 核心函数

### conversion_mobileye::MobileyeToPerceptionObstacles()

```cpp
PerceptionObstacles MobileyeToPerceptionObstacles(
    const Mobileye& mobileye,
    const LocalizationEstimate& localization,
    const Chassis& chassis);
```

**职责**：将 Mobileye 检测结果结合定位和底盘信息转换为感知障碍物列表

### conversion_radar::ContiToRadarObstacles()

```cpp
RadarObstacles ContiToRadarObstacles(
    const ContiRadar& conti_radar,
    const LocalizationEstimate& localization,
    const RadarObstacles& last_radar_obstacles,
    const Chassis& chassis);
```

**职责**：将 Continental 雷达数据转换为内部雷达障碍物格式，支持跨帧跟踪

### conversion_radar::RadarObstaclesToPerceptionObstacles()

```cpp
PerceptionObstacles RadarObstaclesToPerceptionObstacles(
    const RadarObstacles& radar_obstacles);
```

**职责**：将内部雷达障碍物格式转换为统一感知障碍物格式

## 调用关系

- **被调用方**：`ThirdPartyPerceptionComponent` 主组件
- **依赖**：`common/third_party_perception_util`（几何工具）、定位和底盘消息
