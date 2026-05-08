---
title: "音频公共库"
---

# Audio Common

> 源码路径：`modules/audio/common/`

## 概述

音频模块公共库，提供音频数据处理的核心流程编排（`MessageProcess`）、音频信息管理（`AudioInfo`）和全局配置参数（`audio_gflags`）。

## 核心类

### MessageProcess

```cpp
class MessageProcess {
 public:
  static void OnMicrophone(
      const apollo::drivers::microphone::config::AudioData& audio_data,
      const std::string& respeaker_extrinsics_file,
      AudioInfo* audio_info,
      DirectionDetection* direction_detection,
      MovingDetection* moving_detection,
      SirenDetection* siren_detection,
      AudioDetection* audio_detection);
};
```

**职责**：接收麦克风原始音频数据，依次调用方向检测、移动检测、警笛检测，汇总结果到 `AudioDetection` 消息

### AudioInfo

源文件：`audio_info.h/cc`

**职责**：管理音频元信息（采样率、通道数等），为推理模块提供数据预处理

## 配置

源文件：`audio_gflags.h/cc`

## 调用关系

- **被调用方**：`AudioComponent` 主组件在收到麦克风数据时调用 `MessageProcess::OnMicrophone()`
- **调用**：`DirectionDetection`、`MovingDetection`、`SirenDetection`
