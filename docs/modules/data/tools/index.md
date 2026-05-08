---
title: "数据工具"
---

# Data Tools

> 源码路径：`modules/data/tools/smart_recorder/`

## 概述

智能录制工具（Smart Recorder），基于可配置的触发器规则，从连续录制的数据中自动提取关键片段（如急刹车、碰撞、紧急模式等事件前后的数据），用于数据回放和问题分析。

## 核心类

### TriggerBase

```cpp
class TriggerBase {
 public:
  virtual bool Init(const SmartRecordTrigger& trigger_conf);
  virtual void Pull(const cyber::record::RecordMessage& msg) = 0;
  virtual bool ShouldRestore(const cyber::record::RecordMessage& msg) const = 0;
};
```

**职责**：触发器基类，定义消息判断和时间区间记录接口

### RecordProcessor

```cpp
class RecordProcessor {
 public:
  RecordProcessor(const std::string& source_record_dir,
                  const std::string& restored_output_dir);
  virtual bool Init(const SmartRecordTrigger& trigger_conf);
  virtual bool Process() = 0;
 protected:
  bool ShouldRestore(const cyber::record::RecordMessage& msg) const;
  std::vector<std::unique_ptr<TriggerBase>> triggers_;
  std::unique_ptr<cyber::record::RecordWriter> writer_;
};
```

**职责**：录制处理器基类，管理触发器集合并决定哪些消息需要保留

## 触发器类型

| 触发器 | 文件 | 触发条件 |
| ------ | ---- | -------- |
| `HardBrakeTrigger` | `hard_brake_trigger.h/cc` | 急刹车事件 |
| `BumperCrashTrigger` | `bumper_crash_trigger.h/cc` | 碰撞事件 |
| `EmergencyModeTrigger` | `emergency_mode_trigger.h/cc` | 紧急模式切换 |
| `DriveEventTrigger` | `drive_event_trigger.h/cc` | 驾驶事件标记 |
| `SwerveTrigger` | `swerve_trigger.h/cc` | 急转向事件 |
| `SmallTopicsTrigger` | `small_topics_trigger.h/cc` | 小话题持续记录 |
| `RegularIntervalTrigger` | `regular_interval_trigger.h/cc` | 定时触发 |

## 处理器类型

| 处理器 | 说明 |
| ------ | ---- |
| `RealtimeRecordProcessor` | 实时录制处理，边录边判断 |
| `PostRecordProcessor` | 后处理模式，对已有录制文件提取 |

## 调用关系

- **入口**：`smart_recorder.cc` main 函数
- **依赖**：CyberRT RecordReader/RecordWriter
