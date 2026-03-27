---
title: Data 数据模块
description: Apollo 自动驾驶平台数据管理模块，提供智能录制工具 Smart Recorder，通过事件触发机制选择性录制传感器数据，有效减少存储开销。
---

# Data 数据模块

## 模块职责

Data 模块是 Apollo 自动驾驶平台的数据管理模块，核心功能是提供 **Smart Recorder（智能录制器）** 工具。该工具通过事件驱动的选择性录制策略，在保证关键场景数据完整性的前提下，大幅减少录制数据量。

其核心思路是：

- **小话题（Small Topics）** 始终录制：如定位、底盘、控制指令等数据量小的话题
- **大话题（Large Topics）** 按需录制：如点云、相机图像等传感器数据，仅在特定场景触发时录制

此外，模块还通过 Protobuf 定义了数据帧（`Frame`）和静态信息（`StaticInfo`）等数据结构，用于描述传感器数据帧和车辆/环境/软硬件的元信息。

## 目录结构

```
modules/data/
├── BUILD                          # 顶层构建文件
├── cyberfile.xml                  # 包管理配置
├── README.md
├── proto/
│   ├── frame.proto                # 数据帧定义（点云、相机、雷达等）
│   └── static_info.proto          # 静态信息定义（车辆、环境、软硬件）
└── tools/
    └── smart_recorder/
        ├── smart_recorder.cc      # 主程序入口
        ├── record_processor.h/cc  # 录制处理器基类
        ├── realtime_record_processor.h/cc  # 实时录制处理器
        ├── post_record_processor.h/cc      # 后处理录制处理器
        ├── trigger_base.h/cc      # 触发器基类
        ├── channel_pool.h/cc      # 话题通道池
        ├── interval_pool.h/cc     # 时间区间池
        ├── smart_recorder_gflags.h/cc      # 命令行参数定义
        ├── drive_event_trigger.h/cc        # 驾驶事件触发器
        ├── emergency_mode_trigger.h/cc     # 紧急模式触发器
        ├── hard_brake_trigger.h/cc         # 急刹车触发器
        ├── swerve_trigger.h/cc             # 急转向触发器
        ├── bumper_crash_trigger.h/cc       # 保险杠碰撞触发器
        ├── regular_interval_trigger.h/cc   # 定时触发器
        ├── small_topics_trigger.h/cc       # 小话题触发器
        ├── conf/
        │   └── smart_recorder_config.pb.txt  # 触发器配置
        └── proto/
            └── smart_recorder_triggers.proto # 触发器配置结构定义
```

## 核心类与接口

### RecordProcessor（录制处理器基类）

所有录制处理器的抽象基类，定义了录制处理的核心流程。

```cpp
class RecordProcessor {
 public:
  RecordProcessor(const std::string& source_record_dir,
                  const std::string& restored_output_dir);
  virtual bool Init(const SmartRecordTrigger& trigger_conf);
  virtual bool Process() = 0;
  virtual std::string GetDefaultOutputFile() const = 0;

 protected:
  bool InitTriggers(const SmartRecordTrigger& trigger_conf);
  bool ShouldRestore(const cyber::record::RecordMessage& msg) const;

  std::vector<std::unique_ptr<TriggerBase>> triggers_;
  std::unique_ptr<cyber::record::RecordWriter> writer_;
};
```

`Init()` 方法负责初始化触发器集合、RecordWriter 和时间区间池。`InitTriggers()` 中注册了所有内置触发器实例。

### RealtimeRecordProcessor（实时录制处理器）

继承自 `RecordProcessor`，用于实时场景。在车辆运行过程中，一边通过 Cyber Recorder 录制全量数据到源目录，一边通过快速读取器（fast reader）扫描消息并触发事件，再由慢速读取器（slow reader）将命中区间内的消息写入输出文件。

关键机制：

- 启动 `Recorder` 录制全量数据
- 启动 `MonitorStatus` 线程定期发布录制状态到 `/apollo/data/recorder/status`
- 主循环中逐条消息调用各触发器的 `Pull()` 方法
- `RestoreMessage()` 根据 `IntervalPool` 中的时间区间，选择性地将消息写入输出文件
- 支持过期录制文件自动清理（`reused_record_num` 控制保留数量）

### PostRecordProcessor（后处理录制处理器）

继承自 `RecordProcessor`，用于离线场景。对已完成的录制任务进行两遍扫描：

1. **第一遍扫描**：遍历所有消息，各触发器检测事件并生成时间区间
2. **第二遍扫描**：根据生成的时间区间，将命中的消息写入输出文件

### TriggerBase（触发器基类）

所有触发器的抽象基类，定义了触发器的核心接口。

```cpp
class TriggerBase {
 public:
  virtual bool Init(const SmartRecordTrigger& trigger_conf);
  virtual void Pull(const cyber::record::RecordMessage& msg) = 0;
  virtual bool ShouldRestore(const cyber::record::RecordMessage& msg) const = 0;

 protected:
  void TriggerIt(const uint64_t msg_time) const;
  std::string trigger_name_;
  std::unique_ptr<Trigger> trigger_obj_;
};
```

- `Pull()`：接收消息，判断是否满足触发条件，满足则调用 `TriggerIt()` 向 `IntervalPool` 添加时间区间
- `ShouldRestore()`：判断当前消息是否需要无条件恢复（如小话题）
- `TriggerIt()`：根据配置的 `backward_time` 和 `forward_time`，计算 `[msg_time - backward, msg_time + forward]` 区间并加入池中

### 内置触发器

| 触发器 | 监听话题 | 触发条件 |
|--------|----------|----------|
| `DriveEventTrigger` | `/apollo/common/drive_event` | 收到驾驶事件消息 |
| `EmergencyModeTrigger` | `/apollo/canbus/chassis` | 驾驶模式从 `COMPLETE_AUTO_DRIVE` 切换到 `EMERGENCY_MODE` |
| `HardBrakeTrigger` | `/apollo/canbus/chassis` | 使用双滑动窗口（history 和 current 队列）比较历史窗口与当前窗口的速度均值差，差值超过阈值（10 m/s）时触发 |
| `SwerveTrigger` | `/apollo/canbus/chassis` | 使用双滑动窗口（history 和 current 队列）比较历史窗口与当前窗口的转向百分比均值差，差值超过阈值（10%）时触发 |
| `BumperCrashTrigger` | `/apollo/canbus/chassis` | 前/后保险杠状态变为 `BUMPER_PRESSED` |
| `RegularIntervalTrigger` | 任意消息 | 每 300 秒（5 分钟）定时触发一次 |
| `SmallTopicsTrigger` | 小话题集合 | 不触发事件，但通过 `ShouldRestore()` 确保小话题消息始终被恢复 |

### ChannelPool（话题通道池）

单例类，维护三个话题集合：

- **small_channels_**：定位、底盘、控制、规划、预测、感知等约 30 个小数据量话题
- **large_channels_**：相机（压缩/视频）、激光雷达点云、毫米波雷达等约 27 个大数据量话题
- **all_channels_**：两者的并集

### IntervalPool（时间区间池）

单例类，管理所有触发器产生的时间区间。

- `AddInterval()`：添加新区间，自动与当前区间合并
- `ReorgIntervals()`：按起始时间排序（后处理模式使用）
- `MessageFallIntoRange()`：O(N) 复杂度判断消息时间戳是否落入某个区间
- `LogIntervalEvent()`：将触发事件写入日志文件

## 数据流

### 实时录制模式

```
传感器数据 → Cyber Recorder（全量录制到源目录）
                    ↓
            Fast Reader（逐条读取）
                    ↓
            各 Trigger.Pull()（检测事件）
                    ↓
            IntervalPool（记录时间区间）
                    ↓
            RestoreMessage()（Slow Reader 按区间恢复）
                    ↓
            RecordWriter（写入精简后的输出文件）
```

### 后处理模式

```
已录制的 Record 文件
        ↓
  第一遍扫描：各 Trigger.Pull() → IntervalPool
        ↓
  IntervalPool.ReorgIntervals()（排序合并）
        ↓
  第二遍扫描：MessageFallIntoRange() + ShouldRestore()
        ↓
  RecordWriter（写入精简后的输出文件）
```

### Proto 数据结构

**Frame**（数据帧）：描述单帧传感器数据，包含：
- 设备位姿（`device_position`、`device_heading`、`device_gps_pose`）
- 点云数据（`points`，`Vector4` 含 x/y/z/i 和 is_ground 标记）
- 雷达点（`radar_points`，区分前/后雷达）
- 相机图像（`images`，含内参、畸变参数和通道名）

**StaticInfo**（静态信息）：描述一次数据采集任务的元信息，包含：
- `VehicleInfo`：车辆品牌/型号、CAN 总线配置、车辆参数
- `EnvironmentInfo`：地图名称、温度
- `HardwareInfo`：硬件配置文件映射
- `SoftwareInfo`：Docker 镜像、commit ID、软件配置、最新路由请求
- `UserInfo`：组织、驾驶员、副驾驶员

## 配置方式

### 命令行参数

通过 gflags 定义，在 `smart_recorder_gflags.cc` 中：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--source_records_dir` | `""` | 原始录制文件目录 |
| `--restored_output_dir` | `""` | 处理后输出目录（默认为源目录 + `_restored`） |
| `--smart_recorder_config_filename` | `/apollo/modules/data/tools/smart_recorder/conf/smart_recorder_config.pb.txt` | 触发器配置文件路径 |
| `--real_time_trigger` | `true` | 是否使用实时触发模式 |

### 触发器配置文件

配置文件 `smart_recorder_config.pb.txt` 对应 `SmartRecordTrigger` Protobuf 结构：

```protobuf
message SmartRecordTrigger {
  optional RecordSegmentSetting segment_setting = 1;  // 分段设置
  repeated Trigger triggers = 2;                       // 触发器列表
  optional double max_backward_time = 3;               // 最大回溯时间（秒）
  optional double min_restore_chunk = 4;               // 最小恢复块（秒）
  optional string trigger_log_file_path = 5;           // 触发日志路径
  optional int32 reused_record_num = 6;                // 保留的录制文件数量
}
```

每个触发器可独立配置：

```protobuf
message Trigger {
  optional string trigger_name = 1;    // 触发器名称
  optional bool enabled = 2;           // 是否启用
  optional double backward_time = 3;   // 触发后向前回溯时间（秒）
  optional double forward_time = 4;    // 触发后向后延续时间（秒）
  optional string description = 5;     // 描述
}
```

默认配置中，`EmergencyModeTrigger`（回溯 25s，延续 10s）和 `BumperCrashTrigger`（回溯 60s，延续 30s）处于启用状态。

### 扩展新触发器

1. 在 `smart_recorder_config.pb.txt` 中添加新触发器配置项
2. 创建继承自 `TriggerBase` 的新类，实现 `Pull()` 和 `ShouldRestore()` 接口
3. 在 `RecordProcessor::InitTriggers()` 中注册新触发器实例

### 包依赖

模块依赖 `cyber`、`canbus`、`common`、`monitor`、`dreamview` 等包，通过 `cyberfile.xml` 管理。

### 使用方式

```bash
# 构建 Apollo 后使用脚本启动
python3 /apollo/scripts/record_message.py --help
```
