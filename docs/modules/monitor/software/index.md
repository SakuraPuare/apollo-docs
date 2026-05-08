---
title: "软件监控"
---

# 软件监控

> 源码路径: `modules/monitor/software/`

## 概述

软件监控子模块负责对 Apollo 系统中各软件组件的运行状态进行周期性检查。所有监控器均继承自 `RecurrentRunner` 基类，通过 `RunOnce()` 方法按固定间隔执行检查逻辑。`SummaryMonitor` 作为总调度器，汇总所有子监控器的结果并发布系统状态。

状态优先级从高到低为：`FATAL > ERROR > WARN > OK > UNKNOWN`。

## 核心类

| 类名 | 文件 | 职责 |
|------|------|------|
| `SummaryMonitor` | summary_monitor.h | 汇总所有监控结果，发布 `SystemStatus` |
| `ChannelMonitor` | channel_monitor.h | 检查通道消息延迟、必填字段、频率 |
| `ProcessMonitor` | process_monitor.h | 检查进程是否存活 |
| `ModuleMonitor` | module_monitor.h | 通过 `NodeManager` 检查模块节点状态 |
| `LatencyMonitor` | latency_monitor.h | 统计通道延迟并发布延迟报告 |
| `FunctionalSafetyMonitor` | functional_safety_monitor.h | 判断是否需要切换安全模式，触发 Guardian |
| `CameraMonitor` | camera_monitor.h | 检查相机设备状态 |
| `LocalizationMonitor` | localization_monitor.h | 检查定位模块状态 |
| `RecorderMonitor` | recorder_monitor.h | 检查数据录制器状态 |

## 核心函数

### SummaryMonitor::EscalateStatus

将组件状态提升到更高优先级的新状态，仅当新状态严重程度高于当前状态时才覆盖。

```cpp
static void EscalateStatus(const ComponentStatus::Status new_status,
                           const std::string& message,
                           ComponentStatus* current_status);
```

在 `RunOnce()` 中，遍历所有组件的 `process_status`、`module_status`、`channel_status`、`resource_status`、`other_status`，逐一调用 `EscalateStatus` 汇总到 `summary`。当状态指纹变化或超过发布间隔时，通过 Writer 发布 `SystemStatus`。

### ProcessMonitor::UpdateStatus

通过读取 `/proc/*/cmdline` 获取所有运行中进程的命令行，与配置中的 `command_keywords` 逐一匹配。

```cpp
static void UpdateStatus(
    const std::vector<std::string>& running_processes,
    const apollo::dreamview::ProcessMonitorConfig& config,
    ComponentStatus* status);
```

若所有关键词均匹配成功，则状态为 `OK`；否则标记为 `FATAL`。检查范围包括 HMI 模块、monitored components、other components 和 global components。

### ChannelMonitor::UpdateStatus

检查通道健康状态，包括三项检查：

```cpp
static void UpdateStatus(
    const apollo::dreamview::ChannelMonitorConfig& config,
    ComponentStatus* status, const bool update_freq, const double freq);
```

1. **延迟检查** — 消息时间戳与当前时间差超过 `delay_fatal` 阈值则报 FATAL，超过 `delay_warn` 则报 WARN
2. **必填字段检查** — 验证消息中 `mandatory_fields` 配置的字段是否存在，缺失则报 ERROR
3. **频率检查** — 通过 `LatencyMonitor::GetFrequency` 获取频率，超出 `[min, max]` 范围则报 WARN

### LatencyMonitor::GetFrequency

查询指定通道的消息频率，供 `ChannelMonitor` 使用。

```cpp
bool GetFrequency(const std::string& channel_name, double* freq);
```

内部维护 `freq_map_` 和 `track_map_`，通过 `UpdateStat` 更新统计数据，`AggregateLatency` 聚合延迟信息，`PublishLatencyReport` 发布延迟报告。

## 调用关系

```text
MonitorManager (调度所有 RecurrentRunner)
 ├── SummaryMonitor        ← 汇总并发布 SystemStatus
 ├── ChannelMonitor        ← 依赖 LatencyMonitor 获取频率
 │    └── LatencyMonitor   ← 统计通道延迟与频率
 ├── ProcessMonitor        ← 读取 /proc 检查进程存活
 ├── ModuleMonitor         ← 通过 NodeManager 检查节点
 ├── FunctionalSafetyMonitor ← 安全检查，触发 Guardian
 ├── CameraMonitor         ← 相机设备状态
 ├── LocalizationMonitor   ← 定位状态
 └── RecorderMonitor       ← 录制器状态
```

各子监控器通过 `SummaryMonitor::EscalateStatus` 上报状态，`SummaryMonitor` 在每次 tick 时汇总所有组件状态并在变化时发布。
