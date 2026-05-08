---
title: "硬件监控"
---

# Hardware Monitor

> 源码路径：`modules/monitor/hardware/`

## 概述

硬件监控模块包含四个 `RecurrentRunner` 子类，分别监控 GPS、ESD CAN、SocketCAN 和系统资源（磁盘/CPU/内存/磁盘负载）。所有监控器由 `MonitorComponent` 主循环定时调用。

## 核心类

| 类名 | 文件 | 职责 |
| ---- | ---- | ---- |
| `GpsMonitor` | `gps_monitor.h/cc` | 检查 GPS 设备状态 |
| `EsdCanMonitor` | `esdcan_monitor.h/cc` | 检查 ESD CAN 总线状态 |
| `SocketCanMonitor` | `socket_can_monitor.h/cc` | 检查 SocketCAN 接口状态 |
| `ResourceMonitor` | `resource_monitor.h/cc` | 检查磁盘空间、CPU、内存、磁盘负载 |

所有类继承 `RecurrentRunner`，实现 `RunOnce(current_time)` 方法。

### ResourceMonitor

```cpp
class ResourceMonitor : public RecurrentRunner {
 public:
  ResourceMonitor();
  void RunOnce(const double current_time) override;
 private:
  static void UpdateStatus(const ResourceMonitorConfig& config,
                           ComponentStatus* status);
  static void CheckDiskSpace(const ResourceMonitorConfig& config,
                             ComponentStatus* status);
  static void CheckCPUUsage(const ResourceMonitorConfig& config,
                            ComponentStatus* status);
  static void CheckMemoryUsage(const ResourceMonitorConfig& config,
                               ComponentStatus* status);
  static void CheckDiskLoads(const ResourceMonitorConfig& config,
                             ComponentStatus* status);
};
```

## 核心函数

### ResourceMonitor::RunOnce()

**职责**：遍历 HMI 模式中配置的所有 `ResourceMonitorConfig`，对每个组件执行资源检查

### ResourceMonitor::CheckDiskSpace()

**职责**：检查配置路径的磁盘剩余空间，低于阈值时上报 WARN/ERROR

### ResourceMonitor::CheckCPUUsage()

**职责**：通过读取 `/proc/<pid>/stat` 计算指定进程的 CPU 使用率，超阈值告警

### ResourceMonitor::CheckMemoryUsage()

**职责**：通过读取 `/proc/<pid>/status` 中 VmRSS 计算内存使用率，超阈值告警

### ResourceMonitor::CheckDiskLoads()

**职责**：通过 `/proc/diskstats` 计算磁盘 IO 负载，超阈值告警

## 配置

资源监控阈值通过 `ResourceMonitorConfig` protobuf 配置（在 HMI 模式文件中定义）：

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `disk_spaces` | repeated | 磁盘路径及空间阈值 |
| `cpu_usages` | repeated | 进程 DAG 路径及 CPU 阈值 |
| `memory_usages` | repeated | 进程 DAG 路径及内存阈值 |
| `disk_load_usages` | repeated | 设备名及 IO 负载阈值 |

## 调用关系

- **被调用方**：`MonitorComponent` 主循环通过 `Tick()` 调用各监控器
- **依赖**：`MonitorManager`（获取配置和状态）、`SummaryMonitor::EscalateStatus()`（上报告警）
- **数据源**：Linux `/proc` 文件系统、`boost::filesystem`
