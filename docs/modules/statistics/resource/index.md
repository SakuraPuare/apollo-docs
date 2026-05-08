---
title: "资源统计"
---

# Resource Statistic

> 源码路径：`modules/statistics/resource/`

## 概述

资源统计组件，定时采集系统级硬件资源使用情况（CPU、GPU、内存、磁盘），汇总为统一指标消息发布。

## 核心类

### ResourceStatisticComponent

```cpp
class ResourceStatisticComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
 private:
  void LoadConf(const ResourceStatisticConf& conf);

  CpuResourceStatistic cpu_statistic_;
  GpuResourceStatistic gpu_statistic_;
  DiskResourceStatistic disk_statistic_;
  MemoryResourceStatistic mem_statistic_;
  std::shared_ptr<cyber::Writer<ResourceMetrics>> metric_writer_;
};
```

## 子统计器

| 类名 | 文件 | 职责 |
| ---- | ---- | ---- |
| `CpuResourceStatistic` | `cpu_resource_statistic.h/cc` | 采集 CPU 使用率 |
| `GpuResourceStatistic` | `gpu_resource_statistic.h/cc` | 采集 GPU 使用率 |
| `MemoryResourceStatistic` | `memory_resource_statistic.h/cc` | 采集内存使用情况 |
| `DiskResourceStatistic` | `disk_resource_statistic.h/cc` | 采集磁盘使用情况 |

## 核心逻辑

`Proc()` 每次触发时依次调用各子统计器采集数据，填充 `ResourceMetrics` protobuf 消息并通过 Writer 发布。

## 调用关系

- **输入**：读取 `/proc/stat`、`/proc/meminfo`、`nvidia-smi` 等系统接口
- **输出**：发布 `ResourceMetrics` 消息
- **配置**：通过 `ResourceStatisticConf` 指定采集间隔和目标设备
