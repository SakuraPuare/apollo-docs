---
title: "统计模块"
---

# 统计模块 (Statistics)

> 源码路径: `modules/statistics/`

## 概述

统计模块负责 Apollo 自动驾驶系统的运行时监控，包含三个子模块：**延迟统计** (latency)、**进程统计** (process) 和 **资源统计** (resource)。三个子模块均为 Cyber RT 的 `TimerComponent`，按定时器周期采集指标数据，并通过 Cyber Writer 将 Protobuf 消息发布到对应 topic。

整体启动配置在 `statistics.launch` 中声明，加载三个 DAG：

- `resource_statistic.dag`
- `process_statistic.dag`
- `latency_statistic.dag`

## 核心类

### LatencyStatisticComponent

**命名空间:** `apollo::latency_statistic`

**头文件:** `modules/statistics/latency/latency_statistic_component.h`

**职责:** 订阅自动驾驶管线各模块（补偿器、感知、预测、规划、控制）的消息，以 `lidar_timestamp` 为键关联同一帧数据，计算链路延迟 (chain) 和各模块处理延迟 (proc) 的 min/max/avg 统计值。

**关键数据结构:**

```cpp
struct Metric {
    uint64_t compensator = 0;
    uint64_t perception = 0;
    uint64_t prediction = 0;
    uint64_t planning = 0;
    uint64_t control = 0;
};

struct Result {
    std::vector<uint64_t> compensator_chain_list;
    std::vector<uint64_t> compensator_proc_list;
    // ... perception/prediction/planning/control 同上
};
```

**成员变量:**

| 成员 | 说明 |
|------|------|
| `latency_map_` | 以 lidar_timestamp 为键的延迟数据 map |
| `latest_stamp_` | 最新消息时间戳，用于过期清理 |
| `lock_` | 保护 latency_map_ 的互斥锁 |
| `metric_writer_` | 发布 `LatencyMetrics` 的 Writer |

**常量:**

```cpp
constexpr uint32_t MAX_SAMPLE_SIZE = 50;          // 最大采样数
constexpr uint32_t EXPIRED_NANOSECONDS = 3000000000; // 3 秒过期
constexpr uint32_t NANO_TO_MILLI = 1000000;        // 纳秒转毫秒
```

### ProcessStatisticComponent

**命名空间:** `apollo::process_statistic`

**头文件:** `modules/statistics/process/process_statistic_component.h`

**职责:** 周期性扫描 `/proc/[0-9]*/cmdline`，识别系统中运行的三类进程：

1. **mainboard DAG** — 通过 `-d` 参数提取 DAG 路径
2. **mainboard 进程组** — 通过 `-p` 参数提取进程组名
3. **通用进程** — 通过配置中的正则列表匹配

使用缓存机制避免重复解析，每 60 个周期清理超过 10 秒未更新的缓存条目。

**关键数据结构:**

```cpp
struct CacheMetric {
    uint64_t timestamp;
    std::vector<std::string> mainboard_dags;
    std::string mainboard_process;
    std::string general_process;
};
```

**成员变量:**

| 成员 | 说明 |
|------|------|
| `general_process_regex_list_` | 配置加载的通用进程正则列表 |
| `cache_` | cmd_string -> CacheMetric 的哈希缓存 |
| `cycles` | 周期计数器，每 60 次触发缓存清理 |
| `metric_writer_` | 发布 `ProcessMetrics` 的 Writer |

### ResourceStatisticComponent

**命名空间:** `apollo::resource_statistic`

**头文件:** `modules/statistics/resource/resource_statistic_component.h`

**职责:** 聚合四类硬件资源指标的采集器，每次 Proc() 调用收集 CPU、GPU、内存、磁盘的使用数据，打包为 `ResourceMetrics` 消息发布。

**成员变量:**

| 成员 | 说明 |
|------|------|
| `cpu_statistic_` | `CpuResourceStatistic` 实例 |
| `gpu_statistic_` | `GpuResourceStatistic` 实例 |
| `disk_statistic_` | `DiskResourceStatistic` 实例 |
| `mem_statistic_` | `MemoryResourceStatistic` 实例 |
| `metric_writer_` | 发布 `ResourceMetrics` 的 Writer |

### CpuResourceStatistic

**头文件:** `modules/statistics/resource/cpu_resource_statistic.h`

从 `/proc/stat` 读取系统 CPU jiffies，计算瞬时使用率（对比两次采样的 work_jiffies / total_jiffies）。温度从 `/sys/class/hwmon/hwmon*/temp*_input` 读取，取最大值（单位从毫摄氏度转为摄氏度）。

### GpuResourceStatistic

**头文件:** `modules/statistics/resource/gpu_resource_statistic.h`

平台差异化实现：

- **x86_64:** 通过 NVML 库获取 GPU 数量、使用率和温度，取多卡平均使用率和最高温度
- **aarch64 (Jetson):** 解析 `tegrastats` 命令输出，正则提取 `GR3D_FREQ` 使用率和 `GPU@温度C`

### MemoryResourceStatistic

**头文件:** `modules/statistics/resource/memory_resource_statistic.h`

从 `/proc/meminfo` 读取 `MemTotal` 和 `MemAvailable`（前 3 行），计算 used = total - available，输出 total/used/available (MB) 和使用率百分比。

### DiskResourceStatistic

**头文件:** `modules/statistics/resource/disk_resource_statistic.h`

启动时通过正则配置匹配 `/proc/mounts` 中的磁盘设备，运行时从 `/proc/diskstats` 读取 IO 时间计算磁盘负载百分比，从 `boost::filesystem::space` 获取容量使用情况。

## 核心函数

### LatencyStatisticComponent

| 函数 | 说明 |
|------|------|
| `Init()` | 从 gflags 初始化各模块 topic，创建 Reader 订阅和 Writer 发布 |
| `Proc()` | 遍历 latency_map_，清理过期/超量数据，计算延迟并发布 |
| `CreateReaders()` | 为五个模块 topic 创建带回调的 Reader |
| `CreateReader<T>(channel_name)` | 模板函数，回调中按 lidar_timestamp 关联消息并记录时间戳 |
| `FillResult(lidar_timestamp, metric, result)` | 将单帧延迟差值（纳秒转毫秒）填入 chain_list 和 proc_list |
| `CalculateLatency(result, metrics)` | 对 chain/proc 列表分别调用 GetLatencyMetric 计算统计值 |
| `GetLatencyMetric(items, metric)` | 计算向量的 min/max/avg 写入 Protobuf |

### ProcessStatisticComponent

| 函数 | 说明 |
|------|------|
| `Init()` | 加载 Protobuf 配置，创建 Writer |
| `LoadConf(conf)` | 从配置的 `general_process.regex()` 加载正则列表 |
| `Proc()` | 扫描 /proc/cmdline，先查缓存后解析，周期性清理缓存 |
| `ParseFromCache(cmd_string, metrics)` | 缓存命中则直接填充 metrics，未命中则创建新缓存条目 |
| `ParseCmd(cmd_string, metrics, cache_metric)` | 解析 mainboard 的 `-d`/`-p` 参数或匹配通用进程正则 |
| `CleanCache()` | 清除 timestamp 超过 10 秒的缓存条目 |

### ResourceStatisticComponent

| 函数 | 说明 |
|------|------|
| `Init()` | 加载 Protobuf 配置（含磁盘配置），设置磁盘采集间隔，创建 Writer |
| `LoadConf(conf)` | 将 `disk_conf` 传递给 `DiskResourceStatistic` |
| `Proc()` | 依次调用 CPU/GPU/Memory/Disk 的采集方法填充 ResourceMetrics |

### 工具函数

| 函数 | 文件 | 说明 |
|------|------|------|
| `GetStatsLines(stat_file, line_count)` | `resource/util.h` | 读取指定文件的前 N 行，用于读取 /proc 下的伪文件系统 |

## 配置

### gflags 配置

| 标志 | 默认值 | 说明 |
|------|--------|------|
| `FLAGS_latency_statistic_topic` | `/apollo/statistics/latency` | 延迟指标发布 topic |
| `FLAGS_compensator_topic` | `/apollo/sensor/lidar/compensator/PointCloud2` | 激光雷达补偿器 topic |
| `FLAGS_process_statistic_topic` | `/apollo/statistics/process` | 进程指标发布 topic |
| `FLAGS_resource_statistic_topic` | `/apollo/statistics/resources` | 资源指标发布 topic |

其余延迟统计引用的 topic（`perception_obstacle_topic`、`prediction_topic`、`planning_trajectory_topic`、`control_command_topic`）定义在 `modules/common/adapters/adapter_gflags.h` 中。

### Protobuf 配置

- **ProcessStatisticConf** — 通过 `ComponentBase::GetProtoConfig()` 加载，包含 `general_process.regex()` 正则列表
- **ResourceStatisticConf** — 通过 `ComponentBase::GetProtoConfig()` 加载，包含 `disk_conf.device()` 磁盘设备匹配模式

## 调用关系

```text
statistics.launch
 +-- ResourceStatisticComponent (TimerComponent)
 |    +-- CpuResourceStatistic::GetCpuMetric()
 |    |    +-- GetUsage()        -> /proc/stat
 |    |    +-- GetTemperature()  -> /sys/class/hwmon/...
 |    +-- GpuResourceStatistic::GetGpuMetric()
 |    |    +-- [x86_64] NVML API (nvmlInit -> GetCount -> GetHandle -> GetUtilization/GetTemperature)
 |    |    +-- [aarch64] tegrastats 命令解析
 |    +-- MemoryResourceStatistic::GetMemoryMetric()
 |    |    +-- GetMemoryValueFromLine() -> /proc/meminfo
 |    +-- DiskResourceStatistic::GetDiskMetric()
 |         +-- LoadDiskConf() -> /proc/mounts 匹配设备
 |         +-- GetDiskMetric() -> /proc/diskstats + boost::filesystem::space
 |
 +-- ProcessStatisticComponent (TimerComponent)
 |    +-- ParseFromCache()  -- 缓存命中
 |    +-- ParseCmd()        -- 缓存未命中时解析 /proc/<PID>/cmdline
 |    +-- CleanCache()      -- 每 60 周期清理过期缓存
 |
 +-- LatencyStatisticComponent (TimerComponent)
      +-- CreateReader<T>() -- 订阅 5 个模块 topic 的回调
      +-- Proc()
           +-- FillResult()        -- 计算延迟差值
           +-- CalculateLatency()  -- 聚合统计
                +-- GetLatencyMetric() -- 计算 min/max/avg
```

**数据流向:** 各 Component 的 Proc() 周期性采集指标，通过 `cyber::Writer` 发布到对应 topic。下游模块（如 Dreamview 或日志系统）可订阅这些 topic 获取系统运行状态。
