# Monitor 系统监控模块

## 模块职责

Monitor 模块是 Apollo 自动驾驶平台的系统级健康监控组件，负责实时检测硬件设备状态、软件进程运行情况、通道消息时效性以及系统资源使用率。其核心职责包括：

- 监控 CAN 总线（ESD CAN / Socket CAN）硬件连通性
- 检测 GPS 信号质量与定位状态
- 验证各模块进程是否正常运行
- 监控 Cyber RT 通道消息的延迟、频率与字段完整性
- 跟踪系统资源（磁盘空间、CPU、内存、磁盘负载）使用情况
- 汇总所有子监控器的结果，发布统一的 `SystemStatus` 消息
- 在自动驾驶模式下执行功能安全检查，必要时触发紧急停车（EStop）

Monitor 作为 Cyber RT 的 `TimerComponent` 运行，默认每 **500ms** 触发一次 `Proc()` 调用，驱动所有子监控器按各自的独立周期执行检测。

## 架构概览

```
┌─────────────────────────────────────────────────────────┐
│                    Monitor (TimerComponent)              │
│                    interval: 500ms                       │
│                                                         │
│  ┌─────────────────────────────────────────────────┐    │
│  │            MonitorManager (Singleton)            │    │
│  │  - HMI 模式配置管理                               │    │
│  │  - SystemStatus 状态维护                          │    │
│  │  - Cyber Reader/Writer 工厂                       │    │
│  │  - 自动驾驶模式检测                                │    │
│  └─────────────────────────────────────────────────┘    │
│                                                         │
│  runners_: vector<shared_ptr<RecurrentRunner>>           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ EsdCanMonitor│  │SocketCan     │  │ GpsMonitor   │  │
│  │ (3s)         │  │Monitor (3s)  │  │ (3s)         │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │Localization  │  │ CameraMonitor│  │ProcessMonitor│  │
│  │Monitor (5s)  │  │ (5s)         │  │ (1.5s)       │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ModuleMonitor │  │LatencyMonitor│  │ChannelMonitor│  │
│  │ (1.5s)       │  │ (1.5s)       │  │ (5s)         │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ResourceMonit │  │SummaryMonitor│  │Functional    │  │
│  │or (5s)       │  │ (每帧)       │  │SafetyMon (每帧)│ │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## 核心类与接口

### Monitor

主组件类，继承自 `apollo::cyber::TimerComponent`，是整个监控系统的入口。

```cpp
// modules/monitor/monitor.h
class Monitor : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
 private:
  std::vector<std::shared_ptr<RecurrentRunner>> runners_;
};
```

- `Init()` — 初始化 `MonitorManager` 单例，按顺序创建所有子监控器并加入 `runners_` 列表
- `Proc()` — 每 500ms 被 Cyber RT 调用一次，依次调用 `MonitorManager::StartFrame()` → 各 runner 的 `Tick()` → `MonitorManager::EndFrame()`

通过 `CYBER_REGISTER_COMPONENT(Monitor)` 宏注册为 Cyber RT 组件。

### RecurrentRunner

所有子监控器的抽象基类，提供基于时间间隔的周期性执行机制。

```cpp
// modules/monitor/common/recurrent_runner.h
class RecurrentRunner {
 public:
  RecurrentRunner(const std::string &name, const double interval);
  void Tick(const double current_time);
  virtual void RunOnce(const double current_time) = 0;
 protected:
  std::string name_;
  unsigned int round_count_ = 0;
 private:
  double interval_;
  double next_round_ = 0;
};
```

`Tick()` 方法根据 `interval_` 判断是否到达执行时间，到达后调用子类实现的 `RunOnce()`。特殊情况：当 `ProcessMonitor` 检测到 `detect_immediately` 标志时（HMI 有期望模块需要启动），会跳过间隔限制立即执行。

### MonitorManager

集中式的配置与状态管理单例，所有子监控器通过它访问共享状态。

```cpp
// modules/monitor/common/monitor_manager.h
class MonitorManager {
 public:
  void Init(const std::shared_ptr<apollo::cyber::Node>& node);
  bool StartFrame(const double current_time);
  void EndFrame();

  const apollo::dreamview::HMIMode& GetHMIMode() const;
  bool IsInAutonomousMode() const;
  SystemStatus* GetStatus();
  apollo::common::monitor::MonitorLogBuffer& LogBuffer();

  template <class T>
  std::shared_ptr<cyber::Reader<T>> CreateReader(const std::string& channel);
  template <class T>
  std::shared_ptr<cyber::Writer<T>> CreateWriter(const std::string& channel);
};
```

核心职责：

- **帧管理** — `StartFrame()` 读取最新的 `HMIStatus`，检测模式切换并更新监控组件列表；`EndFrame()` 发布监控日志
- **模式感知** — 当 HMI 模式变化时，重新加载 `mode_config_`，清空并重建 `components`、`other_components`、`global_components` 和 `hmi_modules` 的状态映射
- **自动驾驶检测** — `CheckAutonomousDriving()` 通过读取 `Chassis` 消息判断当前是否处于完全自动驾驶模式（排除仿真、回放、旧消息等情况）
- **Reader 缓存** — 使用 `unordered_map` 缓存已创建的 Reader，避免重复创建

### SummaryMonitor::EscalateStatus

全局状态升级工具函数，被所有子监控器调用来设置组件状态。遵循优先级规则：

```
FATAL > ERROR > WARN > OK > UNKNOWN
```

只有当新状态的严重程度高于当前状态时才会覆盖，确保最严重的问题始终被保留。

## 子监控器详解

### 硬件监控器

#### EsdCanMonitor

监控 ESD CAN 卡的连通性。通过 NTCAN API 打开 CAN 设备句柄并执行测试，检查返回状态码。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 3 秒 |
| 组件名 | `ESD-CAN` |
| 编译条件 | 需定义 `USE_ESD_CAN` 宏 |

当未定义 `USE_ESD_CAN` 时，始终报告 `ERROR` 状态。

#### SocketCanMonitor

监控 Linux Socket CAN 接口。通过创建 `PF_CAN` 套接字、设置过滤器、绑定 `can0` 接口来验证 CAN 总线可用性。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 3 秒 |
| 组件名 | `SocketCAN` |

#### GpsMonitor

监控 GPS 信号质量。订阅 `GnssBestPose` 消息，根据解算类型（`SolutionType`）判断状态：

| 解算类型 | 状态 |
|----------|------|
| `NARROW_INT` | OK |
| `SINGLE` | WARN |
| 其他 | ERROR |
| 无消息 | ERROR |

检测间隔为 3 秒。需要在 HMI 模式配置中将 `FLAGS_gps_component_name` 加入 `monitored_components` 才会生效。

#### ResourceMonitor

监控系统资源使用情况，支持对 `monitored_components` 和 `global_components` 中配置了 `resource` 的组件进行检测。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 5 秒 |

检测维度包括：

- **磁盘空间** — 通过 `boost::filesystem::space()` 获取指定路径的可用空间，与配置的阈值比较
- **CPU 使用率** — 读取 `/proc/<pid>/stat` 计算进程 CPU 占用百分比（基于 jiffies 差值）
- **内存使用率** — 读取 `/proc/<pid>/statm` 获取进程驻留内存（RSS），单位为 GB
- **系统内存使用率** — 读取 `/proc/meminfo` 计算系统整体内存使用百分比
- **磁盘负载** — 读取 `/proc/diskstats` 计算磁盘 I/O 繁忙百分比

每个维度均支持 `warning` 和 `error` 两级阈值配置。

### 软件监控器

#### ProcessMonitor

监控进程是否正在运行。扫描 `/proc/*/cmdline` 获取所有运行中进程的命令行，与 HMI 模式配置中的 `command_keywords` 进行匹配。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 1.5 秒 |
| 未匹配状态 | FATAL |

检测范围覆盖四类组件：
1. `hmi_modules` — HMI 模块
2. `monitored_components` — 受监控组件（需配置 `process`）
3. `other_components` — 其他组件
4. `global_components` — 全局组件（需配置 `process`）

#### ModuleMonitor

通过 Cyber RT 的 `NodeManager` 检测模块节点是否在线。与 `ProcessMonitor` 不同，它验证的是 Cyber RT 拓扑中的节点注册状态而非操作系统进程。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 1.5 秒 |
| 节点缺失状态 | FATAL |

对 `monitored_components` 中配置了 `module` 的组件，检查其 `node_name` 列表中的所有节点是否都已注册。

#### ChannelMonitor

监控 Cyber RT 通道消息的健康状态，与 `LatencyMonitor` 协作获取频率数据。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 5 秒 |

检测维度：
- **消息存在性** — 通道是否有消息到达，空消息报 `FATAL`
- **消息延迟** — 通过 `Reader::GetDelaySec()` 获取延迟，超过 `delay_fatal` 阈值报 `FATAL`
- **必填字段** — 递归验证 protobuf 消息中配置的 `mandatory_fields` 是否存在
- **消息频率** — 从 `LatencyMonitor` 获取频率，超出 `[min_frequency_allowed, max_frequency_allowed]` 范围报 `WARN`

支持的通道类型包括：`ControlCommand`、`LocalizationEstimate`、`PerceptionObstacles`、`PredictionObstacles`、`ADCTrajectory`、`PointCloud`（多种型号）、`ChassisDetail`、`ContiRadar`、`NavigationInfo` 等。

#### LatencyMonitor

跟踪各模块的消息处理延迟，计算端到端（E2E）延迟统计，并为 `ChannelMonitor` 提供频率数据。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 1.5 秒 |
| 报告发布间隔 | 15 秒 |
| Reader 队列深度 | 30 |

工作流程：
1. 订阅 `FLAGS_latency_recording_topic` 通道，读取各模块上报的 `LatencyRecordMap`
2. 按 `message_id` 聚合各模块的处理时间记录到 `track_map_`
3. 根据记录的时间范围计算各通道的消息频率，存入 `freq_map_`（供 `ChannelMonitor` 查询）
4. 每 15 秒调用 `AggregateLatency()` 汇总统计并发布 `LatencyReport`

延迟统计包含两个维度：
- **模块延迟** — 每个模块自身的处理耗时（`end_time - begin_time`），统计 min/max/average/sample_size
- **E2E 延迟** — 从 `pointcloud` 起点到各下游模块的端到端延迟，例如 `pointcloud -> perception`、`pointcloud -> planning`、`pointcloud -> control`

#### LocalizationMonitor

监控定位模块的融合状态。订阅 `LocalizationStatus` 消息，将 `MeasureState` 映射为组件状态：

| MeasureState | ComponentStatus |
|-------------|-----------------|
| `OK` | OK |
| `WARNNING` | WARN |
| `ERROR` | WARN |
| `CRITICAL_ERROR` | ERROR |
| `FATAL_ERROR` | FATAL |

检测间隔为 5 秒。需要在 HMI 模式配置中将 `FLAGS_localization_component_name` 加入 `monitored_components`。

#### CameraMonitor

监控相机设备状态。遍历预定义的相机 topic 集合，检测是否有且仅有一个相机在发布图像数据。

| 属性 | 值 |
|------|-----|
| 检测间隔 | 5 秒 |
| 组件名 | `Camera` |

检测逻辑：
- 无相机检测到 → `ERROR`
- 检测到多个相机 → `ERROR`（仅允许一个）
- 检测到一个相机 → `OK`，并报告 `frame_id`

#### RecorderMonitor

监控智能录制器（SmartRecorder）的运行状态。订阅 `SmartRecorderStatus` 消息。

| RecordingState | ComponentStatus |
|---------------|-----------------|
| `RECORDING` | OK |
| `STOPPED` | OK |
| `TERMINATING` | WARN |

检测间隔为 5 秒。

#### SummaryMonitor

汇总监控器，每帧都执行（interval = 0）。负责：

1. 遍历所有 `components` 和 `global_components`，将各维度状态（`process_status`、`module_status`、`channel_status`、`resource_status`、`other_status`）按优先级升级合并为 `summary`
2. 对 `SystemStatus` 进行序列化哈希，仅在状态变化或超过 1 秒未发布时，通过 `FLAGS_system_status_topic` 发布

#### FunctionalSafetyMonitor

功能安全监控器，每帧执行，仅在自动驾驶模式下生效。通过 `FLAGS_enable_functional_safety` 控制是否启用。

安全检查流程：

```
CheckSafety()
  ├── 非自动驾驶模式 → 安全（跳过检查）
  ├── 检查 HMI 模块中 required_for_safety=true 的模块状态
  │   └── 任一模块 ERROR/FATAL → 不安全
  └── 检查 monitored_components 中 required_for_safety=true 的组件 summary
      └── 任一组件 ERROR/FATAL → 不安全
```

不安全时的处理时序：

1. 首次检测到不安全 → 设置 `passenger_msg = "Error! Please disengage."`，记录 `safety_mode_trigger_time`
2. 持续不安全且超过 `FLAGS_safety_mode_seconds_before_estop`（默认 10 秒）→ 设置 `require_emergency_stop = true`，触发紧急停车
3. 恢复安全 → 清除所有安全模式标志

## 数据流

### 输入数据

| 数据源 | 通道/来源 | 用途 |
|--------|----------|------|
| HMIStatus | `FLAGS_hmi_status_topic` | 获取当前 HMI 模式，决定监控哪些组件 |
| Chassis | `FLAGS_chassis_topic` | 判断是否处于自动驾驶模式 |
| GnssBestPose | `FLAGS_gnss_best_pose_topic` | GPS 信号质量检测 |
| LocalizationStatus | `FLAGS_localization_msf_status` | 定位融合状态 |
| SmartRecorderStatus | `FLAGS_recorder_status_topic` | 录制器状态 |
| LatencyRecordMap | `FLAGS_latency_recording_topic` | 各模块延迟记录 |
| Image | 多个相机 topic | 相机在线检测 |
| 各业务通道消息 | control/planning/perception 等 | 通道健康检测 |
| /proc 文件系统 | cmdline/stat/statm/meminfo/diskstats | 进程与资源监控 |
| Cyber RT NodeManager | 拓扑服务 | 模块节点在线检测 |
| HMI 模式配置文件 | Dreamview 配置 | 监控规则定义 |

### 输出数据

| 输出 | 通道 | 说明 |
|------|------|------|
| SystemStatus | `FLAGS_system_status_topic` | 包含所有组件的综合状态，由 SummaryMonitor 发布 |
| LatencyReport | `FLAGS_latency_reporting_topic` | 模块延迟与 E2E 延迟统计报告 |
| MonitorLog | 通过 MonitorLogBuffer | 监控日志，每帧结束时发布 |

### 帧内执行流程

```
Monitor::Proc()
  │
  ├── MonitorManager::StartFrame(current_time)
  │   ├── 读取最新 HMIStatus
  │   ├── 检测模式切换，必要时重建组件映射
  │   ├── 清除上一帧的 component summary
  │   └── CheckAutonomousDriving() 判断驾驶模式
  │
  ├── 依次 Tick 所有 runners（按注册顺序）
  │   ├── EsdCanMonitor.Tick()
  │   ├── SocketCanMonitor.Tick()
  │   ├── GpsMonitor.Tick()
  │   ├── LocalizationMonitor.Tick()
  │   ├── CameraMonitor.Tick()
  │   ├── ProcessMonitor.Tick()
  │   ├── ModuleMonitor.Tick()
  │   ├── LatencyMonitor.Tick()
  │   ├── ChannelMonitor.Tick()      ← 依赖 LatencyMonitor 的频率数据
  │   ├── ResourceMonitor.Tick()
  │   ├── SummaryMonitor.Tick()      ← 汇总上述所有结果
  │   └── FunctionalSafetyMonitor.Tick() ← 基于汇总结果做安全决策
  │
  └── MonitorManager::EndFrame()
      └── 发布监控日志
```

## 配置方式

### DAG 配置

```protobuf
# modules/monitor/dag/monitor.dag
module_config {
    module_library : "modules/monitor/libmonitor.so"
    timer_components {
        class_name : "Monitor"
        config {
            name: "monitor"
            interval: 500        # 主循环间隔，单位毫秒
        }
    }
}
```

### Launch 配置

```xml
<!-- modules/monitor/launch/monitor.launch -->
<cyber>
    <module>
        <name>monitor</name>
        <dag_conf>/apollo/modules/monitor/dag/monitor.dag</dag_conf>
        <process_name>monitor</process_name>
    </module>
</cyber>
```

### GFlags 参数

各子监控器通过 GFlags 定义可调参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_functional_safety` | `true` | 是否启用功能安全检查 |
| `safety_mode_seconds_before_estop` | `10.0` | 安全模式触发后到 EStop 的等待时间（秒） |
| `system_status_publish_interval` | `1.0` | SystemStatus 最大发布间隔（秒） |
| `esdcan_monitor_interval` | `3.0` | ESD CAN 检测间隔（秒） |
| `esdcan_id` | `0` | ESD CAN 设备 ID |
| `socket_can_monitor_interval` | `3.0` | Socket CAN 检测间隔（秒） |
| `gps_monitor_interval` | `3.0` | GPS 检测间隔（秒） |
| `resource_monitor_interval` | `5.0` | 资源检测间隔（秒） |
| `process_monitor_interval` | `1.5` | 进程检测间隔（秒） |
| `module_monitor_interval` | `1.5` | 模块检测间隔（秒） |
| `channel_monitor_interval` | `5.0` | 通道检测间隔（秒） |
| `latency_monitor_interval` | `1.5` | 延迟监控间隔（秒） |
| `latency_report_interval` | `15.0` | 延迟报告发布间隔（秒） |
| `latency_reader_capacity` | `30` | 延迟 Reader 队列深度 |
| `camera_monitor_interval` | `5.0` | 相机检测间隔（秒） |
| `smart_recorder_monitor_interval` | `5.0` | 录制器检测间隔（秒） |
| `localization_monitor_interval` | `5.0` | 定位检测间隔（秒） |

### HMI 模式配置

监控行为由 Dreamview 的 HMI 模式配置文件驱动。每个模式定义了需要监控的组件及其检测规则：

- `monitored_components` — 核心受监控组件，可配置 `process`（进程检测）、`channel`（通道检测）、`resource`（资源检测）、`module`（节点检测）
- `other_components` — 其他组件，仅做进程检测
- `global_components` — 全局组件，支持 `process` 和 `resource` 检测
- `modules` — HMI 模块，配置 `process_monitor_config` 和 `required_for_safety` 标志

`ResourceMonitorConfig` 支持的子配置：
- `disk_spaces` — 磁盘空间阈值（`insufficient_space_warning`、`insufficient_space_error`）
- `cpu_usages` — CPU 使用率阈值（`high_cpu_usage_warning`、`high_cpu_usage_error`），可指定 `process_dag_path` 监控特定进程
- `memory_usages` — 内存使用率阈值（`high_memory_usage_warning`、`high_memory_usage_error`）
- `disk_load_usages` — 磁盘负载阈值（`high_disk_load_warning`、`high_disk_load_error`），需指定 `device_name`

`ChannelMonitorConfig` 支持的子配置：
- `name` — 通道名称
- `delay_fatal` — 延迟致命阈值（秒）
- `mandatory_fields` — 必须存在的 protobuf 字段路径（支持嵌套，如 `header.timestamp_sec`）
- `min_frequency_allowed` / `max_frequency_allowed` — 频率允许范围

## 系统监控指标

### 组件状态等级

```
UNKNOWN < OK < WARN < ERROR < FATAL
```

每个被监控组件维护以下状态维度：

| 状态维度 | 来源监控器 | 说明 |
|----------|-----------|------|
| `process_status` | ProcessMonitor | 进程是否运行 |
| `module_status` | ModuleMonitor | Cyber RT 节点是否在线 |
| `channel_status` | ChannelMonitor | 通道消息健康度 |
| `resource_status` | ResourceMonitor | 资源使用情况 |
| `other_status` | GPS/CAN/Camera/Localization/Recorder | 特定硬件或软件状态 |
| `summary` | SummaryMonitor | 上述所有维度的最高严重级别 |

### SystemStatus 消息结构

`SystemStatus` 是 Monitor 模块的核心输出，包含：

- `header` — 消息头
- `hmi_modules` — HMI 模块状态映射 `map<string, ComponentStatus>`
- `components` — 受监控组件状态映射 `map<string, Component>`（每个 Component 包含上述多维度状态）
- `other_components` — 其他组件状态映射
- `global_components` — 全局组件状态映射
- `passenger_msg` — 乘客提示信息（安全模式时设置）
- `safety_mode_trigger_time` — 安全模式触发时间
- `require_emergency_stop` — 是否需要紧急停车
- `is_realtime_in_simulation` — 是否为仿真实时模式
- `detect_immediately` — 是否需要立即检测（HMI 有期望模块时设置）

## 健康检查机制

### 多层级检查策略

Monitor 模块采用分层检查架构：

1. **硬件层** — CAN 总线连通性、GPS 信号质量
2. **进程层** — 操作系统进程存活检测（`/proc` 文件系统）
3. **节点层** — Cyber RT 拓扑节点注册状态
4. **通道层** — 消息到达、延迟、频率、字段完整性
5. **资源层** — 磁盘/CPU/内存/IO 使用率
6. **汇总层** — 多维度状态合并
7. **安全层** — 功能安全决策与 EStop 触发

### 状态升级机制

`SummaryMonitor::EscalateStatus()` 确保状态只能向更严重的方向升级：

```cpp
// 优先级: FATAL > ERROR > WARN > OK > UNKNOWN
if (new_status > current_status->status()) {
    current_status->set_status(new_status);
    current_status->set_message(message);
}
```

这意味着一个组件的多个检测维度中，最严重的问题会成为该组件的最终状态。

### 变更检测与发布优化

`SummaryMonitor` 通过对 `SystemStatus` 序列化后计算哈希指纹来检测状态变化，避免在状态未变时频繁发布消息。同时设置最大发布间隔（默认 1 秒）作为兜底，确保下游消费者能定期收到心跳。

### 安全模式与紧急停车

功能安全检查仅在以下条件同时满足时生效：
- `FLAGS_enable_functional_safety` 为 `true`
- 车辆处于完全自动驾驶模式（`COMPLETE_AUTO_DRIVE`）
- 非仿真模式（`FLAGS_use_sim_time` 为 `false`）
- 非 SimControl 模式
- Chassis 消息时效性在 `FLAGS_system_status_lifetime_seconds` 内

触发 EStop 的完整时序：
1. 某个 `required_for_safety` 组件状态变为 `ERROR` 或 `FATAL`
2. `FunctionalSafetyMonitor` 检测到不安全，设置乘客提示并记录触发时间
3. 等待驾驶员介入（默认 10 秒）
4. 超时未介入 → 设置 `require_emergency_stop = true`
5. 下游 Guardian 模块读取该标志并执行紧急停车

## 源码目录结构

```
modules/monitor/
├── BUILD                                    # Bazel 构建文件
├── cyberfile.xml                            # 包描述文件
├── monitor.h                                # Monitor 组件头文件
├── monitor.cc                               # Monitor 组件实现
├── dag/
│   └── monitor.dag                          # DAG 配置
├── launch/
│   └── monitor.launch                       # Launch 配置
├── common/
│   ├── monitor_manager.h                    # MonitorManager 单例
│   ├── monitor_manager.cc
│   ├── recurrent_runner.h                   # RecurrentRunner 基类
│   ├── recurrent_runner.cc
│   └── recurrent_runner_test.cc             # 单元测试
├── hardware/
│   ├── esdcan_monitor.h / .cc               # ESD CAN 监控
│   ├── socket_can_monitor.h / .cc           # Socket CAN 监控
│   ├── gps_monitor.h / .cc                  # GPS 监控
│   └── resource_monitor.h / .cc             # 系统资源监控
└── software/
    ├── process_monitor.h / .cc              # 进程监控
    ├── module_monitor.h / .cc               # 模块节点监控
    ├── channel_monitor.h / .cc              # 通道消息监控
    ├── latency_monitor.h / .cc              # 延迟监控
    ├── localization_monitor.h / .cc         # 定位状态监控
    ├── camera_monitor.h / .cc               # 相机监控
    ├── recorder_monitor.h / .cc             # 录制器监控
    ├── summary_monitor.h / .cc              # 汇总监控
    └── functional_safety_monitor.h / .cc    # 功能安全监控
```

## 依赖关系

Monitor 模块的主要外部依赖：

- `cyber` — Cyber RT 框架（TimerComponent、Reader/Writer、NodeManager）
- `dreamview` — HMI 配置加载（`HMIUtil::LoadConfig()`、`HMIUtil::LoadMode()`）
- `common` — 适配器 GFlags、MonitorLogBuffer、工具函数
- `common_msgs` — protobuf 消息定义（SystemStatus、HMIStatus、Chassis、各传感器消息等）
- `drivers/canbus` — CAN 总线驱动（ESD CAN API）
- 第三方库：`gflags`、`absl`、`boost`（filesystem）、`protobuf`
