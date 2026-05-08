---
title: "交通灯跟踪"
---

# 交通灯跟踪模块 (Traffic Light Tracking)

> 源码路径: `modules/perception/traffic_light_tracking/`

## 概述

交通灯跟踪模块负责对摄像头检测到的交通灯进行语义修正与时间序列跟踪，输出稳定可靠的交通灯颜色状态。该模块接收来自上游检测组件的 `TrafficDetectMessage`，通过 `SemanticReviser` 算法对检测结果进行修正（解决闪烁、误检等问题），并融合 V2X 车路协同信息提升准确性，最终将结果以 `TrafficLightDetection` 消息发布到 Cyber RT 通道。

模块架构分为三层：

- **Cyber RT 组件层**：`TrafficLightTrackComponent` 负责消息收发、V2X 融合与结果投票
- **跟踪器接口层**：`BaseTrafficLightTracker` 定义跟踪器抽象接口
- **语义修正层**：`SemanticReviser` 实现基于语义分组和时间序列的灯光修正逻辑

## 核心类

### BaseTrafficLightTracker

交通灯跟踪器的抽象基类，定义在 `interface/base_traffic_light_tracker.h` 中。所有跟踪器插件需继承此类并通过 `REGISTER_TRAFFIC_LIGHT_TRACKER` 宏注册。

```cpp
class BaseTrafficLightTracker {
 public:
  virtual bool Init(const TrafficLightTrackerInitOptions& options =
                        TrafficLightTrackerInitOptions()) = 0;
  virtual bool Track(camera::TrafficLightFrame* frame) = 0;
};
```

### SemanticReviser

核心跟踪器实现，定义在 `tracker/semantic_decision.h` 中。通过语义分组将同一组灯归类，结合投票机制确定颜色，并利用时间序列防止短时间内颜色突变。关键数据结构：

```cpp
struct HystereticWindow {
  int hysteretic_count = 0;
  base::TLColor hysteretic_color = base::TLColor::TL_UNKNOWN_COLOR;
};

struct SemanticTable {
  double time_stamp = 0.0;
  double last_bright_time_stamp = 0.0;
  double last_dark_time_stamp = 0.0;
  bool blink = false;
  std::string semantic;
  std::vector<int> light_ids;
  base::TLColor color;
  HystereticWindow hystertic_window;
};
```

### TrafficLightTrackComponent

Cyber RT 组件，定义在 `traffic_light_tracking_component.h` 中。继承自 `cyber::Component<TrafficDetectMessage>`，负责整体流程编排。

## 核心函数

### SemanticReviser::Init

从配置文件加载语义修正参数，初始化修正时间窗、闪烁阈值和滞后阈值。

```cpp
bool SemanticReviser::Init(const TrafficLightTrackerInitOptions &options);
```

- 读取 `SemanticReviserConfig` protobuf 配置
- 计算 `non_blink_threshold_s_ = blink_threshold_s_ * 2`

### SemanticReviser::Track

主跟踪入口，按语义标签对灯光分组后逐一调用时间序列修正。

```cpp
bool SemanticReviser::Track(camera::TrafficLightFrame *frame);
```

- 为每盏灯生成语义键（`Semantic_N` 或 `No_semantic_light_ID`）
- 相同语义键的灯光归入同一 `SemanticTable`
- 对每个语义组调用 `ReviseByTimeSeries` 进行时序修正

### SemanticReviser::ReviseBySemantic

基于语义组内多盏灯的投票确定颜色。取票数最高的颜色，若存在平票则返回 `TL_UNKNOWN_COLOR`。

```cpp
base::TLColor SemanticReviser::ReviseBySemantic(
    SemanticTable semantic_table, std::vector<base::TrafficLightPtr> *lights);
```

### SemanticReviser::ReviseByTimeSeries

时间序列修正逻辑，防止检测结果在短时间内发生不合理跳变：

- 若当前帧与上一帧间隔小于 `revise_time_s_`（默认 1.5 秒），则根据历史颜色和当前颜色的组合进行修正
- 黄灯检测到前一状态为红灯时，保持红灯不变（避免红灯误判为黄灯）
- 黑灯（灭灯）通过滞后窗口机制确认，需连续超过 `hysteretic_threshold_` 次才更新颜色
- 绿灯闪烁检测：当亮灯间隔超过 `blink_threshold_s_` 时标记为闪烁

```cpp
void SemanticReviser::ReviseByTimeSeries(
    double time_stamp, SemanticTable semantic_table,
    std::vector<base::TrafficLightPtr> *lights);
```

### SemanticReviser::UpdateHistoryAndLights

更新历史记录并修正当前灯光状态。当灯光处于黑灯状态时，通过滞后机制避免颜色抖动。

```cpp
void SemanticReviser::UpdateHistoryAndLights(
    const SemanticTable &cur, std::vector<base::TrafficLightPtr> *lights,
    std::vector<SemanticTable>::iterator *history);
```

### SemanticReviser::ReviseLights

将指定 `light_ids` 对应的灯光颜色统一修正为目标颜色。

```cpp
void SemanticReviser::ReviseLights(std::vector<base::TrafficLightPtr> *lights,
                                   const std::vector<int> &light_ids,
                                   base::TLColor dst_color);
```

### TrafficLightTrackComponent::Init

组件初始化入口，依次完成配置加载、算法插件初始化和 V2X 监听器初始化。

### TrafficLightTrackComponent::InternalProc

核心处理流程：调用跟踪器修正灯光 -> 同步 V2X 数据 -> 投票输出最终颜色 -> 发布消息。

### TrafficLightTrackComponent::SyncV2XTrafficLights

将 V2X 车路协同系统接收到的交通灯信息与摄像头检测结果按信号 ID 进行同步。当时间戳差值在 `v2x_sync_interval_seconds_`（默认 0.1 秒）以内时，直接使用 V2X 的颜色覆盖摄像头检测结果。

### TrafficLightTrackComponent::TransformOutputMessage

对所有检测到的灯光进行置信度加权投票，选出置信度最高的颜色作为最终输出。投票规则：红 > 黄 > 绿，置信度为零的灯光视为未知。

## 配置

### 组件配置 (TrackingParam)

定义在 `proto/traffic_light_tracking_component.proto` 中，配置文件为 `conf/traffic_light_tracking_config.pb.txt`：

```protobuf
message TrackingParam {
  optional perception.PluginParam plugin_param = 1;
  optional string traffic_light_output_channel_name = 2;
  optional string v2x_trafficlights_input_channel_name = 3;
}
```

实际配置值：

| 字段 | 值 |
|------|-----|
| `plugin_param.name` | `SemanticReviser` |
| `plugin_param.config_path` | `perception/traffic_light_tracking/data` |
| `plugin_param.config_file` | `semantic.pb.txt` |
| `traffic_light_output_channel_name` | `/apollo/perception/traffic_light` |
| `v2x_trafficlights_input_channel_name` | `/apollo/v2x/traffic_light` |

### 语义修正器配置 (SemanticReviserConfig)

定义在 `tracker/proto/semantic.proto` 中，配置文件为 `data/semantic.pb.txt`：

```protobuf
message SemanticReviserConfig {
  optional float revise_time_second = 1 [default = 1.5];
  optional float blink_threshold_second = 2 [default = 0.4];
  optional int32 hysteretic_threshold_count = 3 [default = 2];
  optional string traffic_light_semantic_root_dir = 4;
}
```

实际配置值：

| 参数 | 值 | 说明 |
|------|-----|------|
| `revise_time_second` | 1.5 | 时序修正窗口（秒），在此时间内颜色不发生跳变 |
| `blink_threshold_second` | 0.55 | 闪烁判定阈值（秒），超过此间隔标记为闪烁 |
| `hysteretic_threshold_count` | 1 | 黑灯滞后计数阈值，连续检测到新颜色超过此次数才更新 |

## 调用关系

```text
TrafficLightTrackComponent::Init()
  ├── InitConfig()                    // 加载 TrackingParam 配置
  ├── InitAlgorithmPlugin()           // 实例化 SemanticReviser 插件
  │   └── SemanticReviser::Init()     // 加载语义修正参数
  └── InitV2XListener()               // 注册 V2X 消息回调

TrafficLightTrackComponent::Proc()
  └── InternalProc()
      ├── SemanticReviser::Track()     // 语义修正主流程
      │   ├── ReviseBySemantic()       // 组内投票
      │   ├── ReviseByTimeSeries()     // 时间序列修正
      │   │   ├── UpdateHistoryAndLights()  // 更新历史 + 滞后修正
      │   │   └── ReviseLights()       // 批量修正颜色
      │   └── ReviseLights()
      ├── SyncV2XTrafficLights()       // V2X 数据融合
      ├── TransformOutputMessage()     // 投票输出 + 构建 protobuf
      │   └── TransformDebugMessage()  // 调试信息填充
      └── writer_->Write()             // 发布结果消息

OnReceiveV2XMsg()                      // V2X 回调（独立线程）
  └── v2x_msg_buffer_.push_back()      // 写入环形缓冲区
```
