---
title: Storytelling 场景故事模块
description: Apollo 自动驾驶平台 Storytelling 模块技术文档，涵盖场景管理、Story 发布机制、数据流与配置方式。
---

# Storytelling 场景故事模块

## 模块职责

Storytelling 是 Apollo 自动驾驶平台中的全局性高级场景管理器，负责协调跨模块的行为决策。在城市道路自动驾驶中，车辆会遇到各种复杂的驾驶场景（如接近路口、信号灯、人行横道等），这些场景可能需要多个模块（规划、控制等）协同响应。

Storytelling 模块的核心思路是将复杂场景抽象为 **Story**（故事），每个 Story 描述一种特定的驾驶情境。模块根据预定义规则持续检测当前驾驶状态，创建或清除相应的 Story，并通过 `/apollo/storytelling` 通道发布。其他模块（如 Planning、Control）订阅该通道，根据当前活跃的 Story 调整自身行为。

该模块作为 `TimerComponent` 运行，以 **100ms** 的固定周期执行检测与发布逻辑。

## 核心类与接口

### Storytelling（主组件）

- 文件：`storytelling.h` / `storytelling.cc`
- 继承自 `apollo::cyber::TimerComponent`
- 职责：初始化所有 StoryTeller，周期性调用每个 Teller 的 `Update()` 方法，汇总 Stories 并发布

```cpp
class Storytelling final : public apollo::cyber::TimerComponent {
  bool Init() override;   // 初始化 FrameManager、StoryTeller 列表、Writer
  bool Proc() override;   // 每周期：StartFrame -> Update Tellers -> 发布 Stories -> EndFrame
};
```

### BaseTeller（故事讲述者基类）

- 文件：`story_tellers/base_teller.h`
- 定义了所有 StoryTeller 的统一接口

```cpp
class BaseTeller {
  virtual void Init(const StorytellingConfig& storytelling_conf) = 0;
  virtual void Update(Stories* stories) = 0;
};
```

### CloseToJunctionTeller（接近路口检测器）

- 文件：`story_tellers/close_to_junction_teller.h` / `close_to_junction_teller.cc`
- 继承自 `BaseTeller`
- 职责：沿规划轨迹检测车辆是否接近以下地图元素，并生成对应的 Story

| Story 类型 | 检测目标 | 说明 |
|---|---|---|
| `close_to_junction` | Junction（路口） | 车辆接近普通路口 |
| `close_to_pnc_junction` | PNC Junction | 车辆接近 PNC 路口 |
| `close_to_clear_area` | Clear Area（禁停区） | 车辆接近禁停区域 |
| `close_to_crosswalk` | Crosswalk（人行横道） | 车辆接近人行横道 |
| `close_to_signal` | Signal（信号灯） | 车辆接近信号灯 |
| `close_to_stop_sign` | Stop Sign（停车标志） | 车辆接近停车标志 |
| `close_to_yield_sign` | Yield Sign（让行标志） | 车辆接近让行标志 |

检测逻辑：遍历 Planning 输出的 ADC 轨迹点，在 `adc_trajectory_search_distance`（默认 10m）范围内，以 `search_radius`（默认 1m）为半径查询高精地图，判断是否存在上述元素。每个 Story 包含元素 ID 和距离信息。

### FrameManager（帧管理器）

- 文件：`frame_manager.h` / `frame_manager.cc`
- 职责：管理每个处理周期的生命周期，提供 Cyber Reader/Writer 的创建工具和监控日志缓冲

```cpp
class FrameManager {
  void StartFrame();   // 调用 node_->Observe() 刷新所有 Reader 数据
  void EndFrame();     // 发布监控日志
  // 模板方法：创建或获取 Reader / Writer
  template <class T> std::shared_ptr<cyber::Reader<T>> CreateOrGetReader(const std::string& channel);
  template <class T> std::shared_ptr<cyber::Writer<T>> CreateWriter(const std::string& channel);
};
```

## 数据流

```
┌─────────────────┐         ┌──────────────────┐         ┌──────────────────────┐
│  Planning 模块   │         │  Storytelling    │         │  下游模块             │
│                 │  topic   │                  │  topic   │  (Planning/Control)  │
│ ADCTrajectory   ├────────►│  CloseToJunction ├────────►│                      │
│                 │         │  Teller          │         │  订阅 Stories         │
│ /apollo/planning│         │                  │         │  /apollo/storytelling │
└─────────────────┘         │  + HD Map 查询    │         └──────────────────────┘
                            └──────────────────┘
```

1. Storytelling 订阅 `/apollo/planning` 通道，获取 Planning 模块输出的 `ADCTrajectory`（规划轨迹）
2. 每个 StoryTeller 根据轨迹点查询高精地图（HD Map），检测是否接近特定地图元素
3. 检测结果汇总到 `Stories` protobuf 消息中
4. 通过 `/apollo/storytelling` 通道发布，供下游模块订阅使用

## 配置方式

### Protobuf 配置

配置文件路径：`modules/storytelling/conf/storytelling_conf.pb.txt`

```protobuf
topic_config {
  planning_trajectory_topic: "/apollo/planning"
  storytelling_topic: "/apollo/storytelling"
}
```

对应 proto 定义（`proto/storytelling_config.proto`）：

```protobuf
message TopicConfig {
  optional string planning_trajectory_topic = 1;
  optional string storytelling_topic = 2;
}

message StorytellingConfig {
  optional TopicConfig topic_config = 1;
}
```

### GFlags 参数

| 参数名 | 默认值 | 说明 |
|---|---|---|
| `search_radius` | `1.0` | 在高精地图中搜索地图元素的半径（米） |
| `adc_trajectory_search_distance` | `10.0` | 沿规划轨迹向前搜索的距离（米） |

### 启动方式

使用 mainboard 启动：

```bash
mainboard -d modules/storytelling/dag/storytelling.dag
```

使用 cyber_launch 启动：

```bash
cyber_launch start modules/storytelling/launch/storytelling.launch
```

### DAG 配置

```
module_config {
    module_library : "modules/storytelling/libstorytelling_component.so"
    timer_components {
        class_name : "Storytelling"
        config {
            name: "storytelling"
            config_file_path: "/apollo/modules/storytelling/conf/storytelling_conf.pb.txt"
            flag_file_path: "/apollo/modules/common/data/global_flagfile.txt"
            interval: 100
        }
    }
}
```

`interval: 100` 表示组件每 100ms 执行一次 `Proc()` 方法。

## 扩展机制

Storytelling 模块采用可扩展的 Teller 架构。如需添加新的场景检测逻辑：

1. 继承 `BaseTeller` 基类
2. 实现 `Init()` 和 `Update()` 方法
3. 在 `Storytelling::Init()` 中注册新的 Teller 实例

当前已实现的 Teller：
- `CloseToJunctionTeller`：检测车辆接近路口、信号灯、人行横道等场景
