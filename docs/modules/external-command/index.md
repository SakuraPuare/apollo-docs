---
title: External Command 外部命令模块
description: Apollo 自动驾驶平台 External Command 模块技术文档，涵盖外部命令接口、命令处理器插件架构、数据流与配置方式。
---

# External Command 外部命令模块

## 模块职责

External Command 是 Apollo 自动驾驶平台中的外部命令处理模块，作为用户业务层与自动驾驶内部模块之间的桥梁。它提供了一套标准化的外部命令接口，接收来自上层应用（如 Dreamview、远程调度系统等）的驾驶指令，经过预处理和转换后，分发给 Planning、Control、Canbus 等内部模块执行。

该模块的核心设计理念：

- **统一接口**：将各类驾驶指令（导航、泊车、变道、停车等）抽象为标准化的外部命令
- **插件化架构**：每种命令类型对应一个 CommandProcessor 插件，通过 Cyber PluginManager 动态加载
- **命令状态追踪**：提供统一的命令状态查询服务，实时反馈命令执行进度
- **向后兼容**：通过 OldRoutingAdapter 兼容旧版 RoutingRequest 接口

## 核心类与接口

### ExternalCommandProcessComponent（程序入口）

- 文件：`process_component/external_command_process_component.h` / `.cc`
- 继承自 `cyber::Component<>`
- 职责：根据配置加载命令处理器插件，提供命令状态查询服务

```cpp
class ExternalCommandProcessComponent : public cyber::Component<> {
  bool Init() override;
private:
  std::vector<std::shared_ptr<CommandProcessorBase>> command_processors_;
  std::shared_ptr<cyber::Service<CommandStatusRequest, CommandStatus>> command_status_service_;
};
```

`Init()` 阶段：
1. 读取配置文件中的 `processor` 列表
2. 通过 `PluginManager` 动态创建每个 CommandProcessor 实例
3. 创建 `/apollo/external_command/command_status` 服务，遍历所有 Processor 查询命令状态

### CommandProcessorBase（命令处理器基类）

- 文件：`command_processor/command_processor_base/command_processor_base.h` / `.cc`
- 职责：定义命令处理器的公共接口，管理配置加载和 Cyber Node

```cpp
class CommandProcessorBase {
  virtual bool Init(const std::shared_ptr<cyber::Node>& node);
  virtual bool GetCommandStatus(int64_t command_id, CommandStatus* status) const = 0;
protected:
  const CommandProcessorConfig& GetProcessorConfig() const;
};
```

### MotionCommandProcessorBase（运动命令处理器模板基类）

- 文件：`command_processor/command_processor_base/motion_command_processor_base.h`
- 模板类，为所有包含导航动作的命令提供通用处理流程

```cpp
template <typename T>
class MotionCommandProcessorBase : public CommandProcessorBase {
  bool Init(const std::shared_ptr<cyber::Node>& node) override;
  bool GetCommandStatus(int64_t command_id, CommandStatus* status) const override;
protected:
  virtual bool Convert(const std::shared_ptr<T>& command,
                       std::shared_ptr<routing::RoutingRequest>& routing_request) const = 0;
  virtual bool ProcessSpecialCommand(const std::shared_ptr<T>& command,
                                     const std::shared_ptr<planning::PlanningCommand>& planning_command) const = 0;
  void OnCommand(const std::shared_ptr<T>& command, std::shared_ptr<CommandStatus>& status);
};
```

`OnCommand()` 的通用处理流程：
1. 调用子类 `Convert()` 将外部命令转换为 `RoutingRequest`
2. 调用 Routing 模块搜索路由线路
3. 调用子类 `ProcessSpecialCommand()` 处理特殊命令参数
4. 将结果封装为 `PlanningCommand` 发送给 Planning 模块
5. 同时发布 `RoutingResponse` 供 HMI 显示

### 命令处理器插件

所有处理器通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册为插件：

| 处理器类 | 命令类型 | 服务名 | 说明 |
|---|---|---|---|
| `LaneFollowCommandProcessor` | `LaneFollowCommand` | `/apollo/external_command/lane_follow` | 沿车道线点对点行驶 |
| `ValetParkingCommandProcessor` | `ValetParkingCommand` | `/apollo/external_command/valet_parking` | 指定停车位泊车 |
| `ActionCommandProcessor` | `ActionCommand` | `/apollo/external_command/action` | 流程干预（启停、变道、切换模式等） |
| `FreeSpaceCommandProcessor` | `FreeSpaceCommand` | `/apollo/external_command/free_space` | 指定位姿停车（园区） |
| `PathFollowCommandProcessor` | `PathFollowCommand` | `/apollo/external_command/path_follow` | 指定线路行驶（园区） |
| `SpeedCommandProcessor` | `SpeedCommand` | `/apollo/external_command/speed` | 更改目标速度（园区），直接继承 `CommandProcessorBase` |
| `ChassisCommandProcessor` | `ChassisCommand` | `/apollo/external_command/chassis` | 自定义底盘命令（园区），直接继承 `CommandProcessorBase` |
| `PreciseParkingCommandProcessor` | `PreciseParkingCommand` | `/apollo/external_command/precise_parking` | 精确泊车 |
| `ZoneCoverCommandProcessor` | `ZoneCoverCommand` | `/apollo/external_command/zone_cover` | 区域覆盖行驶（园区） |

### ActionCommandProcessor（动作命令处理器）

- 文件：`command_processor/action_command_processor/action_command_processor.h` / `.cc`
- 直接继承 `CommandProcessorBase`（非运动命令，不需要路由搜索）
- 支持的动作类型：

| 动作 | 说明 | 转发目标 |
|---|---|---|
| `FOLLOW` | 跟车行驶 | Planning (`PadMessage`) |
| `CHANGE_LEFT` | 向左变道 | Planning (`PadMessage`) |
| `CHANGE_RIGHT` | 向右变道 | Planning (`PadMessage`) |
| `PULL_OVER` | 靠边停车 | Planning (`PadMessage`) |
| `STOP` | 停车 | Planning (`PadMessage`) |
| `START` | 恢复巡航 | Planning (`PadMessage`) |
| `CLEAR_PLANNING` | 清除规划 | Planning (`PadMessage`) |
| `SWITCH_TO_MANUAL` | 切换手动模式 | Control (`PadMessage`) |
| `SWITCH_TO_AUTO` | 切换自动模式 | Control (`PadMessage`) |
| `ENTER_MISSION` | 进入任务模式 | Planning (`PadMessage`) |
| `EXIT_MISSION` | 退出任务模式 | Planning (`PadMessage`) |
| `VIN_REQ` | VIN 请求 | Control (`PadMessage`) |

模式切换（`SWITCH_TO_AUTO` / `SWITCH_TO_MANUAL`）采用异步重试机制，最多尝试 3 次，每次间隔 500ms。切换到自动模式前会先确保底盘处于手动模式。

### OldRoutingAdapter（旧版路由适配器）

- 文件：`old_routing_adapter/old_routing_adapter.h` / `.cc`
- 继承自 `cyber::Component<>`
- 职责：兼容旧版 `RoutingRequest` 接口

工作流程：
1. 订阅 `/apollo/routing_request` 通道的旧版 `RoutingRequest`
2. 将其转换为 `LaneFollowCommand` 或 `ValetParkingCommand`（根据是否包含停车位信息）
3. 通过 Cyber Client 发送给 `ExternalCommandProcessComponent` 处理

### LaneWayTool（车道路点工具）

- 文件：`command_processor/command_processor_base/util/lane_way_tool.h`
- 职责：提供坐标到车道路点的转换工具

```cpp
class LaneWayTool {
  bool ConvertToLaneWayPoint(const Pose& pose, routing::LaneWaypoint* lane_way_point) const;
  bool GetVehicleLaneWayPoint(routing::LaneWaypoint* lane_way_point) const;
  bool GetParkingLaneWayPoint(const std::string& parking_id,
                              std::vector<routing::LaneWaypoint>* lane_way_points) const;
  bool IsParkandgoScenario() const;
};
```

## 数据流

```
┌──────────────────┐     Cyber Service      ┌──────────────────────────────┐
│  用户业务模块     │ ─────────────────────► │  ExternalCommandProcess      │
│  (Dreamview等)   │  LaneFollowCommand     │  Component                   │
│                  │  ValetParkingCommand    │                              │
│                  │  ActionCommand          │  ┌────────────────────────┐  │
│                  │  FreeSpaceCommand       │  │ LaneFollowCommand      │  │
│                  │  PathFollowCommand      │  │ Processor              │  │
│                  │  SpeedCommand           │  ├────────────────────────┤  │
│                  │  ChassisCommand         │  │ ValetParkingCommand    │  │
└──────────────────┘                        │  │ Processor              │  │
                                            │  ├────────────────────────┤  │
┌──────────────────┐     Cyber Service      │  │ ActionCommand          │  │
│  旧版模块        │ ─────────────────────► │  │ Processor              │  │
│  RoutingRequest  │  OldRoutingAdapter     │  └────────┬───────────────┘  │
└──────────────────┘                        └───────────┼──────────────────┘
                                                        │
                              ┌──────────────────────────┼──────────────────┐
                              │                          │                  │
                              ▼                          ▼                  ▼
                   ┌──────────────────┐    ┌──────────────────┐  ┌──────────────┐
                   │  Planning 模块    │    │  Control 模块     │  │  Canbus 模块  │
                   │  PlanningCommand  │    │  PadMessage      │  │  ChassisCmd  │
                   │  PadMessage       │    │                  │  │              │
                   └──────────────────┘    └──────────────────┘  └──────────────┘
```

### 运动命令处理流程（以 LaneFollowCommand 为例）

1. 用户通过 Cyber Client 发送 `LaneFollowCommand`（包含途经点和终点坐标）
2. `LaneFollowCommandProcessor` 接收命令
3. `Convert()`：将坐标转换为 `RoutingRequest`（通过 `LaneWayTool` 映射到车道路点）
4. 调用 Routing 模块搜索路由线路，得到 `RoutingResponse`
5. 封装为 `PlanningCommand` 发送给 Planning 模块
6. 发布 `RoutingResponse` 到 `/apollo/routing_response` 供 HMI 显示
7. 用户可通过 `/apollo/external_command/command_status` 服务查询命令执行状态

### 输出通道

| Channel 名 | 类型 | 说明 |
|---|---|---|
| `/apollo/planning/command` | `PlanningCommand` | 导航类外部命令转换后的内部命令 |
| `/apollo/routing_response` | `RoutingResponse` | 路由搜索结果，供 HMI 显示 |
| `/apollo/planning/pad` | `planning::PadMessage` | ActionCommand 转换后发送给 Planning |
| `/apollo/control/pad` | `control::PadMessage` | ActionCommand 转换后发送给 Control |
| `/apollo/canbus/chassis_control` | `ChassisCommand` | 底盘命令发送给 Canbus（具体通道名由配置文件定义） |

## 配置方式

### ProcessComponent 配置

配置文件路径：`modules/external_command/process_component/conf/config.pb.txt`

```protobuf
output_command_status_name: "/apollo/external_command/command_status"
processor: "apollo::external_command::LaneFollowCommandProcessor"
processor: "apollo::external_command::ValetParkingCommandProcessor"
processor: "apollo::external_command::ActionCommandProcessor"
```

`processor` 列表定义了要加载的命令处理器插件。如需支持更多命令类型，在此添加对应的处理器类名即可。

对应 proto 定义（`process_component/proto/process_component_config.proto`）：

```protobuf
message ProcessComponentConfig {
  required string output_command_status_name = 1
      [default = "/apollo/external_command/command_status"];
  repeated string processor = 2;
}
```

### CommandProcessor 配置

每个 CommandProcessor 插件有独立的配置文件（`conf/config.pb.txt`），定义输入输出通道：

```protobuf
message CommandProcessorConfig {
  required string input_command_name = 1;       // 接收命令的 Service 名
  repeated string output_command_name = 2;      // 发送内部命令的 Channel 名
  repeated string input_command_status_name = 3; // 订阅命令状态的 Channel 名
  optional string planning_command_history_name = 4
      [default = "/apollo/planning_command_history"];
}
```

### DAG 配置

```
module_config {
    module_library : "modules/external_command/process_component/libexternal_command_process_component.so"
    components {
      class_name : "ExternalCommandProcessComponent"
      config {
        name : "external_command_process"
        config_file_path : "/apollo/modules/external_command/process_component/conf/config.pb.txt"
        flag_file_path: "/apollo/modules/common/data/global_flagfile.txt"
      }
    }
}
```

### 启动方式

使用 mainboard 启动：

```bash
mainboard -d modules/external_command/process_component/dag/external_command_process.dag
```

使用 cyber_launch 启动：

```bash
cyber_launch start modules/external_command/process_component/launch/external_command_process.launch
```

如需兼容旧版 RoutingRequest，同时启动 OldRoutingAdapter：

```bash
cyber_launch start modules/external_command/old_routing_adapter/launch/old_routing_adapter.launch
```

## 插件扩展机制

External Command 模块采用 Cyber PluginManager 插件架构，扩展新的命令类型只需：

1. 定义新的命令 protobuf 消息（如 `MyCustomCommand`）
2. 创建处理器类，继承 `MotionCommandProcessorBase<MyCustomCommand>`（运动命令）或 `CommandProcessorBase`（非运动命令）
3. 实现 `Convert()` 和 `ProcessSpecialCommand()` 方法
4. 使用 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册插件
5. 在 `plugins.xml` 中声明插件信息
6. 在 `process_component/conf/config.pb.txt` 中添加处理器类名
