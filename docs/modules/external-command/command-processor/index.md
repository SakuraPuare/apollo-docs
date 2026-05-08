---
title: "命令处理器"
---

# Command Processor 命令处理器

> 源码路径：`modules/external_command/command_processor/`

## 概述

Command Processor 是 Apollo 外部命令模块的核心处理层，负责将外部输入的各类驾驶命令转换为 Planning 模块可执行的 `PlanningCommand`。该模块采用插件化架构，通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册各处理器，基类定义统一的初始化和状态查询接口，派生类实现具体命令的转换逻辑。

## 核心类

### CommandProcessorBase

所有命令处理器的基类，提供初始化配置加载和 Cyber 节点管理：

```cpp
class CommandProcessorBase {
 public:
  CommandProcessorBase();
  virtual ~CommandProcessorBase() = default;
  virtual bool Init(const std::shared_ptr<cyber::Node>& node);
  virtual bool GetCommandStatus(int64_t command_id,
                                CommandStatus* status) const = 0;
 protected:
  const CommandProcessorConfig& GetProcessorConfig() const;
  const std::shared_ptr<cyber::Node>& Node() const;
};
```

### MotionCommandProcessorBase\<T\>

运动类命令处理器的模板基类，继承自 `CommandProcessorBase`，封装了路由请求转换、规划命令发送、命令状态跟踪等通用流程：

```cpp
template <typename T>
class MotionCommandProcessorBase : public CommandProcessorBase {
 public:
  bool Init(const std::shared_ptr<cyber::Node>& node) override;
  bool GetCommandStatus(int64_t command_id, CommandStatus* status) const override;
 protected:
  virtual bool Convert(const std::shared_ptr<T>& command,
                       std::shared_ptr<routing::RoutingRequest>& routing_request) const = 0;
  virtual bool ProcessSpecialCommand(
      const std::shared_ptr<T>& command,
      const std::shared_ptr<planning::PlanningCommand>& planning_command) const = 0;
  void OnCommand(const std::shared_ptr<T>& command,
                 std::shared_ptr<CommandStatus>& status);
  bool SetStartPose(std::shared_ptr<routing::RoutingRequest>& routing_request) const;
};
```

### 处理器一览

| 类名 | 基类 | 功能说明 |
|------|------|----------|
| `LaneFollowCommandProcessor` | `MotionCommandProcessorBase<LaneFollowCommand>` | 车道跟随命令，将途经点转换为路由请求 |
| `ValetParkingCommandProcessor` | `MotionCommandProcessorBase<ValetParkingCommand>` | 代客泊车命令 |
| `FreeSpaceCommandProcessor` | `MotionCommandProcessorBase<FreeSpaceCommand>` | 自由空间命令（开放区域行驶） |
| `PathFollowCommandProcessor` | `MotionCommandProcessorBase<PathFollowCommand>` | 路径跟随命令（按指定路径行驶） |
| `PreciseParkingCommandProcessor` | `MotionCommandProcessorBase<PreciseParkingCommand>` | 精确泊车命令 |
| `ZoneCoverCommandProcessor` | `MotionCommandProcessorBase<ZoneCoverCommand>` | 区域覆盖命令（清扫等场景） |
| `ActionCommandProcessor` | `CommandProcessorBase` | 动作命令（自动/手动模式切换、停车等） |
| `ChassisCommandProcessor` | `CommandProcessorBase` | 底盘命令（直接控制底盘） |
| `SpeedCommandProcessor` | `CommandProcessorBase` | 速度命令（设置目标速度） |

## 核心函数

### CommandProcessorBase::Init

从插件管理器获取配置文件路径，加载 `CommandProcessorConfig` protobuf 配置，并保存 Cyber 节点引用。

### MotionCommandProcessorBase::OnCommand

运动命令的统一处理入口：

1. 调用 `Convert()` 将命令转换为 `RoutingRequest`
2. 若存在路由请求，通过 `SetStartPose()` 设置起点为当前车辆位置
3. 调用 Routing 模块获取路由结果，写入 `PlanningCommand`
4. 调用 `ProcessSpecialCommand()` 处理命令中的特殊字段
5. 设置 `command_id` 和 `target_speed`，发布 `PlanningCommand`

### ActionCommandProcessor::OnCommand

处理动作类命令，支持自动/手动模式切换（`SwitchToAutoMode` / `SwitchToManualMode`），通过向 Control 模块发送 `DrivingAction` 并轮询 Chassis 状态确认切换结果。

## 调用关系

```text
ExternalCommandComponent
  └── CommandProcessorBase (插件加载)
        ├── MotionCommandProcessorBase<T>
        │     ├── Convert() → RoutingRequest
        │     ├── Routing 模块 → RoutingResponse
        │     ├── ProcessSpecialCommand()
        │     └── PlanningCommand → Planning 模块
        ├── ActionCommandProcessor
        │     ├── PadMessage → Planning 模块
        │     └── DrivingAction → Control 模块
        ├── ChassisCommandProcessor
        │     └── ChassisCommand → Canbus 模块
        └── SpeedCommandProcessor
              └── PlanningCommand (speed) → Planning 模块
```
