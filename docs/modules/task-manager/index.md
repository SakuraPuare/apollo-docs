---
title: Task Manager 任务管理模块
description: Apollo 自动驾驶平台 Task Manager 模块技术文档，涵盖循环路由、泊车路由、任务调度机制与配置方式。
---

# Task Manager 任务管理模块

## 模块职责

Task Manager 是 Apollo 自动驾驶平台中的任务调度管理模块，负责接收上层下发的驾驶任务，并将其分解为具体的导航命令发送给下游模块执行。

该模块的核心能力包括：

- **循环路由（Cycle Routing）**：支持车辆在指定起终点之间反复往返行驶，适用于固定线路巡航、测试验证等场景
- **泊车路由（Parking Routing）**：根据停车位 ID 从高精地图获取车位信息，构建包含泊车点的路由请求
- **任务生命周期管理**：监听定位信息和规划命令状态，判断车辆是否到达目标点，自动触发下一轮路由

模块作为 `cyber::Component<Task>` 运行，通过 `/apollo/task_manager` 通道接收任务消息，处理后通过 Cyber Service 调用 External Command 模块的 `LaneFollowCommand` 接口。

## 核心类与接口

### TaskManagerComponent（主组件）

- 文件：`task_manager_component.h` / `task_manager_component.cc`
- 继承自 `cyber::Component<task_manager::Task>`
- 职责：接收 Task 消息，根据任务类型分发给对应的 Manager 处理

```cpp
class TaskManagerComponent final : public cyber::Component<task_manager::Task> {
  bool Init() override;
  bool Proc(const std::shared_ptr<task_manager::Task>& task) override;
};
```

`Init()` 阶段创建以下 Reader 和 Client：

| 类型 | 通道/服务 | 用途 |
|---|---|---|
| Reader | `/apollo/localization/pose` | 实时获取车辆定位 |
| Reader | `/apollo/planning/command` | 监听规划命令状态 |
| Client | `/apollo/external_command/lane_follow` | 发送沿车道行驶命令 |

`Proc()` 阶段根据 `task.task_type()` 分发处理，当前支持 `CYCLE_ROUTING` 类型。

### CycleRoutingManager（循环路由管理器）

- 文件：`cycle_routing_manager.h` / `cycle_routing_manager.cc`
- 职责：管理循环路由任务的执行，在起终点之间反复发送导航命令

```cpp
class CycleRoutingManager {
  common::Status Init(const localization::Pose& pose,
                      const CycleRoutingTask& cycle_routing_task);
  bool GetNewRouting(const localization::Pose& pose,
                     external_command::LaneFollowCommand* lane_follow_command);
  int GetCycle() const;  // 获取剩余循环次数
};
```

工作流程：

1. `Init()`：记录起点（当前车辆位置）和终点，设置循环次数，保存原始 `LaneFollowCommand`
2. `GetNewRouting()`：每次调用时检查车辆是否到达起点或终点
   - 到达起点：发送原始路由命令（起点 → 终点）
   - 到达终点：构建反向路由命令（终点 → 起点），循环计数减一
3. 到达判断使用 `CheckPointDistanceInThreshold()` 函数，综合考虑距离和方向

### ParkingRoutingManager（泊车路由管理器）

- 文件：`parking_routing_manager.h` / `parking_routing_manager.cc`
- 职责：根据停车位信息构建泊车路由请求

```cpp
class ParkingRoutingManager {
  common::Status Init(const ParkingRoutingTask& parking_routing_task);
  bool ConstructParkingRoutingRequest(ParkingRoutingTask* parking_routing_task);
};
```

`ConstructParkingRoutingRequest()` 的处理逻辑：

1. 通过停车位 ID 从高精地图获取 `ParkingSpaceInfo`
2. 计算车位中心点坐标
3. 根据车位朝向与最近车道朝向的夹角判断车位类型：
   - 夹角 < 60°：**平行泊车**（`PARALLEL_PARKING`）
   - 夹角 >= 60°：**垂直泊车**（`VERTICAL_PLOT`）
4. 设置车位四角坐标和泊车点信息
5. 在路由终点后延伸 20m 的参考线，避免参考线生成失败

## 数据流

```
                          ┌──────────────────────┐
                          │   上层业务模块        │
                          │  (Dreamview 等)       │
                          └──────────┬───────────┘
                                     │ /apollo/task_manager
                                     │ Task (CYCLE_ROUTING)
                                     ▼
┌───────────────┐         ┌──────────────────────┐         ┌──────────────────────┐
│ Localization  │ Reader  │  TaskManagerComponent │ Client  │  External Command    │
│               ├────────►│                      ├────────►│                      │
│ /apollo/      │         │  CycleRoutingManager │         │  LaneFollowCommand   │
│ localization/ │         │  ParkingRoutingMgr   │         │  Service             │
│ pose          │         └──────────────────────┘         └──────────────────────┘
└───────────────┘                    ▲
                                     │ Reader
                          ┌──────────┴───────────┐
                          │  Planning 模块        │
                          │  /apollo/planning/    │
                          │  command              │
                          └──────────────────────┘
```

1. 上层业务模块（如 Dreamview）通过 `/apollo/task_manager` 通道发送 `Task` 消息
2. `TaskManagerComponent` 接收任务，创建 `CycleRoutingManager` 并初始化
3. 循环检测车辆定位，判断是否到达起点/终点
4. 到达后通过 Cyber Client 调用 `/apollo/external_command/lane_follow` 服务发送新的导航命令
5. 同时监听 `/apollo/planning/command` 确认路由是否被 Planning 模块接受

## 配置方式

### Protobuf 配置

配置文件路径：`modules/task_manager/conf/task_manager_config.pb.txt`

```protobuf
topic_config {
  planning_command_topic: "/apollo/planning/command"
  localization_pose_topic: "/apollo/localization/pose"
  lane_follow_command_topic: "/apollo/external_command/lane_follow"
  planning_topic: "/apollo/planning"
}
```

对应 proto 定义（`proto/task_manager_config.proto`）：

```protobuf
message TopicConfig {
  optional string lane_follow_command_topic = 1;
  optional string planning_command_topic = 2;
  optional string localization_pose_topic = 3;
  optional string planning_topic = 4;
}

message TaskManagerConfig {
  optional TopicConfig topic_config = 1;
}
```

### GFlags 参数

| 参数名 | 默认值 | 说明 |
|---|---|---|
| `task_manager_node_name` | `"task_manager"` | 节点名称 |
| `task_manager_threshold_for_destination_check` | `1.0`（conf 中覆盖为 `10.0`） | 判断车辆到达目标点的距离阈值（米） |
| `plot_size_buffer` | `0.2` | 停车位尺寸缓冲（米） |
| `road_width_buffer` | `0.0` | 道路宽度缓冲（米） |
| `search_junction_threshold` | `1.0` | 搜索路口的距离阈值（米） |

GFlags 配置文件路径：`modules/task_manager/conf/task_manager.conf`

```
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--task_manager_threshold_for_destination_check=10.0
--plot_size_buffer=0.2
--road_width_buffer=0.0
```

### DAG 配置

```
module_config {
    module_library : "modules/task_manager/libtask_manager_component.so"
    components {
        class_name : "TaskManagerComponent"
        config {
            name : "task_manager"
            config_file_path: "/apollo/modules/task_manager/conf/task_manager_config.pb.txt"
            flag_file_path: "/apollo/modules/task_manager/conf/task_manager.conf"
            readers: [
                {
                    channel: "/apollo/task_manager"
                    qos_profile: { depth : 15 }
                    pending_queue_size: 50
                }
            ]
        }
    }
}
```

### 启动方式

```bash
cyber_launch start modules/task_manager/launch/task_manager.launch
```

## 任务类型

当前 Task Manager 支持的任务类型：

| 任务类型 | 枚举值 | 说明 |
|---|---|---|
| `CYCLE_ROUTING` | - | 循环路由任务，车辆在起终点间反复行驶 |

任务通过 `task_manager::Task` protobuf 消息下发，其中 `task_type` 字段指定任务类型，`cycle_routing_task` 字段包含循环路由的具体参数（循环次数、`LaneFollowCommand` 等）。
