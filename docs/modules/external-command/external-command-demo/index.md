---
title: "外部命令示例"
---

# External Command Demo

> 源码路径：`modules/external_command/external_command_demo/`

## 概述

外部命令示例组件，演示如何通过 CyberRT Service Client 向各命令处理器发送外部命令。作为 `TimerComponent` 运行，按配置定时发送各类命令并查询执行状态。

## 核心类

### ExternalCommandDemo

```cpp
class ExternalCommandDemo final : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
 private:
  void SendActionCommand(ActionCommandType action_command_type);
  void SendSpeedCommand(double speed);
  void SendLaneFollowCommand(const std::vector<Pose>& way_points,
                             const Pose& end, double target_speed);
  void SendValetParkingCommand(const std::string& parking_spot_id,
                               double target_speed);
  void SendPathFollowCommandWithPathRecord(const std::string& record_path);
  void SendFreespaceCommand(const std::vector<Point>& way_points,
                            const Pose& end);
  void CheckCommandStatus(const uint64_t command_id);
};
```

## 支持的命令类型

| 命令 | Client 类型 | 说明 |
| ---- | ----------- | ---- |
| ActionCommand | action_command_client_ | 动作命令（启停等） |
| SpeedCommand | speed_command_client_ | 速度设置/恢复 |
| LaneFollowCommand | lane_follow_command_client_ | 车道跟随导航 |
| ValetParkingCommand | valet_parking_command_client_ | 代客泊车 |
| PathFollowCommand | path_follow_command_client_ | 路径跟随 |
| FreeSpaceCommand | free_space_command_client_ | 自由空间导航 |
| ChassisCommand | chassis_command_client_ | 底盘控制 |

## 调用关系

- **调用**：通过 CyberRT Service Client 调用各 `CommandProcessor` 服务
- **配置**：通过 `DemoConfig` protobuf 配置发送哪种命令及参数
