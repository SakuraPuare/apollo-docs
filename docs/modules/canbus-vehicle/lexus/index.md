---
title: "Lexus 车辆"
---

# Lexus 车辆

> 源码路径：`modules/canbus_vehicle/lexus/`

## 概述

Lexus 模块是雷克萨斯车型的 CAN 总线适配实现，基于 Dataspeed/AS (AutonomouStuff) 线控底盘套件。继承 `AbstractVehicleFactory` 和 `VehicleController<Lexus>` 接口，通过 CAN 协议实现加速、制动、转向、换挡、转向灯等控制功能，以及车速、轮速、油门/制动百分比等底盘状态反馈。

CAN 协议地址范围 `0x100`~`0x134`（控制指令）、`0x200`~`0x418`（状态报告），定义在 `modules/canbus_vehicle/lexus/protocol/` 目录。

## 核心类

### LexusController

雷克萨斯车辆控制器，实现所有底盘控制接口。维护 12 个 CAN 协议消息指针，运行安全监控线程进行持续状态检查。

```cpp
class LexusController final : public VehicleController<::apollo::canbus::Lexus> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<::apollo::canbus::Lexus>* can_sender,
                 MessageManager<::apollo::canbus::Lexus>* message_manager) override;
  bool Start() override;
  void Stop() override;
  Chassis chassis() override;

 private:
  void Emergency() override;
  ErrorCode EnableAutoMode() override;
  ErrorCode DisableAutoMode() override;
  ErrorCode EnableSteeringOnlyMode() override;
  ErrorCode EnableSpeedOnlyMode() override;

  void Gear(Chassis::GearPosition state) override;
  void Brake(double acceleration) override;       // 0.00~99.99 %
  void Throttle(double throttle) override;         // 0.00~99.99 %
  void Acceleration(double acc) override;          // -7.0~5.0 m/s^2
  void Steer(double angle) override;               // -99.99~99.99 %
  void Steer(double angle, double angle_spd) override;
  void SetEpbBreak(const ControlCommand& command) override;
  ErrorCode HandleCustomOperation(const ChassisCommand& command) override;
  void SetBeam(const VehicleSignal& signal) override;
  void SetHorn(const VehicleSignal& signal) override;
  void SetTurningSignal(const VehicleSignal& signal) override;
  bool VerifyID() override;

  // CAN 协议消息
  Accelcmd100* accel_cmd_100_;                       // 0x100 加速指令
  Brakecmd104* brake_cmd_104_;                       // 0x104 制动指令
  Cruisecontrolbuttonscmd108* cruise_control_buttons_cmd_108_;  // 0x108 巡航控制
  Steeringcmd12c* steering_cmd_12c_;                 // 0x12C 转向指令
  Shiftcmd128* shift_cmd_128_;                       // 0x128 换挡指令
  Turncmd130* turn_cmd_130_;                         // 0x130 转向灯指令
  // ... 以及更多协议消息

  void SecurityDogThreadFunc();  // 安全监控线程（50ms 周期）
  bool CheckResponse(const int32_t flags, bool need_wait);
  void ResetProtocol();
};
```

**源码**：`modules/canbus_vehicle/lexus/lexus_controller.h`

### LexusVehicleFactory

雷克萨斯车辆工厂，创建控制器和消息管理器，管理 CAN 通信全生命周期。通过 `CYBER_REGISTER_VEHICLEFACTORY` 宏注册到 CyberRT 插件系统。

```cpp
class LexusVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const ControlCommand* control_command) override;
  void UpdateCommand(const ChassisCommand* chassis_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;

 private:
  std::unique_ptr<VehicleController<::apollo::canbus::Lexus>> CreateVehicleController();
  std::unique_ptr<MessageManager<::apollo::canbus::Lexus>> CreateMessageManager();
};
```

**源码**：`modules/canbus_vehicle/lexus/lexus_vehicle_factory.h`

### LexusMessageManager

雷克萨斯 CAN 报文管理器，维护 CAN ID 到协议解析器的映射。注册 12 个发送协议和 35 个接收协议。

**源码**：`modules/canbus_vehicle/lexus/lexus_message_manager.h`

## 核心函数

### LexusController::Init

初始化控制器，从 `MessageManager` 获取各协议消息对象的指针，将核心指令消息（加速、制动、换挡、转向、转向灯）注册到 `CanSender`。

```cpp
ErrorCode LexusController::Init(const VehicleParameter& params,
                                 CanSender<Lexus>* can_sender,
                                 MessageManager<Lexus>* message_manager);
```

### LexusController::chassis

从 `MessageManager` 获取传感器数据（`Lexus` protobuf），组装并返回 `Chassis` 消息。包含车速、四轮轮速、油门/制动百分比、档位、转向百分比、转向灯状态及驾驶模式。

### LexusController::EnableAutoMode

启用完全自动驾驶模式。依次使能加速、制动、转向、换挡四个协议的 `enable` 和 `clear_override` 标志，调用 `CheckResponse` 验证各 ECU 响应，失败则进入紧急模式。

### LexusController::SecurityDogThreadFunc

安全监控线程，以 50ms 周期运行。在自动驾驶模式下持续检查转向和速度 ECU 响应状态，连续 10 次失败（`kMaxFailAttempt`）则切换到紧急模式并重置协议消息。

```cpp
void LexusController::SecurityDogThreadFunc() {
  // 50ms 周期循环
  // 1. 检查横向控制（转向）：COMPLETE_AUTO_DRIVE 或 AUTO_STEER_ONLY
  // 2. 检查纵向控制（速度）：COMPLETE_AUTO_DRIVE 或 AUTO_SPEED_ONLY
  // 3. 连续失败 >= 10 次 -> EMERGENCY_MODE
}
```

### LexusVehicleFactory::Init

初始化完整的 CAN 通信链路：创建 CAN 客户端 -> 创建消息管理器 -> 初始化 CAN 接收器 -> 初始化 CAN 发送器 -> 创建并初始化车辆控制器 -> 创建 CyberRT 节点和底盘详细信息发布者。

## 配置

### CAN 协议

**控制指令**（发送）：

| CAN ID | 协议类 | 说明 |
|--------|--------|------|
| 0x100 | Accelcmd100 | 加速控制 |
| 0x104 | Brakecmd104 | 制动控制 |
| 0x108 | Cruisecontrolbuttonscmd108 | 巡航控制按钮 |
| 0x10C | Dashcontrolsleftcmd10c | 仪表左侧控制 |
| 0x110 | Dashcontrolsrightcmd110 | 仪表右侧控制 |
| 0x114 | Hazardlightscmd114 | 双闪灯控制 |
| 0x118 | Headlightcmd118 | 大灯控制 |
| 0x11C | Horncmd11c | 喇叭控制 |
| 0x120 | Mediacontrolscmd120 | 媒体控制 |
| 0x124 | Parkingbrakecmd124 | 驻车制动控制 |
| 0x128 | Shiftcmd128 | 换挡控制 |
| 0x12C | Steeringcmd12c | 转向控制 |
| 0x130 | Turncmd130 | 转向灯控制 |
| 0x134 | Wipercmd134 | 雨刷控制 |

**状态报告**（接收，部分）：

| CAN ID | 协议类 | 说明 |
|--------|--------|------|
| 0x10 | Globalrpt10 | 全局状态报告 |
| 0x20 | Componentrpt20 | 组件状态报告 |
| 0x200 | Accelrpt200 | 加速状态报告 |
| 0x204 | Brakerpt204 | 制动状态报告 |
| 0x228 | Shiftrpt228 | 换挡状态报告 |
| 0x22C | Steeringrpt22c | 转向状态报告 |
| 0x230 | Turnrpt230 | 转向灯状态报告 |
| 0x400 | Vehiclespeedrpt400 | 车速报告 |
| 0x407 | Wheelspeedrpt407 | 四轮轮速报告 |

完整协议定义见 `modules/canbus_vehicle/lexus/protocol/`。

### 底盘参数范围

- **制动**：`0.00`~`99.99`（pedal 单位）
- **油门**：`0.00`~`99.99`（pedal 单位）
- **转向角**：`-32.768`~`32.767` rad（CAN 协议，左负右正）
- **转向角速度**：`0`~`65.535` rad/s（CAN 协议）
- **加速**：`-7.0`~`5.0` m/s^2（当前 `Acceleration()` 未实现）
- **档位**：PARK / NEUTRAL / REVERSE / DRIVE / LOW / NONE

## 调用关系

- **父类**：`AbstractVehicleFactory`、`VehicleController<Lexus>`
- **依赖**：`drivers/canbus`（CAN 通信）、`protocol/`（CAN 协议解析）
- **被调用**：`CanbusComponent` 通过 CyberRT 插件机制加载 `LexusVehicleFactory`
- **启动顺序**：`Init()` -> `Start()`（CAN 客户端 -> CAN 接收器 -> CAN 发送器 -> 控制器）
