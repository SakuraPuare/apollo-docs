---
title: "GEM 车辆"
---

# GEM 车辆适配模块

> 源码路径：`modules/canbus_vehicle/gem/`

## 概述

GEM 模块是 Polaris GEM 社区电动车的 CAN 总线适配实现，基于 PACMod（Pacifica Modular）自动驾驶接口协议。该模块继承 `AbstractVehicleFactory` 和 `VehicleController<Gem>` 接口，通过 PACMod 全局使能命令切换自动/手动驾驶模式，并将上层控制指令（制动、油门、转向、档位、转向灯）转换为 GEM 特有的 CAN 协议报文。

核心依赖 PACMod 系统的 `global_cmd_69` 进行自动驾驶使能与超控清除，通过 `global_rpt_6a` 反馈 PACMod 状态来判断当前驾驶模式。

## 核心类

### GemController

GEM 车辆控制器，实现所有车辆控制接口，管理 6 条 CAN 控制报文的发送。

```cpp
class GemController final : public VehicleController<::apollo::canbus::Gem> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Gem>* can_sender,
                 MessageManager<Gem>* message_manager) override;
  bool Start() override;
  void Stop() override;
  Chassis chassis() override;

 private:
  void Emergency() override;
  ErrorCode EnableAutoMode() override;
  ErrorCode DisableAutoMode() override;
  ErrorCode EnableSteeringOnlyMode() override;  // 未实现
  ErrorCode EnableSpeedOnlyMode() override;      // 未实现

  void Gear(Chassis::GearPosition state) override;
  void Brake(double acceleration) override;      // 0.00~99.99 %
  void Throttle(double throttle) override;        // 0.00~99.99 %
  void Acceleration(double acc) override;         // -7.0~5.0 m/s²
  void Steer(double angle) override;              // -99.99~99.99 %
  void Steer(double angle, double angle_spd) override;
  void SetEpbBreak(const ControlCommand& command) override;
  void SetBeam(const VehicleSignal& signal) override;
  void SetHorn(const VehicleSignal& signal) override;
  void SetTurningSignal(const VehicleSignal& signal) override;
  bool VerifyID() override;

  // CAN 控制协议指针
  Accelcmd67* accel_cmd_67_;      // 0x67 油门
  Brakecmd6b* brake_cmd_6b_;      // 0x6B 制动
  Shiftcmd65* shift_cmd_65_;      // 0x65 档位
  Steeringcmd6d* steering_cmd_6d_; // 0x6D 转向
  Turncmd63* turn_cmd_63_;        // 0x63 转向灯
  Globalcmd69* global_cmd_69_;    // 0x69 PACMod 全局控制

  void SecurityDogThreadFunc();   // 安全监控线程（50ms 周期）
};
```

**源码**：`modules/canbus_vehicle/gem/gem_controller.h`、`gem_controller.cc`

### GemVehicleFactory

GEM 车辆工厂，负责创建 CAN 客户端、收发器、消息管理器和车辆控制器，并管理整个生命周期。

```cpp
class GemVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const ControlCommand* control_command) override;
  void UpdateCommand(const ChassisCommand* chassis_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;

 private:
  std::unique_ptr<VehicleController<Gem>> CreateVehicleController();
  std::unique_ptr<MessageManager<Gem>> CreateMessageManager();
};

CYBER_REGISTER_VEHICLEFACTORY(GemVehicleFactory)
```

**源码**：`modules/canbus_vehicle/gem/gem_vehicle_factory.h`、`gem_vehicle_factory.cc`

### GemMessageManager

GEM CAN 报文管理器，在构造函数中注册所有发送（9 条）和接收（21 条）协议数据。

**源码**：`modules/canbus_vehicle/gem/gem_message_manager.h`、`gem_message_manager.cc`

## 核心函数

### EnableAutoMode

通过 PACMod 全局命令启用自动驾驶。设置 `pacmod_enable = CONTROL_ENABLED` 并清除超控，发送后等待转向和速度单元响应，失败则进入紧急模式。

### SecurityDogThreadFunc

安全监控线程，以 50ms 周期运行。分别检查横向控制（转向）和纵向控制（速度）的 CAN 响应，连续失败达到 `kMaxFailAttempt`（10 次）时触发紧急停车，将驾驶模式切换为 `EMERGENCY_MODE` 并重置所有发送报文。

### chassis

从 `GemMessageManager` 获取传感器数据，填充 `Chassis` protobuf。解析关键信号：

- 车速：`vehicle_speed_rpt_6f`
- 油门踏板：`accel_rpt_68`
- 制动踏板：`brake_rpt_6c`
- 档位：`shift_rpt_66`（NEUTRAL / REVERSE / FORWARD）
- 转向角：`steering_rpt_1_6e`（需除以 `max_steer_angle` 转为百分比）
- PACMod 状态：`global_rpt_6a`（`CONTROL_ENABLED` 时设为 `COMPLETE_AUTO_DRIVE`）
- 转向灯：`light` 消息

根据底盘状态生成 `engage_advice`：无错误且制动踏板踩下时建议 `READY_TO_ENGAGE`，否则 `DISALLOW_ENGAGE`。

### Init（GemController）

从 `MessageManager` 获取 6 条控制协议的可变指针，通过 `CanSender::AddMessage` 注册为周期发送消息。

### Init（GemVehicleFactory）

初始化链路：`CanClientFactory` -> `CanClient` -> `MessageManager` -> `CanReceiver` -> `CanSender` -> `VehicleController`，并创建 Cyber 节点发布 `chassis_detail` 主题。

## CAN 协议

### 控制报文（发送）

| CAN ID | 协议类 | 说明 |
|--------|--------|------|
| 0x63 | Turncmd63 | 转向灯控制 |
| 0x65 | Shiftcmd65 | 档位控制（NEUTRAL/REVERSE/FORWARD） |
| 0x67 | Accelcmd67 | 油门控制（0.0~1.0） |
| 0x69 | Globalcmd69 | PACMod 全局控制（使能/超控清除） |
| 0x6B | Brakecmd6b | 制动控制（0.0~1.0） |
| 0x6D | Steeringcmd6d | 转向控制（位置+速度限制） |
| 0x76 | Headlightcmd76 | 大灯控制 |
| 0x78 | Horncmd78 | 喇叭控制 |
| 0x90 | Wipercmd90 | 雨刮控制 |

### 报告报文（接收）

| CAN ID | 协议类 | 说明 |
|--------|--------|------|
| 0x64 | Turnrpt64 | 转向灯报告 |
| 0x66 | Shiftrpt66 | 档位报告 |
| 0x68 | Accelrpt68 | 油门报告 |
| 0x6A | Globalrpt6a | PACMod 全局状态报告 |
| 0x6C | Brakerpt6c | 制动报告 |
| 0x6E | Steeringrpt16e | 转向报告 |
| 0x6F | Vehiclespeedrpt6f | 车速报告 |
| 0x70-0x72 | Brakemotorrpt1-3 | 制动电机报告 |
| 0x73-0x75 | Steeringmotorrpt1-3 | 转向电机报告 |
| 0x77 | Headlightrpt77 | 大灯报告 |
| 0x79 | Hornrpt79 | 喇叭报告 |
| 0x7A | Wheelspeedrpt7a | 轮速报告 |
| 0x80 | Parkingbrakestatusrpt80 | 驻车制动状态报告 |
| 0x81 | Yawraterpt81 | 横摆角速度报告 |
| 0x82 | Latlonheadingrpt82 | 经纬度航向报告 |
| 0x83 | Datetimerpt83 | 日期时间报告 |
| 0x91 | Wiperrpt91 | 雨刮报告 |

协议定义在 `modules/canbus_vehicle/gem/protocol/` 目录。

## 配置

- **车辆参数**：通过 `VehicleParameter` protobuf 配置，需设置 `driving_mode`、`max_steer_angle`、`min/max_steer_angle_rate`
- **CAN 总线配置**：通过 `CanbusConf` protobuf 配置 CAN 卡参数、收发日志开关
- **ChassisDetail 主题**：通过 `FLAGS_chassis_detail_topic`（`adapters/adapter_gflags.h`）配置发布主题名
- **PACMod 使能**：`global_cmd_69` 的 `pacmod_enable` 和 `clear_override` 字段控制自动驾驶切换

## 调用关系

- **父类**：`AbstractVehicleFactory`（GemVehicleFactory）、`VehicleController<Gem>`（GemController）
- **依赖**：`drivers/canbus`（CanClient、CanSender、CanReceiver）、`protocol/`（CAN 协议解析）
- **被调用**：`CanbusComponent` 通过 `CYBER_REGISTER_VEHICLEFACTORY` 宏注册的插件机制加载 `GemVehicleFactory`
- **调用流程**：`CanbusComponent` -> `GemVehicleFactory::Init/Start` -> `GemController::Init/Start` -> `SecurityDogThreadFunc`（后台安全监控）
