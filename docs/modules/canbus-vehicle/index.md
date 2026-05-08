---
title: "Canbus Vehicle 车辆底盘控制"
---

# Canbus Vehicle 车辆底盘控制

> 源码路径：`modules/canbus/`

## 概述

`canbus` 模块是 Apollo 的车辆底盘控制核心，负责将控制模块的指令（油门、刹车、转向）转换为 CAN 报文发送到车辆 ECU，同时从 ECU 读取底盘状态（车速、档位等）并发布 `Chassis` 消息。采用抽象工厂模式，通过 `AbstractVehicleFactory` 接口支持多种车型的即插即用。

## 架构

```text
ControlCommand / GuardianCommand
    │
    ▼
CanbusComponent (TimerComponent)
    │
    ├── AbstractVehicleFactory (车型插件)
    │     ├── VehicleController<T>   ← 控制指令 → CAN 报文
    │     ├── MessageManager<T>      ← CAN 报文解析
    │     ├── CanSender<T>           ← 发送 CAN 报文
    │     └── CanReceiver<T>         ← 接收 CAN 报文
    │
    └── Chassis 消息发布
```

## 核心类

### CanbusComponent

CyberRT 定时组件，驱动底盘通信循环。

```cpp
class CanbusComponent final : public cyber::TimerComponent {
 public:
  std::string Name() const;

 private:
  bool Init() override;
  bool Proc() override;    // 定时回调
  void Clear() override;

  void PublishChassis();
  void OnControlCommand(const control::ControlCommand& command);
  void OnGuardianCommand(const guardian::GuardianCommand& command);

  CanbusConf canbus_conf_;
  std::shared_ptr<AbstractVehicleFactory> vehicle_object_;
  std::shared_ptr<cyber::Reader<control::ControlCommand>> control_command_reader_;
  std::shared_ptr<cyber::Reader<guardian::GuardianCommand>> guardian_cmd_reader_;
  std::shared_ptr<cyber::Writer<Chassis>> chassis_writer_;
};

CYBER_REGISTER_COMPONENT(CanbusComponent)
```

**源码**：`modules/canbus/canbus_component.h`

### AbstractVehicleFactory

车辆抽象工厂，定义了车辆控制器的创建和管理接口。

```cpp
class AbstractVehicleFactory {
 public:
  void SetVehicleParameter(const VehicleParameter& vehicle_parameter);

  virtual bool Init(const CanbusConf* canbus_conf) = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;

  virtual void UpdateCommand(const ControlCommand* control_command) = 0;
  virtual void UpdateCommand(const ChassisCommand* chassis_command) = 0;
  virtual Chassis publish_chassis() = 0;
  virtual void PublishChassisDetail() = 0;

  virtual void AddSendProtocol();
  virtual void ClearSendProtocol();
  virtual bool IsSendProtocolClear();
  virtual Chassis::DrivingMode Driving_Mode();
};
```

**源码**：`modules/canbus/vehicle/abstract_vehicle_factory.h`

**插件注册宏**：`CYBER_REGISTER_VEHICLEFACTORY(name)`

### VehicleController\<SensorType\>

车辆控制器模板基类，定义了所有车型必须实现的控制接口。

```cpp
template <typename SensorType>
class VehicleController {
 public:
  virtual ErrorCode Init(const VehicleParameter& params,
                         CanSender<SensorType>* can_sender,
                         MessageManager<SensorType>* message_manager) = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual Chassis chassis() = 0;

  // 控制指令更新
  virtual ErrorCode Update(const control::ControlCommand& command);
  virtual ErrorCode Update(const external_command::ChassisCommand& command);

  // 驾驶模式切换
  virtual ErrorCode SetDrivingMode(const Chassis::DrivingMode& driving_mode);

 private:
  // 子类必须实现的纯虚函数
  virtual ErrorCode EnableAutoMode() = 0;
  virtual ErrorCode DisableAutoMode() = 0;
  virtual ErrorCode EnableSteeringOnlyMode() = 0;
  virtual ErrorCode EnableSpeedOnlyMode() = 0;
  virtual void Gear(Chassis::GearPosition state) = 0;
  virtual void Brake(double acceleration) = 0;
  virtual void Throttle(double throttle) = 0;
  virtual void Acceleration(double acc) = 0;
  virtual void Steer(double angle) = 0;
  virtual void Steer(double angle, double angle_spd) = 0;
  virtual void SetEpbBreak(const control::ControlCommand& command) = 0;
  virtual void SetBeam(const common::VehicleSignal& signal) = 0;
  virtual void SetHorn(const common::VehicleSignal& signal) = 0;
  virtual void SetTurningSignal(const common::VehicleSignal& signal) = 0;
  virtual void Emergency() = 0;
  virtual bool VerifyID() = 0;
};
```

**源码**：`modules/canbus/vehicle/vehicle_controller.h`

## 核心函数

### VehicleController::Update(ControlCommand)

处理控制指令的核心方法，根据驾驶模式分发控制操作。

```cpp
ErrorCode VehicleController<SensorType>::Update(
    const ControlCommand& control_command) {
  // 1. 处理 Pad 消息（驾驶模式切换请求）
  if (control_command.has_pad_msg()) {
    switch (action) {
      case START: SetDrivingMode(COMPLETE_AUTO_DRIVE); break;
      case RESET: SetDrivingMode(COMPLETE_MANUAL); break;
      case VIN_REQ: VerifyID(); break;
    }
  }

  // 2. 自动驾驶或速度模式下：控制油门/刹车/档位
  if (driving_mode == COMPLETE_AUTO_DRIVE || driving_mode == AUTO_SPEED_ONLY) {
    Gear(control_command.gear_location());
    Throttle(control_command.throttle());
    Acceleration(control_command.acceleration());
    Brake(control_command.brake());
  }

  // 3. 自动驾驶或转向模式下：控制转向
  if (driving_mode == COMPLETE_AUTO_DRIVE || driving_mode == AUTO_STEER_ONLY) {
    Steer(control_command.steering_target());
  }

  // 4. 处理车辆信号（灯光、喇叭）
  HandleVehicleSignal(control_command.signal());
}
```

### VehicleController::SetDrivingMode()

驾驶模式切换，包含安全检查：

```cpp
ErrorCode VehicleController<SensorType>::SetDrivingMode(
    const Chassis::DrivingMode& driving_mode) {
  // 不能直接设置 EMERGENCY_MODE
  // EMERGENCY_MODE 只响应 COMPLETE_MANUAL（重置）
  // 模式相同则跳过
  // 根据目标模式调用 EnableAutoMode/DisableAutoMode/...
}
```

### VehicleController::CheckChassisCommunicationError()

底盘通信故障检测，连续 100 次（约 1 秒）未收到底盘详情则判定通信故障。

## 支持车型

通过插件机制支持以下车型（每个车型一个独立子目录）：

| 车型 | 说明 |
|------|------|
| Lincoln | 林肯 MKZ |
| Ge3 | 广汽 GE3 |
| Lexus | 雷克萨斯 |
| Transit | 福特全顺 |
| Ch | 中华 |
| Wey | 长城 WEY |
| Devkit | 开发套件 |
| Gemini | GEM 电动车 |
| Neolix EDU | 新石器无人车 |
| Zhongyun | 中云 |
| Demo | 演示车型 |

## 配置

通过 `CanbusConf` protobuf 加载：

| 字段 | 类型 | 说明 |
|------|------|------|
| vehicle_parameter | VehicleParameter | 车型参数（品牌、型号等） |
| can_card_parameter | CANCardParameter | CAN 卡硬件参数 |

gflags：

| 标志位 | 说明 |
|--------|------|
| `FLAGS_chassis_debug_mode` | 底盘调试模式（跳过 Pad 消息时间检查） |
| `FLAGS_pad_msg_delay_interval` | Pad 消息延迟阈值（秒） |

## 调用关系

- **上游**：控制模块发布 `ControlCommand`；Guardian 发布 `GuardianCommand`
- **下游**：发布 `Chassis` 消息（车速、档位、转向等）
- **依赖**：`drivers/canbus`（CAN 通信）、`common/configs`（车辆配置）
