---
title: "Lincoln 林肯 MKZ 车型适配"
---

# Lincoln 林肯 MKZ 车型适配

> 源码路径：`modules/canbus_vehicle/lincoln/`

## 概述

Lincoln 模块是林肯 MKZ 车型的 CAN 总线适配实现，继承 `AbstractVehicleFactory` 和 `VehicleController<Lincoln>` 接口，将控制指令转换为林肯特有的 CAN 协议报文（制动 0x60、油门 0x62、转向 0x64、档位 0x66、转向灯 0x68）。

## 核心类

### LincolnController

林肯车辆控制器，实现所有控制接口。

```cpp
class LincolnController final : public VehicleController<apollo::canbus::Lincoln> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Lincoln>* can_sender,
                 MessageManager<Lincoln>* message_manager) override;
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
  void Brake(double acceleration) override;     // 0.00~99.99 %
  void Throttle(double throttle) override;       // 0.00~99.99 %
  void Acceleration(double acc) override;        // -7.0~5.0 m/s²
  void Steer(double angle) override;             // -99.99~99.99 %
  void Steer(double angle, double angle_spd) override;
  void SetEpbBreak(const ControlCommand& command) override;
  void SetBeam(const VehicleSignal& signal) override;
  void SetHorn(const VehicleSignal& signal) override;
  void SetTurningSignal(const VehicleSignal& signal) override;
  bool VerifyID() override;

  // CAN 协议消息
  Brake60* brake_60_;         // 0x60 制动
  Throttle62* throttle_62_;   // 0x62 油门
  Steering64* steering_64_;   // 0x64 转向
  Gear66* gear_66_;           // 0x66 档位
  Turnsignal68* turnsignal_68_; // 0x68 转向灯

  void SecurityDogThreadFunc();  // 安全监控线程
};
```

**源码**：`modules/canbus_vehicle/lincoln/lincoln_controller.h`

### LincolnVehicleFactory

林肯车辆工厂，创建控制器和消息管理器。

```cpp
class LincolnVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const ControlCommand* control_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;
};

CYBER_REGISTER_VEHICLEFACTORY(LincolnVehicleFactory)
```

**源码**：`modules/canbus_vehicle/lincoln/lincoln_vehicle_factory.h`

### LincolnMessageManager

林肯 CAN 报文管理器，维护 CAN ID 到协议解析器的映射。

**源码**：`modules/canbus_vehicle/lincoln/lincoln_message_manager.h`

## CAN 协议

| CAN ID | 协议类 | 说明 |
|--------|--------|------|
| 0x60 | Brake60 | 制动控制 |
| 0x62 | Throttle62 | 油门控制 |
| 0x64 | Steering64 | 转向控制 |
| 0x66 | Gear66 | 档位控制 |
| 0x68 | Turnsignal68 | 转向灯控制 |

协议定义在 `modules/canbus_vehicle/lincoln/protocol/` 目录。

## 安全机制

- **SecurityDogThreadFunc()**：安全监控线程，持续检查底盘状态
- **CheckChassisError()**：底盘错误检测
- **CheckSafetyError()**：安全相关错误检测（如超速、转向异常）
- **chassis_error_mask_**：错误掩码，按位标识各类故障

## 调用关系

- **父类**：`AbstractVehicleFactory`、`VehicleController<Lincoln>`
- **依赖**：`drivers/canbus`（CAN 通信）、`protocol/`（CAN 协议解析）
- **被调用**：`CanbusComponent` 通过插件机制加载
