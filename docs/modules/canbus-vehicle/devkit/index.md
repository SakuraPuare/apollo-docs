---
title: "DevKit 车辆"
---

# DevKit 车辆

> 源码路径：`modules/canbus_vehicle/devkit/`

## 概述

DevKit 模块是 Apollo 为开发套件车型提供的 CAN 总线适配实现，继承 `AbstractVehicleFactory` 和 `VehicleController<Devkit>` 接口，将上层控制指令转换为 DevKit 特有的 CAN 协议报文。模块包含 6 组控制报文（节气门 0x100、制动 0x101、转向 0x102、档位 0x103、驻车 0x104、车辆模式 0x105）和 16 组反馈报文（涵盖底盘各子系统状态、超声波传感器、BMS 电池、VIN 等）。

## 核心类

### DevkitController

DevKit 车辆控制器，实现所有车辆控制接口，负责将控制指令编码为 CAN 报文并通过 `CanSender` 发送，同时组装 `Chassis` 反馈消息。

```cpp
class DevkitController final
    : public VehicleController<::apollo::canbus::Devkit> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Devkit>* can_sender,
                 MessageManager<Devkit>* message_manager) override;
  bool Start() override;
  void Stop() override;
  Chassis chassis() override;
  void AddSendMessage() override;

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
  void Speed(double speed);                        // m/s
  void Steer(double angle) override;               // -99.99~99.99 %
  void Steer(double angle, double angle_spd) override;
  void SetEpbBreak(const ControlCommand& command) override;
  void SetBeam(const VehicleSignal& signal) override;
  void SetHorn(const VehicleSignal& signal) override;
  void SetTurningSignal(const VehicleSignal& signal) override;
  bool VerifyID() override;

  void SecurityDogThreadFunc();  // 安全监控线程
  bool CheckResponse(const int32_t flags, bool need_wait);
  bool CheckChassisError();

  // CAN 协议控制消息
  Brakecommand101* brake_command_101_;
  Gearcommand103* gear_command_103_;
  Parkcommand104* park_command_104_;
  Steeringcommand102* steering_command_102_;
  Throttlecommand100* throttle_command_100_;
  Vehiclemodecommand105* vehicle_mode_command_105_;
};
```

**源码**：`modules/canbus_vehicle/devkit/devkit_controller.h`

### DevkitVehicleFactory

DevKit 车辆工厂，负责创建 CAN 客户端、消息管理器和控制器，管理 CAN 通信全生命周期。

```cpp
class DevkitVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const ControlCommand* control_command) override;
  void UpdateCommand(const ChassisCommand* chassis_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;
  void PublishChassisDetailSender() override;
  void UpdateHeartbeat() override;
  bool CheckChassisCommunicationFault() override;
  void AddSendProtocol() override;
  void ClearSendProtocol() override;
  bool IsSendProtocolClear() override;
  Chassis::DrivingMode Driving_Mode() override;

 private:
  std::unique_ptr<VehicleController<Devkit>> CreateVehicleController();
  std::unique_ptr<MessageManager<Devkit>> CreateMessageManager();
};

CYBER_REGISTER_VEHICLEFACTORY(DevkitVehicleFactory)
```

**源码**：`modules/canbus_vehicle/devkit/devkit_vehicle_factory.h`

### DevkitMessageManager

CAN 报文管理器，构造时注册全部 6 组发送协议和 16 组接收协议。

**源码**：`modules/canbus_vehicle/devkit/devkit_message_manager.h`

## 核心函数

### DevkitController::Init

初始化控制器：从 `MessageManager` 获取各协议对象指针（`dynamic_cast`），校验非空后调用 `AddSendMessage()` 注册到 `CanSender`。

### DevkitController::chassis

组装 `Chassis` 消息，从各反馈报文中提取速度、轮速、档位、转向角、制动/油门踏板、电池 SOC、超声波传感器距离、VIN 等字段，并设置 `engage_advice`。

### DevkitController::EnableAutoMode

使能全自动驾驶模式：对制动、油门、转向、档位、驻车五路使能控制位进行置位，可选使能 AEB，随后通过 `CheckResponse` 验证 EPS 和 VCU 在线状态。

### DevkitController::SecurityDogThreadFunc

安全监控线程，以 50ms 周期运行：

1. 横向控制校验：检测 EPS 是否在线，连续 10 次失败触发紧急模式
2. 纵向控制校验：检测 VCU/ESP 是否在线，连续 10 次失败触发紧急模式
3. 底盘故障检测：转向/驱动/制动硬件故障、电池低温/高温/低 SOC 均触发紧急制动
4. 紧急模式下执行 3 秒紧急制动（油门归零、制动 40%、转向归中）

### DevkitVehicleFactory::Init

工厂初始化流程：创建 CAN 客户端 -> 创建消息管理器 -> 初始化 CAN 接收器 -> 初始化 CAN 发送器 -> 创建并初始化车辆控制器 -> 创建 Cyber 节点和 chassis_detail 写入器。

## 配置

- `FLAGS_enable_aeb`：控制是否启用 AEB（自动紧急制动）功能
- `CanbusConf`：通过 protobuf 配置 CAN 卡参数、收发日志开关、车辆参数等
- 驾驶模式通过 `VehicleParameter.driving_mode` 预设

## 调用关系

- **父类**：`AbstractVehicleFactory`、`VehicleController<::apollo::canbus::Devkit>`
- **依赖**：`drivers/canbus`（CAN 通信）、`protocol/`（CAN 协议解析）、`cyber`（节点/Writer）
- **被调用**：`CanbusComponent` 通过 `CYBER_REGISTER_VEHICLEFACTORY` 插件机制加载
- **协议层**：每个 `protocol/*.h/.cc` 文件对应一条 CAN 报文，实现 `ProtocolData<Devkit>` 接口的 `Parse`/`UpdateData`/`Reset` 方法
