---
title: "Demo 车辆"
---

# Demo 车辆

> 源码路径：`modules/canbus_vehicle/demo/`

## 概述

Demo 模块是 Apollo 提供的车辆适配参考模板，继承 `AbstractVehicleFactory` 和 `VehicleController<Demo>` 接口，演示如何将控制指令转换为 CAN 协议报文。开发者可基于此模板快速实现新车型的 CAN 总线适配。模块包含完整的控制指令发送（制动、油门、转向、档位、驻车、车辆模式）和底盘状态报告接收（VCU、制动、转向、档位、驻车、超声波传感器、BMS、轮速、VIN）。

## 核心类

### DemoController

Demo 车辆控制器，实现 `VehicleController<Demo>` 的全部控制接口。

```cpp
class DemoController final : public VehicleController<apollo::canbus::Demo> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Demo>* can_sender,
                 MessageManager<Demo>* message_manager) override;
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
  void Brake(double acceleration) override;
  void Throttle(double throttle) override;
  void Acceleration(double acc) override;
  void Steer(double angle) override;
  void Steer(double angle, double angle_spd) override;
  void SetEpbBreak(const ControlCommand& command) override;
  void SetBeam(const VehicleSignal& signal) override;
  void SetHorn(const VehicleSignal& signal) override;
  void SetTurningSignal(const VehicleSignal& signal) override;

  void SecurityDogThreadFunc();
  bool CheckResponse(const int32_t flags, bool need_wait);

  Brakecommand101* brake_command_101_;
  Gearcommand103* gear_command_103_;
  Parkcommand104* park_command_104_;
  Steeringcommand102* steering_command_102_;
  Throttlecommand100* throttle_command_100_;
  Vehiclemodecommand105* vehicle_mode_command_105_;
};
```

**源码**：`modules/canbus_vehicle/demo/demo_controller.h`

### DemoVehicleFactory

Demo 车辆工厂，负责创建控制器和消息管理器，并管理 CAN 收发器生命周期。

```cpp
class DemoVehicleFactory : public AbstractVehicleFactory {
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

 private:
  std::unique_ptr<VehicleController<Demo>> CreateVehicleController();
  std::unique_ptr<MessageManager<Demo>> CreateMessageManager();
};

CYBER_REGISTER_VEHICLEFACTORY(DemoVehicleFactory)
```

**源码**：`modules/canbus_vehicle/demo/demo_vehicle_factory.h`

### DemoMessageManager

Demo CAN 报文管理器，注册所有发送和接收的 CAN 协议消息。

```cpp
class DemoMessageManager : public MessageManager<apollo::canbus::Demo> {
 public:
  DemoMessageManager();
  virtual ~DemoMessageManager();
};
```

构造函数中调用 `AddSendProtocolData` 注册 6 条发送协议，调用 `AddRecvProtocolData` 注册 16 条接收协议。

**源码**：`modules/canbus_vehicle/demo/demo_message_manager.h`

## 核心函数

### DemoController::Init

初始化控制器，从 `MessageManager` 获取各 CAN 协议数据指针（制动、档位、驻车、转向、油门、车辆模式），并调用 `AddSendMessage` 注册到 CAN 发送器。

### DemoController::chassis

从接收到的 CAN 报文中提取底盘状态，组装 `Chassis` protobuf 消息，包含车速（`vcu_report_505`）、油门踏板（`throttle_report_500`）、制动踏板（`brake_report_501`）、档位（`gear_report_503`，支持 N/R/D/P）、转向角（`steering_report_502`，按 `max_steer_angle` 换算为百分比）、EPS/VCU/ESP 在线状态（`check_response`）以及底盘错误和通信丢失告警。

### DemoController::EnableAutoMode

启用完全自动驾驶模式：依次使能制动、档位、转向、油门控制，调用 `CheckResponse` 等待确认响应。若失败则进入紧急模式。

### DemoController::SecurityDogThreadFunc

安全监控线程，以 50ms 周期运行，检查：

1. 横向控制响应（EPS），连续失败 10 次触发紧急模式
2. 纵向控制响应（VCU + ESP），连续失败 10 次触发紧急模式
3. 底盘错误，发现即触发紧急模式

### DemoController::CheckResponse

轮询检查底盘响应状态，支持重试机制（最多 20 次，间隔 20ms），验证 EPS/VCU/ESP 是否在线。

### DemoVehicleFactory::Init

工厂初始化流程：

1. 创建 CAN 客户端（通过 `CanClientFactory`）
2. 创建 `DemoMessageManager`
3. 初始化 CAN 接收器和发送器
4. 创建并初始化 `DemoController`
5. 创建 Cyber 节点，注册底盘详情 Topic 写入器

### DemoVehicleFactory::Start

按顺序启动：CAN 客户端 -> CAN 接收器 -> CAN 发送器 -> 车辆控制器。

## CAN 协议

### 发送消息（控制指令）

| CAN ID | 协议类 | 说明 |
|--------|--------|------|
| 0x100 | Throttlecommand100 | 油门控制 |
| 0x101 | Brakecommand101 | 制动控制 |
| 0x102 | Steeringcommand102 | 转向控制 |
| 0x103 | Gearcommand103 | 档位控制 |
| 0x104 | Parkcommand104 | 驻车控制 |
| 0x105 | Vehiclemodecommand105 | 车辆模式控制 |

### 接收消息（底盘报告）

| CAN ID | 协议类 | 说明 |
|--------|--------|------|
| 0x500 | Throttlereport500 | 油门状态 |
| 0x501 | Brakereport501 | 制动状态 |
| 0x502 | Steeringreport502 | 转向状态 |
| 0x503 | Gearreport503 | 档位状态 |
| 0x504 | Parkreport504 | 驻车状态 |
| 0x505 | Vcureport505 | VCU 整车状态（含车速） |
| 0x506 | Wheelspeedreport506 | 轮速报告 |
| 0x507-0x511 | Ultrsensor1-5 | 超声波传感器 |
| 0x512 | Bmsreport512 | 电池管理系统 |
| 0x514-0x516 | Vinresp1-3 | VIN 响应 |

协议定义在 `modules/canbus_vehicle/demo/protocol/` 目录。

## 配置

Demo 模块通过以下 protobuf 配置：

- `modules/canbus/proto/canbus_conf.pb.h` — CAN 总线配置（CAN 卡参数、收发日志开关）
- `modules/canbus/proto/vehicle_parameter.pb.h` — 车辆参数（最大转向角 `max_steer_angle`、驾驶模式 `driving_mode` 等）
- `modules/canbus_vehicle/demo/proto/demo.pb.h` — Demo 车型底盘详情消息定义

配置文件路径通常在 `modules/canbus/conf/` 下，由 `canbus_component` 加载并传递给 `DemoVehicleFactory::Init`。

## 调用关系

- **父类**：`AbstractVehicleFactory`（工厂基类）、`VehicleController<Demo>`（控制器基类）
- **依赖**：`drivers/canbus`（`CanClient`、`CanSender`、`CanReceiver`、`MessageManager`）、`protocol/`（CAN 协议解析）
- **被调用**：`CanbusComponent` 通过 `CYBER_REGISTER_VEHICLEFACTORY` 宏注册的插件机制加载
- **调用顺序**：`Init` -> `Start` -> `UpdateCommand`（循环）-> `Stop`
- **数据流**：`ControlCommand` -> `DemoController` -> CAN 协议报文 -> `CanSender` -> CAN 总线；CAN 总线 -> `CanReceiver` -> 协议解析 -> `DemoController::chassis` -> `Chassis` protobuf
