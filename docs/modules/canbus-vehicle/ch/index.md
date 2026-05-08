---
title: "CH 车辆"
---

# CH 车辆

> 源码路径: `modules/canbus_vehicle/ch/`

## 概述

CH 模块是 Apollo CAN 总线框架中的 CH 平台车辆适配层，通过 CAN 协议实现底盘控制与状态采集。模块遵循工厂模式组织：`ChVehicleFactory` 负责创建和管理 CAN 通信组件（CanClient、CanSender、CanReceiver）以及车辆控制器；`ChController` 承担具体的车辆操控逻辑（油门、制动、转向、档位等）；`ChMessageManager` 注册所有发送和接收的 CAN 报文协议。

CH 平台支持完全自动驾驶（`COMPLETE_AUTO_DRIVE`）和完全手动（`COMPLETE_MANUAL`）两种驾驶模式，不支持仅转向或仅速度模式。车辆控制器内部运行独立的安全看门狗线程，以 50ms 周期检测 EPS/VCU/ESP 响应状态，连续失败 10 次自动切入紧急模式。

## 核心类

### ChController

继承自 `VehicleController<apollo::canbus::Ch>`，是 CH 车辆的核心控制类。

```cpp
class ChController final : public VehicleController<::apollo::canbus::Ch> {
 public:
  ChController() {}
  virtual ~ChController();
  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::apollo::canbus::Ch>* const can_sender,
      MessageManager<::apollo::canbus::Ch>* const message_manager) override;
  bool Start() override;
  void Stop() override;
  Chassis chassis() override;
  // ...
};
```

关键成员变量：

| 成员 | 类型 | 说明 |
|---|---|---|
| `brake_command_111_` | `Brakecommand111*` | 制动命令报文（ID 0x111） |
| `throttle_command_110_` | `Throttlecommand110*` | 油门命令报文（ID 0x110） |
| `steer_command_112_` | `Steercommand112*` | 转向命令报文（ID 0x112） |
| `gear_command_114_` | `Gearcommand114*` | 档位命令报文（ID 0x114） |
| `turnsignal_command_113_` | `Turnsignalcommand113*` | 转向灯命令报文（ID 0x113） |
| `vehicle_mode_command_116_` | `Vehiclemodecommand116*` | 车辆模式命令报文（ID 0x116） |
| `thread_` | `unique_ptr<std::thread>` | 安全看门狗线程 |
| `chassis_error_mask_` | `int32_t` | 底盘错误掩码（互斥保护） |
| `chassis_error_code_` | `Chassis::ErrorCode` | 底盘错误码（互斥保护） |

### ChMessageManager

继承自 `MessageManager<apollo::canbus::Ch>`，在构造函数中注册所有 CH 平台的 CAN 报文。

```cpp
class ChMessageManager : public MessageManager<::apollo::canbus::Ch> {
 public:
  ChMessageManager();
  virtual ~ChMessageManager();
};
```

注册的报文分为两类：

- **发送报文（6 条）**：`Throttlecommand110`、`Brakecommand111`、`Steercommand112`、`Turnsignalcommand113`、`Gearcommand114`、`Vehiclemodecommand116`
- **接收报文（13 条）**：`Throttlestatus510`、`Brakestatus511`、`Steerstatus512`、`Turnsignalstatus513`、`Gearstatus514`、`Ecustatus1515` ~ `Ecustatus4518`、`Vinresp151b` ~ `Vinresp351d`、`Wheelspeedreport51e`

### ChVehicleFactory

继承自 `AbstractVehicleFactory`，通过 `CYBER_REGISTER_VEHICLEFACTORY` 宏注册到 CyberRT 工厂。负责组装完整的 CAN 通信链路。

```cpp
class ChVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf *canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const apollo::control::ControlCommand *control_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;
  void PublishChassisDetailSender() override;
  bool CheckChassisCommunicationFault() override;
};
```

## 核心函数

### ChController 初始化与生命周期

- **`Init()`** -- 校验参数后从 `MessageManager` 获取 6 个发送报文的可变指针，调用 `CanSender::AddMessage()` 注册。返回错误码。
- **`Start()`** -- 启动安全看门狗线程 `SecurityDogThreadFunc`。
- **`Stop()`** -- 等待看门狗线程退出。

### 车辆控制

- **`EnableAutoMode()`** -- 使能制动、油门、转向的电子控制，调用 `CheckResponse()` 等待 EPS 和 VCU 确认，成功则切换至 `COMPLETE_AUTO_DRIVE`。
- **`DisableAutoMode()`** -- 重置所有发送报文，切换至 `COMPLETE_MANUAL`。
- **`Brake(pedal)`** -- 设置制动踏板指令值（0 ~ 99.99），写入 `brake_command_111_`。
- **`Throttle(pedal)`** -- 设置油门踏板指令值（0 ~ 99.99），写入 `throttle_command_110_`。
- **`Steer(angle)`** -- 将百分比角度换算为实际角度（基于 `max_steer_angle`），写入 `steer_command_112_`。
- **`Steer(angle, angle_spd)`** -- 带角速度的转向重载。
- **`Gear(state)`** -- 映射 `GEAR_NEUTRAL/REVERSE/DRIVE/PARK` 到 CAN 档位枚举，写入 `gear_command_114_`。
- **`SetBeam()`** -- 控制近光灯开关。
- **`SetTurningSignal()`** -- 控制转向灯（左转、右转、双闪）。

### 底盘状态采集

- **`chassis()`** -- 核心状态聚合函数。从接收报文中提取速度（`ecu_status_1_515`）、油门踏板（`throttle_status__510`）、制动踏板（`brake_status__511`）、档位（`gear_status_514`）、转向角（`steer_status__512`）、电池 SOC（`ecu_status_2_516`）、VIN（`vin_resp1/2/3`）、前后保险杠事件、EPS/VCU/ESP 在线状态，组装为 `Chassis` protobuf 消息返回。当电池 SOC 低于 15% 或存在底盘错误时，设置 `DISALLOW_ENGAGE` 建议。

### 安全监控

- **`SecurityDogThreadFunc()`** -- 50ms 周期运行的安全线程。分别检查转向（EPS）和速度（VCU+ESP）的响应状态，连续 10 次失败则切入 `EMERGENCY_MODE` 并重置发送报文。同时检查底盘硬件故障（转向电机、驱动电机、制动系统）。
- **`CheckResponse(flags, need_wait)`** -- 通过 `chassis_.check_response()` 字段判断 EPS/VCU/ESP 是否在线。`need_wait=true` 时最多重试 20 次，每次间隔 20ms。
- **`CheckChassisError()`** -- 检查转向电机故障（`steer_err`）、驱动电机故障（`drive_motor_err`）、制动系统故障（`brake_err`）。

### 工厂方法

- **`ChVehicleFactory::Init()`** -- 按顺序创建 `CanClient`、`MessageManager`、`CanReceiver`、`CanSender`、`VehicleController`，并创建 CyberRT 节点发布 `chassis_detail` 和 `chassis_detail_sender` 话题。
- **`ChVehicleFactory::Start()`** -- 按顺序启动 CanClient -> CanReceiver -> CanSender -> VehicleController。
- **`ChVehicleFactory::UpdateCommand()`** -- 将 `ControlCommand` 或 `ChassisCommand` 传递给控制器并刷新发送缓冲区。

## 配置

CH 模块本身无独立配置文件，配置来源于上层 `CanbusConf` protobuf：

- `can_card_parameter` -- CAN 硬件卡参数，用于创建 `CanClient`
- `vehicle_parameter` -- 车辆参数（含 `driving_mode`、`max_steer_angle` 等），传递给控制器初始化
- `enable_receiver_log` / `enable_sender_log` -- CAN 收发日志开关

CAN 报文 ID 映射遵循 CH 平台协议规范：

| 方向 | 报文 ID | 报文名称 | 说明 |
|---|---|---|---|
| 发送 | 0x110 | Throttlecommand | 油门踏板命令 |
| 发送 | 0x111 | Brakecommand | 制动踏板命令 |
| 发送 | 0x112 | Steercommand | 转向角度命令 |
| 发送 | 0x113 | Turnsignalcommand | 转向灯/灯光命令 |
| 发送 | 0x114 | Gearcommand | 档位命令 |
| 发送 | 0x116 | Vehiclemodecommand | 车辆模式/VIN 请求命令 |
| 接收 | 0x510 | Throttlestatus | 油门踏板状态 |
| 接收 | 0x511 | Brakestatus | 制动踏板状态 |
| 接收 | 0x512 | Steerstatus | 转向角度状态 |
| 接收 | 0x513 | Turnsignalstatus | 转向灯状态 |
| 接收 | 0x514 | Gearstatus | 档位状态 |
| 接收 | 0x515~0x518 | Ecustatus1~4 | ECU 综合状态 |
| 接收 | 0x51B~0x51D | Vinresp1~3 | VIN 码响应 |
| 接收 | 0x51E | Wheelspeedreport | 轮速报告 |

## 调用关系

```text
CanbusComponent
  |
  +-- ChVehicleFactory::Init()
  |     +-- CanClientFactory::CreateCANClient()
  |     +-- ChMessageManager()              // 注册 6 发送 + 13 接收报文
  |     +-- CanReceiver::Init()
  |     +-- CanSender::Init()
  |     +-- ChController::Init()
  |           +-- 获取 6 个发送报文指针
  |           +-- CanSender::AddMessage() x6
  |
  +-- ChVehicleFactory::Start()
  |     +-- CanClient::Start()
  |     +-- CanReceiver::Start()
  |     +-- CanSender::Start()
  |     +-- ChController::Start()
  |           +-- SecurityDogThreadFunc()   // 50ms 安全监控线程
  |
  +-- UpdateCommand(ControlCommand)
  |     +-- ChController::Update()
  |           +-- Brake() / Throttle() / Steer() / Gear()
  |     +-- CanSender::Update()
  |
  +-- publish_chassis()
        +-- ChController::chassis()         // 聚合所有接收报文为 Chassis
```
