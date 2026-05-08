---
title: "GE3 车辆"
---

# GE3 车辆

> 源码路径: `modules/canbus_vehicle/ge3/`

## 概述

GE3 模块是 Apollo 自动驾驶平台中针对北汽 GE3 电动汽车的 CAN 总线驱动实现。该模块通过 CAN 协议与车辆底层控制器（制动系统 BCS、电动助力转向 EPS、整车控制器 VCU、车身控制器 BCM、电子驻车 EPB）进行双向通信，实现自动驾驶所需的纵向控制（油门/制动）、横向控制（转向）、挡位切换、灯光信号以及安全监控等功能。

模块采用工厂模式组织，核心由三部分组成：`Ge3VehicleFactory` 负责创建和管理整个 CAN 通信链路，`Ge3Controller` 实现具体的车辆控制逻辑，`Ge3MessageManager` 管理所有 CAN 协议报文的注册与分发。

## 核心类

### Ge3VehicleFactory

继承自 `AbstractVehicleFactory`，是 GE3 车辆的工厂入口。负责创建 CAN 客户端、消息管理器和车辆控制器，并编排初始化、启动、停止的完整生命周期。

```cpp
// 命名空间: apollo::canbus
class Ge3VehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf *canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const apollo::control::ControlCommand *control_command) override;
  void UpdateCommand(const apollo::external_command::ChassisCommand *chassis_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;
 private:
  std::unique_ptr<VehicleController<::apollo::canbus::Ge3>> CreateVehicleController();
  std::unique_ptr<MessageManager<::apollo::canbus::Ge3>> CreateMessageManager();
};
```

通过 `CYBER_REGISTER_VEHICLEFACTORY(Ge3VehicleFactory)` 宏注册到 CyberRT 框架中，运行时按配置动态加载。

### Ge3Controller

继承自 `VehicleController<Ge3>`，是车辆控制的核心类。负责将上层控制指令（油门、制动、转向、挡位、灯光、喇叭）转换为 CAN 协议帧，同时从车辆反馈报文中提取底盘状态信息。

```cpp
// 命名空间: apollo::canbus::ge3
class Ge3Controller final : public VehicleController<::apollo::canbus::Ge3> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Ge3>* can_sender,
                 MessageManager<Ge3>* message_manager) override;
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
  void Brake(double acceleration) override;
  void Throttle(double throttle) override;
  void Acceleration(double acc) override;
  void Steer(double angle) override;
  void Steer(double angle, double angle_spd) override;
  // ...
};
```

内部持有五个控制报文对象指针（`pc_bcm_201_`、`pc_bcs_202_`、`pc_epb_203_`、`pc_eps_204_`、`pc_vcu_205_`），以及一个独立的看门狗线程 `SecurityDogThreadFunc` 用于周期性安全检查。

### Ge3MessageManager

继承自 `MessageManager<Ge3>`，负责注册所有 GE3 车型的 CAN 协议数据帧。

```cpp
// 命名空间: apollo::canbus::ge3
class Ge3MessageManager : public MessageManager<::apollo::canbus::Ge3> {
 public:
  Ge3MessageManager();
  virtual ~Ge3MessageManager();
};
```

构造函数中通过 `AddSendProtocolData` 和 `AddRecvProtocolData` 注册全部发送帧和接收帧。

## 核心函数

### 车辆控制函数

| 函数 | 说明 | 参数范围 |
|------|------|----------|
| `Brake(double pedal)` | 设置制动踏板请求 | 0.00 ~ 99.99 |
| `Throttle(double pedal)` | 设置油门踏板请求 | 0.00 ~ 99.99 |
| `Steer(double angle)` | 设置转向角（固定角速度 500 deg/s） | -99.99 ~ 99.99，左正右负 |
| `Steer(double angle, double angle_spd)` | 设置转向角及角速度 | angle: -99.99 ~ 99.99, angle_spd: 0 ~ 99.99 |
| `Gear(Chassis::GearPosition)` | 切换挡位 | P/R/N/D |
| `SetEpbBreak(const ControlCommand&)` | 控制电子驻车 | APPLY / RELEASE |
| `SetBeam(const VehicleSignal&)` | 控制远近光灯 | high_beam / low_beam |
| `SetHorn(const VehicleSignal&)` | 控制喇叭 | on / off |
| `SetTurningSignal(const VehicleSignal&)` | 控制转向灯 | LEFT / RIGHT / NONE |

### 模式切换函数

| 函数 | 说明 |
|------|------|
| `EnableAutoMode()` | 切换到完全自动驾驶模式，启用全部控制子系统（制动、油门、挡位、驻车、转向），并通过 `CheckResponse` 等待 EPS/VCU/ESP 上线确认 |
| `DisableAutoMode()` | 切换到完全手动模式，重置所有发送报文 |
| `EnableSteeringOnlyMode()` | 仅启用转向控制，禁用纵向控制 |
| `EnableSpeedOnlyMode()` | 仅启用纵向控制（油门/制动/挡位/驻车），禁用转向 |
| `Emergency()` | 进入紧急模式，重置协议并开启双闪灯 |

### 安全监控函数

| 函数 | 说明 |
|------|------|
| `SecurityDogThreadFunc()` | 看门狗线程，以 50ms 周期检查横向和纵向控制响应，连续 10 次失败则触发紧急模式 |
| `CheckResponse(int32_t flags, bool need_wait)` | 检查 EPS（转向）、VCU（速度）、ESP（制动）的在线状态，最多重试 20 次，每次间隔 20ms |
| `CheckChassisError()` | 检查底盘各子系统故障状态：EPS 转向故障、VCU 故障、BCS 制动故障、VCU 挡位故障、EPB 驻车故障、SCU 整车故障 |
| `chassis()` | 从 MessageManager 获取传感器数据，组装完整的底盘状态，包括车速、轮速、油门、制动、挡位、转向角、驻车、灯光、VIN 等 |

## 配置

GE3 模块的配置通过 `CanbusConf` protobuf 消息传入，主要包含：

- `can_card_parameter`: CAN 卡硬件参数（设备类型、通道号等），用于创建 `CanClient`
- `vehicle_parameter`: 车辆参数，包含 `driving_mode`（驾驶模式）和 `max_steer_angle`（最大转向角）等
- `enable_receiver_log` / `enable_sender_log`: 是否启用 CAN 收发日志

GE3 的 CAN 协议报文定义在 `proto/ge3.proto` 中。

## 调用关系

```text
CanbusComponent (CyberRT 组件)
  └── Ge3VehicleFactory (工厂，通过 CYBER_REGISTER_VEHICLEFACTORY 注册)
        ├── Init()
        │     ├── CanClientFactory::CreateCANClient() → 创建 CAN 硬件客户端
        │     ├── CreateMessageManager() → Ge3MessageManager
        │     │     └── 构造时注册 5 个发送帧 + 11 个接收帧
        │     ├── CanReceiver::Init() → 绑定 CAN 客户端与消息管理器
        │     ├── CanSender::Init()
        │     └── CreateVehicleController() → Ge3Controller::Init()
        │           └── 从 MessageManager 获取 5 个发送协议对象指针
        │               (pc_bcm_201_, pc_bcs_202_, pc_epb_203_, pc_eps_204_, pc_vcu_205_)
        ├── Start()
        │     ├── CanClient::Start()
        │     ├── CanReceiver::Start()
        │     ├── CanSender::Start()
        │     └── Ge3Controller::Start()
        │           └── 启动 SecurityDogThreadFunc 看门狗线程
        ├── UpdateCommand(ControlCommand)
        │     ├── Ge3Controller::Update() → 执行具体控制逻辑
        │     │     ├── EnableAutoMode / DisableAutoMode
        │     │     ├── Brake / Throttle / Steer / Gear
        │     │     └── SetEpbBreak / SetBeam / SetHorn / SetTurningSignal
        │     └── CanSender::Update() → 将报文发送到 CAN 总线
        ├── publish_chassis() → Ge3Controller::chassis() → 组装底盘状态
        └── PublishChassisDetail() → 发布 GE3 原始底盘详情到 CyberRT 通道
```

### CAN 协议帧映射

**发送帧（平台 → 车辆）：**

| 帧 ID | 类名 | 功能 |
|--------|------|------|
| 0x201 | Pcbcm201 | 车身控制：灯光（远/近光、转向灯、双闪）、喇叭 |
| 0x202 | Pcbcs202 | 制动控制：制动踏板请求、制动使能 |
| 0x203 | Pcepb203 | 电子驻车：驻车请求/释放、驻车使能 |
| 0x204 | Pceps204 | 转向控制：转向角请求、转向速度、转向使能 |
| 0x205 | Pcvcu205 | 整车控制：油门踏板请求、挡位请求、油门使能、挡位使能 |

**接收帧（车辆 → 平台）：**

| 帧 ID | 类名 | 功能 |
|--------|------|------|
| 0x301 | Scu1301 | VIN 后 1 位、整车故障状态 |
| 0x302 | Scu2302 | VIN 前 8 位 |
| 0x303 | Scu3303 | VIN 中间 8 位 |
| 0x304 | Scubcm304 | 车身状态：远光灯、转向灯、喇叭状态 |
| 0x306 | Scubcs1306 | 制动状态：踏板行程、故障状态、驾驶模式 |
| 0x307 | Scubcs2307 | 车速信息 |
| 0x308 | Scubcs3308 | 四轮轮速及方向 |
| 0x310 | Scuepb310 | 驻车状态、驻车故障 |
| 0x311 | Scueps311 | 转向角、EPS 故障状态、驾驶模式 |
| 0x312 | Scuvcu1312 | 油门踏板、挡位、VCU 故障状态、驾驶模式 |
| 0x313 | Scuvcu2313 | VCU 扩展状态信息 |
