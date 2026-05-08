---
title: "Neolix Edu 车辆"
---

# Neolix Edu 车辆

> 源码路径: `modules/canbus_vehicle/neolix_edu/`

## 概述

Neolix Edu 是 Apollo CAN 总线车辆适配层中针对新石器无人配送车（Neolix Education 版）的驱动实现。该模块遵循 Apollo 的 `AbstractVehicleFactory` 工厂模式，由三大核心组件构成：车辆控制器（Controller）、消息管理器（MessageManager）和车辆工厂（VehicleFactory）。

模块通过 CAN 协议与车辆底层 ECU 通信，实现自动驾驶模式切换、转向/制动/油门控制、挡位管理、灯光喇叭控制以及底盘状态反馈等功能。

CAN 协议报文分为五组：

| 分组 | 方向 | CAN ID | 说明 |
|------|------|--------|------|
| `ads_*` (5条) | 发送 | 0x46, 0x50, 0x56, 0x310, 0x628 | 制动/驱动/转向/灯光喇叭/诊断指令 |
| `vcu_*` (7条) | 接收 | 0x47, 0x52, 0x57, 0x101, 0x201, 0x214, 0x502 | 制动/驱动/转向/状态/故障/电源/信息回报 |
| `aeb_*` (6条) | 接收 | 0x11, 0x353, 0x354, 0x355, 0x626, 0x718 | AEB 系统状态/轮速/脉冲/诊断回报 |
| `pas_*` (2条) | 接收 | 0x311, 0x312 | 泊车辅助超声波数据 |

## 核心类

### `Neolix_eduController`

继承自 `VehicleController<Neolix_edu>`，是车辆控制的核心实现。负责将上层控制指令转换为具体的 CAN 报文，同时从底盘回报中提取传感器数据组装 `Chassis` 消息。

```cpp
class Neolix_eduController final
    : public VehicleController<::apollo::canbus::Neolix_edu> {
  // 持有五条 ADS 发送协议的指针
  Adsbrakecommand46* ads_brake_command_46_;
  Adsdiagnosis628* ads_diagnosis_628_;
  Adsdrivecommand50* ads_drive_command_50_;
  Adsepscommand56* ads_eps_command_56_;
  Adslighthorncommand310* ads_light_horn_command_310_;
  // 安全监控线程
  std::unique_ptr<std::thread> thread_;
};
```

### `Neolix_eduMessageManager`

继承自 `MessageManager<Neolix_edu>`，在构造函数中注册全部发送（5条）和接收（16条）CAN 协议，是报文收发的调度中心。

```cpp
class Neolix_eduMessageManager
    : public MessageManager<::apollo::canbus::Neolix_edu> {
 public:
  Neolix_eduMessageManager();   // 注册所有协议
};
```

### `Neolix_eduVehicleFactory`

继承自 `AbstractVehicleFactory`，通过 `CYBER_REGISTER_VEHICLEFACTORY` 宏注册为可插拔工厂。负责创建 CAN Client/Receiver/Sender、Controller 和 MessageManager 实例，以及 Cyber RT 节点和 Writer 通道。

```cpp
class Neolix_eduVehicleFactory : public AbstractVehicleFactory {
  std::unique_ptr<CanClient> can_client_;
  CanSender<Neolix_edu> can_sender_;
  CanReceiver<Neolix_edu> can_receiver_;
  std::unique_ptr<MessageManager<Neolix_edu>> message_manager_;
  std::unique_ptr<VehicleController<Neolix_edu>> vehicle_controller_;
};
```

## 核心函数

### 车辆工厂生命周期

| 函数 | 说明 |
|------|------|
| `Neolix_eduVehicleFactory::Init(const CanbusConf*)` | 创建 CAN Client、MessageManager、CanReceiver、CanSender、VehicleController，初始化 Cyber RT 节点并创建 `chassis_detail` / `chassis_detail_sender` 两个 Writer |
| `Neolix_eduVehicleFactory::Start()` | 按序启动 CAN Client -> Receiver -> Sender -> Controller |
| `Neolix_eduVehicleFactory::Stop()` | 按序停止 Sender -> Receiver -> Client -> Controller |
| `Neolix_eduVehicleFactory::UpdateCommand(ControlCommand*)` | 将控制指令转发给 Controller 并触发 `can_sender_.Update()` |
| `Neolix_eduVehicleFactory::publish_chassis()` | 获取最新 Chassis 消息 |
| `Neolix_eduVehicleFactory::PublishChassisDetail()` | 将底盘详情发布到 `chassis_detail` 通道 |

### 控制器核心逻辑

| 函数 | 说明 |
|------|------|
| `Neolix_eduController::Init()` | 校验参数，从 MessageManager 获取五条发送协议指针，调用 `AddSendMessage()` 注册到 CAN Sender |
| `Neolix_eduController::Start()` | 启动安全监控线程 `SecurityDogThreadFunc` |
| `Neolix_eduController::chassis()` | 从接收报文中组装完整 Chassis 消息，包含车速（四轮平均）、轮速方向、SOC、转向角、油门/刹车百分比、挡位、EPB、碰撞检测、engage 建议等 |
| `Neolix_eduController::EnableAutoMode()` | 同时设置 brake/drive/eps 的 `drive_enable=true`，等待底层响应后切换为 `COMPLETE_AUTO_DRIVE` |
| `Neolix_eduController::DisableAutoMode()` | 重置所有发送协议，切换为 `COMPLETE_MANUAL` |
| `Neolix_eduController::Brake(double)` | 设置制动踏板值 (0~99.99) |
| `Neolix_eduController::Throttle(double)` | 设置油门踏板值 (0~99.99)，内部除以 2 写入协议 |
| `Neolix_eduController::Steer(double)` | 设置转向角，将百分比转换为实际角度写入 EPS 协议 |
| `Neolix_eduController::Gear(GearPosition)` | 通过 `ads_drive_command_50` 设置 N/R/D 挡，P 挡通过 `ads_brake_command_46` 的 parking 命令实现 |

### 安全监控

| 函数 | 说明 |
|------|------|
| `SecurityDogThreadFunc()` | 以 50ms 周期循环运行，检查横向控制（EPS 响应）和纵向控制（VCU/ESP 响应），连续 10 次失败则触发紧急模式 |
| `CheckResponse(int32_t flags, bool need_wait)` | 检查 EPS/VCU/ESP 的在线状态，`need_wait=true` 时最多重试 20 次（每次间隔 20ms） |
| `Emergency()` | 设置驾驶模式为 `EMERGENCY_MODE` 并重置所有发送协议 |

## 配置

- **CAN 总线配置**: 通过 `CanbusConf` protobuf 传入，包含 CAN 卡参数、收发日志开关等
- **车辆参数**: 通过 `VehicleParameter` protobuf 传入，关键参数包括 `driving_mode`（初始驾驶模式）和 `max_steer_angle`（最大转向角，用于百分比换算）
- **GFLAGS**:
  - `FLAGS_chassis_detail_topic` -- 底盘详情发布主题
  - `FLAGS_chassis_detail_sender_topic` -- 底盘发送详情发布主题

## 调用关系

```text
CanbusComponent
  └── Neolix_eduVehicleFactory (AbstractVehicleFactory)
        ├── Init()
        │     ├── CanClientFactory::CreateCANClient()
        │     ├── CreateMessageManager() → Neolix_eduMessageManager
        │     │     └── 注册 5 条发送协议 + 16 条接收协议
        │     ├── CanReceiver::Init()
        │     ├── CanSender::Init()
        │     └── CreateVehicleController() → Neolix_eduController
        │           └── Init() → 获取协议指针 + AddSendMessage()
        ├── Start()
        │     └── CanClient → CanReceiver → CanSender → Controller::Start()
        │                                                        └── SecurityDogThreadFunc (50ms 监控线程)
        ├── UpdateCommand(ControlCommand*)
        │     └── Neolix_eduController::Update()
        │           ├── EnableAutoMode() / DisableAutoMode()
        │           ├── Brake() / Throttle() / Steer() / Gear()
        │           └── SetEpbBreak() / SetBeam() / SetHorn()
        ├── publish_chassis()
        │     └── Neolix_eduController::chassis() (组装反馈数据)
        └── PublishChassisDetail()
              └── Writer::Write() → Cyber RT 通道
```
