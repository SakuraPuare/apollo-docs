---
title: "WEY 车辆"
---

# WEY 车辆 CAN 总线控制模块

> 源码路径: `modules/canbus_vehicle/wey/`

## 概述

WEY 车辆模块是 Apollo CAN 总线框架中针对长城 WEY 品牌车型的适配实现。该模块通过 CAN 协议实现自动驾驶系统与 WEY 车辆底层线控系统的通信，涵盖转向、加速/制动、挡位切换、灯光、喇叭等控制功能，并实时采集底盘状态信息（车速、轮速、转向角、发动机转速等）上报给上层模块。

模块采用工厂模式组织，由 `WeyVehicleFactory` 统一创建和管理控制器与消息管理器，遵循 Apollo `AbstractVehicleFactory` 接口规范。

## 核心类

### WeyController

继承自 `VehicleController<apollo::canbus::Wey>`，是 WEY 车辆的控制核心。负责：

- 初始化 CAN 发送协议对象（5 个控制报文）
- 实现自动驾驶模式的进入与退出（全自动、仅转向、仅速度、紧急模式）
- 执行具体的车辆控制指令：转向、加速、挡位、灯光、喇叭等
- 运行安全看门狗线程，周期性检测底盘响应状态
- 解析底盘反馈报文并组装 `Chassis` 消息上报

```cpp
class WeyController final : public VehicleController<::apollo::canbus::Wey> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Wey>* can_sender,
                 MessageManager<Wey>* message_manager) override;
  bool Start() override;
  void Stop() override;
  Chassis chassis() override;
};
```

### WeyMessageManager

继承自 `MessageManager<apollo::canbus::Wey>`，在构造函数中注册所有 CAN 报文的收发协议：

- **发送报文（5 个）**：`Ads1_111`（纵向控制）、`Ads3_38e`（BCM 车身控制）、`Ads_eps_113`（EPS 转向控制）、`Ads_req_vin_390`（VIN 请求）、`Ads_shifter_115`（换挡控制）
- **接收报文（9 个）**：`Fail_241`（故障码）、`Fbs1_243`/`Fbs2_240`/`Fbs3_237`/`Fbs4_235`（底盘反馈）、`Status_310`（状态）、`Vin_resp1_391`/`Vin_resp2_392`/`Vin_resp3_393`（VIN 应答）

```cpp
class WeyMessageManager : public MessageManager<::apollo::canbus::Wey> {
 public:
  WeyMessageManager();
  virtual ~WeyMessageManager();
};
```

### WeyVehicleFactory

继承自 `AbstractVehicleFactory`，是整个 WEY 车辆模块的入口。通过 `CYBER_REGISTER_VEHICLEFACTORY` 宏注册到 CyberRT 框架，负责创建 `WeyController` 和 `WeyMessageManager`，并管理 CAN 收发器的生命周期。

```cpp
class WeyVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const ControlCommand* control_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;
};
```

## 核心函数

### WeyController::Init

初始化控制器，从 `MessageManager` 获取 5 个发送协议对象的指针，将其注册到 `CanSender`，并保存车辆参数配置。

### WeyController::EnableAutoMode

进入全自动驾驶模式。依次设置 ADS 模式为 Active、EPS 模式为 Active、换挡模式为 Valid、BCM 车身控制为 Active，然后调用 `CheckResponse` 验证转向和速度子系统均已在线响应。验证失败则触发 `Emergency`。

### WeyController::Acceleration

通过 `Ads1_111` 报文设置目标加速度（范围 -7.0 到 5.0 m/s^2）。正值同时发送起步请求，负值同时发送减速停车请求。

### WeyController::Steer

通过 `Ads_eps_113` 报文设置目标转向角。输入范围 -99.99 到 99.99（百分比），内部转换为实际角度：`real_angle = 500 * angle / 100`。

### WeyController::chassis

核心上报函数。从消息管理器获取传感器数据，解析各反馈报文组装 `Chassis` 消息，包括：车速、四轮轮速及方向、挡位、转向角、油门踏板、转向扭矩、EPB 状态、灯光状态、VIN 码，以及 `engage_advice` 和 `check_response` 信号。

### WeyController::SecurityDogThreadFunc

安全看门狗线程，以 50ms 周期运行。分别检测横向控制（EPS 在线状态）和纵向控制（VCU/ESP 在线状态）的响应，连续失败 10 次则触发紧急模式。同时检查底盘故障码（EPS 故障、发动机故障、ESP 故障、变速箱故障、EPB 故障）。

### WeyController::CheckResponse

验证底盘子系统的响应状态。通过位标志区分需要检查转向（`CHECK_RESPONSE_STEER_UNIT_FLAG`）或速度（`CHECK_RESPONSE_SPEED_UNIT_FLAG`）子系统。支持重试模式（最多 20 次，每次间隔 20ms）。

### WeyVehicleFactory::Init

工厂初始化流程：创建 CAN 客户端 -> 创建消息管理器 -> 初始化 CAN 接收器 -> 初始化 CAN 发送器 -> 创建并初始化车辆控制器 -> 创建 Cyber 节点和底盘详情 Writer。

### WeyVehicleFactory::Start

按顺序启动各组件：CAN 硬件 -> CAN 接收器 -> CAN 发送器 -> 车辆控制器。

## 配置

该模块通过以下配置文件和参数进行配置：

- **CanbusConf**：通过 `canbus_conf.pb.h` 定义，包含 CAN 卡参数、收发日志开关、车辆参数等
- **VehicleParameter**：通过 `vehicle_parameter.pb.h` 定义，必须设置 `driving_mode` 字段
- **max_steer_angle**：从 `VehicleConfig` 获取，用于转向百分比到实际角度的换算
- **底盘详情 Topic**：通过 `FLAGS_chassis_detail_topic` 配置，`WeyVehicleFactory` 使用此 Topic 发布原始底盘数据

关键常量：

| 常量 | 值 | 说明 |
|------|-----|------|
| `kMaxFailAttempt` | 10 | 安全看门狗连续失败阈值 |
| 看门狗周期 | 50ms | `SecurityDogThreadFunc` 循环周期 |
| CheckResponse 重试 | 20 次 x 20ms | 模式切换时的响应验证 |

## 调用关系

```text
CanbusComponent
  |
  v
WeyVehicleFactory::Init()
  |-- CanClientFactory::CreateCANClient()     // 创建 CAN 硬件客户端
  |-- CreateMessageManager() -> WeyMessageManager()
  |     |-- AddSendProtocolData (5 个发送报文)
  |     |-- AddRecvProtocolData (9 个接收报文)
  |-- CanReceiver::Init()
  |-- CanSender::Init()
  |-- CreateVehicleController() -> WeyController::Init()
  |     |-- 获取 5 个协议对象指针
  |     |-- CanSender::AddMessage() 注册发送报文
  |-- Cyber::CreateNode() + CreateWriter<Wey>()

WeyVehicleFactory::Start()
  |-- CanClient::Start()
  |-- CanReceiver::Start()
  |-- CanSender::Start()
  |-- WeyController::Start()
        |-- SecurityDogThreadFunc() [独立线程, 50ms 周期]
              |-- CheckResponse()    // 检查 EPS/VCU/ESP 在线
              |-- CheckChassisError() // 检查 Fail_241 故障报文
              |-- Emergency()        // 连续失败则紧急退出

ControlCommand 到达:
  WeyVehicleFactory::UpdateCommand()
    |-- WeyController::Update()
    |     |-- EnableAutoMode() / DisableAutoMode()
    |     |-- Acceleration() -> Ads1_111 设置加速度
    |     |-- Steer()        -> Ads_eps_113 设置转向角
    |     |-- Gear()         -> Ads_shifter_115 设置挡位
    |     |-- SetBeam()      -> Ads3_38e 设置灯光
    |     |-- SetHorn()      -> Ads3_38e 设置喇叭
    |-- CanSender::Update()  // 将控制报文发送到 CAN 总线

chassis() 上报:
  WeyVehicleFactory::publish_chassis()
    |-- WeyController::chassis()
          |-- MessageManager::GetSensorData()
          |-- 解析 Fbs1~Fbs4 / Status_310 / Vin_resp 报文
          |-- 组装 Chassis 消息返回
```
