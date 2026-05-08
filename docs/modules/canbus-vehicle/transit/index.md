---
title: "Transit 车辆"
---

# Transit 车辆

> 源码路径：`modules/canbus_vehicle/transit/`

## 概述

Transit 模块是 Udelv Transit 自动驾驶车辆的 CAN 总线适配实现，继承 `AbstractVehicleFactory` 和 `VehicleController<Transit>` 接口。模块将上层控制指令转换为 Transit 特有的 LLC（Low-Level Controller）CAN 协议报文，通过 ADC 前缀报文发送控制命令，接收 LLC 前缀报文获取底盘反馈状态。

## 核心类

### TransitController

Transit 车辆控制器，负责所有车辆控制操作。

```cpp
class TransitController final
    : public VehicleController<::apollo::canbus::Transit> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Transit>* can_sender,
                 MessageManager<Transit>* message_manager) override;
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
  void SetEpbBreak(const ControlCommand& command) override;
  void SetBeam(const VehicleSignal& signal) override;
  void SetHorn(const VehicleSignal& signal) override;
  void SetTurningSignal(const VehicleSignal& signal) override;
  bool VerifyID() override;

  // CAN 协议消息指针
  Adcauxiliarycontrol110* adc_auxiliarycontrol_110_;
  Adcmotioncontrol110* adc_motioncontrol1_10_;
  Adcmotioncontrollimits112* adc_motioncontrollimits1_12_;
  Llcdiagbrakecontrol721* llc_diag_brakecontrol_721_;
  Llcdiagsteeringcontrol722* llc_diag_steeringcontrol_722_;

  void SecurityDogThreadFunc();   // 安全监控线程
  void SetLimits();               // 设置控制限幅
  void ResetProtocol();           // 重置所有发送消息
  bool CheckResponse();           // 检查 LLC 响应状态
  bool CheckChassisError();       // 底盘错误检测
  bool CheckSafetyError(const canbus::Transit& chassis);  // 安全错误检测
};
```

**源码**：`modules/canbus_vehicle/transit/transit_controller.h`

### TransitVehicleFactory

Transit 车辆工厂，创建控制器和消息管理器，管理 CAN 通信生命周期。

```cpp
class TransitVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const ControlCommand* control_command) override;
  void UpdateCommand(const ChassisCommand* chassis_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;

 private:
  std::unique_ptr<VehicleController<Transit>> CreateVehicleController();
  std::unique_ptr<MessageManager<Transit>> CreateMessageManager();

  std::unique_ptr<CanClient> can_client_;
  CanSender<Transit> can_sender_;
  CanReceiver<Transit> can_receiver_;
  std::unique_ptr<MessageManager<Transit>> message_manager_;
  std::unique_ptr<VehicleController<Transit>> vehicle_controller_;
};

CYBER_REGISTER_VEHICLEFACTORY(TransitVehicleFactory)
```

**源码**：`modules/canbus_vehicle/transit/transit_vehicle_factory.h`

### TransitMessageManager

CAN 报文管理器，注册所有发送和接收的协议数据。

**源码**：`modules/canbus_vehicle/transit/transit_message_manager.h`

## 核心函数

### TransitController::Init

初始化控制器，从 `MessageManager` 获取各 CAN 协议数据指针并注册到 `CanSender`。

```cpp
ErrorCode TransitController::Init(
    const VehicleParameter& params,
    CanSender<Transit>* can_sender,
    MessageManager<Transit>* message_manager);
```

- 校验 `driving_mode` 参数、`can_sender`、`message_manager` 非空
- 通过 `GetMutableProtocolDataById` 获取 5 个控制协议指针
- 调用 `can_sender_->AddMessage` 注册发送消息

### TransitController::chassis

从 `MessageManager` 获取底盘传感器数据并填充 `Chassis` protobuf。

- `llc_motionfeedback1_20`（CAN 0x20）：油门位置、制动百分比、档位状态、自动驾驶状态
- `llc_motionfeedback2_21`（CAN 0x21）：车速、转向角度
- `llc_auxiliaryfeedback_120`（CAN 0x78）：转向灯状态
- 转向角度按 `max_steer_angle` 归一化为百分比

### TransitController::EnableAutoMode

切换到完全自动驾驶模式。设置自动驾驶请求、角度转向模式、直接油门制动纵向控制，然后调用 `CheckResponse()` 验证。需物理按键按下（`button_pressed_`）才能真正进入 `COMPLETE_AUTO_DRIVE`。

### TransitController::Gear

设置档位。支持 P/D/N/R 四种档位映射到 `adc_motioncontrol1_10_` 的 `adc_cmd_gear` 字段。`GEAR_LOW` 映射为 `D_DRIVE`，`GEAR_NONE` 和 `GEAR_INVALID` 默认为 `N_NEUTRAL`。

### TransitController::Brake / Throttle

设置制动踏板百分比和油门踏板位置（0.00~99.99%），写入 `adc_motioncontrol1_10_`。仅在 `COMPLETE_AUTO_DRIVE` 或 `AUTO_SPEED_ONLY` 模式下生效，且需要物理按键按下。

### TransitController::Steer

设置方向盘角度，将百分比角度转换为实际角度（度），写入 `adc_motioncontrol1_10_->set_adc_cmd_steerwheelangle`。仅在 `COMPLETE_AUTO_DRIVE` 或 `AUTO_STEER_ONLY` 模式下生效。

### TransitController::SecurityDogThreadFunc

安全监控线程，以 50ms 周期循环检查横向和纵向控制响应。连续失败 10 次（`kMaxFailAttempt`）后触发紧急模式，重置所有发送消息。

### TransitVehicleFactory::Init

初始化整个 CAN 通信链路：创建 `CanClient` -> 创建 `MessageManager` -> 初始化 `CanReceiver` -> 初始化 `CanSender` -> 创建并初始化 `VehicleController`。同时创建 Cyber 节点 `chassis_detail` 并注册 chassis detail writer。

### TransitVehicleFactory::Start

按顺序启动 CAN 硬件：`can_client_->Start()` -> `can_receiver_.Start()` -> `can_sender_.Start()` -> `vehicle_controller_->Start()`。

## 配置

- **VehicleParameter**：通过 `canbus_conf->vehicle_parameter()` 传入，包含 `driving_mode` 和 `max_steer_angle` 等参数
- **CanbusConf**：通过 `canbus_conf->can_card_parameter()` 配置 CAN 硬件卡参数
- **SetLimits**：控制器硬编码控制限幅，油门命令限制 100，转向角度限制 1275，转向速率 500

## 调用关系

- **父类**：`AbstractVehicleFactory`（工厂）、`VehicleController<Transit>`（控制器）
- **依赖**：`drivers/canbus`（CAN 通信）、`protocol/`（CAN 协议解析）
- **被调用**：`CanbusComponent` 通过 `CYBER_REGISTER_VEHICLEFACTORY` 插件机制加载
