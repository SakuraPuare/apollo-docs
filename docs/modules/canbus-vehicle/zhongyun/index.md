---
title: "中云车辆"
---

# 中云车辆

> 源码路径: `modules/canbus_vehicle/zhongyun/`

## 概述

中云(Zhongyun)车辆是 Apollo 自动驾驶平台支持的一种特种车辆类型，通过 CAN 总线协议实现底盘控制与状态反馈。该模块采用标准的 Apollo 车辆适配架构，由工厂类、控制器和消息管理器三个核心组件组成，负责制动、转向、驱动扭矩、挡位和驻车等底盘子系统的指令下发与状态上报。

CAN 通信分为控制报文(发送)和反馈报文(接收)两组，共 9 条报文定义，分别对应不同的底盘控制域。

## 核心类

### ZhongyunController

继承自 `VehicleController<Zhongyun>`，是中云车辆的底盘控制器核心实现。负责将上层控制命令转化为具体的 CAN 协议指令，同时维护驾驶模式状态机和安全看门狗线程。

```cpp
class ZhongyunController final
    : public VehicleController<::apollo::canbus::Zhongyun> {
 public:
  ErrorCode Init(const VehicleParameter& params,
                 CanSender<Zhongyun>* can_sender,
                 MessageManager<Zhongyun>* message_manager);
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
  void Steer(double angle) override;
  void Steer(double angle, double angle_spd) override;
  void SetEpbBreak(const control::ControlCommand& command) override;
  // ...
};
```

**关键成员变量：**

- `brake_control_a4_` — 制动控制报文指针
- `gear_control_a1_` — 挡位控制报文指针
- `parking_control_a5_` — 驻车控制报文指针
- `steering_control_a2_` — 转向控制报文指针
- `torque_control_a3_` — 驱动扭矩控制报文指针
- `chassis_` — 底盘状态缓存
- `thread_` — 安全看门狗线程

### ZhongyunMessageManager

继承自 `MessageManager<Zhongyun>`，负责注册和管理所有 CAN 报文的收发映射关系。构造函数中通过模板方法将 5 条控制报文注册为发送端，4 条反馈报文注册为接收端。

```cpp
class ZhongyunMessageManager
    : public MessageManager<::apollo::canbus::Zhongyun> {
 public:
  ZhongyunMessageManager();
  virtual ~ZhongyunMessageManager();
};
```

### ZhongyunVehicleFactory

继承自 `AbstractVehicleFactory`，是中云车辆的工厂入口，通过 `CYBER_REGISTER_VEHICLEFACTORY` 宏注册到 Cyber RT 框架。负责创建和管理 CAN 客户端、收发器、控制器和消息管理器的完整生命周期。

```cpp
class ZhongyunVehicleFactory : public AbstractVehicleFactory {
 public:
  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;
  void UpdateCommand(const control::ControlCommand* control_command) override;
  void UpdateCommand(const external_command::ChassisCommand* chassis_command) override;
  Chassis publish_chassis() override;
  void PublishChassisDetail() override;

 private:
  std::unique_ptr<VehicleController<Zhongyun>> CreateVehicleController();
  std::unique_ptr<MessageManager<Zhongyun>> CreateMessageManager();
};
```

## 核心函数

### 控制器初始化与生命周期

| 函数 | 说明 |
|------|------|
| `ZhongyunController::Init` | 从 `MessageManager` 获取 5 条控制报文对象，注册到 `CanSender`，初始化车辆参数 |
| `ZhongyunController::Start` | 启动安全看门狗线程 `SecurityDogThreadFunc` |
| `ZhongyunController::Stop` | 等待看门狗线程退出，释放资源 |

### 驾驶模式切换

| 函数 | 说明 |
|------|------|
| `EnableAutoMode` | 设置全部 5 个子系统(转向、挡位、驱动、制动、驻车)为自动控制，调用 `CheckResponse` 验证后进入 `COMPLETE_AUTO_DRIVE` |
| `DisableAutoMode` | 重置所有协议报文，切换为 `COMPLETE_MANUAL` |
| `EnableSteeringOnlyMode` | 仅将转向子系统设为自动，其余保持手动 |
| `EnableSpeedOnlyMode` | 仅将挡位、驱动、制动、驻车设为自动，转向保持手动 |
| `Emergency` | 切换为 `EMERGENCY_MODE`，重置所有协议报文 |

### 底盘控制指令

| 函数 | 范围 | 说明 |
|------|------|------|
| `Brake(double)` | 0.00 ~ 99.99 % | 设置制动扭矩百分比 |
| `Throttle(double)` | 0.00 ~ 99.99 % | 设置驱动扭矩百分比 |
| `Gear(GearPosition)` | P/N/D/R | 设置目标挡位 |
| `Steer(double)` | -99.99 ~ 99.99 % | 设置转向角度，内部转换为实际角度值 |
| `Steer(double, double)` | 角度 + 角速度 | 设置转向角度(中云暂不支持角速度参数) |
| `SetEpbBreak` | 触发/释放 | 控制电子驻车制动 |
| `Acceleration` | -7.0 ~ 5.0 m/s^2 | 预留接口，当前未实现 |

### 状态反馈与安全监控

| 函数 | 说明 |
|------|------|
| `chassis` | 从 `MessageManager` 读取全部反馈报文，组装 `Chassis` protobuf，包含车速、挡位、转向角、制动扭矩、错误码和 `engage_advice` |
| `SecurityDogThreadFunc` | 以 50ms 周期运行，检查横向(转向)和纵向(速度)控制响应，连续 10 次失败或底盘报错时进入紧急模式 |
| `CheckResponse` | 轮询 `enable_state_feedback_c3` 报文，验证 EPS/VCU/ESP 在线状态，超时 20 次(每次 20ms)返回失败 |
| `CheckChassisError` | 读取 `error_state_e1` 报文，检查转向、驱动、制动、挡位、驻车 5 个域的错误标志 |

## 配置

中云车辆模块依赖以下配置和 Proto 定义：

- **车辆参数**：通过 `VehicleParameter` protobuf 配置，包含 `driving_mode`、`max_steer_angle` 等参数
- **CAN 总线配置**：通过 `CanbusConf` protobuf 配置，指定 CAN 卡参数和收发日志开关
- **Proto 定义**：`modules/canbus_vehicle/zhongyun/proto/zhongyun.pb.h`，定义 `Zhongyun` 顶层消息及其子消息

### CAN 报文表

**控制报文(发送方向)：**

| 报文类 | ID | 域 | 关键字段 |
|--------|----|----|----------|
| `Brakecontrola4` | 0xA4 | 制动 | `brake_torque` (0~100%), `brake_enable_control` |
| `Gearcontrola1` | 0xA1 | 挡位 | `gear_state_target` (P/N/D/R), `gear_enable_control` |
| `Parkingcontrola5` | 0xA5 | 驻车 | `parking_target` (触发/释放), `parking_enable_control` |
| `Steeringcontrola2` | 0xA2 | 转向 | `steering_target` (角度), `steering_enable_control` |
| `Torquecontrola3` | 0xA3 | 驱动 | `driven_torque` (0~100%), `driven_enable_control` |

**反馈报文(接收方向)：**

| 报文类 | ID | 域 | 关键字段 |
|--------|----|----|----------|
| `Vehiclestatefeedbackc1` | 0xC1 | 车辆状态1 | `speed` (kph), `steering_actual` (deg), `gear_state_actual`, `parking_actual` |
| `Enablestatefeedbackc3` | 0xC3 | 使能状态 | `steering_enable_state`, `gear_enable_actual`, `driven_enable_state`, `brake_enable_state` |
| `Vehiclestatefeedback2c4` | 0xC4 | 车辆状态2 | `motor_speed`, `driven_torque_feedback` |
| `Errorstatee1` | 0xE1 | 错误状态 | `steering_error_code`, `driven_error_code`, `brake_error_code`, `gear_error_msg`, `parking_error_code` |

## 调用关系

```text
ZhongyunVehicleFactory (工厂入口)
├── Init()
│   ├── 创建 CanClient → CanReceiver + CanSender
│   ├── 创建 ZhongyunMessageManager
│   │   ├── 注册 5 条控制报文 (A1~A5)
│   │   └── 注册 4 条反馈报文 (C1, C3, C4, E1)
│   └── 创建 ZhongyunController
│       └── Init(): 从 MessageManager 获取控制报文指针，注册到 CanSender
├── Start()
│   ├── CanClient::Start()
│   ├── CanReceiver::Start() → 接收 CAN 帧 → MessageManager::Parse → Protobuf
│   ├── CanSender::Start() → 周期发送控制报文
│   └── ZhongyunController::Start() → 启动 SecurityDogThreadFunc
├── UpdateCommand(ControlCommand)
│   ├── ZhongyunController::Update()
│   │   ├── EnableAutoMode / Gear / Brake / Steer / ...
│   │   └── 设置报文字段值
│   └── CanSender::Update() → 将报文数据写入 CAN 总线
├── publish_chassis()
│   └── ZhongyunController::chassis() → 读取反馈报文 → 组装 Chassis protobuf
└── Stop()
    ├── CanSender::Stop()
    ├── CanReceiver::Stop()
    ├── CanClient::Stop()
    └── ZhongyunController::Stop() → 等待安全线程退出

安全看门狗 (SecurityDogThreadFunc, 50ms 周期)
├── 检查横向控制: CheckResponse(STEER_UNIT_FLAG)
├── 检查纵向控制: CheckResponse(SPEED_UNIT_FLAG)
├── 检查底盘错误: CheckChassisError() → 读取 Errorstatee1
└── 异常时 → Emergency() → ResetProtocol() + 通知 CanSender
```
