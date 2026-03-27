# Canbus 模块

## 模块职责

Canbus 模块是 Apollo 自动驾驶系统中负责车辆底盘通信的核心模块。它通过 CAN 总线协议与车辆底盘 ECU 进行双向数据交换，承担以下职责：

- 接收上游控制模块（Control）或守护模块（Guardian）下发的控制指令（`ControlCommand`），将其转换为具体的 CAN 协议帧并发送到车辆底盘
- 接收车辆底盘上报的 CAN 帧数据，解析为结构化的底盘状态信息（`Chassis`），并发布到 Cyber RT 消息通道供其他模块使用
- 监控底盘通信健康状态，在指令超时或通信故障时触发紧急制动保护
- 支持通过动态库加载机制适配不同车型

源码位于两个目录：

| 目录 | 说明 |
|------|------|
| `modules/canbus/` | 模块框架层：组件入口、抽象接口、配置、工具 |
| `modules/canbus_vehicle/` | 车辆适配层：各车型的具体协议实现 |

## 整体架构

```
Control/Guardian 模块
        │
        ▼
┌─────────────────────────────────────────────┐
│              CanbusComponent                │
│         (TimerComponent, 100Hz)             │
│                                             │
│  ┌───────────────────────────────────────┐  │
│  │       AbstractVehicleFactory          │  │
│  │      (动态库加载，车型工厂)            │  │
│  │                                       │  │
│  │  ┌─────────────┐  ┌───────────────┐   │  │
│  │  │ VehicleCtrl │  │ MessageManager│   │  │
│  │  │ (控制逻辑)   │  │ (协议注册)     │   │  │
│  │  └──────┬──────┘  └───────┬───────┘   │  │
│  │         │                 │           │  │
│  │  ┌──────┴──────┐  ┌──────┴───────┐   │  │
│  │  │  CanSender  │  │ CanReceiver  │   │  │
│  │  └──────┬──────┘  └──────┬───────┘   │  │
│  └─────────┼────────────────┼───────────┘  │
└────────────┼────────────────┼──────────────┘
             │                │
             ▼                ▲
      ┌──────────────────────────────┐
      │        CanClient             │
      │   (CAN 硬件抽象层)            │
      └──────────────────────────────┘
             │                ▲
             ▼                │
        ═══════════════════════════
              CAN 总线 (硬件)
        ═══════════════════════════
             │                ▲
             ▼                │
          车辆底盘 ECU
```

## 核心类与接口

### CanbusComponent

文件：`modules/canbus/canbus_component.h`、`modules/canbus/canbus_component.cc`

模块的入口组件，继承自 `apollo::cyber::TimerComponent`，以定时器模式运行（默认 10ms 周期，即 100Hz）。

主要职责：

- **Init()**：加载配置文件（`CanbusConf`），通过 `ClassLoader` 动态加载车型工厂库（`.so` 文件），创建并初始化 `AbstractVehicleFactory` 实例；创建 Cyber RT 的 Reader/Writer 用于消息收发
- **Proc()**：每个定时周期执行一次，检查控制指令超时、检测底盘通信故障、发布 `Chassis` 状态消息和 `ChassisDetail` 详情消息、更新心跳
- **OnControlCommand()**：接收 `ControlCommand` 回调，做最小指令间隔过滤（默认 5ms），然后转发给车型工厂处理
- **OnChassisCommand()**：接收外部底盘指令 `ChassisCommand`，用于自定义车辆操作
- **ProcessGuardianCmdTimeout()**：指令超时时的紧急处理，将油门置零、`steering_target` 置零、`steering_rate` 设为 25.0、制动设为 `estop_brake`（默认 30%）

订阅的 Cyber RT 通道：

| 通道 | 消息类型 | 说明 |
|------|---------|------|
| `/apollo/control` | `ControlCommand` | 控制指令（非 Guardian 模式） |
| `/apollo/guardian` | `GuardianCommand` | 守护指令（Guardian 模式） |
| `/apollo/chassis_control` | `ChassisCommand` | 外部底盘指令 |

发布的 Cyber RT 通道：

| 通道 | 消息类型 | 说明 |
|------|---------|------|
| `/apollo/canbus/chassis` | `Chassis` | 底盘状态摘要 |
| `/apollo/canbus/chassis_detail` | 车型特定 proto | 底盘详细数据（接收侧） |
| `/apollo/canbus/chassis_detail_sender` | 车型特定 proto | 底盘详细数据（发送侧） |

### AbstractVehicleFactory

文件：`modules/canbus/vehicle/abstract_vehicle_factory.h`

抽象工厂基类，定义了车型适配的统一接口。每个具体车型需要继承此类并实现以下纯虚函数：

```cpp
class AbstractVehicleFactory {
 public:
  virtual bool Init(const CanbusConf *canbus_conf) = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual void UpdateCommand(const ControlCommand *control_command) = 0;
  virtual void UpdateCommand(const ChassisCommand *chassis_command) = 0;
  virtual Chassis publish_chassis() = 0;
  virtual void PublishChassisDetail() = 0;

  // 以下为可选覆盖的虚函数（有默认实现）
  virtual void PublishChassisDetailSender();
  virtual void UpdateHeartbeat();
  virtual bool CheckChassisCommunicationFault();
  virtual void AddSendProtocol();
  virtual void ClearSendProtocol();
  virtual bool IsSendProtocolClear();
  virtual Chassis::DrivingMode Driving_Mode();
};
```

通过宏 `CYBER_REGISTER_VEHICLEFACTORY(name)` 将具体工厂类注册到 Cyber 的类加载器中，实现运行时动态加载。

### VehicleController\<SensorType\>

文件：`modules/canbus/vehicle/vehicle_controller.h`

模板类，`SensorType` 为车型特定的 Protobuf 消息类型（如 `Ch`、`Lincoln` 等）。定义了车辆控制的完整接口：

**纯虚函数（子类必须实现）：**

| 方法 | 说明 |
|------|------|
| `Init()` | 初始化控制器，返回 `ErrorCode`，接受三个参数：`VehicleParameter`、`CanSender`、`MessageManager` |
| `Start()` / `Stop()` | 启停控制器 |
| `chassis()` | 读取底盘状态，组装 `Chassis` 消息 |
| `Emergency()` | 紧急模式处理 |
| `EnableAutoMode()` / `DisableAutoMode()` | 自动/手动模式切换 |
| `EnableSteeringOnlyMode()` | 仅转向模式 |
| `EnableSpeedOnlyMode()` | 仅速度模式 |
| `Gear()` | 档位控制（P/R/N/D） |
| `Brake()` | 制动踏板控制（0~100%） |
| `Throttle()` | 油门踏板控制（0~100%） |
| `Acceleration()` | 加速度控制（m/s^2） |
| `Steer()` | 转向控制（角度/角速度） |
| `Steer(double, double)` | 转向控制（带角速度的重载版本） |
| `SetEpbBreak()` | 电子驻车制动控制 |
| `HandleCustomOperation()` | 自定义操作处理 |
| `SetBeam()` | 灯光控制 |
| `SetHorn()` | 喇叭控制 |
| `SetTurningSignal()` | 转向灯控制 |
| `VerifyID()` | 车辆 VIN 码验证 |

**基类已实现的关键方法：**

- `Update(const ControlCommand&)`：根据当前驾驶模式分发控制指令到对应的执行函数（Gear、Throttle、Brake、Steer 等）
- `Update(const ChassisCommand&)`：处理外部底盘指令和自定义操作
- `SetDrivingMode()`：驾驶模式状态机切换，支持 `COMPLETE_AUTO_DRIVE`、`COMPLETE_MANUAL`、`AUTO_STEER_ONLY`、`AUTO_SPEED_ONLY`、`EMERGENCY_MODE`
- `CheckChassisCommunicationError()`：检测底盘通信丢失（连续 100 次接收数据为空则判定通信故障）

**关键成员变量：**

```cpp
CanSender<SensorType> *can_sender_;           // CAN 发送器
CanReceiver<SensorType> *can_receiver_;       // CAN 接收器
MessageManager<SensorType> *message_manager_; // 协议消息管理器
Chassis::DrivingMode driving_mode_;           // 当前驾驶模式
```

## 车辆适配层设计

### 目录结构

`modules/canbus_vehicle/` 目录下按车型组织，当前支持 11 种车型：

| 车型目录 | 说明 |
|---------|------|
| `ch/` | Coolhigh 车型 |
| `demo/` | 示例/模板车型 |
| `devkit/` | Apollo D-Kit 开发套件 |
| `ge3/` | 广汽 GE3 |
| `gem/` | Polaris GEM |
| `lexus/` | Lexus |
| `lincoln/` | Lincoln MKZ（默认车型） |
| `neolix_edu/` | Neolix 教育版 |
| `transit/` | Transit |
| `wey/` | 长城 WEY |
| `zhongyun/` | 中云智车 |

### 每个车型的标准文件结构

以 `ch/`（Coolhigh）车型为例：

```
ch/
├── ch_controller.h / .cc          # VehicleController 实现
├── ch_message_manager.h / .cc     # MessageManager 实现（注册收发协议）
├── ch_vehicle_factory.h / .cc     # AbstractVehicleFactory 实现
├── proto/
│   └── ch.proto                   # 车型特定的 Protobuf 定义
└── protocol/
    ├── brake_command_111.h / .cc   # 发送协议：制动指令 (CAN ID 0x111)
    ├── throttle_command_110.h / .cc# 发送协议：油门指令 (CAN ID 0x110)
    ├── steer_command_112.h / .cc   # 发送协议：转向指令 (CAN ID 0x112)
    ├── gear_command_114.h / .cc    # 发送协议：档位指令 (CAN ID 0x114)
    ├── turnsignal_command_113.h / .cc # 发送协议：转向灯指令 (CAN ID 0x113)
    ├── vehicle_mode_command_116.h / .cc # 发送协议：VIN 请求 (CAN ID 0x116)
    ├── control_command_115.h / .cc # 发送协议：控制指令 (CAN ID 0x115)
    ├── throttle_status__510.h / .cc# 接收协议：油门状态 (CAN ID 0x510)
    ├── brake_status__511.h / .cc   # 接收协议：制动状态 (CAN ID 0x511)
    ├── steer_status__512.h / .cc   # 接收协议：转向状态 (CAN ID 0x512)
    ├── gear_status_514.h / .cc     # 接收协议：档位状态 (CAN ID 0x514)
    ├── turnsignal_status__513.h / .cc # 接收协议：转向灯状态 (CAN ID 0x513)
    ├── ecu_status_1_515.h / .cc    # 接收协议：ECU 状态（速度/加速度/控制状态）
    ├── ecu_status_2_516.h / .cc    # 接收协议：电池 BMS 状态
    ├── ecu_status_3_517.h / .cc    # 接收协议：超声波 1~8
    ├── ecu_status_4_518.h / .cc    # 接收协议：超声波 9~16
    ├── vin_resp1_51b.h / .cc       # 接收协议：VIN 响应 1
    ├── vin_resp2_51c.h / .cc       # 接收协议：VIN 响应 2
    ├── vin_resp3_51d.h / .cc       # 接收协议：VIN 响应 3
    └── wheelspeed_report_51e.h / .cc # 接收协议：轮速
```

### 适配新车型的三要素

1. **VehicleFactory**：继承 `AbstractVehicleFactory`，负责组装 CanClient、CanSender、CanReceiver、MessageManager、VehicleController，并管理它们的生命周期
2. **VehicleController**：继承 `VehicleController<SensorType>`，实现具体的控制逻辑（模式切换、踏板/转向映射、底盘状态组装）
3. **MessageManager**：继承 `MessageManager<SensorType>`，在构造函数中注册所有发送和接收协议

## CAN 协议封装方式

### ProtocolData 基类

每个 CAN 协议帧对应一个 `ProtocolData<SensorType>` 子类（来自 `modules/drivers/canbus/`），封装了：

- **CAN ID**：静态常量 `ID`，如 `Brakecommand111::ID = 0x111`
- **发送周期**：`GetPeriod()` 返回发送间隔（微秒），如制动指令为 20ms
- **数据编码**：`UpdateData(uint8_t* data)` 将当前状态编码为 CAN 帧的 8 字节数据
- **数据解码**：接收协议通过 `Parse()` 方法从原始字节解析到 Protobuf 字段
- **值域约束**：通过 `BoundedValue()` 进行范围裁剪

### 信号编码细节

以制动指令 `Brakecommand111` 为例，CAN 帧结构为：

| 字节偏移 | 位范围 | 信号名 | 说明 |
|---------|--------|--------|------|
| Byte 0 | [0:7] | `BRAKE_PEDAL_EN_CTRL` | 制动使能（0=禁用, 1=启用） |
| Byte 1 | [0:7] | `BRAKE_PEDAL_CMD` | 制动踏板百分比（0~100%） |

编码使用 `Byte` 工具类进行位级操作：

```cpp
void Brakecommand111::set_p_brake_pedal_en_ctrl(uint8_t* data, ...) {
  Byte to_set(data + 0);        // 定位到 Byte 0
  to_set.set_value(x, 0, 8);   // 从 bit 0 开始写入 8 位
}
```

每个信号的元数据（bit 位置、长度、字节序、精度、偏移、物理范围）以注释形式记录在代码中，格式统一，便于自动化代码生成。

### MessageManager 的协议注册

`MessageManager` 通过模板方法注册发送和接收协议：

```cpp
ChMessageManager::ChMessageManager() {
  // 发送协议（Control Messages）
  AddSendProtocolData<Brakecommand111, true>();
  AddSendProtocolData<Gearcommand114, true>();
  AddSendProtocolData<Steercommand112, true>();
  AddSendProtocolData<Throttlecommand110, true>();
  AddSendProtocolData<Turnsignalcommand113, true>();
  AddSendProtocolData<Vehiclemodecommand116, true>();

  // 接收协议（Report Messages）
  AddRecvProtocolData<Brakestatus511, true>();
  AddRecvProtocolData<Ecustatus1515, true>();
  // ... 其余接收协议
}
```

第二个模板参数 `true` 表示启用该协议的校验检查。

## 数据流

### 控制指令下发流程

```
ControlCommand (Cyber RT)
    │
    ▼
CanbusComponent::OnControlCommand()
    │  最小间隔过滤 (5ms)
    ▼
AbstractVehicleFactory::UpdateCommand()
    │
    ▼
VehicleController::Update(ControlCommand)
    │  根据 DrivingMode 分发
    ├── Gear()      → gear_command_114_->set_gear_cmd()
    ├── Throttle()  → throttle_command_110_->set_throttle_pedal_cmd()
    ├── Brake()     → brake_command_111_->set_brake_pedal_cmd()
    ├── Steer()     → steer_command_112_->set_steer_angle_cmd()
    └── Signal()    → turnsignal_command_113_->set_turn_signal_cmd()
    │
    ▼
CanSender::Update()
    │  按各协议的周期 (20ms) 定时发送
    ▼
CanClient → CAN 总线 → 车辆 ECU
```

### 底盘状态上报流程

```
车辆 ECU → CAN 总线 → CanClient
    │
    ▼
CanReceiver (后台线程持续接收)
    │  按 CAN ID 分发到对应 ProtocolData::Parse()
    ▼
MessageManager (存储解析后的 Protobuf 数据)
    │
    ▼
CanbusComponent::Proc() (100Hz 定时触发)
    │
    ├── VehicleController::chassis()
    │       从 MessageManager 获取最新数据
    │       组装为统一的 Chassis 消息
    │       （速度、油门、制动、档位、转向、电池、故障码等）
    │
    ├── PublishChassis()
    │       发布到 /apollo/canbus/chassis
    │
    └── PublishChassisDetail()
            发布车型特定的详细数据
```

### 安全保护机制

1. **指令超时检测**：`Proc()` 中检查控制指令时间戳，超过阈值（默认 `max_control_miss_num * control_period = 5 * 10ms = 50ms`）触发紧急制动
2. **通信故障检测**：`CheckChassisCommunicationError()` 连续 100 次（约 1 秒）未收到有效底盘数据则判定通信故障，进入 `EMERGENCY_MODE`
3. **SecurityDog 线程**：各车型 Controller 内部运行看门狗线程（50ms 周期），检测转向和速度控制响应，发现人工接管或底盘故障时自动切换到紧急模式
4. **紧急制动处理**：超时时将油门置零、`steering_target` 置零、`steering_rate` 设为 25.0、制动设为 `estop_brake`（默认 30%）

## 配置方式

### Protobuf 配置（CanbusConf）

定义在 `modules/canbus/proto/canbus_conf.proto`：

```protobuf
// canbus_conf.proto
message CanbusConf {
  // VehicleParameter 来自 vehicle_parameter.proto（通过 import 引入）
  optional VehicleParameter vehicle_parameter = 1;
  optional CANCardParameter can_card_parameter = 2;
  optional bool enable_debug_mode = 3 [default = false];
  optional bool enable_receiver_log = 4 [default = false];
  optional bool enable_sender_log = 5 [default = false];
}

// vehicle_parameter.proto
message VehicleParameter {
  optional apollo.common.VehicleBrand brand = 1;
  optional double max_engine_pedal = 2;
  optional int32 max_enable_fail_attempt = 3;
  optional Chassis.DrivingMode driving_mode = 4;
}
```

配置文件路径：`/apollo/modules/canbus/conf/canbus_conf.pb.txt`

### GFlags 运行时参数

定义在 `modules/canbus/common/canbus_gflags.cc`，通过 `modules/canbus/conf/canbus.conf` 文件设置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `chassis_freq` | 100 | 底盘反馈频率 (Hz) |
| `min_cmd_interval` | 5 | 最小指令间隔 (ms) |
| `control_period` | 0.01 | 控制周期 (s) |
| `max_control_miss_num` | 5 | 最大控制指令丢失次数 |
| `estop_brake` | 30.0 | 紧急制动力度 (%) |
| `receive_guardian` | false | 是否接收 Guardian 指令 |
| `enable_chassis_detail_pub` | true | 是否发布底盘详情 |
| `enable_chassis_detail_sender_pub` | true | 是否发布发送侧底盘详情 |
| `chassis_debug_mode` | false | 底盘调试模式 |
| `use_control_cmd_check` | false | 是否启用控制指令超时检查 |
| `load_vehicle_library` | `.../liblincoln_vehicle_factory_lib.so` | 车型动态库路径 |
| `load_vehicle_class_name` | `LincolnVehicleFactory` | 车型工厂类名 |

### DAG 配置

文件：`modules/canbus/dag/canbus.dag`

```
module_config {
    module_library : "modules/canbus/libcanbus_component.so"
    timer_components {
        class_name : "CanbusComponent"
        config {
            name: "canbus"
            config_file_path: "/apollo/modules/canbus/conf/canbus_conf.pb.txt"
            flag_file_path: "/apollo/modules/canbus/conf/canbus.conf"
            interval: 10
        }
    }
}
```

`interval: 10` 表示定时器周期为 10ms（100Hz）。

### 切换车型

修改 `canbus.conf` 中的两个参数即可切换车型：

```bash
--load_vehicle_library=/opt/apollo/neo/lib/modules/canbus_vehicle/ch/libch_vehicle_factory_lib.so
--load_vehicle_class_name=ChVehicleFactory
```

## 辅助工具

### canbus_tester

文件：`modules/canbus/tools/canbus_tester.cc`

从 Protobuf 文本文件加载 `ControlCommand`，以 1Hz 频率持续发送到控制通道，用于离线测试 CAN 总线通信。

### teleop

文件：`modules/canbus/tools/teleop.cc`

键盘遥控工具，通过终端按键实时控制车辆：

- `W/S`：加速/减速
- `A/D`：左转/右转
- `P`：驻车制动
- `E`：紧急制动
- `G+数字`：切换档位
- `M+数字`：切换驾驶模式（RESET/START/VIN_REQ）
- `Q`：切换转向灯
- `L`：切换近光灯
