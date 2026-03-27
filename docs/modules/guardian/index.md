# Guardian 安全守护模块

> Apollo 自动驾驶系统的最后一道安全防线——当系统异常时主动接管并触发制动。

## 模块职责

Guardian 模块是 Apollo 自动驾驶系统中的安全守护组件，职责类似于电路中的"保险丝"。它位于控制模块（Control）和底盘模块（Canbus）之间，持续监控系统健康状态，在正常情况下透传控制指令，在异常情况下立即接管并触发安全制动。

具体职责包括：

- 订阅并监控 Monitor 模块上报的 `SystemStatus`，判断系统是否处于安全状态
- 在系统正常时，将 Control 模块的控制指令原样透传给底盘
- 在系统异常时，拦截控制指令，生成安全制动指令（软停车或紧急停车）
- 结合超声波传感器数据判断周围障碍物情况，决定制动力度
- 以 10ms 定时周期运行，确保对异常的快速响应（最大引入 10ms 控制延迟）

## 核心类与接口

### GuardianComponent

`GuardianComponent` 是模块的唯一核心类，继承自 `apollo::cyber::TimerComponent`，以定时器模式运行。

```
文件位置：modules/guardian/guardian_component.h
         modules/guardian/guardian_component.cc
```

**类定义：**

```cpp
class GuardianComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  void PassThroughControlCommand();
  void TriggerSafetyMode();

  apollo::guardian::GuardianConf guardian_conf_;
  apollo::canbus::Chassis chassis_;
  apollo::monitor::SystemStatus system_status_;
  apollo::control::ControlCommand control_cmd_;
  apollo::guardian::GuardianCommand guardian_cmd_;

  double last_status_received_s_{};

  std::shared_ptr<cyber::Reader<Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<ControlCommand>> control_cmd_reader_;
  std::shared_ptr<cyber::Reader<SystemStatus>> system_status_reader_;
  std::shared_ptr<cyber::Writer<GuardianCommand>> guardian_writer_;

  std::mutex mutex_;
};
```

**关键方法说明：**

| 方法 | 说明 |
| --- | --- |
| `Init()` | 加载配置文件，创建三个 Reader（Chassis、ControlCommand、SystemStatus）和一个 Writer（GuardianCommand） |
| `Proc()` | 定时回调入口，判断是否触发安全模式，分发到 `TriggerSafetyMode()` 或 `PassThroughControlCommand()` |
| `PassThroughControlCommand()` | 正常模式：将 `control_cmd_` 原样拷贝到 `guardian_cmd_` 中透传 |
| `TriggerSafetyMode()` | 安全模式：根据传感器状态和系统要求，生成紧急停车或软停车指令 |

### GuardianConf（配置 Proto）

```protobuf
// 文件：modules/guardian/proto/guardian_conf.proto
message GuardianConf {
  optional bool guardian_enable = 1 [default = false];
  optional double guardian_cmd_emergency_stop_percentage = 2 [default = 50];
  optional double guardian_cmd_soft_stop_percentage = 3 [default = 25];
}
```

### GuardianCommand（输出消息 Proto）

```protobuf
// 文件：modules/common_msgs/guardian_msgs/guardian.proto
message GuardianCommand {
  optional apollo.common.Header header = 1;
  optional apollo.control.ControlCommand control_command = 2;
}
```

`GuardianCommand` 是对 `ControlCommand` 的封装。正常模式下内部的 `control_command` 与原始控制指令一致；安全模式下则被替换为制动指令。

## 数据流

### 输入通道

| Channel | 消息类型 | 说明 |
| --- | --- | --- |
| `/apollo/canbus/chassis` | `apollo::canbus::Chassis` | 底盘状态，包含超声波传感器数据 |
| `/apollo/control` | `apollo::control::ControlCommand` | 控制模块输出的控制指令 |
| `/apollo/monitor/system_status` | `apollo::monitor::SystemStatus` | Monitor 模块上报的系统健康状态 |

### 输出通道

| Channel | 消息类型 | 说明 |
| --- | --- | --- |
| `/apollo/guardian` | `apollo::guardian::GuardianCommand` | Guardian 处理后的控制指令，发送给底盘执行 |

### 数据流向图

```
┌──────────┐    ┌──────────┐    ┌──────────┐
│  Canbus  │    │ Control  │    │ Monitor  │
│ (Chassis)│    │(CtrlCmd) │    │(SysStatus│
└────┬─────┘    └────┬─────┘    └────┬─────┘
     │               │               │
     ▼               ▼               ▼
┌─────────────────────────────────────────┐
│           GuardianComponent             │
│                                         │
│  ┌─────────────────────────────────┐    │
│  │          Proc() 定时回调        │    │
│  │                                 │    │
│  │  安全模式？──否──▶ 透传控制指令  │    │
│  │      │                          │    │
│  │      是                         │    │
│  │      ▼                          │    │
│  │  TriggerSafetyMode()           │    │
│  │   ├─ 紧急停车（brake=50%）      │    │
│  │   └─ 软停车  （brake=25%）      │    │
│  └─────────────────────────────────┘    │
└────────────────┬────────────────────────┘
                 │
                 ▼
          /apollo/guardian
          (GuardianCommand)
                 │
                 ▼
          ┌──────────┐
          │  Canbus  │
          │  底盘执行 │
          └──────────┘
```

## 配置方式

### 配置文件

Guardian 模块的配置文件位于：

```
modules/guardian/conf/guardian_conf.pb.txt
```

当前默认配置内容：

```protobuf
guardian_enable: true
```

### 配置项说明

| 配置项 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `guardian_enable` | `bool` | `false` | 是否启用 Guardian 安全守护功能。设为 `false` 时模块仅透传控制指令 |
| `guardian_cmd_emergency_stop_percentage` | `double` | `50` | 紧急停车时的制动力百分比（0-100） |
| `guardian_cmd_soft_stop_percentage` | `double` | `25` | 软停车时的制动力百分比（0-100） |

### DAG 配置

```protobuf
// 文件：modules/guardian/dag/guardian.dag
module_config {
    module_library : "modules/guardian/libguardian_component.so"
    timer_components {
        class_name : "GuardianComponent"
        config {
            name: "guardian"
            config_file_path: "/apollo/modules/guardian/conf/guardian_conf.pb.txt"
            interval: 10
        }
    }
}
```

`interval: 10` 表示定时器周期为 10ms，即 Guardian 以 100Hz 的频率运行。

### 启动方式

```bash
cyber_launch start modules/guardian/launch/guardian.launch
```

## 安全守护策略

Guardian 的安全守护策略分为两个层级：正常透传和安全模式。

### 正常透传模式

当以下条件全部满足时，Guardian 处于正常透传模式：

1. `guardian_enable` 配置为 `false`，或
2. `guardian_enable` 为 `true`，且：
   - 距离上次收到 `SystemStatus` 消息的时间未超过 2.5 秒（`kSecondsTillTimeout`）
   - `SystemStatus` 中不包含 `safety_mode_trigger_time` 字段

此模式下，`PassThroughControlCommand()` 将 Control 模块的指令原样拷贝到 `GuardianCommand` 中输出。

### 安全模式触发条件

当 `guardian_enable` 为 `true` 时，满足以下任一条件即触发安全模式：

1. **通信超时**：距离上次收到 `SystemStatus` 消息超过 2.5 秒。这意味着 Monitor 模块可能已经崩溃或通信链路中断。
2. **Monitor 主动触发**：`SystemStatus` 消息中包含 `safety_mode_trigger_time` 字段，表示 Monitor 检测到了需要进入安全模式的异常。

```cpp
// 安全模式判断核心逻辑
constexpr double kSecondsTillTimeout(2.5);

bool safety_mode_triggered = false;
if (guardian_conf_.guardian_enable()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (Time::Now().ToSecond() - last_status_received_s_ > kSecondsTillTimeout) {
        safety_mode_triggered = true;
    }
    safety_mode_triggered =
        safety_mode_triggered || system_status_.has_safety_mode_trigger_time();
}
```

## 紧急制动逻辑

进入安全模式后，`TriggerSafetyMode()` 执行以下逻辑：

### 第一步：传感器状态检测

检查底盘超声波传感器（Sonar）状态：

- **传感器故障**（`sensor_malfunction = true`）：超声波未启用（`sonar_enabled() == false`）或传感器报错（`sonar_fault() == true`）
- **障碍物检测**（`obstacle_detected = true`）：任一超声波传感器的测距值在 `(0, 2.5m)` 范围内（近距离障碍物），或测距值大于 30m（传感器异常输出）

### 第二步：设置安全控制指令

无论何种停车方式，都会设置以下基础参数：

```cpp
guardian_cmd_.mutable_control_command()->set_throttle(0.0);        // 油门归零
guardian_cmd_.mutable_control_command()->set_steering_target(0.0);  // 方向盘回正
guardian_cmd_.mutable_control_command()->set_steering_rate(25.0);   // 方向盘回正速率
guardian_cmd_.mutable_control_command()->set_is_in_safe_mode(true); // 标记安全模式
```

### 第三步：选择制动力度

根据条件选择紧急停车或软停车：

| 条件 | 制动方式 | 制动力（默认） |
| --- | --- | --- |
| `require_emergency_stop == true` | 紧急停车 | 50% |
| `sensor_malfunction == true` | 紧急停车 | 50% |
| `obstacle_detected == true` | 紧急停车 | 50% |
| 以上均不满足 | 软停车 | 25% |

紧急停车使用 `guardian_cmd_emergency_stop_percentage`（默认 50%）的制动力，软停车使用 `guardian_cmd_soft_stop_percentage`（默认 25%）的制动力。

```cpp
if (system_status_.require_emergency_stop() || sensor_malfunction ||
    obstacle_detected) {
    // 紧急停车
    guardian_cmd_.mutable_control_command()->set_brake(
        guardian_conf_.guardian_cmd_emergency_stop_percentage());
} else {
    // 软停车
    guardian_cmd_.mutable_control_command()->set_brake(
        guardian_conf_.guardian_cmd_soft_stop_percentage());
}
```

::: warning 硬件对齐临时处理
当前代码中存在一段临时逻辑，在硬件重新对齐完成前，强制将 `sensor_malfunction` 和 `obstacle_detected` 设为 `false`，忽略超声波传感器的输出。这意味着当前版本中，紧急停车仅由 `require_emergency_stop` 字段触发。
:::

## 目录结构

```
modules/guardian/
├── BUILD                          # Bazel 构建文件
├── conf/
│   └── guardian_conf.pb.txt       # 运行时配置（protobuf text 格式）
├── cyberfile.xml                  # 包描述文件（依赖声明）
├── dag/
│   └── guardian.dag               # DAG 调度配置
├── guardian_component.cc          # 核心实现
├── guardian_component.h           # 核心头文件
├── launch/
│   └── guardian.launch            # Cyber 启动文件
├── proto/
│   ├── BUILD                      # Proto 构建文件
│   └── guardian_conf.proto        # 配置消息定义
└── README.md                      # 模块说明
```

## 依赖关系

Guardian 模块的主要依赖：

- `cyber`：Apollo 的通信框架，提供 TimerComponent、Reader/Writer 等基础设施
- `common_msgs/chassis_msgs`：底盘消息定义（`Chassis`）
- `common_msgs/control_msgs`：控制指令消息定义（`ControlCommand`）
- `common_msgs/guardian_msgs`：Guardian 输出消息定义（`GuardianCommand`）
- `common_msgs/monitor_msgs`：系统状态消息定义（`SystemStatus`）
- `common/adapters`：Channel 名称定义（gflags）
