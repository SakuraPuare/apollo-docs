# Control 控制模块

> 基于规划轨迹与车辆状态，计算油门、刹车、方向盘指令，实现车辆横纵向跟踪控制。

## 模块职责

Control 模块是 Apollo 自动驾驶软件栈中连接 Planning（规划）与 Canbus（底盘）的桥梁。它以固定周期（默认 10ms）运行，每个控制周期完成以下工作：

1. 读取定位（Localization）、底盘（Chassis）、规划轨迹（ADCTrajectory）等输入消息
2. 校验输入数据的有效性与时间戳
3. 依次调用控制器流水线（Controller Pipeline）计算控制指令
4. 输出 `ControlCommand`（油门、刹车、方向盘角度、档位等）发送至 Canbus

模块支持两种运行模式：

- **单体模式**（默认）：`ControlComponent` 作为 `TimerComponent` 独立完成全部流程
- **子模块模式**（`FLAGS_use_control_submodules=true`）：将预处理、控制核心、后处理拆分为独立的 Cyber 组件，通过 channel 级联，适用于需要更灵活调度的场景

## 核心类与接口

### ControlComponent

入口类，继承自 `cyber::TimerComponent`，在 `Proc()` 中驱动整个控制流程。

- 源码：`control_component/control_component.h`
- 职责：消息收发、输入校验、E-Stop 处理、调用 `ControlTaskAgent`、发布控制指令

### ControlTask（控制器基类）

所有控制器的抽象接口，定义于 `control_component/controller_task_base/control_task.h`。

```cpp
class ControlTask {
 public:
  virtual Status Init(std::shared_ptr<DependencyInjector> injector) = 0;
  virtual Status ComputeControlCommand(
      const LocalizationEstimate*, const Chassis*,
      const ADCTrajectory*, ControlCommand*) = 0;
  virtual Status Reset() = 0;
  virtual std::string Name() const = 0;
  virtual void Stop() = 0;
 protected:
  bool LoadConfig(T* config);           // 从插件路径加载 pb.txt 配置
  bool LoadCalibrationTable(...);       // 加载标定表
};
```

每个控制器以 **Cyber 插件** 形式注册（`CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN`），由 `ControlTaskAgent` 在运行时通过 `PluginManager` 动态加载。

### ControlTaskAgent

控制器编排器，定义于 `control_component/controller_task_base/control_task_agent.h`。

- `Init()`：根据 `ControlPipeline` 配置依次实例化并初始化各控制器插件
- `ComputeControlCommand()`：按顺序调用每个控制器的 `ComputeControlCommand()`，后一个控制器可以读取/修改前一个控制器写入 `ControlCommand` 的结果
- `Reset()`：重置所有控制器状态（如切换到手动模式或 E-Stop 时）

### DependencyInjector

依赖注入容器，定义于 `controller_task_base/common/dependency_injector.h`，在各控制器间共享以下状态：

- `VehicleStateProvider`：车辆状态（位置、速度、加速度、航向等）
- 上一周期的 `ControlCommand` 和 `ControlDebugInfo`
- Planning 命令状态（`CommandStatus`）
- 控制过程标志位

### TrajectoryAnalyzer

轨迹分析工具，定义于 `controller_task_base/common/trajectory_analyzer.h`，提供：

- `QueryNearestPointByAbsoluteTime()` / `QueryNearestPointByRelativeTime()`：按时间查询轨迹点
- `QueryNearestPointByPosition()`：按位置查询最近轨迹点
- `QueryMatchedPathPoint()`：查询匹配路径点（支持插值）
- `ToTrajectoryFrame()`：将车辆状态转换到 Frenet 坐标系（纵向 s、横向 d）
- `TrajectoryTransformToCOM()`：将轨迹参考点从后轴中心转换到质心

### ControlTaskExtend

扩展控制器基类，定义于 `control_task_base_extend/control_task_extend.h`，继承自 `ControlTask`，为坡道防溜、重规划等高级控制任务提供额外的公共方法（车辆状态识别、坑洼检测、大曲率判断等）。

## 控制器算法概述

### 横向控制器：LQR（LatController）

源码：`controllers/lat_based_lqr_controller/lat_controller.cc`

基于线性二次调节器（LQR）的横向控制器，计算方向盘转角百分比。

**车辆动力学模型**：采用自行车模型（bicycle model），状态向量包含 4 个基本状态：

| 状态 | 含义 |
|------|------|
| `lateral_error` | 横向偏差（m） |
| `lateral_error_rate` | 横向偏差变化率（m/s） |
| `heading_error` | 航向偏差（rad） |
| `heading_error_rate` | 航向偏差变化率（rad/s） |

可选地附加 `preview_window` 个预瞄横向偏差状态。

**状态空间方程**：

```
A = [0,   1,           0,                    0;
     0,  -(cf+cr)/m/v, (cf+cr)/m,           (lr*cr-lf*cf)/m/v;
     0,   0,           0,                    1;
     0,  (lr*cr-lf*cf)/iz/v, (lf*cf-lr*cr)/iz, -(lf²*cf+lr²*cr)/iz/v]

B = [0, cf/m, 0, lf*cf/iz]^T
```

其中 `cf`/`cr` 为前/后轮侧偏刚度，`lf`/`lr` 为前/后轴到质心距离，`m` 为车辆质量，`iz` 为转动惯量。

**控制律**：

```
u_feedback = -K * x    （K 由 LQR 求解器迭代计算）
u_feedforward = f(ref_curvature, v, vehicle_params)
u = u_feedback + u_feedforward + u_augment
```

**增强特性**：

- **增益调度（Gain Scheduler）**：根据车速动态调整 Q 矩阵中横向偏差和航向偏差的权重
- **Lead-Lag 补偿器**：在低速或倒车时提供额外的反馈增强
- **MRAC 自适应控制**：模型参考自适应控制，补偿转向执行器延迟
- **Look-ahead/Look-back 控制**：前瞻/后视控制，根据车速插值选择前瞻距离，改善大曲率和低速场景的跟踪性能
- **倒车模式**：自动切换运动学模型，反转控制矩阵

### 纵向控制器：PID（LonController）

源码：`controllers/lon_based_pid_controller/lon_controller.cc`

基于级联 PID 的纵向控制器，计算油门/刹车百分比。

**控制架构**（级联双环）：

```
station_error → [Station PID] → speed_offset
                                      ↓
speed_error + speed_offset → [Speed PID] → acceleration_cmd_closeloop
                                                    ↓
acceleration_cmd = acceleration_cmd_closeloop
                 + preview_acceleration_reference
                 + slope_compensation
                                                    ↓
(speed, acceleration_cmd) → [Calibration Table] → throttle/brake
```

**PID 参数切换策略**：

| 条件 | Station PID | Speed PID |
|------|-------------|-----------|
| 倒车档 | `reverse_station_pid_conf` | `reverse_speed_pid_conf` |
| 低速（≤ switch_speed） | `station_pid_conf` | `low_speed_pid_conf` |
| 高速（> switch_speed） | `station_pid_conf` | `high_speed_pid_conf` |

**关键功能**：

- **坡度补偿**：通过俯仰角滤波计算重力分量补偿 `slope_offset = g * sin(pitch)`
- **完全停车逻辑**：根据剩余路径、停车原因（到达目的地/行人等待）决定是否进入完全停车状态
- **电子驻车制动（EPB）**：长时间停车后自动拉起驻车制动
- **转向等待**：倒车时若转向偏差过大，暂停纵向加速等待转向到位
- **Lead-Lag 补偿**：倒车时可选启用超前-滞后补偿器

### MPC 控制器（MPCController）

源码：`controllers/mpc_controller/mpc_controller.cc`

模型预测控制器，**同时处理横向和纵向控制**，是 LQR+PID 方案的替代选择。

**状态向量**（6 维）：

| 索引 | 状态 | 含义 |
|------|------|------|
| 0 | lateral_error | 横向偏差 |
| 1 | lateral_error_rate | 横向偏差变化率 |
| 2 | heading_error | 航向偏差 |
| 3 | heading_error_rate | 航向偏差变化率 |
| 4 | station_error | 纵向位置偏差 |
| 5 | speed_error | 速度偏差 |

**控制输入**（2 维）：`[steering, acceleration]`

**求解方法**：使用 OSQP 求解器在预测时域（默认 horizon=10）内求解带约束的二次规划问题，同时优化转向和加速度指令。

**与 LQR+PID 方案的对比**：

- MPC 可以显式处理约束（转向角限制、加速度限制等）
- MPC 同时优化横纵向，天然协调两个方向的控制
- 计算量更大，但对模型精度要求更高

### 辅助控制器

| 控制器 | 源码路径 | 功能 |
|--------|----------|------|
| `AntiSlipControlTask` | `controllers/slope_anti_slip_control_task/` | 坡道防溜控制，检测坡道起步条件并调整油门/刹车策略 |
| `ReplanControlTask` | `controllers/replan_control_task/` | 重规划控制，根据安全决策判断是否需要触发轨迹重规划 |
| `DemoControlTask` | `controllers/demo_control_task/` | 示例控制器，演示插件开发模式 |
| `DebugInfoControlTask` | `controllers/debug_info_control_task/` | 调试信息收集控制器 |

### 基础控制算法组件

| 组件 | 路径 | 说明 |
|------|------|------|
| `PIDController` | `controller_task_base/common/pid_controller.h` | 标准 PID 控制器（默认积分保持） |
| `PIDBCController` | `controller_task_base/common/pid_BC_controller.h` | PID + 反向计算抗积分饱和 |
| `PIDICController` | `controller_task_base/common/pid_IC_controller.h` | PID + 积分钳位抗积分饱和 |
| `LeadlagController` | `controller_task_base/common/leadlag_controller.h` | 超前-滞后补偿器（双线性变换离散化） |
| `MracController` | `controller_task_base/common/mrac_controller.h` | 模型参考自适应控制器（1/2 阶） |
| `Interpolation1D` | `controller_task_base/common/interpolation_1d.h` | 一维线性插值 |
| `Interpolation2D` | `controller_task_base/common/interpolation_2d.h` | 二维线性插值（用于标定表查表） |
| `HysteresisFilter` | `controller_task_base/common/hysteresis_filter.h` | 迟滞滤波器 |

## 横向/纵向控制分离设计

Apollo 控制模块的核心设计理念是**横纵向解耦**：

### 默认方案：LQR（横向）+ PID（纵向）

通过 `pipeline.pb.txt` 配置控制器执行顺序：

```protobuf
controller {
  name: "LAT_CONTROLLER"
  type: "LatController"
}
controller {
  name: "LON_CONTROLLER"
  type: "LonController"
}
```

`ControlTaskAgent` 按配置顺序依次调用：先由 `LatController` 计算 `steering_target`，再由 `LonController` 计算 `throttle`/`brake`。两者共享同一个 `ControlCommand` 对象，纵向控制器可以读取横向控制器已写入的转向指令（例如用于转向等待逻辑）。

### 替代方案：MPC（横纵向联合）

MPC 控制器在一次优化中同时输出转向和加速度指令，pipeline 中只需配置一个控制器。对应的 DAG 文件为 `dag/mpc_module.dag`。

### 子模块模式

当 `FLAGS_use_control_submodules=true` 时，控制流程被拆分为 Cyber 组件级联：

```
ControlComponent → [/apollo/control/localview]
    → PreprocessorSubmodule → [/apollo/control/preprocessor]
        → LatLonControllerSubmodule (或 MPCControllerSubmodule) → [/apollo/control/controlcore]
            → PostprocessorSubmodule → [/apollo/control/control_cmd]
```

这种模式下，`LatLonControllerSubmodule` 内部分别持有 `lateral_controller_` 和 `longitudinal_controller_` 两个 `ControlTask` 实例。

### Plus 版本控制器

模块还提供了增强版控制器：

- `LatPlusController`（`controllers/lat_based_lqr_plus_controller/`）
- `LonPlusController`（`controllers/lon_based_pid_plus_controller/`）

它们继承自 `ControlTaskExtend`，在基础算法之上增加了更多工程化特性（如扩展的轨迹分析、指数平滑、增强的 PID/Lead-Lag 等）。

## 数据流

### 输入

| 数据 | Channel | Proto 类型 | 说明 |
|------|---------|-----------|------|
| 底盘状态 | `/apollo/canbus/chassis` | `canbus.Chassis` | 车速、档位、转向角、驾驶模式等 |
| 规划轨迹 | `/apollo/planning` | `planning.ADCTrajectory` | 轨迹点序列（位置、速度、加速度、曲率）、档位、决策信息 |
| 定位信息 | `/apollo/localization/pose` | `localization.LocalizationEstimate` | 车辆位置、航向、速度、加速度 |
| 操控面板 | `/apollo/control/pad` | `PadMessage` | 驾驶动作指令（启动/停止/重置） |
| 规划命令状态 | `/apollo/planning/command_status` | `external_command.CommandStatus` | 规划任务完成状态 |

### 处理流程

```
1. Proc() 触发（10ms 周期）
2. 读取并缓存各 channel 最新消息 → 组装 LocalView
3. CheckInput()：校验轨迹非空、过滤低速零点
4. CheckTimestamp()：检查各消息时间戳是否超时
5. VehicleStateProvider::Update()：更新车辆状态
6. CheckAutoMode()：检测自动/手动模式切换
7. ProduceControlCommand()：
   a. E-Stop 检查（规划 estop、空轨迹、负速保护等）
   b. 非 E-Stop 时调用 ControlTaskAgent::ComputeControlCommand()
   c. E-Stop 时输出零速、软制动指令
8. 设置信号灯、延迟统计、俯仰角等附加信息
9. 发布 ControlCommand 到 /apollo/control
10. 保存当前控制指令和调试信息供下一周期使用
```

### 输出

| 数据 | Channel | Proto 类型 |
|------|---------|-----------|
| 控制指令 | `/apollo/control` | `control.ControlCommand` |
| 控制交互消息 | `/apollo/control/interactive` | `control.ControlInteractiveMsg` |

`ControlCommand` 的关键字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `throttle` | double | 油门百分比 [0, 100] |
| `brake` | double | 刹车百分比 [0, 100] |
| `steering_target` | double | 方向盘角度百分比 [-100, 100] |
| `steering_rate` | double | 方向盘转速百分比 |
| `acceleration` | double | 目标加速度（m/s²） |
| `speed` | double | 目标速度（m/s） |
| `gear_location` | enum | 档位（D/R/N/P） |
| `parking_brake` | bool | 驻车制动 |
| `debug` | Debug | 调试信息（横向/纵向/MPC 调试数据） |

## Proto 消息定义

### 控制模块内部 Proto

| Proto 文件 | 消息 | 用途 |
|-----------|------|------|
| `proto/pipeline.proto` | `ControlPipeline` | 控制器流水线配置，包含有序的 `PluginDeclareInfo` 列表 |
| `proto/plugin_declare_info.proto` | `PluginDeclareInfo` | 插件声明（name + type） |
| `proto/local_view.proto` | `LocalView` | 控制模块本地视图，聚合 Chassis/Trajectory/Localization/PadMessage |
| `proto/preprocessor.proto` | `Preprocessor` | 预处理子模块输出，包含 LocalView + engage_advice + estop 状态 |
| `proto/calibration_table.proto` | `calibration_table` | 标定表，包含 `ControlCalibrationInfo`（speed, acceleration, command）三元组列表 |
| `proto/pid_conf.proto` | `PidConf` | PID 控制器配置（kp, ki, kd, kaw, 积分饱和限制等） |
| `proto/leadlag_conf.proto` | `LeadlagConf` | Lead-Lag 控制器配置（alpha, beta, tau） |
| `proto/mrac_conf.proto` | `MracConf` | MRAC 控制器配置（模型阶数、参考模型参数、自适应增益等） |
| `proto/gain_scheduler_conf.proto` | `GainScheduler` | 增益调度配置，按速度/坡度调整增益比例 |
| `proto/control_debug.proto` | `ControlDebugInfo` | 扩展调试信息（横向/纵向/MPC/防溜/重规划等） |
| `proto/calibration_debug.proto` | `CalibrationDebug` | 标定调试信息 |
| `proto/check_status.proto` | `ControlCheckStatus` | 控制检查状态枚举（NONE/WARNING/ERROR） |

### 控制器专属 Proto

| Proto 文件 | 消息 | 用途 |
|-----------|------|------|
| `controllers/lat_based_lqr_controller/proto/lat_based_lqr_controller_conf.proto` | `LatBaseLqrControllerConf` | LQR 横向控制器配置 |
| `controllers/lon_based_pid_controller/proto/lon_based_pid_controller_conf.proto` | `LonBasedPidControllerConf` | PID 纵向控制器配置 |
| `controllers/mpc_controller/proto/mpc_controller.proto` | `MPCControllerConf` | MPC 控制器配置 |

### 公共消息 Proto（`modules/common_msgs/control_msgs/`）

| Proto 文件 | 关键消息 | 用途 |
|-----------|---------|------|
| `control_cmd.proto` | `ControlCommand`, `Debug`, `SimpleLongitudinalDebug`, `SimpleLateralDebug`, `SimpleMPCDebug` | 控制指令及调试信息 |
| `pad_msg.proto` | `PadMessage` | 操控面板消息 |
| `input_debug.proto` | `InputDebug` | 输入调试信息 |

## 标定表（Calibration Table）

标定表是纵向控制的核心查找表，建立了 **(车速, 加速度) → 执行器指令** 的映射关系。

### 数据结构

```protobuf
message calibration_table {
  repeated ControlCalibrationInfo calibration = 1;
}
message ControlCalibrationInfo {
  optional double speed = 1;         // 车速 (m/s)
  optional double acceleration = 2;  // 加速度 (m/s²)
  optional double command = 3;       // 执行器指令（正值=油门%, 负值=刹车%）
}
```

### 使用方式

标定表数据通过 `Interpolation2D` 进行二维线性插值：

```cpp
// 初始化
for (const auto& cal : calibration_table_.calibration()) {
    xyz.push_back(std::make_tuple(cal.speed(), cal.acceleration(), cal.command()));
}
control_interpolation_->Init(xyz);

// 查表
double calibration_value = control_interpolation_->Interpolate(
    std::make_pair(current_speed, desired_acceleration));
```

- `calibration_value > 0`：输出为油门指令
- `calibration_value < 0`：输出为刹车指令（取绝对值）

### 标定表特征

默认标定表（`conf/calibration_table.pb.txt`）覆盖：

- 车速范围：0.0 ~ 10.0 m/s（步长 0.2 m/s）
- 加速度范围：约 -9.0 ~ +3.3 m/s²
- 指令范围：-35（最大刹车）~ 80（最大油门）

标定表需要针对具体车型通过实车测试获取，是控制效果的关键因素之一。

## 配置方式

### 全局配置：control.conf

路径：`control_component/conf/control.conf`

使用 gflags 格式，关键配置项：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pipeline_file` | `conf/pipeline.pb.txt` | 控制器流水线配置文件路径 |
| `calibration_table_file` | `conf/calibration_table.pb.txt` | 标定表文件路径 |
| `control_period` | 0.01 | 控制周期（秒） |
| `enable_gain_scheduler` | true | 启用增益调度 |
| `use_control_submodules` | false | 是否使用子模块模式 |
| `enable_input_timestamp_check` | false | 启用输入时间戳检查 |
| `soft_estop_brake` | 15.0 | 软急停刹车百分比 |
| `minimum_speed_protection` | 0.1 | 最小速度保护值（m/s） |
| `enable_persistent_estop` | false | 持久化急停模式 |

### 控制器流水线：pipeline.pb.txt

路径：`control_component/conf/pipeline.pb.txt`

```protobuf
controller {
  name: "LAT_CONTROLLER"
  type: "LatController"
}
controller {
  name: "LON_CONTROLLER"
  type: "LonController"
}
```

控制器按声明顺序依次执行。`type` 字段对应插件类名，`ControlTaskAgent` 通过 `PluginManager` 以 `"apollo::control::" + type` 为全限定名加载插件。

### 控制器专属配置

每个控制器插件通过 `ControlTask::LoadConfig<T>()` 自动从插件目录加载 `conf/controller_conf.pb.txt`。例如：

- LQR 横向控制器：`lat_based_lqr_controller/conf/controller_conf.pb.txt` → `LatBaseLqrControllerConf`
- PID 纵向控制器：`lon_based_pid_controller/conf/controller_conf.pb.txt` → `LonBasedPidControllerConf`
- MPC 控制器：`mpc_controller/conf/controller_conf.pb.txt` → `MPCControllerConf`

### DAG 与 Launch 文件

| 文件 | 用途 |
|------|------|
| `dag/control.dag` | 单体模式，仅启动 `ControlComponent` |
| `dag/lateral_longitudinal_module.dag` | 子模块模式（LQR+PID），启动 ControlComponent + Preprocessor + LatLonController + Postprocessor |
| `dag/mpc_module.dag` | 子模块模式（MPC），启动 ControlComponent + Preprocessor + MPCController + Postprocessor |

### 自定义控制器开发

1. 继承 `ControlTask`（或 `ControlTaskExtend`）
2. 实现 `Init` / `ComputeControlCommand` / `Reset` / `Name` / `Stop`
3. 使用 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册为插件
4. 在 `pipeline.pb.txt` 中添加控制器声明
5. 参考 `DemoControlTask` 了解最小实现
