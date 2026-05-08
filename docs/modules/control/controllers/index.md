---
title: "Control Controllers 控制器函数级源码解析"
---

# Control Controllers 控制器函数级源码解析

本文聚焦 `modules/control/controllers/` 目录，按函数级粒度拆解两个核心控制器的实现：`LatController`（基于 LQR 的横向控制器）和 `LonController`（基于 PID 的纵向控制器）。

## 1. 模块定位

控制器是 Apollo 控制栈的**执行核心**。`ControlComponent` 通过 `ControlTaskAgent` 串行调用控制器插件，每个控制器专注于一个控制维度：

- **LatController**：计算方向盘转角（steering），跟踪规划轨迹的横向偏差
- **LonController**：计算油门/刹车指令（throttle/brake），跟踪规划轨迹的速度/位置

```
ControlComponent → ControlTaskAgent → [LatController, LonController]
                                           │              │
                                     steering_angle   throttle/brake
                                           │              │
                                           └──── ControlCommand ────→ Canbus
```

## 2. 目录结构

```
modules/control/controllers/
├── lat_based_lqr_controller/
│   ├── lat_controller.h / .cc             # LQR 横向控制器
│   ├── lat_controller_test.cc             # 单元测试
│   ├── proto/                             # 配置 Protobuf
│   ├── conf/                              # 默认配置
│   └── BUILD                              # Bazel 构建规则
└── lon_based_pid_controller/
    ├── lon_controller.h / .cc             # PID 纵向控制器
    ├── lon_controller_test.cc
    ├── util/
    │   ├── check_pit.h / .cc              # 坑检测工具
    │   └── BUILD
    ├── proto/                             # 配置 Protobuf
    ├── conf/                              # 默认配置
    └── BUILD
```

## 3. LatController — LQR 横向控制器

### 3.1 类声明

```cpp
class LatController : public ControlTask {
 public:
  LatController();
  virtual ~LatController();
  common::Status Init(std::shared_ptr<DependencyInjector> injector) override;
  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate* localization,
      const canbus::Chassis* chassis,
      const planning::ADCTrajectory* trajectory,
      ControlCommand* cmd) override;
  common::Status Reset() override;
  void Stop() override;
  std::string Name() const override;
};
```

通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册为 `ControlTask` 插件。

### 3.2 车辆动力学模型

LatController 基于**自行车模型（Bicycle Model）**，状态向量为：

```
x = [lateral_error, lateral_error_rate, heading_error, heading_error_rate]
```

状态方程：

```
ẋ = A·x + B·δ
```

其中 `δ` 为前轮转角，矩阵 A 和 B 由以下车辆参数决定：

| 参数 | 成员变量 | 说明 |
|------|---------|------|
| `cf` | `cf_` | 前轮侧偏刚度 |
| `cr` | `cr_` | 后轮侧偏刚度 |
| `lf` | `lf_` | 前轴到质心距离 |
| `lr` | `lr_` | 后轴到质心距离 |
| `mass` | `mass_` | 车辆质量 |
| `iz` | `iz_` | 绕 z 轴转动惯量 |
| `wheelbase` | `wheelbase_` | 轴距 |
| `steer_ratio` | `steer_ratio_` | 方向盘到前轮转角传动比 |

### 3.3 `Init` — 初始化

```cpp
common::Status LatController::Init(std::shared_ptr<DependencyInjector> injector);
```

1. 加载配置 `LatBaseLqrControllerConf`
2. `LoadControlConf`：读取车辆参数、控制周期 `ts_`、侧偏刚度 `cf_`/`cr_`、前瞻参数等
3. 初始化矩阵维度：`matrix_a_`(4×4)、`matrix_b_`(4×1)、`matrix_q_`(4×4)、`matrix_r_`(1×1)
4. `InitializeFilters`：初始化数字滤波器和均值滤波器
5. `LoadLatGainScheduler`：加载增益调度表（速度→增益插值）
6. 初始化 Lead-Lag 补偿器和 MRAC 自适应控制器（若启用）

### 3.4 `ComputeControlCommand` — 核心计算

```cpp
common::Status LatController::ComputeControlCommand(
    const localization::LocalizationEstimate* localization,
    const canbus::Chassis* chassis,
    const planning::ADCTrajectory* trajectory,
    ControlCommand* cmd);
```

**算法流程**：

#### Step 1：轨迹分析器更新

```cpp
trajectory_analyzer_.Update(trajectory);
```

- 从 `ADCTrajectory` 中提取轨迹点序列
- 构建 `TrajectoryAnalyzer` 用于后续最近点匹配和误差计算

#### Step 2：状态更新

```cpp
UpdateState(&debug, chassis);
```

- `UpdateState`：
  - 获取自车位置 `(x, y)` 和航向角 `theta`
  - `ComputeLateralErrors`：计算横向误差
    - 在轨迹上找到匹配点
    - 计算横向误差 `lateral_error`、航向误差 `heading_error`
    - 计算误差变化率 `lateral_error_rate`、`heading_error_rate`
  - `UpdateDrivingOrientation`：倒车模式下翻转驾驶方向

#### Step 3：矩阵更新

```cpp
UpdateMatrix();
UpdateMatrixCompound();
```

**`UpdateMatrix`**：

- 根据当前车速更新状态矩阵 `A` 和控制矩阵 `B`
- 矩阵元素是车速的函数（自行车模型线性化）
- 离散化：`Ad = I + A·ts`，`Bd = B·ts`

**`UpdateMatrixCompound`**：

- 若启用 preview 控制器，扩展状态矩阵以包含前瞻信息
- `preview_window_` 个前瞻周期的状态叠加

#### Step 4：LQR 求解

```cpp
matrix_k_ = SolveLQR(matrix_adc_, matrix_bdc_, matrix_q_updated_, matrix_r_,
                      lqr_max_iteration_, lqr_eps_);
```

- 调用 `LinearQuadraticRegulator` 迭代求解 Riccati 方程
- 得到最优反馈增益矩阵 `K`
- `matrix_q_updated_`：使用增益调度表根据速度调整状态权重

#### Step 5：前馈补偿

```cpp
double steer_angle_feedforward = ComputeFeedForward(ref_curvature);
```

```cpp
double LatController::ComputeFeedForward(double ref_curvature) const {
  double kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  double steer_angle_feedforward = wheelbase_ * ref_curvature
      + kv * v * v * ref_curvature - matrix_k_(0, 2) *
      (lr_ * ref_curvature - lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_);
  return steer_angle_feedforward * steer_ratio_ * 180 / M_PI / steer_single_direction_max_degree_;
}
```

- 基于稳态曲率的前馈补偿
- 公式考虑了车辆参数 `kv`（不足转向梯度）
- 将弧度转换为方向盘百分比

#### Step 6：方向盘指令计算

```cpp
double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0);
double steer_angle = steer_angle_feedback + steer_angle_feedforward;
```

- 反馈项：`-K·x`（LQR 最优控制律）
- 前馈项：稳态曲率补偿
- 总转角 = 反馈 + 前馈

#### Step 7：后处理

- `Lead-Lag` 补偿器（若启用）：改善相位裕度
- `MRAC` 自适应控制器（若启用）：在线调整增益
- 数字滤波器：平滑输出
- 限幅：`[-steer_single_direction_max_degree_, +steer_single_direction_max_degree_]`
- 横向加速度限制：`max_lat_acc_` 约束

#### Step 8：输出

```cpp
cmd->set_steering_target(steer_angle);
cmd->set_steering_rate(steer_rate);
```

### 3.5 `ComputeLateralErrors` — 横向误差计算

```cpp
void LatController::ComputeLateralErrors(
    double x, double y, double theta, double linear_v, double angular_v,
    double linear_a, const TrajectoryAnalyzer& trajectory_analyzer,
    SimpleLateralDebug* debug, const canbus::Chassis* chassis);
```

1. 在轨迹上找到与自车最近的匹配点
2. 计算横向误差 `l`（自车到轨迹的法向距离）
3. 计算航向误差 `Δθ = θ_vehicle - θ_trajectory`
4. 计算误差变化率（微分）
5. 填充 `SimpleLateralDebug` 用于调试输出

### 3.6 Look-Ahead/Look-Back 控制

```cpp
bool enable_look_ahead_back_control_ = false;
double lookahead_station_low_speed_ = 0.0;
double lookback_station_low_speed_ = 0.0;
double lookahead_station_high_speed_ = 0.0;
double lookback_station_high_speed_ = 0.0;
```

- **前瞻控制**：不使用当前位置的误差，而是使用前方某点的预测误差
- 低速时使用 `lookahead_station_low_speed_`，高速时使用 `lookahead_station_high_speed_`
- 倒车时使用 `lookback_station_` 参数
- 改善低速和高速场景的跟踪性能

### 3.7 增益调度

```cpp
std::unique_ptr<Interpolation1D> lat_err_interpolation_;
std::unique_ptr<Interpolation1D> heading_err_interpolation_;
```

- 根据车速插值调整 `matrix_q_` 中横向误差和航向误差的权重
- 低速时更重视横向误差，高速时更重视航向误差
- 使用 `Interpolation1D` 做一维插值

## 4. LonController — PID 纵向控制器

### 4.1 类声明

```cpp
class LonController : public ControlTask {
 public:
  LonController();
  virtual ~LonController();
  common::Status Init(std::shared_ptr<DependencyInjector> injector) override;
  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate* localization,
      const canbus::Chassis* chassis,
      const planning::ADCTrajectory* trajectory,
      control::ControlCommand* cmd) override;
  common::Status Reset() override;
  void Stop() override;
  std::string Name() const override;
};
```

### 4.2 双环 PID 架构

LonController 采用**级联 PID** 控制结构：

```
位置误差 ──> Station PID ──> 速度参考 ──> Speed PID ──> 加速度指令
                                                         │
                                                   标定表插值
                                                         │
                                                    油门/刹车
```

- **外环（Station PID）**：位置误差 → 速度参考
- **内环（Speed PID）**：速度误差 → 加速度指令
- **标定表**：加速度 → 油门/刹车值

### 4.3 核心成员

```cpp
PIDController speed_pid_controller_;      // 速度 PID
PIDController station_pid_controller_;    // 位置 PID
LeadlagController speed_leadlag_controller_;    // 速度 Lead-Lag
LeadlagController station_leadlag_controller_;  // 位置 Lead-Lag
std::unique_ptr<Interpolation2D> control_interpolation_;  // 2D 标定表
```

### 4.4 `Init` — 初始化

1. 加载 `LonBasedPidControllerConf` 配置
2. 初始化 `speed_pid_controller_` 和 `station_pid_controller_` 的 PID 参数
3. 初始化 Lead-Lag 控制器参数
4. `InitControlCalibrationTable`：加载 (速度, 加速度) → 油门/刹车 2D 标定表
5. `SetDigitalFilterPitchAngle`：初始化坡度角滤波器

### 4.5 `ComputeControlCommand` — 核心计算

**Step 1：轨迹分析**

```cpp
ComputeLongitudinalErrors(trajectory_analyzer, preview_time, ts, &debug);
```

- 计算位置误差 `station_error = s_ref - s_actual`
- 计算速度误差 `speed_error = v_ref - v_actual`
- 支持前瞻（preview）补偿

**Step 2：外环 — 位置 PID**

```cpp
double station_pid_output = station_pid_controller_.Control(station_error, ts);
```

- 输入：位置误差
- 输出：速度参考增量
- `reference_spd_cmd_ = reference_spd_ + station_pid_output`

**Step 3：内环 — 速度 PID**

```cpp
double speed_pid_output = speed_pid_controller_.Control(speed_error, ts);
```

- 输入：速度误差
- 输出：加速度闭环修正量

**Step 4：加速度合成**

```cpp
double acceleration_cmd = speed_pid_output + preview_acceleration_reference;
```

- 总加速度 = PID 闭环修正 + 前瞻加速度前馈

**Step 5：Lead-Lag 补偿**

```cpp
if (enable_speed_leadlag_) {
  acceleration_cmd = speed_leadlag_controller_.Control(acceleration_cmd, ts);
}
```

**Step 6：标定表插值**

```cpp
double calibration_value = control_interpolation_->Interpolate(
    std::make_pair(reference_spd_, acceleration_cmd));
```

- 从 2D 标定表中查表：`(当前速度, 目标加速度) → 油门/刹车值`
- 正值 → 油门，负值 → 刹车

**Step 7：停车逻辑**

- `IsStopByDestination`：到达目的地停车
- `IsPedestrianStopLongTerm`：行人导致的长时间停车
- `IsFullStopLongTerm`：完全停车状态
- `SetParkingBrake`：长时间停车时启用电子驻车（EPB）

**Step 8：输出**

```cpp
cmd->set_throttle(calibration_value > 0 ? calibration_value : 0.0);
cmd->set_brake(calibration_value < 0 ? -calibration_value : 0.0);
```

### 4.6 `ComputeLongitudinalErrors` — 纵向误差计算

```cpp
void LonController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer* trajectory, double preview_time, double ts,
    SimpleLongitudinalDebug* debug);
```

1. 在轨迹上找到匹配点
2. 计算纵向位置误差 `station_error`
3. 计算速度误差 `speed_error`
4. 若启用 preview：在 `preview_time` 处计算前瞻误差
5. 填充 `SimpleLongitudinalDebug`

### 4.7 CheckPit 坑检测

```cpp
class CheckPit {
 public:
  static bool CheckInPit(SimpleLongitudinalDebug* debug,
                         const LonBasedPidControllerConf* conf,
                         double speed, bool replan);
};
```

- 检测车辆是否陷入"坑"状态（长时间低速或停车后无法正常起步）
- 用于触发特殊恢复逻辑

### 4.8 停车状态管理

```cpp
bool IsStopByDestination(SimpleLongitudinalDebug* debug);     // 目的地停车
bool IsPedestrianStopLongTerm(SimpleLongitudinalDebug* debug); // 行人长时间停车
bool IsFullStopLongTerm(SimpleLongitudinalDebug* debug);       // 完全停车
void SetParkingBrake(const LonBasedPidControllerConf* conf,
                     control::ControlCommand* control_command); // 电子驻车
```

- **目的地停车**：检测到规划轨迹的停车原因码为 `DESTINATION`
- **行人停车**：行人导致停车超过阈值时间后触发 EPB
- **完全停车**：速度为 0 且持续时间超过阈值
- **电子驻车**：长时间停车时自动启用 EPB，起步时释放

## 5. 控制器协作

### 5.1 串行执行

`ControlTaskAgent` 按 `pipeline.pb.txt` 配置的顺序串行调用控制器：

```protobuf
# conf/pipeline.pb.txt 示例
task {
  name: "lat_controller"
  type: "LatController"
}
task {
  name: "lon_controller"
  type: "LonController"
}
```

### 5.2 共享输入

两个控制器共享相同的输入：

- `LocalizationEstimate`：自车位置和姿态
- `Chassis`：车速、加速度、方向盘角度
- `ADCTrajectory`：规划轨迹

### 5.3 输出合并

- LatController 设置 `cmd->steering_target`
- LonController 设置 `cmd->throttle` 和 `cmd->brake`
- 最终 `ControlCommand` 包含完整的控制指令

## 6. 关键设计决策

### 6.1 LQR vs PID

| 特性 | LatController (LQR) | LonController (PID) |
|------|--------------------|--------------------|
| 控制维度 | 横向（方向盘） | 纵向（油门/刹车） |
| 算法 | 线性二次调节器 | 级联 PID + 标定表 |
| 模型依赖 | 自行车模型（需精确车辆参数） | 经验标定表 |
| 前馈 | 曲率前馈 | 加速度前馈（preview） |
| 自适应 | 增益调度 + MRAC | Lead-Lag 补偿 |

### 6.2 Preview 控制

- 横向：`preview_window_` 个控制周期的前瞻状态叠加
- 纵向：`preview_time` 的前瞻加速度前馈
- 改善高速跟踪性能，减少相位滞后

### 6.3 多层滤波

- 数字滤波器（`DigitalFilter`）：平滑传感器噪声
- 均值滤波器（`MeanFilter`）：平滑横向/航向误差
- 坡度角滤波器：平滑坡度估计

### 6.4 安全保护

- 方向盘转角限幅：`steer_single_direction_max_degree_`
- 横向加速度限制：`max_lat_acc_`
- 最小速度保护：`minimum_speed_protection_`（防止除零）
- 电子驻车：长时间停车自动启用

---

title: "Controller Base Common 控制算法基础组件函数级源码解析"
---

# Controller Base Common 控制算法基础组件函数级源码解析

本文聚焦 `modules/control/control_component/controller_task_base/common/` 目录，按函数级粒度拆解控制模块的 **10 个基础算法组件**：PID 控制器族（标准/BC/IC）、Lead-Lag 补偿器、MRAC 自适应控制器、迟滞滤波器、插值器（1D/2D）、轨迹分析器、依赖注入器。

## 1. 模块定位

`controller_task_base/common/` 是控制模块的**算法积木库**，为横向控制器（LatController）和纵向控制器（LonController）提供标准化的控制算法和工具类。

```
LatController ──┬── PIDController / LeadlagController / MracController
                ├── Interpolation1D（增益调度）
                ├── TrajectoryAnalyzer（轨迹查询）
                └── HysteresisFilter（防抖）

LonController ──┬── PIDController / LeadlagController
                ├── Interpolation2D（标定表）
                ├── TrajectoryAnalyzer
                └── DependencyInjector（状态共享）
```

## 2. PID 控制器族

### 2.1 PIDController — 标准 PID 基类

```cpp
class PIDController {
 public:
  void Init(const PidConf& pid_conf);
  void SetPID(const PidConf& pid_conf);
  void Reset();
  void Reset_integral();
  virtual double Control(const double error, const double dt);
  int IntegratorSaturationStatus() const;
  bool IntegratorHold() const;
  void SetIntegratorHold(bool hold);
};
```

**控制律**：

```
output = kp * error + ki * integral + kd * derivative
```

**成员变量**：

| 变量 | 说明 |
|------|------|
| `kp_`, `ki_`, `kd_` | PID 增益 |
| `kaw_` | Anti-windup 增益（回算补偿用） |
| `previous_error_` | 上一帧误差（用于微分） |
| `integral_` | 积分累积器 |
| `integrator_saturation_high_/low_` | 积分饱和限幅 |
| `integrator_hold_` | 积分冻结标志 |
| `output_saturation_high_/low_` | 输出饱和限幅（子类使用） |

**`Control(error, dt)` 算法**：

1. 积分累加：`integral_ += error * dt`
2. 积分限幅：`clamp(integral_, saturation_low, saturation_high)`
3. 微分计算：`derivative = (error - previous_error) / dt`
4. 输出计算：`output = kp * error + ki * integral + kd * derivative`
5. 更新 `previous_error_`

**积分冻结**：`SetIntegratorHold(true)` 时停止积分累加，用于停车等场景。

### 2.2 PIDBCController — 回算补偿 Anti-Windup

```cpp
class PIDBCController : public PIDController {
  double Control(const double error, const double dt) override;
  int OutputSaturationStatus();
};
```

**回算补偿（Back-Calculation）算法**：

1. 计算未饱和输出 `u_unsat`
2. 饱和限幅：`u_sat = clamp(u_unsat, low, high)`
3. 回算修正积分：`integral_ += kaw * (u_sat - u_unsat) * dt`
4. 返回 `u_sat`

**原理**：当输出饱和时，饱和差值 `(u_sat - u_unsat)` 通过 `kaw` 增益反馈到积分器，防止积分持续增长（windup）。

### 2.3 PIDICController — 积分限幅 Anti-Windup

```cpp
class PIDICController : public PIDController {
  double Control(const double error, const double dt) override;
  int OutputSaturationStatus();
};
```

**积分限幅（Integral Clamping）算法**：

1. 计算输出 `u = kp * error + ki * integral + kd * derivative`
2. 若输出饱和且积分会加剧饱和 → 冻结积分器
3. 否则正常累加积分

**与 BC 的区别**：BC 通过回算主动修正积分值，IC 直接冻结积分器。IC 更简单但响应可能稍慢。

### 2.4 三种 PID 对比

| 特性 | PIDController | PIDBCController | PIDICController |
|------|--------------|----------------|-----------------|
| Anti-Windup | 无（基类） | 回算补偿 | 积分限幅 |
| 积分修正 | 无 | `kaw * (u_sat - u_unsat)` | 冻结积分器 |
| 适用场景 | 无饱和风险 | 执行器饱和频繁 | 简单饱和处理 |
| 复杂度 | 低 | 中 | 低 |

## 3. LeadlagController — Lead-Lag 补偿器

```cpp
class LeadlagController {
 public:
  void Init(const LeadlagConf& leadlag_conf, double dt);
  void SetLeadlag(const LeadlagConf& leadlag_conf);
  void TransformC2d(double dt);
  void Reset();
  virtual double Control(double error, double dt);
  int InnerstateSaturationStatus() const;
};
```

**传递函数**：

```
G(s) = (alpha * s + beta) / (s + tau)
```

- `alpha > 0, beta > 0`：超前补偿（Lead），改善相位裕度
- `alpha < 0, beta < 0`：滞后补偿（Lag），改善稳态精度

**连续→离散转换**：使用 **双线性变换（Tustin/梯形积分）**

```
s = (2/T) * (z-1)/(z+1)
```

**离散化系数**：

```
kn1 = alpha * (2/T) + beta
kn0 = -alpha * (2/T) + beta
kd1 = 2/T + tau
kd0 = -2/T + tau
```

**Direct Form II 实现**：

```
innerstate = error * kn1/ kd1 - previous_innerstate * kd0/kd1
output = innerstate + previous_output (需要调整)
```

**内部状态饱和**：`innerstate_saturation_high_/low_` 限制内部状态，防止溢出。

## 4. MracController — 模型参考自适应控制器

```cpp
class MracController {
 public:
  void Init(const MracConf&, const LatencyParam&, double dt);
  Status SetReferenceModel(const MracConf&);
  Status SetAdaptionModel(const MracConf&);
  Status BuildReferenceModel();
  Status BuildAdaptionModel();
  bool CheckLyapunovPD(MatrixXd A, MatrixXd P) const;
  void EstimateInitialGains(const LatencyParam&);
  void UpdateReference();
  void UpdateAdaption(MatrixXd* law_adp, MatrixXd state_adp, MatrixXd gain_adp);
  void AntiWindupCompensation(double command, double previous_command);
  virtual double Control(double command, MatrixXd state, double input_limit, double input_rate_limit);
  int BoundOutput(double output_unbounded, double previous_output, double* output_bounded);
  void Reset();
  void ResetStates();
  void ResetGains();
};
```

### 4.1 参考模型

支持 1 阶和 2 阶参考模型：

**1 阶**：`G(s) = 1 / (tau * s + 1)`

**2 阶**：`G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)`

- `tau`：时间常数
- `wn`：自然频率
- `zeta`：阻尼比

通过双线性变换离散化为状态空间形式。

### 4.2 自适应律

三类自适应增益：

| 增益 | 变量 | 说明 |
|------|------|------|
| 状态自适应 | `gain_state_adaption_` | 补偿状态偏差 |
| 输入自适应 | `gain_input_adaption_` | 补偿输入偏差 |
| 非线性自适应 | `gain_nonlinear_adaption_` | 补偿非线性特性 |

自适应律基于 **Lyapunov 稳定性理论**：

- `CheckLyapunovPD`：验证代数 Lyapunov 方程的解是否对称正定
- `UpdateAdaption`：每步迭代更新自适应增益

### 4.3 Anti-Windup 补偿

```cpp
void AntiWindupCompensation(double control_command, double previous_command);
```

- 当执行器饱和时，计算补偿量防止自适应增益持续增长
- `gain_anti_windup_`：Anti-Windup 增益矩阵

### 4.4 输出限幅

```cpp
int BoundOutput(double output_unbounded, double previous_output, double* output_bounded);
```

- 值限幅：`bound_command_`
- 变化率限幅：`bound_command_rate_`
- 返回饱和状态码

### 4.5 初始化

```cpp
void EstimateInitialGains(const LatencyParam&);
```

- 从已知的执行器动力学延迟估计初始自适应增益
- 加速自适应收敛

## 5. HysteresisFilter — 迟滞滤波器

```cpp
class HysteresisFilter {
 public:
  void filter(double input_value, double threshold,
              double hysteresis_upper, double hysteresis_lower,
              int* state, double* output_value);
};
```

**算法**：

```
if previous_state == 0:
    if input > threshold + hysteresis_upper:
        state = 1
    else:
        state = 0
else:  // previous_state == 1
    if input < threshold - hysteresis_lower:
        state = 0
    else:
        state = 1
```

**应用场景**：

- 变道决策中的障碍物检测防抖
- 停车/起步判断的阈值切换
- 任何需要消除传感器噪声导致的快速切换

**设计要点**：上阈值和下阈值不对称，允许根据场景调整灵敏度。

## 6. Interpolation1D — 一维插值器

```cpp
class Interpolation1D {
 public:
  using DataType = vector<pair<double, double>>;
  bool Init(const DataType& xy);
  double Interpolate(double x) const;
};
```

**实现**：

- 使用 **Eigen 样条库**（`Eigen::Spline<double, 1>`）
- 输入 x 值归一化到 `[0, 1]` 后拟合样条
- 超出范围时 clamp 到端点值 `y_start_` / `y_end_`

**应用场景**：

- LatController 的增益调度：根据车速插值调整 `matrix_q_` 权重
- 速度→前瞻距离插值

## 7. Interpolation2D — 二维插值器

```cpp
class Interpolation2D {
 public:
  using DataType = vector<tuple<double, double, double>>;
  using KeyType = pair<double, double>;
  bool Init(const DataType& xyz);
  double Interpolate(const KeyType& xy) const;
  bool CheckMap() const;
};
```

**实现**：

- 数据结构：`map<double, map<double, double>>`（嵌套 map）
- **双线性插值**：
  1. 找到 x 方向的两个边界值 `x_low`, `x_high`
  2. 对每个 x 边界，在 y 方向做线性插值
  3. 在 x 方向对两个插值结果做线性插值

**应用场景**：

- LonController 的标定表：`(速度, 加速度) → 油门/刹车值`
- 任何需要 2D 查表的场景

## 8. TrajectoryAnalyzer — 轨迹分析器

```cpp
class TrajectoryAnalyzer {
 public:
  TrajectoryAnalyzer(const planning::ADCTrajectory*);
  unsigned int seq_num();
  TrajectoryPoint QueryNearestPointByAbsoluteTime(double t) const;
  TrajectoryPoint QueryNearestPointByRelativeTime(double t) const;
  TrajectoryPoint QueryNearestPointByPosition(double x, double y) const;
  PathPoint QueryMatchedPathPoint(double x, double y) const;
  void ToTrajectoryFrame(double x, double y, double theta, double v,
                         const PathPoint& matched_point,
                         double* ptr_s, double* ptr_s_dot,
                         double* ptr_d, double* ptr_d_dot) const;
  void TrajectoryTransformToCOM(double rear_to_com_distance);
  Vec2d ComputeCOMPosition(double rear_to_com_distance, const PathPoint&) const;
  const vector<TrajectoryPoint>& trajectory_points() const;
};
```

### 8.1 轨迹点查询

| 方法 | 查询方式 | 算法 |
|------|---------|------|
| `QueryNearestPointByAbsoluteTime` | 绝对时间最近 | 遍历 + 比较时间戳 |
| `QueryNearestPointByRelativeTime` | 相对时间最近 | 遍历 + 比较相对时间 |
| `QueryNearestPointByPosition` | 空间距离最近 | 遍历 + 欧氏距离 |
| `QueryMatchedPathPoint` | 路径上最近点（可插值） | 遍历 + `FindMinDistancePoint` |

### 8.2 `ToTrajectoryFrame` — Frenet 坐标转换

```cpp
void ToTrajectoryFrame(double x, double y, double theta, double v,
                       const PathPoint& matched_point,
                       double* ptr_s, double* ptr_s_dot,
                       double* ptr_d, double* ptr_d_dot) const;
```

- 将全局坐标 `(x, y, theta, v)` 转换为轨迹 Frenet 坐标
- 输出：
  - `s`：纵向距离（沿轨迹）
  - `s_dot`：纵向速度
  - `d`：横向偏移（轨迹法向）
  - `d_dot`：横向速度

### 8.3 `TrajectoryTransformToCOM` — 后轴→质心转换

```cpp
void TrajectoryTransformToCOM(double rear_to_com_distance);
```

- 将轨迹点从后轴坐标系转换到质心坐标系
- 沿轨迹航向偏移 `rear_to_com_distance`
- `ComputeCOMPosition`：计算单个路径点的质心位置

## 9. DependencyInjector — 依赖注入器

```cpp
class DependencyInjector {
 public:
  VehicleStateProvider* vehicle_state();
  void Set_pervious_control_command(ControlCommand* control_command);
  const SimpleLongitudinalDebug* Get_previous_lon_debug_info() const;
  void set_planning_command_status(const CommandStatus&);
  const CommandStatus* get_planning_command_status() const;
  ControlCommand* previous_control_command_mutable();
  const ControlCommand& previous_control_command() const;
  ControlDebugInfo* mutable_control_debug_info();
  const ControlDebugInfo& control_debug_info() const;
  void control_debug_info_clear();
  void set_control_process(bool);
  bool control_process() const;
  ControlInteractiveMsg* mutable_control_interactive_info();
  const ControlInteractiveMsg& control_interactive_info() const;
};
```

**职责**：中央服务定位器，解耦控制模块各组件间的依赖。

**存储内容**：

| 成员 | 说明 |
|------|------|
| `vehicle_state_` | 车辆状态提供者 |
| `control_command_` | 上一帧控制指令 |
| `lon_debug_` | 上一帧纵向调试信息 |
| `control_debug_info_` | 当前帧调试信息 |
| `control_debug_previous_` | 上一帧调试信息 |
| `planning_command_status_` | 规划命令状态 |
| `control_interactive_msg_` | 交互消息 |
| `control_process_` | 控制处理标志 |

## 10. 组件协作关系

```
ControlComponent
    │
    ├── DependencyInjector (共享状态)
    │       ├── vehicle_state
    │       ├── previous_control_command
    │       └── control_debug_info
    │
    ├── PreprocessorSubmodule
    │       └── TrajectoryAnalyzer (轨迹查询)
    │
    ├── LatController
    │       ├── TrajectoryAnalyzer (Frenet 转换)
    │       ├── Interpolation1D (增益调度)
    │       ├── LeadlagController (相位补偿)
    │       ├── MracController (自适应控制)
    │       └── HysteresisFilter (防抖)
    │
    ├── LonController
    │       ├── PIDController (速度/位置 PID)
    │       ├── LeadlagController (速度补偿)
    │       ├── Interpolation2D (标定表)
    │       └── TrajectoryAnalyzer (纵向误差)
    │
    └── PostprocessorSubmodule
            └── DependencyInjector (调试输出)
```

## 11. 算法选型指南

| 场景 | 推荐算法 | 原因 |
|------|---------|------|
| 纵向速度控制 | PID + Lead-Lag | 简单可靠，Lead-Lag 改善响应 |
| 横向转向控制 | LQR + Lead-Lag + MRAC | LQR 理论最优，MRAC 补偿模型不确定性 |
| 执行器饱和 | PIDBC / PIDIC | BC 响应快，IC 实现简单 |
| 增益调度 | Interpolation1D/2D | 根据工况平滑切换增益 |
| 传感器噪声 | HysteresisFilter | 消除阈值附近的快速切换 |
| 轨迹跟踪 | TrajectoryAnalyzer | 统一的时间/空间/Frenet 查询接口 |
