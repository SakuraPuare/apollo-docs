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