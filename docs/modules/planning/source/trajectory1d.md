# Trajectory1d 一维轨迹原语函数级源码解析

本文聚焦 `modules/planning/planning_base/common/trajectory1d/` 目录，按函数级粒度拆解规划模块的 **6 个一维轨迹原语类**。这些类是 Lattice Planner 纵向/横向轨迹生成的基础积木，统一继承自 `Curve1d` 接口。

## 1. 模块定位

`trajectory1d/` 提供一组**参数化一维轨迹**（s(t) 或 d(s)），用于描述车辆在纵向或横向上的运动。每个类封装一种运动学模型，支持按阶数（0=位置, 1=速度, 2=加速度, 3=jerk）求值。

```
Curve1d (math/curve1d/curve1d.h)
    ▲
    │ public 继承
    │
    ├── ConstantJerkTrajectory1d         恒定 jerk 段
    ├── ConstantDecelerationTrajectory1d 恒定减速段
    ├── PiecewiseJerkTrajectory1d        分段恒定 jerk 拼接
    ├── PiecewiseAccelerationTrajectory1d 分段恒定加速度拼接
    ├── PiecewiseTrajectory1d            通用分段 Curve1d 拼接
    └── StandingStillTrajectory1d        静止段
```

**使用场景**：

| 类 | 典型用途 |
|----|---------|
| `ConstantJerkTrajectory1d` | Lattice 纵向采样的基本段 |
| `ConstantDecelerationTrajectory1d` | 紧急制动轨迹 |
| `PiecewiseJerkTrajectory1d` | 多段 jerk 拼接的完整纵向轨迹 |
| `PiecewiseAccelerationTrajectory1d` | 备份轨迹生成器 |
| `PiecewiseTrajectory1d` | 通用多段曲线拼接（横向） |
| `StandingStillTrajectory1d` | 停车等待轨迹 |

## 2. Curve1d 基类接口

```cpp
class Curve1d {
 public:
  virtual double Evaluate(const std::uint32_t order, const double param) const = 0;
  virtual double ParamLength() const = 0;
  virtual std::string ToString() const = 0;
};
```

所有 trajectory1d 类必须实现：
- `Evaluate(order, param)`：按阶数求值，`order=0` 返回位置，`order=1` 返回速度，以此类推
- `ParamLength()`：返回参数域长度（通常是时间 T）
- `ToString()`：调试输出

## 3. ConstantJerkTrajectory1d — 恒定 Jerk 段

源码：`constant_jerk_trajectory1d.h:30` 与 `.cc:29`

### 3.1 类定义

```cpp
class ConstantJerkTrajectory1d : public Curve1d {
 public:
  ConstantJerkTrajectory1d(double p0, double v0, double a0,
                           double jerk, double param);
  double Evaluate(std::uint32_t order, double param) const;
  double ParamLength() const;
  double start_position() const;
  double start_velocity() const;
  double start_acceleration() const;
  double end_position() const;
  double end_velocity() const;
  double end_acceleration() const;
  double jerk() const;
 private:
  double p0_, v0_, a0_;       // 起始状态
  double p1_, v1_, a1_;       // 终止状态（构造时预计算）
  double param_;              // 时间长度
  double jerk_;               // 恒定 jerk 值
};
```

### 3.2 构造函数

```cpp
ConstantJerkTrajectory1d(p0, v0, a0, jerk, param)
```

**前置条件**：`param > FLAGS_numerical_epsilon`

**执行步骤**：
1. 保存初始状态 `p0_, v0_, a0_`
2. 保存参数 `param_` 和 `jerk_`
3. 预计算终止状态：`p1_ = Evaluate(0, param_)`, `v1_ = Evaluate(1, param_)`, `a1_ = Evaluate(2, param_)`

### 3.3 `Evaluate` — 运动学求值

**运动学公式**（恒定 jerk 模型）：

| order | 公式 | 物理含义 |
|-------|------|---------|
| 0 | `p0 + v0*t + 0.5*a0*t² + jerk*t³/6` | 位置 s(t) |
| 1 | `v0 + a0*t + 0.5*jerk*t²` | 速度 v(t) |
| 2 | `a0 + jerk*t` | 加速度 a(t) |
| 3 | `jerk` | jerk（常数） |
| ≥4 | `0.0` | 高阶导数为零 |

## 4. ConstantDecelerationTrajectory1d — 恒定减速段

源码：`constant_deceleration_trajectory1d.h:30` 与 `.cc:31`

### 4.1 类定义

```cpp
class ConstantDecelerationTrajectory1d : public Curve1d {
 public:
  ConstantDecelerationTrajectory1d(double init_s, double init_v, double a);
  double Evaluate(std::uint32_t order, double param) const override;
  double ParamLength() const override;
 private:
  double Evaluate_s(double t) const;
  double Evaluate_v(double t) const;
  double Evaluate_a(double t) const;
  double Evaluate_j(double t) const;
  double init_s_, init_v_, deceleration_;
  double end_t_, end_s_;
};
```

### 4.2 构造函数

```cpp
ConstantDecelerationTrajectory1d(init_s, init_v, a)
```

**执行步骤**：
1. 保存 `init_s_`、取绝对值 `init_v_ = |init_v|`
2. 取反存储：`deceleration_ = -a`（确保 `deceleration_ > 0`）
3. 计算停车时间：`end_t_ = init_v_ / deceleration_`
4. 计算停车位置：`end_s_ = init_v_² / (2 * deceleration_) + init_s_`

### 4.3 `Evaluate` — 带外推处理

**关键设计**：超过 `end_t_` 后自动外推为静止状态。

| order | t < end_t_ | t ≥ end_t_ |
|-------|-----------|-----------|
| 0 (s) | `init_s + (v + init_v) * t * 0.5` | `end_s_`（停在原地） |
| 1 (v) | `init_v - deceleration * t` | `0.0` |
| 2 (a) | `-deceleration` | `0.0` |
| 3 (j) | `0.0` | `0.0` |

### 4.4 `ParamLength`

返回 `end_t_`（从初速度减速到零所需时间）。

## 5. PiecewiseJerkTrajectory1d — 分段恒定 Jerk 拼接

源码：`piecewise_jerk_trajectory1d.h:32` 与 `.cc:31`

### 5.1 类定义

```cpp
class PiecewiseJerkTrajectory1d : public Curve1d {
 public:
  PiecewiseJerkTrajectory1d(double p, double v, double a);
  void AppendSegment(double jerk, double param);
  double Evaluate(std::uint32_t order, double param) const;
  double ParamLength() const;
 private:
  std::vector<ConstantJerkTrajectory1d> segments_;
  double last_p_, last_v_, last_a_;
  std::vector<double> param_;  // 累积参数断点
};
```

### 5.2 构造函数

初始化起始状态 `last_p_`, `last_v_`, `last_a_`，并在 `param_` 中压入 `0.0` 作为起始断点。

### 5.3 `AppendSegment` — 追加一段

```cpp
void AppendSegment(double jerk, double param);
```

**执行步骤**：
1. 累积断点：`param_.push_back(param_.back() + param)`
2. 构造新段：`ConstantJerkTrajectory1d(last_p_, last_v_, last_a_, jerk, param)`
3. 更新末端状态：从新段的 `end_position/velocity/acceleration` 读取

### 5.4 `Evaluate` — 分段查找求值

**算法**：
1. 用 `std::lower_bound` 在 `param_` 中定位 `param` 所在段
2. 计算段内局部参数：`param - param_[index-1]`
3. 调用对应 `ConstantJerkTrajectory1d::Evaluate(order, local_param)`

**边界处理**：
- `param` 在第一个断点之前 → 用第一段求值
- `param` 超过最后一个断点 → 用最后一段外推

## 6. PiecewiseAccelerationTrajectory1d — 分段恒定加速度

源码：`piecewise_acceleration_trajectory1d.h:32` 与 `.cc:35`

### 6.1 类定义

```cpp
class PiecewiseAccelerationTrajectory1d : public Curve1d {
 public:
  PiecewiseAccelerationTrajectory1d(double start_s, double start_v);
  void AppendSegment(double a, double t_duration);
  void PopSegment();
  double Evaluate(std::uint32_t order, double param) const override;
  std::array<double, 4> Evaluate(double t) const;  // 批量求值
  double ParamLength() const override;
 private:
  std::vector<double> s_;  // 累积位置
  std::vector<double> v_;  // 各断点速度
  std::vector<double> t_;  // 累积时间
  std::vector<double> a_;  // 各段加速度
};
```

### 6.2 `AppendSegment` — 追加恒定加速度段

```cpp
void AppendSegment(double a, double t_duration);
```

**执行步骤**：
1. 取末端状态：`s0 = s_.back()`, `v0 = v_.back()`, `t0 = t_.back()`
2. 计算新末端速度：`v1 = v0 + a * t_duration`（断言 `v1 ≥ -ε`）
3. 计算位移：`delta_s = (v0 + v1) * t_duration * 0.5`
4. 压入新断点：`s1`, `v1`, `a`, `t1 = t0 + t_duration`

### 6.3 `PopSegment` — 弹出最后一段

移除 `s_`, `v_`, `a_`, `t_` 各自的最后一个元素。用于回溯搜索。

### 6.4 `Evaluate` — 线性插值求值

**位置求值 `Evaluate_s(t)`**：
1. `lower_bound` 定位时间段
2. 线性插值速度：`v = lerp(v0, t0, v1, t1, t)`
3. 梯形积分位置：`s = (v0 + v) * (t - t0) * 0.5 + s0`

**批量求值 `Evaluate(t) → array<double,4>`**：
一次返回 `{s, v, a, j}`，避免重复查找。

## 7. PiecewiseTrajectory1d — 通用分段曲线拼接

源码：`piecewise_trajectory1d.h:32` 与 `.cc:30`

### 7.1 类定义

```cpp
class PiecewiseTrajectory1d : public Curve1d {
 public:
  void AppendSegment(const std::shared_ptr<Curve1d> trajectory);
  void PopSegment();
  size_t NumOfSegments() const;
  double Evaluate(std::uint32_t order, double param) const;
  double ParamLength() const;
 private:
  std::vector<std::shared_ptr<Curve1d>> trajectory_segments_;
  std::vector<double> accumulated_param_lengths_;
};
```

### 7.2 `AppendSegment` — 带连续性检查的追加

```cpp
void AppendSegment(const std::shared_ptr<Curve1d> trajectory);
```

**执行步骤**：
1. 若非首段，检查与前一段末端的连续性（0~3 阶，阈值 `1e-4`）
2. 不连续时输出 `AWARN`（不阻断，仅告警）
3. 压入 `trajectory_segments_`
4. 累积参数长度：`accumulated_param_lengths_.push_back(prev + new.ParamLength())`

**设计意图**：通用容器，可拼接任意 `Curve1d` 子类（五次多项式、恒定 jerk 段等）。

### 7.3 `Evaluate` — 分段查找

用 `lower_bound` 在 `accumulated_param_lengths_` 中定位段索引，减去前段累积长度得到局部参数，委托给对应段求值。

## 8. StandingStillTrajectory1d — 静止段

源码：`standing_still_trajectory1d.h:30` 与 `.cc:26`

### 8.1 类定义

```cpp
class StandingStillTrajectory1d : public Curve1d {
 public:
  StandingStillTrajectory1d(double p, double duration);
  double Evaluate(std::uint32_t order, double param) const override;
  double ParamLength() const override;
 private:
  double fixed_position_;
  double duration_;
};
```

### 8.2 求值逻辑

| order | 返回值 |
|-------|--------|
| 0 | `fixed_position_`（恒定位置） |
| 1 | `0.0`（零速度） |
| 2 | `0.0`（零加速度） |
| 3 | `0.0`（零 jerk） |

最简单的轨迹原语，用于表示车辆完全静止的时间段。

## 9. 类间协作关系

```
PiecewiseJerkTrajectory1d
    │ 内部持有
    └── vector<ConstantJerkTrajectory1d>
            │ 每段是一个恒定 jerk 段

PiecewiseTrajectory1d
    │ 内部持有
    └── vector<shared_ptr<Curve1d>>
            │ 可以是任意 Curve1d 子类
            ├── ConstantJerkTrajectory1d
            ├── StandingStillTrajectory1d
            └── 五次多项式等

BackupTrajectoryGenerator (lattice)
    │ 使用
    └── PiecewiseAccelerationTrajectory1d
```

## 10. 使用指引

| 场景 | 推荐类 | 原因 |
|------|--------|------|
| Lattice 纵向采样 | `PiecewiseJerkTrajectory1d` | 自然表达多段 jerk 优化结果 |
| 紧急制动 | `ConstantDecelerationTrajectory1d` | 自动处理停车外推 |
| 备份轨迹 | `PiecewiseAccelerationTrajectory1d` | 支持 `PopSegment` 回溯 |
| 横向拼接 | `PiecewiseTrajectory1d` | 通用容器，带连续性检查 |
| 停车等待 | `StandingStillTrajectory1d` | 零开销静止表示 |
