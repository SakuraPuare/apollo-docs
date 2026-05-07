# Planning Math 规划数学库函数级源码解析

本文聚焦 `modules/planning/planning_base/math/` 目录，按函数级粒度拆解规划模块的 **36 个数学工具类**：1D 曲线族（Curve1d）、平滑样条（Smoothing Spline）、约束检查器（Constraint Checker）、分段 Jerk 优化（Piecewise Jerk）、离散点平滑（Discretized Points Smoothing）、以及顶层数学工具。

## 1. 模块定位

`planning_base/math/` 是规划模块的**数学引擎库**，为路径规划、速度规划、轨迹生成提供底层数学工具。

```
路径规划 ──> SmoothingSpline (参考线平滑)
          ──> PiecewiseJerkPathProblem (路径优化)
          ──> FemPosDeviationSmoother (离散点平滑)

速度规划 ──> PiecewiseJerkSpeedProblem (速度优化)

轨迹生成 ──> Curve1d (多项式曲线)
          ──> QuinticPolynomialCurve1d (Lattice 1D 轨迹)

约束验证 ──> ConstraintChecker / ConstraintChecker1d

几何计算 ──> CurveMath / DiscretePointsMath
```

## 2. Curve1d — 1D 曲线族

### 2.1 类层次

```
Curve1d (抽象基类)
  └── PolynomialCurve1d (多项式抽象)
        ├── CubicPolynomialCurve1d (3次)
        ├── QuarticPolynomialCurve1d (4次)
        └── QuinticPolynomialCurve1d (5次)
              └── QuinticSpiralPath (螺旋线)
  └── PiecewiseQuinticSpiralPath (分段螺旋线)
```

### 2.2 Curve1d — 抽象基类

```cpp
class Curve1d {
 public:
  virtual double Evaluate(uint32_t order, double param) const = 0;
  virtual double ParamLength() const = 0;
  virtual string ToString() const = 0;
};
```

- `Evaluate(order, param)`：计算曲线在参数 `param` 处的 `order` 阶导数
  - order=0：位置
  - order=1：速度
  - order=2：加速度
  - order=3：jerk
- `ParamLength()`：参数范围长度

### 2.3 PolynomialCurve1d — 多项式抽象

```cpp
class PolynomialCurve1d : public Curve1d {
 public:
  virtual double Coef(size_t order) const = 0;
  virtual size_t Order() const = 0;
 protected:
  double param_ = 0.0;
};
```

- `Coef(order)`：获取 `order` 阶系数
- `param_`：曲线参数长度（通常是弧长或时间）

### 2.4 CubicPolynomialCurve1d — 三次多项式

```cpp
class CubicPolynomialCurve1d : public PolynomialCurve1d {
 public:
  CubicPolynomialCurve1d(double x0, double dx0, double ddx0, double x1, double param);
  void DerivedFromQuarticCurve(const QuarticPolynomialCurve1d& other);
};
```

- 4 个系数：`a0, a1, a2, a3`
- 构造条件：初态 `(x0, dx0, ddx0)` + 终态 `(x1)`
- `DerivedFromQuarticCurve`：对四次多项式求导得到三次

### 2.5 QuarticPolynomialCurve1d — 四次多项式

```cpp
class QuarticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuarticPolynomialCurve1d(array<double,3> start, array<double,2> end, double param);
  void FitWithEndPointFirstOrder(double x0, double dx0, double x1, double dx1, double param);
  void FitWithEndPointSecondOrder(double x0, double dx0, double x1, double ddx1, double param);
  void IntegratedFromCubicCurve(const CubicPolynomialCurve1d& other, double intercept);
  void DerivedFromQuinticCurve(const QuinticPolynomialCurve1d& other);
};
```

- 5 个系数：`a0, a1, a2, a3, a4`
- 构造条件：初态 `(x0, dx0, ddx0)` + 终态 `(dx1, ddx1)`
- **Lattice Planner 纵向巡航**使用：给定初态速度/加速度 + 终态速度/加速度

### 2.6 QuinticPolynomialCurve1d — 五次多项式

```cpp
class QuinticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuinticPolynomialCurve1d(array<double,3> start, array<double,3> end, double param);
  void IntegratedFromQuarticCurve(const QuarticPolynomialCurve1d& other, double intercept);
};
```

- 6 个系数：`a0, a1, a2, a3, a4, a5`
- 构造条件：初态 `(x0, dx0, ddx0)` + 终态 `(x1, dx1, ddx1)`
- **Lattice Planner 横向规划和纵向停车**使用

### 2.7 QuinticSpiralPath — 五次螺旋线

```cpp
class QuinticSpiralPath : public QuinticPolynomialCurve1d {
 public:
  QuinticSpiralPath(double theta0, double kappa0, double dkappa0,
                    double theta1, double kappa1, double dkappa1, double delta_s);
  template <size_t N> double ComputeCartesianDeviationX(double s) const;
  template <size_t N> double ComputeCartesianDeviationY(double s) const;
  template <size_t N> array<double, 7> DeriveCartesianDeviation(size_t param_index) const;
};
```

- 7 个设计参数：`(theta0, kappa0, dkappa0, theta1, kappa1, dkappa1, delta_s)`
- 曲率随弧长线性变化的 clothoid 曲线
- `ComputeCartesianDeviationX/Y<N>`：使用 Gauss-Legendre 积分计算笛卡尔偏差
- `DeriveCartesianDeviation`：对 7 个设计参数的解析导数

### 2.8 PiecewiseQuinticSpiralPath — 分段五次螺旋线

```cpp
class PiecewiseQuinticSpiralPath : public Curve1d {
 public:
  PiecewiseQuinticSpiralPath(double theta, double kappa, double dkappa);
  void Append(double theta, double kappa, double dkappa, double delta_s);
  double Evaluate(uint32_t order, double param) const override;
  double DeriveKappaS(double s) const;
};
```

- 链式拼接多个 `QuinticSpiralPath` 段
- `Append`：追加新的螺旋段
- `DeriveKappaS`：计算曲率对弧长的导数

## 3. Smoothing Spline — 平滑样条

### 3.1 样条段

#### Spline1dSeg — 1D 样条段

```cpp
class Spline1dSeg {
  Spline1dSeg(uint32_t order);
  double operator()(double x) const;
  double Derivative(double x) const;
  double SecondOrderDerivative(double x) const;
  double ThirdOrderDerivative(double x) const;
};
```

- 单段多项式，预计算 1/2/3 阶导数多项式

#### Spline2dSeg — 2D 样条段

```cpp
class Spline2dSeg {
  pair<double,double> operator()(double t) const;
  double x(double t) const;
  double y(double t) const;
};
```

- `(x(t), y(t))` 参数化 2D 曲线段

### 3.2 样条组合

#### Spline1d — 1D 分段样条

```cpp
class Spline1d {
  Spline1d(vector<double> x_knots, uint32_t order);
  double operator()(double x) const;
  double Derivative(double x) const;
  void SetSplineSegs(const MatrixXd& coeffs, uint32_t order);
};
```

- 由 `Spline1dSeg` 在 knot 点拼接而成
- 用于参考线平滑、速度剖面优化

#### Spline2d — 2D 分段样条

```cpp
class Spline2d {
  Spline2d(vector<double> t_knots, uint32_t order);
  pair<double,double> operator()(double t) const;
  double x(double t) const;
  double y(double t) const;
};
```

- 由 `Spline2dSeg` 在 knot 点拼接而成
- 用于参考线平滑（QpSplineReferenceLineSmoother）

### 3.3 核矩阵

#### SplineSegKernel — 核矩阵生成器（单例）

```cpp
class SplineSegKernel {
  DECLARE_SINGLETON(SplineSegKernel);
  MatrixXd Kernel(uint32_t num_params, double accumulated_x);
  MatrixXd NthDerivativeKernel(uint32_t n, uint32_t num_params, double accumulated_x);
};
```

- 生成积分核矩阵 P，使得 `x' * P * x = ∫(f^(k)(x))² dx`
- k=0：位置平滑
- k=1：速度平滑
- k=2：加速度平滑
- k=3：jerk 平滑

#### Spline1dKernel / Spline2dKernel — 代价矩阵构建器

```cpp
class Spline1dKernel {
  void AddRegularization(double);
  void AddDerivativeKernelMatrix(double weight);
  void AddSecondOrderDerivativeMatrix(double weight);
  void AddThirdOrderDerivativeMatrix(double weight);
  void AddReferenceLineKernelMatrix(vector<double> x_coord, vector<double> ref_fx, double weight);
  MatrixXd kernel_matrix() const;
  MatrixXd offset() const;
};
```

- 构建 QP 问题的二次代价矩阵 H 和偏移向量 f
- `AddDerivativeKernelMatrix`：添加平滑性惩罚
- `AddReferenceLineKernelMatrix`：添加参考线跟踪惩罚

### 3.4 约束构建器

#### AffineConstraint — 仿射约束

```cpp
class AffineConstraint {
  AffineConstraint(bool is_equality);
  void AddConstraint(const MatrixXd& constraint_matrix, const MatrixXd& constraint_boundary);
  MatrixXd constraint_matrix() const;
  MatrixXd constraint_boundary() const;
};
```

- 表示 `A * x ≤ b`（不等式）或 `A * x = b`（等式）

#### Spline1dConstraint / Spline2dConstraint — 样条约束

```cpp
class Spline1dConstraint {
  void AddBoundary(vector<double> x_coord, vector<double> lower, vector<double> upper);
  void AddDerivativeBoundary(...);
  void AddSecondDerivativeBoundary(...);
  void AddThirdDerivativeBoundary(...);
  void AddPointConstraint(double x, double fx);
  void AddSmoothConstraint();
  void AddDerivativeSmoothConstraint();
  void AddSecondDerivativeSmoothConstraint();
  void AddMonotoneInequalityConstraint(vector<double> x_coord, double angle);
  AffineConstraint inequality_constraint() const;
  AffineConstraint equality_constraint() const;
};
```

- `AddBoundary`：添加值/导数边界约束
- `AddSmoothConstraint`：添加 knot 点连续性约束
- `AddMonotoneInequalityConstraint`：添加单调性约束

### 3.5 求解器

#### Spline1dSolver / Spline2dSolver — 抽象求解器

```cpp
class Spline1dSolver {
  virtual bool Solve() = 0;
  virtual void Reset(vector<double> x_knots, uint32_t order);
  Spline1dConstraint* mutable_spline_constraint();
  Spline1dKernel* mutable_spline_kernel();
  const Spline1d& spline() const;
};
```

#### OsqpSpline1dSolver / OsqpSpline2dSolver — OSQP 求解器

```cpp
class OsqpSpline1dSolver : public Spline1dSolver {
  bool Solve() override;
};
```

- 使用 OSQP 库求解二次规划问题
- 输入：kernel_matrix + offset + constraints
- 输出：最优样条系数

## 4. Constraint Checker — 约束检查器

### 4.1 ConstraintChecker — 轨迹约束检查

```cpp
class ConstraintChecker {
  enum Result {
    VALID,
    LON_VELOCITY_OUT_OF_BOUND,
    LON_ACCELERATION_OUT_OF_BOUND,
    LON_JERK_OUT_OF_BOUND,
    CURVATURE_OUT_OF_BOUND,
    LAT_ACCELERATION_OUT_OF_BOUND,
    LAT_JERK_OUT_OF_BOUND
  };
  static Result ValidTrajectory(const DiscretizedTrajectory&);
};
```

- 静态工具类，检查离散轨迹的动力学可行性
- 检查项：纵向速度/加速度/jerk、曲率、横向加速度/jerk
- 被 Lattice Planner 的轨迹评估步骤调用

### 4.2 ConstraintChecker1d — 1D 约束检查

```cpp
class ConstraintChecker1d {
  static bool IsValidLongitudinalTrajectory(const Curve1d&);
  static bool IsValidLateralTrajectory(const Curve1d& lat, const Curve1d& lon);
};
```

- 检查 1D 曲线的运动学可行性
- 被 Lattice Planner 在 1D 轨迹生成后预筛选

## 5. Piecewise Jerk — 分段 Jerk 优化

### 5.1 PiecewiseJerkProblem — 基类

```cpp
class PiecewiseJerkProblem {
  PiecewiseJerkProblem(size_t num_of_knots, double delta_s, array<double,3> x_init);
  void set_x_bounds(vector<pair<double,double>>);
  void set_dx_bounds(vector<pair<double,double>>);
  void set_ddx_bounds(vector<pair<double,double>>);
  void set_dddx_bound(double dddx_bound);
  void set_weight_x(double weight);
  void set_weight_dx(double weight);
  void set_weight_ddx(double weight);
  void set_weight_dddx(double weight);
  void set_x_ref(double weight, vector<double> x_ref);
  void set_end_state_ref(array<double,3> weight, array<double,3> end_state);
  virtual bool Optimize(size_t max_iter);
  vector<double> opt_x() const;
  vector<double> opt_dx() const;
  vector<double> opt_ddx() const;
};
```

**优化目标**：

```
min Σ [w_x * (x - x_ref)² + w_dx * dx² + w_ddx * ddx² + w_dddx * dddx²]
```

**约束**：

- `x ∈ [x_lower, x_upper]`
- `dx ∈ [dx_lower, dx_upper]`
- `ddx ∈ [ddx_lower, ddx_upper]`
- `|dddx| ≤ dddx_bound`
- 运动学一致性：`x[k+1] = x[k] + dx[k]*Δt + 0.5*ddx[k]*Δt²`
- 终态约束（可选）

### 5.2 PiecewiseJerkPathProblem — 路径优化

```cpp
class PiecewiseJerkPathProblem : public PiecewiseJerkProblem {
  void set_extra_constraints(ObsCornerConstraints);
  void set_vertex_constraints(ADCVertexConstraints);
};
```

- 用于横向路径优化（`x(s)` = 横向偏移随纵向距离变化）
- 额外约束：障碍物角点约束、自车顶点约束

### 5.3 PiecewiseJerkSpeedProblem — 速度优化

```cpp
class PiecewiseJerkSpeedProblem : public PiecewiseJerkProblem {
  void set_dx_ref(double weight_dx_ref, double dx_ref);
  void set_dx_ref(vector<double> weight_dx_ref, vector<double> dx_ref);
  void set_penalty_dx(vector<double> penalty_dx);
};
```

- 用于速度剖面优化（`x(t)` = 纵向距离随时间变化）
- `set_dx_ref`：设置参考速度（`dx` = 速度）
- `set_penalty_dx`：逐 knot 点的速度偏差惩罚

## 6. Discretized Points Smoothing — 离散点平滑

### 6.1 CosThetaSmoother — Cos-Theta 平滑器

```cpp
class CosThetaSmoother {
  CosThetaSmoother(CosThetaSmootherConfig config);
  bool Solve(const vector<Vec2d>& raw_point2d,
             const vector<pair<double,double>>& bounds,
             vector<double>* opt_x, vector<double>* opt_y);
};
```

- 优化目标：最大化相邻线段的方向一致性（cos-theta 最大化）
- 约束：每个点在边界矩形内
- 用于参考线平滑

### 6.2 FemPosDeviationSmoother — FEM 位置偏差平滑器

```cpp
class FemPosDeviationSmoother {
  FemPosDeviationSmoother(FemPosDeviationSmootherConfig config);
  bool QpWithOsqp(const vector<Vec2d>& raw_point2d, ...);
  bool NlpWithIpopt(const vector<Vec2d>& raw_point2d, ...);
  bool SqpWithOsqp(const vector<Vec2d>& raw_point2d, ...);
  bool Solve(const vector<Vec2d>& raw_point2d, ...,
             const vector<vector<Vec2d>>& point_box);
};
```

**三种求解后端**：

| 方法 | 求解器 | 特点 |
|------|--------|------|
| `QpWithOsqp` | OSQP (QP) | 快速，凸问题 |
| `NlpWithIpopt` | IPOPT (NLP) | 精确，非凸问题 |
| `SqpWithOsqp` | SQP (序列QP) | 平衡速度和精度 |

- `point_box`：可选的多边形障碍物约束（泊车场景）
- 用于离散参考线点的平滑

## 7. 顶层数学工具

### 7.1 PolynomialXd — 通用多项式

```cpp
class PolynomialXd {
  PolynomialXd(uint32_t order);
  PolynomialXd(vector<double> params);
  double operator()(double value) const;
  double operator[](uint32_t index) const;
  void SetParams(vector<double>);
  static PolynomialXd DerivedFrom(const PolynomialXd&);
  static PolynomialXd IntegratedFrom(const PolynomialXd&, double intercept);
  uint32_t order() const;
  vector<double> params() const;
};
```

- 通用 N 次多项式，系数存储为向量
- `DerivedFrom`：求导得到低一次多项式
- `IntegratedFrom`：积分得到高一次多项式

### 7.2 CurveMath — 曲线曲率计算

```cpp
class CurveMath {
  static double ComputeCurvature(double dx, double d2x, double dy, double d2y);
  static double ComputeCurvatureDerivative(double dx, double d2x, double d3x,
                                           double dy, double d2y, double d3y);
};
```

- `ComputeCurvature`：`κ = (dx·d²y - dy·d²x) / (dx² + dy²)^(3/2)`
- `ComputeCurvatureDerivative`：`dκ/ds` 的计算

### 7.3 DiscretePointsMath — 离散点几何计算

```cpp
class DiscretePointsMath {
  static bool ComputePathProfile(const vector<Vec2d>& xy_points,
                                  vector<double>* headings,
                                  vector<double>* accumulated_s,
                                  vector<double>* kappas,
                                  vector<double>* dkappas);
};
```

- 从离散 `(x,y)` 点序列计算：
  - `headings`：航向角
  - `accumulated_s`：累积弧长
  - `kappas`：曲率
  - `dkappas`：曲率变化率

## 8. 组件协作关系

```
参考线平滑:
  SplineSegKernel → Spline1dKernel/Spline2dKernel (代价矩阵)
  Spline1dConstraint/Spline2dConstraint (约束矩阵)
  OsqpSpline1dSolver/OsqpSpline2dSolver (OSQP 求解)
  → Spline1d/Spline2d (平滑样条)

路径优化:
  PiecewiseJerkPathProblem → OSQP → opt_x (横向偏移)

速度优化:
  PiecewiseJerkSpeedProblem → OSQP → opt_x (纵向距离)

Lattice 轨迹生成:
  QuinticPolynomialCurve1d (横向 5 次多项式)
  QuarticPolynomialCurve1d (纵向 4 次多项式)
  → Curve1d 序列 → TrajectoryCombiner

约束验证:
  ConstraintChecker::ValidTrajectory (2D 轨迹)
  ConstraintChecker1d::IsValid (1D 曲线)

参考线点平滑:
  CosThetaSmoother / FemPosDeviationSmoother
  → 平滑后的 (x,y) 点序列
```

## 9. 算法选型指南

| 场景 | 推荐算法 | 原因 |
|------|---------|------|
| 参考线平滑 | OsqpSpline2dSolver | QP 高效，满足实时性 |
| 横向路径优化 | PiecewiseJerkPathProblem | 统一框架，支持障碍物约束 |
| 速度剖面优化 | PiecewiseJerkSpeedProblem | 最小化 jerk，舒适性好 |
| Lattice 横向轨迹 | QuinticPolynomialCurve1d | 6 个自由度，完全约束 |
| Lattice 纵向巡航 | QuarticPolynomialCurve1d | 5 个自由度，不定位移 |
| 离散点平滑（简单） | CosThetaSmoother | 凸问题，快速 |
| 离散点平滑（复杂） | FemPosDeviationSmoother | 支持非凸和障碍物约束 |
| 曲率计算 | CurveMath | 解析公式，精度高 |

## 10. 优化器接口类

### 10.1 CosThetaIpoptInterface

- IPOPT 非线性优化接口，用于 CosThetaSmoother
- 实现 `TNLP` 接口，定义 cos-theta 目标函数和边界约束

### 10.2 FemPosDeviationIpoptInterface

- IPOPT 非线性优化接口，用于 FemPosDeviationSmoother 的 NLP 模式
- 最小化有限元位置偏差

### 10.3 FemPosDeviationOsqpInterface

- OSQP 二次规划接口，用于 FemPosDeviationSmoother 的 QP 模式
- 将位置偏差问题转化为 QP 形式

### 10.4 FemPosDeviationSqpOsqpInterface

- 序列二次规划（SQP）接口，用于 FemPosDeviationSmoother 的 SQP 模式
- 迭代线性化 + OSQP 求解

### 10.5 QuinticSpiralPathWithDerivation

- `QuinticSpiralPath` 的扩展版本，额外提供对设计参数的解析导数
- 用于螺旋线平滑器的梯度优化