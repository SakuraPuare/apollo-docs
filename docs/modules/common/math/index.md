---
title: "数学工具库"
---

# 数学工具库

> 源码路径: `modules/common/math/`

## 概述

`math` 模块为 Apollo 自动驾驶系统提供基础数学工具集，涵盖以下核心能力：

- **2D 几何图元** — Vec2d 向量、LineSegment2d 线段、AABox2d / Box2d 包围盒、Polygon2d 多边形，支持距离计算、重叠检测、凸包生成及 IoU 计算。
- **空间索引** — AABoxKDTree2d 基于轴对齐包围盒的 KD 树，支持最近邻和范围查询。
- **角度与旋转** — Angle 整数角度类（快速查表三角函数）、EulerAnglesZXY 欧拉角、四元数辅助函数。
- **坐标转换** — CartesianFrenetConverter 笛卡尔与 Frenet 坐标系互转。
- **数值方法** — 卡尔曼滤波、多项式拟合、Hermite 样条插值、线性/球面插值、Simpson / 梯形 / Gauss-Legendre 数值积分、黄金分割搜索。
- **控制求解器** — LQR 线性二次调节器、MPC（基于 OSQP）、QP 二次规划基类。
- **矩阵工具** — 伪逆、连续到离散系统转换、CSC 稀疏矩阵格式转换。

## 核心类

### Vec2d / LineSegment2d / AABox2d / Box2d

2D 几何图元层级。`Vec2d` 为基础向量类，`LineSegment2d` 由起终点定义并缓存方向和长度，`AABox2d` 为轴对齐包围盒，`Box2d` 为有方向矩形包围盒（由中心、航向角、长宽定义）。Box2d 可从 AABox2d 或两点构造，提供旋转和重叠检测。全局常量 `kMathEpsilon = 1e-10` 定义数值容差。

### Polygon2d

通用 2D 多边形，可从顶点列表或 Box2d 构造。支持凸包计算（`ComputeConvexHull`）、距离计算、重叠/交集检测、`ComputeIoU`（凸多边形交并比）、`MinAreaBoundingBox`（最小面积包围盒）和 `ExpandByDistance`（膨胀）。

### AABoxKDTree2d

基于轴对齐包围盒的 KD 树，模板参数 `ObjectType` 需提供 `aabox()` 和 `DistanceSquareTo()` 接口。通过 `AABoxKDTreeParams` 控制最大深度、叶节点最大元素数和最大尺寸。提供 `GetNearestObject` 和 `GetObjects` 两种查询。

### Angle

基于整数表示的模板化角度类，利用整数溢出自动归一化。Angle8/Angle16 通过 16K 查表 (`SIN_TABLE`) 实现快速三角函数。Angle32 适用于经纬度（精度 < 1cm）。

```cpp
using Angle8  = Angle<int8_t>;
using Angle16 = Angle<int16_t>;
using Angle32 = Angle<int32_t>;
using Angle64 = Angle<int64_t>;
```

### EulerAnglesZXY / Quaternion 辅助

ZXY 内旋序 Tait-Bryan 欧拉角，车辆参考系为 RFU（右/前/上）。支持从四元数构造和转回四元数。`quaternion.h` 提供 `QuaternionToHeading`、`HeadingToQuaternion`、`QuaternionRotate` 等辅助函数，航向角定义为 0 朝东、PI/2 朝北。

### CartesianFrenetConverter

将笛卡尔坐标系下的车辆运动状态解耦为沿参考线的纵向 s 和横向 d 两个独立一维运动。提供 `cartesian_to_frenet` 和 `frenet_to_cartesian` 互转，以及 `CalculateTheta`、`CalculateKappa` 等辅助计算。所有方法为静态，不可实例化。

### KalmanFilter

离散时间卡尔曼滤波器，模板参数为状态维度 XN、观测维度 ZN、控制维度 UN。核心流程 `Predict` -> `Correct`。创新量和卡尔曼增益作为成员变量预分配以避免重复内存分配。

### HermiteSpline

Hermite 样条插值，支持 1D 和 2D。N=3 为三次（传入位置和速度），N=5 为五次（额外传入加速度）。`Evaluate(order, z)` 中 order 为求导阶数。

### MpcOsqp / QpSolver

`MpcOsqp` 基于 OSQP 的离散时间 MPC 求解器，构造时传入系统矩阵和约束，调用 `Solve` 输出控制序列。`QpSolver` 为二次规划抽象基类，定义 `min 0.5*x'Qx + x'c` 的标准 QP 问题，派生类需实现 `Solve()`。

### PathMatcher

路径匹配工具类（静态方法）。`MatchToPath` 按坐标 (x, y) 或弧长 s 在参考线上找最近路径点，`GetPathFrenetCoordinate` 获取 Frenet 坐标。

## 核心函数

### 数值积分

| 函数 | 说明 |
|---|---|
| `IntegrateBySimpson(funv_vec, dx, nsteps)` | Simpson 公式积分 |
| `IntegrateByTrapezoidal(funv_vec, dx, nsteps)` | 梯形公式积分 |
| `IntegrateByGaussLegendre<N>(func, lower, upper)` | N 阶 Gauss-Legendre 积分（N=2-10） |

### 插值与拟合

| 函数 | 说明 |
|---|---|
| `lerp(x0, t0, x1, t1, t)` | 模板化线性插值 |
| `slerp(a0, t0, a1, t1, t)` | 球面线性插值（角度 [-PI, PI)） |
| `InterpolateUsingLinearApproximation(p0, p1, w)` | PathPoint / TrajectoryPoint / SLPoint 线性插值 |
| `FitPolynomial<N, M>(points, *error)` | M 个 2D 点拟合 N 阶多项式，返回升序系数 |
| `EvaluatePolynomial<N>(coef, p)` | 多项式求值 |

### 控制与矩阵

| 函数 | 说明 |
|---|---|
| `SolveLQRProblem(A, B, Q, R, [M], tol, max_iter, *K)` | 离散 LQR 求解，DARE 迭代得反馈增益 K |
| `PseudoInverse<T, N>(m, epsilon)` | Moore-Penrose 伪逆（基于 SVD） |
| `ContinuousToDiscrete(A, B, C, D, ts, ...)` | 双线性变换连续系统到离散系统 |
| `DenseToCSCMatrix(dense, *data, *indices, *indptr)` | 密集矩阵转 CSC 稀疏格式 |

### 通用数学工具

| 函数 | 说明 |
|---|---|
| `Square<T> / Clamp<T>` | 平方、区间钳位 |
| `WrapAngle / NormalizeAngle / AngleDiff` | 角度归一化（[0, 2PI) 或 [-PI, PI)）和差值 |
| `CrossProd / InnerProd` | 2D 向量叉积和点积 |
| `Gaussian / Sigmoid` | 高斯函数、Sigmoid 激活函数 |
| `RotateVector2d(v, theta)` | 2D 向量逆时针旋转 |
| `RFUToFLU / FLUToRFU` | 右前上 / 前左上坐标系互转 |
| `Cartesian2Polar(x, y)` | 笛卡尔转极坐标 |
| `GoldenSectionSearch(func, lower, upper, tol)` | 黄金分割搜索求单峰函数极小值 |

## 配置

本模块为纯数学工具库，无运行时配置文件。关键参数通过函数参数或模板参数指定：

- **KalmanFilter** — 通过 `SetTransitionMatrix` / `SetTransitionNoise` / `SetObservationMatrix` / `SetObservationNoise` / `SetControlMatrix` 设置滤波器参数
- **AABoxKDTree2d** — 通过 `AABoxKDTreeParams` 控制：`max_depth`、`max_leaf_size`、`max_leaf_dimension`
- **MpcOsqp** — 构造时传入 `max_iter`（最大迭代）和 `eps_abs`（收敛精度）
- **Gauss-Legendre 积分** — 模板参数 N 指定阶数（2-10）

## 调用关系

```text
2D 几何图元
===========
Vec2d -> LineSegment2d -> Box2d -> Polygon2d
Vec2d -> AABox2d -------> Box2d
AABox2d -> AABoxKDTree2d<ObjectType>

坐标与旋转
==========
EulerAnglesZXY <--> Quaternion 辅助函数 --> NormalizeAngle
Angle<T> --> SIN_TABLE (sin_table.cc)
CartesianFrenetConverter --> Vec2d

控制与滤波
==========
KalmanFilter --> PseudoInverse (matrix_operations)
SolveLQRProblem --> PseudoInverse
MpcOsqp --> DenseToCSCMatrix, OSQP 外部库
QpSolver (基类) --> 派生类实现 Solve()
```
