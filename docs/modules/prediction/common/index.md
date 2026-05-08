---
title: "预测通用工具"
---

# Common

> 源码路径: `modules/prediction/common/`

## 概述

`prediction/common` 为预测系统提供基础设施层，包括地图查询（`PredictionMap`）、消息处理（`MessageProcess`）、特征输出（`FeatureOutput`）、gflags 配置、分层线程池、数学工具、路口分析（`JunctionAnalyzer`）、环境特征（`EnvironmentFeatures`）、道路图构建（`RoadGraph`）、语义地图（`SemanticMap`）、轨迹验证（`ValidationChecker`）和 GPU 仿射变换（`AffineTransform`）。

## 核心类

### PredictionMap

静态地图查询工具类，封装 `hdmap` 接口。提供车道位置/航向/曲率查询、Frenet 投影、车道拓扑关系（前驱/后继/左邻/右邻）、路口判定、附近车道搜索等功能。构造函数被 `delete`，不可实例化。

### MessageProcess

静态消息处理工具类。`Init()` 初始化容器/评估器/预测器；`OnPerception()` 执行完整预测管线（场景分析 -> 评估 -> 预测）；`OnLocalization/OnPlanning/OnStoryTelling()` 更新对应容器；`ProcessOfflineData()` 从 Record 文件回放离线数据。

### FeatureOutput

静态特征序列化工具类。支持五种数据类型的插入和写出：`FeatureProto`、`DataForLearning`、`PredictionResult`、`FrameEnv`、`DataForTuning`。内部使用 `mutex` 保证线程安全。

### EnvironmentFeatures

封装自车状态（位置、速度、加速度、航向）和环境信息（当前车道、左右邻车道、前方路口、障碍物 ID 列表、不可忽略的逆行道）。

### JunctionAnalyzer

路口分析器。根据 `junction_id` 初始化后，维护出口车道到 `JunctionExit` 的哈希表，支持按起始车道查询路口特征。

### RoadGraph

沿车道拓扑构建 `LaneGraph`。构造时指定起始 s 值、搜索长度和起始车道，`BuildLaneGraph()` 通过 DFS 递归构建车道序列，支持双向搜索。

### SemanticMap

语义地图生成器，为神经网络评估器提供鸟瞰图输入。异步绘制基础地图（道路/路口/人行道/车道），按障碍物历史裁剪子图。aarch64 平台使用 `AffineTransform` 进行 GPU 加速。

### ValidationChecker

静态轨迹验证工具类。`ProbabilityByCentripetalAcceleration()` 基于向心加速度计算概率；`ValidCentripetalAcceleration()` 和 `ValidTrajectoryPoint()` 检查轨迹物理合法性。

### AffineTransform

基于 NPP 的 GPU 仿射变换，管理 `CCObjectPool` 复用 CUDA 内存，提供 `GetCoeffs()` 和 `AffineTransformsFromMat()` 两个核心方法。

### BaseThreadPool / PredictionThreadPool

分层线程池。`BaseThreadPool` 基于 Cyber `BoundedQueue` 实现任务队列和工作线程；`LevelThreadPool<LEVEL>` 为每层单例（默认三级各 20 线程）；`PredictionThreadPool` 通过 `thread_local` 变量按层级路由到对应线程池。

## 核心函数

### 数学工具（`math_util`）

| 函数 | 说明 |
|---|---|
| `Normalize / Relu / Softmax` | 归一化与激活函数 |
| `SolveQuadraticEquation` | 求解一元二次方程 |
| `EvaluateQuinticPolynomial` | 五次多项式求值 |
| `EvaluateQuarticPolynomial` | 四次多项式求值 |
| `EvaluateCubicPolynomial` | 三次多项式求值 |
| `ComputePolynomial<N>` | 由边界条件计算多项式系数 |
| `GetSByConstantAcceleration` | 匀加速运动位移 |

### 预测器工具（`predictor_util`）

| 函数 | 说明 |
|---|---|
| `TranslatePoint` | 平移轨迹点 |
| `GenerateFreeMoveTrajectoryPoints` | 生成自由运动轨迹序列 |
| `AdjustSpeedByCurvature` | 根据曲率调整速度 |

### PredictionConstants

离线模式常量：`kOnlineMode(0)`、`kDumpFeatureProto(1)`、`kDumpDataForLearning(2)`、`kDumpPredictionResult(3)`、`kDumpFrameEnv(4)`、`kDumpDataForTuning(5)`、`kDumpRecord(6)`。

## 配置

通过 `prediction_gflags.h/cc` 和 `prediction_system_gflags.h/cc` 管理，使用 `DECLARE_*`/`DEFINE_*` 模式。

- **prediction_gflags**：轨迹参数（`prediction_trajectory_time_length=6.0s`）、地图搜索（`lane_search_radius=3.0m`）、场景阈值（`junction_distance_threshold=10.0m`）、障碍物特征、评估器模型路径、交互预测权重等
- **prediction_system_gflags**：模块名称、配置文件路径、离线模式（`prediction_offline_mode`）、多线程开关（`enable_multi_thread`）、CUDA 开关（`use_cuda`）、子模块 topic 名称等

## 调用关系

```text
PredictionComponent
  +-- MessageProcess::Init()
  |     +-- InitContainers/InitEvaluators/InitPredictors
  |     +-- PredictionMap::Ready()
  +-- MessageProcess::OnPerception()
        +-- ContainerProcess()
        |     +-- ObstaclesContainer::Insert()
        |     +-- ScenarioManager::Run()
        |     +-- JunctionAnalyzer::Init()
        |     +-- RoadGraph::BuildLaneGraph()
        |     +-- RightOfWay::Analyze()
        +-- EvaluatorManager::Run()
        |     +-- SemanticMap::RunCurrFrame()
        |     +-- PredictionThreadPool::ForEach()
        +-- PredictorManager::Run()
              +-- ValidationChecker / PredictionMap / math_util
MessageProcess::ProcessOfflineData()
  +-- RecordReader -> OnPerception/OnLocalization/OnPlanning
  +-- FeatureOutput::Insert*() / Write*()
```
