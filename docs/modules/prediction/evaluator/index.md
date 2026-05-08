---
title: "预测评估器"
---

# 预测评估器 (Evaluator)

> 源码路径: `modules/prediction/evaluator/`

## 概述

预测评估器模块负责为感知检测到的障碍物评估运动意图和未来轨迹概率分布。模块采用策略模式，通过 `Evaluator` 基类定义统一接口，针对不同障碍物类型（车辆、行人、骑行者）提供多种评估器实现。`EvaluatorManager` 作为管理中枢，根据障碍物类型、所在场景（车道/路口）和优先级（正常/警示）动态选择合适的评估器，支持多线程并行和多智能体联合评估。评估器的输出（意图标签、轨迹概率等）写入 `Obstacle::Feature`，供下游 Predictor 使用。

## 核心类

### Evaluator（基类）

所有评估器的抽象基类，定义统一的 `Evaluate` 接口和坐标变换辅助方法。

```cpp
class Evaluator {
 public:
  virtual ~Evaluator() = default;

  // 基础评估接口（纯虚函数）
  virtual bool Evaluate(Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container) = 0;

  // 带动态环境信息的评估（可选重写）
  virtual bool Evaluate(Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container,
                        std::vector<Obstacle*> dynamic_env);

  // 带 ADC 轨迹的评估（可选重写）
  virtual bool Evaluate(const ADCTrajectoryContainer* adc_trajectory_container,
                        Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container);

  virtual std::string GetName() = 0;

 protected:
  ObstacleConf::EvaluatorType evaluator_type_;
};
```

辅助坐标变换方法：

| 方法 | 说明 |
|------|------|
| `WorldCoordToObjCoord()` | 世界坐标转障碍物前方参考局部坐标 |
| `WorldCoordToObjCoordNorth()` | 世界坐标转障碍物北向参考局部坐标 |
| `WorldAngleToObjAngle()` | 世界角度转障碍物局部角度 |
| `VectorToMatrixXf()` | 向量转 Eigen 矩阵（用于神经网络输入） |

**源码**：`modules/prediction/evaluator/evaluator.h`

### EvaluatorManager

评估器管理器，负责创建、注册和调度所有评估器。

```cpp
class EvaluatorManager {
 public:
  void Init(const PredictionConf& config);
  Evaluator* GetEvaluator(const ObstacleConf::EvaluatorType& type);
  void Run(const ADCTrajectoryContainer* adc_trajectory_container,
           ObstaclesContainer* obstacles_container);
  void EvaluateObstacle(const ADCTrajectoryContainer* adc_trajectory_container,
                        Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container,
                        std::vector<Obstacle*> dynamic_env);
  void EvaluateMultiObstacle(
      const ADCTrajectoryContainer* adc_trajectory_container,
      ObstaclesContainer* obstacles_container);

 private:
  std::unique_ptr<Evaluator> CreateEvaluator(
      const ObstacleConf::EvaluatorType& type);
  void RegisterEvaluator(const ObstacleConf::EvaluatorType& type);
  void RegisterEvaluators();
  void BuildObstacleIdHistoryMap(ObstaclesContainer* obstacles_container,
                                 size_t max_num_frame);
  void DumpCurrentFrameEnv(ObstaclesContainer* obstacles_container);

  std::map<ObstacleConf::EvaluatorType, std::unique_ptr<Evaluator>> evaluators_;
  std::unordered_map<int, ObstacleHistory> obstacle_id_history_map_;
  std::unique_ptr<SemanticMap> semantic_map_;
};
```

**源码**：`modules/prediction/evaluator/evaluator_manager.h`

### 具体评估器

`CreateEvaluator()` 工厂方法支持以下类型：

| 类型枚举 | 实现类 | 子目录 |
|---|---|---|
| `MLP_EVALUATOR` | `MLPEvaluator` | `vehicle/` |
| `CRUISE_MLP_EVALUATOR` | `CruiseMLPEvaluator` | `vehicle/` |
| `JUNCTION_MLP_EVALUATOR` | `JunctionMLPEvaluator` | `vehicle/` |
| `COST_EVALUATOR` | `CostEvaluator` | `vehicle/` |
| `LANE_SCANNING_EVALUATOR` | `LaneScanningEvaluator` | `vehicle/` |
| `LANE_AGGREGATING_EVALUATOR` | `LaneAggregatingEvaluator` | `vehicle/` |
| `JUNCTION_MAP_EVALUATOR` | `JunctionMapEvaluator` | `vehicle/` |
| `SEMANTIC_LSTM_EVALUATOR` | `SemanticLSTMEvaluator` | `vehicle/` |
| `JOINTLY_PREDICTION_PLANNING_EVALUATOR` | `JointlyPredictionPlanningEvaluator` | `vehicle/` |
| `VECTORNET_EVALUATOR` | `VectornetEvaluator` | `vehicle/` |
| `MULTI_AGENT_EVALUATOR` | `MultiAgentEvaluator` | `vehicle/` |
| `CYCLIST_KEEP_LANE_EVALUATOR` | `CyclistKeepLaneEvaluator` | `cyclist/` |

额外子目录：`pedestrian/`（行人交互评估器）、`model_manager/`（模型管理）、`warm_up/`（模型预热）。

## 核心函数

### EvaluatorManager::Init

初始化函数，解析配置并注册评估器。

```cpp
void EvaluatorManager::Init(const PredictionConf& config);
```

流程：

1. 若 `enable_semantic_map` 启用，初始化 `SemanticMap` 实例
2. 调用 `RegisterEvaluators()` 注册全部评估器
3. 遍历 `obstacle_conf` 配置，按障碍物类型 + 状态 + 优先级设置评估器映射（如 `vehicle_on_lane_evaluator_`、`vehicle_in_junction_caution_evaluator_` 等）

### EvaluatorManager::Run

主运行入口，对当前帧所有关注障碍物执行评估。

```cpp
void EvaluatorManager::Run(
    const ADCTrajectoryContainer* adc_trajectory_container,
    ObstaclesContainer* obstacles_container);
```

流程：

1. 构建障碍物历史帧映射 (`BuildObstacleIdHistoryMap`)，若启用语义地图则运行 `SemanticMap::RunCurrFrame()`
2. 若启用多智能体评估器，先执行 `EvaluateMultiObstacle()`
3. 根据 `enable_multi_thread` 选择多线程（使用 `PredictionThreadPool`）或单线程模式，逐障碍物调用 `EvaluateObstacle()`

### EvaluatorManager::EvaluateObstacle

根据障碍物类型和状态选择评估器并执行评估。

```cpp
void EvaluatorManager::EvaluateObstacle(
    const ADCTrajectoryContainer* adc_trajectory_container,
    Obstacle* obstacle,
    ObstaclesContainer* obstacles_container,
    std::vector<Obstacle*> dynamic_env);
```

评估器选择策略：

- **车辆**：警示级别优先（交互型 → 路口警示 → 车道警示 → 默认警示），失败则降级到正常评估（路口 → 车道）
- **骑行者**：仅在车道上时使用 `CyclistKeepLaneEvaluator`
- **行人**：警示级别或离线训练模式下使用语义 LSTM 评估器
- **其他**：在车道上时使用 `MLPEvaluator`

### EvaluatorManager::EvaluateMultiObstacle

多智能体评估入口，对行人和车辆分别调用 `MultiAgentEvaluator`。受 `enable_multi_agent_pedestrian_evaluator` 和 `enable_multi_agent_vehicle_evaluator` 标志控制。

## 配置

评估器通过 `PredictionConf` 中的 `obstacle_conf` 字段配置，每条配置包含：

- `obstacle_type`：障碍物类型（VEHICLE / BICYCLE / PEDESTRIAN / UNKNOWN）
- `obstacle_status`：所在场景（ON_LANE / IN_JUNCTION）
- `priority_type`：优先级（NORMAL / CAUTION）
- `evaluator_type`：对应评估器类型

场景与评估器默认映射：

| 场景 | 默认评估器 | 警戒评估器 |
|------|------------|------------|
| 车辆巡航 | `CRUISE_MLP_EVALUATOR` | `CRUISE_MLP_EVALUATOR` |
| 车辆路口 | `JUNCTION_MLP_EVALUATOR` | `JUNCTION_MAP_EVALUATOR` |
| 车辆默认 | `MLP_EVALUATOR` | `SEMANTIC_LSTM_EVALUATOR` |
| 骑行者车道 | `CYCLIST_KEEP_LANE_EVALUATOR` | — |
| 行人 | `SEMANTIC_LSTM_EVALUATOR` | — |

关键 gflags：

- `enable_multi_thread`：启用多线程评估
- `enable_semantic_map`：启用语义地图
- `enable_multi_agent_pedestrian_evaluator`：行人多智能体评估
- `enable_multi_agent_vehicle_evaluator`：车辆多智能体评估

## 调用关系

```text
Prediction::Run()
  -> EvaluatorManager::Run()
       -> SemanticMap::RunCurrFrame()           // 语义地图（可选）
       -> EvaluateMultiObstacle()               // 多智能体评估（可选）
       -> [多线程/单线程] EvaluateObstacle()     // 逐障碍物评估
            -> Evaluator::Evaluate()             // 具体评估器执行
```

- **上游**：`PredictionComponent` 调用 `EvaluatorManager::Run()`
- **输入**：`ObstaclesContainer`（障碍物数据）、`ADCTrajectoryContainer`（自车轨迹）
- **输出**：`Obstacle::Feature` 中的意图标签、轨迹概率等
- **下游**：Predictor 根据评估器输出生成预测轨迹
