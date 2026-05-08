---
title: "预测器"
---

# Predictor 预测器模块

> 源码路径: `modules/prediction/predictor/`

## 概述

预测器模块负责根据障碍物的类型、位置和状态生成未来运动轨迹。模块采用工厂+策略模式，由 `PredictorManager` 统一管理所有预测器实例，根据障碍物属性（车辆/行人/骑行者、车道内/车道外、路口内/路口外、优先级等）选择对应的预测算法。系统支持 8 种预测器类型，并可通过配置切换默认算法，同时支持串行和并行两种预测模式。

## 核心类

### Predictor

预测器抽象基类，定义了所有预测器的统一接口。

```cpp
class Predictor {
 public:
  virtual bool Predict(const ADCTrajectoryContainer* adc_trajectory_container,
                       Obstacle* obstacle,
                       ObstaclesContainer* obstacles_container) = 0;
  int NumOfTrajectories(const Obstacle& obstacle);
  virtual void Clear();
  void TrimTrajectories(const ADCTrajectoryContainer& adc_trajectory_container,
                        Obstacle* obstacle);
  const ObstacleConf::PredictorType& predictor_type();

 protected:
  static Trajectory GenerateTrajectory(
      const std::vector<apollo::common::TrajectoryPoint>& points);
  void SetEqualProbability(const double probability, const int start_index,
                           Obstacle* obstacle_ptr);
  bool TrimTrajectory(const ADCTrajectoryContainer& adc_trajectory_container,
                      Obstacle* obstacle, Trajectory* trajectory);
  bool SupposedToStop(const Feature& feature, const double stop_distance,
                      double* acceleration);
  ObstacleConf::PredictorType predictor_type_;
};
```

### PredictorManager

预测器管理器，负责预测器的注册、创建、路由和调度。

```cpp
class PredictorManager {
 public:
  void Init(const PredictionConf& config);
  void Run(const apollo::perception::PerceptionObstacles& perception_obstacles,
           const ADCTrajectoryContainer* adc_trajectory_container,
           ObstaclesContainer* obstacles_container);
  Predictor* GetPredictor(const ObstacleConf::PredictorType& type);
  void PredictObstacle(const ADCTrajectoryContainer* adc_trajectory_container,
                       Obstacle* obstacle,
                       ObstaclesContainer* obstacles_container,
                       PredictionObstacle* const prediction_obstacle);
  const PredictionObstacles& prediction_obstacles();

 private:
  std::map<ObstacleConf::PredictorType, std::unique_ptr<Predictor>> predictors_;
  // 各场景下的预测器类型映射（见下方配置章节）
};
```

## 核心函数

### Predictor 基类方法

| 方法 | 说明 |
|------|------|
| `Predict()` | 纯虚函数，子类实现具体预测逻辑，生成障碍物的预测轨迹 |
| `NumOfTrajectories()` | 返回障碍物最新特征中已生成的预测轨迹数量 |
| `GenerateTrajectory()` | 将轨迹点序列转换为 `Trajectory` 对象 |
| `SetEqualProbability()` | 从 `start_index` 开始，为所有轨迹均匀分配概率 |
| `TrimTrajectories()` | 遍历障碍物的所有预测轨迹，移除进入路口的尾部部分 |
| `TrimTrajectory()` | 裁剪单条轨迹：若起始点已在路口内则跳过，否则移除轨迹中首次进入路口之后的点 |
| `SupposedToStop()` | 根据停车距离和当前速度计算减速度，判断障碍物是否应在指定距离内停车 |

### PredictorManager 核心方法

| 方法 | 说明 |
|------|------|
| `Init()` | 根据 `PredictionConf` 配置各类障碍物对应的预测器类型 |
| `Run()` | 主入口，根据 `FLAGS_enable_multi_thread` 选择串行或并行模式 |
| `PredictObstacles()` | 串行模式：遍历所有感知障碍物，逐个调用 `PredictObstacle()` |
| `PredictObstaclesInParallel()` | 并行模式：按障碍物 ID 分组到线程池，各线程独立预测 |
| `PredictObstacle()` | 对单个障碍物进行预测，根据障碍物类型和状态路由到对应预测器 |
| `CreatePredictor()` | 工厂方法，根据 `PredictorType` 创建具体预测器实例 |
| `RegisterPredictor()` | 注册预测器到 `predictors_` 映射表 |

### 障碍物类型路由方法

| 方法 | 说明 |
|------|------|
| `RunVehiclePredictor()` | 车辆预测路由：交互模式 > 警戒模式 > 普通模式（车道内/路口内/车道外） |
| `RunPedestrianPredictor()` | 行人预测路由：使用 `pedestrian_predictor_` 配置的预测器 |
| `RunCyclistPredictor()` | 骑行者预测路由：根据是否在车道上选择对应预测器 |
| `RunDefaultPredictor()` | 默认预测路由：未知类型障碍物，按车道内外选择预测器 |
| `RunEmptyPredictor()` | 空操作预测器：用于被忽略、静止或启用了多智能体评估器的障碍物 |

## 配置

### 预测器类型

系统通过 `ObstacleConf::PredictorType` 枚举注册了以下 8 种预测器：

| 类型 | 实现类 | 说明 |
|------|--------|------|
| `LANE_SEQUENCE_PREDICTOR` | `LaneSequencePredictor` | 车道序列预测器，沿车道拓扑生成轨迹 |
| `MOVE_SEQUENCE_PREDICTOR` | `MoveSequencePredictor` | 运动序列预测器 |
| `SINGLE_LANE_PREDICTOR` | `SingleLanePredictor` | 单车道预测器 |
| `FREE_MOVE_PREDICTOR` | `FreeMovePredictor` | 自由运动预测器，用于车道外障碍物 |
| `JUNCTION_PREDICTOR` | `JunctionPredictor` | 路口预测器 |
| `EXTRAPOLATION_PREDICTOR` | `ExtrapolationPredictor` | 外推预测器 |
| `INTERACTION_PREDICTOR` | `InteractionPredictor` | 交互式预测器，考虑与主车的交互 |
| `EMPTY_PREDICTOR` | `EmptyPredictor` | 空预测器，不生成轨迹 |

### 默认预测器映射

`PredictorManager` 按障碍物类型和状态维护如下默认映射：

| 场景 | 默认预测器 |
|------|-----------|
| 车辆-车道内 | `LANE_SEQUENCE_PREDICTOR` |
| 车辆-车道外 | `FREE_MOVE_PREDICTOR` |
| 车辆-路口内 | `LANE_SEQUENCE_PREDICTOR` |
| 车辆-车道内(警戒) | `MOVE_SEQUENCE_PREDICTOR` |
| 车辆-路口内(警戒) | `INTERACTION_PREDICTOR` |
| 车辆-默认(警戒) | `EXTRAPOLATION_PREDICTOR` |
| 车辆-交互 | `EMPTY_PREDICTOR` |
| 骑行者-车道内 | `LANE_SEQUENCE_PREDICTOR` |
| 骑行者-车道外 | `FREE_MOVE_PREDICTOR` |
| 行人 | `FREE_MOVE_PREDICTOR` |
| 未知-车道内 | `LANE_SEQUENCE_PREDICTOR` |
| 未知-车道外 | `FREE_MOVE_PREDICTOR` |

### 关键运行参数

| 参数 | 说明 |
|------|------|
| `FLAGS_enable_multi_thread` | 是否启用并行预测模式 |
| `FLAGS_enable_trim_prediction_trajectory` | 是否裁剪进入路口的轨迹尾部 |
| `FLAGS_enable_multi_agent_vehicle_evaluator` | 车辆是否使用多智能体评估器（启用时跳过预测器） |
| `FLAGS_enable_multi_agent_pedestrian_evaluator` | 行人是否使用多智能体评估器 |
| `FLAGS_prediction_trajectory_time_length` | 预测轨迹时长 |
| `FLAGS_max_thread_num` | 并行预测最大线程数 |
| `FLAGS_distance_beyond_junction` | 路口距离阈值，用于轨迹裁剪 |
| `FLAGS_distance_to_slow_down_at_stop_sign` | 停车标志减速距离 |

## 调用关系

```text
Prediction (预测子系统)
  └─ PredictorManager::Run()
       ├─ [串行] PredictObstacles()
       │    └─ PredictObstacle()  (逐个障碍物)
       └─ [并行] PredictObstaclesInParallel()
            └─ PredictObstacle()  (线程池并行)

PredictObstacle()
  ├─ 障碍物忽略/静止 → RunEmptyPredictor() → EmptyPredictor::Predict()
  ├─ 车辆 → RunVehiclePredictor()
  │    ├─ 交互模式 → InteractionPredictor::Predict()
  │    ├─ 警戒模式 → MoveSequencePredictor / InteractionPredictor / ExtrapolationPredictor
  │    └─ 普通车辆 → LaneSequencePredictor / FreeMovePredictor / JunctionPredictor
  │         └─ [可选] TrimTrajectories() 裁剪路口轨迹
  ├─ 行人 → RunPedestrianPredictor() → FreeMovePredictor::Predict()
  ├─ 骑行者 → RunCyclistPredictor() → LaneSequencePredictor / FreeMovePredictor
  └─ 未知 → RunDefaultPredictor() → LaneSequencePredictor / FreeMovePredictor
```
