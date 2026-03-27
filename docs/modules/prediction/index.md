---
title: Prediction 预测模块
description: Apollo 预测模块架构分析 —— 障碍物轨迹预测、评估器设计、场景分析与数据流
---

# Prediction 预测模块

预测模块是 Apollo 自动驾驶系统的核心模块之一，负责对感知模块检测到的障碍物（车辆、行人、自行车等）进行未来轨迹预测。预测结果将作为规划模块（Planning）的关键输入，帮助自车做出安全的行驶决策。

## 模块职责

- 接收感知模块输出的障碍物信息（`PerceptionObstacles`）
- 分析当前驾驶场景（巡航、路口等）
- 对障碍物进行优先级分类（Caution / Normal / Ignore）
- 评估障碍物的行为意图（车道保持、变道、路口转向等）
- 生成障碍物的未来预测轨迹
- 输出 `PredictionObstacles` 消息供规划模块使用

## 目录结构

```
modules/prediction/
├── BUILD                          # Bazel 构建文件
├── prediction_component.h/.cc     # 主组件入口（Cyber Component）
├── common/                        # 公共工具库
│   ├── environment_features.*     # 环境特征（自车车道、邻居车道、路口等）
│   ├── feature_output.*           # 特征输出工具
│   ├── junction_analyzer.*        # 路口分析器
│   ├── message_process.*          # 消息处理流程编排
│   ├── prediction_gflags.*        # 运行时参数（gflags）
│   ├── prediction_map.*           # 地图查询封装
│   ├── road_graph.*               # 道路图构建
│   ├── semantic_map.*             # 语义地图（用于深度学习模型）
│   ├── validation_checker.*       # 轨迹合法性校验
│   └── prediction_thread_pool.*   # 线程池
├── conf/                          # 配置文件
│   ├── prediction.conf            # gflags 配置
│   ├── prediction_conf.pb.txt     # Protobuf 文本格式配置
│   └── prediction_navi*.conf      # 导航模式配置
├── container/                     # 数据容器
│   ├── container.h                # Container 基类
│   ├── container_manager.*        # 容器管理器
│   ├── adc_trajectory/            # 自车轨迹容器
│   ├── obstacles/                 # 障碍物容器（核心）
│   ├── pose/                      # 位姿容器
│   └── storytelling/              # Storytelling 容器
├── dag/                           # DAG 流水线配置
│   ├── prediction.dag             # 单体模式
│   ├── prediction_lego.dag        # 分体模式（子模块拆分）
│   └── prediction_navi.dag        # 导航模式
├── data/                          # 模型权重文件（.pt / .bin）
├── evaluator/                     # 评估器
│   ├── evaluator.h                # Evaluator 基类
│   ├── evaluator_manager.*        # 评估器管理器
│   ├── cyclist/                   # 自行车评估器
│   ├── pedestrian/                # 行人评估器
│   ├── vehicle/                   # 车辆评估器（多种实现）
│   ├── model_manager/             # 深度学习模型管理（插件化）
│   └── warm_up/                   # 模型预热
├── network/                       # 神经网络层定义
├── pipeline/                      # VectorNet 地图向量化管线
├── predictor/                     # 预测器
│   ├── predictor.h                # Predictor 基类
│   ├── predictor_manager.*        # 预测器管理器
│   ├── empty/                     # 空预测器
│   ├── extrapolation/             # 外推预测器
│   ├── free_move/                 # 自由运动预测器
│   ├── interaction/               # 交互预测器
│   ├── junction/                  # 路口预测器
│   ├── lane_sequence/             # 车道序列预测器
│   ├── move_sequence/             # 运动序列预测器
│   ├── sequence/                  # 序列预测器基类
│   └── single_lane/               # 单车道预测器
├── proto/                         # Protobuf 定义
│   ├── prediction_conf.proto      # 配置消息定义
│   ├── vector_net.proto           # VectorNet 相关
│   ├── fnn_model_base.proto       # FNN 模型定义
│   └── network_*.proto            # 网络层定义
├── scenario/                      # 场景分析
│   ├── scenario_manager.*         # 场景管理器
│   ├── analyzer/                  # 场景分析器
│   ├── feature_extractor/         # 特征提取器
│   ├── interaction_filter/        # 交互过滤器
│   ├── prioritization/            # 障碍物优先级排序
│   ├── right_of_way/              # 路权分析
│   └── scenario_features/         # 场景特征（Cruise / Junction）
└── submodules/                    # 子模块（Lego 模式）
    ├── evaluator_submodule.*      # 评估器子模块
    ├── predictor_submodule.*      # 预测器子模块
    └── submodule_output.*         # 子模块间数据传递
```

## 核心架构

### 组件入口

`PredictionComponent` 是预测模块的 Cyber RT 组件入口，继承自 `cyber::Component<PerceptionObstacles>`。它持有四个核心管理器：

```cpp
class PredictionComponent : public cyber::Component<PerceptionObstacles> {
  std::shared_ptr<ContainerManager> container_manager_;
  std::unique_ptr<EvaluatorManager> evaluator_manager_;
  std::unique_ptr<PredictorManager> predictor_manager_;
  std::unique_ptr<ScenarioManager> scenario_manager_;
};
```

### 处理流程

模块的主处理流程由 `MessageProcess` 类编排，核心方法 `OnPerception` 串联了完整的预测管线：

```
PerceptionObstacles
       │
       ▼
┌──────────────┐
│ ContainerMgr │  ← 数据注入（障碍物、自车轨迹、位姿等）
└──────┬───────┘
       │
       ▼
┌──────────────┐
│ ScenarioMgr  │  ← 场景分析（Cruise / Junction）
│              │  ← 特征提取 → 优先级排序 → 路权分析
└──────┬───────┘
       │
       ▼
┌──────────────┐
│ EvaluatorMgr │  ← 行为评估（概率计算）
└──────┬───────┘
       │
       ▼
┌──────────────┐
│ PredictorMgr │  ← 轨迹生成
└──────┬───────┘
       │
       ▼
 PredictionObstacles
```

## 运行模式

预测模块支持两种运行模式，通过不同的 DAG 文件配置：

### 单体模式（prediction.dag）

所有逻辑在 `PredictionComponent` 内顺序执行，适合调试和低延迟场景。

```
/apollo/perception/obstacles → PredictionComponent → /apollo/prediction
```

### 分体模式（prediction_lego.dag）

将预测流程拆分为三个 Cyber Component，通过 channel 通信实现流水线并行：

```
PredictionComponent（容器 + 场景分析）
       │
       ├─→ /apollo/prediction/container
       ├─→ /apollo/prediction/adccontainer
       └─→ /apollo/prediction/perception_obstacles
              │
              ▼
      EvaluatorSubmodule（评估）
              │
              └─→ /apollo/prediction/evaluator
                          │
                          ▼
               PredictorSubmodule（预测）
                          │
                          └─→ /apollo/prediction
```

## Container 容器设计

Container 是数据存储的抽象基类，通过 `Insert(protobuf::Message)` 接口注入数据。`ContainerManager` 使用工厂模式按消息类型管理所有容器实例。

| 容器 | 类名 | 职责 |
|------|------|------|
| 障碍物容器 | `ObstaclesContainer` | 管理所有障碍物的历史特征，构建车道图和路口特征，使用 LRU 缓存 |
| 自车轨迹容器 | `ADCTrajectoryContainer` | 存储规划模块输出的自车轨迹，供交互预测使用 |
| 位姿容器 | `PoseContainer` | 存储自车定位信息 |
| Storytelling 容器 | `StorytellingContainer` | 存储上游 Storytelling 模块的场景故事信息 |

`ObstaclesContainer` 是最核心的容器，内部维护：
- `LRUCache<int, Obstacle>` —— 按 ID 缓存障碍物对象
- `ObstacleClusters` —— 障碍物聚类信息
- `JunctionAnalyzer` —— 路口分析器
- 当前帧可移动 / 不可移动 / 需关注的障碍物 ID 列表

## Scenario 场景分析

场景分析模块负责判断当前自车所处的驾驶场景，并据此调整预测策略。

### 分析流程

```
ScenarioManager::Run()
       │
       ▼
FeatureExtractor::ExtractEnvironmentFeatures()
  → 提取自车车道、邻居车道、前方路口等环境特征
       │
       ▼
ScenarioAnalyzer::Analyze()
  → 根据环境特征判断场景类型
       │
       ▼
ObstaclesPrioritizer
  → AssignIgnoreLevel()   忽略远处/不相关障碍物
  → AssignCautionLevel()  标记需要重点关注的障碍物
       │
       ▼
InteractionFilter
  → AssignInteractiveTag()  标记与自车存在交互的障碍物
       │
       ▼
RightOfWay::Analyze()
  → 分析各车道序列的路权关系
```

### 场景类型

| 场景 | 类名 | 说明 |
|------|------|------|
| 巡航场景 | `CruiseScenarioFeatures` | 自车在车道内正常行驶，维护"感兴趣车道"集合 |
| 路口场景 | `JunctionScenarioFeatures` | 自车接近或进入路口区域 |

### 障碍物优先级

`ObstaclesPrioritizer` 根据场景为障碍物分配优先级：

- `CAUTION` —— 需要重点关注（近距离、同车道、合流区域等）
- `NORMAL` —— 正常预测
- `IGNORE` —— 忽略（远处、不相关）

不同优先级的障碍物会使用不同的 Evaluator 和 Predictor 组合，CAUTION 级别使用更精确但计算量更大的模型。

## Evaluator 评估器

Evaluator 负责评估障碍物的行为意图，为每条可能的车道序列计算概率。所有评估器继承自 `Evaluator` 基类：

```cpp
class Evaluator {
  virtual bool Evaluate(Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container) = 0;
  virtual std::string GetName() = 0;
};
```

### 评估器类型

| 评估器 | 枚举值 | 适用对象 | 算法 |
|--------|--------|----------|------|
| `MlpEvaluator` | `MLP_EVALUATOR` | 默认（on-lane） | 多层感知机，预测车道序列概率 |
| `CruiseMLPEvaluator` | `CRUISE_MLP_EVALUATOR` | 车辆（on-lane, normal） | 巡航场景 MLP，区分 go/cutin 行为 |
| `JunctionMLPEvaluator` | `JUNCTION_MLP_EVALUATOR` | 车辆（in-junction, normal） | 路口场景 MLP |
| `JunctionMapEvaluator` | `JUNCTION_MAP_EVALUATOR` | 车辆（in-junction, caution） | 基于地图的路口评估 |
| `CostEvaluator` | `COST_EVALUATOR` | 导航模式 | 基于代价函数的评估 |
| `CyclistKeepLaneEvaluator` | `CYCLIST_KEEP_LANE_EVALUATOR` | 自行车（on-lane） | 自行车车道保持评估 |
| `LaneScanningEvaluator` | `LANE_SCANNING_EVALUATOR` | 车辆 | 车道扫描评估 |
| `LaneAggregatingEvaluator` | `LANE_AGGREGATING_EVALUATOR` | 车辆 | 车道聚合评估 |
| `PedestrianInteractionEvaluator` | `PEDESTRIAN_INTERACTION_EVALUATOR` | 行人 | 行人交互评估 |
| `SemanticLSTMEvaluator` | `SEMANTIC_LSTM_EVALUATOR` | 行人/车辆（caution） | 基于语义地图的 LSTM，使用 PyTorch |
| `VectornetEvaluator` | `VECTORNET_EVALUATOR` | 车辆（caution） | 基于 VectorNet 的图神经网络 |
| `JointlyPredictionPlanningEvaluator` | `JOINTLY_PREDICTION_PLANNING_EVALUATOR` | 车辆（交互） | 联合预测-规划评估，考虑自车轨迹 |
| `MultiAgentEvaluator` | `MULTI_AGENT_EVALUATOR` | 多智能体 | 多智能体联合预测，最多处理 50 个 agent |

### 模型管理

`ModelManager` 采用插件化架构管理深度学习模型，支持 CPU/GPU 后端自动选择：

```
evaluator/model_manager/
├── model_manager.h          # 模型管理器（插件加载、后端选择）
└── model/
    ├── model_base.h         # 模型基类
    ├── semantic_lstm_vehicle_torch_cpu/
    ├── semantic_lstm_vehicle_torch_gpu/
    ├── semantic_lstm_pedestrian_torch_cpu/
    ├── semantic_lstm_pedestrian_torch_gpu/
    ├── multi_agent_vehicle_torch_cpu/
    ├── multi_agent_vehicle_torch_gpu/
    ├── multi_agent_pedestrian_torch_cpu/
    └── multi_agent_pedestrian_torch_gpu/
```

### EvaluatorManager 分发逻辑

`EvaluatorManager` 根据障碍物类型、状态和优先级选择对应的评估器：

| 障碍物类型 | 状态 | 优先级 | 评估器 |
|-----------|------|--------|--------|
| VEHICLE | ON_LANE | CAUTION | `VECTORNET_EVALUATOR` |
| VEHICLE | ON_LANE | NORMAL | `CRUISE_MLP_EVALUATOR` |
| VEHICLE | IN_JUNCTION | CAUTION | `VECTORNET_EVALUATOR` |
| VEHICLE | IN_JUNCTION | NORMAL | `JUNCTION_MLP_EVALUATOR` |
| VEHICLE | OFF_LANE | - | 无评估（直接 free move） |
| VEHICLE | INTERACTION | - | `JOINTLY_PREDICTION_PLANNING_EVALUATOR` |
| PEDESTRIAN | MOVING | - | `SEMANTIC_LSTM_EVALUATOR` |
| BICYCLE | ON_LANE | - | `CYCLIST_KEEP_LANE_EVALUATOR` |
| UNKNOWN | ON_LANE | - | `MLP_EVALUATOR` |

## Predictor 预测器

Predictor 负责根据评估结果生成障碍物的未来轨迹点序列。所有预测器继承自 `Predictor` 基类：

```cpp
class Predictor {
  virtual bool Predict(const ADCTrajectoryContainer* adc_trajectory_container,
                       Obstacle* obstacle,
                       ObstaclesContainer* obstacles_container) = 0;
};
```

### 预测器继承体系

```
Predictor（基类）
├── FreeMovePredictor          # 自由运动
├── JunctionPredictor          # 路口
├── EmptyPredictor             # 空预测
└── SequencePredictor（序列基类）
    ├── LaneSequencePredictor  # 车道序列
    ├── MoveSequencePredictor  # 运动序列
    ├── SingleLanePredictor    # 单车道
    ├── InteractionPredictor   # 交互
    └── ExtrapolationPredictor # 外推
```

### 预测器类型

| 预测器 | 枚举值 | 适用场景 | 算法 |
|--------|--------|----------|------|
| `FreeMovePredictor` | `FREE_MOVE_PREDICTOR` | 离开车道的障碍物、行人 | 基于当前速度/加速度的运动学外推 |
| `LaneSequencePredictor` | `LANE_SEQUENCE_PREDICTOR` | 车道内车辆（normal） | 沿车道序列生成等加速度轨迹 |
| `MoveSequencePredictor` | `MOVE_SEQUENCE_PREDICTOR` | 车道内车辆（caution on-lane） | 纵向五次多项式 + 横向四次多项式拟合 |
| `SingleLanePredictor` | `SINGLE_LANE_PREDICTOR` | 单车道场景 | 沿单一车道生成轨迹 |
| `JunctionPredictor` | `JUNCTION_PREDICTOR` | 路口内车辆 | 基于路口出口的多项式轨迹生成 |
| `InteractionPredictor` | `INTERACTION_PREDICTOR` | 路口内需关注车辆 | 考虑与自车交互的代价函数优化，含碰撞代价、加速度代价 |
| `ExtrapolationPredictor` | `EXTRAPOLATION_PREDICTOR` | CAUTION 级别车辆 | 对评估器输出轨迹进行外推延伸（沿车道或自由运动） |
| `EmptyPredictor` | `EMPTY_PREDICTOR` | 交互标记车辆 | 不生成轨迹（由联合预测-规划评估器直接输出） |

### PredictorManager 分发逻辑

`PredictorManager` 按障碍物类型分别调用不同的预测流程：

```
RunVehiclePredictor()
  ├── CAUTION + ON_LANE     → vehicle_on_lane_caution_predictor_    (MOVE_SEQUENCE)
  ├── CAUTION + IN_JUNCTION → vehicle_in_junction_caution_predictor_(INTERACTION)
  ├── CAUTION + default     → vehicle_default_caution_predictor_    (EXTRAPOLATION)
  ├── INTERACTION           → vehicle_interactive_predictor_        (EMPTY)
  ├── ON_LANE               → vehicle_on_lane_predictor_            (LANE_SEQUENCE)
  ├── IN_JUNCTION           → vehicle_in_junction_predictor_        (LANE_SEQUENCE)
  └── OFF_LANE              → vehicle_off_lane_predictor_           (FREE_MOVE)

RunPedestrianPredictor()
  └── pedestrian_predictor_                                         (FREE_MOVE)

RunCyclistPredictor()
  ├── ON_LANE               → cyclist_on_lane_predictor_            (LANE_SEQUENCE)
  └── OFF_LANE              → cyclist_off_lane_predictor_           (FREE_MOVE)

RunDefaultPredictor()
  ├── ON_LANE               → default_on_lane_predictor_            (LANE_SEQUENCE)
  └── OFF_LANE              → default_off_lane_predictor_           (FREE_MOVE)
```

## VectorNet 管线

`VectorNet` 类负责将 HD Map 数据向量化，为深度学习评估器提供地图特征输入。它从高精地图中提取：

- 道路中心线（Roads）
- 车道线及边界类型（Lanes，含虚线/实线/黄线/白线/路缘等）
- 路口区域（Junctions）
- 人行横道（Crosswalks）

地图元素被编码为 polyline 向量序列，每个向量包含起止坐标、属性类型和边界类型。

## Proto 消息定义

### prediction_conf.proto

核心配置消息定义：

```protobuf
message ObstacleConf {
  enum ObstacleStatus { ON_LANE, OFF_LANE, STATIONARY, MOVING, IN_JUNCTION }
  enum EvaluatorType  { MLP_EVALUATOR, COST_EVALUATOR, CRUISE_MLP_EVALUATOR, ... }
  enum PredictorType  { LANE_SEQUENCE_PREDICTOR, FREE_MOVE_PREDICTOR, ... }

  optional PerceptionObstacle.Type obstacle_type = 1;
  optional ObstacleStatus obstacle_status = 2;
  optional ObstaclePriority.Priority priority_type = 5;
  optional ObstacleInteractiveTag.InteractiveTag interactive_tag = 6;
  optional EvaluatorType evaluator_type = 3;
  optional PredictorType predictor_type = 4;
}

message PredictionConf {
  optional TopicConf topic_conf = 1;
  repeated ObstacleConf obstacle_conf = 2;        // 障碍物配置规则列表
  optional EvaluatorModelConf evaluator_model_conf = 3;  // 模型配置
}
```

### 其他 Proto 文件

| 文件 | 用途 |
|------|------|
| `vector_net.proto` | VectorNet 地图向量化相关消息 |
| `fnn_model_base.proto` | 前馈神经网络模型基础定义 |
| `fnn_vehicle_model.proto` | 车辆 FNN 模型参数 |
| `network_layers.proto` | 网络层定义（Dense、LSTM 等） |
| `network_model.proto` | 网络模型结构定义 |
| `offline_features.proto` | 离线特征数据格式 |

## 配置方式

### 主配置文件（prediction_conf.pb.txt）

通过 `obstacle_conf` 列表定义不同障碍物类型 + 状态 + 优先级组合对应的评估器和预测器：

```protobuf
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  priority_type: CAUTION
  evaluator_type: VECTORNET_EVALUATOR
  predictor_type: EXTRAPOLATION_PREDICTOR
}
```

### Topic 配置

```protobuf
topic_conf {
  perception_obstacle_topic: "/apollo/perception/obstacles"
  planning_trajectory_topic: "/apollo/planning"
  localization_topic: "/apollo/localization/pose"
  prediction_topic: "/apollo/prediction"
  storytelling_topic: "/apollo/storytelling"
}
```

### 模型配置

通过 `evaluator_model_conf` 配置深度学习模型的后端（CPU/GPU）和类型：

```protobuf
evaluator_model_conf {
  model {
    evaluator_type: SEMANTIC_LSTM_EVALUATOR
    obstacle_type: PEDESTRIAN
    backend: GPU
    type: "SemanticLstmPedestrianGpuTorch"
  }
}
```

### DAG 配置

通过 `dag/` 目录下的 `.dag` 文件选择运行模式。在 Cyber RT 启动时指定对应的 DAG 文件即可切换单体/分体/导航模式。

### gflags 运行时参数

`common/prediction_gflags.h` 和 `common/prediction_system_gflags.h` 定义了大量运行时可调参数，通过 `conf/prediction.conf` 文件设置。

## 模型权重文件

`data/` 目录存放预训练模型权重：

| 文件 | 对应评估器 |
|------|-----------|
| `cruise_go_vehicle_model.pt` | CruiseMLPEvaluator（go 模型） |
| `cruise_cutin_vehicle_model.pt` | CruiseMLPEvaluator（cutin 模型） |
| `junction_mlp_vehicle_model.pt` | JunctionMLPEvaluator |
| `junction_map_vehicle_model.pt` | JunctionMapEvaluator |
| `lane_scanning_vehicle_model.pt` | LaneScanningEvaluator |
| `vectornet_vehicle_model.pt` | VectornetEvaluator（GPU） |
| `vectornet_vehicle_cpu_model.pt` | VectornetEvaluator（CPU） |
| `jointly_prediction_planning_vehicle_model.pt` | JointlyPredictionPlanningEvaluator（GPU） |
| `jointly_prediction_planning_vehicle_cpu_model.pt` | JointlyPredictionPlanningEvaluator（CPU） |
| `pedestrian_interaction_*.pt` | PedestrianInteractionEvaluator |
| `traced_online_*.pt` | SemanticLSTMEvaluator |
| `mlp_vehicle_model.bin` | MlpEvaluator |
| `rnn_vehicle_model.bin` | RnnEvaluator（已废弃） |
