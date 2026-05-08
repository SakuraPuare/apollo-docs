---
title: "预测子模块"
---

# 预测子模块

> 源码路径: `modules/prediction/submodules/`

## 概述

预测子模块是 Apollo 预测模块的 Cyber RT 组件层，将评估（Evaluator）和预测（Predictor）两个阶段拆分为独立的 `Component` 子模块，通过 Cyber RT 的 Writer/Reader 机制实现进程内或跨进程通信。

子模块采用两阶段流水线架构：

1. **EvaluatorSubmodule** 接收障碍物容器输出和 ADC 轨迹，运行评估器为每个障碍物计算特征和评估结果，输出 `SubmoduleOutput`
2. **PredictorSubmodule** 接收感知障碍物、ADC 轨迹和评估器输出，运行预测器生成最终的 `PredictionObstacles` 并发布

`SubmoduleOutput` 作为两个子模块之间的数据载体，在容器层和评估层之间传递帧级障碍物信息。

## 核心类

### EvaluatorSubmodule

继承自 `cyber::Component<ADCTrajectoryContainer, SubmoduleOutput>`，负责管理所有评估器的生命周期和执行。

```cpp
class EvaluatorSubmodule
    : public cyber::Component<ADCTrajectoryContainer, SubmoduleOutput> {
 public:
  ~EvaluatorSubmodule();
  std::string Name() const;
  bool Init() override;
  bool Proc(const std::shared_ptr<ADCTrajectoryContainer>& adc_trajectory_container,
            const std::shared_ptr<SubmoduleOutput>&) override;
 private:
  std::shared_ptr<cyber::Writer<SubmoduleOutput>> evaluator_writer_;
  std::unique_ptr<EvaluatorManager> evaluator_manager_;
};
```

- 通过 `CYBER_REGISTER_COMPONENT(EvaluatorSubmodule)` 注册为 Cyber RT 组件
- `Init()` 加载 `PredictionConf` 配置并初始化评估器列表
- `Proc()` 回调中构建 `ObstaclesContainer`，调用 `EvaluatorManager::Run()` 运行评估器，将结果通过 `evaluator_writer_` 发布

### PredictorSubmodule

继承自 `cyber::Component<PerceptionObstacles, ADCTrajectoryContainer, SubmoduleOutput>`，负责管理所有预测器并发布最终预测结果。

```cpp
class PredictorSubmodule
    : public cyber::Component<apollo::perception::PerceptionObstacles,
                              ADCTrajectoryContainer, SubmoduleOutput> {
 public:
  ~PredictorSubmodule() = default;
  std::string Name() const;
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::perception::PerceptionObstacles>&,
            const std::shared_ptr<ADCTrajectoryContainer>&,
            const std::shared_ptr<SubmoduleOutput>&) override;
 private:
  std::shared_ptr<cyber::Writer<PredictionObstacles>> predictor_writer_;
  std::unique_ptr<PredictorManager> predictor_manager_;
};
```

- 接收三个输入通道：感知障碍物、ADC 轨迹、评估器输出
- `Proc()` 回调中运行 `PredictorManager::Run()` 生成预测轨迹，填充感知时间戳头信息后通过 `predictor_writer_` 发布 `PredictionObstacles`

### SubmoduleOutput

子模块间的数据传递载体，封装单帧内的障碍物信息和场景上下文。

```cpp
class SubmoduleOutput {
 public:
  void InsertObstacle(const Obstacle&& obstacle);
  void InsertEgoVehicle(const Obstacle&& ego_vehicle);
  void set_curr_frame_movable_obstacle_ids(const std::vector<int>& ids);
  void set_curr_frame_unmovable_obstacle_ids(const std::vector<int>& ids);
  void set_curr_frame_considered_obstacle_ids(const std::vector<int>& ids);
  void set_frame_start_time(const apollo::cyber::Time& time);
  void set_curr_scenario(const Scenario& scenario);
  const std::vector<Obstacle>& curr_frame_obstacles() const;
  const Obstacle& GetEgoVehicle() const;
  // ... getter 方法
 protected:
  std::vector<Obstacle> curr_frame_obstacles_;
  Obstacle ego_vehicle_;
  std::vector<int> curr_frame_movable_obstacle_ids_;
  std::vector<int> curr_frame_unmovable_obstacle_ids_;
  std::vector<int> curr_frame_considered_obstacle_ids_;
  apollo::cyber::Time frame_start_time_;
  Scenario curr_scenario_;
};
```

支持分类管理障碍物 ID 列表：

- `curr_frame_movable_obstacle_ids_` — 可移动障碍物（车辆、行人等）
- `curr_frame_unmovable_obstacle_ids_` — 不可移动障碍物（锥桶、路障等）
- `curr_frame_considered_obstacle_ids_` — 本轮评估中被考虑的障碍物

## 核心函数

### EvaluatorSubmodule::Init()

```cpp
bool EvaluatorSubmodule::Init() {
  evaluator_manager_.reset(new EvaluatorManager());
  PredictionConf prediction_conf;
  ComponentBase::GetProtoConfig(&prediction_conf);
  MessageProcess::InitEvaluators(evaluator_manager_.get(), prediction_conf);
  evaluator_writer_ = node_->CreateWriter<SubmoduleOutput>(
      prediction_conf.topic_conf().evaluator_topic_name());
  return true;
}
```

加载 protobuf 格式的 `PredictionConf` 配置文件，通过 `MessageProcess::InitEvaluators` 初始化评估器，创建 Cyber RT Writer 用于发布评估结果。

### EvaluatorSubmodule::Proc()

```cpp
bool EvaluatorSubmodule::Proc(
    const std::shared_ptr<ADCTrajectoryContainer>& adc_trajectory_container,
    const std::shared_ptr<SubmoduleOutput>& container_output) {
  constexpr static size_t kHistorySize = 1;
  ObstaclesContainer obstacles_container(*container_output);
  evaluator_manager_->Run(adc_trajectory_container.get(), &obstacles_container);
  SubmoduleOutput submodule_output =
      obstacles_container.GetSubmoduleOutput(kHistorySize, frame_start_time);
  evaluator_writer_->Write(submodule_output);
  return true;
}
```

将容器层的 `SubmoduleOutput` 转换为 `ObstaclesContainer`，运行评估器后重新打包为新的 `SubmoduleOutput` 发布。`kHistorySize = 1` 表示仅保留当前帧历史。

### PredictorSubmodule::Init()

```cpp
bool PredictorSubmodule::Init() {
  predictor_manager_.reset(new PredictorManager());
  PredictionConf prediction_conf;
  ComponentBase::GetProtoConfig(&prediction_conf);
  MessageProcess::InitPredictors(predictor_manager_.get(), prediction_conf);
  predictor_writer_ = node_->CreateWriter<PredictionObstacles>(
      prediction_conf.topic_conf().prediction_topic());
  return true;
}
```

初始化预测器管理器并创建 Writer，发布目标 topic 为 `prediction_topic`。

### PredictorSubmodule::Proc()

```cpp
bool PredictorSubmodule::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles,
    const std::shared_ptr<ADCTrajectoryContainer>& adc_trajectory_container,
    const std::shared_ptr<SubmoduleOutput>& submodule_output) {
  ObstaclesContainer obstacles_container(*submodule_output);
  predictor_manager_->Run(*perception_obstacles,
      adc_trajectory_container.get(), &obstacles_container);
  PredictionObstacles prediction_obstacles =
      predictor_manager_->prediction_obstacles();
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  // 填充感知头时间戳
  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  predictor_writer_->Write(prediction_obstacles);
  return true;
}
```

运行预测器后，将感知原始时间戳（lidar/camera/radar）透传到预测输出头中，并记录端到端耗时。

## 配置

子模块通过 `PredictionConf` protobuf 配置（由 `ComponentBase::GetProtoConfig` 加载），关键配置项包括：

| 配置项 | 来源 | 说明 |
|---|---|---|
| `topic_conf().evaluator_topic_name()` | `PredictionConf` | 评估器输出 topic |
| `topic_conf().prediction_topic()` | `PredictionConf` | 最终预测结果 topic |
| 评估器/预测器初始化参数 | `MessageProcess::InitEvaluators/InitPredictors` | 通过 gflags 和 conf 文件控制 |

子模块名称由 gflags 控制：`FLAGS_evaluator_submodule_name`。

## 调用关系

```text
感知模块 (PerceptionObstacles)
    |
    v
容器层 (Container)
    |
    | 输出 SubmoduleOutput (障碍物列表 + 场景信息)
    v
EvaluatorSubmodule::Proc()
    |-- 构建 ObstaclesContainer
    |-- EvaluatorManager::Run()  <-- 评估器计算特征
    |-- 发布 SubmoduleOutput (带评估结果)
    |
    v
PredictorSubmodule::Proc()
    |-- 构建 ObstaclesContainer
    |-- PredictorManager::Run()  <-- 预测器生成轨迹
    |-- 填充时间戳头信息
    |-- 发布 PredictionObstacles
    |
    v
规划模块 (Planning)
```

三个类之间的依赖关系：

- `EvaluatorSubmodule` 和 `PredictorSubmodule` 均依赖 `SubmoduleOutput` 作为输入和/或输出
- `PredictorSubmodule` 的三通道输入设计使其能同时接收感知原始数据、ADC 轨迹和评估器输出
- `SubmoduleOutput` 通过 `ObstaclesContainer` 构造函数与容器层双向连接
