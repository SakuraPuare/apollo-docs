# Planning Learning-Based 学习型规划组件函数级源码解析

本文聚焦 `modules/planning/planning_base/learning_based/` 目录，按函数级粒度拆解规划模块的 **10 个学习型组件**：鸟瞰图渲染器、模型推理接口、轨迹模仿推理、评估器、以及自动调参系统（特征生成、特征构建、MLP 模型）。

## 1. 模块定位

`learning_based/` 是规划模块的**机器学习子系统**，提供两种能力：

1. **轨迹模仿推理**：使用 LibTorch 加载 CNN/CNN-LSTM 模型，从鸟瞰图特征直接预测轨迹
2. **自动调参（Autotuning）**：使用 MLP 神经网络评估轨迹/速度剖面的代价，替代手工调参

```
轨迹模仿:
  BirdviewImgFeatureRenderer → cv::Mat (鸟瞰图)
       ↓
  TrajectoryImitationLibtorchInference → 预测轨迹

自动调参:
  AutotuningRawFeatureGenerator → 原始特征
       ↓
  AutotuningFeatureBuilder → 模型输入特征
       ↓
  AutotuningBaseModel (MLP) → 代价评分
```

## 2. 目录结构

```
modules/planning/planning_base/learning_based/
├── img_feature_renderer/
│   └── birdview_img_feature_renderer.h / .cc    # 鸟瞰图渲染
├── model_inference/
│   ├── model_inference.h                        # 推理接口基类
│   └── trajectory_imitation_libtorch_inference.h / .cc  # LibTorch 推理
├── pipeline/
│   ├── evaluator.h / .cc                        # 评估器
│   └── record_to_learning_data.cc               # 数据录制
└── tuning/
    ├── autotuning_base_model.h                  # 调参模型基类
    ├── autotuning_feature_builder.h             # 特征构建接口
    ├── autotuning_mlp_net_model.h / .cc         # MLP 网络模型
    ├── autotuning_raw_feature_generator.h / .cc # 原始特征生成
    └── speed_model/
        ├── autotuning_speed_feature_builder.h / .cc  # 速度特征构建
        └── autotuning_speed_mlp_model.h / .cc        # 速度 MLP 模型
```

## 3. 鸟瞰图渲染

### 3.1 BirdviewImgFeatureRenderer

```cpp
class BirdviewImgFeatureRenderer {
  DECLARE_SINGLETON(BirdviewImgFeatureRenderer);
 public:
  bool Init(const PlanningSemanticMapConfig& config);
  bool RenderMultiChannelEnv(const LearningDataFrame&, cv::Mat*);
  bool RenderBGREnv(const LearningDataFrame&, cv::Mat*);
  bool RenderCurrentEgoStatus(const LearningDataFrame&, cv::Mat*);
  bool RenderCurrentEgoPoint(const LearningDataFrame&, cv::Mat*);
  bool RenderCurrentEgoBox(const LearningDataFrame&, cv::Mat*);
};
```

**职责**：将驾驶环境渲染为鸟瞰图图像，作为学习模型的输入。

**渲染通道**：

| 方法 | 输出 | 说明 |
|------|------|------|
| `RenderMultiChannelEnv` | 多通道 cv::Mat | 道路图+限速图+障碍物+交通灯+路由 |
| `RenderBGREnv` | BGR cv::Mat | 彩色环境图（可视化用） |
| `RenderCurrentEgoStatus` | 2 通道 | 自车 box + 自车点 |
| `RenderCurrentEgoPoint` | 1 通道 | 自车中心点 |
| `RenderCurrentEgoBox` | 1 通道 | 自车 bounding box |

**成员变量**：

- `base_roadmap_img_`：预加载的完整道路图
- `base_speedlimit_img_`：预加载的完整限速图
- `map_bottom_left_point_x_/y_`：地图原点坐标
- `ego_vehicle_config_`：自车参数（用于渲染 box）

## 4. 模型推理

### 4.1 ModelInference — 推理接口基类

```cpp
class ModelInference {
 public:
  explicit ModelInference(const LearningModelInferenceTaskConfig& config);
  virtual string GetName() = 0;
  virtual bool LoadModel() = 0;
  virtual bool DoInference(LearningDataFrame*) = 0;
 protected:
  LearningModelInferenceTaskConfig config_;
};
```

- 纯虚接口，定义模型加载和推理的契约
- 所有具体推理类继承此基类

### 4.2 TrajectoryImitationLibtorchInference — LibTorch 轨迹模仿

```cpp
class TrajectoryImitationLibtorchInference : public ModelInference {
 public:
  string GetName() override;  // "TRAJECTORY_IMITATION_INFERENCE"
  bool LoadModel() override;
  bool DoInference(LearningDataFrame*) override;
 private:
  bool LoadCNNModel();
  bool LoadCNNLSTMModel();
  bool DoCNNMODELInference();
  bool DoCNNLSTMMODELInference();
  void output_postprocessing();
  torch::jit::script::Module model_;
  torch::Device device_;
};
```

**支持两种模型架构**：

| 架构 | 加载方法 | 推理方法 | 特点 |
|------|---------|---------|------|
| CNN | `LoadCNNModel` | `DoCNNMODELInference` | 纯卷积，单帧输入 |
| CNN-LSTM | `LoadCNNLSTMModel` | `DoCNNLSTMMODELInference` | 卷积+时序，多帧输入 |

**推理流程**：

1. `LoadModel`：加载 TorchScript 模型到 `device_`（CPU/GPU）
2. `DoInference`：从 `LearningDataFrame` 提取鸟瞰图特征
3. 前向推理：模型输出轨迹预测
4. `output_postprocessing`：后处理（坐标变换、轨迹平滑）

## 5. 评估器

### 5.1 Evaluator

```cpp
class Evaluator {
 public:
  void Init();
  void Close();
  void Evaluate(const string& source_file);
 private:
  LearningData learning_data_;
  TrajectoryEvaluator trajectory_evaluator_;
};
```

- 学习数据管线的一部分
- 读取源数据文件，评估轨迹质量，输出处理后的学习数据
- 用于离线训练数据准备

## 6. 自动调参系统

### 6.1 整体架构

```
AutotuningRawFeatureGenerator
  (从轨迹+环境生成原始特征)
        ↓
AutotuningFeatureBuilder (抽象)
  └── AutotuningSpeedFeatureBuilder (速度特征)
  (将原始特征转换为模型输入格式)
        ↓
AutotuningBaseModel (抽象)
  └── AutotuningSpeedMLPModel (速度模型)
  (MLP 前向推理，输出代价评分)
```

### 6.2 AutotuningRawFeatureGenerator — 原始特征生成

```cpp
class AutotuningRawFeatureGenerator {
 public:
  AutotuningRawFeatureGenerator(double time_range, size_t num_points,
                                 const ReferenceLineInfo&, const Frame&,
                                 const SpeedLimit&);
  Status EvaluateTrajectory(const vector<TrajectoryPoint>&, TrajectoryRawFeature*) const;
  Status EvaluateTrajectoryPoint(const TrajectoryPoint&, TrajectoryPointRawFeature*) const;
  Status EvaluateSpeedProfile(const vector<SpeedPoint>&, TrajectoryRawFeature*) const;
 private:
  vector<double> eval_time_;
  const ReferenceLineInfo& reference_line_info_;
  const Frame& frame_;
  const SpeedLimit& speed_limit_;
  vector<const STBoundary*> boundaries_;
  vector<vector<array<double,3>>> obs_boundaries_;
  vector<vector<array<double,3>>> stop_boundaries_;
  vector<vector<array<double,3>>> nudge_boundaries_;
  vector<vector<array<double,3>>> side_pass_boundaries_;
};
```

**职责**：从候选轨迹/速度剖面与驾驶环境的交互中提取原始特征。

**三种评估模式**：

| 方法 | 输入 | 输出 |
|------|------|------|
| `EvaluateTrajectory` | 轨迹点序列 | 完整轨迹原始特征 |
| `EvaluateTrajectoryPoint` | 单个轨迹点 | 单点原始特征 |
| `EvaluateSpeedProfile` | 速度剖面 | 速度相关原始特征 |

**特征来源**：

- 参考线信息（曲率、限速）
- ST 边界（障碍物决策类型分类）
- 帧信息（障碍物位置、交通信号）

### 6.3 AutotuningFeatureBuilder — 特征构建接口

```cpp
class AutotuningFeatureBuilder {
 public:
  virtual Status BuildFeature(const TrajectoryRawFeature&, TrajectoryFeature*) const = 0;
  virtual Status BuildPointFeature(const TrajectoryPointRawFeature&, TrajectoryPointwiseFeature*) const = 0;
};
```

- 纯虚接口，将原始特征转换为 MLP 可消费的格式
- 子类实现具体的特征映射逻辑

### 6.4 AutotuningMLPModel — MLP 网络模型

```cpp
class AutotuningMLPModel : public prediction::network::NetModel {
 public:
  void Run(const vector<MatrixXf>& inputs, MatrixXf* output) const override;
};
```

- 继承 `NetModel`，实现 MLP 前向推理
- 输入：特征矩阵向量
- 输出：代价/奖励矩阵

### 6.5 AutotuningBaseModel — 调参模型基类

```cpp
class AutotuningBaseModel {
 public:
  virtual Status SetParams() = 0;
  virtual double Evaluate(const TrajectoryFeature&) const = 0;
  virtual double Evaluate(const TrajectoryPointwiseFeature&) const = 0;
 protected:
  unique_ptr<AutotuningMLPModel> mlp_model_;
  unique_ptr<AutotuningFeatureBuilder> feature_builder_;
};
```

- 组合 `mlp_model_` 和 `feature_builder_`
- `Evaluate`：评估轨迹/点的代价评分

### 6.6 速度自动调参实现

#### AutotuningSpeedFeatureBuilder

```cpp
class AutotuningSpeedFeatureBuilder : public AutotuningFeatureBuilder {
  Status BuildFeature(const TrajectoryRawFeature&, TrajectoryFeature*) const override;
  Status BuildPointFeature(const TrajectoryPointRawFeature&, TrajectoryPointwiseFeature*) const override;
 private:
  void map_obstacle_feature();
  void map_nudge_obs_feature();
  void map_sidepass_obs_feature();
};
```

- 将速度相关原始特征映射为模型输入
- 处理障碍物、nudge、sidepass 三种特征类型

#### AutotuningSpeedMLPModel

```cpp
class AutotuningSpeedMLPModel : public AutotuningBaseModel {
  Status SetParams() override;
  double Evaluate(const TrajectoryFeature&) const override;
  double Evaluate(const TrajectoryPointwiseFeature&) const override;
 private:
  void FlattenFeatures(const TrajectoryFeature&, MatrixXd*) const;
  void FlattenFeatures(const SpeedPointwiseFeature&, int row, MatrixXd*) const;
};
```

- `SetParams`：初始化 MLP 模型和速度特征构建器
- `FlattenFeatures`：将 Protobuf 特征展平为 Eigen 矩阵供 MLP 输入
- `Evaluate`：调用 `mlp_model_->Run` 获取代价评分

## 7. 数据流全景

```
在线推理:
  LearningDataFrame
       ↓
  BirdviewImgFeatureRenderer → cv::Mat
       ↓
  TrajectoryImitationLibtorchInference → 预测轨迹

自动调参:
  候选轨迹/速度剖面
       ↓
  AutotuningRawFeatureGenerator → TrajectoryRawFeature
       ↓
  AutotuningSpeedFeatureBuilder → TrajectoryFeature
       ↓
  AutotuningSpeedMLPModel (MLP) → 代价评分
       ↓
  轨迹评估/排序

离线训练数据:
  源数据文件
       ↓
  Evaluator → 处理后的 LearningData
```

## 8. 设计模式

### 8.1 模板方法模式

- `ModelInference` 定义 `LoadModel → DoInference` 骨架
- `AutotuningBaseModel` 定义 `SetParams → Evaluate` 骨架

### 8.2 策略模式

- `AutotuningFeatureBuilder` 可替换（速度/路径/其他特征构建器）
- `ModelInference` 可替换（CNN/CNN-LSTM/其他模型）

### 8.3 单例模式

- `BirdviewImgFeatureRenderer` 使用 `DECLARE_SINGLETON`，全局唯一实例

### 8.4 管线模式

- 原始特征生成 → 特征构建 → MLP 推理 → 代价评分，各阶段解耦