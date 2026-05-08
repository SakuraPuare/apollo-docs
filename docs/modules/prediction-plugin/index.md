---
title: "预测插件模块"
---

# 预测插件模块

> 源码路径: `modules/prediction_plugin/`

## 概述

`prediction_plugin` 模块是 Apollo 预测系统中的模型推理插件集合，基于 TensorRT 提供高性能 GPU 推理能力。该模块包含四个独立的深度学习推理插件，分别面向车辆和行人两类交通参与者的轨迹预测，采用两种不同的神经网络架构：

- **Multi-Agent Vectornet**：多智能体向量化网络，支持批量处理最多 50 个障碍物的轨迹预测，使用向量化地图特征作为输入
- **Semantic LSTM**：语义 LSTM 网络，处理单个障碍物的轨迹预测，使用语义分割图像和历史轨迹作为输入

所有插件均继承自 `ModelBase` 纯虚基类，通过 Cyber RT 插件管理器（`CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN`）注册为可动态加载的模型插件。推理过程通过 `std::mutex` 保证线程安全，并使用 CUDA Stream 实现异步数据传输。

## 核心类

### ModelBase（基类）

```cpp
class ModelBase {
 public:
  virtual bool Init() = 0;
  virtual bool Inference(const std::vector<void*>& input_buffer,
                         unsigned int input_size,
                         std::vector<void*>* output_buffer,
                         unsigned int output_size) = 0;
  virtual bool LoadModel() = 0;
  virtual void Destory() = 0;

  std::string model_path_;
  uint8_t init_ = 0;
};
```

定义在 `modules/prediction/evaluator/model_manager/model/model_base.h`，为所有推理插件提供统一接口。`model_path_` 存储序列化模型文件路径，`init_` 标记初始化状态（0=未初始化，1=已初始化）。

### MultiAgentPedestrianTensorrt

面向行人轨迹预测的多智能体推理插件。使用 Vectornet 架构，输入包含 8 个张量（多障碍物位置序列、向量化地图数据、遮罩等），输出 `max_agent_num`（50）个障碍物的未来 30 步预测轨迹。

**继承关系**：`ModelBase` -> `MultiAgentPedestrianTensorrt`

**关键成员**：

| 成员 | 类型 | 说明 |
|---|---|---|
| `engine_` | `nvinfer1::ICudaEngine*` | TensorRT 推理引擎 |
| `runtime_` | `nvinfer1::IRuntime*` | TensorRT 运行时 |
| `context_` | `nvinfer1::IExecutionContext*` | 推理执行上下文 |
| `buffers_` | `std::vector<void*>` | GPU 设备内存缓冲区 |
| `stream_` | `cudaStream_t` | CUDA 异步传输流 |
| `max_agent_num` | `int` | 最大同时推理障碍物数量，默认 50 |
| `mtx` | `std::mutex` | 推理线程安全互斥锁 |

### MultiAgentVehicleTensorrt

面向车辆轨迹预测的多智能体推理插件。与 `MultiAgentPedestrianTensorrt` 结构和接口完全对称，仅在目标检测类别上不同（车辆 vs 行人）。输入输出张量维度和模型结构一致。

**继承关系**：`ModelBase` -> `MultiAgentVehicleTensorrt`

### SemanticLstmPedestrianTensorrt

面向行人轨迹预测的语义 LSTM 推理插件。输入为 3 个张量（语义分割图像 `3x224x224`、历史位置序列、位置步进序列），输出单个障碍物的未来 30 步预测轨迹。

**继承关系**：`ModelBase` -> `SemanticLstmPedestrianTensorrt`

**输入张量**：

| 名称 | 维度 | 说明 |
|---|---|---|
| `img_tensor` | `[1, 3, 224, 224]` | 语义分割图像 |
| `obstacle_pos` | `[1, 20, 2]` | 历史位置坐标序列（20 步） |
| `obstacle_pos_step` | `[1, 20, 2]` | 位置步进增量 |

### SemanticLstmVehicleTensorrt

面向车辆轨迹预测的语义 LSTM 推理插件。与 `SemanticLstmPedestrianTensorrt` 结构相同，仅目标类别不同。

**继承关系**：`ModelBase` -> `SemanticLstmVehicleTensorrt`

## 核心函数

所有四个插件类实现相同的虚函数接口：

### Init()

```cpp
bool Init();
```

初始化模型，流程如下：

1. 检查 `init_` 标志，避免重复初始化
2. 通过 `abi::__cxa_demangle` 获取运行时类名
3. 调用 `PluginManager::GetPluginConfPath()` 获取配置文件路径 `conf/default_conf.pb.txt`
4. 使用 `GetProtoFromFile()` 解析 `ModelConf` Protobuf 配置，提取 `model_path_`
5. 调用 `LoadModel()` 加载 TensorRT 引擎
6. 设置 `init_ = 1`

### LoadModel()

```cpp
bool LoadModel();
```

加载并反序列化 TensorRT 引擎文件，流程如下：

1. 以二进制模式读取 `model_path_` 指向的 `.engine` 文件
2. 创建 `nvinfer1::IRuntime` 运行时实例
3. 调用 `deserializeCudaEngine()` 反序列化为 CUDA 引擎
4. 创建 `IExecutionContext` 推理上下文
5. 通过 `getBindingIndex()` 查询各输入输出张量的绑定索引
6. 为所有输入输出张量调用 `cudaMalloc()` 分配 GPU 内存
7. 创建 CUDA Stream 用于异步数据传输

**Multi-Agent 模型**分配 9 个缓冲区（8 输入 + 1 输出），**Semantic LSTM 模型**分配 4 个缓冲区（3 输入 + 1 输出）。

### Inference()

```cpp
bool Inference(const std::vector<void*>& input_buffer,
               unsigned int input_size,
               std::vector<void*>* output_buffer,
               unsigned int output_size);
```

执行 GPU 推理，流程如下：

1. 校验输入输出缓冲区数量（Multi-Agent: 8/1, LSTM: 3/1）
2. 若未初始化则调用 `Init()`（懒加载）
3. 加锁 `std::lock_guard<std::mutex>` 保证线程安全
4. 通过 `cudaMemcpyAsync()` 将所有输入数据从主机内存异步拷贝到 GPU
5. 调用 `context_->enqueueV2()` 执行 TensorRT 推理
6. 通过 `cudaMemcpyAsync()` 将推理结果从 GPU 异步拷贝回主机内存
7. `cudaStreamSynchronize()` 等待所有异步操作完成

### Destory()

```cpp
void Destory();
```

释放所有 GPU 资源：销毁 CUDA Stream，释放各输入输出缓冲区的 GPU 内存。在析构函数中自动调用。

## 配置

每个插件通过 `conf/default_conf.pb.txt` 文件配置，使用 `ModelConf` Protobuf 消息解析。配置文件由 Cyber RT 插件管理器按类名自动定位，主要配置项：

| 配置项 | 类型 | 说明 |
|---|---|---|
| `model_path` | `string` | TensorRT 序列化引擎文件的绝对路径 |

**Multi-Agent 模型输入输出尺寸**（基于绑定维度）：

| 张量 | 尺寸 | 字节数（float） |
|---|---|---|
| `multi_obstacle_pos` | `50 x 20 x 2` | 8000 B |
| `multi_obstacle_pos_step` | `50 x 20 x 2` | 8000 B |
| `vector_data` | `450 x 50 x 9` | 810000 B |
| `bool_vector_mask` | `450 x 50` | 22500 B |
| `bool_polyline_mask` | `450` | 450 B |
| `rand_mask` | `450` | 450 B |
| `polyline_id` | `450 x 2` | 3600 B |
| `obs_position` | `50 x 3` | 600 B |
| `predict` (输出) | `50 x 30 x 2` | 12000 B |

**Semantic LSTM 模型输入输出尺寸**：

| 张量 | 尺寸 | 字节数（float） |
|---|---|---|
| `img_tensor` | `3 x 224 x 224` | 602112 B |
| `obstacle_pos` | `20 x 2` | 160 B |
| `obstacle_pos_step` | `20 x 2` | 160 B |
| `predict` (输出) | `30 x 2` | 240 B |

## 调用关系

```text
prediction 评估器 (Evaluator)
  |
  v
ModelBase::Inference()  <-- 纯虚接口，由具体插件实现
  |
  +-- MultiAgentPedestrianTensorrt::Inference()
  |     +-- Init() -> LoadModel()  (懒加载，首次调用时触发)
  |     +-- cudaMemcpyAsync() x8   (H2D: 主机到设备)
  |     +-- enqueueV2()             (TensorRT 推理)
  |     +-- cudaMemcpyAsync() x1   (D2H: 设备到主机)
  |
  +-- MultiAgentVehicleTensorrt::Inference()
  |     +-- [同上]
  |
  +-- SemanticLstmPedestrianTensorrt::Inference()
  |     +-- Init() -> LoadModel()
  |     +-- cudaMemcpyAsync() x3   (H2D)
  |     +-- enqueueV2()
  |     +-- cudaMemcpyAsync() x1   (D2H)
  |
  +-- SemanticLstmVehicleTensorrt::Inference()
        +-- [同上]

配置加载链路:
  Init() -> PluginManager::GetPluginConfPath()
         -> GetProtoFromFile(ModelConf)
         -> LoadModel()
         -> deserializeCudaEngine()
```

**插件注册**：每个插件类通过宏 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(ClassName, ModelBase)` 注册到 Cyber RT 插件管理器，上层评估器通过 `PluginManager` 按类型名动态实例化对应的推理插件。

**线程安全模型**：所有 `Inference()` 调用通过 `std::mutex` 互斥锁串行化，确保同一插件实例的多次并发调用不会产生 GPU 资源竞争。
