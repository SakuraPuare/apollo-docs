---
title: "预测神经网络"
---

# Network

> 源码路径: `modules/prediction/network/`

## 概述

network 模块实现了一个轻量级的 C++ 神经网络推理引擎，供预测模块（prediction）在运行时加载预训练模型并执行前向推理。模块不依赖任何第三方深度学习框架，所有层的计算均基于 Eigen 矩阵库原生实现。

整体架构分为三层：

- **NetModel** -- 网络模型容器，从 Protobuf 反序列化完整网络结构，按顺序驱动各层推理
- **Layer 及其派生类** -- 各种网络层（全连接、卷积、池化、LSTM 等），统一通过 `Run()` 接口执行前向计算
- **net_util** -- 激活函数、张量加载、矩阵展平等工具函数

网络参数以 `NetParameter` / `LayerParameter`（Protobuf）形式存储，运行时由 `NetModel::LoadModel` 解析并构建层对象链。

## 核心类

### Layer

所有网络层的基类，定义了统一的接口契约。

```cpp
class Layer {
 public:
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void ResetState() {}
  virtual void SetState(const std::vector<Eigen::MatrixXf>& states) {}
  virtual void State(std::vector<Eigen::MatrixXf>* states) const {}
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output) = 0;
  std::string Name() const;
  int OrderNumber() const;
};
```

### 派生层类型

| 类名 | 功能 | 计算公式 |
|---|---|---|
| **Dense** | 全连接层 | `y = activation(x * W + b)` |
| **Conv1d** | 一维卷积层 | `y = Conv(x, kernel) + bias` |
| **MaxPool1d** | 一维最大池化 | 窗口内取最大值 |
| **AvgPool1d** | 一维平均池化 | 窗口内取均值 |
| **Activation** | 独立激活层 | `y = f(x)` |
| **BatchNormalization** | 批归一化 | `y = (x - mu) / sqrt(sigma + eps) * gamma + beta` |
| **LSTM** | 长短时记忆单元 | 标准 LSTM 四门机制（输入/遗忘/细胞/输出门） |
| **Flatten** | 展平层 | 将矩阵展平为行向量 |
| **Input** | 输入层 | 校验输入形状，直接透传 |
| **Concatenate** | 拼接层 | 沿列方向拼接两个输入矩阵 |

### LSTM 状态管理

LSTM 是唯一具有内部状态的层，维护隐藏状态 `ht_1_` 和细胞状态 `ct_1_`：

```cpp
// 逐步计算 LSTM，更新隐藏状态和细胞状态
void LSTM::Step(const Eigen::MatrixXf& input, Eigen::MatrixXf* output,
                Eigen::MatrixXf* ht_1, Eigen::MatrixXf* ct_1);
```

关键配置项：

- `return_sequences` -- 是否返回完整序列输出（否则仅返回最后一步）
- `stateful` -- 是否跨 batch 保持状态
- `unit_forget_bias` -- 是否使用单位遗忘偏置

### NetModel

网络模型容器，持有 `Layer` 对象链，负责从 Protobuf 加载模型并执行顺序推理。

```cpp
class NetModel {
 public:
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output) const = 0;
  virtual void SetState(const std::vector<Eigen::MatrixXf>& states) {}
  virtual void State(std::vector<Eigen::MatrixXf>* states) const {}
  virtual void ResetState() const {}
  bool LoadModel(const NetParameter& net_parameter);
  std::string PerformanceString() const;
  const std::string& Name() const;
  int Id() const;
  bool IsOk() const;
 protected:
  std::vector<std::unique_ptr<Layer>> layers_;
  NetParameter net_parameter_;
  bool ok_ = false;
};
```

`Run` 为纯虚函数，需由具体预测模型子类实现（例如循环驱动各层依次推理）。

## 核心函数

### net_util 工具函数

| 函数 | 说明 |
|---|---|
| `sigmoid(x)` | Sigmoid 激活: `1 / (1 + exp(-x))` |
| `tanh(x)` | 双曲正切激活 |
| `linear(x)` | 线性激活: `f(x) = x` |
| `hard_sigmoid(x)` | 硬 Sigmoid: 分段线性近似 |
| `relu(x)` | ReLU 激活: `max(0, x)` |
| `FlattenMatrix(matrix)` | 将矩阵按行优先展平为 `1 x N` 行向量 |
| `serialize_to_function(str)` | 将字符串名称映射为激活函数对象 |
| `LoadTensor(pb, matrix)` | 从 `TensorParameter` 加载 2D 矩阵 |
| `LoadTensor(pb, vector)` | 从 `TensorParameter` 加载 1D 向量 |
| `LoadTensor(pb, tensor3d)` | 从 `TensorParameter` 加载 3D 张量（`vector<MatrixXf>`） |

`serialize_to_function` 内部维护静态映射表，支持的激活函数名称为：`"linear"`、`"tanh"`、`"sigmoid"`、`"hard_sigmoid"`、`"relu"`。

### NetModel::LoadModel

```cpp
bool NetModel::LoadModel(const NetParameter& net_parameter);
```

遍历 `NetParameter` 中的 `layers` 字段，根据 `oneof_layers_case()` 枚举值创建对应的 `Layer` 派生类实例，依次调用 `Layer::Load` 加载参数，最终将所有层按顺序存入 `layers_` 向量。加载成功后设置 `ok_ = true`。

## 配置

模型配置通过 Protobuf 消息定义，主要包括：

- `NetParameter` -- 顶层网络参数，包含网络名称、ID、层列表和性能指标（accuracy / recall / precision）
- `LayerParameter` -- 层参数容器，通过 `oneof` 机制携带具体层类型参数（`DenseParameter`、`Conv1dParameter`、`LSTMParameter` 等）
- `TensorParameter` -- 张量数据载体，包含 `shape`（维度列表）和 `data`（浮点数值列表）

LSTM 层的主要配置项：

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `units` | int | 必填 | LSTM 单元数 |
| `return_sequences` | bool | false | 是否返回全部时间步输出 |
| `stateful` | bool | false | 是否跨 batch 保持状态 |
| `use_bias` | bool | true | 是否使用偏置 |
| `unit_forget_bias` | bool | true | 是否初始化遗忘门偏置为 1 |
| `activation` | string | `"tanh"` | 细胞状态激活函数 |
| `recurrent_activation` | string | `"hard_tanh"` | 门控激活函数 |

## 调用关系

```text
预测器 (Predictor)
  |
  v
NetModel::LoadModel(NetParameter)
  |-- 解析 Protobuf，创建 Layer 派生类实例
  |-- Layer::Load(LayerParameter)
  |     |-- 加载权重、偏置等参数 (LoadTensor)
  |     |-- 设置激活函数 (serialize_to_function)
  |
  v
NetModel::Run(inputs, output)
  |-- 顺序调用各 Layer::Run()
        |-- Dense::Run        -- 全连接前向计算
        |-- Conv1d::Run       -- 卷积前向计算
        |-- MaxPool1d::Run    -- 最大池化
        |-- AvgPool1d::Run    -- 平均池化
        |-- Activation::Run   -- 激活函数应用
        |-- BatchNormalization::Run -- 批归一化
        |-- LSTM::Run         -- LSTM 逐步推理 (Step)
        |     |-- Step()      -- 单步 LSTM 门控计算
        |-- Flatten::Run      -- 矩阵展平
        |-- Concatenate::Run  -- 输入拼接
```

所有层均通过 Eigen 矩阵运算完成前向计算，模型参数以 Protobuf 二进制格式持久化，推理过程无动态内存分配（参数在 `Load` 阶段一次性加载）。
