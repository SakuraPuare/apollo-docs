---
title: "预测工具"
---

# Prediction Tools

> 源码路径：`modules/tools/prediction/`

## 概述

预测模块的离线训练和评估工具集（Python），用于训练预测模型、评估预测精度、数据预处理等。

## 子目录

| 目录/文件 | 说明 |
| --------- | ---- |
| `prediction_eval.py` | 预测轨迹评估脚本，计算预测与真值的平均误差 |
| `data_pipelines/` | 数据预处理和模型训练管线 |
| `data_pipelines/cruiseMLP_train.py` | Cruise MLP 模型训练 |
| `data_pipelines/junctionMLP_train.py` | Junction MLP 模型训练 |
| `data_pipelines/mlp_train.py` | 通用 MLP 模型训练 |
| `data_pipelines/merge_h5.py` | HDF5 数据合并 |
| `multiple_gpu_estimator/` | 多 GPU 训练工具 |
| `fake_prediction/` | 假预测数据生成（测试用） |

## 核心函数

### prediction_eval.py::cal_diff_avg()

```python
def cal_diff_avg(dt, gt):
    """计算预测轨迹与真值轨迹各点的平均欧氏距离误差"""
    for i in range(traj_len):
        point_error = ((dt[i][0] - gt[i][0])**2 + (dt[i][1] - gt[i][1])**2)**0.5
        sum += point_error
    return [sum, point_num]
```

## 调用关系

- **输入**：预测模块产生的轨迹数据（record 文件或 HDF5）
- **输出**：训练好的模型文件、评估报告
