---
title: "通用滤波器"
---

# 通用滤波器

> 源码路径: `modules/common/filters/`

## 概述

`filters` 模块提供自动驾驶系统中常用的数字信号滤波器实现，用于平滑传感器数据和控制信号。模块包含三部分：

- **DigitalFilter** — 二阶 IIR 数字滤波器，基于传递函数的分母/分子系数实现，支持死区（dead zone）抑制小幅抖动。
- **MeanFilter** — 滑动窗口截断均值滤波器，自动去除窗口内的最大值和最小值后取平均，减少异常值影响。
- **滤波器系数生成** — 提供二阶低通（Butterworth）和一阶低通（ZOH）系数计算函数，可直接用于配置 DigitalFilter。

## 核心类

### DigitalFilter

IIR 数字滤波器，内部维护输入队列 `x_values_` 和输出队列 `y_values_`（均为 `std::deque`，前端为最新值）。滤波方程：

```text
den[0] * y + den[1] * y[0] + ... + den[n-1] * y[n-2]
  = num[0] * x[0] + num[1] * x[1] + ... + num[n-1] * x[n-1]
```

主要成员变量：

```cpp
std::deque<double> x_values_;      // 输入历史，前端最新
std::deque<double> y_values_;      // 输出历史，前端最新
std::vector<double> denominators_; // 分母系数（y 侧）
std::vector<double> numerators_;   // 分子系数（x 侧）
double dead_zone_ = 0.0;          // 死区阈值
double last_ = 0.0;               // 上一次滤波输出
```

### MeanFilter

滑动窗口截断均值滤波器。维护窗口大小为 `window_size_` 的数据序列，计算平均值时自动去除最大值和最小值（视为离群点）。使用单调队列在 O(1) 均摊时间内维护极值，适合高频实时信号处理。

主要成员变量：

```cpp
std::uint_fast8_t window_size_ = 0;  // 窗口大小
double sum_ = 0.0;                    // 窗口内元素总和
std::uint_fast8_t time_ = 0;         // 逻辑时间戳（环形）
std::deque<double> values_;           // 滑动窗口数据
std::deque<std::pair<std::uint_fast8_t, double>> min_candidates_; // 最小值单调队列
std::deque<std::pair<std::uint_fast8_t, double>> max_candidates_; // 最大值单调队列
```

## 核心函数

### DigitalFilter

| 函数 | 说明 |
|---|---|
| `DigitalFilter(denominators, numerators)` | 构造函数，传入分母和分子系数 |
| `Filter(x_insert)` | 核心滤波函数。将新输入 `x_insert` 推入输入队列，利用差分方程计算输出并推入输出队列。经过 `UpdateLast` 死区判断后返回最终输出 |
| `set_coefficients(denominators, numerators)` | 同时设置分母和分子系数，调整队列大小 |
| `set_dead_zone(deadzone)` | 设置死区阈值，输出变化量小于阈值时保持上一次输出不变 |
| `reset_values()` | 将输入和输出队列全部清零 |
| `denominators() / numerators()` | 获取当前系数 |
| `inputs_queue() / outputs_queue()` | 获取输入/输出历史队列 |

私有方法：

| 函数 | 说明 |
|---|---|
| `UpdateLast(input)` | 若 `input - last_` 的绝对值小于 `dead_zone_` 则返回 `last_`，否则更新并返回 |
| `Compute(values, coefficients, coeff_start, coeff_end)` | 计算系数与值序列的内积，用于差分方程求解 |

### MeanFilter

| 函数 | 说明 |
|---|---|
| `MeanFilter(window_size)` | 构造函数，指定窗口大小（范围 1 ~ 127） |
| `Update(measurement)` | 核心滤波函数。若窗口已满则移除最早数据，插入新数据。窗口元素大于 2 时返回 `(sum - min - max) / (size - 2)`，否则返回简单均值 |

### 滤波器系数生成

| 函数 | 说明 |
|---|---|
| `LpfCoefficients(ts, cutoff_freq, denominators, numerators)` | 计算二阶 Butterworth 低通滤波器系数。`ts` 为采样周期，`cutoff_freq` 为截止频率（Hz） |
| `LpFirstOrderCoefficients(ts, settling_time, dead_time, denominators, numerators)` | 计算一阶 ZOH 低通滤波器系数。`settling_time` 为稳定时间，`dead_time` 为延迟时间 |

## 配置

DigitalFilter 的核心配置项为滤波器系数（分母/分子向量）和死区阈值。系数可通过 `LpfCoefficients` 或 `LpFirstOrderCoefficients` 函数生成：

```cpp
// 生成二阶低通系数示例
std::vector<double> den, num;
LpfCoefficients(0.01, 10.0, &den, &num);  // 10ms 采样，10Hz 截止

// 创建滤波器并应用
DigitalFilter filter(den, num);
filter.set_dead_zone(0.05);
double output = filter.Filter(raw_signal);
```

MeanFilter 通过构造函数指定窗口大小：

```cpp
MeanFilter mean_filter(10);  // 窗口大小 10
double smoothed = mean_filter.Update(noisy_value);
```

## 调用关系

```text
LpfCoefficients / LpFirstOrderCoefficients
       |
       v  (生成系数)
DigitalFilter::set_coefficients
       |
       v
DigitalFilter::Filter(x_insert)
       |
       +---> Compute(x_values_, numerators_)  // 计算 x 侧内积
       +---> Compute(y_values_, denominators_) // 计算 y 侧内积
       +---> 求解 y_insert = (xside - yside) / den[0]
       +---> UpdateLast(y_insert)              // 死区判断
       |
       v
     输出滤波结果

MeanFilter::Update(measurement)
       |
       +---> RemoveEarliest()  // 窗口满时移除最早数据
       +---> Insert(value)     // 插入新数据，维护单调队列
       +---> 计算截断均值
       |
       v
     输出滤波结果
```
