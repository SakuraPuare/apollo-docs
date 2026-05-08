---
title: "延迟统计"
---

# Latency Statistic

> 源码路径：`modules/statistics/latency/`

## 概述

延迟统计组件，定时采集自动驾驶核心链路（Compensator → Perception → Prediction → Planning → Control）的端到端延迟和处理延迟，计算统计指标并发布。

## 核心类

### LatencyStatisticComponent

```cpp
class LatencyStatisticComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
 private:
  void CreateReaders();
  void FillResult(const uint64_t& lidar_timestamp, const Metric& metric, Result* result);
  void CalculateLatency(const Result& result, std::shared_ptr<LatencyMetrics> metrics);
  void GetLatencyMetric(const std::vector<uint64_t>& items, LatencyMetrics_Metric* metric);

  std::map<uint64_t, Metric> latency_map_;
  std::shared_ptr<cyber::Writer<LatencyMetrics>> metric_writer_;
};
```

### 辅助结构体

- `Metric`：记录单帧各模块的处理时间戳（compensator/perception/prediction/planning/control）
- `Result`：汇总多帧的链路延迟和处理延迟列表，用于统计计算

## 核心函数

### LatencyStatisticComponent::Proc()

**职责**：定时触发，从 `latency_map_` 中提取已完成的帧数据，计算各模块延迟统计值并发布

### LatencyStatisticComponent::CalculateLatency()

**职责**：对 Result 中各模块的延迟列表计算 min/max/mean/p99 等统计指标

## 调用关系

- **输入**：订阅 compensator/perception/prediction/planning/control 各话题
- **输出**：发布 `LatencyMetrics` 统计消息
