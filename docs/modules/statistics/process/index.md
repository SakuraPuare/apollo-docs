---
title: "进程统计"
---

# Process Statistic

> 源码路径：`modules/statistics/process/`

## 概述

进程统计组件，定时扫描系统中运行的进程，解析 mainboard 进程的 DAG 文件和进程名，采集进程级指标并发布。

## 核心类

### ProcessStatisticComponent

```cpp
class ProcessStatisticComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
 private:
  void LoadConf(const ProcessStatisticConf& conf);
  bool ParseFromCache(const std::string& cmd_string, std::shared_ptr<ProcessMetrics> metrics);
  void CleanCache();
  void ParseCmd(const std::string& cmd_string,
                std::shared_ptr<ProcessMetrics> metrics,
                std::shared_ptr<CacheMetric> cache_metric);

  std::vector<std::regex> general_process_regex_list_;
  std::unordered_map<std::string, std::shared_ptr<CacheMetric>> cache_;
  std::shared_ptr<cyber::Writer<ProcessMetrics>> metric_writer_;
};
```

### CacheMetric

```cpp
struct CacheMetric {
  uint64_t timestamp;
  std::vector<std::string> mainboard_dags;
  std::string mainboard_process;
  std::string general_process;
};
```

## 核心逻辑

- 通过正则 `-d ([\w/\.-]+)` 从命令行提取 mainboard 加载的 DAG 文件
- 通过正则 `-p ([\w-]+)` 提取进程名
- 使用缓存避免重复解析，每 60 个周期清理一次

## 调用关系

- **输入**：读取 `/proc` 文件系统获取进程信息
- **输出**：发布 `ProcessMetrics` 消息
