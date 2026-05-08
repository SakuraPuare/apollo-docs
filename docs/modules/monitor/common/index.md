---
title: "监控公共框架"
---

# Monitor Common

> 源码路径：`modules/monitor/common/`

## 概述

Monitor Common 提供监控系统的核心基础设施：`MonitorManager` 单例管理监控帧的生命周期和系统状态，`RecurrentRunner` 提供定时执行的抽象基类供各监控检查器继承。

## 核心类

### MonitorManager

集中管理监控配置和状态的单例类。

```cpp
class MonitorManager {
 public:
  void Init(const std::shared_ptr<apollo::cyber::Node>& node);
  bool StartFrame(const double current_time);
  void EndFrame();

  const apollo::dreamview::HMIMode& GetHMIMode() const;
  bool IsInAutonomousMode() const;
  SystemStatus* GetStatus();
  apollo::common::monitor::MonitorLogBuffer& LogBuffer();

  template <class T>
  std::shared_ptr<cyber::Reader<T>> CreateReader(const std::string& channel);
  template <class T>
  std::shared_ptr<cyber::Writer<T>> CreateWriter(const std::string& channel);

  DECLARE_SINGLETON(MonitorManager)
};
```

### MonitorManager::StartFrame()

```cpp
bool MonitorManager::StartFrame(const double current_time) {
  static auto hmi_status_reader =
      CreateReader<apollo::dreamview::HMIStatus>(FLAGS_hmi_status_topic);
  hmi_status_reader->Observe();
  const auto hmi_status = hmi_status_reader->GetLatestObserved();
  if (hmi_status == nullptr) { return false; }

  if (current_mode_ != hmi_status->current_mode()) {
    current_mode_ = hmi_status->current_mode();
    mode_config_ = apollo::dreamview::util::HMIUtil::LoadMode(...);
    // 重新初始化 hmi_modules、components、other_components、global_components
  }
  in_autonomous_driving_ = CheckAutonomousDriving(current_time);
  return true;
}
```

**职责**：开始一个监控帧，读取最新 HMI 状态，检测模式切换并更新监控组件列表

### MonitorManager::CheckAutonomousDriving()

**职责**：通过读取 Chassis 消息判断当前是否处于自动驾驶模式

**关键逻辑**：

1. 仿真模式（`use_sim_time`）返回 false
2. SimControl 模块返回 false
3. 过期消息（超过 `system_status_lifetime_seconds`）返回 false
4. 仅当 `driving_mode == COMPLETE_AUTO_DRIVE` 时返回 true

### RecurrentRunner

定时执行的抽象基类，所有监控检查器继承此类。

```cpp
class RecurrentRunner {
 public:
  RecurrentRunner(const std::string& name, const double interval);
  void Tick(const double current_time);
  virtual void RunOnce(const double current_time) = 0;
 protected:
  std::string name_;
  unsigned int round_count_ = 0;
};
```

### RecurrentRunner::Tick()

```cpp
void RecurrentRunner::Tick(const double current_time) {
  if (name_ == "ProcessMonitor" &&
      MonitorManager::Instance()->GetStatus()->detect_immediately()) {
    RunOnce(current_time);
  } else if (next_round_ <= current_time) {
    next_round_ = current_time + interval_;
    RunOnce(current_time);
  }
}
```

**职责**：按设定间隔调用 `RunOnce()`，`ProcessMonitor` 在需要立即检测时跳过间隔限制

## 调用关系

- **被调用方**：`MonitorComponent` 主循环调用 `StartFrame()` → 各 Runner 的 `Tick()` → `EndFrame()`
- **子类**：`HardwareMonitor`、`ProcessMonitor`、`ResourceMonitor` 等继承 `RecurrentRunner`
- **依赖**：HMI 状态话题、Chassis 话题、DreamView 配置工具
