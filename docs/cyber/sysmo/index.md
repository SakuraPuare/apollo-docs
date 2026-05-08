---
title: "Sysmo"
---

# Sysmo

> 源码路径: `cyber/sysmo/`

## 概述

SysMo（System Monitor）是 Cyber RT 框架中的系统监控模块。它以独立线程定期轮询调度器的运行状态，检测调度异常。模块通过环境变量 `sysmo_start` 控制是否启动，为单例模式，生命周期与 Cyber 进程一致。

## 核心类

### SysMo

系统监控单例类，内部维护一个周期性运行的检测线程。

```cpp
class SysMo {
 public:
  void Start();
  void Shutdown();

 private:
  void Checker();

  std::atomic<bool> shut_down_{false};
  bool start_ = false;
  int sysmo_interval_ms_ = 100;
  std::condition_variable cv_;
  std::mutex lk_;
  std::thread sysmo_;

  DECLARE_SINGLETON(SysMo);
};
```

- `shut_down_`：原子布尔标志，用于通知检测线程退出
- `start_`：标记模块是否已启动
- `sysmo_interval_ms_`：检测间隔，默认 100 毫秒
- `cv_` / `lk_`：条件变量与互斥锁，配合实现可中断的定时等待
- `sysmo_`：检测线程对象

## 核心函数

### SysMo::Start()

```cpp
void SysMo::Start() {
  auto sysmo_start = GetEnv("sysmo_start");
  if (sysmo_start != "" && std::stoi(sysmo_start)) {
    start_ = true;
    sysmo_ = std::thread(&SysMo::Checker, this);
  }
}
```

- **职责**：根据环境变量决定是否启动监控线程
- **输入**：无（从环境变量 `sysmo_start` 读取配置）
- **输出**：无
- **关键步骤**：
  1. 调用 `GetEnv("sysmo_start")` 读取环境变量
  2. 若变量非空且转换为整数后非零，设置 `start_` 为 true
  3. 创建 `sysmo_` 线程，执行 `Checker()` 循环

### SysMo::Shutdown()

```cpp
void SysMo::Shutdown() {
  if (!start_ || shut_down_.exchange(true)) {
    return;
  }
  cv_.notify_all();
  if (sysmo_.joinable()) {
    sysmo_.join();
  }
}
```

- **职责**：安全停止监控线程
- **输入**：无
- **输出**：无
- **关键步骤**：
  1. 若模块未启动或已被标记关闭，直接返回
  2. 原子设置 `shut_down_` 为 true，防止重复关闭
  3. 通知条件变量唤醒等待中的线程
  4. 等待 `sysmo_` 线程结束

### SysMo::Checker()

```cpp
void SysMo::Checker() {
  while (cyber_unlikely(!shut_down_.load())) {
    scheduler::Instance()->CheckSchedStatus();
    std::unique_lock<std::mutex> lk(lk_);
    cv_.wait_for(lk, std::chrono::milliseconds(sysmo_interval_ms_));
  }
}
```

- **职责**：周期性检查调度器状态
- **输入**：无
- **输出**：无
- **关键步骤**：
  1. 在循环中判断 `shut_down_` 标志
  2. 调用调度器单例的 `CheckSchedStatus()` 检查调度状态
  3. 通过 `condition_variable::wait_for` 等待 100ms 或被 `Shutdown` 唤醒

## 配置

| 环境变量 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `sysmo_start` | string | 空（不启动） | 设为非零值时启用系统监控 |

## 调用关系

```text
Cyber Init
  -> SysMo() 构造函数
    -> Start()
      -> GetEnv("sysmo_start")
      -> std::thread(Checker)  [若环境变量启用]

Checker() 循环:
  -> scheduler::Instance()->CheckSchedStatus()

Cyber Shutdown
  -> SysMo::Shutdown()
    -> cv_.notify_all()
    -> sysmo_.join()
```
