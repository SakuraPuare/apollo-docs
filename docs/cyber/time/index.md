---
title: "Time"
---

# Time

> 源码路径: `cyber/time/`

## 概述

`cyber/time` 模块为 Cyber RT 框架提供统一的时间抽象，包含四个核心组件：

- **Duration** -- 时间段，以纳秒为内部存储，支持算术运算和线程休眠
- **Time** -- 时间点，基于 `std::chrono` 高精度时钟，支持多种构造和格式转换
- **Clock** -- 全局单例时钟，支持系统时钟（CYBER）和模拟时钟（MOCK）两种模式切换
- **Rate** -- 循环频率控制器，用于按固定频率执行周期性任务

该模块是 Cyber RT 调度、定时器、消息时间戳等子系统的基础设施。

## 核心类

### Duration

以纳秒（`int64_t`）为内部单位的时间段表示，支持四则运算和休眠。

```cpp
class Duration {
 public:
  explicit Duration(int64_t nanoseconds);
  explicit Duration(double seconds);
  Duration(uint32_t seconds, uint32_t nanoseconds);

  double ToSecond() const;
  int64_t ToNanosecond() const;
  bool IsZero() const;
  void Sleep() const;  // 调用 std::this_thread::sleep_for

  Duration operator+(const Duration &rhs) const;
  Duration operator-(const Duration &rhs) const;
  Duration operator*(double scale) const;
  // ... 全套比较运算符

 private:
  int64_t nanoseconds_ = 0;
};
```

### Time

以纳秒（`uint64_t`）为内部单位的时间点表示，提供静态工厂方法获取当前时间。

```cpp
class Time {
 public:
  static const Time MAX;
  static const Time MIN;

  explicit Time(uint64_t nanoseconds);
  explicit Time(double seconds);
  Time(uint32_t seconds, uint32_t nanoseconds);

  static Time Now();       // high_resolution_clock，挂钟时间
  static Time MonoTime();  // steady_clock，单调递增不受系统调时影响
  static void SleepUntil(const Time& time);

  double ToSecond() const;
  uint64_t ToMicrosecond() const;
  uint64_t ToNanosecond() const;
  std::string ToString() const;  // 格式: "YYYY-MM-DD HH:MM:SS.nnnnnnnnn"
  bool IsZero() const;

  Duration operator-(const Time& rhs) const;
  Time operator+(const Duration& rhs) const;
  Time operator-(const Duration& rhs) const;
  // ... 全套比较运算符

 private:
  uint64_t nanoseconds_ = 0;
};
```

### Clock

全局单例时钟，根据运行模式决定时间来源。使用 `AtomicRWLock` 保证线程安全。

```cpp
class Clock {
 public:
  static constexpr int64_t PRECISION =
      std::chrono::system_clock::duration::period::den /
      std::chrono::system_clock::duration::period::num;
  // 编译期断言：系统时钟精度不低于 1 微秒

  static Time Now();
  static double NowInSeconds();
  static void SetNow(const Time& now);       // 仅 MOCK 模式有效
  static void SetMode(ClockMode mode);
  static ClockMode mode();
  static void SetNowInSeconds(const double seconds);

 private:
  ClockMode mode_;
  Time mock_now_;
  base::AtomicRWLock rwlock_;
  DECLARE_SINGLETON(Clock)
};
```

### Rate

循环频率控制器，参考 ROS `Rate` 设计（BSD 许可）。在每次循环中计算休眠时间，保持稳定的执行频率。

```cpp
class Rate {
 public:
  explicit Rate(double frequency);        // 频率 (Hz)
  explicit Rate(uint64_t nanoseconds);    // 周期 (纳秒)
  explicit Rate(const Duration&);         // 周期

  void Sleep();            // 休眠至下一周期
  void Reset();            // 重置起始时间为当前时间
  Duration CycleTime() const;           // 实际上一周期耗时
  Duration ExpectedCycleTime() const;   // 预期周期时长

 private:
  Time start_;
  Duration expected_cycle_time_;
  Duration actual_cycle_time_;
};
```

## 核心函数

### Time::Now()

| 项目 | 说明 |
|------|------|
| 职责 | 获取当前系统挂钟时间 |
| 输入 | 无 |
| 输出 | `Time` 对象，内部存储自 epoch 以来的纳秒数 |
| 关键步骤 | 调用 `high_resolution_clock::now()`，转换为纳秒 `time_point`，取 `time_since_epoch().count()` |

### Time::MonoTime()

| 项目 | 说明 |
|------|------|
| 职责 | 获取单调递增时间，不受系统时间调整影响 |
| 输入 | 无 |
| 输出 | `Time` 对象 |
| 关键步骤 | 调用 `steady_clock::now()`，转换为纳秒 |

### Time::ToString()

| 项目 | 说明 |
|------|------|
| 职责 | 将时间点格式化为可读字符串 |
| 输入 | 无 |
| 输出 | `"YYYY-MM-DD HH:MM:SS.nnnnnnnnn"` 格式字符串 |
| 关键步骤 | 纳秒转 `system_clock::time_point`，`localtime_r` 转本地时间，`put_time` 格式化日期时间，拼接纳秒尾数 |

### Clock::Now()

| 项目 | 说明 |
|------|------|
| 职责 | 根据当前时钟模式返回时间 |
| 输入 | 无 |
| 输出 | `Time` 对象 |
| 关键步骤 | 获取单例实例，加读锁，判断 `mode_`：`MODE_CYBER` 调用 `Time::Now()`，`MODE_MOCK` 返回 `mock_now_` |

### Clock::SetNow()

| 项目 | 说明 |
|------|------|
| 职责 | 设置模拟时钟的当前时间（仅 MOCK 模式） |
| 输入 | `const Time& now` |
| 输出 | 无 |
| 关键步骤 | 获取单例，加写锁，检查 `mode_ == MODE_MOCK`，满足则更新 `mock_now_`，否则输出错误日志并返回 |

### Rate::Sleep()

| 项目 | 说明 |
|------|------|
| 职责 | 休眠到下一周期结束，维持稳定频率 |
| 输入 | 无 |
| 输出 | 无 |
| 关键步骤 | 1. 计算 `expected_end = start_ + expected_cycle_time_`；2. 检测时间回跳并修正；3. 计算剩余休眠时间；4. 若已超时则不休眠（若超时超过一个完整周期则重置 `start_`）；5. 调用 `Time::SleepUntil(expected_end)`；6. 更新 `start_` 为 `expected_end` |

## 配置

| 配置项 | 来源 | 说明 |
|--------|------|------|
| `clock_mode` | `run_mode_conf` (protobuf) | `Clock` 构造时读取，决定初始时钟模式：`MODE_CYBER`（系统时钟）或 `MODE_MOCK`（模拟时钟） |

## 调用关系

```text
Clock::Now()
  |-- MODE_CYBER --> Time::Now()
  |                    |-- high_resolution_clock::now()
  |-- MODE_MOCK  --> 返回内部 mock_now_

Clock::SetNow()
  |-- 检查 mode_ == MODE_MOCK
  |-- 更新 mock_now_

Rate::Sleep()
  |-- Time::Now()              -- 获取当前时间
  |-- Time::SleepUntil()       -- 休眠到目标时间
  |      |-- std::this_thread::sleep_until()

Duration::Sleep()
  |-- std::this_thread::sleep_for()

Time::ToString()
  |-- system_clock::to_time_t()
  |-- localtime_r()
  |-- put_time / strftime
```

整体依赖链：`Duration` 为基础类型，`Time` 依赖 `Duration`（算术运算返回 `Duration`），`Clock` 依赖 `Time`，`Rate` 同时依赖 `Time` 和 `Duration`。
