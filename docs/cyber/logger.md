---
title: Logger - 日志系统
---

# Logger - 日志系统

## 模块职责概述

Logger 模块是 Apollo Cyber 框架的日志基础设施，构建在 Google glog 之上，提供了按模块名分文件输出和异步写入两大核心增强能力。它通过替换 glog 的默认 Logger 实现，将不同模块的日志分离到独立文件中，同时通过双缓冲异步写入机制大幅降低日志 I/O 对业务线程的性能影响。

## 核心类/接口说明

### Logger（同步日志器）

继承 `google::base::Logger`，实现按模块名分文件的同步日志写入。

```cpp
class Logger : public google::base::Logger {
 public:
  explicit Logger(google::base::Logger* wrapped);
  ~Logger();

  void Write(bool force_flush, time_t timestamp,
             const char* message, int message_len) override;
  void Flush() override;
  uint32_t LogSize() override;

 private:
  google::base::Logger* const wrapped_;
  std::mutex mutex_;
};
```

工作原理：
- `Write()` 被调用时，先通过 `FindModuleName()` 从日志消息中提取模块名（由方括号 `[module_name]` 标记）
- 根据模块名查找或创建对应的 `LogFileObject`，将日志写入模块专属文件
- 文件命名格式：`{module_name}.log.INFO.{timestamp}`
- 使用 mutex 保护模块日志器映射表的并发访问

### AsyncLogger（异步日志器）

继承 `google::base::Logger`，通过双缓冲 + 后台线程实现异步日志写入。

```cpp
class AsyncLogger : public google::base::Logger {
 public:
  explicit AsyncLogger(google::base::Logger* wrapped);
  ~AsyncLogger();

  void Start();
  void Stop();

  void Write(bool force_flush, time_t timestamp,
             const char* message, int message_len) override;
  void Flush() override;
  uint32_t LogSize() override;
};
```

关键设计：
- 双缓冲机制：`active_buf_` 接收应用线程写入，`flushing_buf_` 由后台线程消费
- 后台线程 `RunThread()` 周期性交换两个缓冲区并执行 flush
- 使用 `atomic_flag` 自旋锁（而非 mutex）保护缓冲区交换，最小化锁竞争
- 当缓冲区消息少于 800 条时，后台线程 sleep 1ms 以降低 CPU 占用
- 每条消息携带日志级别（F=3, E=2, W=1, I=0），WARNING 及以上级别触发强制 flush
- 同样支持按模块名分文件输出，内部维护 `module_logger_map_`

消息结构：

```cpp
struct Msg {
  time_t ts;           // 时间戳
  std::string message; // 日志内容
  int level;           // 日志级别
};
```

### LogFileObject

继承 `google::base::Logger`，封装单个日志文件的创建、写入和轮转逻辑。

```cpp
class LogFileObject : public google::base::Logger {
 public:
  LogFileObject(LogSeverity severity, const char* base_filename);
  ~LogFileObject();

  void Write(bool force_flush, time_t timestamp,
             const char* message, int message_len) override;
  void Flush() override;
  uint32 LogSize() override;

  void SetBasename(const char* basename);
  void SetExtension(const char* ext);
  void SetSymlinkBasename(const char* symlink_basename);
};
```

关键行为：
- 日志文件命名：`{base_filename}{YYYYMMDD-HHmmss.pid}`
- 创建文件时同时创建符号链接，便于快速定位最新日志
- 支持按文件大小轮转（由 `FLAGS_max_log_size` 控制）
- 磁盘满时自动停止写入（`FLAGS_stop_logging_if_full_disk`）
- 基于时间和数据量的自动 flush 策略：每 100 万字符或 `FLAGS_logbufsecs` 秒

### logger_util 工具函数

```cpp
// 获取高精度时钟（微秒）
inline int64_t CycleClock_Now();

// 获取主机名
static inline void GetHostName(std::string* hostname);

// 获取主线程 PID
int32_t GetMainThreadPid();

// 检测 PID 是否变化（fork 后场景）
bool PidHasChanged();

// 最大日志文件大小（MB）
inline int32_t MaxLogSize();

// 从日志消息中提取模块名
inline void FindModuleName(std::string* log_message, std::string* module_name);
```

`FindModuleName()` 的工作方式：
1. 在日志消息中查找 `[` 和 `]` 括号对
2. 提取括号内的字符串作为模块名
3. 从原始消息中移除模块名标记
4. 若未找到模块名，使用 `GlobalData::Instance()->ProcessGroup()` 作为默认值

## 数据流描述

```
应用代码 AINFO/AWARN/AERROR/AFATAL 宏
  → glog 格式化日志消息（附加时间、文件、行号等）
  → 调用已注册的 Logger::Write()
  → FindModuleName() 提取模块名
  → 路由到对应模块的 LogFileObject
  → LogFileObject 写入磁盘文件
```

异步模式下的数据流：

```
应用线程 → AsyncLogger::Write() → active_buf_ 追加消息
                                        ↓ (后台线程交换缓冲区)
后台线程 ← flushing_buf_ ← active_buf_
  → FindModuleName() 提取模块名
  → 路由到对应模块的 LogFileObject
  → LogFileObject 写入磁盘文件
```

## 配置方式

Logger 模块主要通过 glog 的 FLAGS 进行配置：

| 参数 | 说明 |
|------|------|
| `FLAGS_log_dir` | 日志输出目录 |
| `FLAGS_max_log_size` | 单个日志文件最大大小（MB） |
| `FLAGS_stop_logging_if_full_disk` | 磁盘满时停止写入 |
| `FLAGS_logbufsecs` | 日志缓冲刷新间隔（秒） |
| `FLAGS_minloglevel` | 最低日志级别 |

日志级别对应关系：

| 级别 | glog 值 | 宏 |
|------|---------|-----|
| INFO | 0 | `AINFO` |
| WARNING | 1 | `AWARN` |
| ERROR | 2 | `AERROR` |
| FATAL | 3 | `AFATAL` |

## 与其他模块的关系

- **glog**：Logger 模块是 glog 的扩展层，通过实现 `google::base::Logger` 接口替换 glog 的默认日志后端。所有 `AINFO`、`AERROR` 等宏最终通过 glog 的日志流机制触发 `Write()` 调用
- **common/global_data**：当日志消息中未标记模块名时，使用 `GlobalData::Instance()->ProcessGroup()` 作为默认模块名
- **common/log.h**：定义了 `AINFO`、`AWARN`、`AERROR`、`AFATAL`、`ACHECK` 等宏，这些宏在日志消息中插入 `[module_name]` 标记
- **Cyber Init**：在 `cyber::Init()` 中完成 Logger 的初始化和 glog Logger 的替换
