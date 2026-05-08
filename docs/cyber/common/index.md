---
title: "Common 基础工具库"
---

# Common

> 源码路径：`cyber/common/`

## 概述

Common 是 CyberRT 的基础工具库，提供日志、文件操作、全局数据管理、时间转换、宏定义等底层工具。几乎所有 CyberRT 组件都依赖此模块。

## 架构

| 文件 | 职责 |
| --- | --- |
| `log.h` | 日志宏，封装 glog |
| `file.h` / `file.cc` | 文件与 protobuf 序列化工具 |
| `global_data.h` / `global_data.cc` | 全局单例，管理进程、主机、配置信息 |
| `macros.h` | 通用宏（单例声明、拷贝禁止等） |
| `environment.h` | 环境变量与工作路径 |
| `time_conversion.h` | UNIX / GPS 时间互转 |
| `types.h` | 基础类型定义（ReturnCode、Relation） |
| `util.h` | 哈希与枚举转整型工具 |

## 核心类

### GlobalData

`cyber/common/global_data.h` + `cyber/common/global_data.cc`

全局单例（通过 `DECLARE_SINGLETON` 宏），管理 CyberRT 运行时的全局状态：

```cpp
class GlobalData {
 public:
  int ProcessId() const;

  void SetProcessGroup(const std::string& process_group);
  const std::string& ProcessGroup() const;

  void SetSchedName(const std::string& sched_name);
  const std::string& SchedName() const;

  const std::string& HostIp() const;
  const std::string& HostName() const;

  const CyberConfig& Config() const;

  void EnableSimulationMode();
  void DisableSimulationMode();
  bool IsRealityMode() const;
  bool IsMockTimeMode() const;

  // 名称注册与查询（基于 AtomicHashMap，线程安全）
  static uint64_t RegisterNode(const std::string& node_name);
  static std::string GetNodeById(uint64_t id);

  static uint64_t RegisterChannel(const std::string& channel);
  static std::string GetChannelById(uint64_t id);

  static uint64_t RegisterService(const std::string& service);
  static std::string GetServiceById(uint64_t id);

  static uint64_t GenerateHashId(const std::string& name);
};
```

## 核心函数

### GlobalData::GlobalData()

`cyber/common/global_data.cc:53`

```cpp
GlobalData::GlobalData() {
  InitHostInfo();
  ACHECK(InitConfig());
  process_id_ = getpid();
  auto prog_path = program_path();
  if (!prog_path.empty()) {
    process_group_ = GetFileName(prog_path) + "_" + std::to_string(process_id_);
  } else {
    process_group_ = "cyber_default_" + std::to_string(process_id_);
  }
  run_mode_ = config_.run_mode_conf().run_mode();
  clock_mode_ = config_.run_mode_conf().clock_mode();
}
```

**职责**：初始化全局数据
**关键步骤**：

1. `InitHostInfo()` — 获取主机名和 IP（优先使用 `CYBER_IP` 环境变量，否则遍历网卡找非回环地址）
2. `InitConfig()` — 从 `CYBER_PATH/conf/cyber.pb.conf` 加载配置
3. 设置进程组名为 `<程序名>_<PID>`

### GlobalData::RegisterNode()

`cyber/common/global_data.cc:213`

```cpp
uint64_t GlobalData::RegisterNode(const std::string& node_name) {
  auto id = Hash(node_name);
  while (node_id_map_.Has(id)) {
    std::string* name = nullptr;
    node_id_map_.Get(id, &name);
    if (node_name == *name) break;
    ++id;
  }
  node_id_map_.Set(id, node_name);
  return id;
}
```

**职责**：将节点名注册到全局 ID 映射表，返回哈希 ID
**关键逻辑**：哈希冲突时递增 ID 直到找到空位（线性探测）

### GetProtoFromFile()

`cyber/common/file.cc:132`

```cpp
bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message) {
  static const std::string kBinExt = ".bin";
  if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
    return GetProtoFromBinaryFile(file_name, message) ||
           GetProtoFromASCIIFile(file_name, message);
  }
  return GetProtoFromASCIIFile(file_name, message) ||
         GetProtoFromBinaryFile(file_name, message);
}
```

**职责**：从文件加载 protobuf 消息，自动判断格式
**关键逻辑**：`.bin` 后缀优先尝试二进制解析，否则优先尝试 ASCII 文本解析；两种格式互为 fallback

### GetFilePathWithEnv()

`cyber/common/file.cc:422`

```cpp
bool GetFilePathWithEnv(const std::string &path, const std::string &env_var,
                        std::string *file_path);
```

**职责**：按优先级查找文件路径
**查找顺序**：

1. 绝对路径 — 直接使用
2. 相对路径以 `.` 开头 — 使用当前目录
3. 按环境变量（如 `APOLLO_CONF_PATH`）中以 `:` 分隔的路径逐个尝试
4. 普通相对路径

### EnsureDirectory()

`cyber/common/file.cc:285`

**职责**：递归创建目录（类似 `mkdir -p`），忽略 `EEXIST` 错误

### 时间转换函数

`cyber/common/time_conversion.h`

UNIX 时间与 GPS 时间互转，考虑闰秒：

| 函数 | 说明 |
| --- | --- |
| `UnixToGpsSeconds()` | UNIX 秒 → GPS 秒 |
| `GpsToUnixSeconds()` | GPS 秒 → UNIX 秒 |
| `UnixToGpsMicroseconds()` | UNIX 微秒 → GPS 微秒 |
| `GpsToUnixMicroseconds()` | GPS 微秒 → UNIX 微秒 |
| `StringToUnixSeconds()` | 字符串 → UNIX 秒（默认格式 `%Y-%m-%d %H:%M:%S`） |
| `UnixSecondsToString()` | UNIX 秒 → 字符串 |

闰秒表 `LEAP_SECONDS` 硬编码在头文件中，以 UNIX 时间戳为索引。

## 日志宏

`cyber/common/log.h`

封装 glog，所有宏自动附带模块名前缀 `[MODULE_NAME]`：

| 宏 | 等价于 |
| --- | --- |
| `AINFO` | `google::LogMessage(INFO) << [module]` |
| `AWARN` | `google::LogMessage(WARNING) << [module]` |
| `AERROR` | `google::LogMessage(ERROR) << [module]` |
| `AFATAL` | `google::LogMessage(FATAL) << [module]` |
| `ADEBUG` | `VLOG(4) << [module][DEBUG]` |
| `ACHECK(cond)` | `CHECK(cond) << [module]` |
| `AINFO_IF(cond)` | 条件为真时输出 INFO |
| `AINFO_EVERY(n)` | 每 n 次输出一次 INFO |

辅助宏用于早返回：

| 宏 | 说明 |
| --- | --- |
| `RETURN_IF_NULL(ptr)` | 指针为空时 return |
| `RETURN_VAL_IF_NULL(ptr, val)` | 指针为空时 return val |
| `RETURN_IF(condition)` | 条件为真时 return |
| `RETURN_VAL_IF(condition, val)` | 条件为真时 return val |

## 工具宏

`cyber/common/macros.h`

| 宏 | 说明 |
| --- | --- |
| `DECLARE_SINGLETON(classname)` | 声明单例类（`Instance()` + `CleanUp()`），通过 `std::call_once` 保证线程安全 |
| `DISALLOW_COPY_AND_ASSIGN(classname)` | 禁用拷贝构造和赋值 |
| `UNUSED(param)` | 标记参数未使用 |
| `CallShutdown(T*)` | SFINAE 自动调用 `Shutdown()`（如果类型有该方法） |

## 基础类型

`cyber/common/types.h`

```cpp
enum ReturnCode { SUCC = 0, FAIL = 1 };

enum Relation : std::uint8_t {
  NO_RELATION = 0,
  DIFF_HOST,   // 不同主机
  DIFF_PROC,   // 同主机不同进程
  SAME_PROC,   // 同进程
};

static const char SRV_CHANNEL_REQ_SUFFIX[] = "__SRV__REQUEST";
static const char SRV_CHANNEL_RES_SUFFIX[] = "__SRV__RESPONSE";
```

## 调用关系

- **被依赖**：CyberRT 几乎所有模块都依赖 `common/log.h` 和 `common/macros.h`
- **GlobalData**：被 `Node`、`Transport`、`Scheduler` 等使用，管理全局配置和名称注册
- **file 工具**：被 `GlobalData::InitConfig()`、各模块配置加载调用
- **time_conversion**：被 `Time` 类和传感器驱动使用
