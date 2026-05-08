---
title: "通用工具函数"
---

# 通用工具函数

> 源码路径: `modules/common/util/`

## 概述

`common/util` 模块为 Apollo 各子系统提供一组通用的、无业务耦合的基础设施工具。涵盖以下能力域：

- **数据结构** — LRU 缓存、Factory 工厂模式模板、Eigen 容器别名
- **序列化与格式** — JSON 与 Protobuf 互转、Base64 编码、字符串格式化
- **几何与点操作** — PathPoint/PointENU/SpeedPoint 构造、路径点降采样、加权平均
- **时间** — UNIX/GPS 时间戳互转
- **性能度量** — 函数/代码块耗时统计宏
- **杂项** — Proto 比较、Map 辅助、消息头填充、控制台颜色

## 核心类

### JsonUtil

静态工具类，提供 Protobuf Message 与 `nlohmann::json` 之间的转换，以及按 key/path 安全读取 JSON 字段。

```cpp
namespace apollo::common::util {

class JsonUtil {
 public:
  // Proto -> 带 type 标签的 JSON
  static nlohmann::json ProtoToTypedJson(const std::string &json_type,
                                         const google::protobuf::Message &proto);
  // Proto -> JSON
  static nlohmann::json ProtoToJson(const google::protobuf::Message &proto);

  // 按 key 读取
  static bool GetString(const nlohmann::json &json, const std::string &key,
                        std::string *value);
  template <class T>
  static bool GetNumber(const nlohmann::json &json, const std::string &key, T *value);
  static bool GetBoolean(const nlohmann::json &json, const std::string &key, bool *value);
  static bool GetStringVector(const nlohmann::json &json, const std::string &key,
                              std::vector<std::string> *value);

  // 按路径读取，path 格式 "a.b.c"
  static bool GetJsonByPath(const nlohmann::json &json,
                            const std::vector<std::string> &paths, nlohmann::json *value);
  static bool GetStringByPath(const nlohmann::json &json, const std::string &path,
                              std::string *value);
  static bool GetBooleanByPath(const nlohmann::json &json, const std::string &path,
                               bool *value);
  template <class T>
  static bool GetNumberByPath(const nlohmann::json &json, const std::string &path,
                              T *value);
};

}  // namespace apollo::common::util
```

### Factory

通用工厂模式模板，通过 `Register` 注册标识符到创建函数的映射，通过 `CreateObject` / `CreateObjectOrNull` 延迟创建对象。

```cpp
template <typename IdentifierType, class AbstractProduct,
          class ProductCreator = AbstractProduct *(*)(),
          class MapContainer = std::map<IdentifierType, ProductCreator>>
class Factory {
 public:
  bool Register(const IdentifierType &id, ProductCreator creator);
  bool Unregister(const IdentifierType &id);
  bool Contains(const IdentifierType &id);

  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType &id, Args &&... args);

  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType &id, Args &&... args);
};
```

### LRUCache

基于双向链表 + `std::map` 的 LRU 缓存。支持容量限制、按访问顺序淘汰、静默读取（不改变顺序）等操作。

```cpp
template <class K, class V>
class LRUCache {
 public:
  explicit LRUCache(size_t capacity = 10);

  template <typename VV> bool Put(const K &key, VV &&val);
  V *Get(const K &key);
  V *GetSilently(const K &key);        // 读取不更新 LRU 顺序
  bool Contains(const K &key);
  bool Remove(const K &key);
  bool Prioritize(const K &key);       // 将 key 提升到最前
  size_t size();
  bool Full();
};
```

### TimeUtil

UNIX 时间戳（1970-01-01）与 GPS 时间戳（1980-01-06）之间的秒级转换，自动处理闰秒。

```cpp
class TimeUtil {
 public:
  static double Unix2Gps(double unix_time);
  static double Gps2Unix(double gps_time);
 private:
  static const int UNIX_GPS_DIFF = 315964782;
  static const int LEAP_SECOND_TIMESTAMP = 1483228799;
};
```

### Timer / TimerWrapper

性能计时工具。`Timer` 手动调用 `Start()` / `End()`，`TimerWrapper` 利用 RAII 在构造/析构时自动计时。配合宏使用可编译期开关。

```cpp
class Timer {
 public:
  void Start();
  int64_t End(const std::string &msg);  // 返回毫秒，自动重启计时
};

class TimerWrapper {
 public:
  explicit TimerWrapper(const std::string &msg);
  ~TimerWrapper();  // 析构时输出耗时
};
```

### PointFactory

静态工厂方法，快速构造 `Vec2d`、`SLPoint`、`PointENU`、`SpeedPoint`、`PathPoint` 等 Protobuf 点类型。

```cpp
class PointFactory {
 public:
  template <typename XY>
  static math::Vec2d ToVec2d(const XY &xy);
  static SLPoint ToSLPoint(double s, double l);
  static PointENU ToPointENU(double x, double y, double z = 0);
  static SpeedPoint ToSpeedPoint(double s, double t, double v = 0,
                                 double a = 0, double da = 0);
  static PathPoint ToPathPoint(double x, double y, double z = 0, double s = 0,
                               double theta = 0, double kappa = 0,
                               double dkappa = 0, double ddkappa = 0);
};
```

## 核心函数

### Proto 与消息工具 (`message_util.h`)

```cpp
// 为 Protobuf 消息自动填充 header（模块名、时间戳、序列号）
template <typename T>
static void FillHeader(const std::string &module_name, T *msg);

// 将消息序列化为 pb.txt 文件，按序列号命名存入 dump_dir
template <typename T>
bool DumpMessage(const std::shared_ptr<T> &msg, const std::string &dump_dir = "/tmp");

// 计算消息的哈希指纹（基于序列化字节）
size_t MessageFingerprint(const google::protobuf::Message &message);
```

### 几何与路径工具 (`util.h`)

```cpp
// Protobuf 消息比较（序列化后逐字节比较，比 MessageDifferencer 快 5 倍）
template <typename ProtoA, typename ProtoB>
bool IsProtoEqual(const ProtoA &a, const ProtoB &b);

// 将区间 [start, end] 均匀切分为 num 段
template <typename T>
void uniform_slice(T start, T end, uint32_t num, std::vector<T> *sliced);

// 两点间 XY 平面欧氏距离
template <typename U, typename V>
double DistanceXY(const U &u, const V &v);

// 两点是否在 XY 平面上视为同一位置（精度 1e-8）
template <typename U, typename V>
bool SamePointXY(const U &u, const V &v);

// 两路径点加权平均
PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint &p1,
                                            const PathPoint &p2, double w1, double w2);

// 浮点数近似相等判断（ULP 单位）
template <typename T>
bool IsFloatEqual(T x, T y, int ulp = 2);
```

### 路径降采样 (`points_downsampler.h`)

```cpp
// 按累积转角降采样：方向变化超过 angle_threshold 时保留采样点
template <typename Points>
std::vector<size_t> DownsampleByAngle(const Points &points, double angle_threshold);

// 按距离降采样：急弯段使用更小的采样间隔
template <typename Points>
std::vector<size_t> DownsampleByDistance(const Points &points,
                                         int downsampleDistance,
                                         int steepTurnDownsampleDistance);
```

### 字符串与编码工具 (`string_util.h`)

```cpp
// Base64 编码
std::string EncodeBase64(std::string_view in);

// absl::StrFormat 导出
using absl::StrFormat;

// 调用对象的 DebugString() 追加到输出
struct DebugStringFormatter { ... };
```

### Map 辅助函数 (`map_util.h`)

从 `google::protobuf::stubs/map_util.h` 导出一组常用 Map 操作：

- **查找** — `FindOrDie`, `FindOrNull`, `FindCopy`, `FindWithDefault`
- **包含** — `ContainsKey`, `ContainsKeyValuePair`
- **插入** — `InsertIfNotPresent`, `InsertOrUpdate`, `InsertOrDie`
- **查找或插入** — `LookupOrInsert`, `LookupOrInsertNew`

### Eigen 容器别名 (`eigen_defs.h`)

为使用 Eigen 的 STL 容器提供对齐分配器类型别名：

```cpp
template <class EigenType> using EigenVector = std::vector<EigenType, Eigen::aligned_allocator<EigenType>>;
template <class EigenType> using EigenDeque  = std::deque<EigenType, Eigen::aligned_allocator<EigenType>>;
template <typename T, class EigenType> using EigenMap     = std::map<T, EigenType, ...>;
template <typename T, class EigenType> using EigenMultiMap = std::multimap<T, EigenType, ...>;

using EigenVector3dVec = EigenVector<Eigen::Vector3d>;
using EigenAffine3dVec = EigenVector<Eigen::Affine3d>;
```

### 数据提取 (`data_extraction.h`)

递归遍历目录收集 Record 文件路径（Prediction 模块使用）：

```cpp
namespace apollo::prediction {
void GetRecordFileNames(const boost::filesystem::path &p,
                        std::vector<std::string> *data_files);
}
```

### C++ 兼容层 (`future.h`)

在 C++11/14 环境下通过 `absl` 补齐 C++17 特性：`std::optional`、`std::string_view`、`std::make_unique`、`std::make_integer_sequence`。

### 控制台颜色 (`color.h`)

ANSI 终端颜色常量：`ANSI_RED`、`ANSI_GREEN`、`ANSI_YELLOW`、`ANSI_BLUE`、`ANSI_MAGENTA`、`ANSI_CYAN`、`ANSI_RESET`。

## 配置

- `FLAGS_multithread_run` — 控制 `UNIQUE_LOCK_MULTITHREAD` 宏是否实际加锁（源自 `config_gflags.h`）
- `ENABLE_PERF` 编译宏 — 启用 `PERF_FUNCTION` / `PERF_BLOCK_START` / `PERF_BLOCK_END` 性能计时宏；未定义时这些宏展开为空操作

## 调用关系

- `JsonUtil` 内部调用 `google::protobuf::util::MessageToJsonString` 进行 Proto 到 JSON 转换，路径解析使用 `absl::StrSplit` 分割
- `FillHeader` 依赖 `apollo::cyber::Clock::NowInSeconds()` 获取当前时间
- `DumpMessage` 依赖 `cyber::common::SetProtoToASCIIFile` 写文件
- `Timer` 依赖 `apollo::cyber::Time` 获取高精度时间
- `DownsampleByDistance` 内部使用 `common::math::Vec2d` 计算距离和角度
- `PointFactory` 依赖 `common::math::Vec2d` 和 Protobuf 定义的点类型
- `map_util.h` 直接复用 Protobuf 库的 Map 工具函数
