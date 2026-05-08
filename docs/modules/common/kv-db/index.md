---
title: "KV 数据库"
---

# KV 数据库

> 源码路径: `modules/common/kv_db/`

## 概述

KVDB 是 Apollo 的轻量级键值数据库模块，基于 SQLite3 实现，用于存储系统级参数和配置数据。模块以静态类 `KVDB` 对外提供服务，每次调用都会自动打开数据库、执行操作、关闭连接，保证线程安全与资源释放。

键名推荐使用冒号分隔的层次化命名规范，例如 `"apollo:data:commit_id"`。

底层通过匿名命名空间中的 `SqliteWraper` 类封装 SQLite 连接管理，在构造时自动建表 `key_value(key VARCHAR(128) PRIMARY KEY, value TEXT)`。

## 核心类

### KVDB

位于 `apollo::common` 命名空间，所有方法均为 `static`，无需实例化即可调用。

```cpp
class KVDB {
 public:
  static bool Put(std::string_view key, std::string_view value);
  static bool Delete(std::string_view key);
  static std::optional<std::string> Get(std::string_view key);
  static std::vector<std::pair<std::string, std::string>> GetWithStart(
      std::string_view start);
};
```

### SqliteWraper（内部）

匿名命名空间中的 SQLite 包装类，每个公开方法调用时临时构造，生命周期覆盖单次 SQL 操作。

```cpp
class SqliteWraper {
 public:
  // sqlite3_exec 回调，将查询结果写入 std::string
  static int Callback(void *data, int argc, char **argv, char **col_name);

  SqliteWraper();   // 打开数据库并确保 key_value 表存在
  ~SqliteWraper();  // 关闭数据库连接

  bool SQL(std::string_view sql, std::string *value = nullptr);
  sqlite3* GetDB() const;
};
```

## 核心函数

### KVDB::Put

```cpp
static bool Put(std::string_view key, std::string_view value);
```

写入或更新键值对。使用 `INSERT OR REPLACE` 语义，若 key 已存在则覆盖旧值。返回操作是否成功。

### KVDB::Delete

```cpp
static bool Delete(std::string_view key);
```

根据 key 删除记录。返回操作是否成功。

### KVDB::Get

```cpp
static std::optional<std::string> Get(std::string_view key);
```

查询单个 key 对应的 value。返回 `std::optional<std::string>`：

- 查询成功且值非空时返回 `std::optional` 包含的值
- 查询失败或值为空时返回 `std::nullopt`

可通过 `has_value()` 检查是否有值，`value()` 获取实际值，`value_or("")` 提供默认值。

### KVDB::GetWithStart

```cpp
static std::vector<std::pair<std::string, std::string>> GetWithStart(
    std::string_view start);
```

按前缀批量查询。使用 SQL `LIKE 'start%'` 匹配所有以 `start` 开头的键，返回 `vector` 形式的键值对列表。

### SqliteWraper::Callback

```cpp
static int Callback(void *data, int argc, char **argv, char **col_name);
```

`sqlite3_exec` 的通用回调函数，用于单值查询场景。将查询结果的第一列写入 `data` 指向的 `std::string`。`GetWithStart` 方法使用自定义 lambda 回调处理多行结果。

## 配置

通过 gflags 命令行参数配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kv_db_path` | `/apollo/data/kv_db.sqlite` | SQLite 数据库文件路径 |

命令行工具 `kv_db_tool` 还支持以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `op` | `get` | 操作类型，支持 `put`、`get`、`del` |
| `key` | `""` | 要操作的键 |
| `value` | `""` | 写入时的值 |

## 调用关系

```text
KVDB::Put / Delete / Get / GetWithStart
  └── SqliteWraper（临时构造，自动管理 SQLite 连接生命周期）
        ├── sqlite3_open()    // 打开数据库文件
        ├── sqlite3_exec()    // 执行 SQL 语句
        ├── sqlite3_close()   // 析构时关闭连接
        └── SQL()             // 内部统一封装 exec 调用

kv_db_tool（命令行工具）
  └── KVDB::Get / Put / Delete（通过 gflags 参数选择操作）
```

内部实现中，`KVDB` 的每个静态方法都会创建一个局部 `SqliteWraper` 对象，该对象在构造时通过 `sqlite3_open` 打开由 `--kv_db_path` 指定的数据库文件并确保 `key_value` 表存在，在析构时通过 `sqlite3_close` 释放连接。这种"每调用一次打开/关闭"的设计牺牲了部分性能，但避免了跨调用的连接状态管理问题。
