---
title: "状态管理"
---

# Status 状态管理模块

> 源码路径: `modules/common/status/`

## 概述

`Status` 是 Apollo 各模块 API 调用的统一返回状态封装。当操作成功时返回 `OK` 状态；当操作失败时携带错误码 (`ErrorCode`) 和人类可读的错误信息。该类定义在 `apollo::common` 命名空间下，是整个项目中最基础的错误传播机制之一。

核心设计要点：

- 轻量级值类型，仅包含一个 `ErrorCode` 枚举和一个 `std::string` 消息
- 通过 protobuf `StatusPb` 实现跨进程序列化，嵌入 `Header.status` 随消息传递
- `ErrorCode` 按模块分段编号（Control 1000、Canbus 2000、Localization 3000、Perception 4000、Prediction 5000、Planning 6000、HDMap 7000、Routing 8000 等），方便快速定位故障模块

## 核心类

### Status

```cpp
namespace apollo::common {

class Status {
 public:
  explicit Status(ErrorCode code = ErrorCode::OK, std::string_view msg = "");
  ~Status() = default;

  static Status OK();
  bool ok() const;
  ErrorCode code() const;
  const std::string& error_message() const;
  std::string ToString() const;
  void Save(StatusPb* status_pb);

  bool operator==(const Status& rh) const;
  bool operator!=(const Status& rh) const;

 private:
  ErrorCode code_;
  std::string msg_;
};

}  // namespace apollo::common
```

**成员变量：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `code_` | `ErrorCode` | 错误码枚举，`OK` 表示成功 |
| `msg_` | `std::string` | 人类可读的错误描述，成功时为空 |

### StatusPb（protobuf）

```protobuf
message StatusPb {
  optional ErrorCode error_code = 1 [default = OK];
  optional string msg = 2;
}
```

定义在 `modules/common_msgs/basic_msgs/error_code.proto` 中，用于跨进程传递状态信息，通常嵌入 `Header.status` 字段。

## 核心函数

### 构造函数

```cpp
explicit Status(ErrorCode code = ErrorCode::OK, std::string_view msg = "");
```

默认构造为 `OK` 状态。可指定 `ErrorCode` 和消息创建失败状态：

```cpp
Status error(ErrorCode::CONTROL_INIT_ERROR, "controller not ready");
```

### OK()

```cpp
static Status OK();
```

静态工厂方法，返回 `ErrorCode::OK` 的成功状态。适用于无需错误信息的成功路径。

### ok()

```cpp
bool ok() const;
```

判断状态是否为成功（`code_ == ErrorCode::OK`）。典型用法：

```cpp
Status status = SomeFunction();
if (!status.ok()) {
  AERROR << "Failed: " << status.ToString();
  return status;
}
```

### code() / error_message()

```cpp
ErrorCode code() const;
const std::string& error_message() const;
```

分别获取错误码和错误消息文本，用于日志或上层错误处理逻辑。

### ToString()

```cpp
std::string ToString() const;
```

成功时返回 `"OK"`，失败时返回 `"ErrorCode_Name: message"` 格式，如 `"PLANNING_ERROR: Failed to generate trajectory"`。

### Save()

```cpp
void Save(StatusPb* status_pb);
```

将当前状态序列化到 `StatusPb` protobuf 消息中，用于跨进程通信。空指针检查后设置 `error_code` 和 `msg` 字段。

### operator== / operator!=

```cpp
bool operator==(const Status& rh) const;
bool operator!=(const Status& rh) const;
```

基于错误码和消息文本的相等比较。

## 配置

`Status` 类本身不依赖配置文件。其行为由 `ErrorCode` 枚举定义（位于 `modules/common_msgs/basic_msgs/error_code.proto`），各模块错误码按千位段划分：

| 段范围 | 模块 |
|--------|------|
| 0 | OK（成功） |
| 1000-1999 | Control 控制模块 |
| 2000-2999 | Canbus 底盘通信模块 |
| 3000-3999 | Localization 定位模块 |
| 4000-4999 | Perception 感知模块 |
| 5000-5999 | Prediction 预测模块 |
| 6000-6999 | Planning 规划模块 |
| 7000-7999 | HDMap 高精地图模块 |
| 8000-8999 | Routing 路由模块 |

## 调用关系

```text
各模块 API 函数
    │
    ├── 返回 apollo::common::Status
    │       │
    │       ├── ok() ──→ 判断是否成功
    │       ├── code() / error_message() ──→ 获取错误详情
    │       └── ToString() ──→ 日志输出
    │
    └── Save(StatusPb*) ──→ 序列化
            │
            └── Header.status 字段 ──→ 跨进程传递
                    │
                    └── 反序列化后检查 ok() 继续处理
```

`Status` 贯穿 Apollo 全模块，是函数返回值的标准模式。典型用法：

```cpp
Status SomeModule::Init() {
  if (!LoadConfig()) {
    return Status(ErrorCode::SOME_MODULE_ERROR, "config load failed");
  }
  return Status::OK();
}
```
