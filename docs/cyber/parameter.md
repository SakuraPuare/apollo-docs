---
title: Parameter 参数服务模块
---

# Parameter 参数服务模块

## 1. 模块职责概述

Parameter 模块为 Apollo Cyber RT 提供分布式参数服务，允许节点之间共享键值对形式的配置参数。该模块基于 Service/Client 通信模式实现，ParameterServer 在某个 Node 上注册三个 Service 端点，ParameterClient 通过发送 RPC 请求来读取、写入和列举参数。

参数支持的值类型包括：`bool`、`int64`、`double`、`string` 以及任意 Protobuf 消息。

## 2. 核心类/接口说明

### 2.1 Parameter

参数值的封装类，内部持有一个 `proto::Param` 对象。提供多种构造方式和类型安全的值访问接口。

```cpp
class Parameter {
public:
  Parameter();
  explicit Parameter(const std::string& name);
  Parameter(const std::string& name, const bool bool_value);
  Parameter(const std::string& name, const int int_value);
  Parameter(const std::string& name, const int64_t int_value);
  Parameter(const std::string& name, const float float_value);
  Parameter(const std::string& name, const double double_value);
  Parameter(const std::string& name, const std::string& string_value);
  Parameter(const std::string& name, const char* string_value);
  // Protobuf 消息参数
  Parameter(const std::string& name, const google::protobuf::Message& msg);

  // 类型信息
  ParamType Type() const;
  std::string TypeName() const;
  const std::string Name() const;

  // 类型安全的值访问（模板特化）
  template <typename ValueType>
  ValueType value() const;

  // 便捷访问方法
  bool AsBool() const;
  int64_t AsInt64() const;
  double AsDouble() const;
  const std::string AsString() const;

  // Proto 序列化
  void FromProtoParam(const Param& param);
  Param ToProtoParam() const;

  std::string DebugString() const;
};
```

`value<T>()` 通过 `std::enable_if` 模板特化实现类型安全访问，类型不匹配时会输出错误日志。

### 2.2 ParameterServer

参数服务端，在指定 Node 上创建三个 Service 端点来响应参数操作请求。参数存储在本地内存的 `unordered_map` 中。

```cpp
class ParameterServer {
public:
  explicit ParameterServer(const std::shared_ptr<Node>& node);

  void SetParameter(const Parameter& parameter);
  bool GetParameter(const std::string& parameter_name, Parameter* parameter);
  void ListParameters(std::vector<Parameter>* parameters);

private:
  std::shared_ptr<Node> node_;
  std::shared_ptr<Service<ParamName, Param>> get_parameter_service_;
  std::shared_ptr<Service<Param, BoolResult>> set_parameter_service_;
  std::shared_ptr<Service<NodeName, Params>> list_parameters_service_;

  std::mutex param_map_mutex_;
  std::unordered_map<std::string, Param> param_map_;
};
```

构造时自动注册三个 Service：

| Service 名称 | 请求类型 | 响应类型 | 功能 |
|---|---|---|---|
| `{node_name}/get_parameter` | `ParamName` | `Param` | 按名称获取参数 |
| `{node_name}/set_parameter` | `Param` | `BoolResult` | 设置参数值 |
| `{node_name}/list_parameters` | `NodeName` | `Params` | 列举所有参数 |

所有 Service 回调内部使用 `std::mutex` 保护 `param_map_` 的并发访问。

ParameterServer 同时提供本地直接访问方法（`SetParameter`、`GetParameter`、`ListParameters`），无需经过 RPC 调用。

### 2.3 ParameterClient

参数客户端，通过 Service/Client RPC 机制远程访问指定 Node 上的 ParameterServer。

```cpp
class ParameterClient {
public:
  ParameterClient(const std::shared_ptr<Node>& node,
                  const std::string& service_node_name);

  bool GetParameter(const std::string& param_name, Parameter* parameter);
  bool SetParameter(const Parameter& parameter);
  bool ListParameters(std::vector<Parameter>* parameters);

private:
  std::shared_ptr<Node> node_;
  std::shared_ptr<Client<ParamName, Param>> get_parameter_client_;
  std::shared_ptr<Client<Param, BoolResult>> set_parameter_client_;
  std::shared_ptr<Client<NodeName, Params>> list_parameters_client_;
};
```

构造时需要指定目标 `service_node_name`（即运行 ParameterServer 的 Node 名称），Client 会自动拼接 Service 名称并创建对应的 RPC Client。

所有操作通过 `SendRequest` 发送同步请求，返回 `nullptr` 表示调用失败或超时。

### 2.4 参数服务名称约定

```cpp
// parameter_service_names.h
constexpr auto SERVICE_NAME_DELIMITER = "/";
constexpr auto GET_PARAMETER_SERVICE_NAME = "get_parameter";
constexpr auto SET_PARAMETER_SERVICE_NAME = "set_parameter";
constexpr auto LIST_PARAMETERS_SERVICE_NAME = "list_parameters";

static inline std::string FixParameterServiceName(
    const std::string& node_name, const char* service_name) {
  return node_name + SERVICE_NAME_DELIMITER + service_name;
}
```

Service 名称格式：`{node_name}/{operation}`，例如 `/apollo/planning/get_parameter`。

## 3. 数据流描述

### 参数设置流程

```
ParameterClient                    ParameterServer
     |                                   |
     |  SendRequest(Param)               |
     |---------------------------------->|
     |                                   | lock(param_map_mutex_)
     |                                   | param_map_[name] = param
     |                                   | unlock
     |  BoolResult(true)                 |
     |<----------------------------------|
```

### 参数获取流程

```
ParameterClient                    ParameterServer
     |                                   |
     |  SendRequest(ParamName)           |
     |---------------------------------->|
     |                                   | lock(param_map_mutex_)
     |                                   | lookup param_map_[name]
     |                                   | unlock
     |  Param (or NOT_SET type)          |
     |<----------------------------------|
     |                                   |
     | FromProtoParam(response)          |
```

### Service/Client 底层通信

参数服务的 RPC 通信基于 Cyber 的 Service/Client 模式：

1. Service 在构造时创建 `request_receiver_`（Reader）和 `response_transmitter_`（Writer）
2. Client 在构造时创建 `request_transmitter_`（Writer）和 `response_receiver_`（Reader）
3. 请求/响应通过 Channel 传输，Channel 名称为 `{service_name}_req` 和 `{service_name}_res`
4. Client 使用 `sequence_number` 和 `std::promise/future` 实现请求-响应的异步匹配

## 4. 配置方式

Parameter 模块本身无需额外配置文件。使用方式为在代码中直接创建 ParameterServer 和 ParameterClient。

### 使用示例

#### 服务端

```cpp
#include "cyber/cyber.h"
#include "cyber/parameter/parameter_server.h"

// 初始化节点
auto node = cyber::CreateNode("parameter_server_node");

// 创建参数服务
auto param_server = std::make_shared<cyber::ParameterServer>(node);

// 设置参数
param_server->SetParameter(cyber::Parameter("max_speed", 60.0));
param_server->SetParameter(cyber::Parameter("enable_lidar", true));
param_server->SetParameter(cyber::Parameter("vehicle_id", "vehicle_001"));
```

#### 客户端

```cpp
#include "cyber/cyber.h"
#include "cyber/parameter/parameter_client.h"

auto node = cyber::CreateNode("parameter_client_node");

// 连接到目标节点的参数服务
cyber::ParameterClient param_client(node, "parameter_server_node");

// 获取参数
cyber::Parameter param;
if (param_client.GetParameter("max_speed", &param)) {
  double speed = param.AsDouble();
}

// 设置参数
param_client.SetParameter(cyber::Parameter("max_speed", 80.0));

// 列举所有参数
std::vector<cyber::Parameter> params;
param_client.ListParameters(&params);
```

## 5. Proto 定义

### parameter.proto

```protobuf
enum ParamType {
  NOT_SET = 0;
  BOOL = 1;
  INT = 2;
  DOUBLE = 3;
  STRING = 4;
  PROTOBUF = 5;
}

message Param {
  optional string name = 1;
  optional ParamType type = 2;
  optional string type_name = 3;
  oneof oneof_value {
    bool bool_value = 4;
    int64 int_value = 5;
    double double_value = 6;
    string string_value = 7;
  }
  optional bytes proto_desc = 8;
}

message NodeName   { optional string value = 1; }
message ParamName  { optional string value = 1; }
message BoolResult { optional bool value = 1; }
message Params     { repeated Param param = 1; }
```

`Param` 使用 `oneof` 存储不同类型的值，`proto_desc` 字段用于 `PROTOBUF` 类型参数的描述符传递，使接收端能够反序列化未知的 Protobuf 消息。

## 6. 与其他模块的关系

| 模块 | 关系 |
|------|------|
| `cyber/service/` | 直接依赖，ParameterServer 创建 Service，ParameterClient 创建 Client |
| `cyber/node/` | 依赖 Node 来创建 Service/Client 实例 |
| `cyber/service_discovery/` | 间接依赖，Service/Client 的注册和发现通过 ServiceManager 完成 |
| `cyber/message/protobuf_factory.h` | Parameter 的 PROTOBUF 类型使用 ProtobufFactory 进行动态消息创建 |
| `cyber/proto/parameter.proto` | 定义参数的序列化格式 |
