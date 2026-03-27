---
title: Component 模块
---

# Component 模块

## 模块职责概述

Component 是 Cyber RT 中构建功能模块的标准方式。开发者只需继承 Component 基类并实现 `Init()` 和 `Proc()` 方法，框架会自动完成消息订阅、调度触发和生命周期管理。

Component 模块提供两种组件模型：

- **Component\<M0, M1, M2, M3\>**：消息驱动型，当订阅的 channel 上有新消息到达时触发 `Proc()`
- **TimerComponent**：定时驱动型，按固定时间间隔周期性触发 `Proc()`

两者均通过 DAG 配置文件声明，由框架动态加载和管理。

## 核心类与接口

### ComponentBase

所有 Component 的抽象基类，定义于 `cyber/component/component_base.h`。提供生命周期管理、配置加载等公共能力。

```cpp
class ComponentBase : public std::enable_shared_from_this<ComponentBase> {
public:
  virtual ~ComponentBase() {}

  // 由框架调用，传入 proto 配置进行初始化
  virtual bool Initialize(const ComponentConfig& config) { return false; }
  virtual bool Initialize(const TimerComponentConfig& config) { return false; }

  // 关闭组件：设置 shutdown 标志、清理资源、关闭所有 Reader、移除调度任务
  virtual void Shutdown();

  // 从配置文件加载 protobuf 配置
  template <typename T>
  bool GetProtoConfig(T* config) const;

protected:
  // 用户实现的初始化逻辑
  virtual bool Init() = 0;

  // 用户实现的清理逻辑（可选）
  virtual void Clear() { return; }

  // 获取配置文件路径
  const std::string& ConfigFilePath() const;

  // 加载 config_file 和 flag_file
  void LoadConfigFiles(const ComponentConfig& config);
  void LoadConfigFiles(const TimerComponentConfig& config);

  std::atomic<bool> is_shutdown_ = {false};
  std::shared_ptr<Node> node_ = nullptr;
  std::string config_file_path_ = "";
  std::vector<std::shared_ptr<ReaderBase>> readers_;
};
```

关键设计：
- 继承 `std::enable_shared_from_this`，确保组件在异步回调中安全引用自身
- `Shutdown()` 流程：设置 `is_shutdown_` 标志 -> 调用 `Clear()` -> 关闭所有 Reader -> 从调度器移除任务
- 配置文件路径支持 `APOLLO_CONF_PATH` 环境变量解析，flag 文件支持 `APOLLO_FLAG_PATH` 环境变量

### Component\<M0, M1, M2, M3\>

消息驱动型组件模板类，定义于 `cyber/component/component.h`。支持 1 到 4 个 channel 输入，通过模板特化实现。

```cpp
// 四消息版本（完整形式）
template <typename M0 = NullType, typename M1 = NullType,
          typename M2 = NullType, typename M3 = NullType>
class Component : public ComponentBase {
public:
  // 框架调用，根据 ComponentConfig 初始化
  bool Initialize(const ComponentConfig& config) override;

  // 框架调用，转发到用户的 Proc()
  bool Process(const std::shared_ptr<M0>& msg0,
               const std::shared_ptr<M1>& msg1,
               const std::shared_ptr<M2>& msg2,
               const std::shared_ptr<M3>& msg3);

private:
  // 用户实现的处理逻辑
  virtual bool Proc(const std::shared_ptr<M0>& msg0,
                    const std::shared_ptr<M1>& msg1,
                    const std::shared_ptr<M2>& msg2,
                    const std::shared_ptr<M3>& msg3) = 0;
};
```

框架提供了四个模板特化版本，分别处理 1、2、3、4 个消息输入：

```cpp
// 单消息
template <typename M0>
class Component<M0, NullType, NullType, NullType>;

// 双消息
template <typename M0, typename M1>
class Component<M0, M1, NullType, NullType>;

// 三消息
template <typename M0, typename M1, typename M2>
class Component<M0, M1, M2, NullType>;

// 四消息（默认模板）
template <typename M0, typename M1, typename M2, typename M3>
class Component;
```

Initialize 流程（以单消息为例）：
1. 创建 `Node` 实例
2. 调用 `LoadConfigFiles()` 加载配置文件和 flag 文件
3. 调用用户的 `Init()` 方法
4. 根据 `ComponentConfig` 中的 `readers` 配置创建 Reader
5. 构建处理函数闭包，包含性能统计（proc latency、cyber latency）
6. 创建 `DataVisitor` 绑定所有输入 channel
7. 通过 `RoutineFactory` 创建协程，注册到调度器

### TimerComponent

定时驱动型组件，定义于 `cyber/component/timer_component.h` 和 `timer_component.cc`。

```cpp
class TimerComponent : public ComponentBase {
public:
  // 框架调用，根据 TimerComponentConfig 初始化
  bool Initialize(const TimerComponentConfig& config) override;

  // 清理定时器
  void Clear() override;

  // 获取定时间隔（毫秒）
  uint32_t GetInterval() const;

  // 框架调用，转发到用户的 Proc()
  bool Process();

private:
  // 用户实现的周期处理逻辑
  virtual bool Proc() = 0;

  uint32_t interval_ = 0;
  std::unique_ptr<Timer> timer_;
};
```

Initialize 流程：
1. 校验配置中 `name` 和 `interval` 字段
2. 创建 `Node` 实例
3. 调用 `LoadConfigFiles()` 加载配置
4. 调用用户的 `Init()` 方法
5. 注册性能统计 channel
6. 创建 `Timer` 对象，以 `interval` 毫秒为周期执行 `Process()`
7. 启动定时器

## 消息触发模型

### Component（消息驱动）

消息驱动型 Component 使用 `DataVisitor` + 协程调度实现触发：

```
Channel 消息到达
  -> Receiver -> DataDispatcher::Dispatch()
  -> DataVisitor 检测到所有输入 channel 数据就绪
  -> 调度器唤醒对应协程
  -> Component::Process() -> 用户 Proc()
```

对于多消息输入的 Component，`DataVisitor` 会等待所有订阅的 channel 都有新消息后才触发一次 `Proc()`。这是一种数据融合（data fusion）机制。

### TimerComponent（定时驱动）

定时驱动型使用 `Timer` 实现周期触发：

```
Timer 到期
  -> 回调函数执行
  -> TimerComponent::Process() -> 用户 Proc()
```

定时器独立于消息到达，适用于需要固定频率执行的场景（如控制回路）。

## 配置方式

### DAG 配置文件

Component 通过 DAG（Directed Acyclic Graph）配置文件声明，框架根据配置动态加载组件。相关 proto 定义于 `cyber/proto/dag_conf.proto`：

```protobuf
message DagConfig {
  repeated ModuleConfig module_config = 1;
}

message ModuleConfig {
  optional string module_library = 1;        // 动态库路径
  repeated ComponentInfo components = 2;     // 消息驱动型组件列表
  repeated TimerComponentInfo timer_components = 3;  // 定时驱动型组件列表
}

message ComponentInfo {
  optional string class_name = 1;            // 组件类名（用于动态加载）
  optional ComponentConfig config = 2;       // 组件配置
}

message TimerComponentInfo {
  optional string class_name = 1;
  optional TimerComponentConfig config = 2;
}
```

### ComponentConfig

消息驱动型组件的配置，定义于 `cyber/proto/component_conf.proto`：

```protobuf
message ComponentConfig {
  optional string name = 1;                  // 组件名称（即 Node 名称）
  optional string config_file_path = 2;      // 业务配置文件路径
  optional string flag_file_path = 3;        // gflags 文件路径
  repeated ReaderOption readers = 4;         // Reader 配置列表
}

message ReaderOption {
  optional string channel = 1;               // 订阅的 channel 名称
  optional QosProfile qos_profile = 2;       // QoS 配置
  optional uint32 pending_queue_size = 3;    // 未处理消息队列容量（默认 1）
}
```

### TimerComponentConfig

定时驱动型组件的配置：

```protobuf
message TimerComponentConfig {
  optional string name = 1;                  // 组件名称
  optional string config_file_path = 2;      // 业务配置文件路径
  optional string flag_file_path = 3;        // gflags 文件路径
  optional uint32 interval = 4;              // 定时间隔（毫秒）
}
```

### DAG 配置示例

```
module_config {
  module_library: "/apollo/bazel-bin/modules/planning/libplanning_component.so"
  components {
    class_name: "PlanningComponent"
    config {
      name: "planning"
      config_file_path: "/apollo/modules/planning/conf/planning.conf"
      flag_file_path: "/apollo/modules/planning/conf/planning.flag"
      readers {
        channel: "/apollo/prediction"
      }
      readers {
        channel: "/apollo/routing_response"
        pending_queue_size: 10
      }
    }
  }
  timer_components {
    class_name: "MonitorComponent"
    config {
      name: "monitor"
      config_file_path: "/apollo/modules/monitor/conf/monitor.conf"
      interval: 100
    }
  }
}
```

### 组件注册宏

每个自定义 Component 必须使用注册宏，使框架能够通过 `class_loader` 动态加载：

```cpp
// 在 .cc 文件末尾使用
CYBER_REGISTER_COMPONENT(YourComponentClass)

// 宏展开为：
CLASS_LOADER_REGISTER_CLASS(YourComponentClass, apollo::cyber::ComponentBase)
```

## 数据流描述

### 消息驱动型 Component 完整数据流

```
DAG 配置加载
  -> class_loader 动态加载 .so
  -> 创建 Component 实例
  -> Component::Initialize(ComponentConfig)
     -> 创建 Node
     -> 加载配置文件
     -> 用户 Init()
     -> 创建 Reader（根据 readers 配置）
     -> 创建 DataVisitor（绑定所有输入 channel）
     -> 创建协程任务注册到调度器
  -> 运行时：
     消息到达 -> DataVisitor 数据就绪 -> 调度器触发协程
     -> Process() -> 用户 Proc(msg0, msg1, ...)
```

### 定时驱动型 TimerComponent 完整数据流

```
DAG 配置加载
  -> class_loader 动态加载 .so
  -> 创建 TimerComponent 实例
  -> TimerComponent::Initialize(TimerComponentConfig)
     -> 创建 Node
     -> 加载配置文件
     -> 用户 Init()
     -> 创建 Timer（interval 毫秒）
     -> 启动定时器
  -> 运行时：
     Timer 周期触发 -> Process() -> 用户 Proc()
```

## 与其他模块的关系

| 依赖模块 | 关系说明 |
|---------|---------|
| **Node** | 每个 Component 内部持有一个 `Node` 实例，通过 Node 创建 Reader 进行消息订阅 |
| **Scheduler** | 消息驱动型 Component 的协程任务由调度器管理；Shutdown 时从调度器移除 |
| **Data** | 使用 `DataVisitor` 实现多 channel 数据融合触发 |
| **Timer** | TimerComponent 使用 `Timer` 实现周期触发 |
| **Class Loader** | 通过 `CYBER_REGISTER_COMPONENT` 宏注册，框架通过 `class_loader` 动态加载组件 |
| **Proto** | `ComponentConfig`、`TimerComponentConfig`、`DagConfig` 等 proto 定义了组件的声明式配置 |
| **Statistics** | 运行时采集 proc latency 和 cyber latency 性能指标 |
