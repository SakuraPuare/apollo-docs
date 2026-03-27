---
title: 创建 Cyber 模块
description: 从零开始创建一个 Apollo Cyber RT 模块的完整开发者指南，涵盖目录结构、Proto 定义、Component 实现、构建配置与运行部署。
---

# 创建 Cyber 模块

## 概述

Cyber RT 是 Apollo 自动驾驶平台的运行时框架，负责模块间的通信调度、消息传递和生命周期管理。每个功能模块（如感知、规划、控制）都以 **Cyber Component** 的形式运行，由 Cyber RT 统一加载和调度。

一个 Cyber 模块的核心职责：

- 通过 `Reader` 订阅上游 channel 的消息
- 在 `Proc()` 中执行业务逻辑
- 通过 `Writer` 将处理结果发布到下游 channel

Apollo 提供两种 Component 基类：

| 基类 | 触发方式 | 适用场景 |
|------|---------|---------|
| `cyber::Component<M0, M1, ...>` | 消息驱动，收到指定 channel 消息时触发 `Proc()` | 感知、预测等依赖输入数据的模块 |
| `cyber::TimerComponent` | 定时触发，按固定间隔调用 `Proc()` | 监控、状态上报等周期性任务 |

本文以 Apollo 源码中的 `guardian` 模块为主要示例，演示如何从零创建一个基于 `TimerComponent` 的 Cyber 模块。

## 模块目录结构

以 `guardian` 模块为例，一个典型的 Cyber 模块目录结构如下：

```
modules/guardian/
├── BUILD                        # Bazel 构建文件
├── cyberfile.xml                # 包管理描述文件
├── conf/
│   └── guardian_conf.pb.txt     # 运行时配置（protobuf text 格式）
├── dag/
│   └── guardian.dag             # DAG 调度配置
├── launch/
│   └── guardian.launch          # 启动文件
├── proto/
│   ├── BUILD                    # proto 构建文件
│   └── guardian_conf.proto      # 配置消息定义
├── guardian_component.h         # Component 头文件
└── guardian_component.cc        # Component 实现
```

更复杂的模块（如 `storytelling`）还会包含子目录来组织业务逻辑：

```
modules/storytelling/
├── BUILD
├── cyberfile.xml
├── conf/
├── dag/
├── launch/
├── proto/
├── common/                      # 公共工具（gflags 等）
├── story_tellers/               # 业务逻辑子模块
│   ├── base_teller.h
│   └── close_to_junction_teller.{h,cc}
├── frame_manager.{h,cc}
├── storytelling.h               # Component 入口
└── storytelling.cc
```

## 详细步骤

### 第一步：创建目录结构

```bash
mkdir -p modules/my_module/{conf,dag,launch,proto}
```

### 第二步：定义 Proto 消息

在 `proto/` 下定义模块的配置消息。Apollo 使用 proto2 语法。

`proto/my_module_conf.proto`：

```protobuf
syntax = "proto2";

package apollo.my_module;

message MyModuleConf {
  optional bool enable = 1 [default = false];
  optional double process_rate = 2 [default = 10.0];
  optional string output_topic = 3;
}
```

`proto/BUILD`：

```python
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "my_module_conf_proto",
    srcs = ["my_module_conf.proto"],
)

apollo_package()
```

### 第三步：编写 Component 类

Component 是模块的入口。继承 `TimerComponent` 并实现 `Init()` 和 `Proc()` 两个方法。

`my_module_component.h`：

```cpp
#pragma once

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/my_module/proto/my_module_conf.pb.h"

namespace apollo {
namespace my_module {

class MyModuleComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  MyModuleConf config_;
  // Reader：订阅上游消息
  std::shared_ptr<cyber::Reader<SomeInputMsg>> input_reader_;
  // Writer：发布处理结果
  std::shared_ptr<cyber::Writer<SomeOutputMsg>> output_writer_;
  std::mutex mutex_;
};

// 注册组件，使 Cyber RT 能够通过类名动态加载
CYBER_REGISTER_COMPONENT(MyModuleComponent)

}  // namespace my_module
}  // namespace apollo
```

`my_module_component.cc`：

```cpp
#include "modules/my_module/my_module_component.h"
#include "cyber/common/log.h"

namespace apollo {
namespace my_module {

bool MyModuleComponent::Init() {
  // 1. 加载 protobuf text 格式的配置文件
  if (!GetProtoConfig(&config_)) {
    AERROR << "Unable to load config file: " << ConfigFilePath();
    return false;
  }

  // 2. 创建 Reader 订阅上游 channel
  input_reader_ = node_->CreateReader<SomeInputMsg>(
      "/apollo/some_input",
      [this](const std::shared_ptr<SomeInputMsg>& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        // 缓存最新消息
      });

  // 3. 创建 Writer 发布到下游 channel
  output_writer_ = node_->CreateWriter<SomeOutputMsg>(
      config_.output_topic());

  return true;
}

bool MyModuleComponent::Proc() {
  // 定时触发的业务逻辑
  // 读取缓存的输入数据，处理后通过 Writer 发布
  SomeOutputMsg output;
  // ... 处理逻辑 ...
  output_writer_->Write(output);
  return true;
}

}  // namespace my_module
}  // namespace apollo
```

关键点说明：

- `GetProtoConfig()` 会自动读取 DAG 中 `config_file_path` 指定的配置文件，并解析为对应的 protobuf 消息
- `node_` 是基类提供的 `cyber::Node` 指针，用于创建 Reader/Writer
- `CYBER_REGISTER_COMPONENT` 宏将类注册到 Cyber RT 的工厂中，使其可以通过 DAG 配置中的 `class_name` 动态实例化
- 对于消息驱动型模块，继承 `cyber::Component<M0>` 并实现 `bool Proc(const std::shared_ptr<M0>& msg)`

### 第四步：配置 BUILD 文件

模块根目录的 `BUILD` 文件使用 `apollo_component` 宏来构建共享库。

```python
load("//tools:apollo_package.bzl",
     "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# 如果有独立的业务逻辑库，先定义 cc_library
apollo_cc_library(
    name = "apollo_my_module",
    srcs = ["my_logic.cc"],
    hdrs = ["my_logic.h"],
    deps = [
        "//cyber",
        "//modules/my_module/proto:my_module_conf_cc_proto",
    ],
)

# 构建 Component 共享库（.so），Cyber RT 在运行时动态加载
apollo_component(
    name = "libmy_module_component.so",
    srcs = ["my_module_component.cc"],
    hdrs = ["my_module_component.h"],
    copts = ['-DMODULE_NAME=\\"my_module\\"'],
    deps = [
        "//cyber",
        ":apollo_my_module",
        "//modules/my_module/proto:my_module_conf_cc_proto",
    ],
)

# 运行时数据文件分组
filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()
cpplint()
```

注意事项：

- `apollo_component` 的 `name` 必须以 `lib` 开头、`.so` 结尾，这是 Cyber RT 加载共享库的约定
- `copts` 中的 `MODULE_NAME` 宏用于日志系统标识模块来源
- proto 依赖使用 `_cc_proto` 后缀，这是 Apollo 构建系统自动生成 C++ 绑定的命名规则
- 如果模块逻辑简单（如 guardian），可以不单独定义 `apollo_cc_library`，直接在 `apollo_component` 中包含所有源文件

### 第五步：编写 DAG 配置

DAG（Directed Acyclic Graph）文件告诉 Cyber RT 如何加载和调度你的 Component。

`dag/my_module.dag`：

```
module_config {
    module_library : "modules/my_module/libmy_module_component.so"
    timer_components {
        class_name : "MyModuleComponent"
        config {
            name: "my_module"
            config_file_path: "/apollo/modules/my_module/conf/my_module_conf.pb.txt"
            interval: 100
        }
    }
}
```

字段说明：

- `module_library`：指向 `apollo_component` 构建产物的路径（相对于 Apollo 工作空间根目录）
- `class_name`：必须与 `CYBER_REGISTER_COMPONENT()` 注册的类名完全一致
- `config_file_path`：protobuf text 格式的配置文件绝对路径
- `interval`：仅 `TimerComponent` 使用，单位为毫秒。`100` 表示每 100ms 调用一次 `Proc()`

如果使用消息驱动的 `Component<M0>`，DAG 配置略有不同：

```
module_config {
    module_library : "modules/my_module/libmy_module_component.so"
    components {
        class_name : "MyModuleComponent"
        config {
            name: "my_module"
            config_file_path: "/apollo/modules/my_module/conf/my_module_conf.pb.txt"
            readers {
                channel: "/apollo/some_input"
            }
        }
    }
}
```

### 第六步：编写 launch 文件

launch 文件是 Cyber RT 的启动入口，定义了要加载哪些 DAG 文件。

`launch/my_module.launch`：

```xml
<cyber>
    <module>
        <name>my_module</name>
        <dag_conf>/apollo/modules/my_module/dag/my_module.dag</dag_conf>
        <process_name>my_module</process_name>
    </module>
</cyber>
```

字段说明：

- `name`：模块名称，用于日志和监控标识
- `dag_conf`：DAG 文件的绝对路径
- `process_name`：进程名称。相同 `process_name` 的模块会运行在同一个进程中，不同的则各自独立进程

启动模块：

```bash
cyber_launch start /apollo/modules/my_module/launch/my_module.launch
```

### 第七步：编写 conf 配置

配置文件使用 protobuf text 格式，字段名和值必须与 proto 定义匹配。

`conf/my_module_conf.pb.txt`：

```
enable: true
process_rate: 10.0
output_topic: "/apollo/my_module/output"
```

这个文件会被 `GetProtoConfig()` 自动加载并解析为 `MyModuleConf` 消息对象。

### 第八步：编写 cyberfile.xml

`cyberfile.xml` 是 Apollo 包管理系统的描述文件，声明模块的元信息和依赖关系。

```xml
<package format="2">
  <name>my-module</name>
  <version>local</version>
  <description>
    My custom Apollo module.
  </description>

  <maintainer email="dev@example.com">Developer</maintainer>
  <license>Apache License 2.0</license>

  <type>module</type>
  <src_path>//modules/my_module</src_path>

  <!-- 运行时依赖 -->
  <depend type="binary" repo_name="cyber">cyber</depend>
  <depend type="binary" repo_name="common" lib_names="common">common</depend>
  <depend type="binary" repo_name="common-msgs" lib_names="common-msgs">common-msgs</depend>

  <!-- 构建工具依赖 -->
  <depend>bazel-extend-tools</depend>
  <depend expose="False">3rd-rules-python</depend>
  <depend expose="False">3rd-grpc</depend>
  <depend expose="False">3rd-bazel-skylib</depend>
  <depend expose="False">3rd-rules-proto</depend>
  <depend expose="False">3rd-py</depend>
</package>
```

## 最佳实践

### 线程安全

Guardian 模块展示了标准的线程安全模式：Reader 回调和 `Proc()` 运行在不同线程，共享数据必须加锁。

```cpp
// Reader 回调中写入
chassis_reader_ = node_->CreateReader<Chassis>(
    FLAGS_chassis_topic,
    [this](const std::shared_ptr<Chassis>& chassis) {
      std::lock_guard<std::mutex> lock(mutex_);
      chassis_.CopyFrom(*chassis);
    });

// Proc() 中读取
bool GuardianComponent::Proc() {
  std::lock_guard<std::mutex> lock(mutex_);
  // 安全地访问 chassis_
}
```

### 配置与代码分离

- 将所有可调参数放入 proto 配置，避免硬编码
- topic 名称、阈值、开关等都应该是配置项
- 使用 `[default = ...]` 为 proto 字段提供合理的默认值

### 模块分层

对于复杂模块，参考 storytelling 的做法：

- Component 类只负责生命周期管理和消息收发
- 业务逻辑拆分到独立的类中（如 `BaseTeller`、`CloseToJunctionTeller`）
- 使用 `apollo_cc_library` 单独构建业务逻辑库，Component 依赖它

### 日志规范

使用 Cyber RT 提供的日志宏，而非 `std::cout`：

```cpp
AINFO << "Module initialized successfully";
AWARN << "Configuration value out of expected range";
AERROR << "Failed to load config: " << ConfigFilePath();
ADEBUG << "Processing frame " << frame_id;
```

### 命名约定

| 元素 | 约定 | 示例 |
|------|------|------|
| 模块目录 | 小写下划线 | `modules/my_module/` |
| 共享库 | `lib` + 模块名 + `_component.so` | `libmy_module_component.so` |
| Component 类 | 大驼峰 | `MyModuleComponent` |
| Proto 包名 | `apollo.模块名` | `apollo.my_module` |
| 配置文件 | 模块名 + `_conf.pb.txt` | `my_module_conf.pb.txt` |
| DAG 文件 | 模块名 + `.dag` | `my_module.dag` |
| Launch 文件 | 模块名 + `.launch` | `my_module.launch` |
