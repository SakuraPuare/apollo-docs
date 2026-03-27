# Apollo BUILD 文件模式分析

## 1. 概述

Apollo 项目使用 Bazel 作为构建系统，并在原生 Bazel 规则之上封装了一套自定义宏（定义在 `tools/apollo_package.bzl` 和 `tools/proto/proto.bzl` 中），统一管理 C++ 库、组件、插件、Proto 和安装部署等构建目标。

核心自定义宏包括：

- `apollo_cc_library` — C++ 库目标，内部同时生成 `.so` 共享库和静态链接的 `cc_library`
- `apollo_cc_binary` — C++ 可执行文件目标
- `apollo_cc_test` — C++ 测试目标
- `apollo_component` — Cyber 组件目标，生成 `lib*.so` 动态加载库
- `apollo_plugin` — 插件目标，配合 `plugins.xml` 描述文件实现运行时插件发现
- `apollo_package` — 自动生成安装规则，必须在 BUILD 文件末尾调用
- `proto_library` — Proto 目标，自动生成 C++ 和 Python 绑定
- `cpplint` — 自动为所有 C++ 源文件生成 lint 检查测试

每个 BUILD 文件遵循固定的结构模板：

```starlark
# 1. load 语句
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", ...)
load("//tools:cpplint.bzl", "cpplint")

# 2. 包声明
package(default_visibility = ["//visibility:public"])

# 3. 编译选项常量
MODULE_COPTS = ['-DMODULE_NAME=\\"module_name\\"']

# 4. 库、组件、二进制、测试等目标定义
# ...

# 5. 文件组（运行时数据、测试数据）
# ...

# 6. 收尾调用（必须在末尾）
apollo_package()
cpplint()
```

## 2. 常见模式分析

### 2.1 包声明和可见性设置

所有模块统一使用公开可见性：

```starlark
package(default_visibility = ["//visibility:public"])
```

这使得跨模块依赖无需额外声明可见性。个别目标（如 `cyber` 核心库）会显式重复声明 `visibility = ["//visibility:public"]`，但这在 `default_visibility` 已设置的情况下是冗余的。

### 2.2 C++ 库目标（apollo_cc_library）的组织方式

`apollo_cc_library` 是使用最广泛的宏。它的内部实现会同时生成：
- 一个 `cc_binary`（`linkshared=True`），产出 `lib<name>.so` 共享库
- 一个 `cc_library`（`alwayslink=True`），以该 `.so` 作为源文件

这意味着每个 `apollo_cc_library` 最终都会产出一个独立的共享库，支持 Apollo 的包管理和动态部署机制。

典型用法 — 将模块内所有源文件聚合到一个库目标中：

```starlark
apollo_cc_library(
    name = "apollo_routing",
    srcs = [
        "common/routing_gflags.cc",
        "core/black_list_range_generator.cc",
        "core/navigator.cc",
        # ...
    ],
    hdrs = [
        "common/routing_gflags.h",
        "core/black_list_range_generator.h",
        # ...
    ],
    copts = ROUTING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/map:apollo_map",
        "//modules/routing/proto:routing_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        # ... 共 13 个依赖
    ],
)
```

命名约定：
- 模块主库通常命名为 `apollo_<module_name>`，如 `apollo_routing`、`apollo_prediction`
- 子库使用 `apollo_<module>_<sub>` 格式，如 `apollo_prediction_network`
- Cyber 子模块使用 `cyber_<sub>` 格式，如 `cyber_node`、`cyber_timer`

大型模块（如 prediction）会拆分为多个库层级：

```starlark
# 底层网络库
apollo_cc_library(
    name = "apollo_prediction_network",
    srcs = ["network/net_layer.cc", ...],
    deps = ["//cyber", "@eigen"],
)

# 主库依赖底层库
apollo_cc_library(
    name = "apollo_prediction",
    srcs = [...],  # 大量源文件
    deps = [
        ":apollo_prediction_network",  # 包内依赖
        "//cyber",
        # ...
    ],
)
```

### 2.3 组件目标（apollo_component）

Apollo 的 Cyber 框架使用组件（Component）作为运行时加载单元。`apollo_component` 宏要求输出名必须以 `lib` 开头、`.so` 结尾：

```starlark
apollo_component(
    name = "librouting_component.so",
    srcs = ["routing_component.cc"],
    hdrs = ["routing_component.h"],
    copts = ROUTING_COPTS,
    deps = [
        ":apollo_routing",
        "//cyber",
    ],
)
```

内部实现会生成一个带 `DO_NOT_IMPORT_` 前缀的中间 `cc_library`（如 `DO_NOT_IMPORT_routing_component`），然后将其链接为共享库。这个前缀是一个约定，表示该中间库不应被外部直接依赖，但在测试中会被引用：

```starlark
apollo_cc_test(
    name = "control_component_test",
    deps = [
        ":DO_NOT_IMPORT_control_component",  # 测试时引用中间库
        "@com_google_googletest//:gtest_main",
    ],
)
```

### 2.4 插件目标（apollo_plugin）

Apollo 的控制器、规划场景等采用插件架构，通过 `apollo_plugin` 宏定义：

```starlark
apollo_plugin(
    name = "liblat_controller.so",
    srcs = ["lat_controller.cc"],
    hdrs = ["lat_controller.h"],
    description = ":plugins.xml",  # 插件描述文件
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base:control_task",
        "//cyber",
        # ...
    ],
)
```

与 `apollo_component` 的区别：
- `apollo_plugin` 需要 `description` 属性指向 `plugins.xml` 文件
- 内部会生成 `cyber_plugin_description` 规则，用于插件元数据的安装和发现
- 插件库的中间 `cc_library` 命名为 `<name>_lib`（如 `lat_controller_lib`），可在测试中引用

规划场景同样使用插件模式：

```starlark
apollo_plugin(
    name = "liblane_follow_scenario.so",
    srcs = ["lane_follow_scenario.cc", "lane_follow_stage.cc"],
    hdrs = ["lane_follow_scenario.h", "lane_follow_stage.h"],
    description = ":plugins.xml",
    deps = [
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        # ...
    ],
)
```

### 2.5 二进制目标（apollo_cc_binary）的配置

二进制目标用于工具程序和独立可执行文件：

```starlark
# 工具程序
apollo_cc_binary(
    name = "topo_creator",
    srcs = ["topo_creator/topo_creator.cc"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":apollo_routing",
        "//modules/map:apollo_map",
    ],
)

# 共享库形式的子模块
apollo_cc_binary(
    name = "evaluator_submodule.so",
    linkshared = True,
    linkstatic = True,
    deps = [":apollo_prediction"],
)
```

特殊用法 — 以 `.so` 结尾的 `cc_binary` 用于生成可动态加载的子模块共享库，配合 `linkshared = True` 和 `linkstatic = True`。

DreamView 等应用程序会使用额外的链接选项：

```starlark
apollo_cc_binary(
    name = "dreamview_plus",
    srcs = ["main.cc"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    data = [":frontend"],
    linkopts = [
        "-ltcmalloc",
        "-lprofiler",
    ],
    deps = [
        "//cyber",
        "//modules/dreamview_plus/backend:apollo_dreamview_plus_backend",
    ],
)
```

### 2.6 测试目标的组织

测试统一使用 `apollo_cc_test`，依赖 Google Test 框架：

```starlark
apollo_cc_test(
    name = "topo_node_test",
    size = "small",                    # 测试大小标记
    srcs = ["graph/topo_node_test.cc"],
    deps = [
        ":apollo_routing",
        "@com_google_googletest//:gtest_main",
    ],
)
```

常见模式：
- `size = "small"` — 几乎所有测试都标记为 small
- 测试数据通过 `data` 属性引用 `filegroup`
- 部分测试需要 `linkstatic = True`（尤其是涉及 Cyber 框架的测试）
- 使用 `-fno-access-control` 编译选项访问私有成员

```starlark
apollo_cc_test(
    name = "junction_predictor_test",
    size = "small",
    srcs = ["predictor/junction/junction_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)
```

### 2.7 Proto 相关目标

Proto 文件放在模块的 `proto/` 子目录中，使用自定义 `proto_library` 宏（来自 `tools/proto/proto.bzl`）：

```starlark
load("//tools/proto:proto.bzl", "proto_library")

proto_library(
    name = "routing_config_proto",
    srcs = ["routing_config.proto"],
)

proto_library(
    name = "topo_graph_proto",
    srcs = ["topo_graph.proto"],
    deps = [
        "//modules/common_msgs/map_msgs:map_geometry_proto",
    ],
)
```

命名约定：
- 目标名必须以 `_proto` 结尾
- 该宏自动生成多个派生目标：
  - `_<name>` — 原始 proto_library
  - `_<name>_cc_proto` — C++ proto 库
  - `<name>_py_pb2` — Python proto 绑定
  - `lib_<name>_*.so` — 共享库形式的 proto 实现
- 引用 C++ proto 时使用 `<name>_cc_proto` 后缀，如 `routing_config_cc_proto`

Cyber 框架的 proto BUILD 文件由 `proto_build_generator.py` 自动生成：

```starlark
## Auto generated by `proto_build_generator.py`
load("//tools/proto:proto.bzl", "proto_library")

proto_library(
    name = "topology_change_proto",
    srcs = ["topology_change.proto"],
    deps = [":role_attributes_proto"],
)
```

### 2.8 Python 目标

Python 目标使用标准的 `py_library` 规则，主要出现在 `cyber/python/` 目录中：

```starlark
load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "cyber",
    srcs = ["cyber.py"],
    data = [
        "//cyber/python/internal:_cyber_wrapper.so",
    ],
)

py_library(
    name = "record",
    srcs = ["record.py"],
    data = [
        "//cyber/python/internal:_cyber_record_wrapper.so",
    ],
)
```

Python 绑定通过 `data` 属性引用 C++ 编译的 `.so` 包装器，实现 Python 对 Cyber 框架的调用。

### 2.9 依赖管理模式

Apollo 的依赖分为四类：

1. Cyber 框架依赖 — 直接引用 `//cyber`：
```starlark
deps = ["//cyber"]
```

2. 模块间依赖 — 使用完整路径：
```starlark
deps = [
    "//modules/common/configs:vehicle_config_helper",
    "//modules/map:apollo_map",
]
```

3. Proto 依赖 — 分为公共消息和模块私有 proto：
```starlark
deps = [
    # 公共消息（modules/common_msgs/）
    "//modules/common_msgs/routing_msgs:routing_cc_proto",
    "//modules/common_msgs/planning_msgs:planning_cc_proto",
    # 模块私有 proto
    "//modules/routing/proto:routing_config_cc_proto",
]
```

4. 第三方依赖 — 使用 `@` 前缀引用外部仓库：
```starlark
deps = [
    "@com_github_gflags_gflags//:gflags",
    "@com_google_googletest//:gtest_main",
    "@com_google_absl//:absl",
    "@eigen",
    "@opencv//:highgui",
]
```

条件依赖通过平台选择宏实现：

```starlark
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")

apollo_cc_library(
    name = "apollo_prediction",
    srcs = [...] + if_aarch64(["common/affine_transform.cc"]),
    hdrs = [...] + if_aarch64(["common/affine_transform.h"]),
    deps = [...] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)
```

### 2.10 运行时数据和文件组

每个可部署模块都定义 `runtime_data` 文件组，包含配置、DAG 和 launch 文件：

```starlark
filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)
```

测试数据单独组织：

```starlark
filegroup(
    name = "test_data",
    srcs = glob(["testdata/**"]),
)

filegroup(
    name = "prediction_data",
    srcs = glob([
        "data/*.pt",
        "data/*.bin",
    ]),
)
```

### 2.11 模块名称宏和编译选项

每个模块定义一个 `MODULE_COPTS` 常量，通过 `-DMODULE_NAME` 注入模块名称：

```starlark
ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']
CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']
PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]
```

注意转义方式存在两种风格（`\\"` 和 `\\\"`），功能等价，但项目中并未完全统一。

### 2.12 cpplint 和 apollo_package 收尾

每个 BUILD 文件末尾必须调用这两个函数：

```starlark
apollo_package()  # 自动生成安装规则
cpplint()         # 自动生成 C++ lint 检查
```

`apollo_package()` 会遍历当前 BUILD 文件中所有已定义的规则，自动生成对应的 `install` 目标，支持 Apollo 的包管理系统。它还会递归收集子包的安装目标。

`cpplint()` 会为每个包含 C++ 源文件的规则生成一个 `py_test`，运行 Google cpplint 检查代码风格。

## 3. 模块 BUILD 文件示例解析

### 3.1 routing 模块 — 典型的完整模块

文件路径：`modules/routing/BUILD`

routing 模块是一个结构清晰的中等规模模块，展示了 Apollo BUILD 文件的标准模式：

```starlark
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library",
     "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

# ---- 核心库 ----
# 将模块所有源文件聚合为一个库
apollo_cc_library(
    name = "apollo_routing",
    srcs = [
        "common/routing_gflags.cc",
        "core/black_list_range_generator.cc",
        "core/navigator.cc",
        "core/result_generator.cc",
        "graph/node_with_range.cc",
        # ... 更多源文件
    ],
    hdrs = [
        "common/routing_gflags.h",
        "core/black_list_range_generator.h",
        # ... 更多头文件
    ],
    copts = ROUTING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/map:apollo_map",
        "//modules/routing/proto:routing_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        # ... 共 13 个依赖
    ],
)

# ---- Cyber 组件（运行时入口）----
apollo_component(
    name = "librouting_component.so",
    srcs = ["routing_component.cc"],
    hdrs = ["routing_component.h"],
    copts = ROUTING_COPTS,
    deps = [":apollo_routing", "//cyber"],
)

# ---- 运行时数据 ----
filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf", "conf/*.pb.txt",
        "dag/*.dag", "launch/*.launch",
    ]),
)

# ---- 工具程序 ----
apollo_cc_binary(
    name = "topo_creator",
    srcs = ["topo_creator/topo_creator.cc"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [":apollo_routing", "//modules/map:apollo_map"],
)

# ---- 测试 ----
apollo_cc_test(
    name = "topo_node_test",
    size = "small",
    srcs = ["graph/topo_node_test.cc"],
    deps = [":apollo_routing", "@com_google_googletest//:gtest_main"],
)
# ... 更多测试

# ---- 收尾 ----
apollo_package()
cpplint()
```

结构层次：`apollo_cc_library`（核心库） -> `apollo_component`（组件入口） + `apollo_cc_binary`（工具） + `apollo_cc_test`（测试）。组件和工具都依赖核心库，形成清晰的依赖树。

### 3.2 lat_based_lqr_controller — 插件模式

文件路径：`modules/control/controllers/lat_based_lqr_controller/BUILD`

控制器作为插件实现，展示了 `apollo_plugin` 的用法：

```starlark
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_test", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

# ---- 插件定义 ----
apollo_plugin(
    name = "liblat_controller.so",
    srcs = ["lat_controller.cc"],
    hdrs = ["lat_controller.h"],
    description = ":plugins.xml",       # 插件元数据
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base:control_task",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/control/controllers/lat_based_lqr_controller/proto:lat_based_lqr_controller_conf_cc_proto",
        "@eigen",
        # ... 共 20 个依赖
    ],
)

# ---- 测试（引用插件生成的 _lib 中间库）----
apollo_cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = ["lat_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["lateral_controller_test"],
    deps = [
        ":lat_controller_lib",          # 插件的中间库
        "//cyber",
        "@com_google_googletest//:gtest_main",
        # ... 共 7 个依赖
    ],
)

filegroup(
    name = "lateral_controller_test",
    srcs = glob(["lateral_controller_test/**"]) + glob(["conf/*"]),
)

apollo_package()
cpplint()
```

插件模式的关键点：
- `description = ":plugins.xml"` 声明插件描述文件
- `apollo_plugin` 内部生成 `lat_controller_lib`（`cc_library`）和 `liblat_controller.so`（`cc_binary`）
- 测试依赖 `:lat_controller_lib` 而非 `.so` 文件

## 4. BUILD 文件编写最佳实践

### 4.1 文件结构

- 始终按照 load -> package -> copts -> library -> component/plugin -> binary -> test -> filegroup -> apollo_package -> cpplint 的顺序组织
- `apollo_package()` 和 `cpplint()` 必须放在文件末尾，因为它们会遍历已定义的所有规则

### 4.2 库目标设计

- 每个模块提供一个主库目标（`apollo_<module>`），聚合模块内所有源文件
- 如果模块较大，可拆分为层级库（如 `apollo_prediction_network` + `apollo_prediction`）
- 避免过度拆分 — Apollo 倾向于粗粒度的库目标，而非每个文件一个目标

### 4.3 组件与插件选择

- 模块的主入口使用 `apollo_component`（如 `librouting_component.so`）
- 可扩展的子功能使用 `apollo_plugin`（如控制器、规划场景）
- 插件必须提供 `plugins.xml` 描述文件

### 4.4 依赖管理

- 优先依赖模块的主库目标，而非内部子目标
- Proto 依赖使用 `_cc_proto` 后缀引用 C++ 绑定
- 公共消息放在 `modules/common_msgs/` 下，模块私有 proto 放在模块的 `proto/` 子目录
- 第三方依赖通过 `@` 引用 WORKSPACE 中定义的外部仓库

### 4.5 测试组织

- 测试文件与被测源文件放在同一模块目录下
- 测试数据通过 `filegroup` + `glob` 收集，通过 `data` 属性传递
- 组件测试引用 `DO_NOT_IMPORT_` 前缀的中间库
- 插件测试引用 `_lib` 后缀的中间库

### 4.6 平台适配

- 使用 `if_gpu()`、`if_aarch64()` 等选择宏处理平台差异
- 避免在 BUILD 文件中硬编码平台判断

## 5. 常见问题

### Q: 为什么 `apollo_cc_library` 会生成 `.so` 文件？

Apollo 的包管理系统要求每个库都以共享库形式存在，以支持二进制包的分发和动态加载。`apollo_cc_library` 宏内部会同时生成一个 `cc_binary`（`linkshared=True`）产出 `.so`，以及一个 `cc_library`（`alwayslink=True`）用于 Bazel 依赖图中的符号传递和链接。

### Q: `DO_NOT_IMPORT_` 前缀是什么意思？

这是 `apollo_component` 宏生成的中间 `cc_library` 的命名约定。例如 `apollo_component(name = "librouting_component.so")` 会生成 `DO_NOT_IMPORT_routing_component`。这个库包含组件的实际实现代码，但不应被其他模块直接依赖 — 它仅用于链接生成 `.so` 和在测试中引用。

### Q: `apollo_package()` 必须放在末尾吗？

是的。`apollo_package()` 调用 `native.existing_rules()` 遍历当前 BUILD 文件中已定义的所有规则，为每个规则自动生成安装目标。如果在中间调用，后续定义的规则将不会被包含在安装目标中。

### Q: Proto 目标的命名规则是什么？

`proto_library` 宏要求目标名以 `_proto` 结尾（如 `routing_config_proto`）。引用时：
- C++ 代码使用 `<name>_cc_proto`（如 `routing_config_cc_proto`）
- Python 代码使用 `<name>_py_pb2`（如 `routing_config_py_pb2`）
- 直接引用 proto 信息使用原名（如 `routing_config_proto`）

### Q: 如何添加新的控制器或规划场景？

1. 在对应目录下创建源文件和 `plugins.xml`
2. 在 BUILD 文件中使用 `apollo_plugin` 定义插件目标
3. 如果有私有配置 proto，在 `proto/` 子目录创建 BUILD 文件使用 `proto_library`
4. 添加 `apollo_cc_test` 测试目标
5. 末尾调用 `apollo_package()` 和 `cpplint()`

### Q: `cpplint()` 做了什么？

`cpplint()` 遍历当前 BUILD 文件中所有规则的 `srcs` 和 `hdrs`，为每个包含 C++ 源文件的规则生成一个 `py_test` 目标（名为 `<rule_name>_cpplint`），运行 Google cpplint 工具检查代码风格。配置文件从 `//tools:CPPLINT.cfg`（即 `tools/` 目录下的 `CPPLINT.cfg`）和当前目录的 `CPPLINT.cfg` 中读取。
