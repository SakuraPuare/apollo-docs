# Apollo 自定义 Bazel 规则详解

## 概述

Apollo 自动驾驶平台基于 Bazel 构建系统，在 `tools/` 目录下定义了一套自定义的 Starlark 宏和规则，用于统一管理 C++/Python 编译、Protobuf 代码生成、组件打包、插件注册、代码风格检查以及第三方依赖配置等构建流程。

核心文件一览：

| 文件 | 职责 |
|------|------|
| `apollo_package.bzl` | 顶层构建宏，提供 `apollo_cc_library`、`apollo_component`、`apollo_plugin` 等核心规则 |
| `apollo.bzl` | Cyber 插件描述规则 `cyber_plugin_description` |
| `common.bzl` | 路径操作工具函数（`basename`、`dirname`、`join_paths` 等） |
| `workspace.bzl` | 第三方依赖统一初始化入口 |
| `cc_so_proto_rules.bzl` | 将 proto 编译为 C++ 动态链接库（`.so`） |
| `python_rules.bzl` | Python Protobuf / gRPC 代码生成规则 |
| `cpplint.bzl` | 自动为 C++ 目标添加 cpplint 检查 |
| `install/install.bzl` | 安装规则，控制产物的部署路径 |
| `platform/build_defs.bzl` | 平台条件选择宏（GPU、架构、ESD CAN 等） |
| `platform/common.bzl` | 仓库规则工具函数（文件拷贝、环境检测等） |
| `package/dynamic_deps.bzl` | 动态依赖状态常量（`STATUS`、`SOURCE`、`BINARY`） |
| `proto/proto.bzl` | Apollo 统一 proto 编译宏 `apollo_proto_library` |
| `ros/ros_configure.bzl` | ROS2 自动检测与配置仓库规则 |

---

## 逐个规则详解

### 1. `apollo_package.bzl` — 核心构建宏

这是 Apollo 构建体系中最重要的文件，提供了对原生 `cc_library`、`cc_binary`、`cc_test` 的封装，核心能力是**动态依赖填充**——根据 `package/dynamic_deps.bzl` 中的 `SOURCE` / `BINARY` 映射表，自动将源码包依赖替换为对应的二进制包或反向映射。

#### `apollo_cc_library`

将一个 C++ 库同时编译为动态链接库（`.so`）和静态引用目标。

```starlark
apollo_cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
    deps = [
        "//modules/common/util",
        "//modules/planning/proto:planning_cc_proto",
    ],
)
```

内部实现逻辑：
1. 将 `srcs` 和 `hdrs` 合并，通过 `CC_BINARY` 生成 `lib<name>.so`（`linkshared=True`）
2. 再用 `CC_LIBRARY` 将该 `.so` 作为 `srcs`，设置 `alwayslink=True`
3. 通过 `dynamic_fill_deps` 自动解析和替换依赖

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `string` | 目标名称 |
| `srcs` | `list[label]` | C++ 源文件 |
| `hdrs` | `list[label]` | 头文件 |
| `deps` | `list[label]` | 依赖列表，支持自动填充 |
| `copts` | `list[string]` | 编译选项 |
| `auto_find_deps` | `bool` | 是否自动查找并补全依赖（默认 `False`） |

#### `apollo_component`

用于构建 Cyber RT 组件（动态加载的 `.so` 模块）。名称必须以 `lib` 开头、`.so` 结尾。

```starlark
apollo_component(
    name = "libplanning_component.so",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    deps = [
        "//modules/planning:planning_base",
        "//cyber",
    ],
)
```

内部实现：
1. 创建一个带 `DO_NOT_IMPORT_` 前缀的内部 `apollo_cc_library`
2. 通过 `CC_BINARY` 以 `linkshared=True` 生成最终 `.so`

#### `apollo_plugin`

构建 Cyber 插件，与 `apollo_component` 类似但额外注册插件描述信息。

```starlark
apollo_plugin(
    name = "libsample_plugin.so",
    description = ":plugins/sample_plugin.xml",
    srcs = ["sample_plugin.cc"],
    hdrs = ["sample_plugin.h"],
    deps = ["//cyber"],
)
```

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `string` | 必须以 `lib` 开头、`.so` 结尾 |
| `description` | `label` | 插件描述 XML 文件 |
| `srcs` / `hdrs` / `deps` | 同 `cc_library` | 标准 C++ 构建参数 |

#### `apollo_cc_binary`

对 `cc_binary` 的简单封装，自动进行依赖填充。

```starlark
apollo_cc_binary(
    name = "planning_main",
    srcs = ["main.cc"],
    deps = ["//modules/planning:planning_base"],
)
```

#### `apollo_cc_test`

对 `cc_test` 的简单封装，自动进行依赖填充。

```starlark
apollo_cc_test(
    name = "planning_base_test",
    srcs = ["planning_base_test.cc"],
    deps = [
        "//modules/planning:planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)
```

#### `apollo_package`

在 BUILD 文件末尾调用，自动扫描当前包中已定义的所有规则，生成对应的 `install` 和 `install_src` 目标。

```starlark
# BUILD 文件末尾
apollo_package()
```

参数：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_source` | `bool` | `True` | 是否生成源码安装规则 |

#### `apollo_qt_library`

为 Qt5 项目提供 MOC / UIC / RCC 自动处理。

```starlark
apollo_qt_library(
    name = "dreamview_ui",
    srcs = ["main_window.cc"],
    hdrs = ["main_window.h"],
    moc_hdrs = ["main_window.h"],
    uis = ["main_window.ui"],
    res = ["resources.qrc"],
    deps = ["@qt5//:qt_widgets"],
)
```

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `moc_hdrs` | `list[label]` | 需要 MOC 处理的头文件 |
| `uis` | `list[label]` | Qt `.ui` 文件 |
| `res` | `list[label]` | Qt `.qrc` 资源文件 |
| `normal_hdrs` | `list[label]` | 不需要 MOC 的普通头文件 |
| `data` | `list[label]` | 资源编译所需的数据文件 |

#### `dynamic_fill_deps`

核心内部函数，当 `STATUS == 2`（二进制包模式）时，自动将依赖路径从源码路径替换为预编译二进制包路径。支持 `list` 和 `select` 两种依赖格式。

---

### 2. `apollo.bzl` — Cyber 插件描述规则

定义了 `cyber_plugin_description` 规则，用于将插件的描述文件（通常是 XML）注册到 `cyber_plugin_index` 目录中。

```starlark
cyber_plugin_description(
    name = "plugin_sample_description",
    plugin = ":libsample_plugin.so",
    description = ":plugins/sample_plugin.xml",
)
```

该规则在 `apollo_plugin` 宏中被自动调用，一般不需要手动使用。

内部实现：将 `description` 文件的路径写入一个以插件包路径和名称命名的索引文件，供 Cyber RT 运行时发现插件。

---

### 3. `common.bzl` — 路径工具函数库

提供了一组纯 Starlark 实现的路径操作函数，被 `install.bzl` 等多个文件依赖。

| 函数 | 说明 |
|------|------|
| `basename(p)` | 返回路径的文件名部分 |
| `dirname(p)` | 返回路径的目录部分 |
| `join_paths(path, *others)` | 类似 Python `os.path.join`，智能拼接路径 |
| `remove_prefix(path, prefix)` | 移除路径前缀，支持 glob 通配符 `*` |
| `output_path(ctx, input_file, strip_prefix)` | 计算文件的安装输出路径 |
| `clean_dep(dep)` | 清理依赖标签，确保子模块引用正确 |

使用示例：

```starlark
load("//tools:common.bzl", "join_paths", "basename")

path = join_paths("share", "modules/planning", "conf")
# => "share/modules/planning/conf"

name = basename("modules/planning/planning_base.h")
# => "planning_base.h"
```

---

### 4. `workspace.bzl` — 第三方依赖初始化

集中管理所有第三方依赖的加载和初始化，在 `WORKSPACE` 文件中调用。

#### `initialize_third_party()`

逐一调用各第三方库的 `repo()` 函数，注册外部仓库。涵盖的依赖包括：

- 基础库：`absl`、`glog`、`gflags`、`gtest`、`protobuf`、`boost`
- 数学/优化：`eigen`、`ipopt`、`osqp`、`adolc`
- 视觉/图形：`opencv`、`opengl`、`qt5`、`ffmpeg`、`nvjpeg`、`npp`
- 深度学习：`libtorch`（CPU/GPU）、`paddleinference`
- 通信：`fastrtps`
- 其他：`yaml_cpp`、`sqlite3`、`tinyxml2`、`uuid`、`proj` 等

#### `apollo_repositories()`

配置硬件相关的仓库规则，然后调用 `initialize_third_party()`：

```starlark
# WORKSPACE 文件中
load("//tools:workspace.bzl", "apollo_repositories")
apollo_repositories()
```

内部依次配置：
- `cuda_configure` — CUDA GPU 支持
- `rocm_configure` — ROCm GPU 支持
- `tensorrt_configure` — TensorRT 推理加速
- `python_configure` — Python 环境检测
- `vtk_configure` — VTK 可视化库
- `pcl_configure` — 点云库

---

### 5. `cc_so_proto_rules.bzl` — Proto 动态链接库生成

将 `proto_library` 编译为 C++ 动态链接库（`.so`），适用于需要在运行时动态加载 proto 序列化/反序列化代码的场景。

#### `cc_so_proto_library`

```starlark
cc_so_proto_library(
    name = "planning_proto_so",
    srcs = ["//modules/planning/proto:planning_proto"],
    deps = ["//modules/common/proto:common_proto_so"],
)
```

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `string` | 目标名称 |
| `srcs` | `list[label]` | 单个 `proto_library` 目标（仅支持一个） |
| `deps` | `list[label]` | 依赖的其他 `cc_so_proto_library` |
| `well_known_protos` | `bool` | 是否依赖 protobuf well-known types |

内部生成三个目标：
1. `_<name>_codegen` — 通过 `generate_cc` 生成 C++ 代码
2. `lib<name>.so` — 编译为动态链接库
3. `<name>` — `cc_library` 封装，供其他目标依赖

---

### 6. `python_rules.bzl` — Python Proto/gRPC 代码生成

从 `proto_library` 生成 Python protobuf 和 gRPC 桩代码。

#### `py_proto_library`

```starlark
load("//tools:python_rules.bzl", "py_proto_library")

py_proto_library(
    name = "planning_py_proto",
    deps = ["//modules/planning/proto:planning_proto"],
)
```

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `string` | 目标名称 |
| `deps` | `list[label]` | `proto_library` 目标列表 |
| `plugin` | `label` | 可选的自定义 protoc 插件 |

#### `py_grpc_library`

```starlark
load("//tools:python_rules.bzl", "py_grpc_library")

py_grpc_library(
    name = "planning_py_grpc",
    srcs = ["//modules/planning/proto:planning_proto"],
    deps = [":planning_py_proto"],
)
```

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `string` | 目标名称 |
| `srcs` | `list[label]` | 单个 `proto_library`（仅支持一个） |
| `deps` | `list[label]` | 单个 `py_proto_library`（仅支持一个） |
| `strip_prefixes` | `list[string]` | 从 import 路径中移除的前缀 |
| `plugin` | `label` | 可选的自定义 protoc 插件 |

---

### 7. `cpplint.bzl` — C++ 代码风格检查

自动为 BUILD 文件中所有 C++ 规则添加 cpplint 测试目标。

#### `cpplint`

在 BUILD 文件**末尾**调用，会扫描当前文件中所有已定义规则的 `srcs` 和 `hdrs`，为每个包含 C++ 源文件的规则生成一个 `<name>_cpplint` 测试目标。

```starlark
load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
)

# 必须放在 BUILD 文件末尾
cpplint()
```

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `data` | `list[label]` | 额外的 `CPPLINT.cfg` 配置文件 |
| `extra_srcs` | `list[label]` | 无法通过规则自动发现的额外源文件 |

支持的文件扩展名：`.c`、`.cc`、`.cpp`、`.cxx`、`.c++`、`.C`、`.h`、`.hh`、`.hpp`、`.hxx`、`.inc`

生成的测试目标带有 `cpplint` tag，可通过以下命令批量运行：

```bash
bazel test --test_tag_filters=cpplint //modules/planning/...
```

---

### 8. `install/install.bzl` — 安装部署规则

提供产物安装能力，控制编译产物（库、二进制、数据文件等）的部署路径。改编自 Drake 项目。

#### `install`

主安装规则，支持安装库文件、二进制文件、头文件和数据文件。

```starlark
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    targets = [":planning_component"],
    library_dest = "lib/modules/planning",
    runtime_dest = "bin",
    data_dest = "share/modules/planning",
    data = [":conf_files"],
)
```

关键参数：

| 参数 | 类型 | 说明 |
|------|------|------|
| `targets` | `list[label]` | 要安装的构建目标 |
| `library_dest` | `string` | 库文件安装路径 |
| `runtime_dest` | `string` | 可执行文件安装路径 |
| `data_dest` | `string` | 数据文件安装路径 |
| `data` | `list[label]` | 数据文件 |
| `rename` | `dict[string, string]` | 文件重命名映射 |
| `package_path` | `string` | 包路径标识 |
| `type` | `string` | 安装类型标识 |

路径中支持以下占位符：
- `@WORKSPACE@` — 当前工作区名称
- `@PACKAGE@` — 包名（`/` 替换为 `-`）
- `@PACKAGE_PATH@` — 包的完整路径

#### `install_files`

安装指定文件到目标路径。

```starlark
install_files(
    name = "install_conf",
    files = [":planning.conf"],
    dest = "share/modules/planning/conf",
)
```

#### `install_src_files`

安装源码文件，通常用于源码分发。

```starlark
install_src_files(
    name = "install_src",
    src_dir = [":src_files"],
    dest = "src/modules/planning",
)
```

#### `install_plugin`

专门用于安装 Cyber 插件及其描述文件。

```starlark
install_plugin(
    name = "install_plugin",
    plugin = ":libsample_plugin.so",
    description = ":plugins/sample_plugin.xml",
)
```

---

### 9. `platform/build_defs.bzl` — 平台条件选择宏

提供一组条件选择宏，用于根据编译平台和配置选项切换依赖或编译参数。

| 宏 | 说明 | 对应配置 |
|----|------|----------|
| `if_gpu(if_true, if_false)` | GPU 是否启用 | `//tools/platform:use_gpu` |
| `copts_if_gpu()` | GPU 编译宏 | `-DUSE_GPU=1` / `0` |
| `if_teleop(if_true, if_false)` | 远程操控是否启用 | `//tools/platform:with_teleop` |
| `copts_if_teleop()` | 远程操控编译宏 | `-DWITH_TELEOP=1` / `0` |
| `if_x86_64(if_true, if_false)` | x86_64 架构判断 | `@platforms//cpu:x86_64` |
| `if_aarch64(if_true, if_false)` | ARM64 架构判断 | `@platforms//cpu:aarch64` |
| `if_esd_can(if_true, if_false)` | ESD CAN 卡支持 | `//tools/platform:use_esd_can` |
| `copts_if_esd_can()` | ESD CAN 编译宏 | `-DUSE_ESD_CAN=1` / `0` |
| `if_profiler()` | 性能分析器开关 | `-DENABLE_PROFILER=1` / `0` |

使用示例：

```starlark
load("//tools/platform:build_defs.bzl", "if_gpu", "copts_if_gpu")

cc_library(
    name = "perception_inference",
    srcs = ["inference.cc"],
    deps = [
        "//modules/perception/base",
    ] + if_gpu(
        ["@local_config_cuda//cuda:cudart"],
        [],
    ),
    copts = copts_if_gpu(),
)
```

---

### 10. `platform/common.bzl` — 仓库规则工具函数

为 `repository_rule` 实现提供底层工具函数，被 CUDA、TensorRT、ROS 等配置规则共同依赖。

| 函数 | 说明 |
|------|------|
| `execute(repository_ctx, cmdline)` | 执行 shell 命令并返回结果 |
| `which(repository_ctx, program_name)` | 查找可执行文件路径 |
| `get_python_bin(repository_ctx)` | 获取 Python 解释器路径 |
| `get_bash_bin(repository_ctx)` | 获取 Bash 路径 |
| `read_dir(repository_ctx, src_dir)` | 递归列出目录下所有文件 |
| `get_host_environ(repository_ctx, name)` | 读取环境变量 |
| `make_copy_dir_rule(...)` | 生成目录拷贝的 `genrule` |
| `make_copy_files_rule(...)` | 生成文件拷贝的 `genrule` |
| `flag_enabled(repository_ctx, flag_name)` | 检查环境变量开关 |
| `tpl_gpus(repository_ctx, tpl, substitutions)` | 从模板生成 GPU 配置文件 |

---

### 11. `package/dynamic_deps.bzl` — 动态依赖状态

定义三个全局常量，控制 Apollo 的构建模式：

```starlark
STATUS = 0       # 0: 源码模式, 2: 二进制包模式
SOURCE = {}      # 源码包映射表
BINARY = {}      # 二进制包映射表
```

当 `STATUS == 2` 时，`apollo_package.bzl` 中的 `dynamic_fill_deps` 会自动将源码依赖替换为预编译的二进制包依赖，实现源码/二进制混合编译。

---

### 12. `proto/proto.bzl` — 统一 Proto 编译宏

#### `apollo_proto_library`

一次性生成 `proto_library`、`cc_proto_library` 和 `py_proto_library` 三个目标。

```starlark
load("//tools/proto:proto.bzl", "apollo_proto_library")

apollo_proto_library(
    name = "planning_proto",
    srcs = ["planning.proto"],
    deps = ["//modules/common/proto:common_proto"],
)
```

参数说明：

| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `string` | 目标名称 |
| `srcs` | `list[label]` | `.proto` 源文件 |
| `deps` | `list[label]` | 依赖的其他 `apollo_proto_library` |

自动生成的目标：
- `<name>` — 聚合规则，同时提供 `ProtoInfo`、`CcInfo`、`PyInfo`
- `<name>_cc_proto` — C++ proto 库（也可通过 `<name>` 直接获取 `CcInfo`）
- `<name>_py_proto` — Python proto 库

---

### 13. `ros/ros_configure.bzl` — ROS2 自动配置

自动检测系统中安装的 ROS2 发行版，生成对应的 Bazel 外部仓库。

```starlark
# WORKSPACE 文件中
load("//tools/ros:ros_configure.bzl", "ros_configure")
ros_configure(name = "local_config_ros")
```

检测逻辑：
1. 优先读取 `ROS_DISTRO` 环境变量
2. 否则扫描 `/opt/ros/` 目录（若存在多个发行版则报错）
3. 同时检测用户工作空间 `~/ros_ws/install/`
4. 自动收集头文件和库文件，生成 `cc_library` 规则

---

## 规则之间的关系和依赖

```
WORKSPACE
  └── workspace.bzl::apollo_repositories()
        ├── cuda_configure / rocm_configure / tensorrt_configure ...
        └── initialize_third_party()
              └── 各 third_party/*/workspace.bzl

BUILD 文件
  ├── apollo_package.bzl
  │     ├── apollo_cc_library ──→ dynamic_fill_deps ──→ package/dynamic_deps.bzl
  │     ├── apollo_component  ──→ apollo_cc_library
  │     ├── apollo_plugin     ──→ apollo.bzl::cyber_plugin_description
  │     ├── apollo_cc_binary  ──→ dynamic_fill_deps
  │     ├── apollo_cc_test    ──→ dynamic_fill_deps
  │     ├── apollo_qt_library
  │     └── apollo_package()  ──→ install/install.bzl (install, install_files, install_plugin)
  │
  ├── proto/proto.bzl::apollo_proto_library
  │     ├── proto_library (原生)
  │     ├── cc_proto_library ──→ _cc_proto_clean_rule
  │     └── py_proto_library ──→ python_rules.bzl
  │
  ├── platform/build_defs.bzl (if_gpu, if_x86_64, ...)
  │
  └── cpplint.bzl::cpplint()

底层工具
  ├── common.bzl (路径操作) ←── install.bzl, apollo.bzl
  └── platform/common.bzl (仓库工具) ←── ros_configure.bzl, cuda_configure 等
```

关键依赖关系：
- `apollo_package.bzl` 是最上层的构建宏，依赖 `install.bzl`、`apollo.bzl`、`dynamic_deps.bzl`
- `common.bzl` 是被最多文件依赖的基础工具库
- `platform/common.bzl` 是所有 `repository_rule` 的共享工具库
- `dynamic_deps.bzl` 通过 `STATUS` 控制整个构建系统的源码/二进制模式切换

---

## 使用示例

### 典型 BUILD 文件结构

```starlark
load("//tools:apollo_package.bzl",
    "apollo_cc_library",
    "apollo_cc_test",
    "apollo_component",
    "apollo_package",
)
load("//tools/proto:proto.bzl", "apollo_proto_library")
load("//tools/platform:build_defs.bzl", "if_gpu", "copts_if_gpu")
load("//tools:cpplint.bzl", "cpplint")

# 1. 定义 proto
apollo_proto_library(
    name = "planning_proto",
    srcs = ["proto/planning.proto"],
    deps = ["//modules/common/proto:common_proto"],
)

# 2. 定义库
apollo_cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
    deps = [
        ":planning_proto",
        "//modules/common/util",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
    copts = copts_if_gpu(),
)

# 3. 定义组件
apollo_component(
    name = "libplanning_component.so",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    deps = [":planning_base"],
)

# 4. 定义测试
apollo_cc_test(
    name = "planning_base_test",
    srcs = ["planning_base_test.cc"],
    deps = [
        ":planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

# 5. 自动生成安装规则（必须在所有规则之后）
apollo_package()

# 6. cpplint 检查（必须在最后）
cpplint()
```

### WORKSPACE 文件配置

```starlark
load("//tools:workspace.bzl", "apollo_repositories")
load("//tools/ros:ros_configure.bzl", "ros_configure")

apollo_repositories()
ros_configure(name = "local_config_ros")
```

---

## 常见问题

### Q: `apollo_component` 和 `apollo_cc_library` 有什么区别？

`apollo_cc_library` 生成一个可被其他目标静态链接的库（内部也会生成 `.so`）。`apollo_component` 专门用于 Cyber RT 组件，生成的 `.so` 由 Cyber RT 在运行时动态加载，不应被其他目标直接依赖。

### Q: 为什么 `apollo_component` 的 name 必须以 `lib` 开头、`.so` 结尾？

这是 Cyber RT 组件加载器的约定。加载器通过 DAG 配置文件中的库名查找对应的 `.so` 文件，命名格式必须符合 `lib<module_name>.so`。

### Q: `STATUS`、`SOURCE`、`BINARY` 是什么？

这三个常量定义在 `package/dynamic_deps.bzl` 中，控制 Apollo 的混合编译模式：
- `STATUS = 0`：纯源码编译模式，所有依赖从源码构建
- `STATUS = 2`：二进制包模式，`dynamic_fill_deps` 会将源码依赖自动替换为预编译包
- `SOURCE`：记录哪些包以源码形式存在
- `BINARY`：记录哪些包以预编译二进制形式存在

### Q: `apollo_package()` 应该放在 BUILD 文件的什么位置？

必须放在所有构建规则之后、`cpplint()` 之前。因为它通过 `native.existing_rules()` 扫描当前 BUILD 文件中已定义的所有规则来自动生成安装目标。

### Q: 如何为特定平台添加条件依赖？

使用 `platform/build_defs.bzl` 中的条件宏：

```starlark
load("//tools/platform:build_defs.bzl", "if_gpu", "if_x86_64")

apollo_cc_library(
    name = "my_lib",
    deps = [
        "//modules/common/util",
    ] + if_gpu(["@local_config_tensorrt//:tensorrt"])
      + if_x86_64(["//modules/perception/lib:x86_opt"]),
)
```

### Q: `cpplint()` 生成的测试目标如何运行？

```bash
# 运行单个目标的 cpplint 检查
bazel test //modules/planning:planning_base_cpplint

# 运行整个模块的 cpplint 检查
bazel test --test_tag_filters=cpplint //modules/planning/...
```

### Q: `apollo_plugin` 和 `apollo_component` 有什么区别？

`apollo_plugin` 在 `apollo_component` 的基础上增加了插件描述文件的注册（通过 `cyber_plugin_description`），使得 Cyber RT 的插件管理器能够发现和加载该插件。如果你的模块需要作为可插拔插件被动态发现，使用 `apollo_plugin`；如果只是普通的 Cyber 组件，使用 `apollo_component`。