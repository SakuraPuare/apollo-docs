---
title: "Apollo 构建系统"
---

# Apollo 构建系统

## 1. 概述

Apollo 自动驾驶平台采用 [Bazel](https://bazel.build/) 作为其核心构建系统。Bazel 是 Google 开源的构建工具，特别适合 Apollo 这类大规模、多语言（C++、Python、Protobuf）的单体仓库项目。

选择 Bazel 的主要原因：

- **可重现构建**：通过 SHA256 校验和锁定所有外部依赖，确保构建结果的确定性
- **增量编译**：精确的依赖图分析使得只重新编译变更部分，大幅缩短迭代时间
- **多语言支持**：原生支持 C++、Python、Protobuf，并通过 `rules_foreign_cc` 集成 CMake 等外部构建系统
- **GPU 构建支持**：通过自定义配置规则无缝切换 NVIDIA CUDA / AMD ROCm 平台
- **远程缓存**：支持远程构建缓存（配置中已预留 `remote_cache` 接口）

Apollo 要求的最低 Bazel 版本为 **3.7.0**（在 WORKSPACE 中通过 `bazel_skylib` 的 `versions.check` 强制校验）。

## 2. WORKSPACE 配置解析

WORKSPACE 文件（`WORKSPACE` 和 `WORKSPACE.source` 内容一致）定义了工作区名称和所有外部依赖。

### 2.1 工作区声明

```starlark
workspace(name = "apollo")
```

将整个仓库注册为名为 `apollo` 的 Bazel 工作区，所有内部目标均可通过 `@apollo//` 引用。

### 2.2 核心仓库规则

WORKSPACE 通过 `http_archive` 引入以下 Bazel 官方规则集：

| 规则集 | 版本 | 用途 |
|--------|------|------|
| `rules_foreign_cc` | 0.8.0 | 集成 CMake/Make 等外部构建系统 |
| `rules_cc` | 0.0.1 | C/C++ 编译规则（含 Apollo 自定义补丁 `//tools/package:rules_cc.patch`） |
| `bazel_skylib` | 1.0.3 | Bazel 通用工具库，提供版本检查等功能 |
| `rules_proto` | 97d8af4 | Protocol Buffers 编译规则 |
| `rules_python` | 0.1.0 | Python 构建规则 |

### 2.3 gRPC 与 Protobuf

```starlark
http_archive(
    name = "com_github_grpc_grpc",
    sha256 = "2378b608557a4331c6a6a97f89a9257aee2f8e56a095ce6619eea62e288fcfbe",
    patches = ["//third_party/absl:grpc.patch"],
    strip_prefix = "grpc-1.30.0",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/8.0/v1.30.0-apollo.tar.gz",
    ],
)
```

Apollo 使用 gRPC 1.30.0（Apollo 定制版本），并应用了自定义补丁。同时引入 zlib 1.2.11 作为 Protobuf 的压缩依赖。

gRPC 的传递依赖通过 `grpc_deps()` 和 `grpc_extra_deps()` 自动加载。

### 2.4 Apollo 自定义仓库（apollo_repositories）

`apollo_repositories()` 定义在 `tools/workspace.bzl` 中，负责配置平台检测和加载所有第三方库：

**平台自动检测（configure 规则）：**

```starlark
cuda_configure(name = "local_config_cuda")       # NVIDIA CUDA 工具链
rocm_configure(name = "local_config_rocm")       # AMD ROCm 工具链
tensorrt_configure(name = "local_config_tensorrt") # NVIDIA TensorRT
python_configure(name = "local_config_python")     # Python 解释器
vtk_configure(name = "local_config_vtk")           # VTK 可视化库
pcl_configure(name = "local_config_pcl")           # PCL 点云库
```

这些 `configure` 规则会在构建时自动探测本地系统环境，生成对应的 BUILD 文件。

**第三方库加载（initialize_third_party）：**

通过 `initialize_third_party()` 函数统一加载约 39 个第三方依赖，涵盖以下类别：

- **基础库**：abseil-cpp、Boost、gflags、glog、Protobuf、yaml-cpp
- **数学/优化**：Eigen3、IPOPT、OSQP、ADOLC、ATLAS
- **感知/推理**：OpenCV、PCL、LibTorch、PaddleInference、TensorRT、CenterPoint、CADDN
- **通信**：Fast-RTPS/Fast-DDS、gRPC、CivetWeb
- **可视化**：Qt5、VTK、OpenGL、~~GLEW~~（注：GLEW 在 `tools/workspace.bzl` 中的加载已被注释掉，当前未实际引入）
- **多媒体**：FFmpeg、OpenH264、PortAudio、FFTW3、nvJPEG
- **工具**：cpplint、Google Test、Google Benchmark、SQLite3、tinyxml2、nlohmann_json

### 2.5 镜像加速

所有外部依赖均配置了双 URL 源，使用两种百度 CDN 域名：

```starlark
# 大多数依赖使用 cdn.bcebos.com：
urls = [
    "https://apollo-system.cdn.bcebos.com/archive/6.0/...",  # 百度 CDN 镜像（国内加速）
    "https://github.com/...",                                  # GitHub 原始源
]

# 部分依赖（如 rules_foreign_cc）使用 bj.bcebos.com：
urls = [
    "https://apollo-system.bj.bcebos.com/archive/...",        # 百度 BJ 镜像
    "https://github.com/...",                                  # GitHub 原始源
]
```

Bazel 会按顺序尝试下载，国内环境优先使用百度 CDN 镜像以提升下载速度。

## 3. .bazelrc 编译选项解析

Apollo 的 Bazel 配置采用分层加载机制。根目录 `.bazelrc` 通过 `try-import` 引入实际配置：

```starlark
try-import %workspace%/tools/bazel.rc       # 主配置文件
try-import %workspace%/.apollo.bazelrc       # Apollo 环境专用配置（可选）
try-import %workspace%/.custom.bazelrc       # 用户自定义配置（可选）
```

核心配置集中在 `tools/bazel.rc` 中，按功能分为以下几个部分。

### 3.1 启动选项（Startup Options）

```starlark
startup --batch_cpu_scheduling
startup --host_jvm_args="-XX:-UseParallelGC"
```

- `--batch_cpu_scheduling`：使用批处理 CPU 调度策略，降低 Bazel 服务端对系统资源的争抢
- `-XX:-UseParallelGC`：禁用 JVM 并行垃圾回收器，减少 GC 暂停对构建的影响

### 3.2 构建配置（Build Configurations）

**基础编译选项：**

```starlark
build --show_timestamps
build --spawn_strategy=standalone
build --cxxopt="-fdiagnostics-color=always"
build --cxxopt="-std=c++14"
build --host_cxxopt="-std=c++14"
```

- 默认使用 C++14 标准编译
- 使用 `standalone` 沙箱策略（绕过沙箱限制）
- 启用 GCC 彩色输出

**编译警告控制：**

```starlark
build --per_file_copt=external/upb/.*@-Wno-sign-compare
build --copt="-Werror=return-type"
build --copt="-Werror=unused-but-set-variable"
build --copt="-Werror=switch"
build --cxxopt="-Werror=reorder"
```

将关键警告提升为错误，包括：缺少返回值、未使用的变量、switch 缺少分支、成员初始化顺序不一致。对外部依赖 `upb` 则放宽符号比较警告。

**系统路径定义：**

```starlark
build --define=PREFIX=/usr
build --define=LIBDIR=$(PREFIX)/lib
build --define=INCLUDEDIR=$(PREFIX)/include
build --define=use_fast_cpp_protos=true
```

**GPU 平台预定义值：**

`tools/bazel.rc` 中还定义了全局默认的 GPU 平台标志：

```starlark
build --define NVIDIA=0
build --define AMD=1
```

这些默认值会被具体的 `--config=nvidia` 或 `--config=amd` 配置覆盖。

### 3.3 GPU 平台配置

Apollo 支持 NVIDIA 和 AMD 两种 GPU 平台，通过 `--config` 切换：

**NVIDIA 平台（`--config=gpu` 或 `--config=nvidia`）：**

```starlark
build:nvidia --define GPU_PLATFORM=NVIDIA
build:nvidia --cxxopt="-DGPU_PLATFORM=NVIDIA"
build:nvidia --define USE_GPU=true
build:nvidia --cxxopt="-DUSE_GPU=1"
build:nvidia --cxxopt="-DNVIDIA=1"
```

**AMD 平台（`--config=amd`）：**

```starlark
build:amd --define GPU_PLATFORM=AMD
build:amd --cxxopt="-DGPU_PLATFORM=AMD"
build:amd --define USE_GPU=true
build:amd --cxxopt="-DUSE_GPU=1"
build:amd --cxxopt="-DAMD=1"
```

**CPU-only 模式（`--config=cpu`）：**

```starlark
build:cpu --verbose_failures
```

`--config=gpu` 是 `--config=nvidia` 的别名。

### 3.4 其他构建配置

**调试与优化：**

```starlark
build:dbg -c dbg    # 调试模式
build:opt -c opt    # 优化模式
```

**性能分析：**

```starlark
build:prof --linkopt=-lprofiler
build:prof --cxxopt="-DENABLE_PERF=1"
```

**C++17 支持：**

```starlark
build:c++17 --cxxopt=-std=c++1z
build:c++1z --config=c++17
```

默认使用 C++14，需要 C++17 特性时通过 `--config=c++17` 启用。

### 3.5 测试配置（Test Configurations）

```starlark
test --flaky_test_attempts=3
test --test_size_filters=small,medium
test --test_output=errors
```

- 不稳定测试自动重试 3 次
- 默认只运行 small 和 medium 规模的测试
- 仅输出失败测试的详细信息

**cpplint 代码风格检查：**

```starlark
test:cpplint --test_tag_filters=cpplint
test:cpplint --build_tests_only
test:cpplint --test_timeout=3600
test:cpplint --flaky_test_attempts=1
```

**单元测试（排除 cpplint）：**

```starlark
test:unit_test --test_tag_filters=-cpplint
test:unit_test --test_verbose_timeout_warnings
```

### 3.6 覆盖率配置（Coverage）

```starlark
coverage --instrument_test_targets
coverage --combined_report=lcov
coverage --nocache_test_results
coverage --javabase="@bazel_tools//tools/jdk:remote_jdk11"
coverage --cxxopt=--coverage
coverage --cxxopt=-fprofile-arcs
coverage --cxxopt=-ftest-coverage
coverage --linkopt=-lgcov
coverage --test_tag_filters=-cpplint
```

使用 GCC 的 `gcov` 工具链生成 LCOV 格式的覆盖率报告，排除 cpplint 测试。

## 4. 根 BUILD 文件解析

根目录的 `BUILD` 文件定义了全局包配置和顶层安装目标。

### 4.1 全局可见性与导出文件

```starlark
load("//tools/install:install.bzl", "install", "install_src_files")
load("//third_party/gpus:common.bzl", "if_gpu")

package(
    default_visibility = ["//visibility:public"],
)

exports_files([
    "CPPLINT.cfg",
    "tox.ini",
])
```

- `default_visibility = ["//visibility:public"]`：根包下的所有目标默认对整个工作区可见
- 导出 `CPPLINT.cfg`（C++ 代码风格配置）和 `tox.ini`（Python 测试配置）供子包引用

### 4.2 安装目标（install）

根 BUILD 文件定义了两个核心安装目标：

**`deprecated_install`** — 安装编译产物（二进制、库文件）：

```starlark
install(
    name = "deprecated_install",
    deps = if_gpu(
        [ ... GPU 依赖列表 ... ],
        [ ... CPU 依赖列表 ... ],
    ),
)
```

**`deprecated_install_src`** — 安装源文件和头文件：

```starlark
install_src_files(
    name = "deprecated_install_src",
    deps = if_gpu(
        [ ... GPU 源文件依赖 ... ],
        [ ... CPU 源文件依赖 ... ],
    ),
)
```

### 4.3 GPU 条件编译（if_gpu）

`if_gpu` 宏来自 `//third_party/gpus:common.bzl`，根据是否启用 GPU 选择不同的依赖列表：

- **GPU 模式**额外包含：`paddleinference`、`caddn_infer_op`（GPU 专用推理库）。注意 `tensorrt`、`npp`、`nvjpeg` 在 GPU 和 CPU 列表中均存在
- **CPU 模式**排除 GPU 专用推理库（`paddleinference`、`caddn_infer_op`），但仍包含 `centerpoint_infer_op` 等可在 CPU 上运行的组件

安装目标覆盖约 45 个第三方库（GPU 模式）或约 43 个（CPU 模式），以及 `//scripts` 和 `//tools` 两个内部包。

## 5. 使用示例

### 5.1 常用构建命令

```bash
# 构建整个项目（CPU 模式）
bazel build --config=cpu //...

# 构建整个项目（NVIDIA GPU 模式）
bazel build --config=gpu //...

# 构建整个项目（AMD GPU 模式）
bazel build --config=amd //...

# 构建特定模块（以 planning 为例）
bazel build --config=gpu //modules/planning/...

# 调试模式构建
bazel build --config=gpu --config=dbg //modules/planning/...

# 优化模式构建
bazel build --config=gpu --config=opt //modules/planning/...

# 使用 C++17 标准构建
bazel build --config=gpu --config=c++17 //modules/planning/...
```

### 5.2 测试命令

```bash
# 运行所有单元测试
bazel test --config=gpu --config=unit_test //modules/planning/...

# 运行 cpplint 代码风格检查
bazel test --config=cpplint //modules/planning/...

# 生成代码覆盖率报告
bazel coverage --config=gpu //modules/planning/...
```

### 5.3 性能分析构建

```bash
# 启用 gperftools 性能分析
bazel build --config=gpu --config=prof //modules/planning/...
```

### 5.4 查询依赖关系

```bash
# 查看某个目标的所有依赖
bazel query 'deps(//modules/planning:planning_component)'

# 查看某个目标的反向依赖
bazel query 'rdeps(//..., //modules/common/math:math)'

# 可视化依赖图
bazel query 'deps(//modules/planning:planning_component)' --output graph | dot -Tpng > deps.png
```

### 5.5 自定义配置

创建 `.custom.bazelrc` 文件可覆盖默认配置而不影响版本控制：

```starlark
# .custom.bazelrc 示例
build --jobs=16
build --local_ram_resources=HOST_RAM*0.7
build --remote_cache=http://your-cache-server:8080
```

## 6. 常见问题

### Q1: 构建时提示 Bazel 版本不兼容

Apollo 要求 Bazel >= 3.7.0。检查当前版本：

```bash
bazel version
```

建议使用 [Bazelisk](https://github.com/bazelbuild/bazelisk) 自动管理 Bazel 版本。

### Q2: 外部依赖下载失败

Apollo 的依赖默认从百度 CDN 下载，如果网络不通，会回退到 GitHub。可以通过设置代理或使用本地镜像解决：

```bash
# 设置 HTTP 代理
export http_proxy=http://your-proxy:port
export https_proxy=http://your-proxy:port
```

也可以手动下载依赖包放到 Bazel 缓存目录 `~/.cache/bazel/` 中。

### Q3: GPU 相关构建错误

确保正确安装了 CUDA/cuDNN/TensorRT（NVIDIA）或 ROCm（AMD），并且环境变量配置正确：

```bash
# 检查 CUDA 安装
nvcc --version
echo $CUDA_HOME

# 检查 TensorRT
dpkg -l | grep tensorrt
```

如果不需要 GPU 支持，使用 `--config=cpu` 构建。

### Q4: 沙箱相关错误

Apollo 默认使用 `--spawn_strategy=standalone` 绕过沙箱。如果仍遇到沙箱问题，可在 `.custom.bazelrc` 中添加：

```starlark
build --sandbox_debug
```

查看详细的沙箱日志以定位问题。

### Q5: 构建速度慢

几个优化建议：

- 启用远程缓存：在 `.custom.bazelrc` 中配置 `--remote_cache`
- 调整并行度：`--jobs=N`（N 为 CPU 核心数）
- 限制内存使用：`--local_ram_resources=HOST_RAM*0.7`
- 只构建需要的目标，避免 `//...` 全量构建

### Q6: cpplint 检查不通过

Apollo 使用 Google C++ 代码风格。确保 BUILD 文件中加载了 cpplint 规则：

```starlark
load("//tools:cpplint.bzl", "cpplint")

# ... 其他规则 ...

cpplint()
```

代码风格配置位于根目录的 `CPPLINT.cfg` 文件中。

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
| `proto/proto.bzl` | Apollo 统一 proto 编译宏 `proto_library` |
| `ros/ros_configure.bzl` | ROS2 自动检测与配置仓库规则 |

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

### 12. `proto/proto.bzl` — 统一 Proto 编译宏

#### `proto_library`

一次性生成 `proto_library`、`cc_proto_library` 和 `py_proto_library` 三个目标。

```starlark
load("//tools/proto:proto.bzl", "proto_library")

proto_library(
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
| `deps` | `list[label]` | 依赖的其他 `proto_library` |

自动生成的目标：

- `<name>` — 聚合规则，同时提供 `ProtoInfo`、`CcInfo`、`PyInfo`
- `_<name>_cc_proto` — C++ proto 库（也可通过 `<name>` 直接获取 `CcInfo`）
- `<name>_py_pb2` — Python proto 库

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
  ├── proto/proto.bzl::proto_library
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

`apollo_plugin` 在 `apollo_component` 的基础上增加了插件描述文件的注册（通过 `cyber_plugin_description`），使得 Cyber RT 的插件管理器能够发现和加载该插件。如果你的模块需要作为可插拔插件被动态发现，使用 `apollo_plugin`；如果只是普通的 Cyber 组件，使用 `apollo_component`。---
title: "Apollo BUILD 文件模式分析"
title: 代码规范指南
description: Apollo 项目代码风格、命名约定、Lint 工具链与最佳实践的完整参考
title: "Apollo 第三方依赖库总览"

## 通信与序列化

用于进程间通信、数据序列化和 RPC 调用的基础库。

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| Fast DDS | 系统安装 | DDS 通信中间件，Apollo Cyber RT 的底层传输层 | `new_local_repository` + `cc_library`（linkopts） |
| Fast RTPS | 系统安装 | RTPS 协议实现，Fast DDS 的前身 | `new_local_repository` + `cc_library`（linkopts） |
| gRPC | 1.30.0 | Google 高性能 RPC 框架 | `http_archive`（WORKSPACE 直接引入） |
| Protobuf | 3.14.0 | Google Protocol Buffers 序列化框架 | `http_archive` |
| nlohmann_json | 3.8.0 | 现代 C++ JSON 库，header-only | `http_archive` |
| yaml-cpp | 0.6.3 | YAML 解析库 | `http_archive` |
| TinyXML2 | 系统安装 | 轻量级 XML 解析库 | `new_local_repository` + `cc_library`（linkopts） |

Fast DDS 和 Fast RTPS 是 Apollo Cyber RT 通信框架的核心传输层实现，支持高效的发布/订阅模式。gRPC 和 Protobuf 用于服务间的 RPC 调用和消息序列化。

## 点云处理与 3D 可视化

用于激光雷达点云数据处理和三维数据可视化。

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| PCL | 系统安装（自动检测） | Point Cloud Library，点云处理核心库 | `pcl_configure` 自动检测 |
| VTK | 系统安装（自动检测） | Visualization Toolkit，3D 数据可视化 | `vtk_configure` 自动检测 |

PCL 和 VTK 均通过自定义的 `*_configure.bzl` 规则在构建时自动检测系统中的安装路径和版本号，动态生成 BUILD 文件。PCL 依赖 VTK、FLANN、libusb 等多个系统库。

## GPU 计算

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| CUDA | 系统安装（自动检测） | NVIDIA GPU 通用计算平台 | `cuda_configure` 自动检测（位于 `third_party/gpus/`） |
| ROCm | 系统安装（自动检测） | AMD GPU 计算平台 | `rocm_configure` 自动检测（位于 `third_party/gpus/`） |
| NPP | 系统安装 | NVIDIA Performance Primitives，GPU 加速图像处理 | `new_local_repository` + `cc_library`（linkopts） |
| OpenGL | 系统安装 | 跨平台图形渲染 API | `new_local_repository` + `cc_library`（linkopts） |
| GLEW | 系统安装 | OpenGL Extension Wrangler，OpenGL 扩展加载 | `new_local_repository` + `cc_library`（linkopts） |

Apollo 同时支持 NVIDIA CUDA 和 AMD ROCm 两种 GPU 计算后端，通过 `common.bzl` 中的 `if_cuda()` / `if_rocm()` 宏实现条件编译。

## UI 与可视化

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| Qt5 | 系统安装 | 跨平台 GUI 框架，用于 Dreamview 等可视化工具 | `new_local_repository` + `cc_library`（linkopts，按模块拆分：qt_core/qt_widgets/qt_gui/qt_opengl） |

## 基础工具库

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| Abseil (absl) | 系统安装 | Google C++ 基础库（字符串、容器、同步等） | `new_local_repository` + `cc_library`（srcs .so） |
| Boost | 系统安装 | C++ 准标准库集合（filesystem/regex/thread 等） | `new_local_repository` + `cc_library`（linkopts） |
| gflags | 系统安装 | Google 命令行参数解析库 | `new_local_repository` + `cc_library`（linkopts） |
| glog | 系统安装 | Google 日志库 | `new_local_repository` + `cc_library`（linkopts） |
| GTest | 1.10.0 | Google 单元测试框架 | `http_archive` |
| Google Benchmark | 1.5.1 | Google 微基准测试框架 | `http_archive` |
| cpplint | 1.5.2 | C++ 代码风格检查工具 | `http_archive` |
| SQLite3 | 系统安装 | 轻量级嵌入式数据库 | `new_local_repository` + `cc_library`（linkopts） |
| UUID | 系统安装 | UUID 生成库 | `new_local_repository` + `cc_library`（linkopts） |
| ncurses5 | 系统安装 | 终端 UI 库 | `new_local_repository` + `cc_library`（linkopts） |
| OpenSSL | 系统安装 | TLS/SSL 加密库 | `cc_library`（srcs + linkopts） |
| PortAudio | 系统安装 | 跨平台音频 I/O 库 | `new_local_repository` + `cc_library`（linkopts） |
| CivetWeb | 1.11 | 轻量级嵌入式 HTTP 服务器 | `http_archive` |

## 构建工具与规则

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| Bazel Skylib | 1.0.3 | Bazel 基础工具库 | `http_archive` |
| rules_cc | 0.0.1 | Bazel C/C++ 构建规则 | `http_archive` |
| rules_proto | - | Bazel Protobuf 构建规则 | `http_archive` |
| rules_python | 0.1.0 | Bazel Python 构建规则 | `http_archive` |
| rules_foreign_cc | 0.8.0 | Bazel 外部 C/C++ 构建系统集成规则 | `http_archive` |
| zlib | 1.2.11 | 通用压缩库（gRPC 依赖） | `http_archive` |
| Python | 系统安装 | Python 运行时（通过 python_configure 检测） | `python_configure` 自动检测 |

title: "Apollo WORKSPACE 外部依赖分析"

## 一、WORKSPACE 文件直接声明的依赖

以下依赖直接在 `WORKSPACE` 文件中通过 `http_archive` 规则声明：

| 名称 | 版本 | 来源 | 规则类型 | 用途说明 |
|------|------|------|----------|----------|
| `rules_foreign_cc` | 0.8.0 | [GitHub](https://github.com/bazelbuild/rules_foreign_cc) | `http_archive` | 支持在 Bazel 中构建 CMake/Make 等外部项目 |
| `rules_cc` | 0.0.1 | [GitHub](https://github.com/bazelbuild/rules_cc) | `http_archive` | Bazel C/C++ 构建规则（已打补丁） |
| `bazel_skylib` | 1.0.3 | [GitHub](https://github.com/bazelbuild/bazel-skylib) | `http_archive` | Bazel 通用工具库，提供常用 Starlark 函数 |
| `rules_proto` | 97d8af4 (commit) | [GitHub](https://github.com/bazelbuild/rules_proto) | `http_archive` | Protobuf 构建规则 |
| `rules_python` | 0.1.0 | [GitHub](https://github.com/bazelbuild/rules_python) | `http_archive` | Python 构建规则 |
| `com_github_grpc_grpc` | 1.30.0 (Apollo 定制) | [百度 CDN](https://apollo-system.cdn.bcebos.com/) | `http_archive` | gRPC 远程过程调用框架（Apollo 定制版 v1.30.0-apollo，已打补丁） |
| `zlib` | 1.2.11 | [GitHub](https://github.com/madler/zlib) | `http_archive` | 通用数据压缩库，gRPC/Protobuf 的传递依赖 |

> 最低 Bazel 版本要求：**3.7.0**（通过 `bazel_skylib` 的 `versions.check` 校验）

## 三、依赖集成方式说明

### 3.1 加载流程

```
WORKSPACE
  ├── 直接声明 http_archive（rules_foreign_cc, rules_cc, bazel_skylib, ...）
  ├── load("//tools:workspace.bzl", "apollo_repositories")
  │     ├── *_configure()  →  自动探测 CUDA / TensorRT / Python / VTK / PCL
  │     └── initialize_third_party()
  │           ├── http_archive(...)   →  从 CDN/GitHub 下载
  │           └── new_local_repository(...)  →  引用容器内预装库
  └── grpc_deps() / grpc_extra_deps()  →  gRPC 传递依赖
```

### 3.2 镜像策略

大部分 `http_archive` 依赖配置了双 URL：

1. 百度 CDN 镜像（`apollo-system.cdn.bcebos.com` 或 `apollo-system.bj.bcebos.com`）— 优先使用
2. GitHub 原始地址 — 作为备用

Bazel 会按 `urls` 列表顺序尝试下载，国内环境优先命中 CDN，海外环境回退到 GitHub。

### 3.3 本地依赖的容器化管理

使用 `new_local_repository` 的依赖依赖于 Apollo Docker 开发容器中的预装环境。主要安装路径包括：

| 路径 | 说明 |
|------|------|
| `/opt/apollo/sysroot/include` | Apollo 定制 sysroot，包含 Boost、OpenCV、FFmpeg 等 |
| `/opt/apollo/absl/` | Abseil 库 |
| `/opt/apollo/pkgs/` | Apollo 平台特定包 |
| `/usr/local/cuda` | NVIDIA CUDA 工具链 |
| `/usr/local/fast-rtps/include` | Fast-RTPS DDS 中间件 |
| `/usr/local/qt5/include` | Qt5 框架 |
| `/usr/local/libtorch_*/include` | PyTorch C++ 库 |
| `/usr/include` | 系统标准头文件路径 |

### 3.4 在 BUILD 文件中引用依赖

在各模块的 `BUILD` 文件中，通过 `@name` 语法引用 WORKSPACE 中声明的外部依赖：

```python
cc_library(
    name = "my_module",
    srcs = ["my_module.cc"],
    deps = [
        "@com_google_protobuf//:protobuf",
        "@eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
)
```

### 3.5 补丁机制

部分依赖应用了 Apollo 定制补丁以适配项目需求：

| 依赖 | 补丁文件 |
|------|----------|
| `rules_cc` | `//tools/package:rules_cc.patch` |
| `com_github_grpc_grpc` | `//third_party/absl:grpc.patch` |
| `com_google_protobuf` | `//third_party/protobuf:protobuf.patch` |
