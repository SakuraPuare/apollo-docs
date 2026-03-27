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
| `rules_cc` | 0.0.1 | C/C++ 编译规则（含 Apollo 自定义补丁） |
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

通过 `initialize_third_party()` 函数统一加载约 40 个第三方依赖，涵盖以下类别：

- **基础库**：abseil-cpp、Boost、gflags、glog、Protobuf、yaml-cpp
- **数学/优化**：Eigen3、IPOPT、OSQP、ADOLC、ATLAS
- **感知/推理**：OpenCV、PCL、LibTorch、PaddleInference、TensorRT、CenterPoint、CADDN
- **通信**：Fast-RTPS/Fast-DDS、gRPC、CivetWeb
- **可视化**：Qt5、VTK、OpenGL、GLEW
- **多媒体**：FFmpeg、OpenH264、PortAudio、FFTW3、nvJPEG
- **工具**：cpplint、Google Test、Google Benchmark、SQLite3、tinyxml2、nlohmann_json

### 2.5 镜像加速

所有外部依赖均配置了双 URL 源：

```starlark
urls = [
    "https://apollo-system.cdn.bcebos.com/archive/6.0/...",  # 百度 CDN 镜像（国内加速）
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

- **GPU 模式**额外包含：`paddleinference`、`caddn_infer_op`、`tensorrt`、`npp`、`nvjpeg` 等 GPU 专用库
- **CPU 模式**排除 GPU 专用推理库，但仍包含 `centerpoint_infer_op` 等可在 CPU 上运行的组件

安装目标覆盖约 40 个第三方库以及 `//scripts` 和 `//tools` 两个内部包。

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
