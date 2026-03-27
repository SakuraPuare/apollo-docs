# Apollo WORKSPACE 外部依赖分析

## 概述

Apollo 采用 [Bazel](https://bazel.build/) 作为构建系统，所有外部依赖通过 `WORKSPACE` 文件及 `third_party/` 目录下的 `.bzl` 文件进行声明和管理。依赖的引入方式主要有三种：

- **`http_archive`**：从远程 URL 下载压缩包，适用于有明确版本发布的开源库
- **`new_local_repository`**：引用宿主机（Docker 容器）上已预装的本地库
- **`*_configure`**：通过 repository rule 自动探测系统环境并生成构建配置（如 CUDA、TensorRT）

Apollo 同时维护了百度 CDN 镜像（`apollo-system.cdn.bcebos.com`）和 GitHub 原始地址作为双源下载，保证国内外网络环境下均可正常拉取依赖。

`WORKSPACE` 和 `WORKSPACE.source` 两个文件内容完全一致，`WORKSPACE.source` 作为源文件模板存在。

---

## 一、WORKSPACE 文件直接声明的依赖

以下依赖直接在 `WORKSPACE` 文件中通过 `http_archive` 规则声明：

| 名称 | 版本 | 来源 | 规则类型 | 用途说明 |
|------|------|------|----------|----------|
| `rules_foreign_cc` | 0.8.0 | [GitHub](https://github.com/bazelbuild/rules_foreign_cc) | `http_archive` | 支持在 Bazel 中构建 CMake/Make 等外部项目 |
| `rules_cc` | 0.0.1 | [GitHub](https://github.com/bazelbuild/rules_cc) | `http_archive` | Bazel C/C++ 构建规则（已打补丁） |
| `bazel_skylib` | 1.0.3 | [GitHub](https://github.com/bazelbuild/bazel-skylib) | `http_archive` | Bazel 通用工具库，提供常用 Starlark 函数 |
| `rules_proto` | 97d8af4 (commit) | [GitHub](https://github.com/bazelbuild/rules_proto) | `http_archive` | Protobuf 构建规则 |
| `rules_python` | 0.1.0 | [GitHub](https://github.com/bazelbuild/rules_python) | `http_archive` | Python 构建规则 |
| `com_github_grpc_grpc` | 1.30.0 (Apollo 定制) | [GitHub](https://github.com/grpc/grpc) | `http_archive` | gRPC 远程过程调用框架（已打补丁） |
| `zlib` | 1.2.11 | [GitHub](https://github.com/madler/zlib) | `http_archive` | 通用数据压缩库，gRPC/Protobuf 的传递依赖 |

> 最低 Bazel 版本要求：**3.7.0**（通过 `bazel_skylib` 的 `versions.check` 校验）

---

## 二、third_party 目录声明的依赖（通过 `apollo_repositories()` 加载）

`WORKSPACE` 通过 `load("//tools:workspace.bzl", "apollo_repositories")` 加载 `tools/workspace.bzl`，该文件汇总了 `third_party/` 下所有子模块的依赖声明。

### 2.1 远程下载依赖（http_archive）

| 名称 | 版本 | 来源 | 用途说明 |
|------|------|------|----------|
| `com_google_protobuf` | 3.14.0 | [GitHub](https://github.com/protocolbuffers/protobuf) | Protocol Buffers 序列化框架（已打补丁） |
| `com_google_googletest` | 1.10.0 | [GitHub](https://github.com/google/googletest) | Google Test / Google Mock 单元测试框架 |
| `com_google_benchmark` | 1.5.1 | [GitHub](https://github.com/google/benchmark) | Google Benchmark 性能基准测试库 |
| `eigen` | 3.3.7 | [GitHub](https://github.com/eigenteam/eigen-git-mirror) | 线性代数运算库（矩阵、向量） |
| `com_github_nlohmann_json` | 3.8.0 | [GitHub](https://github.com/nlohmann/json) | 现代 C++ JSON 解析库 |
| `com_github_jbeder_yaml_cpp` | 0.6.3 | [GitHub](https://github.com/jbeder/yaml-cpp) | YAML 解析库 |
| `cpplint` | 1.5.2 | [GitHub](https://github.com/cpplint/cpplint) | C++ 代码风格检查工具 |
| `civetweb` | 1.11 | [GitHub](https://github.com/civetweb/civetweb) | 轻量级嵌入式 HTTP 服务器 |
| `ad_rss_lib` | 1.1.0 | [GitHub](https://github.com/intel/ad-rss-lib) | Intel 自动驾驶 RSS（责任敏感安全）库 |
| `paddleinference-x86_64` | — | [百度 CDN](https://apollo-pkg-beta.cdn.bcebos.com/) | PaddlePaddle 推理引擎（x86_64） |
| `paddleinference-aarch64` | 2.0.0 | [百度 CDN](https://apollo-pkg-beta.bj.bcebos.com/) | PaddlePaddle 推理引擎（aarch64） |
| `centerpoint_infer_op-x86_64` | — | [百度 CDN](https://apollo-pkg-beta.cdn.bcebos.com/) | CenterPoint 3D 目标检测推理算子（x86_64） |
| `centerpoint_infer_op-aarch64` | 2.0.0 | [百度 CDN](https://apollo-pkg-beta.bj.bcebos.com/) | CenterPoint 3D 目标检测推理算子（aarch64） |
| `caddn_infer_op-x86_64` | — | [百度 CDN](https://apollo-system.bj.bcebos.com/) | CADDN 深度估计推理算子（x86_64） |
| `caddn_infer_op-aarch64` | 1.0.0 | [百度 CDN](https://apollo-pkg-beta.bj.bcebos.com/) | CADDN 深度估计推理算子（aarch64） |
| `localization_msf` | 1.0.0 | [百度 CDN](https://apollo-pkg-beta.bj.bcebos.com/) | 多传感器融合定位模块 |

### 2.2 本地预装依赖（new_local_repository）

以下依赖要求在构建环境（通常为 Apollo Docker 容器）中预先安装：

| 名称 | 本地路径 | 用途说明 |
|------|----------|----------|
| `com_google_absl` | `/opt/apollo/absl/` | Google Abseil C++ 通用库 |
| `adolc` | `/usr/include` | 自动微分库（ADOL-C） |
| `adv_plat` | `/opt/apollo/pkgs/adv_plat/include` | Apollo 高级平台抽象层 |
| `atlas` | `/usr/include` | ATLAS 线性代数库 |
| `boost` | `/opt/apollo/sysroot/include` | Boost C++ 库集合 |
| `fastcdr` | `/usr/local/fast-rtps/include` | Fast-CDR 序列化库（Fast-RTPS 组件） |
| `fastrtps` | `/usr/local/fast-rtps/include` | eProsima Fast-RTPS DDS 通信中间件 |
| `ffmpeg` | `/opt/apollo/sysroot/include` | 音视频编解码框架 |
| `fftw3` | `/usr/include` | 快速傅里叶变换库 |
| `com_github_gflags_gflags` | `/usr/local/include` | Google 命令行参数解析库 |
| `com_github_google_glog` | `/usr/local/include` | Google 日志库 |
| `ipopt` | `/usr/include` | 大规模非线性优化求解器 |
| `libtorch_cpu` | `/usr/local/libtorch_cpu/include` | PyTorch C++ 推理库（CPU 版） |
| `libtorch_gpu` | `/usr/local/libtorch_gpu/include` | PyTorch C++ 推理库（GPU 版） |
| `ncurses5` | `/usr/include` | 终端 UI 库 |
| `npp` | `/usr/local/cuda` | NVIDIA Performance Primitives（CUDA 图像处理） |
| `nvjpeg` | `/usr/src` | NVIDIA JPEG 编解码库 |
| `opencv` | `/opt/apollo/sysroot/include/opencv4` | 计算机视觉库 |
| `opengl` | `/usr/include` | OpenGL 图形渲染接口 |
| `openh264` | `/opt/apollo/sysroot/include` | H.264 视频编解码库 |
| `osqp` | `/opt/apollo/sysroot/include` | 二次规划求解器 |
| `portaudio` | `/usr/include` | 跨平台音频 I/O 库 |
| `proj` | `/opt/apollo/sysroot/include` | 地理坐标投影转换库 |
| `qt` | `/usr/local/qt5/include` | Qt5 GUI 框架（用于 Dreamview 等可视化） |
| `sqlite3` | `/usr/include` | 轻量级嵌入式数据库 |
| `tinyxml2` | `/usr/include` | 轻量级 XML 解析库 |
| `uuid` | `/usr/include` | UUID 生成库 |

### 2.3 环境自动配置依赖（repository rule）

以下依赖通过 `*_configure` 规则在构建时自动探测系统环境：

| 配置名称 | 规则 | 用途说明 |
|----------|------|----------|
| `local_config_cuda` | `cuda_configure` | NVIDIA CUDA 工具链自动配置 |
| `local_config_rocm` | `rocm_configure` | AMD ROCm 工具链自动配置 |
| `local_config_tensorrt` | `tensorrt_configure` | NVIDIA TensorRT 推理加速引擎配置 |
| `local_config_python` | `python_configure` | Python 解释器及头文件路径配置 |
| `local_config_vtk` | `vtk_configure` | VTK 3D 可视化工具库配置 |
| `local_config_pcl` | `pcl_configure` | PCL 点云处理库配置 |

---

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
