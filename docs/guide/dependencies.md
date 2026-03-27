# Apollo 第三方依赖库总览

## 概述

Apollo 自动驾驶平台依赖大量第三方库，涵盖通信中间件、深度学习推理、点云处理、计算机视觉、数学优化、GPU 计算、UI 可视化等多个领域。所有第三方依赖统一放置在 `third_party/` 目录下，通过 Bazel 构建系统进行管理。

依赖的集成方式主要分为三类：

- **`http_archive`**：从远程下载源码包，在构建时编译（如 Eigen、Protobuf、gRPC）
- **`new_local_repository`**：引用系统或 Apollo sysroot 中预装的库，通过自定义 BUILD 文件暴露给 Bazel（如 OpenCV、Boost、CUDA）
- **`cc_library` / `cc_binary`（源码编译）**：直接在 `third_party/` 中包含源码并编译（如 RTKLIB、TF2）

Apollo 使用 `cyberfile.xml` 作为包管理元数据，其中 `type` 字段标识了包的类型：
- `third-binary`：预编译二进制包
- `third-wrapper`：对系统库的 Bazel 封装
- `module`：包含源码的模块

---

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

---

## 深度学习与推理

支撑 Apollo 感知模块中各类神经网络模型的训练和推理。

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| LibTorch (CPU) | 系统安装 | PyTorch C++ 推理库（CPU 版本） | `new_local_repository` + `cc_library`（srcs/linkopts） |
| LibTorch (GPU) | 系统安装 | PyTorch C++ 推理库（GPU 版本，支持 CUDA/ROCm） | `new_local_repository` + `cc_library`（linkopts + select） |
| PaddleInference | 2.0.0 (aarch64) | 百度飞桨推理引擎 | `http_archive`（区分 x86_64/aarch64） |
| TensorRT | 系统安装 | NVIDIA 深度学习推理优化器 | `tensorrt_configure` 自动检测 |
| CADDN Infer Op | 本地 | CADDN 模型自定义推理算子 | `install`（预编译 .so） |
| CenterPoint Infer Op | 本地 | CenterPoint 模型自定义推理算子 | `install`（预编译 .so） |

LibTorch 通过 `select()` 机制在 CPU 和 GPU 版本之间切换。TensorRT 通过 `tensorrt_configure.bzl` 在构建时自动检测系统安装路径和版本。PaddleInference 针对不同架构提供独立的预编译包。

---

## 点云处理与 3D 可视化

用于激光雷达点云数据处理和三维数据可视化。

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| PCL | 系统安装（自动检测） | Point Cloud Library，点云处理核心库 | `pcl_configure` 自动检测 |
| VTK | 系统安装（自动检测） | Visualization Toolkit，3D 数据可视化 | `vtk_configure` 自动检测 |

PCL 和 VTK 均通过自定义的 `*_configure.bzl` 规则在构建时自动检测系统中的安装路径和版本号，动态生成 BUILD 文件。PCL 依赖 VTK、FLANN、libusb 等多个系统库。

---

## 计算机视觉与图像处理

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| OpenCV | 系统安装 | 计算机视觉核心库（core/highgui/imgproc/imgcodecs/calib3d） | `new_local_repository` + `cc_library`（linkopts，按模块拆分） |
| FFmpeg | 系统安装 | 音视频编解码框架（avcodec/avformat/swscale/avutil） | `new_local_repository` + `cc_library`（linkopts，按模块拆分） |
| OpenH264 | 系统安装 | H.264 视频编解码库 | `new_local_repository` + `cc_library`（linkopts） |
| nvJPEG | 系统安装 | NVIDIA GPU 加速 JPEG 编解码 | `new_local_repository` + `cc_library` |

OpenCV 在 BUILD 文件中按功能模块（core、highgui、imgproc、imgcodecs、calib3d）拆分为独立的 `cc_library` target，便于按需依赖。

---

## GPU 计算

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| CUDA | 系统安装（自动检测） | NVIDIA GPU 通用计算平台 | `cuda_configure` 自动检测 |
| ROCm | 系统安装（自动检测） | AMD GPU 计算平台 | `rocm_configure` 自动检测 |
| NPP | 系统安装 | NVIDIA Performance Primitives，GPU 加速图像处理 | `new_local_repository` |
| OpenGL | 系统安装 | 跨平台图形渲染 API | `new_local_repository` + `cc_library`（linkopts） |
| GLEW | 系统安装 | OpenGL Extension Wrangler，OpenGL 扩展加载 | `new_local_repository` + `cc_library`（linkopts） |

Apollo 同时支持 NVIDIA CUDA 和 AMD ROCm 两种 GPU 计算后端，通过 `common.bzl` 中的 `if_cuda()` / `if_rocm()` 宏实现条件编译。

---

## 数学、线性代数与优化

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| Eigen | 3.3.7 | C++ 模板线性代数库 | `http_archive` |
| ATLAS | 系统安装 | 自动调优线性代数库（含 BLAS/CBLAS/LAPACK） | `new_local_repository` + `cc_library`（linkopts） |
| OSQP | 系统安装 | 二次规划求解器，用于规划控制 | `new_local_repository` + `cc_library`（linkopts） |
| Ipopt | 系统安装 | 大规模非线性优化求解器 | `new_local_repository` + `cc_library`（linkopts） |
| ADOLC | 系统安装 | 自动微分库 | `new_local_repository` + `cc_library`（linkopts） |
| FFTW3 | 系统安装 | 快速傅里叶变换库 | `new_local_repository` + `cc_library`（linkopts） |

Eigen 是 Apollo 中使用最广泛的数学库，几乎所有涉及矩阵运算的模块都依赖它。OSQP 和 Ipopt 主要用于规划和控制模块中的优化问题求解。

---

## UI 与可视化

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| Qt5 | 系统安装 | 跨平台 GUI 框架，用于 Dreamview 等可视化工具 | `new_local_repository` |

---

## 坐标变换与定位

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| TF2 | 源码编译 | ROS 风格坐标变换库（Apollo 定制版） | `cc_binary` + `cc_library`（源码编译为 libtf2.so） |
| PROJ | 系统安装 | 地理坐标投影变换库 | `new_local_repository` + `cc_library`（linkopts） |
| RTKLIB | 源码编译 | GNSS 精密定位库（RTK 算法） | `apollo_cc_library`（源码编译） |
| Localization MSF | 1.0.0 | Apollo 多传感器融合定位库 | `http_archive`（预编译包） |

TF2 是从 ROS 移植的坐标变换库，Apollo 在 `third_party/tf2/` 中维护了定制版本并从源码编译。RTKLIB 同样以源码形式集成，用于 GNSS RTK 定位算法。

---

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

---

## 硬件驱动与平台适配

| 库名 | 版本 | 用途说明 | 集成方式 |
|------|------|---------|---------|
| CAN Card Library | 本地 | CAN 总线通信卡驱动（Hermes CAN/ESD CAN） | `cc_library`（srcs .so） |
| Camera Library | 本地 | 相机驱动库（SmarterEye 等） | `cc_library`（srcs .so） |
| AD RSS Lib | 1.1.0 | Intel 责任敏感安全（RSS）库 | `http_archive` |
| ADV Plat | 系统安装 | 高级平台触发/CAN 接口库 | `new_local_repository` + `cc_library`（linkopts） |
| SSE2NEON | 本地 | SSE 指令到 NEON 指令的转换头文件（ARM 适配） | `cc_library`（header-only） |
| Intrinsics Translation | 本地 | 跨平台 SIMD 指令翻译层（x86/ARM） | `cc_library`（header-only，条件编译） |

CAN Card Library 和 Camera Library 以预编译动态库形式集成，支持特定硬件设备。SSE2NEON 和 Intrinsics Translation 用于实现 x86 到 ARM 架构的 SIMD 指令兼容。

---

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

---

## 集成方式统计

| 集成方式 | 数量 | 说明 |
|---------|------|------|
| `new_local_repository` | ~25 | 引用系统预装库，最常见的方式 |
| `http_archive` | ~15 | 从远程下载源码包构建 |
| `*_configure` 自动检测 | 5 | CUDA/ROCm/TensorRT/VTK/PCL/Python |
| 源码编译 | 3 | TF2、RTKLIB、Intrinsics Translation |
| 预编译 .so 直接引入 | 4 | CAN Card/Camera/CADDN/CenterPoint |
