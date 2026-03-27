---
title: Docker 开发环境
---

# Docker 开发环境

## 概述

Apollo 的开发环境完全基于 Docker 容器化方案，通过一套脚本驱动的构建体系管理镜像生命周期。整个体系不依赖 docker-compose，而是由独立的 Dockerfile 加上 Shell 编排脚本组成。

### 镜像层级

Apollo 定义了四个官方镜像阶段，按依赖关系逐层叠加：

| 阶段 | 用途 |
|------|------|
| `base` | NVIDIA CUDA 基础层，安装 cuDNN 和 TensorRT |
| `cyber` | CyberRT 开发环境，包含 Bazel、protobuf、Fast-RTPS 等 |
| `dev` | 完整开发环境，在 cyber 基础上叠加感知、ML 框架、可视化工具等依赖 |
| `runtime` | 精简运行时镜像，以运行依赖为主 |

此外，`standalone` 为独立构建的特殊用途镜像，将编译产物和模型数据一并打包，拥有单独的构建脚本。

### Dockerfile 命名规范

```
[prefix_]<stage>.<arch>.<gpu>.dockerfile
```

- `arch`：`x86_64` 或 `aarch64`
- `gpu`：`nvidia` 或 `amd`

例如：`dev.x86_64.nvidia.dockerfile`、`cyber.x86_64.amd.dockerfile`

---

## 使用方法

所有脚本位于 `docker/scripts/` 目录下。

### 启动开发容器

```bash
bash docker/scripts/dev_start.sh
```

该脚本会：

- 创建并启动一个新的 dev 容器
- 挂载 Apollo 源码、地图、数据目录
- 自动检测 GPU 类型并配置 GPU 透传
- 配置 X11 显示转发
- 挂载 `/dev` 设备以支持硬件访问

常用选项：

| 选项 | 说明 |
|------|------|
| `-f, --fast` | 快速模式，跳过加载全量地图卷 |
| `-g, --geo <us|cn>` | 指定地理区域镜像源 |
| `-l, --local` | 使用本地镜像 |
| `-d, --dist <stable|testing>` | 指定发行版类型 |
| `-c, --cross-platform` | 交叉编译模式 |
| `--co-dev <path>` | 协同开发模式 |
| `--shm-size <bytes>` | 设置共享内存大小 |
| `-t, --tag <tag>` | 指定镜像 tag |
| `-y` | 非交互式确认 Apollo 许可协议 |

### 进入已运行的容器

```bash
bash docker/scripts/dev_into.sh
```

附加到已有的 dev 容器，进入交互式 Shell。

### 使用轻量 CyberRT 容器

如果只需要 CyberRT 环境而不需要完整开发依赖：

```bash
# 启动
bash docker/scripts/cyber_start.sh

# 进入
bash docker/scripts/cyber_into.sh
```

### 使用运行时容器

用于部署测试，不包含构建工具：

```bash
# 启动
bash docker/scripts/runtime_start.sh

# 进入
bash docker/scripts/runtime_into.sh

# 进入 standalone 容器
bash docker/scripts/runtime_into_standalone.sh
```

### 远程调试（GDB Server）

```bash
bash docker/scripts/dev_start_gdb_server.sh
```

在已运行的 dev 容器中启动 GDB Server，支持远程调试。

### 构建镜像

镜像构建由 `docker/build/build_docker.sh` 统一编排，负责注入构建参数、处理镜像 tag 和推送操作：

```bash
bash docker/build/build_docker.sh -f <dockerfile> -m build -g cn
```

### 宿主机初始化

首次使用前，需在宿主机上完成系统初始化（配置 core dump 格式、ntpdate 定时同步、udev 规则及 uvcvideo 设置）：

```bash
bash docker/setup_host/setup_host.sh
```

---

## 配置选项

### base 阶段构建参数

| 参数 | 说明 |
|------|------|
| `BASE_IMAGE` | 基础 CUDA devel 镜像 |
| `CUDA_LITE` | CUDA 精简版本号 |
| `CUDNN_VERSION` | cuDNN 版本 |
| `TENSORRT_VERSION` | TensorRT 版本 |

### dev 镜像包含的主要组件

- 感知库：OpenCV、PCL、激光雷达驱动
- 可视化工具（Dreamview 依赖）
- 机器学习框架

这些依赖通过 `docker/build/installers/` 下的安装脚本分模块管理，cyber 阶段和 dev 阶段均使用此机制。

### GPU 类型自动检测

`docker/scripts/docker_base.sh` 提供所有脚本共用的基础函数，包括：

- 自动检测宿主机 GPU 类型（NVIDIA / AMD）
- 设置镜像名称和 tag
- 管理容器命名
- 处理卷挂载逻辑

无需手动指定 GPU 类型，脚本会根据检测结果选择对应的 Dockerfile 和运行时参数。

---

## 常见问题

### 容器启动后无法使用 GPU

确认宿主机已正确安装 GPU 驱动并完成系统初始化：

```bash
bash docker/setup_host/setup_host.sh
```

对于 NVIDIA GPU，验证 `nvidia-smi` 在宿主机可正常运行；对于 AMD GPU，确认 ROCm 驱动已安装。

### X11 图形界面无法显示

`dev_start.sh` 会自动配置 X11 转发，但需要宿主机允许本地连接：

```bash
xhost +local:root
```

### 如何选择镜像阶段

- 日常开发：使用 `dev` 镜像，功能最完整
- 仅调试 CyberRT 通信：使用 `cyber` 镜像，体积更小，启动更快
- 部署验证：使用 `runtime` 或 `standalone` 镜像，贴近生产环境

### 多人共用宿主机时容器命名冲突

容器名称默认基于用户名自动生成，通常不会冲突。如遇特殊情况，可通过设置 `USER` 环境变量区分不同用户。

### 如何更新镜像

拉取最新镜像后重新启动容器：

```bash
docker pull <registry>/apollo_dev:<tag>
bash docker/scripts/dev_start.sh -t <tag>
```
