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

title: 脚本与工具链
title: "Init Scripts 开发环境脚本解析"

# 数据采集模块 (collection/)

## 概述

`collection/` 目录是 Apollo 的模块元数据与包聚合层，充当 Apollo Cyber 模块的注册中心和打包管理层。它本身不包含具体的功能实现代码，而是通过 `cyberfile.xml` 清单文件统一管理各模块的包信息、版本声明和依赖关系。

目前共收录 **26 个模块**，覆盖感知、定位、规划、控制、基础设施及工具等完整自动驾驶技术栈。

### 模块分类

**感知与传感器**

| 模块 | 说明 |
|------|------|
| `audio` | 紧急车辆警笛检测模块 |
| `drivers` | 传感器驱动模块（激光雷达、摄像头、GPS 等） |
| `canbus` | CAN 总线通信模块 |
| `perception` | 感知模块（目标检测、跟踪、分类） |
| `third-party-perception` | 第三方感知系统集成 |

**定位与地图**

| 模块 | 说明 |
|------|------|
| `localization` | 车辆定位模块 |
| `map` | 高精地图模块 |
| `calibration` | 传感器标定模块 |
| `transform` | 坐标系变换模块 |

**规划与控制**

| 模块 | 说明 |
|------|------|
| `planning` | 路径规划模块 |
| `prediction` | 障碍物行为预测模块 |
| `control` | 车辆控制模块 |
| `task-manager` | 任务管理模块 |
| `external-command` | 外部命令接口模块 |

**基础设施**

| 模块 | 说明 |
|------|------|
| `cyber` | CyberRT 通信框架 |
| `common` | 公共基础库 |
| `common-msgs` | 公共消息类型定义 |
| `bridge` | 跨系统桥接模块 |
| `v2x` | V2X 车路协同通信模块 |

**工具与可视化**

| 模块 | 说明 |
|------|------|
| `dreamview` | Web 可视化调试平台 |
| `monitor` | 系统运行状态监控 |
| `guardian` | 安全守护模块 |
| `storytelling` | 场景叙述与回放模块 |
| `smart-recorder` | 智能数据记录器 |
| `tools` | 开发与调试工具集 |
| `contrib` | 社区贡献模块 |

## 配置选项

### cyberfile.xml 字段说明

每个模块的 `cyberfile.xml` 定义了该模块的包元数据：

```xml
<package format="2">
  <name>module-perception</name>        <!-- 包名，约定使用 module-* 前缀 -->
  <version>local</version>              <!-- 版本号，本地开发时使用 "local" -->
  <description>感知模块</description>
  <type>module</type>                   <!-- 类型：module 或 collection -->
  <src_path url="https://github.com/ApolloAuto/apollo">
    //collection/perception
  </src_path>

  <depend repo_name="apollo-perception-camera" type="binary">...</depend>
</package>
```

| 字段 | 说明 |
|------|------|
| `package format` | 根元素的 `format` 属性，当前为 `"2"` |
| `name` | 包的唯一标识名称，约定使用 `module-*` 前缀 |
| `version` | 版本号，本地开发时通常为 `"local"` |
| `type` | `module`（单模块）或 `collection`（聚合包） |
| `src_path` | 对应的源码路径（Bazel 标签格式），包含 `url` 属性指向仓库地址 |
| `depend` | 依赖的其他 Apollo 包，包含 `repo_name` 和 `type` 属性 |

### BUILD 文件配置

`BUILD` 文件使用 Bazel 语法声明构建目标，通常引用源码目录中的实际构建规则：

```python
load("//tools:apollo_package.bzl", "apollo_package")
package(default_visibility = ["//visibility:public"])
apollo_package()
```

**Q: 如何添加一个新模块到 collection？**

在 `collection/` 下新建模块目录，并创建以下三个文件：

1. `cyberfile.xml` — 填写包名、版本、依赖
2. `BUILD` — 声明 Bazel 构建目标或别名
3. `README.md` — 简要说明模块用途

**Q: `canbus` 模块采集哪些数据？**

`canbus` 模块通过 CAN 总线与车辆底盘通信，采集并发布车速、转向角、油门/制动状态、档位等底盘信息，同时接收控制模块下发的控制指令。

**Q: 构建时提示找不到依赖包怎么办？**

检查对应模块 `cyberfile.xml` 中的 `<depend>` 声明，确认依赖包已在本地编译或通过包管理器安装。可运行以下命令检查依赖状态：

```bash
buildtool info <module-name>
```
