---
title: 数据采集模块 (collection/)
---

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

---

## 使用方法

### 目录结构

每个模块目录的结构统一如下：

```
collection/
└── <module-name>/
    ├── BUILD          # Bazel 构建配置
    ├── cyberfile.xml  # 包清单（依赖声明）
    └── README.md      # 模块说明文档
```

### 查看模块依赖

通过 `cyberfile.xml` 可以查看某个模块的完整依赖树：

```bash
cat collection/<module-name>/cyberfile.xml
```

### 使用 Bazel 构建指定模块

```bash
# 使用 buildtool 构建（推荐）
buildtool build --packages collection/<module-name>

# 使用 Bazel 直接构建
bazel build //collection/<module-name>/...

# 构建所有模块
bazel build //collection/...
```

### 数据记录（smart-recorder）

`smart-recorder` 模块依赖 `apollo-data`，用于按策略智能录制传感器和系统数据：

```bash
# 启动智能录制器
python3 /apollo/scripts/record_message.py
```

### 传感器数据采集（drivers）

`drivers` 模块负责对接各类传感器硬件，采集原始数据并发布到 CyberRT 话题：

```bash
# 启动激光雷达驱动（以 Velodyne 为例）
cyber_launch start modules/drivers/lidar/velodyne/launch/velodyne128.launch
```

---

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

---

## 常见问题

**Q: `collection/` 目录和 `modules/` 目录有什么区别？**

`collection/` 是元数据层，只包含包清单和构建入口，不含实现代码。实际的功能实现位于 `modules/` 对应的子目录中。`collection/` 的作用是统一管理模块的版本和依赖关系，便于包管理工具解析。

---

**Q: 如何添加一个新模块到 collection？**

在 `collection/` 下新建模块目录，并创建以下三个文件：

1. `cyberfile.xml` — 填写包名、版本、依赖
2. `BUILD` — 声明 Bazel 构建目标或别名
3. `README.md` — 简要说明模块用途

---

**Q: `smart-recorder` 和普通的 `cyber_recorder` 有什么区别？**

`cyber_recorder` 是全量录制工具，会记录所有话题的数据。`smart-recorder` 基于触发策略（如碰撞事件、异常检测）进行选择性录制，依赖 `apollo-data` 包提供的数据管理能力，更适合在车端长时间运行时节省存储空间。

---

**Q: `canbus` 模块采集哪些数据？**

`canbus` 模块通过 CAN 总线与车辆底盘通信，采集并发布车速、转向角、油门/制动状态、档位等底盘信息，同时接收控制模块下发的控制指令。

---

**Q: `v2x` 模块的数据来源是什么？**

`v2x`（Vehicle-to-Everything）模块通过路侧单元（RSU）或云端平台接收交通信号灯状态、道路事件、其他车辆位置等协同信息，并将其融合到 Apollo 的感知和规划流程中。

---

**Q: 构建时提示找不到依赖包怎么办？**

检查对应模块 `cyberfile.xml` 中的 `<depend>` 声明，确认依赖包已在本地编译或通过包管理器安装。可运行以下命令检查依赖状态：

```bash
buildtool info <module-name>
```
