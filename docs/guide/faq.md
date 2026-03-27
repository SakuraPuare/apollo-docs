---
title: 常见问题 FAQ
---

# 常见问题 FAQ

## Q1: 什么是 Apollo？它能做什么？

Apollo（阿波罗）是百度开源的自动驾驶平台，旨在为开发者提供一套高性能、灵活的自动驾驶软件框架，加速自动驾驶车辆的开发、测试和部署。

Apollo 的核心能力包括：

- **感知（Perception）**：融合 LiDAR、Camera、Radar 等多传感器数据，实现障碍物检测、车道线识别、交通信号灯识别等
- **预测（Prediction）**：预测周围交通参与者的未来轨迹和行为意图
- **规划（Planning）**：基于场景化的路径和速度规划，支持变道、掉头、无保护转弯等复杂场景
- **控制（Control）**：通过线控系统精准控制车辆的转向、油门和制动
- **高精地图（HD Map）**：提供厘米级精度的高精地图服务
- **定位（Localization）**：融合 GNSS/IMU 和 LiDAR 点云匹配实现高精度定位
- **仿真（Simulation）**：通过 Dreamview 和 Dreamview Plus 提供可视化和仿真调试工具

Apollo 从 1.0 版本逐步发展到 11.0 版本，已覆盖从封闭园区到复杂城市道路的多种自动驾驶场景。

## Q2: Apollo 支持哪些硬件平台？系统要求是什么？

**最低硬件要求：**

- CPU：8 核处理器
- 内存：16GB RAM 以上
- GPU：NVIDIA Turing 架构及以上（如 RTX 20/30/40 系列），或 AMD GFX9/RDNA/CDNA 架构
- 磁盘：建议预留充足空间（Docker 镜像 + 编译缓存较大）

**支持的操作系统：**

- Ubuntu 18.04
- Ubuntu 20.04
- Ubuntu 22.04

**GPU 驱动要求：**

- NVIDIA 驱动版本 >= 520.61.05（CUDA 11.8）
- 或 AMD ROCm v5.1 及以上

**CPU 架构支持：**

- x86_64（主要开发平台）
- aarch64 / ARM64（Apollo 9.0+ 支持在 NVIDIA Orin 等 ARM 平台上编译运行）

**实车部署额外硬件：**

- 线控车辆（brake-by-wire、steering-by-wire、throttle-by-wire、shift-by-wire），Apollo 主要在 Lincoln MKZ 上测试
- LiDAR（如 Velodyne HDL-64E S3、Hesai 等）
- 摄像头（支持多种工业相机）
- 毫米波雷达（Apollo 9.0+ 支持 4D 毫米波雷达）
- GNSS/IMU 组合惯导（如 NovAtel SPAN）

## Q3: 如何搭建 Apollo 开发环境？Docker 还是本地编译？

**推荐方式：Docker 容器开发环境。** Apollo 官方维护了预构建的 Docker 镜像，包含所有依赖库，避免环境配置问题。

**Docker 环境搭建步骤：**

1. 安装 Docker-CE 19.03+：

```bash
# 参考 Docker 官方安装文档
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

2. 安装 NVIDIA Container Toolkit（GPU 支持）：

```bash
# 参考 NVIDIA 官方文档安装 nvidia-docker
```

3. 克隆 Apollo 代码：

```bash
git clone https://github.com/ApolloAuto/apollo.git
cd apollo
```

4. 启动开发容器：

```bash
bash docker/scripts/dev_start.sh
```

5. 进入容器：

```bash
bash docker/scripts/dev_into.sh
```

6. 在容器内编译：

```bash
bash apollo.sh build
```

**关于本地编译：** 虽然理论上可以在宿主机直接编译，但由于 Apollo 依赖链极其庞大（包括 CUDA、PCL、OpenCV、Protobuf、Eigen 等数十个库），不推荐本地编译。Docker 方式可以确保一致的编译环境。

**Apollo 9.0+ 包管理方式：** 从 Apollo 9.0 开始引入了包管理工具，支持以 Package 方式安装和使用各功能模块，降低了二次开发门槛。

## Q4: CyberRT 是什么？它和 ROS 有什么区别？

CyberRT 是 Apollo 自研的高性能运行时通信框架，从 Apollo 3.5 版本开始取代 ROS 成为 Apollo 的底层通信中间件。

**CyberRT 的核心特性：**

- **高性能通信**：基于共享内存的进程间通信，降低数据拷贝开销
- **确定性调度**：支持协程（Coroutine）调度，可配置调度策略，满足自动驾驶实时性要求
- **组件化架构**：通过 Component 机制组织算法模块，每个 Component 处理输入数据并产生输出
- **丰富的开发工具**：提供 `cyber_monitor`、`cyber_recorder`、`cyber_channel`、`cyber_node`、`cyber_launch` 等命令行工具
- **Python API 支持**：提供 Python 接口用于快速原型开发和脚本编写

**CyberRT 与 ROS 的主要区别：**

| 对比维度 | CyberRT | ROS |
|----------|---------|-----|
| 通信机制 | 共享内存 + 进程内通信 | 基于 TCP/UDP 的话题通信 |
| 调度 | 协程调度，支持确定性调度策略 | 基于回调的线程模型 |
| 实时性 | 针对自动驾驶优化，延迟更低 | 通用设计，实时性较弱 |
| 序列化 | Protobuf | 自定义 msg 格式 |
| 服务发现 | 自研的 Topology Manager | ROS Master |
| 数据录制 | cyber_recorder（Record 格式） | rosbag |

**CyberRT 核心概念：**

- **Node**：通信的基本单元，类似 ROS 中的 Node
- **Channel**：数据通信的通道，类似 ROS 中的 Topic
- **Component**：封装算法逻辑的组件，由框架调度执行
- **DAG（有向无环图）**：描述 Component 的依赖关系和数据流
- **Launch 文件**：启动配置文件，定义要加载的 DAG 和进程信息

## Q5: 如何创建自定义模块/Component？

创建一个 CyberRT Component 需要以下步骤：

**1. 创建目录结构：**

```
my_component/
├── my_component.h        # 头文件
├── my_component.cc       # 源文件
├── BUILD                 # Bazel 构建文件
├── my_component.dag      # DAG 配置
└── my_component.launch   # Launch 启动文件
```

**2. 实现 Component 类（头文件）：**

```cpp
#include "cyber/component/component.h"
#include "my_component/proto/my_message.pb.h"

using apollo::cyber::Component;

class MyComponent : public Component<MyInputMsg> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<MyInputMsg>& msg) override;
};
CYBER_REGISTER_COMPONENT(MyComponent)
```

**3. 实现 Init 和 Proc 函数（源文件）：**

- `Init()`：初始化时调用一次，用于加载配置、初始化资源
- `Proc()`：每次收到输入数据时被框架调度调用

**4. 配置 DAG 文件：**

```protobuf
module_config {
  module_library : "/apollo/bazel-bin/my_component/libmy_component.so"
  components {
    class_name : "MyComponent"
    config {
      name : "my_component"
      readers {
        channel: "/apollo/my_input_channel"
      }
    }
  }
}
```

**5. 配置 Launch 文件：**

```xml
<cyber>
    <module>
        <name>my_component</name>
        <dag_conf>/apollo/my_component/my_component.dag</dag_conf>
        <process_name>my_component</process_name>
    </module>
</cyber>
```

**6. 编译与运行：**

```bash
# 编译
bash apollo.sh build

# 配置环境
source cyber/setup.bash

# 启动（二选一）
cyber_launch start my_component/my_component.launch
# 或
mainboard -d my_component/my_component.dag
```

除了标准 Component（数据触发），CyberRT 还支持 TimerComponent（定时触发），适用于不依赖外部消息、按固定频率执行的算法模块。

## Q6: Dreamview 是什么？如何使用？

Dreamview 是 Apollo 的可视化与仿真调试工具，提供基于 Web 的图形界面，用于查看车辆状态、传感器数据、规划轨迹等信息。

**Dreamview 版本：**

- **Dreamview（经典版）**：Apollo 早期版本提供的可视化工具，默认端口 8888
- **Dreamview Plus（新版）**：Apollo 9.0 引入的全新开发者工具，提供模式切换、面板自定义布局、资源中心等增强功能

**Dreamview 的主要功能：**

- **可视化显示**：实时显示车辆位置、障碍物、规划轨迹、车道线、交通信号灯等
- **模块管理**：启动/停止各个功能模块（感知、规划、控制等）
- **仿真调试**：在 Apollo 8.0+ 集成了本地仿真器，支持 PnC（Planning and Control）开发者直接在 Dreamview 中进行仿真调试
- **数据回放**：加载 Record 文件进行离线数据回放和分析
- **车辆配置**：选择和切换车辆配置、地图等

**启动 Dreamview：**

```bash
# 在 Apollo Docker 容器内
bash scripts/bootstrap.sh
```

启动后在浏览器访问 `http://localhost:8888` 即可打开 Dreamview。如果从远程主机访问，需将 `localhost` 替换为 Apollo 主机的实际 IP 地址。

**常见问题：**

- 无法访问页面：检查 Dreamview 进程是否正常运行（`ps aux | grep dreamview`）
- 页面空白：确认编译成功，且 `bootstrap.sh` 执行无报错
- 远程访问被拒：检查防火墙是否放行 8888 端口

## Q7: Apollo 如何处理传感器数据（LiDAR、Camera、Radar）？

Apollo 的感知模块（Perception）采用多传感器融合架构处理各类传感器数据：

**LiDAR 处理流程：**

1. 驱动层通过 CyberRT Channel 发布原始点云数据
2. 点云预处理：地面检测（`pointcloud_ground_detection`）、ROI 过滤（`pointcloud_map_based_roi`）
3. 目标检测：支持多种检测模型，包括 `lidar_detection`、`lidar_cpdet_detection`、`lidar_segmentation` 等
4. 目标跟踪：`lidar_tracking` 模块对检测结果进行多帧关联和跟踪

**Camera 处理流程：**

1. 驱动层发布图像数据
2. 目标检测：支持单阶段（`camera_detection_single_stage`）和多阶段（`camera_detection_multi_stage`）检测
3. Apollo 9.0+ 新增 BEV（Bird's Eye View）视角检测（`camera_detection_bev`）和 Occupancy 检测（`camera_detection_occupancy`）
4. 位置估计和精细化：`camera_location_estimation` 和 `camera_location_refinement`
5. 目标跟踪：`camera_tracking`

**Radar 处理流程：**

- 毫米波雷达数据通过 `third_party_perception` 模块或直接融合处理
- Apollo 9.0+ 新增对 4D 毫米波雷达的支持

**多传感器融合：**

`multi_sensor_fusion` 模块将来自 LiDAR、Camera、Radar 的检测结果进行时空对齐和融合，输出统一的障碍物列表，包含位置、速度、类别、置信度等信息，供下游预测和规划模块使用。

**附加功能：**

- `lane_detection`：车道线检测
- `barrier_recognition`：护栏识别
- `motion_service`：运动状态服务

## Q8: 如何运行仿真测试？

Apollo 提供多种仿真测试方式：

**方式一：Dreamview 数据回放仿真**

最常用的方式，通过回放预录制的 Record 数据来验证算法：

```bash
# 在 Docker 容器内启动 Dreamview
bash scripts/bootstrap.sh

# 回放 Record 文件
cyber_recorder play -f /path/to/your_record_file -l  # -l 表示循环播放
```

在 Dreamview 界面中可以实时查看回放数据的感知结果、规划轨迹等。

**方式二：Dreamview 内置仿真器（Apollo 8.0+）**

Apollo 8.0 在 Dreamview 中集成了本地仿真器，主要面向 PnC（规划与控制）开发者：

1. 在 Dreamview 中选择仿真模式
2. 选择测试场景
3. 启动仿真，观察规划和控制模块的表现

**方式三：Apollo 云端仿真平台**

百度 Apollo 提供云端仿真服务（Apollo Studio），支持：

- 大规模场景测试
- 自动化回归测试
- 多种天气和光照条件模拟

**仿真调试技巧：**

- 使用 `cyber_monitor` 工具实时监控各 Channel 的数据频率和内容
- 使用 `cyber_recorder` 录制特定 Channel 的数据用于后续分析
- 结合日志（AINFO、ADEBUG、AERROR）定位问题

## Q9: 如何进行传感器标定（Calibration）？

传感器标定是实车部署前的关键步骤，用于确定各传感器之间以及传感器与车辆之间的空间变换关系。

**Apollo 支持的标定类型：**

- LiDAR 到 IMU/GNSS 的外参标定
- Camera 到 LiDAR 的外参标定
- Radar 到 Camera 的外参标定
- Camera 内参标定

**标定前置条件：**

1. 所有传感器正常输出数据（可通过 `cyber_channel echo` 命令验证）：

```bash
# 检查 LiDAR 数据
cyber_channel echo /apollo/sensor/velodyne64/VelodyneScanUnified
```

2. 定位状态良好（RTK_FIXED，pos_type = 56）：

```bash
cyber_channel echo /apollo/sensor/gnss/ins_stat
```

3. 在开阔、特征丰富的场地采集标定数据

**标定流程：**

1. 按照要求采集标定数据（通常需要驾驶车辆以特定模式行驶）
2. 使用 Apollo 标定工具处理数据
3. 检查标定结果质量：良好的标定会产生锐利清晰的拼接点云，可清楚反映建筑立面、路灯杆、路沿等细节
4. 将标定结果（extrinsics 文件）放置到对应的车辆配置目录

**标定质量检查：**

- 观察点云拼接结果，好的标定结果会产生清晰的点云，差的标定会出现模糊、重影
- 以建筑立面、灯杆、路沿等直线特征作为参照物

**常见问题：**

- 标定程序权限错误：为输出目录添加写权限 `sudo chmod a+w /apollo/modules/calibration/data/<vehicle> -R`
- 日志权限错误：`sudo chmod a+x /apollo/data/log`

## Q10: Apollo 的地图格式是什么？如何制作高精地图？

**Apollo 高精地图格式：**

Apollo 使用自定义的 OpenDRIVE 扩展格式（通常称为 Apollo OpenDRIVE 或 Apollo HD Map 格式），以 Protobuf 二进制或 XML 格式存储。地图数据位于 `modules/map/` 目录下。

**地图核心数据结构包括：**

- **Road**：道路信息
- **Lane**：车道信息，包括车道线类型、限速、转向等
- **Junction**：交叉路口
- **Signal**：交通信号灯
- **StopSign**：停止标志
- **Crosswalk**：人行横道
- **ParkingSpace**：停车位
- **SpeedBump**：减速带

**地图相关工具（`modules/tools/` 目录）：**

- `create_map`：地图创建工具
- `map_gen`：地图生成工具
- `mapshow` / `mapviewers`：地图可视化查看工具
- `map_datachecker`：地图数据检查工具

**高精地图制作流程概述：**

1. 使用搭载 LiDAR 和 GNSS/IMU 的采集车辆在目标区域行驶，采集点云和定位数据
2. 对采集的点云数据进行拼接，生成高精度三维点云地图
3. 在点云地图基础上标注车道线、交通标志、信号灯等语义信息
4. 导出为 Apollo 兼容的地图格式
5. 使用地图检查工具验证地图质量

**使用现有地图：**

Apollo 附带了若干示例地图（位于 `modules/map/data/` 目录），可直接用于仿真和测试。在 Dreamview 中可以切换不同的地图。

## Q11: 如何在实车上部署 Apollo？

实车部署是一个系统工程，主要步骤如下：

**1. 硬件准备：**

- 线控车辆（Apollo 主要在 Lincoln MKZ 上验证）
- 工控机（IPC），满足算力要求
- 传感器套件（LiDAR、Camera、Radar、GNSS/IMU）
- CAN 卡（用于车辆线控通信）

**2. 硬件安装与接线：**

按照 Apollo 硬件安装指南完成传感器安装、接线和供电配置。

**3. 软件环境部署：**

```bash
# 在工控机上安装 Docker 和 NVIDIA 驱动
# 启动 Apollo 运行时容器
bash docker/scripts/runtime_start.sh

# 进入容器
bash docker/scripts/runtime_into.sh
```

注意：实车部署使用 `runtime_start.sh`（运行时容器），而非开发时使用的 `dev_start.sh`（开发容器）。

**4. 传感器标定：**

完成所有传感器的外参标定（参见 Q9）。

**5. 车辆适配与配置：**

- 配置 CAN 总线通信参数（`modules/canbus/`）
- 配置车辆动力学参数
- 在 Dreamview 中选择正确的车辆配置

**6. 分步验证：**

强烈建议按照 Apollo 版本路线逐步验证：

- 先验证 GPS 循迹（Apollo 1.0 级别功能）
- 再验证带感知的固定车道巡航
- 最后进行复杂场景测试

**安全注意事项：**

- 始终确保安全员在驾驶位
- 初次测试应在封闭场地进行
- 确认紧急停车机制（E-Stop）可靠

## Q12: Bazel 构建系统的常见问题

Apollo 使用 Bazel 作为构建系统。以下是常见问题和解决方法：

**问题：编译缓存占用空间过大**

Bazel 的编译缓存默认存储在 `/apollo/.cache/` 目录下，大型项目可能占用数十 GB 空间。

```bash
# 清除编译缓存
rm -rf /apollo/.cache/{bazel,build,repos}
```

**问题：编译速度慢**

- 确保分配了足够的内存（建议 16GB+）
- 使用 `--jobs` 参数控制并行编译数：`bazel build --jobs=8 //modules/planning/...`
- 增量编译：只编译修改过的模块，而非全量编译

**问题：依赖下载失败**

Bazel 需要从网络下载外部依赖，在国内网络环境下可能遇到超时：

- 使用 Apollo Docker 镜像（已预置大部分依赖）
- 配置代理或使用国内镜像源

**问题：BUILD 文件语法错误**

```bash
# 检查 BUILD 文件格式
# Apollo 使用 Starlark 语法（类 Python），注意使用 load() 导入规则
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
```

**问题：Protobuf 编译错误**

Apollo 大量使用 Protobuf 定义数据结构。如果修改了 `.proto` 文件：

- 确保 `BUILD` 文件中正确声明了 `proto_library` 和对应的 `cc_proto_library`
- 检查 `proto` 文件的 `import` 路径是否正确

**常用编译命令：**

```bash
# 全量编译
bash apollo.sh build

# 编译特定模块
bazel build //modules/planning/...

# 编译并运行测试
bazel test //modules/planning/...

# 清除编译输出
bazel clean --expunge
```

## Q13: 如何切换/适配不同的车辆平台？

Apollo 支持适配不同的线控车辆平台，主要涉及以下配置：

**车辆适配的核心模块：**

- `modules/canbus/`：CAN 总线通信模块，处理与车辆底盘的通信
- `modules/canbus_vehicle/`：各车辆平台的具体协议实现
- `modules/control/`：控制模块，需要根据车辆动力学特性调整参数

**适配步骤：**

1. **实现 CAN 协议**：根据目标车辆的 CAN 通信协议，在 `modules/canbus_vehicle/` 下创建新的车辆协议实现。Apollo 提供了协议生成工具（`modules/tools/gen_vehicle_protocol/`）辅助生成协议代码框架。

2. **配置车辆参数**：包括车辆长宽高、轮距、轴距、最大转向角、最大加减速度等物理参数。

3. **调整控制参数**：根据车辆的响应特性调整 PID 控制器参数或 MPC 控制器参数。

4. **配置传感器布局**：根据传感器在新车辆上的实际安装位置，更新传感器外参配置。

5. **在 Dreamview 中注册**：将新车辆配置注册到 Dreamview 中，使其可以在界面中选择。

**注意事项：**

- 不同车辆平台的线控响应特性差异较大，控制参数需要在实车上反复调试
- 建议先在低速封闭场地验证基本的线控功能（油门、刹车、转向）
- 安全起见，初次调试时应限制最大车速和最大转向角

## Q14: Apollo 支持哪些深度学习模型？如何替换？

Apollo 感知模块集成了多种深度学习模型：

**LiDAR 检测模型：**

- PointPillars 系列
- CenterPoint 系列
- 自研的 CPDET（`lidar_cpdet_detection`）模型
- LiDAR 分割模型（`lidar_segmentation`）

**Camera 检测模型：**

- 单阶段检测（`camera_detection_single_stage`）
- 多阶段检测（`camera_detection_multi_stage`）
- BEV 检测（`camera_detection_bev`）— Apollo 9.0+ 新增
- Occupancy 检测（`camera_detection_occupancy`）— Apollo 9.0+ 新增

**车道线检测模型：**

- `lane_detection` 模块中的深度学习模型

**模型替换方法：**

1. **使用 Apollo 增量训练**：Apollo 9.0+ 开放了 LiDAR 和 Camera 检测模型的增量训练方法，可以在已有模型基础上用自己的数据进行微调。

2. **替换推理模型**：
   - 将训练好的模型导出为 Apollo 支持的推理格式（通常为 ONNX 或 LibTorch）
   - 替换对应模块 `data/` 或 `model/` 目录下的模型文件
   - 修改对应的配置文件，指定新模型路径和参数

3. **自定义感知插件**：Apollo 9.0+ 通过 `perception_plugin` 机制支持以插件方式扩展感知算法，无需修改主干代码。

**模型推理框架：**

- x86_64 平台：LibTorch 1.7.0
- aarch64 平台：LibTorch 1.11.0（Apollo 2024 年 11 月更新）
- 同时支持 TensorRT 加速推理

## Q15: 常见编译错误和解决方法

**错误：CUDA 相关编译失败**

```
nvcc fatal: Unsupported gpu architecture 'compute_XX'
```

确认 CUDA 版本与 GPU 架构匹配。Apollo 当前使用 CUDA 11.8，支持 NVIDIA Ada Lovelace（40 系列）及之前的 GPU。确保 NVIDIA 驱动版本 >= 520.61.05。

**错误：内存不足（OOM）**

```
C++ compilation of rule ... failed: (Exit 137)
```

Bazel 并行编译可能耗尽内存。解决方法：

```bash
# 减少并行 job 数
bazel build --jobs=4 //modules/...
# 或为容器分配更多内存
```

**错误：Docker 容器内 GPU 不可用**

```bash
# 确认 NVIDIA Container Toolkit 已安装
nvidia-smi  # 在容器内执行，应能看到 GPU 信息

# 如果无法看到，重启容器
bash docker/scripts/dev_start.sh
```

**错误：Protobuf 版本冲突**

如果引入第三方库导致 Protobuf 版本冲突，需确保所有依赖使用 Apollo 内置的 Protobuf 版本。

**错误：`source cyber/setup.bash` 失败**

确保在 Apollo Docker 容器内执行，且已完成编译。该脚本设置 CyberRT 相关的环境变量和路径。

**升级后编译失败：**

拉取新版本代码后，建议清除旧的编译缓存：

```bash
rm -rf /apollo/.cache/{bazel,build,repos}
# 重启容器
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
# 重新编译
bash apollo.sh build
```

**调试建议：**

- 使用 `AINFO`、`ADEBUG`、`AERROR` 等日志宏输出调试信息
- 大多数问题可以通过日志定位，如需更详细调试可使用 GDB（Apollo 提供了 `dev_start_gdb_server.sh` 脚本）

## Q16: 如何使用 Record 进行数据回放？

Record 是 CyberRT 的数据录制和回放格式（类似 ROS 中的 rosbag），用于记录和重放各 Channel 上的消息数据。

**录制数据：**

```bash
# 录制所有 Channel
cyber_recorder record -a -o /path/to/output.record

# 录制指定 Channel
cyber_recorder record -c /apollo/sensor/lidar128/compensator/PointCloud2 \
  -c /apollo/localization/pose \
  -c /apollo/perception/obstacles \
  -o /path/to/output.record
```

**查看 Record 信息：**

```bash
# 查看 Record 文件的基本信息
cyber_recorder info /path/to/your.record
```

该命令会显示 Record 文件的时长、消息数量、包含的 Channel 列表及每个 Channel 的消息类型和数量。

**回放数据：**

```bash
# 基本回放
cyber_recorder play -f /path/to/your.record

# 循环回放
cyber_recorder play -f /path/to/your.record -l

# 指定回放速率（0.5 表示半速）
cyber_recorder play -f /path/to/your.record -r 0.5

# 回放指定 Channel
cyber_recorder play -f /path/to/your.record -c /apollo/sensor/lidar128/compensator/PointCloud2
```

**配合 Dreamview 使用：**

1. 启动 Dreamview：`bash scripts/bootstrap.sh`
2. 在 Dreamview 中选择对应的地图和车辆配置
3. 启动需要测试的模块（如 Perception、Planning）
4. 在另一个终端回放 Record 文件
5. 在 Dreamview 界面中实时查看各模块的处理结果

**其他 CyberRT 工具：**

- `cyber_monitor`：实时监控各 Channel 的数据频率和内容
- `cyber_channel list`：列出当前活跃的 Channel
- `cyber_channel echo <channel_name>`：打印指定 Channel 的实时数据
- `cyber_node list`：列出当前活跃的 Node
- `cyber_launch start <launch_file>`：通过 Launch 文件启动组件

**常见问题：**

- 回放时模块未收到数据：检查 Record 中的 Channel 名称是否与模块订阅的 Channel 一致
- 回放数据时间戳过旧：某些模块可能会丢弃时间戳过旧的数据，使用 Dreamview 的 "Sim Control" 模式可以解决此问题
