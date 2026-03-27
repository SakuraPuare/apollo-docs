---
title: Common 公共工具库与消息定义
description: Apollo 自动驾驶平台 common 工具库和 common_msgs 消息定义模块的技术文档，涵盖数学库、滤波器、键值数据库、车辆状态管理及全平台 Protobuf 消息结构。
---

# Common 公共工具库与消息定义

Apollo 的 `common` 和 `common_msgs` 是两个基础支撑模块。`common` 提供全平台共享的工具类、数学库和基础设施；`common_msgs` 统一定义了各模块间通信所使用的 Protobuf 消息结构。几乎所有上层模块（感知、规划、控制等）都依赖这两个模块。

## 第一部分：common 工具库

### 模块职责

`common` 模块（位于 `modules/common/`）为 Apollo 各功能模块提供：

- 模块间通信的 Topic 名称注册与管理
- 通用数学运算（几何、滤波、插值）
- 车辆状态与车辆模型的统一抽象
- 轻量级键值存储
- 日志与监控基础设施
- 工厂模式、序列化解析等通用工具

### 子模块详解

#### adapters — Topic 注册与通信适配

`adapters` 子模块通过 `adapter_gflags` 定义了 Apollo 系统中 110+ 个 Topic 名称，是模块间基于 Cyber RT 消息通信的基础。每个 Topic 对应一个 `DEFINE_string` 类型的 gflag，上层模块通过引用这些 flag 来订阅或发布消息，避免硬编码 Topic 字符串。

```cpp
// 典型用法示例
DEFINE_string(chassis_topic, "/apollo/canbus/chassis", "chassis topic name");
DEFINE_string(localization_topic, "/apollo/localization/pose", "localization topic name");
```

#### configs / data — 车辆配置

`configs` 和 `data` 是两个独立的子目录，分别存放配置定义和车辆物理参数数据文件，包括车辆尺寸、轮距、最大转向角等。这些配置以 Protobuf 文本格式或 JSON 存储，在系统启动时加载，供规划、控制等模块使用。

#### filters — 数字滤波器

提供信号处理所需的滤波器实现：

| 滤波器类型 | 类名 | 用途 |
|---|---|---|
| 数字低通滤波器 | `DigitalFilter` | 平滑传感器噪声 |
| 均值滤波器 | `MeanFilter` | 滑动窗口均值计算 |

滤波器广泛用于控制模块中对速度、加速度等信号的平滑处理。

#### kv_db — 键值数据库

轻量级键值数据库，用于存储系统范围的运行时参数。典型使用场景包括：

- 存储标定结果
- 缓存模块间需要持久化的中间状态
- 保存用户偏好设置

接口简洁，支持 `Put(key, value)` 和 `Get(key)` 操作。

#### latency_recorder — 时延记录

记录各模块处理时延，用于性能监控和瓶颈分析。与 `monitor_log` 配合，可将时延数据上报至 Dreamview 进行可视化展示。

#### math — 数学库

Apollo 数学库是规划和控制模块的核心依赖，提供丰富的几何与数值计算能力：

**几何基元：**

| 类 | 描述 |
|---|---|
| `Vec2d` | 二维向量，支持加减、点积、叉积、旋转等 |
| `LineSegment2d` | 二维线段，支持投影、距离计算 |
| `Box2d` | 有向矩形包围盒，用于碰撞检测 |
| `Polygon2d` | 凸多边形，支持包含判断、面积计算 |
| `AABox2d` | 轴对齐包围盒 |

**数值算法：**

| 算法 | 描述 |
|---|---|
| `KalmanFilter` | 卡尔曼滤波，用于状态估计与预测 |
| `lerp` / `slerp` / `InterpolateUsingLinearApproximation` | 线性插值函数，用于轨迹点补全 |
| `IntegrateBySimpson` / `IntegrateByTrapezoidal` / `IntegrateByGaussLegendre` | 数值积分函数 |
| `CartesianFrenetConverter` | 笛卡尔坐标与 Frenet 坐标互转 |

#### monitor_log — 日志与监控

提供 `MonitorLogBuffer` 类，用于向 Dreamview 的监控面板发送监控消息。与 Apollo 的 `MonitorMessage` proto 对应，支持 INFO、WARN、ERROR、FATAL 等级别。

> 注意：`AINFO`、`AERROR` 等日志宏由 Cyber RT 日志模块提供，而非 `monitor_log`。`monitor_log` 专门用于向 Dreamview 上报监控信息。

```cpp
// monitor_log 使用示例
MonitorLogBuffer buffer(MonitorMessageItem::PLANNING);
buffer.INFO("Planning module initialized");
buffer.ERROR("Failed to get localization data");
```

#### proto — 公共序列化定义

定义 `common` 模块自身使用的 Protobuf 消息。注意 proto 文件分散在各子模块中，没有统一的 `common/proto` 顶级目录。常见的 proto 定义包括：

- `VehicleState`：车辆综合状态
- `AdapterConfig`：适配器配置

> 注意：`VehicleParam`（车辆物理参数）定义在 `common_msgs/config_msgs/vehicle_config.proto` 中，而非 `common/proto`。`Status` 是 C++ 类（位于 `status/status.h`），不是 Proto 消息。

#### status — 执行状态

提供 `Status` 类，用于函数返回值的统一状态判断，类似于 gRPC 的 Status 模式：

```cpp
Status status = planning.Plan(current_time);
if (!status.ok()) {
  AERROR << "Planning failed: " << status.error_message();
}
```

#### util — 通用工具集

| 工具 | 描述 |
|---|---|
| `Factory` | 工厂设计模式模板，支持按名称动态创建对象 |
| `EncodeBase64` / `StrFormat` 等 | 字符串编码与格式化独立函数 |
| `MessageUtil` | Protobuf 消息的序列化/反序列化辅助 |
| `PointFactory` | 快速构造各类几何点对象 |
| `JsonUtil` | JSON 解析工具 |

`Factory` 模式在 Apollo 中广泛使用，例如规划模块通过工厂注册不同的 Planner 实现。

#### vehicle_state — 车辆状态

封装车辆当前运行状态的统一访问接口，聚合来自底盘（Chassis）和定位（Localization）的数据：

- 当前位置（x, y, z）
- 速度与加速度
- 航向角（heading）
- 转向角
- 档位状态

#### service_wrapper — 服务封装

提供对底层服务调用的封装工具，简化模块间的 Service 通信模式。

#### vehicle_model — 车辆模型

提供车辆运动学模型，用于轨迹预测和控制仿真。基于自行车模型（Bicycle Model），输入转向角和速度，输出预测轨迹。

### 数据流

```
传感器数据 ──→ adapters (Topic 路由) ──→ 各功能模块
                                           │
车辆配置 ──→ configs / data ─────────────────┤
                                           │
              math / filters ──────────────┤ (计算支持)
                                           │
              vehicle_state ←── Chassis + Localization
                                           │
              monitor_log ──→ Dreamview 监控面板
              kv_db ──→ 持久化存储
```

### 配置方式

- **Topic 名称**：通过 gflags 在 `adapter_gflags` 中定义，可通过命令行参数或 flagfile 覆盖
- **车辆参数**：`configs/` 和 `data/` 下的配置文件，Protobuf 文本格式
- **滤波器参数**：在各使用模块的配置文件中指定截止频率、窗口大小等

---

## 第二部分：common_msgs 消息定义

### 模块职责

`common_msgs` 模块（位于 `modules/common_msgs/`）集中管理 Apollo 全平台的 Protobuf 消息定义。将消息定义从功能模块中解耦出来，使得：

- 各模块只需依赖 `common_msgs` 即可获取所需消息类型
- 消息结构变更不会导致功能模块间的循环依赖
- 统一版本管理，保证消息兼容性

该模块包含 22 个消息子目录，覆盖音频、底盘、控制、感知、规划、定位、地图等全部子系统。

### 核心消息定义

#### basic_msgs — 基础消息

全平台最基础的消息类型，被几乎所有其他消息引用：

| 消息 | 描述 |
|---|---|
| `Header` | 通用消息头，包含时间戳、模块名、序列号 |
| `ErrorCode` | 统一错误码枚举 |
| `VehicleSignal` | 车辆信号（转向灯、喇叭等） |
| `EngageAdvice` | 驾驶介入建议（定义于 `drive_state.proto`） |
| `SLPoint` / `FrenetFramePoint` / `SpeedPoint` / `PathPoint` / `TrajectoryPoint` | 规划控制用的路径与轨迹点（定义于 `pnc_point.proto`） |
| `PointENU` / `PointLLH` / `Point2D` / `Point3D` / `Quaternion` / `Polygon` | 几何基元（定义于 `geometry.proto`） |

#### chassis_msgs — 底盘消息

`Chassis` 是底盘状态的核心消息，包含：

- `DrivingMode`：驾驶模式（自动/手动/紧急）
- `GearPosition`：档位（P/R/N/D）
- 车速、方向盘转角、油门/刹车踏板位置
- 车辆信号灯状态

#### control_msgs — 控制消息

`ControlCommand` 是控制模块的输出消息：

```protobuf
message ControlCommand {
  optional double throttle = 3;   // 油门开度 [0, 100]
  optional double brake = 4;      // 制动开度 [0, 100]
  optional double steering_target = 7;  // 目标转向角
  // ...
}
```

#### sensor_msgs — 传感器消息

| 消息 | 描述 |
|---|---|
| `PointCloud` | 激光雷达点云数据 |
| `Image` | 相机图像帧 |
| `RadarObstacles` | 毫米波雷达检测结果 |
| `Gnss` | 全球导航卫星系统数据 |
| `Imu` | 惯性测量单元数据 |

#### perception_msgs — 感知消息

| 消息 | 描述 |
|---|---|
| `PerceptionObstacles` | 感知到的障碍物列表，每个障碍物包含类型、位置、速度、包围盒 |
| `TrafficLightDetection` | 交通信号灯检测结果，包含颜色和置信度 |

#### planning_msgs — 规划消息

| 消息 | 描述 |
|---|---|
| `ADCTrajectory` | 自动驾驶车辆规划轨迹，包含轨迹点序列、决策信息 |
| `PlanningCommand` | 规划指令 |
| `DecisionResult` | 规划决策（跟车、超车、停车等） |

#### localization_msgs — 定位消息

| 消息 | 描述 |
|---|---|
| `LocalizationEstimate` | 定位估计结果，包含位姿、不确定性 |
| `Pose` | 六自由度位姿（位置 + 姿态） |

#### routing_msgs — 路由消息

| 消息 | 描述 |
|---|---|
| `RoutingRequest` | 路由请求，包含起点、终点、途经点 |
| `RoutingResponse` | 路由响应，包含规划的道路级路径 |

#### transform_msgs — 坐标变换消息

| 消息 | 描述 |
|---|---|
| `Transform` | 坐标变换（平移 + 旋转） |
| `TransformStamped` | 带时间戳的坐标变换，用于 TF 树 |

### 其他消息子目录

| 子目录 | 描述 |
|---|---|
| `audio_msgs` | 音频相关消息 |
| `config_msgs` | 系统配置消息 |
| `dreamview_msgs` | Dreamview 可视化界面消息 |
| `drivers_msgs` | 驱动层消息 |
| `external_command_msgs` | 外部指令消息（HMI 交互） |
| `guardian_msgs` | 安全守护模块消息 |
| `map_msgs` | 高精地图消息 |
| `monitor_msgs` | 系统监控消息 |
| `prediction_msgs` | 障碍物预测消息 |
| `simulation_msgs` | 仿真环境消息 |
| `storytelling_msgs` | 场景叙事消息（用于调试与回放） |
| `task_manager_msgs` | 任务管理消息 |
| `v2x_msgs` | 车路协同（V2X）消息 |

### 消息数据流

```
common_msgs 在 Apollo 数据流中的位置：

传感器硬件
  │
  ▼
drivers (sensor_msgs) ──→ 感知 (perception_msgs) ──→ 预测 (prediction_msgs)
  │                                                         │
  ▼                                                         ▼
定位 (localization_msgs)                              规划 (planning_msgs)
  │                                                         │
  ▼                                                         ▼
底盘 (chassis_msgs) ←────────────────────────── 控制 (control_msgs)
  │
  ▼
Dreamview (dreamview_msgs) ←── 监控 (monitor_msgs)
```

所有模块间的消息传递均通过 Cyber RT 的发布-订阅机制完成，Topic 名称由 `common/adapters` 统一管理。

### 配置方式

- 消息定义文件位于各子目录的 `.proto` 文件中
- 编译时通过 Bazel 的 `proto_library` 规则生成对应语言的绑定代码
- 模块通过在 `BUILD` 文件中声明对 `common_msgs` 相应子包的依赖来使用消息类型
