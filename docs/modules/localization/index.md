# Localization 定位模块

Apollo 定位模块负责为自动驾驶系统提供高精度的车辆位姿估计（位置 + 姿态），是规划、控制等下游模块的基础依赖。模块提供三种可选的定位方案：RTK（实时动态差分定位）、MSF（多传感器融合定位）和 NDT（正态分布变换定位），通过配置文件灵活切换。

## 模块职责

Localization 模块实时输出车辆在世界坐标系（UTM 投影）下的六自由度位姿（位置 x/y/z + 姿态 roll/pitch/yaw），具体职责包括：

- 接收并处理多种传感器数据：GNSS（GPS/BDS/GLONASS 等）、IMU（惯性测量单元）、LiDAR 点云
- 根据选定的定位方案，对传感器数据进行融合计算，输出 `LocalizationEstimate` 消息
- 通过 TF 广播车辆坐标系到世界坐标系的变换关系（`world` -> `localization`）
- 输出定位状态信息 `LocalizationStatus`，供下游模块判断定位质量
- 支持 GNSS 时间戳补偿（`LocalizationGnssCompensator`）、IMU-GPS 时间对齐与插值等数据预处理
- 默认输出频率 100Hz（由 `localization_publish_freq` 控制），输出 channel 为 `/apollo/localization/pose`

模块内置三种可切换的定位方案：

| 方案 | 精度 | 依赖传感器 | 适用场景 |
|------|------|-----------|---------|
| RTK | 厘米级（依赖 RTK 信号质量） | GNSS + IMU | 开阔环境、快速验证 |
| MSF（多传感器融合） | 厘米级 | GNSS + IMU + LiDAR + 高精地图 | 生产部署、复杂城市环境 |
| NDT | 分米级 | GNSS（里程计） + LiDAR + NDT 地图 | 无 RTK 信号的隧道/地下场景 |

通过 `localization_config.pb.txt` 中的 `localization_type` 字段（`RTK` / `MSF`）选择方案，NDT 方案通过独立的 launch 文件启动。

---

## 核心类与接口

### 公共层

| 类 / 文件 | 路径 | 职责 |
|-----------|------|------|
| `localization_gflags` | `common/localization_gflags.h` | 全局 GFlags 参数声明（LiDAR、GNSS、INS、NDT 等） |
| `LocalizationGnssCompensator` | `common/gnss_compensator.h` | GNSS 无效时间戳补偿 |
| `LocalizationConfig` | `proto/localization_config.proto` | 定位方案选择（RTK=0, MSF=1） |

### RTK 方案

| 类 | 路径 | 职责 |
|----|------|------|
| `RTKLocalizationComponent` | `rtk/rtk_localization_component.h` | Cyber 组件入口，订阅 GPS/IMU/InsStat，发布定位结果与 TF |
| `RTKLocalization` | `rtk/rtk_localization.h` | 核心算法：GPS-IMU 时间对齐、IMU 插值、位姿合成 |

### MSF 方案

| 类 | 路径 | 职责 |
|----|------|------|
| `MSFLocalizationComponent` | `msf/msf_localization_component.h` | Cyber 组件入口，订阅 IMU/LiDAR/GnssBestPose/Heading |
| `LocalizationMsgPublisher` | `msf/msf_localization_component.h` | 统一发布融合结果、GNSS 子结果、LiDAR 子结果、状态 |
| `MSFLocalization` | `msf/msf_localization.h` | 参数初始化，将传感器数据分发给 `LocalizationInteg` |
| `LocalizationInteg` | `msf/local_integ/localization_integ.h` | 融合引擎外观接口，IMU 坐标系转换（FLU/RFU） |
| `LocalizationIntegImpl` | `msf/local_integ/localization_integ_impl.h` | 融合引擎实现，协调四大子处理器 |
| `LocalizationIntegProcess` | `msf/local_integ/localization_integ_process.h` | SINS 惯导递推（封装 `Sins` 引擎），IMU 积分与量测更新 |
| `LocalizationLidarProcess` | `msf/local_integ/localization_lidar_process.h` | LiDAR 点云与高精地图匹配定位 |
| `LocalizationGnssProcess` | `msf/local_integ/localization_gnss_process.h` | GNSS 原始观测量处理、RTK 解算（封装 `GnssSolver`） |
| `MeasureRepublishProcess` | `msf/local_integ/measure_republish_process.h` | 量测数据预处理与重发布（BestGnssPos/LiDAR/Heading → `MeasureData`） |
| `OnlineLocalizationExpert` | `msf/local_integ/online_localization_expert.h` | 在线状态监控：IMU 延迟/丢失检测、GNSS/LiDAR 丢失检测、定位精度评估 |
| `LocalizationLidar` | `msf/local_integ/localization_lidar.h` | LiDAR 地图匹配核心（基于 PyramidMap 的强度/高度匹配） |
| `LidarMsgTransfer` | `msf/local_integ/lidar_msg_transfer.h` | PointCloud protobuf → `LidarFrame` 转换 |
| `GnssMagTransfer` | `msf/local_integ/gnss_msg_transfer.h` | GNSS 原始观测量 protobuf → 内部结构体转换 |

### NDT 方案

| 类 | 路径 | 职责 |
|----|------|------|
| `NDTLocalizationComponent` | `ndt/ndt_localization_component.h` | Cyber 组件入口，订阅 Odometry/LiDAR/InsStat |
| `NDTLocalization` | `ndt/ndt_localization.h` | NDT 定位主逻辑：里程计缓冲、点云匹配、位姿合成 |
| `LidarLocatorNdt` | `ndt/ndt_locator/lidar_locator_ndt.h` | NDT 点云-地图匹配器，加载 NdtMap 并执行配准 |
| `NormalDistributionsTransform` | `ndt/ndt_locator/ndt_solver.h` | NDT 配准求解器（基于 Magnusson 2009），牛顿法优化 |
| `LocalizationPoseBuffer` | `ndt/localization_pose_buffer.h` | LiDAR 位姿与里程计位姿的环形缓冲区，用于帧间位姿推算 |

### 地图子系统（MSF 内部）

| 子目录 | 职责 |
|--------|------|
| `msf/local_map/` | 基础地图框架（BaseMap）、无损地图（LosslessMap）、有损地图（LossyMap2D）、NDT 地图 |
| `msf/local_pyramid_map/` | 金字塔多分辨率地图：PyramidMap（强度+高度）、NdtMap（正态分布） |

---

## 定位方案与算法概述

### RTK 定位

RTK（Real-Time Kinematic）是最简单的定位方案，直接使用 GNSS 接收机输出的高精度位置，结合 IMU 数据进行姿态补全。

**工作流程：**

1. `RTKLocalizationComponent` 以 GPS Odometry（`/apollo/sensor/gnss/odometry`）为主触发消息
2. 收到 GPS 消息后，`RTKLocalization::GpsCallback` 执行：
   - 检查 IMU 缓冲区和 GPS 状态缓冲区是否非空
   - 调用 `FindMatchingIMU` 在 IMU 列表中查找与 GPS 时间戳最近的两帧 IMU 数据
   - 调用 `InterpolateIMU` 对两帧 IMU 进行线性插值，得到与 GPS 同一时刻的姿态角速度和线加速度
   - 调用 `ComposeLocalizationMsg` 将 GPS 位置 + 插值后的 IMU 姿态合成为 `LocalizationEstimate`
3. WatchDog 机制监控 GPS 消息间隔，超过 `gps_time_delay_tolerance_`（默认 1s）则告警
4. 支持 `map_offset` 地图偏移补偿
5. 支持 IMU-Localization 外参补偿（`CompensateImuLocalizationExtrinsic`）

**关键参数：**
- `gps_imu_time_diff_threshold`：GPS 与 IMU 时间差阈值（默认 0.02s）
- `imu_list_max_size`：IMU 缓冲区大小（代码默认 50，配置文件覆盖为 20）
- 发布频率：100Hz（由 GPS 消息驱动）

### MSF 多传感器融合定位

MSF（Multi-Sensor Fusion）是生产级定位方案，采用 SINS（捷联惯导系统）为核心，融合 GNSS 和 LiDAR 量测进行误差校正。

**整体架构：**

```
IMU (100Hz+)  ──→  LocalizationIntegProcess (SINS 递推)
                         ↑ MeasureData 量测更新
                         │
GNSS BestPose ──→  MeasureRepublishProcess ──→ MeasureData
GNSS Heading  ──→       │
GNSS Raw Obs  ──→  LocalizationGnssProcess (RTK 解算) ──→ MeasureData
GNSS Ephemeris──→       │
                         │
LiDAR PointCloud──→ LocalizationLidarProcess (地图匹配) ──→ MeasureData
```

**SINS 惯导递推（`LocalizationIntegProcess`）：**

- 封装底层 `Sins` 引擎（`localization_msf/sins.h`），以 IMU 数据为驱动进行高频位姿递推
- 状态机：`NOT_INIT → NOT_STABLE → OK → VALID`
- 接收 `MeasureData` 量测数据进行误差校正（通过异步队列 `measure_data_queue_` 处理）
- 输出：校正后的 `InsPva`（位置/速度/姿态）及 9x9 协方差矩阵

**GNSS 处理（`LocalizationGnssProcess` + `MeasureRepublishProcess`）：**

- 支持两种 GNSS 模式（`GnssMode`）：
  - `NOVATEL`（默认）：直接使用 NovAtel 接收机输出的 BestGnssPos
  - `SELF`：使用原始观测量（EpochObservation）+ 星历（GnssEphemeris）自主 RTK 解算
- `MeasureRepublishProcess` 负责：
  - 将 BestGnssPos 转换为 UTM 坐标系下的 `MeasureData`
  - 校验 GNSS 状态（`sol_status`、XY 标准差阈值 5.0m）
  - 从 BestGnssPos 计算速度（差分法）
  - 处理 GNSS Heading（双天线航向）
  - 处理 LiDAR 定位结果转 `MeasureData`
- `MeasureData` 量测类型（`IntegMeasure.MeasureType`）：
  - `GNSS_POS_ONLY` / `GNSS_POS_VEL` / `GNSS_VEL_ONLY` / `GNSS_POS_XY`
  - `POINT_CLOUD_POS`（LiDAR）
  - `ODOMETER_VEL_ONLY` / `VEHICLE_CONSTRAINT`

**LiDAR 地图匹配定位（`LocalizationLidarProcess`）：**

- 使用 `PoseForcast` 进行 IMU 辅助的位姿预测，为 LiDAR 匹配提供初始值
- 预测状态机：`NOT_VALID → INITIAL → INCREMENT`
- 预测位置来源优先级：`INSPVA_IMU_WHEEL > INSPVA_IMU > INSPVA_ONLY`
- `LocalizationLidar` 执行实际匹配：
  - 加载 PyramidMap（金字塔多分辨率地图），包含强度和高度信息
  - 定位模式（`localization_mode`）：0=强度匹配，1=高度匹配，2=融合匹配（默认）
  - 航向对齐模式（`yaw_align_mode`）：0=关闭，1=融合，2=多线程融合（默认）
  - 输出位姿、3x3 协方差矩阵和匹配得分
- LiDAR 状态机：`NOT_VALID → NOT_STABLE → OK`
- 支持 AVX 指令集加速（`if_use_avx` 标志）

**在线状态监控（`OnlineLocalizationExpert`）：**

- IMU 延迟检测：三级阈值（0.02s / 0.05s / 0.1s）
- IMU 丢失检测：三级阈值（0.01s / 0.05s / 0.1s）
- GNSS BestPose 丢失检测：阈值 2.0s
- LiDAR 丢失检测：阈值 2.0s
- 定位精度评估：XY 标准差两级阈值（0.15m / 0.3m）
- 输出 `MsfStatus` 和 `MsfSensorMsgStatus`

**融合状态等级：**
```
LocalizationIntegState: OK → WARNNING → ERROR → CRITIAL_ERROR → FATAL_ERROR
```

### NDT 定位

NDT（Normal Distributions Transform）方案使用 LiDAR 点云与预建的 NDT 地图进行配准定位，不依赖 RTK 信号。

**工作流程：**

1. `NDTLocalizationComponent` 以 GNSS Odometry 为主触发，同时订阅 LiDAR 点云和 InsStat
2. 收到 LiDAR 点云后，`NDTLocalization::LidarCallback` 执行：
   - 将 PointCloud protobuf 转换为 `LidarFrame`（过滤高度超过 `max_height_` 的点）
   - 通过 TF2 或里程计缓冲区查询当前时刻的预测位姿
   - 应用 LiDAR 外参变换
   - 调用 `LidarLocatorNdt::Update` 将点云投影到地图坐标系
   - 调用 NDT 配准求解器进行点云-地图匹配
3. 收到 Odometry 消息后：
   - 将里程计位姿存入环形缓冲区（最大 100 帧）
   - 通过 `LocalizationPoseBuffer` 利用最近一次 LiDAR 匹配结果和当前里程计增量推算输出位姿

**NDT 配准算法（`NormalDistributionsTransform`）：**

- 基于 Magnusson 2009 论文实现
- 将目标点云（地图）体素化，每个体素计算均值和协方差矩阵
- 使用牛顿法优化 6-DOF 变换参数，最小化点到正态分布的负对数似然
- 计算 Jacobian（6.18 式）和 Hessian（6.20 式）矩阵
- 使用 More-Thuente 线搜索确定步长
- 关键参数：
  - `resolution`：体素边长（目标分辨率）
  - `step_size`：最大步长
  - `transformation_epsilon`：收敛阈值
  - `max_iterations`：最大迭代次数
  - `outlier_ratio`：离群点比例

**NDT 得分监控：**
- `warnning_ndt_score`（默认 1.0）：匹配得分超过此值发出警告
- `error_ndt_score`（默认 2.0）：匹配得分超过此值判定为错误
- `bad_score_count_threshold`（默认 10）：连续坏分数计数阈值

---

## 传感器融合策略

Apollo 定位模块的传感器融合策略因方案而异，核心思想是利用不同传感器的互补特性实现高精度、高鲁棒性的定位。

### RTK 方案：GPS-IMU 简单融合

RTK 方案采用最简单的融合策略——时间对齐插值：

- GPS 提供绝对位置（经纬度 + 高程），IMU 提供高频姿态（角速度 + 线加速度）
- 融合方式：以 GPS 时间戳为基准，在 IMU 缓冲队列中查找前后两帧，通过线性插值（`InterpolateIMU`）得到同一时刻的 IMU 数据
- 位置直接取 GPS 值，姿态取插值后的 IMU 值，合成最终 `LocalizationEstimate`
- 局限性：完全依赖 GPS 信号质量，无法在 GPS 失锁时维持定位

### MSF 方案：SINS + GNSS + LiDAR 紧耦合融合

MSF 是生产级融合方案，采用经典的 SINS（捷联惯导系统）+ 外部量测更新架构：

**融合框架：**

1. SINS 惯导递推（预测步）：以 IMU 数据（100Hz+）为驱动，通过 `Sins` 引擎进行高频位姿递推，输出预测的位置/速度/姿态（PVA）及 9x9 协方差矩阵
2. 量测更新（校正步）：当 GNSS 或 LiDAR 量测到达时，将其转换为统一的 `MeasureData` 格式，送入 `LocalizationIntegProcess` 的量测队列进行误差校正

**多源量测融合优先级与策略：**

- GNSS 量测：提供全局绝对位置约束，通过 `MeasureRepublishProcess` 预处理
  - 状态校验：检查 `sol_status`、XY 标准差（阈值 5.0m）
  - 坐标转换：WGS84 经纬度 → UTM 投影坐标
  - 速度估计：通过相邻 BestGnssPos 差分计算
  - 杆臂补偿：利用 IMU-天线杆臂参数（`imu_to_ant_offset`）将天线位置转换到 IMU 中心
- LiDAR 量测：提供高精度局部位姿约束
  - 通过 `LocalizationLidarProcess` 执行点云-地图匹配
  - 使用 SINS 预测位姿作为匹配初始值（`PoseForcast`）
  - 匹配结果经 `MeasureRepublishProcess::LidarLocalProcess` 转换为 `MeasureData`
  - 输出包含 3x3 协方差矩阵，反映匹配置信度
- GNSS Heading：双天线航向量测，提供航向角约束

**量测类型（`MeasureType`）：**

| 类型 | 来源 | 提供信息 |
|------|------|---------|
| `GNSS_POS_ONLY` | GNSS BestPos | 仅位置 |
| `GNSS_POS_VEL` | GNSS BestPos（含速度） | 位置 + 速度 |
| `GNSS_VEL_ONLY` | GNSS BestPos | 仅速度 |
| `GNSS_POS_XY` | GNSS BestPos | 仅水平位置 |
| `POINT_CLOUD_POS` | LiDAR 地图匹配 | 位置 + 姿态 |
| `ODOMETER_VEL_ONLY` | 里程计 | 仅速度 |
| `VEHICLE_CONSTRAINT` | 车辆运动约束 | 非完整性约束 |

**退化处理：**

- GNSS 信号丢失（超过 `bestgnsspose_loss_time_threshold` 2.0s）：依赖 SINS + LiDAR 维持定位
- LiDAR 信号丢失（超过 `lidar_loss_time_threshold` 2.0s）：依赖 SINS + GNSS 维持定位
- 双源均丢失：SINS 纯惯导递推（精度随时间快速退化）
- `OnlineLocalizationExpert` 实时评估融合状态，输出五级状态：`OK → WARNNING → ERROR → CRITIAL_ERROR → FATAL_ERROR`

### NDT 方案：里程计 + LiDAR-NDT 匹配融合

NDT 方案的融合策略相对简单：

- 里程计（GNSS Odometry）提供帧间相对运动，存入环形缓冲区（最大 100 帧）
- LiDAR 点云通过 NDT 配准与预建地图匹配，获得绝对位姿
- 融合方式：以最近一次 LiDAR NDT 匹配结果为基准，叠加里程计增量推算当前位姿（`LocalizationPoseBuffer`）
- NDT 匹配质量通过 fitness score 监控，连续坏分数超过阈值（默认 10 次）则判定定位失效

---

## 数据流

### RTK 数据流

```
/apollo/sensor/gnss/odometry (Gps)
    │
    ▼
RTKLocalizationComponent::Proc()
    │
    ├── /apollo/sensor/gnss/corrected_imu (CorrectedImu) ──→ IMU 缓冲区
    ├── /apollo/sensor/gnss/ins_stat (InsStat) ──→ GPS 状态缓冲区
    │
    ▼
RTKLocalization::GpsCallback()
    ├── FindMatchingIMU() → InterpolateIMU()
    ├── ComposeLocalizationMsg()
    └── FillLocalizationStatusMsg()
         │
         ▼
    /apollo/localization/pose (LocalizationEstimate)
    /apollo/localization/msf_status (LocalizationStatus)
    TF: world → localization
```

### MSF 数据流

```
/apollo/sensor/gnss/imu (Imu, 100Hz+)
    │
    ▼
MSFLocalizationComponent::Proc()
    │
    ▼
MSFLocalization::OnRawImuCache()  ← 仅缓存 IMU 数据
    │
    由 10ms 定时器 OnLocalizationTimer 驱动：
    ▼
MSFLocalization::OnRawImu()
    ├── TransferImuRfu/Flu() → RawImuProcessRfu()
    │       │
    │       ▼
    │   LocalizationIntegImpl::ImuProcessImpl()
    │       ├── LocalizationIntegProcess::RawImuProcess() ← SINS 递推
    │       ├── LocalizationLidarProcess::RawImuProcess() ← 位姿预测更新
    │       └── OnlineLocalizationExpert::AddImu()
    │
    ├── /apollo/sensor/lidar128/.../PointCloud2 ──→ OnPointCloud()
    │       │
    │       ▼
    │   LocalizationIntegImpl::PcdProcessImpl()
    │       ├── LocalizationLidarProcess::PcdProcess() ← LiDAR 地图匹配
    │       ├── MeasureRepublishProcess::LidarLocalProcess() → MeasureData
    │       └── LocalizationIntegProcess::MeasureDataProcess() ← 量测更新
    │
    ├── /apollo/sensor/gnss/best_pose ──→ OnGnssBestPose()
    │       │
    │       ▼
    │   LocalizationIntegImpl::GnssBestPoseProcessImpl()
    │       ├── MeasureRepublishProcess::NovatelBestgnssposProcess() → MeasureData
    │       └── LocalizationIntegProcess::MeasureDataProcess() ← 量测更新
    │
    └── /apollo/sensor/gnss/heading ──→ OnGnssHeading()
            │
            ▼
        MeasureRepublishProcess::GnssHeadingProcess() → MeasureData
            │
            ▼
    输出（由定时器 OnLocalizationTimer 驱动）：
        /apollo/localization/pose (LocalizationEstimate) ← 融合结果
        /apollo/localization/msf_lidar (LocalizationEstimate) ← LiDAR 子结果
        /apollo/localization/msf_gnss (LocalizationEstimate) ← GNSS 子结果
        /apollo/localization/msf_status (LocalizationStatus)
        TF: world → localization
```

### NDT 数据流

```
/apollo/sensor/gnss/odometry (Gps)
    │
    ▼
NDTLocalizationComponent::Proc()
    │
    ├── /apollo/sensor/lidar/.../PointCloud2 ──→ LidarCallback()
    │       │
    │       ▼
    │   NDTLocalization::LidarCallback()
    │       ├── LidarMsgTransfer() → LidarFrame
    │       ├── QueryPoseFromTF() / QueryPoseFromBuffer() ← 预测位姿
    │       ├── LidarLocatorNdt::Update() ← 点云投影
    │       └── NormalDistributionsTransform::Align() ← NDT 配准
    │
    ├── /apollo/sensor/gnss/ins_stat ──→ OdometryStatusCallback()
    │
    ▼
NDTLocalization::OdometryCallback()
    ├── 里程计缓冲区更新
    ├── LocalizationPoseBuffer::UpdateOdometryPose() ← 帧间推算
    └── ComposeLocalizationEstimate()
         │
         ▼
    /apollo/localization/pose (LocalizationEstimate)
    /apollo/localization/ndt_lidar (LocalizationEstimate) ← LiDAR 匹配结果
    /apollo/localization/msf_status (LocalizationStatus)
    TF: world → localization
```

---

## 配置方式

### 定位方案选择

文件：`conf/localization_config.pb.txt`

```protobuf
localization_type: MSF    # RTK 或 MSF
```

### 启动方式

每种方案对应独立的 launch 文件和 DAG 文件：

| 方案 | Launch 文件 | DAG 文件 | 主触发 Channel |
|------|------------|---------|---------------|
| RTK | `launch/rtk_localization.launch` | `dag/dag_streaming_rtk_localization.dag` | `/apollo/sensor/gnss/odometry` |
| MSF | `launch/msf_localization.launch` | `dag/dag_streaming_msf_localization.dag` | `/apollo/sensor/gnss/imu` |
| NDT | `launch/ndt_localization.launch` | `dag/dag_streaming_ndt_localization.dag` | `/apollo/sensor/gnss/odometry` |

### RTK 配置

文件：`conf/rtk_localization.pb.txt`（对应 `proto/rtk_config.proto`）

```protobuf
localization_topic: "/apollo/localization/pose"
localization_status_topic: "/apollo/localization/msf_status"
imu_topic: "/apollo/sensor/gnss/corrected_imu"
gps_topic: "/apollo/sensor/gnss/odometry"
gps_status_topic: "/apollo/sensor/gnss/ins_stat"
broadcast_tf_frame_id: "world"
broadcast_tf_child_frame_id: "localization"
imu_frame_id: "imu"
imu_list_max_size: 20
gps_imu_time_diff_threshold: 0.02
map_offset_x: 0.0
map_offset_y: 0.0
map_offset_z: 0.0
```

### MSF 配置

MSF 的 Topic 配置在 `conf/msf_localization.pb.txt`：

```protobuf
localization_topic: "/apollo/localization/pose"
imu_topic: "/apollo/sensor/gnss/imu"
lidar_topic: "/apollo/sensor/lidar128/compensator/PointCloud2"
bestgnsspos_topic: "/apollo/sensor/gnss/best_pose"
gnss_heading_topic: "/apollo/sensor/gnss/heading"
broadcast_tf_frame_id: "world"
broadcast_tf_child_frame_id: "localization"
map_dir: "/apollo/modules/map/data/sunnyvale_big_loop"
lidar_localization_topic: "/apollo/localization/msf_lidar"
gnss_localization_topic: "/apollo/localization/msf_gnss"
```

算法参数通过 GFlags 配置，文件：`conf/localization.conf`，关键参数分组：

**LiDAR 定位参数：**
- `--enable_lidar_localization=true`：启用 LiDAR 定位
- `--lidar_localization_mode=2`：定位模式（0=强度，1=高度，2=融合）
- `--lidar_yaw_align_mode=2`：航向对齐模式（0=关闭，1=融合，2=多线程融合）
- `--lidar_filter_size=17`：LiDAR 滤波器尺寸
- `--lidar_map_coverage_theshold=0.9`：点云与地图有效覆盖率阈值（GFlags 默认 0.9，配置文件中覆盖为 0.8）
- `--lidar_height_default=1.80`：LiDAR 距地面默认高度（米）
- `--if_use_avx=true`：启用 AVX 指令集加速

**SINS 惯导参数：**
- `--integ_ins_can_self_align=false`：INS 是否可自对准
- `--integ_sins_align_with_vel=true`：使用速度辅助对准
- `--integ_sins_state_check=true`：启用 SINS 状态检查（异常时重启）
- `--integ_sins_state_span_time=60.0`：状态检查时间窗口（秒）（GFlags 默认 60.0，配置文件中覆盖为 30.0）
- `--integ_sins_state_pos_std=1.0`：位置标准差阈值

**GNSS 参数：**
- `--gnss_mode=0`：GNSS 模式（0=NovAtel BestPos，1=自主解算）
- `--enable_ins_aid_rtk=false`：启用 INS 辅助 RTK
- `--imu_to_ant_offset_x/y/z`：IMU 到 GNSS 天线杆臂偏移
- `--imu_to_ant_offset_ux/uy/uz`：杆臂偏移不确定度
- `--imu_coord_rfu=true`：IMU 坐标系（true=RFU，false=FLU）

### NDT 特有参数（GFlags）

- `--ndt_map_dir`：NDT 地图目录
- `--online_resolution`：在线点云分辨率（默认 2.0）
- `--ndt_max_iterations`：最大迭代次数
- `--ndt_target_resolution`：目标体素分辨率
- `--ndt_line_search_step_size`：线搜索步长
- `--ndt_transformation_epsilon`：变换收敛阈值
- `--ndt_warnning_ndt_score`：警告得分阈值（默认 1.0）
- `--ndt_error_ndt_score`：错误得分阈值（默认 2.0）

### 外参标定文件（YAML）

| 文件 | 用途 |
|------|------|
| `msf/params/velodyne_params/velodyne64_novatel_extrinsics_example.yaml` | LiDAR-IMU 外参（平移 + 四元数旋转） |
| `msf/params/gnss_params/ant_imu_leverarm.yaml` | GNSS 天线-IMU 杆臂（主/副天线偏移及不确定度） |
| `msf/params/vehicle_params/vehicle_imu_extrinsics.yaml` | 车体-IMU 外参（四元数旋转 + 平移） |
| `msf/params/velodyne_params/velodyne64_height.yaml` | LiDAR 距地面高度 |
