# V2X 模块技术文档

## 模块概述

V2X（Vehicle to Everything）模块负责实现车辆与外部环境之间的信息交换。该模块通过 gRPC 协议与车载单元（OBU, On-Board Unit）通信，接收来自路侧单元（RSU, Road-Side Unit）的交通灯信息、道路安全信息（RSI）和障碍物数据，并将这些数据转换为 Apollo 内部消息格式，供感知、规划等下游模块使用。同时，模块还将车辆自身状态（定位、底盘信息等）上报给 OBU。

V2X 模块由两个核心子系统组成：

- **V2X Proxy**：作为独立二进制进程运行，负责与 OBU 之间的 gRPC 双向通信，以及与 Apollo Cyber RT 系统之间的消息桥接。
- **V2X Fusion**：作为 Cyber RT Component 运行，将 V2X 障碍物数据与车载感知障碍物数据进行融合，输出统一的感知结果。

## 目录结构

```
modules/v2x/
├── BUILD                    # Bazel 构建文件
├── README.md
├── common/                  # 公共 gflags 定义
│   ├── v2x_proxy_gflags.h
│   └── v2x_proxy_gflags.cc
├── conf/                    # 配置文件
│   ├── v2x.conf             # V2X Proxy 启动参数
│   └── v2x_fusion_tracker.conf  # Fusion 组件配置
├── cyberfile.xml            # Cyber 包描述
├── dag/                     # DAG 流水线定义
│   └── v2x_perception_fusion.dag
├── data/                    # 运行时数据/模型参数
│   └── fusion_params.pt     # 融合算法参数
├── fusion/                  # V2X 融合子系统
│   ├── apps/                # Fusion Component 入口
│   │   ├── common/          # 类型定义与转换工具
│   │   └── v2x_fusion_component.h/.cc
│   ├── configs/             # 融合配置管理
│   │   ├── ft_config_manager.h/.cc
│   │   └── fusion_tracker_gflags.h/.cc
│   ├── libs/                # 融合算法库
│   │   ├── common/v2x_object.h  # V2X 对象数据结构
│   │   └── fusion/
│   │       ├── fusion.h/.cc     # 融合主逻辑
│   │       └── km.h             # KM 匹配算法
│   └── test_data/           # 测试数据
├── launch/                  # 启动文件
│   └── v2x.launch
├── proto/                   # Protobuf 消息定义
│   ├── v2x_car_status.proto
│   ├── v2x_junction.proto
│   ├── v2x_monitor.proto
│   ├── v2x_obstacles.proto
│   ├── v2x_obu_rsi.proto
│   ├── v2x_obu_traffic_light.proto
│   ├── v2x_rsi.proto
│   ├── v2x_service_car_to_obu.proto
│   ├── v2x_service_obu_to_car.proto
│   ├── v2x_traffic_light_policy.proto
│   └── fusion_params.proto
└── v2x_proxy/               # V2X Proxy 子系统
    ├── app/                  # 主程序与工具函数
    │   ├── main.cc
    │   ├── v2x_proxy.h/.cc
    │   └── utils.h/.cc
    ├── obu_interface/        # OBU 通信接口
    │   ├── obu_interface_abstract_class.h  # 抽象基类
    │   ├── obu_interface_grpc_impl.h/.cc   # gRPC 实现
    │   └── grpc_interface/
    │       ├── grpc_client.h/.cc   # gRPC 客户端
    │       └── grpc_server.h/.cc   # gRPC 服务端
    ├── os_interface/         # Apollo 系统接口
    │   └── os_interface.h/.cc
    └── proto_adapter/        # 协议适配器
        └── proto_adapter.h/.cc
```

## 核心类与接口

### V2X Proxy 子系统

#### V2xProxy（主控类）

文件：`v2x_proxy/app/v2x_proxy.h`

`V2xProxy` 是 V2X Proxy 子系统的核心协调类，负责：

- 创建 Cyber RT 节点 `v2x_proxy`
- 加载 HD Map 用于交通灯信号匹配
- 管理多个工作线程和定时器
- 协调 OBU 接口与 OS 接口之间的数据流转

关键成员：

| 成员 | 类型 | 说明 |
|------|------|------|
| `os_interface_` | `OsInterFace` | Apollo 系统消息读写接口 |
| `obu_interface_` | `ObuInterFaceGrpcImpl` | OBU gRPC 通信接口 |
| `hdmap_` | `HDMap` | 高精地图实例，用于信号灯匹配 |
| `v2x_car_status_timer_` | `cyber::Timer` | 定时上报车辆状态到 OBU |
| `recv_thread_` | `std::thread` | 接收交通灯数据的工作线程 |
| `obs_thread_` | `std::thread` | 接收障碍物数据的工作线程 |
| `rsi_thread_` | `std::thread` | 接收 RSI 数据的工作线程 |
| `planning_thread_` | `std::thread` | 接收规划轨迹的工作线程 |
#### ObuInterFaceBase（OBU 接口抽象基类）

文件：`v2x_proxy/obu_interface/obu_interface_abstract_class.h`

定义了与 OBU 通信的抽象接口，支持以下操作：

- `GetV2xTrafficLightFromObu()` — 从 OBU 获取 V2X 交通灯数据
- `GetV2xObstaclesFromObu()` — 从 OBU 获取 V2X 障碍物数据
- `GetV2xRsiFromObu()` — 从 OBU 获取道路安全信息（RSI）
- `SendCarStatusToObu()` — 向 OBU 发送车辆状态

#### ObuInterFaceGrpcImpl（gRPC 实现）

文件：`v2x_proxy/obu_interface/obu_interface_grpc_impl.h`

继承 `ObuInterFaceBase`，通过 gRPC 实现与 OBU 的双向通信：

- **gRPC Client**（`GrpcClientImpl`）：调用 `CarToObu.PushCarStatus` RPC 将车辆状态推送给 OBU
- **gRPC Server**（`GrpcServerImpl`）：实现 `ObuToCar` 服务，接收 OBU 推送的交通灯、障碍物、RSI 和告警数据

#### OsInterFace（Apollo 系统接口）

文件：`v2x_proxy/os_interface/os_interface.h`

通过 Cyber RT 节点 `v2x_os_interface` 与 Apollo 系统交互：

**读取（Reader）：**

| Channel | 消息类型 | 说明 |
|---------|---------|------|
| `FLAGS_localization_topic` | `LocalizationEstimate` | 车辆定位信息 |
| `FLAGS_planning_trajectory_topic` | `ADCTrajectory` | 规划轨迹 |

**写入（Writer）：**

| Channel | 消息类型 | 说明 |
|---------|---------|------|
| `FLAGS_v2x_obu_traffic_light_topic` | `obu::ObuTrafficLight` | OBU 原始交通灯数据 |
| `FLAGS_v2x_traffic_light_topic` | `IntersectionTrafficLightData` | 转换后的交通灯数据 |
| `FLAGS_v2x_traffic_light_for_hmi_topic` | `TrafficLightDetection` | 供 HMI 显示的交通灯数据 |
| `FLAGS_v2x_internal_obstacle_topic` | `V2XObstacles` | V2X 障碍物数据 |

#### ProtoAdapter（协议适配器）

文件：`v2x_proxy/proto_adapter/proto_adapter.h`

纯静态工具类，负责 OBU 消息格式与 Apollo 内部消息格式之间的转换：

- `LightObu2Sys()` — OBU 交通灯 → Apollo `IntersectionTrafficLightData`
- `RsiObu2Sys()` — OBU RSI → Apollo `RsiMsg`
- `JunctionHd2obu()` — HD Map Junction → OBU `Junction`
- `LightTypeObu2Sys()` — 交通灯类型映射（直行/左转/右转/掉头）
#### InternalData（内部数据处理）

文件：`v2x_proxy/app/utils.h`

封装了交通灯和规划消息的核心处理逻辑：

- `ProcTrafficlight()` — 处理从 OBU 接收的交通灯数据，结合 HD Map 进行信号灯 ID 匹配，校正剩余时间，并根据高峰时段设置置信度（高峰期 0.5，非高峰期 1.0）
- `ProcPlanningMessage()` — 将 V2X 交通灯信息与规划模块的信号灯需求进行匹配，生成供 HMI 显示的 `TrafficLightDetection` 消息
- `TrafficLightProc()` — 利用 HD Map 的 `GetForwardNearestSignalsOnLane` 接口，将 GPS 坐标的交通灯映射到地图中的信号灯 ID

#### utils 命名空间工具函数

文件：`v2x_proxy/app/utils.h`

- `GetRsuInfo()` — 根据车辆定位，通过 HD Map 查找前方最近的 RSU，验证白名单，获取路口信息
- `FindAllRoadId()` — 沿车道后继关系查找起止车道之间的所有道路 ID
- `CheckCarInSet()` — 检查车辆当前所在车道是否在指定道路集合中
- `UniqueOslight()` — 对交通灯数据去重（按信号灯 ID + 类型）
- `GetNextColor()` — 获取交通灯下一个颜色状态（绿→黄→红→绿）

### V2X Fusion 子系统

#### V2XFusionComponent

文件：`fusion/apps/v2x_fusion_component.h`

继承 `cyber::Component<PerceptionObstacles>`，以车载感知障碍物消息作为触发输入。通过 `FLAGS_use_v2x` 开关控制是否启用 V2X 融合：

- 启用时：读取 V2X 障碍物和定位数据，与车载感知结果进行融合
- 禁用时：直接透传车载感知结果

**输入 Channel：**

| Channel | 消息类型 | 说明 |
|---------|---------|------|
| `/perception/vehicle/obstacles` | `PerceptionObstacles` | 车载感知障碍物（触发输入） |
| `FLAGS_v2x_obstacle_topic` | `V2XObstacles` | V2X 障碍物 |
| `FLAGS_localization_topic` | `LocalizationEstimate` | 车辆定位 |

**输出 Channel：**

| Channel | 消息类型 | 说明 |
|---------|---------|------|
| `FLAGS_perception_obstacle_topic` | `PerceptionObstacles` | 融合后的感知障碍物 |

#### Fusion（融合算法）

文件：`fusion/libs/fusion/fusion.h`

基于 KM（Kuhn-Munkres / 匈牙利）算法实现障碍物关联匹配与融合：

1. **关联矩阵计算**（`ComputeAssociateMatrix`）：对已融合对象和新输入对象两两计算匹配分数
   - 距离分数：`2.5 * max(0, max_match_distance - 欧氏距离)`
   - 类型分数（可选）：基于对象子类型概率计算相似度
2. **KM 匹配**（`KMkernal::GetKMResult`）：使用 KM 算法求解最优二部图匹配
3. **融合结果生成**（`GetV2xFusionObjects`）：
   - 仅有单一来源的对象：若来自 V2X 则标记为 `BLIND_ZONE`（盲区目标）
   - 多来源匹配的对象：检查是否包含 `HOST_VEHICLE` 或 `ZOMBIES_CAR` 类型并标记

融合参数通过 `fusion_params.pt` 配置，关键参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_match_distance` | 10 (米) | 最大匹配距离 |
| `min_score` | 0 | 最小匹配分数阈值 |
| `prob_scale` | 0.125 | 类型概率缩放因子 |
| `confidence_level` | C99P | 置信度水平（用于马氏距离阈值） |
| `check_type` | false | 是否启用类型匹配 |
| `use_mahalanobis_distance` | true | 是否使用马氏距离 |

## 通信协议

### gRPC 服务定义

V2X Proxy 通过两个 gRPC 服务实现与 OBU 的双向通信：

#### CarToObu（车端 → OBU）

定义文件：`proto/v2x_service_car_to_obu.proto`

```protobuf
service CarToObu {
  rpc PushCarStatus(CarStatus) returns (UpdateStatus) {}
}
```

车辆定期（默认 10Hz）将自身状态推送给 OBU，包含：
- 定位信息（`LocalizationEstimate`）
- 底盘信息（`ChassisDetail`）
- 当前路口信息（`Junction`，从 HD Map 获取）

#### ObuToCar（OBU → 车端）

定义文件：`proto/v2x_service_obu_to_car.proto`

```protobuf
service ObuToCar {
  rpc SendPerceptionObstacles(V2XObstacles) returns (StatusResponse) {}
  rpc SendV2xTrafficLight(ObuTrafficLight) returns (StatusResponse) {}
  rpc SendV2xRSI(ObuRsi) returns (StatusResponse) {}
  rpc SendObuAlarm(ObuAlarm) returns (StatusResponse) {}
}
```

OBU 通过此服务向车端推送：
- V2X 障碍物数据（来自 RSU 的感知结果）
- 交通灯信号与配时信息（SPAT）
- 道路安全信息（RSI）
- OBU 设备告警

## 支持的消息类型

### V2I（Vehicle to Infrastructure）

V2I 是该模块的主要通信场景，涉及车辆与路侧基础设施（RSU）之间的信息交换：

#### 交通灯信号（SPAT）

- Proto 定义：`v2x_obu_traffic_light.proto`（OBU 侧）、`v2x_traffic_light_policy.proto`（策略侧）
- 数据层级：`ObuTrafficLight` → `RoadTrafficLight` → `LaneTrafficLight` → `SingleTrafficLight`
- 包含信息：灯色（红/黄/绿/闪绿/黑）、剩余时间、下一灯色及剩余时间、灯型（直行/左转/右转/掉头）、道路方向（东/西/南/北）
- 处理流程：OBU 原始数据 → `ProtoAdapter::LightObu2Sys` 转换 → HD Map 信号灯 ID 匹配 → 去重 → 发布到 Cyber RT

#### 道路安全信息（RSI）

- Proto 定义：`v2x_obu_rsi.proto`（OBU 侧）、`v2x_rsi.proto`（系统侧）
- 支持的 RSI 类型（`RsiAlterType` 枚举）：

| 类型 | 编码 | 说明 |
|------|------|------|
| `SPEED_LIMIT` | 85 | 限速 |
| `SPEED_LIMIT_BRIDGE` | 8 | 桥梁限速 |
| `SPEED_LIMIT_TUNNEL` | 21 | 隧道限速 |
| `CONSTRUCTION_AHEAD` | 38 | 前方施工 |
| `BUS_LANE` | 123 | 公交车道 |
| `TIDAL_LANE` | 41 | 潮汐车道 |
| `TRAFFIC_JAM` | 47 | 交通拥堵 |
| `TRAFFIC_ACCIDENT` | 244 | 交通事故 |
| `NO_HONKING` | 80 | 禁止鸣笛 |
| `SLOW_DOWN_SECTION` | 35 | 减速路段 |
| `ACCIDENT_PRONE` | 34 | 事故多发路段 |
| `OVERSPEED_VEHICLE` | 801 | 超速车辆 |
| `EMERGENCY_BRAKING` | 802 | 紧急制动 |
| `ANTIDROMIC_VEHICLE` | 803 | 逆行车辆 |
| `ZOMBIES_VEHICLE` | 804 | 僵尸车辆（异常停驶） |
| `CONTROLLOSS_VEHICLE` | 1000 | 失控车辆 |
| `SPECIAL_VEHICLE` | 2000 | 特种车辆 |

#### V2X 障碍物

- Proto 定义：`v2x_obstacles.proto`
- `V2XObstacle` 包含标准 `PerceptionObstacle` 和 V2X 扩展信息（`V2XInformation`）
- V2X 扩展类型：`ZOMBIES_CAR`（僵尸车）、`BLIND_ZONE`（盲区目标）
- 附带区域地图信息（`MiniAreaMap`）和交通流量数据

### V2V（Vehicle to Vehicle）

当前代码中 V2V 场景主要体现在 V2X 障碍物数据中，通过 `V2XInformation.V2XType` 标识：
- `ZOMBIES_CAR` — 僵尸车辆预警
- `BLIND_ZONE` — 盲区车辆提醒

这些信息由 RSU 汇聚后通过 OBU 转发给车端，本质上仍是 V2I 链路承载的 V2V 语义。

### OBU 监控告警

- Proto 定义：`v2x_monitor.proto`
- 告警类型（`ErrorCode` 枚举）：LTEV（LTE-V 通信）、NET（网络）、CPU、MEM（内存）、GPS、MAP（地图）、SPAT（信号灯）、OBUID（设备标识）

## 数据流

### 整体数据流架构

```
                    ┌─────────────────────────────────────────────┐
                    │              V2X Proxy 进程                  │
                    │                                             │
  RSU/OBU           │  ┌──────────┐    ┌──────────┐              │    Apollo Cyber RT
  ─────────────────►│  │  gRPC    │───►│ V2xProxy │              │
  (gRPC:50101)      │  │  Server  │    │  主控类   │              │
                    │  └──────────┘    └────┬─────┘              │
                    │                       │                     │
                    │  ┌──────────┐    ┌────▼──────┐             │
                    │  │  gRPC    │◄───│   Proto   │             │
  RSU/OBU           │  │  Client  │    │  Adapter  │             │
  ◄─────────────────│  └──────────┘    └────┬──────┘             │
  (gRPC:50100)      │                       │                     │
                    │                  ┌────▼──────┐             │
                    │                  │    OS     │────────────►│──► Cyber RT Topics
                    │                  │ Interface │◄────────────│◄── Cyber RT Topics
                    │                  └───────────┘             │
                    └─────────────────────────────────────────────┘

                    ┌─────────────────────────────────────────────┐
                    │         V2X Fusion Component                │
                    │                                             │
  /perception/      │  ┌───────────┐   ┌─────────┐              │
  vehicle/obstacles │  │ V2XFusion │   │  KM     │              │  /perception/
  ─────────────────►│  │ Component │──►│ Matcher │──────────────►│──► obstacles
                    │  └─────┬─────┘   └─────────┘              │  (融合结果)
  v2x_obstacles     │        │                                    │
  ─────────────────►│────────┘                                    │
  localization      │                                             │
  ─────────────────►│                                             │
                    └─────────────────────────────────────────────┘
```

### 交通灯数据流（详细）

1. **OBU → gRPC Server**：OBU 调用 `ObuToCar.SendV2xTrafficLight` 推送 `ObuTrafficLight` 消息
2. **gRPC Server → V2xProxy**：`recv_thread_` 线程通过 `GetV2xTrafficLightFromObu()` 阻塞等待新数据
3. **原始数据发布**：通过 `OsInterFace::SendV2xObuTrafficLightToOs()` 将 OBU 原始数据发布到 Cyber RT
4. **协议转换**：`ProtoAdapter::LightObu2Sys()` 将 OBU 格式转换为 Apollo 内部格式
5. **HD Map 匹配**：`InternalData::TrafficLightProc()` 利用 HD Map 将 GPS 坐标映射到地图信号灯 ID
6. **路口校验**：验证接收到的 junction_id 与车辆当前所在路口一致
7. **时间校正**：对同一路口的连续消息，校正剩余时间（防止时间跳变）
8. **去重发布**：`utils::UniqueOslight()` 去重后发布到 `v2x_traffic_light_topic`
9. **HMI 适配**：`planning_thread_` 线程结合规划模块的信号灯需求，生成 `TrafficLightDetection` 发布到 HMI topic

### 障碍物数据流

1. **OBU → gRPC Server**：OBU 调用 `ObuToCar.SendPerceptionObstacles` 推送 `V2XObstacles`
2. **gRPC Server → V2xProxy**：`obs_thread_` 线程通过 `GetV2xObstaclesFromObu()` 阻塞等待
3. **发布到 Cyber RT**：通过 `OsInterFace::SendV2xObstacles2Sys()` 发布到 `v2x_internal_obstacle_topic`
4. **V2X Fusion Component**：
   - 以车载感知障碍物 `/perception/vehicle/obstacles` 为触发
   - 读取最新的 V2X 障碍物和定位数据
   - 将车载感知对象和 V2X 对象分别转换为内部 `Object` 格式
   - 通过 KM 算法进行关联匹配
   - 对匹配结果标记 V2X 类型（`BLIND_ZONE`、`ZOMBIES_CAR` 等）
   - 序列化为 `PerceptionObstacles` 发布

### 车辆状态上报流程

1. **定时触发**：`v2x_car_status_timer_` 以 10Hz 频率触发 `OnV2xCarStatusTimer()`
2. **获取定位**：从 `OsInterFace` 读取最新定位数据
3. **RSU 查找**：通过 HD Map 的 `GetForwardNearestRSUs()` 查找前方 RSU
4. **白名单校验**：验证 RSU ID 是否在白名单中（从 `rsu_whitelist.txt` 加载）
5. **路口信息获取**：从 HD Map 获取 RSU 关联的路口信息，转换为 OBU 格式
6. **状态推送**：通过 gRPC Client 调用 `CarToObu.PushCarStatus` 发送给 OBU

## 配置方式

### V2X Proxy 配置（`conf/v2x.conf`）

通过 gflags 命令行参数配置，启动时通过 `--flagfile` 加载：

```
--x2v_traffic_light_timer_frequency=10
--v2x_car_status_timer_frequency=10
--grpc_client_host=192.168.10.21
--grpc_server_host=192.168.10.6
--hdmap_file_name=/apollo/modules/map/data/sunnyvale_big_loop/base_map.bin
--rsu_whitelist_name=/apollo/modules/v2x/conf/rsu_whitelist.txt
```

完整 gflags 参数列表：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grpc_client_host` | `127.0.0.1` | gRPC 客户端目标地址（OBU 地址） |
| `grpc_client_port` | `50100` | gRPC 客户端目标端口 |
| `grpc_server_host` | `127.0.0.1` | gRPC 服务端监听地址 |
| `grpc_server_port` | `50101` | gRPC 服务端监听端口 |
| `grpc_debug_server_port` | `50102` | gRPC 调试端口 |
| `x2v_traffic_light_timer_frequency` | `10` (Hz) | 交通灯定时器频率 |
| `v2x_car_status_timer_frequency` | `10` (Hz) | 车辆状态上报频率 |
| `traffic_light_distance` | `250.0` (米) | 交通灯检测距离 |
| `heading_difference` | `30.0/180.0` (弧度) | 最大航向角差异 |
| `msg_timeout` | `250` (毫秒) | OBU 消息超时时间 |
| `check_time` | `0.5` (秒) | SPAT 消息校验时间 |
| `rsu_whitelist_period` | `3000` (毫秒) | RSU 白名单刷新周期 |
| `rsu_whitelist_name` | `/apollo/modules/v2x/conf/rsu_whitelist.txt` | RSU 白名单文件路径 |
| `use_nearest_flag` | `true` | 是否使用最近信号灯匹配 |
| `spat_period` | `150` (毫秒) | SPAT 消息周期 |

### V2X Fusion 配置（`conf/v2x_fusion_tracker.conf`）

```
--config_path=/apollo/modules/v2x/data
--fusion_conf_file=fusion_params.pt
--input_conf_file=app_config.pt
```

融合算法参数文件 `data/fusion_params.pt`：

```protobuf
score_params {
  prob_scale: 0.125
  max_match_distance: 10
  min_score: 0
  use_mahalanobis_distance: true
  check_type: false
  confidence_level: C99P
}
```

### 启动方式

V2X Proxy 通过 `v2x.launch` 启动：

```bash
cyber_launch start modules/v2x/launch/v2x.launch
```

启动文件将 V2X Proxy 作为独立二进制进程运行：

```xml
<cyber>
    <module>
        <name>v2x</name>
        <type>binary</type>
        <process_name>
           /apollo/bazel-bin/modules/v2x/v2x_proxy/app/v2x
           --flagfile=/apollo/modules/v2x/conf/v2x.conf
        </process_name>
    </module>
</cyber>
```

V2X Fusion 通过 DAG 文件 `dag/v2x_perception_fusion.dag` 配置：

```
module_config {
    module_library : "modules/v2x/libv2x_fusion_component.so"
    components {
      class_name : "V2XFusionComponent"
      config {
        name : "v2x_fusion"
        flag_file_path : "/apollo/modules/v2x/conf/v2x_fusion_tracker.conf"
        readers: [
          { channel: "/perception/vehicle/obstacles" }
        ]
      }
    }
}
```
