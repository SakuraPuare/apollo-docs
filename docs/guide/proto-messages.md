# Protobuf 消息定义

## 概述

Apollo 自动驾驶平台采用 Protocol Buffers（protobuf）作为模块间通信的数据序列化格式。所有公共消息定义集中在 `modules/common_msgs/` 目录下，按功能领域划分为多个子目录，每个子目录包含一组 `.proto` 文件。

`common_msgs` 的核心作用：

- **统一接口契约**：为各模块（感知、规划、控制等）定义标准化的数据结构，确保模块间通信的一致性
- **解耦模块依赖**：各模块只需依赖 `common_msgs` 中的消息定义，而不需要直接依赖其他模块的内部实现
- **支持 Cyber RT 通信**：所有通过 Cyber RT 框架发布/订阅的 channel 消息均基于这些 proto 定义

目录结构概览：

```
modules/common_msgs/
├── audio_msgs/              # 音频相关消息
├── basic_msgs/              # 基础通用消息（Header、几何类型、错误码等）
├── chassis_msgs/            # 底盘状态消息
├── config_msgs/             # 车辆配置消息
├── control_msgs/            # 控制指令消息
├── dreamview_msgs/          # 可视化与 HMI 消息
├── drivers_msgs/            # 驱动参数消息
├── external_command_msgs/   # 外部命令消息
├── guardian_msgs/           # 安全守护消息
├── localization_msgs/       # 定位消息
├── map_msgs/                # 高精地图消息
├── monitor_msgs/            # 系统监控消息
├── perception_msgs/         # 感知消息
├── planning_msgs/           # 规划消息
├── prediction_msgs/         # 预测消息
├── routing_msgs/            # 路由消息
├── sensor_msgs/             # 传感器数据消息
├── storytelling_msgs/       # 场景叙事消息
├── task_manager_msgs/       # 任务管理消息
├── transform_msgs/          # 坐标变换消息
└── v2x_msgs/                # V2X 通信消息
```

## 按领域分类

### 基础消息（basic_msgs）

基础消息定义了整个系统中最通用的数据结构，几乎所有其他消息都会引用这些定义。

**包名**：`apollo.common`

**Proto 文件**：

| 文件 | 说明 |
|------|------|
| `header.proto` | 消息头，包含时间戳、模块名、序列号等 |
| `geometry.proto` | 几何基础类型（点、四元数、多边形等） |
| `error_code.proto` | 全局错误码枚举 |
| `pnc_point.proto` | PnC（规划与控制）路径点 |
| `vehicle_id.proto` | 车辆标识 |
| `vehicle_signal.proto` | 车辆信号（转向灯、喇叭等） |
| `direction.proto` | 方向枚举 |
| `drive_event.proto` | 驾驶事件 |
| `drive_state.proto` | 驾驶状态 |

#### Header

`Header` 是最核心的基础消息，几乎所有模块发布的消息都包含该字段：

```protobuf
message Header {
  optional double timestamp_sec = 1;    // 消息时间戳（秒）
  optional string module_name = 2;      // 发布模块名称
  optional uint32 sequence_num = 3;     // 消息序列号
  optional uint64 lidar_timestamp = 4;  // 激光雷达时间戳
  optional uint64 camera_timestamp = 5; // 相机时间戳
  optional uint64 radar_timestamp = 6;  // 毫米波雷达时间戳
  optional uint32 version = 7;          // 消息版本号
  optional StatusPb status = 8;         // 状态信息
  optional string frame_id = 9;        // 帧标识
}
```

#### 几何类型

```protobuf
// 东北天坐标系下的三维点
message PointENU {
  optional double x = 1;  // 东向坐标（米）
  optional double y = 2;  // 北向坐标（米）
  optional double z = 3;  // 天向坐标（米）
}

// 经纬高坐标
message PointLLH {
  optional double lon = 1;    // 经度
  optional double lat = 2;    // 纬度
  optional double height = 3; // 高度
}

message Point2D { ... }
message Point3D { ... }
message Quaternion { ... }  // 四元数，用于表示旋转
message Polygon { ... }     // 多边形
```

#### 错误码

```protobuf
enum ErrorCode {
  OK = 0;
  CONTROL_ERROR = 1000;
  CANBUS_ERROR = 2000;
  LOCALIZATION_ERROR = 3000;
  PERCEPTION_ERROR = 4000;
  // ... 各模块错误码按千位段划分
}
```

---

### 底盘消息（chassis_msgs）

底盘消息描述车辆底盘的实时状态，是控制模块的重要输入。

**包名**：`apollo.canbus`

**Proto 文件**：`chassis.proto`、`chassis_detail.proto`

#### Chassis

```protobuf
message Chassis {
  optional bool engine_started = 3;
  optional float engine_rpm = 4;
  optional float speed_mps = 5;           // 车速（米/秒）
  optional float odometer_m = 6;          // 里程表（米）
  optional int32 fuel_range_m = 7;        // 续航里程（米）
  optional float throttle_percentage = 8; // 油门百分比
  optional float brake_percentage = 9;    // 刹车百分比
  optional float steering_percentage = 11; // 方向盘转角百分比
  optional float steering_torque_nm = 12;  // 方向盘扭矩
  optional bool parking_brake = 13;       // 驻车制动
  optional DrivingMode driving_mode = 14;
  optional ErrorCode error_code = 15;
  optional GearPosition gear_location = 16;
  optional Header header = 17;
  // ... signal, wheel_speed, battery_soc_percentage 等
}
```

#### 驾驶模式与档位

```protobuf
enum DrivingMode {
  COMPLETE_MANUAL = 0;       // 完全手动
  COMPLETE_AUTO_DRIVE = 1;   // 完全自动驾驶
  AUTO_STEER_ONLY = 2;       // 仅自动转向
  AUTO_SPEED_ONLY = 3;       // 仅自动速度控制
  EMERGENCY_MODE = 4;        // 紧急模式
}

enum GearPosition {
  GEAR_NEUTRAL = 0;
  GEAR_DRIVE = 1;
  GEAR_REVERSE = 2;
  GEAR_PARKING = 3;
  GEAR_LOW = 4;
  GEAR_INVALID = 5;
  GEAR_NONE = 6;
}
```

---

### 控制消息（control_msgs）

控制消息定义了自动驾驶系统向底盘发送的控制指令。

**包名**：`apollo.control`

**Proto 文件**：`control_cmd.proto`、`control_interactive_msg.proto`、`input_debug.proto`、`pad_msg.proto`

#### ControlCommand

```protobuf
message ControlCommand {
  optional Header header = 1;
  optional double throttle = 3;         // 油门指令
  optional double brake = 4;            // 刹车指令
  optional double steering_rate = 6;    // 方向盘转速
  optional double steering_target = 7;  // 方向盘目标角度
  optional bool parking_brake = 8;      // 驻车制动
  optional double speed = 9;            // 目标速度
  optional double acceleration = 10;    // 目标加速度
  optional GearPosition gear_location = 20;
  optional Debug debug = 22;
  optional Signal signal = 23;
  optional LatencyStats latency_stats = 24;
  optional PadMessage pad_msg = 25;
  optional EngageAdvice engage_advice = 26;
  optional bool is_in_safe_mode = 27;
  optional TurnSignal turnsignal = 21;
}
```

---

### 定位消息（localization_msgs）

定位消息提供车辆在世界坐标系中的精确位置和姿态信息。

**包名**：`apollo.localization`

**Proto 文件**：`localization.proto`、`pose.proto`、`gps.proto`、`imu.proto`、`localization_status.proto`

#### LocalizationEstimate

```protobuf
message LocalizationEstimate {
  optional Header header = 1;
  optional Pose pose = 2;                    // 车辆位姿
  optional Uncertainty uncertainty = 3;      // 不确定度
  optional double measurement_time = 4;      // 测量时间
  repeated TrajectoryPoint trajectory_point = 5;
  optional MsfStatus msf_status = 6;          // 多传感器融合状态
  optional MsfSensorMsgStatus sensor_status = 7; // 传感器消息状态
}
```

#### Pose

```protobuf
message Pose {
  optional PointENU position = 1;           // 位置
  optional Quaternion orientation = 2;      // 姿态（四元数）
  optional Point3D linear_velocity = 3;     // 线速度
  optional Point3D linear_acceleration = 4; // 线加速度
  optional Point3D angular_velocity = 5;    // 角速度
  optional double heading = 6;              // 航向角
  optional Point3D linear_acceleration_vrf = 7;  // 车体坐标系线加速度
  optional Point3D angular_velocity_vrf = 8;     // 车体坐标系角速度
  optional Point3D euler_angles = 9;        // 欧拉角
}
```

---

### 感知消息（perception_msgs）

感知消息描述环境感知的结果，包括障碍物检测和交通信号灯识别。

**包名**：`apollo.perception`

**Proto 文件**：`perception_obstacle.proto`、`traffic_light_detection.proto`

#### PerceptionObstacles

```protobuf
message PerceptionObstacles {
  repeated PerceptionObstacle perception_obstacle = 1;
  optional Header header = 2;
  optional ErrorCode error_code = 3;
}

message PerceptionObstacle {
  optional int32 id = 1;                    // 障碍物唯一 ID
  optional Point3D position = 2;            // 位置
  optional double theta = 3;               // 朝向角
  optional Point3D velocity = 4;           // 速度
  optional double length = 5;              // 长度
  optional double width = 6;               // 宽度
  optional double height = 7;              // 高度
  repeated Point3D polygon_point = 8;      // 多边形轮廓点
  optional double tracking_time = 9;       // 跟踪时长
  optional Type type = 10;                 // 障碍物类型
  optional double timestamp = 11;
  repeated double point_cloud = 12 [packed = true];
  optional double confidence = 13;         // 置信度
  // ... acceleration, anchor_point, bbox2d, sub_type 等
}

enum Type {
  UNKNOWN = 0;
  UNKNOWN_MOVABLE = 1;
  UNKNOWN_UNMOVABLE = 2;
  PEDESTRIAN = 3;       // 行人
  BICYCLE = 4;          // 自行车
  VEHICLE = 5;          // 车辆
}
```

#### TrafficLightDetection

```protobuf
message TrafficLightDetection {
  optional Header header = 2;
  repeated TrafficLight traffic_light = 1;
  optional bool contain_lights = 4;
}

message TrafficLight {
  optional Color color = 1;       // RED, YELLOW, GREEN, BLACK, UNKNOWN
  optional string id = 2;         // 交通灯 ID
  optional double confidence = 3;
  optional double tracking_time = 4;
  optional bool blink = 5;        // 是否闪烁
  optional double remaining_time = 6; // 剩余时间
}
```

---

### 预测消息（prediction_msgs）

预测消息描述对感知障碍物未来运动轨迹的预测结果。

**包名**：`apollo.prediction`

**Proto 文件**：`prediction_obstacle.proto`、`feature.proto`、`prediction_point.proto`、`scenario.proto`

#### PredictionObstacles

```protobuf
message PredictionObstacles {
  optional Header header = 1;
  repeated PredictionObstacle prediction_obstacle = 2;
  optional ErrorCode perception_error_code = 3;
  optional double start_timestamp = 4;
  optional double end_timestamp = 5;
  optional Intent intent = 6;
  optional Scenario scenario = 7;
}

message PredictionObstacle {
  optional PerceptionObstacle perception_obstacle = 1; // 引用感知结果
  optional double timestamp = 2;
  optional double predicted_period = 3;     // 预测时间范围
  repeated Trajectory trajectory = 4;       // 预测轨迹（可能多条）
  optional ObstacleIntent intent = 5;               // 意图
  optional ObstaclePriority priority = 6;           // 优先级
  optional bool is_static = 7;              // 是否静止
}
```

---

### 规划消息（planning_msgs）

规划消息定义了自动驾驶的轨迹规划结果和规划指令。

**包名**：`apollo.planning`

**Proto 文件**：`planning.proto`、`planning_command.proto`、`planning_internal.proto`、`pad_msg.proto`、`decision.proto`、`sl_boundary.proto`

#### ADCTrajectory

`ADCTrajectory` 是规划模块的核心输出，包含车辆应遵循的轨迹：

```protobuf
message ADCTrajectory {
  optional Header header = 1;
  optional double total_path_length = 2;    // 总路径长度
  optional double total_path_time = 3;      // 总路径时间
  repeated TrajectoryPoint trajectory_point = 12; // 轨迹点序列
  optional EStop estop = 6;                 // 紧急停车
  repeated PathPoint path_point = 13;
  optional bool is_replan = 9;              // 是否重新规划
  optional apollo.planning_internal.Debug debug = 8; // 调试信息
  optional LatencyStats latency_stats = 15;
  optional RightOfWayStatus right_of_way_status = 17;
  optional DecisionResult decision = 14;   // 决策信息
  optional EngageAdvice engage_advice = 19;
  optional apollo.common.Header routing_header = 16;
  // ... critical_region, trajectory_type 等
}

message EStop {
  optional bool is_estop = 1;   // 是否紧急停车
  optional string reason = 2;   // 紧急停车原因
}
```

#### PlanningCommand

```protobuf
message PlanningCommand {
  optional Header header = 1;
  optional int64 command_id = 2;
  optional RoutingResponse lane_follow_command = 3;
  optional double target_speed = 4;
  optional bool is_motion_command = 5;
  optional ParkingCommand parking_command = 6;
  optional CustomCommand custom_command = 7;
}
```

---

### 路由消息（routing_msgs）

路由消息定义了全局路径规划的请求和响应。

**包名**：`apollo.routing`

**Proto 文件**：`routing.proto`、`geometry.proto`、`poi.proto`

#### RoutingRequest / RoutingResponse

```protobuf
message RoutingRequest {
  optional Header header = 1;
  repeated LaneWaypoint waypoint = 2;       // 途经点列表
  repeated LaneSegment blacklisted_lane = 3; // 黑名单车道
  repeated string blacklisted_road = 4;      // 黑名单道路
  optional bool broadcast = 5;
  optional ParkingInfo parking_info = 6;
  optional bool is_start_pose_set = 7;
}

message RoutingResponse {
  optional Header header = 1;
  repeated RoadSegment road = 2;                   // 路径道路序列
  optional Measurement measurement = 3;     // 路径度量
  optional RoutingRequest routing_request = 4;
  optional bytes map_version = 5;
  optional StatusPb status = 6;
}
```

---

### 地图消息（map_msgs）

地图消息定义了高精地图（HD Map）的完整数据结构。

**包名**：`apollo.hdmap`

**Proto 文件**：

| 文件 | 说明 |
|------|------|
| `map.proto` | 地图顶层结构 |
| `map_lane.proto` | 车道 |
| `map_road.proto` | 道路 |
| `map_junction.proto` | 路口 |
| `map_signal.proto` | 交通信号灯 |
| `map_stop_sign.proto` | 停车标志 |
| `map_crosswalk.proto` | 人行横道 |
| `map_yield_sign.proto` | 让行标志 |
| `map_overlap.proto` | 重叠区域 |
| `map_clear_area.proto` | 禁停区域 |
| `map_speed_bump.proto` | 减速带 |
| `map_speed_control.proto` | 限速区域 |
| `map_parking_space.proto` | 停车位 |
| `map_pnc_junction.proto` | PnC 路口 |
| `map_barrier_gate.proto` | 道闸 |
| `map_area.proto` | 区域 |
| `map_geometry.proto` | 地图几何类型 |
| `map_id.proto` | 地图元素 ID |
| `map_rsu.proto` | 路侧单元 |

#### Map

```protobuf
message Map {
  optional Header header = 1;
  repeated Crosswalk crosswalk = 2;
  repeated Junction junction = 3;
  repeated Lane lane = 4;
  repeated StopSign stop_sign = 5;
  repeated Signal signal = 6;
  repeated YieldSign yield = 7;
  repeated Overlap overlap = 8;
  repeated ClearArea clear_area = 9;
  repeated SpeedBump speed_bump = 10;
  repeated Road road = 11;
  repeated ParkingSpace parking_space = 12;
  repeated PNCJunction pnc_junction = 13;
  repeated RSU rsu = 14;
}
```

---

### 传感器消息（sensor_msgs / drivers_msgs）

传感器消息定义了各类传感器的原始数据格式。

**包名**：`apollo.drivers`

**Proto 文件**（sensor_msgs）：`pointcloud.proto`、`radar.proto`、`conti_radar.proto`、`gnss.proto`、`ins.proto`、`imu.proto`

**Proto 文件**（drivers_msgs）：`can_card_parameter.proto`

#### PointCloud

```protobuf
message PointCloud {
  optional Header header = 1;
  optional string frame_id = 2;
  optional bool is_dense = 3;
  repeated PointXYZIT point = 4;       // 点云数据
  optional double measurement_time = 5;
  optional uint32 width = 6;
  optional uint32 height = 7;
}
```

#### ContiRadar

```protobuf
message ContiRadar {
  optional Header header = 1;
  repeated ContiRadarObs contiobs = 2;       // 大陆雷达障碍物
  optional RadarState_201 radar_state = 3;
  optional ClusterListStatus_600 cluster_list_status = 4;
  optional ObjectListStatus_60A object_list_status = 5;
}
```

#### Image

```protobuf
message Image {
  optional Header header = 1;
  optional string frame_id = 2;
  optional double measurement_time = 3;
  optional uint32 height = 4;
  optional uint32 width = 5;
  optional string encoding = 6;       // 编码格式
  optional uint32 step = 7;           // 每行字节数
  optional bytes data = 8;            // 图像数据
}
```

---

### 可视化消息（dreamview_msgs）

Dreamview 是 Apollo 的可视化与人机交互界面，相关消息定义了 HMI 状态和仿真世界数据。

**包名**：`apollo.dreamview`

**Proto 文件**：`chart.proto`、`hmi_config.proto`、`hmi_mode.proto`、`hmi_status.proto`、`simulation_world.proto`

#### HMIStatus

```protobuf
message HMIStatus {
  optional Header header = 1;
  repeated string modes = 2;
  optional string current_mode = 3;
  repeated string maps = 4;
  optional string current_map = 5;
  repeated string vehicles = 6;
  optional string current_vehicle = 7;
  map<string, bool> modules = 8;
  map<string, ComponentStatus> monitored_components = 9;
}

enum HMIModeOperation {
  None = 0;
  SIM_DEBUG = 1;
  Sim_Control = 2;
  Auto_Drive = 3;
  TRACE = 4;
  Scenario_Sim = 5;
  Record = 6;
  Waypoint_Follow = 7;
}
```

---

### 外部命令消息（external_command_msgs）

外部命令消息提供了从外部系统向 Apollo 发送驾驶指令的接口。

**包名**：`apollo.external_command`

**Proto 文件**：

| 文件 | 说明 |
|------|------|
| `action_command.proto` | 动作指令 |
| `chassis_command.proto` | 底盘指令 |
| `command_status.proto` | 指令状态 |
| `free_space_command.proto` | 自由空间指令 |
| `geometry.proto` | 几何定义 |
| `lane_follow_command.proto` | 车道跟随指令 |
| `lane_segment.proto` | 车道段 |
| `path_follow_command.proto` | 路径跟随指令 |
| `precise_parking_command.proto` | 精确泊车指令 |
| `speed_command.proto` | 速度指令 |
| `valet_parking_command.proto` | 代客泊车指令 |
| `zone_cover_command.proto` | 区域覆盖指令 |

这些命令消息支持多种自动驾驶场景，包括车道跟随、路径跟随、泊车和区域覆盖等。

---

### 安全守护消息（guardian_msgs）

Guardian 模块是 Apollo 的安全守护层，在系统异常时接管控制。

**包名**：`apollo.guardian`

**Proto 文件**：`guardian.proto`

```protobuf
message GuardianCommand {
  optional Header header = 1;
  optional ControlCommand control_command = 2;
}
```

---

### 监控消息（monitor_msgs）

监控消息用于系统健康状态的上报和日志记录。

**包名**：`apollo.monitor`

**Proto 文件**：`system_status.proto`、`monitor_log.proto`

---

### 车辆配置消息（config_msgs）

车辆配置消息定义了车辆的物理参数和配置信息。

**包名**：`apollo.common`

**Proto 文件**：`vehicle_config.proto`

---

### 音频消息（audio_msgs）

音频消息用于车辆音频系统的交互。

**Proto 文件**：`audio.proto`、`audio_common.proto`、`audio_event.proto`

---

### V2X 消息（v2x_msgs）

V2X（Vehicle-to-Everything）消息定义了车路协同通信的数据结构。

**包名**：`apollo.v2x`

**Proto 文件**：`v2x_traffic_light.proto`

V2X 交通灯消息提供来自路侧设备的交通信号灯信息，可与感知模块的视觉检测结果进行融合，提高交通灯识别的准确性和提前量。

---

### 其他消息

#### storytelling_msgs

**Proto 文件**：`story.proto`

Storytelling 模块用于描述当前驾驶场景的上下文信息（如即将进入隧道、通过收费站等），供其他模块参考调整策略。

#### task_manager_msgs

**Proto 文件**：`task_manager.proto`

任务管理器消息用于协调和调度各模块的执行任务。

#### transform_msgs

**Proto 文件**：`transform.proto`

坐标变换消息定义了不同坐标系之间的变换关系（类似 ROS 中的 `tf`）。

---

## 消息间关系

Apollo 各模块通过 Cyber RT 的 channel 机制进行通信，消息之间存在清晰的数据流向关系：

```
传感器原始数据                    高精地图
(PointCloud, Image,              (Map)
 ContiRadar, Gnss)                 │
       │                          │
       ▼                          ▼
    感知模块 ──────────────► 预测模块
(PerceptionObstacles,        (PredictionObstacles)
 TrafficLightDetection)            │
       │                          │
       ▼                          ▼
    定位模块                   规划模块 ◄── 路由模块
(LocalizationEstimate)      (ADCTrajectory)  (RoutingResponse)
       │                          │
       ▼                          ▼
    底盘状态 ──────────────► 控制模块
    (Chassis)               (ControlCommand)
                                  │
                                  ▼
                              底盘执行
```

核心数据流说明：

1. **传感器 → 感知**：`PointCloud`、`Image`、`ContiRadar` 等原始数据经感知模块处理后输出 `PerceptionObstacles` 和 `TrafficLightDetection`
2. **感知 → 预测**：`PerceptionObstacle` 被嵌入到 `PredictionObstacle` 中，预测模块在此基础上生成未来轨迹
3. **预测 + 路由 → 规划**：规划模块综合 `PredictionObstacles`、`RoutingResponse` 和 `LocalizationEstimate` 生成 `ADCTrajectory`
4. **规划 → 控制**：控制模块根据 `ADCTrajectory` 和 `Chassis` 状态计算 `ControlCommand`
5. **Header 贯穿全局**：几乎所有消息都包含 `Header`，用于时间同步和消息溯源
6. **ErrorCode 统一错误处理**：各模块使用统一的 `ErrorCode` 枚举上报错误状态
7. **Guardian 安全兜底**：`GuardianCommand` 可在任何环节异常时介入，发送紧急停车指令

## 示例

以下示例展示如何在 Apollo 模块中使用这些 protobuf 消息。

### 读取定位信息并填充 Header

```cpp
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/basic_msgs/header.pb.h"

using apollo::common::Header;
using apollo::localization::LocalizationEstimate;

void ProcessLocalization(const LocalizationEstimate& localization) {
  // 获取消息头信息
  const Header& header = localization.header();
  double timestamp = header.timestamp_sec();
  std::string module = header.module_name();

  // 获取车辆位姿
  const auto& pose = localization.pose();
  double x = pose.position().x();
  double y = pose.position().y();
  double z = pose.position().z();
  double heading = pose.heading();

  // 获取速度信息
  double vx = pose.linear_velocity().x();
  double vy = pose.linear_velocity().y();

  AINFO << "Vehicle at (" << x << ", " << y << ") heading: " << heading;
}
```

### 构造控制指令

```cpp
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

using apollo::control::ControlCommand;
using apollo::canbus::Chassis;

ControlCommand GenerateControlCommand(
    double throttle, double brake, double steering) {
  ControlCommand cmd;

  // 填充 Header
  auto* header = cmd.mutable_header();
  header->set_timestamp_sec(apollo::cyber::Clock::NowInSeconds());
  header->set_module_name("control");

  // 设置控制量
  cmd.set_throttle(throttle);
  cmd.set_brake(brake);
  cmd.set_steering_target(steering);
  cmd.set_gear_location(Chassis::GEAR_DRIVE);

  return cmd;
}
```

### 遍历感知障碍物

```cpp
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

using apollo::perception::PerceptionObstacles;
using apollo::perception::PerceptionObstacle;

void ProcessObstacles(const PerceptionObstacles& obstacles) {
  for (const auto& obstacle : obstacles.perception_obstacle()) {
    int id = obstacle.id();
    double x = obstacle.position().x();
    double y = obstacle.position().y();

    // 根据类型分类处理
    switch (obstacle.type()) {
      case PerceptionObstacle::VEHICLE:
        HandleVehicle(obstacle);
        break;
      case PerceptionObstacle::PEDESTRIAN:
        HandlePedestrian(obstacle);
        break;
      case PerceptionObstacle::BICYCLE:
        HandleBicycle(obstacle);
        break;
      default:
        HandleUnknown(obstacle);
        break;
    }
  }
}
```

::: tip
所有 proto 文件编译后会生成对应的 `.pb.h` 和 `.pb.cc` 文件。在 Bazel 构建系统中，通过依赖 `//modules/common_msgs/xxx_msgs:xxx_proto` 即可使用对应的消息类型。
:::
