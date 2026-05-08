---
title: "Planning Configuration Reference 规划配置参考"
---

# Planning Configuration Reference 规划配置参考

本文档梳理 `modules/planning/` 下的关键 Protobuf 配置定义，为理解和调优规划模块提供配置参考。

## 1. 模块定位

Apollo 规划模块的配置分为三层：

1. **模块级配置**：`PlanningConfig` 定义 topic、学习模式、参考线配置
2. **场景/任务级配置**：每个场景和任务有独立的 `.proto` 配置文件
3. **跨帧状态**：`PlanningStatus` 存储跨帧持久化的运行时状态

```
PlanningConfig (模块启动时加载)
    ├── TopicConfig (channel topic)
    ├── ReferenceLineConfig (PncMap 插件)
    └── PlanningLearningMode (NO_LEARNING / E2E / HYBRID)

ScenarioConfig (每个场景独立)
    ├── ScenarioBareIntersectionUnprotectedConfig
    ├── ScenarioEmergencyPullOverConfig
    ├── ScenarioStopSignUnprotectedConfig
    └── ...

TaskConfig (每个任务独立)
    ├── LaneFollowPathConfig
    ├── SpeedBoundsDeciderConfig
    ├── PiecewiseJerkSpeedConfig
    └── ...

PlanningStatus (跨帧持久化)
    ├── ChangeLaneStatus
    ├── PathDeciderStatus
    ├── ScenarioStatus
    └── ... (17 个子状态)
```

## 2. PlanningConfig — 模块级配置

```protobuf
message PlanningConfig {
  enum PlanningLearningMode {
    NO_LEARNING = 0;    // 纯规则规划
    E2E = 1;            // 端到端学习
    HYBRID = 2;         // 混合模式
    RL_TEST = 3;        // 强化学习测试
    E2E_TEST = 4;       // 端到端测试
    HYBRID_TEST = 5;    // 混合测试
  }
  optional TopicConfig topic_config = 1;
  optional PlanningLearningMode learning_mode = 2;
  optional ReferenceLineConfig reference_line_config = 3;
  optional string planner = 4;
}
```

### 2.1 TopicConfig — Topic 配置

| 字段 | 说明 |
|------|------|
| `chassis_topic` | 底盘状态 topic |
| `localization_topic` | 定位 topic |
| `prediction_topic` | 预测 topic |
| `planning_trajectory_topic` | 规划轨迹输出 topic |
| `routing_response_topic` | 路由响应 topic |
| `traffic_light_detection_topic` | 交通灯检测 topic |
| `planning_command_topic` | 规划命令 topic |
| `relative_map_topic` | 相对地图 topic（导航模式） |

### 2.2 ReferenceLineConfig — 参考线配置

```protobuf
message ReferenceLineConfig {
  repeated string pnc_map_class = 2;  // PncMap 插件类名列表
}
```

- 默认使用 `apollo::planning::LaneFollowMap`
- 支持多 PncMap 插件配置

## 3. PlanningStatus — 跨帧状态

`PlanningStatus` 是 `PlanningContext` 的核心数据成员，在规划帧之间持久化。

### 3.1 ChangeLaneStatus — 变道状态

```protobuf
message ChangeLaneStatus {
  enum Status {
    IN_CHANGE_LANE = 1;        // 变道中
    CHANGE_LANE_FAILED = 2;    // 变道失败
    CHANGE_LANE_FINISHED = 3;  // 变道完成
  }
  optional Status status = 1;
  optional string path_id = 2;     // 当前驾驶的路由段 ID
  optional double timestamp = 3;   // 状态开始时间戳
}
```

### 3.2 PathDeciderStatus — 路径决策状态

```protobuf
message PathDeciderStatus {
  optional int32 front_static_obstacle_cycle_counter = 1;  // 前方静态障碍物存在帧数
  optional bool is_in_path_lane_borrow_scenario = 2;       // 是否在借道场景
  optional string front_static_obstacle_id = 3;            // 前方阻塞障碍物 ID
  optional bool left_borrow = 4;                           // 是否左借道
  optional bool right_borrow = 5;                          // 是否右借道
}
```

### 3.3 ScenarioStatus — 场景状态

```protobuf
message ScenarioStatus {
  optional string scenario_type = 1;  // 当前场景类型
  optional string stage_type = 2;     // 当前阶段类型
}
```

### 3.4 PullOverStatus — 靠边停车状态

```protobuf
message PullOverStatus {
  enum PullOverType {
    PULL_OVER = 1;            // 到达目的地靠边停车
    EMERGENCY_PULL_OVER = 2;  // 紧急靠边停车
  }
  optional PullOverType pull_over_type = 1;
  optional bool plan_pull_over_path = 2;
  optional apollo.common.PointENU position = 3;  // 停车位置
  optional double theta = 4;                      // 停车航向
  optional double length_front = 5;               // 前方长度
  optional double length_back = 6;                // 后方长度
  optional double width_left = 7;                 // 左侧宽度
  optional double width_right = 8;                // 右侧宽度
}
```

### 3.5 OpenSpaceStatus — 开放空间状态

```protobuf
message OpenSpaceStatus {
  repeated string partitioned_trajectories_index_history = 1;  // 分段轨迹索引历史
  optional bool position_init = 2;                              // 位置是否已初始化
}
```

### 3.6 ParkAndGoStatus — 泊车起步状态

```protobuf
message ParkAndGoStatus {
  optional apollo.common.PointENU adc_init_position = 1;  // 初始位置
  optional double adc_init_heading = 2;                    // 初始航向
  optional bool in_check_stage = 3;                        // 是否在检查阶段
  optional apollo.common.PointENU adc_adjust_end_pose = 4; // 调整终端位姿
}
```

### 3.7 信号灯/标志状态

```protobuf
message StopSignStatus {
  optional string current_stop_sign_overlap_id = 1;  // 当前停车标志
  optional string done_stop_sign_overlap_id = 2;     // 已完成的停车标志
  repeated string wait_for_obstacle_id = 3;           // 等待的障碍物
}

message TrafficLightStatus {
  repeated string current_traffic_light_overlap_id = 1;
  repeated string done_traffic_light_overlap_id = 2;
}

message YieldSignStatus {
  repeated string current_yield_sign_overlap_id = 1;
  repeated string done_yield_sign_overlap_id = 2;
  repeated string wait_for_obstacle_id = 3;
}
```

### 3.8 阻塞状态

```protobuf
message LaneFollowStatus {
  optional bool lane_follow_block = 1;
  optional string block_obstacle_id = 2;
  optional double last_block_timestamp = 3;
  optional double block_duration = 4;
}

message LaneBorrowStatus {
  optional bool lane_borrow_block = 1;
  optional string block_obstacle_id = 2;
  optional double last_block_timestamp = 3;
  optional double block_duration = 4;
}
```

## 4. PlanningStatus 完整字段映射

| 字段号 | 子消息 | 说明 |
|--------|--------|------|
| 2 | ChangeLaneStatus | 变道状态 |
| 3 | CreepDeciderStatus | 蠕行决策状态 |
| 4 | CrosswalkStatus | 人行横道状态 |
| 5 | DestinationStatus | 目的地状态 |
| 6 | EmergencyStopStatus | 紧急停车状态 |
| 7 | OpenSpaceStatus | 开放空间状态 |
| 8 | ParkAndGoStatus | 泊车起步状态 |
| 9 | PathDeciderStatus | 路径决策状态 |
| 10 | PullOverStatus | 靠边停车状态 |
| 11 | ReroutingStatus | 重路由状态 |
| 12 | ScenarioStatus | 场景状态 |
| 13 | SpeedDeciderStatus | 速度决策状态 |
| 14 | StopSignStatus | 停车标志状态 |
| 15 | TrafficLightStatus | 交通灯状态 |
| 16 | YieldSignStatus | 让行标志状态 |
| 17 | LaneFollowStatus | 车道跟随阻塞状态 |
| 18 | LaneBorrowStatus | 借道阻塞状态 |

## 5. 关键配置文件

### 5.1 场景配置

每个场景目录下有独立的 `proto/` 子目录，定义场景特有的配置参数。例如：

- `stop_sign_unprotected.proto`：停车等待时间、蠕行速度等
- `traffic_light_protected.proto`：交通灯检测参数
- `valet_parking.proto`：泊车车位搜索参数

### 5.2 任务配置

每个任务目录下有独立的 `proto/` 子目录，定义任务特有的配置参数。例如：

- `lane_follow_path.proto`：路径边界参数
- `speed_bounds_decider.proto`：速度边界参数
- `piecewise_jerk_speed.proto`：QP 优化权重

### 5.3 参考线平滑配置

```protobuf
message ReferenceLineSmootherConfig {
  // QP Spline / Spiral / DiscretePoints 三选一
  optional QpSplineSmootherConfig qp_spline = 1;
  optional SpiralSmootherConfig spiral = 2;
  optional DiscretePointsSmootherConfig discrete_points = 3;
  // 通用参数
  optional double max_constraint_interval = 4;
  optional double longitudinal_boundary_bound = 5;
  optional double max_lateral_boundary_bound = 6;
  optional double min_lateral_boundary_bound = 7;
  optional double lateral_buffer = 8;
}
```

## 6. 配置加载机制

```
ConfigUtil::LoadMergedConfig(default_path, config_path, config)
    │
    ├── 1. 加载 default_path (默认配置)
    ├── 2. 加载 config_path (用户配置)
    └── 3. 合并：用户配置覆盖默认配置的同名字段

ConfigUtil::LoadOverridedConfig(default_path, config_path, config)
    │
    ├── 1. 加载 default_path (默认配置)
    └── 2. 若 config_path 存在，完全覆盖
```

- `LoadMergedConfig`：增量合并（推荐）
- `LoadOverridedConfig`：完全覆盖

## 7. 配置调优指南

| 参数类别 | 关键参数 | 调优方向 |
|---------|---------|---------|
| 路径规划 | `path_bounds_decider_config` | 边界宽度、借道阈值 |
| 速度规划 | `piecewise_jerk_speed_config` | jerk 权重、加速度限制 |
| 停车决策 | `rule_based_stop_decider_config` | 停车距离、等待时间 |
| 变道 | `lane_change_path_config` | 安全距离、变道速度 |
| 参考线平滑 | `reference_line_smoother_config` | 平滑权重、锚点间距 |
| 开放空间 | `planner_open_space_config` | Hybrid A* 参数、ROI 大小 |