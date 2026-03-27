# Apollo Planning 决策模块深度解析

## 1. 决策模块职责

决策模块是 Apollo Planning 系统中连接感知/预测与轨迹规划的核心中间层。它的职责可以概括为：

- **交通规则执行**：根据高精地图中的交通标志（红绿灯、停止标志、让行标志、人行横道等）生成虚拟障碍物和停车决策
- **障碍物决策**：对参考线上的每个障碍物做出纵向决策（停车/跟车/让行/超车/忽略）和横向决策（左绕行/右绕行/忽略）
- **规划目标构建**：综合所有交通规则产生的停车点，选出最近的硬停车点或软停车点作为 Lattice Planner 的规划目标
- **速度边界计算**：在 ST 图上为障碍物建立边界，为速度优化提供约束

决策模块在架构上分为两个层次：

1. **Traffic Rules 层**：在 Task Pipeline 执行之前运行，处理交通规则相关的决策
2. **Decider Tasks 层**：作为 Task Pipeline 的一部分，在路径/速度规划的各阶段穿插执行

## 2. Traffic Rules 分析

### 2.1 基类设计

Traffic Rules 的基类定义在 `planning_interface_base/traffic_rules_base/traffic_rule.h`：

```cpp
class TrafficRule {
 public:
  virtual bool Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector);
  virtual common::Status ApplyRule(
      Frame* const frame, ReferenceLineInfo* const reference_line_info) = 0;
  virtual void Reset() = 0;
 protected:
  template <typename T>
  bool LoadConfig(T* config);
  std::shared_ptr<DependencyInjector> injector_;
  std::string config_path_;
  std::string name_;
};
```

每个 TrafficRule 通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册为插件，由 `PluginManager` 在运行时动态加载。配置文件路径通过 `PluginManager::GetPluginConfPath` 自动推导，指向各规则目录下的 `conf/default_conf.pb.txt`。

### 2.2 TrafficDecider 调度器

`TrafficDecider`（定义在 `planning_interface_base/traffic_rules_base/traffic_decider.h`）是所有 Traffic Rules 的调度入口。其核心流程：

1. **初始化阶段**：读取 `FLAGS_traffic_rule_config_filename` 指定的 pipeline 配置文件，解析出 `TrafficRulesPipeline` 消息，按顺序实例化每个 `TrafficRule` 插件
2. **执行阶段**：遍历 `rule_list_`，依次调用每个规则的 `Reset()` 和 `ApplyRule()`
3. **目标构建**：调用 `BuildPlanningTarget()` 扫描所有虚拟障碍物的停车决策，选出 s 值最小的停车点

`BuildPlanningTarget` 中的停车点分类逻辑：

| 停车类型 | 对应 StopReasonCode |
|---------|-------------------|
| HARD 停车 | DESTINATION, CROSSWALK, STOP_SIGN, YIELD_SIGN, CREEPER, REFERENCE_END, SIGNAL |
| SOFT 停车 | YELLOW_SIGNAL |

### 2.3 各 Traffic Rule 详解

#### 2.3.1 TrafficLight（红绿灯）

**源码位置**：`traffic_rules/traffic_light/`

处理逻辑：
1. 从参考线获取所有信号灯重叠区域（`signal_overlaps`）
2. 跳过已在车辆后方的信号灯（`end_s <= adc_back_edge_s`）
3. 跳过已由 Scenario/Stage 标记为完成的信号灯
4. 对 s 投影距离与实际欧氏距离差异过大的信号灯做容错跳过（`kSDiscrepanceTolerance = 10.0m`）
5. 绿灯和黑灯（未检测到）直接通过
6. 红灯/黄灯/未知状态：检查停车减速度是否超过 `max_stop_deceleration`（默认 4.0 m/s²），超过则跳过
7. 通过检查后，创建虚拟障碍物（ID 前缀 `TL_`），调用 `BuildStopDecision` 生成停车决策

#### 2.3.2 StopSign（停止标志）

**源码位置**：`traffic_rules/stop_sign/`

处理逻辑：
1. 从参考线获取所有停止标志重叠区域（`stop_sign_overlaps`）
2. 跳过已在车辆后方的停止标志
3. 跳过已由 PlanningContext 标记为完成的停止标志
4. 创建虚拟障碍物（ID 前缀 `SS_`），使用 `STOP_REASON_STOP_SIGN` 生成停车决策
5. 支持 `wait_for_obstacle_id` 列表，等待交叉路口其他车辆通过

#### 2.3.3 YieldSign（让行标志）

**源码位置**：`traffic_rules/yield_sign/`

处理逻辑与 StopSign 类似：
1. 遍历参考线上的 `yield_sign_overlaps`
2. 跳过已通过或已完成的让行标志
3. 创建虚拟障碍物（ID 前缀 `YS_`），使用 `STOP_REASON_YIELD_SIGN` 生成停车决策

#### 2.3.4 Crosswalk（人行横道）

**源码位置**：`traffic_rules/crosswalk/`

这是最复杂的 Traffic Rule 之一，采用多层距离判断策略：

1. **查找人行横道**：通过 `FindCrosswalks` 获取参考线前方的人行横道重叠区域
2. **障碍物检测**：对每个人行横道，检查附近的行人和自行车障碍物
3. **分层决策逻辑**：
   - 横向距离 < `stop_strict_l_distance`（默认 5.0m）：严格停车区域，检查障碍物是否朝向自车移动
   - 横向距离 > `stop_loose_l_distance`（默认 5.0m）：宽松区域，可忽略
   - 两者之间：使用历史决策平滑，避免决策抖动
4. **超时机制**：行人/自行车静止超过 `stop_timeout`（默认 10s）后可通过
5. **减速度检查**：停车减速度超过 `max_stop_deceleration` 且障碍物在宽松区域时跳过
6. 创建虚拟障碍物（ID 前缀 `CW_`），使用 `STOP_REASON_CROSSWALK` 生成停车决策

#### 2.3.5 KeepClear（禁停区）

**源码位置**：`traffic_rules/keepclear/`

处理两类禁停区域：

1. **Clear Area 区域**：从高精地图的 `clear_area_overlaps` 获取，创建虚拟静态障碍物（ID 前缀 `KC_`），设置 ST 边界类型为 `KEEP_CLEAR`
2. **Junction 区域**：处理 PNC Junction 的禁停逻辑（ID 前缀 `KC_JC_`），会与交通标志（人行横道、停止标志、信号灯）的位置对齐，容差为 `align_with_traffic_sign_tolerance`（默认 4.5m）

特殊处理：如果车辆前沿已进入禁停区超过 `min_pass_s_distance`（默认 2.0m），则跳过该区域。

#### 2.3.6 Destination（目的地）

**源码位置**：`traffic_rules/destination/`

处理逻辑：
1. 仅在 `frame->is_near_destination()` 为 true 时生效
2. 如果存在靠边停车（Pull Over）位置，在靠边停车点生成停车决策（`STOP_REASON_PULL_OVER`）
3. 否则在路由终点生成停车决策（`STOP_REASON_DESTINATION`）

#### 2.3.7 ReferenceLineEnd（参考线末端）

**源码位置**：`traffic_rules/reference_line_end/`

当参考线剩余长度不足 `min_reference_line_remain_length`（默认 50.0m）时，在参考线末端创建虚拟停车墙（ID 前缀 `REF_END_`），防止车辆驶出参考线范围。

#### 2.3.8 BacksideVehicle（后方车辆）

**源码位置**：`traffic_rules/backside_vehicle/`

对后方车辆做忽略决策，避免后方车辆干扰前向规划：
1. 在车道保持模式下，对位于自车后方的障碍物添加纵向和横向 `Ignore` 决策
2. 判断条件包括：障碍物 SL 边界、ST 边界、预测轨迹是否与自车重叠
3. 使用 `backside_lane_width`（默认 4.0m）判断横向范围

#### 2.3.9 Rerouting（重新路由）

**源码位置**：`traffic_rules/rerouting/`

在变道失败时触发重新路由请求：
1. 检查当前参考线是否需要变道（`NextAction != FORWARD`）
2. 检查是否已在当前车道上且无法退出当前通道
3. 检查距离通道终点的距离是否小于 `prepare_rerouting_time * speed`
4. 冷却时间检查（`cooldown_time` 默认 3.0s），避免频繁重路由

#### 2.3.10 SpeedSetting（速度设置）

**源码位置**：`traffic_rules/speed_setting/`

处理外部速度指令：
1. 响应 `SpeedCommand` 类型的自定义命令
2. 支持设置绝对目标速度（`target_speed`）和速度因子（`target_speed_factor`）
3. 支持恢复默认目标速度（`is_restore_target_speed`）

## 3. 决策器架构

### 3.1 Decider 基类

Decider 基类定义在 `planning_interface_base/task_base/common/decider.h`，继承自 `Task`：

```cpp
class Decider : public Task {
 public:
  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;
  apollo::common::Status Execute(Frame* frame) override;
 protected:
  virtual apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info);
  virtual apollo::common::Status Process(Frame* frame);
};
```

`Execute` 方法先调用 `Task::Execute` 设置 `frame_` 和 `reference_line_info_` 成员变量，然后调用子类实现的 `Process` 方法。这种模板方法模式将通用的上下文绑定与具体决策逻辑分离。

### 3.2 Decider Task 类型总览

Apollo 中的 Decider Task 按功能可分为以下几类：

| 类别 | Decider | 职责 |
|------|---------|------|
| 路径决策 | PathDecider | 对静态障碍物做横向/纵向决策（Nudge/Stop/Ignore） |
| 路径决策 | PathReferenceDecider | 路径参考线选择 |
| 路径决策 | ObstacleNudgeDecider | 障碍物绕行距离计算 |
| 速度决策 | SpeedDecider | 基于 ST 图对障碍物做纵向决策（Stop/Follow/Yield/Overtake） |
| 速度决策 | SpeedBoundsDecider | 计算速度边界和 ST 图数据 |
| 速度决策 | STBoundsDecider | 生成 ST 可行驶边界 |
| 规则决策 | RuleBasedStopDecider | 基于规则的停车决策（路径末端、紧急变道、逆向绕行） |
| 开放空间 | OpenSpaceRoiDecider | 开放空间 ROI 计算 |
| 开放空间 | OpenSpaceFallbackDecider | 开放空间回退策略 |
| 开放空间 | OpenSpacePreStopDecider | 开放空间预停车 |
| 开放空间 | OpenSpaceReplanDecider | 开放空间重规划判断 |
| 安全 | RssDecider | RSS（Responsibility-Sensitive Safety）安全检查 |

### 3.3 PathDecider 详解

**源码位置**：`tasks/path_decider/`

PathDecider 在路径规划完成后执行，对参考线上的静态障碍物做出横向和纵向决策。核心逻辑在 `MakeStaticObstacleDecision` 中：

1. **IGNORE**：障碍物不在路径 s 范围内，或横向距离超过 `lateral_radius`（半车宽 + `lateral_ignore_buffer`）
2. **STOP**：障碍物与路径横向重叠过大（在 `min_nudge_l` 范围内），生成停车决策
3. **NUDGE**：障碍物在路径附近但不重叠，生成左绕行（`LEFT_NUDGE`）或右绕行（`RIGHT_NUDGE`）决策
4. **Blocking Obstacle 处理**：对阻塞障碍物直接生成停车决策，并通过 `front_static_obstacle_cycle_counter` 做时序平滑

### 3.4 SpeedDecider 详解

**源码位置**：`tasks/speed_decider/`

SpeedDecider 在速度规划完成后执行，基于速度曲线与障碍物 ST 边界的位置关系做出纵向决策：

1. **位置判断**：通过 `GetSTLocation` 判断速度曲线相对于障碍物 ST 边界的位置
   - `ABOVE`：速度曲线在障碍物上方（自车先通过）→ **OVERTAKE**
   - `BELOW`：速度曲线在障碍物下方（障碍物先通过）→ **STOP/FOLLOW/YIELD**
   - `CROSS`：速度曲线穿过障碍物边界 → **STOP**（阻塞障碍物）

2. **BELOW 情况的细分**：
   - KEEP_CLEAR 类型 → STOP（在禁停区前停车）
   - 静态障碍物 → STOP
   - 动态障碍物且满足跟车条件（`CheckIsFollow`）：
     - 跟车距离过近（`IsFollowTooClose`）→ STOP
     - 否则 → FOLLOW
   - 其他动态障碍物 → YIELD

3. **跟车距离计算**：`EstimateProperFollowGap` 使用分段线性函数，根据自车速度计算合适的跟车距离

4. **行人特殊处理**：对行人始终生成 STOP 决策，但有超时机制（4.0s 静止后可通过）

### 3.5 RuleBasedStopDecider 详解

**源码位置**：`tasks/rule_based_stop_decider/`

处理三类基于规则的停车场景：

1. **路径末端停车**（`AddPathEndStop`）：当路径长度小于 `short_path_length_threshold` 时，在路径末端设置停车围栏（`STOP_REASON_REFERENCE_END`）

2. **紧急变道停车**（`CheckLaneChangeUrgency`）：在变道场景中，如果目标车道被阻塞且距离通道终点较近（< `approach_distance_for_lane_change`），设置临时停车围栏等待变道机会（`STOP_REASON_LANE_CHANGE_URGENCY`）

3. **逆向绕行停车**（`StopOnSidePass`）：当路径需要借用对向车道绕行时，在进入逆向车道前设置停车点（`STOP_REASON_SIDEPASS_SAFETY`），确认安全后放行

### 3.6 STBoundsDecider 详解

**源码位置**：`tasks/st_bounds_decider/`

负责在 ST 图上生成可行驶边界，为速度优化提供约束：

1. **Fallback ST Bound**：生成保守的回退边界，确保安全
2. **Regular ST Bound**：生成正常行驶的 ST 边界，考虑障碍物的超车/让行决策
3. **决策排序**（`RankDecisions`）：根据引导线和驾驶限制对可用决策进行排序
4. **后向平滑**（`BackwardFlatten`）：对 ST 边界进行后向平滑处理

## 4. Proto 消息定义

### 4.1 决策消息（decision.proto）

**文件位置**：`modules/common_msgs/planning_msgs/decision.proto`

#### 4.1.1 障碍物决策类型

`ObjectDecisionType` 使用 `oneof` 定义了 8 种互斥的决策类型：

```protobuf
message ObjectDecisionType {
  oneof object_tag {
    ObjectIgnore ignore = 1;     // 忽略
    ObjectStop stop = 2;         // 停车
    ObjectFollow follow = 3;     // 跟车
    ObjectYield yield = 4;       // 让行
    ObjectOvertake overtake = 5; // 超车
    ObjectNudge nudge = 6;       // 绕行
    ObjectAvoid avoid = 7;       // 紧急避让
    ObjectSidePass side_pass = 8;// 侧方通过
  }
}
```

各决策消息的关键字段：

| 决策类型 | 关键字段 | 说明 |
|---------|---------|------|
| ObjectStop | reason_code, distance_s, stop_point, stop_heading | 停车原因码、停车距离、停车点位置和朝向 |
| ObjectFollow | distance_s, fence_point, fence_heading | 跟车距离、围栏点 |
| ObjectYield | distance_s, fence_point, time_buffer | 让行距离、时间缓冲 |
| ObjectOvertake | distance_s, fence_point, time_buffer | 超车距离、时间缓冲 |
| ObjectNudge | type (LEFT/RIGHT/DYNAMIC_LEFT/DYNAMIC_RIGHT), distance_l | 绕行方向和横向距离 |

#### 4.1.2 停车原因码

`StopReasonCode` 枚举定义了所有可能的停车原因：

```protobuf
enum StopReasonCode {
  STOP_REASON_HEAD_VEHICLE = 1;        // 前车
  STOP_REASON_DESTINATION = 2;         // 目的地
  STOP_REASON_PEDESTRIAN = 3;          // 行人
  STOP_REASON_OBSTACLE = 4;            // 障碍物
  STOP_REASON_SIGNAL = 100;            // 红灯
  STOP_REASON_STOP_SIGN = 101;         // 停止标志
  STOP_REASON_YIELD_SIGN = 102;        // 让行标志
  STOP_REASON_CLEAR_ZONE = 103;        // 禁停区
  STOP_REASON_CROSSWALK = 104;         // 人行横道
  STOP_REASON_CREEPER = 105;           // 蠕行
  STOP_REASON_REFERENCE_END = 106;     // 参考线末端
  STOP_REASON_YELLOW_SIGNAL = 107;     // 黄灯
  STOP_REASON_PULL_OVER = 108;         // 靠边停车
  STOP_REASON_SIDEPASS_SAFETY = 109;   // 侧方通过安全
  STOP_REASON_LANE_CHANGE_URGENCY = 201; // 紧急变道
  STOP_REASON_EMERGENCY = 202;         // 紧急停车
}
```

#### 4.1.3 主决策

`MainDecision` 定义了车辆级别的主决策：

```protobuf
message MainDecision {
  oneof task {
    MainCruise cruise = 1;              // 巡航
    MainStop stop = 2;                  // 停车
    MainEmergencyStop estop = 3;        // 紧急停车
    MainMissionComplete mission_complete = 6; // 任务完成
    MainNotReady not_ready = 7;         // 未就绪
    MainParking parking = 8;            // 泊车
  }
}
```

#### 4.1.4 最终决策结果

```protobuf
message DecisionResult {
  optional MainDecision main_decision = 1;       // 主决策
  optional ObjectDecisions object_decision = 2;   // 障碍物决策集合
  optional VehicleSignal vehicle_signal = 3;       // 车辆信号（转向灯等）
}
```

### 4.2 Traffic Rules Pipeline Proto

**文件位置**：`planning_interface_base/traffic_rules_base/proto/traffic_rules_pipeline.proto`

```protobuf
message TrafficRulesPipeline {
  repeated PluginDeclareInfo rule = 1;
}
```

其中 `PluginDeclareInfo`（定义在 `planning_base/proto/plugin_declare_info.proto`）：

```protobuf
message PluginDeclareInfo {
  required string name = 1;  // 插件别名
  required string type = 2;  // 插件类名
}
```

### 4.3 各 Traffic Rule 配置 Proto

每个 Traffic Rule 都有独立的配置 Proto，定义在各自的 `proto/` 目录下：

| Traffic Rule | Proto 消息 | 关键配置项 |
|-------------|-----------|-----------|
| TrafficLight | `TrafficLightConfig` | enabled, stop_distance(1.0m), max_stop_deceleration(4.0) |
| StopSign | `StopSignConfig` | enabled, stop_distance(1.0m) |
| YieldSign | `YieldSignConfig` | enabled, stop_distance(1.0m) |
| Crosswalk | `CrosswalkConfig` | stop_distance(1.0m), max_stop_deceleration(4.0), stop_strict_l_distance(4.0m), stop_loose_l_distance(5.0m), stop_timeout(10.0s), expand_s_distance(2.0m) |
| KeepClear | `KeepClearConfig` | enable_keep_clear_zone, enable_junction, min_pass_s_distance(2.0m), align_with_traffic_sign_tolerance(4.5m) |
| Destination | `DestinationConfig` | stop_distance(0.5m) |
| ReferenceLineEnd | `ReferenceLineEndConfig` | stop_distance(0.5m), min_reference_line_remain_length(50.0m) |
| BacksideVehicle | `BacksideVehicleConfig` | backside_lane_width(4.0m) |
| Rerouting | `ReroutingConfig` | cooldown_time(3.0s), prepare_rerouting_time(2.0s) |

## 5. 配置方式

### 5.1 插件化配置体系

Apollo 的决策模块采用插件化架构，配置分为三个层次：

1. **Pipeline 配置**：通过 `FLAGS_traffic_rule_config_filename` 指定的 `TrafficRulesPipeline` 文件定义 Traffic Rules 的执行顺序和启用列表。每条规则通过 `PluginDeclareInfo` 声明名称和类型。

2. **规则默认配置**：每个 Traffic Rule 插件在自己的 `conf/default_conf.pb.txt` 中定义默认参数。配置路径由 `PluginManager::GetPluginConfPath` 根据类名自动推导。

3. **Task 配置**：Decider Task 的配置通过 `ConfigUtil::LoadMergedConfig` 加载，支持默认配置与场景级配置的合并覆盖。

### 5.2 配置文件示例

以 Crosswalk 为例，其默认配置（`traffic_rules/crosswalk/conf/default_conf.pb.txt`）：

```protobuf
stop_distance: 1.0
max_stop_deceleration: 4.0
min_pass_s_distance: 1.0
expand_s_distance: 2.0
stop_strict_l_distance: 5.0
stop_loose_l_distance: 5.0
stop_timeout: 10.0
```

### 5.3 GFlags 参数

决策相关的全局参数通过 GFlags 定义在 `planning_base/gflags/planning_gflags.cc` 中，包括：
- `traffic_rule_config_filename`：Traffic Rules Pipeline 配置文件路径
- `virtual_stop_wall_length`：虚拟停车墙长度
- `min_stop_distance_obstacle`：障碍物最小停车距离
- `lateral_ignore_buffer`：横向忽略缓冲区
- `destination_obstacle_id`：目的地障碍物 ID

## 6. 数据流

### 6.1 整体数据流

```
感知/预测数据 → Frame (含 ReferenceLineInfo)
                    ↓
            TrafficDecider.Execute()
                    ↓
        ┌───────────────────────────┐
        │  遍历 Traffic Rules       │
        │  (TrafficLight, StopSign, │
        │   Crosswalk, KeepClear,   │
        │   Destination, ...)       │
        │                           │
        │  → 创建虚拟障碍物          │
        │  → 添加停车/忽略决策       │
        └───────────────────────────┘
                    ↓
          BuildPlanningTarget()
          → 选出最近停车点
          → 设置 LatticeStopPoint
                    ↓
        ┌───────────────────────────┐
        │  Task Pipeline 执行       │
        │                           │
        │  PathBoundsDecider        │
        │  → PathOptimizer          │
        │  → PathDecider (横向决策)  │
        │  → RuleBasedStopDecider   │
        │  → SpeedBoundsDecider     │
        │  → STBoundsDecider        │
        │  → SpeedOptimizer         │
        │  → SpeedDecider (纵向决策) │
        └───────────────────────────┘
                    ↓
          DecisionResult 输出
          (MainDecision + ObjectDecisions)
```

### 6.2 关键数据结构

#### PathDecision

`PathDecision` 是障碍物决策的核心容器，维护了参考线上所有障碍物及其决策：

- `obstacles_`：`IndexedList<string, Obstacle>` 类型，存储所有障碍物
- `main_stop_`：当前最近的主停车决策
- `AddLateralDecision` / `AddLongitudinalDecision`：添加横向/纵向决策
- `MergeWithMainStop`：将新的停车决策与当前主停车决策合并（保留更近的）

#### DecisionData

`DecisionData` 对障碍物进行分类管理：

```cpp
enum class VirtualObjectType {
  DESTINATION = 0,
  CROSSWALK = 1,
  TRAFFIC_LIGHT = 2,
  CLEAR_ZONE = 3,
  REROUTE = 4,
  DECISION_JUMP = 5,
  PRIORITY = 6
};
```

提供按类型查询障碍物的接口，并支持创建虚拟障碍物。

### 6.3 决策传递机制

1. **Traffic Rules → PathDecision**：Traffic Rules 通过 `BuildStopDecision` 工具函数创建虚拟障碍物并添加到 `PathDecision` 中
2. **PathDecider → PathDecision**：PathDecider 对静态障碍物添加横向决策（Nudge）和纵向决策（Stop/Ignore）
3. **SpeedDecider → Obstacle**：SpeedDecider 直接在 `Obstacle` 对象上添加纵向决策（Stop/Follow/Yield/Overtake）
4. **PathDecision → DecisionResult**：最终由 Planning 主流程将 `PathDecision` 中的所有决策汇总为 `DecisionResult` 输出
