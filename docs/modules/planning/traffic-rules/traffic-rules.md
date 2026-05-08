---
title: "Traffic Rules 交通规则模块函数级源码解析"
---

# Traffic Rules 交通规则模块函数级源码解析

本文聚焦 `modules/planning/traffic_rules/` 与 `modules/planning/planning_interface_base/traffic_rules_base/` 目录，按函数级粒度拆解交通规则的基类框架和 10 个具体规则实现。

## 1. 模块定位

交通规则模块是 Apollo 规划流水线的**前置决策层**。在场景机（Scenario）和任务（Task）执行之前，`TrafficDecider` 按配置顺序依次调用各交通规则，为参考线上的障碍物和虚拟障碍物设置决策（停车、让行、忽略等），为后续路径/速度规划提供约束。

```
ReferenceLineInfo ──> TrafficDecider ──> TrafficRule[0..N] ──> PathDecision
                        │
                        ├── BacksideVehicle    (后方车辆)
                        ├── Crosswalk          (人行横道)
                        ├── Destination        (目的地)
                        ├── KeepClear          (禁停区域)
                        ├── ReferenceLineEnd   (参考线末端)
                        ├── Rerouting          (重新路由)
                        ├── SpeedSetting       (巡航速度)
                        ├── StopSign           (停车标志)
                        ├── TrafficLight       (交通灯)
                        └── YieldSign          (让行标志)
```

## 2. 基类框架

### 2.1 TrafficRule 交通规则基类

```cpp
class TrafficRule {
 public:
  TrafficRule();
  virtual ~TrafficRule() = default;
  virtual bool Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector);
  virtual common::Status ApplyRule(
      Frame* const frame, ReferenceLineInfo* const reference_line_info) = 0;
  virtual void Reset() = 0;
  std::string Getname() { return name_; }
 protected:
  template <typename T> bool LoadConfig(T* config);
  std::shared_ptr<DependencyInjector> injector_;
  std::string config_path_;
  std::string name_;
};
```

- **`Init`**：保存规则名称，设置配置文件路径 `config_path_`
- **`ApplyRule`**（纯虚函数）：执行规则逻辑，对 `PathDecision` 中的障碍物设置决策
- **`Reset`**（纯虚函数）：重置规则内部状态
- **`LoadConfig<T>`**：从 `config_path_` 加载 Protobuf 配置

### 2.2 TrafficDecider 交通决策器

```cpp
class TrafficDecider {
 public:
  bool Init(const std::shared_ptr<DependencyInjector>& injector);
  common::Status Execute(Frame* frame, ReferenceLineInfo* reference_line_info);
 private:
  void BuildPlanningTarget(ReferenceLineInfo* reference_line_info);
  std::vector<std::shared_ptr<TrafficRule>> rule_list_;
  TrafficRulesPipeline rule_pipeline_;
};
```

**`Init`**：

- 从 `TrafficRulesPipeline` 配置加载规则列表
- 通过 CyberRT 插件管理器创建各规则实例
- 调用每个规则的 `Init` 方法

**`Execute`**：

1. 遍历 `rule_list_`，依次调用 `rule->ApplyRule(frame, reference_line_info)`
2. 任一规则返回错误则中止
3. 调用 `BuildPlanningTarget` 构建规划目标（停车点/巡航速度）

**`BuildPlanningTarget`**：

- 分析 `PathDecision` 中的障碍物决策
- 若存在停车决策，设置 `PlanningTarget.stop_point`
- 否则设置 `PlanningTarget.cruise_speed`

## 3. 具体规则实现

### 3.1 BacksideVehicle 后方车辆规则

```cpp
class BacksideVehicle : public TrafficRule {
 private:
  BacksideVehicleConfig config_;
  void MakeLaneKeepingObstacleDecision(const SLBoundary& adc_sl_boundary,
                                       PathDecision* path_decision,
                                       const common::VehicleState& vehicle_state);
  bool PredictionLineOverlapEgo(const Obstacle& obstacle,
                                const common::VehicleState& vehicle_state);
};
```

**`ApplyRule` 流程**：

1. 若当前正在变道，跳过（变道场景由场景机处理）
2. 调用 `MakeLaneKeepingObstacleDecision` 处理车道保持模式下的后方车辆

**`MakeLaneKeepingObstacleDecision`**：

- 遍历所有障碍物
- `PredictionLineOverlapEgo`：检查障碍物预测轨迹是否与自车路径重叠
- 对后方且预测轨迹不与自车重叠的障碍物设置 `IGNORE` 决策
- 对可能影响自车的障碍物保持默认决策

### 3.2 Crosswalk 人行横道规则

```cpp
class Crosswalk : public TrafficRule {
 private:
  static constexpr char const* CROSSWALK_VO_ID_PREFIX = "CW_";
  std::vector<const hdmap::PathOverlap*> crosswalk_overlaps_;
  void MakeDecisions(Frame*, ReferenceLineInfo*);
  bool FindCrosswalks(ReferenceLineInfo*);
  bool CheckStopForObstacle(ReferenceLineInfo*, const hdmap::CrosswalkInfoConstPtr,
                            const Obstacle&, double stop_deceleration);
};
```

**`ApplyRule` 流程**：

1. `FindCrosswalks`：在参考线上查找所有人行横道区域（`PathOverlap`）
2. `MakeDecisions`：对每个人行横道区域：
   - 创建虚拟障碍物 `"CW_" + crosswalk_id`，跨越人行横道区域
   - 检查区域内是否有行人/骑车人等障碍物
   - `CheckStopForObstacle`：判断是否需要为该障碍物停车
     - 计算自车到人行横道的距离
     - 根据 `stop_deceleration` 判断能否安全停车
     - 若能安全停车且有行人，设置 `STOP` 决策
   - 对虚拟障碍物设置 `STOP` 决策，迫使自车在人行横道前停车

**`Reset`**：清空 `crosswalk_overlaps_`

### 3.3 Destination 目的地规则

```cpp
class Destination : public TrafficRule {
 private:
  DestinationConfig config_;
  int MakeDecisions(Frame*, ReferenceLineInfo*);
};
```

**`ApplyRule` 流程**：

1. 检查当前参考线是否包含路由终点
2. `MakeDecisions`：在路由终点位置设置停车决策
3. 根据配置决定是否需要完全停车或仅减速

### 3.4 KeepClear 禁停区域规则

```cpp
class KeepClear : public TrafficRule {
 private:
  static constexpr char const* KEEP_CLEAR_VO_ID_PREFIX = "KC_";
  static constexpr char const* KEEP_CLEAR_JUNCTION_VO_ID_PREFIX = "KC_JC_";
  bool IsCreeping(double pnc_junction_start_s, double adc_front_edge_s) const;
  bool BuildKeepClearObstacle(Frame*, ReferenceLineInfo*,
                              const std::string& virtual_obstacle_id,
                              double keep_clear_start_s, double keep_clear_end_s);
};
```

**`ApplyRule` 流程**：

1. 在参考线上查找两类禁停区域：
   - **Clear Area**（净空区域）：路口前的禁停区
   - **PNC Junction**（规划控制交叉口）：需要规划通过的路口
2. 对每个禁停区域：
   - `BuildKeepClearObstacle`：创建虚拟障碍物
     - Clear Area 使用前缀 `"KC_"`
     - PNC Junction 使用前缀 `"KC_JC_"`
   - 虚拟障碍物覆盖 `[keep_clear_start_s, keep_clear_end_s]` 区间
   - 设置 `STOP` 决策，防止自车在禁停区停车

**`IsCreeping`**：

- 判断自车是否在交叉口区域蠕行（低速通过）
- 用于 PNC Junction 的特殊处理

### 3.5 ReferenceLineEnd 参考线末端规则

```cpp
class ReferenceLineEnd : public TrafficRule {
 private:
  static constexpr char const* REF_LINE_END_VO_ID_PREFIX = "REF_END_";
  ReferenceLineEndConfig config_;
};
```

**`ApplyRule` 流程**：

1. 计算参考线末端到自车的距离
2. 若距离小于阈值：
   - 创建虚拟障碍物 `"REF_END_"` 在参考线末端
   - 设置 `STOP` 决策
3. 同时触发重新路由请求（若配置启用）

### 3.6 Rerouting 重新路由规则

```cpp
class Rerouting : public TrafficRule {
 private:
  ReroutingConfig config_;
  bool ChangeLaneFailRerouting();
  ReferenceLineInfo* reference_line_info_ = nullptr;
  Frame* frame_ = nullptr;
};
```

**`ApplyRule` 流程**：

1. 保存 `frame` 和 `reference_line_info` 到成员变量
2. `ChangeLaneFailRerouting`：检查变道是否失败
   - 若当前参考线是变道路径且变道持续时间过长
   - 发送重新路由请求
3. **特殊设计**：这是唯一将 `Frame` 和 `ReferenceLineInfo` 保存为成员变量的规则（其他规则仅通过参数传递）

**`Reset`**：将两个指针置为 nullptr

### 3.7 SpeedSetting 巡航速度规则

```cpp
class SpeedSetting : public TrafficRule {
 private:
  double last_cruise_speed_;
  uint32_t last_sequence_num_;
};
```

**`ApplyRule` 流程**：

1. 从 `Frame` 获取最新的巡航速度命令
2. 比较 `sequence_num` 判断是否有新命令
3. 若有新命令，更新参考线的限速
4. 通过 `ReferenceLine::AddSpeedLimit` 叠加限速段

**特殊设计**：

- 唯一不重写 `Init()` 的规则（使用默认构造函数）
- 没有 Protobuf 配置文件
- 通过 `last_sequence_num_` 检测命令更新

### 3.8 StopSign 停车标志规则

```cpp
class StopSign : public TrafficRule {
 private:
  static constexpr char const* STOP_SIGN_VO_ID_PREFIX = "SS_";
  StopSignConfig config_;
  void MakeDecisions(Frame*, ReferenceLineInfo*);
};
```

**`ApplyRule` 流程**：

1. 在参考线上查找停车标志（`PathOverlap`）
2. `MakeDecisions`：
   - 创建虚拟障碍物 `"SS_" + stop_sign_id`
   - 设置停车位置（停车线前 `stop_distance` 处）
   - 设置 `STOP` 决策
   - 处理停车等待逻辑（配置等待时间、多辆车排队等）

### 3.9 TrafficLight 交通灯规则

```cpp
class TrafficLight : public TrafficRule {
 private:
  static constexpr char const* TRAFFIC_LIGHT_VO_ID_PREFIX = "TL_";
  TrafficLightConfig config_;
  void MakeDecisions(Frame*, ReferenceLineInfo*);
};
```

**`ApplyRule` 流程**：

1. 获取参考线上所有交通灯信号（通过 Perception 模块）
2. `MakeDecisions`：
   - 对每个交通灯：
     - **红灯**：创建虚拟障碍物 `"TL_" + light_id`，设置 `STOP`
     - **黄灯**：根据距离和速度判断能否通过，不能则 `STOP`
     - **绿灯**：不设置停车决策
   - 黄灯决策逻辑：
     - 计算自车到停车线的距离和所需减速度
     - 若减速度 < 阈值，允许通过
     - 否则设置 `STOP`

### 3.10 YieldSign 让行标志规则

```cpp
class YieldSign : public TrafficRule {
 private:
  static constexpr char const* YIELD_SIGN_VO_ID_PREFIX = "YS_";
  YieldSignConfig config_;
  void MakeDecisions(Frame*, ReferenceLineInfo*);
};
```

**`ApplyRule` 流程**：

1. 在参考线上查找让行标志
2. `MakeDecisions`：
   - 创建虚拟障碍物 `"YS_" + yield_sign_id`
   - 设置 `YIELD` 或 `STOP` 决策
   - 与 `StopSign` 不同，让行标志允许低速通过而非完全停车

## 4. 虚拟障碍物 ID 前缀汇总

| 规则 | 前缀 | 说明 |
|------|------|------|
| Crosswalk | `CW_` | 人行横道区域 |
| KeepClear | `KC_` | 净空区域 |
| KeepClear (Junction) | `KC_JC_` | PNC 交叉口 |
| ReferenceLineEnd | `REF_END_` | 参考线末端 |
| StopSign | `SS_` | 停车标志位置 |
| TrafficLight | `TL_` | 交通灯停车线 |
| YieldSign | `YS_` | 让行标志位置 |

## 5. 设计模式与扩展

### 5.1 插件化架构

- 所有规则通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 注册
- 规则列表通过 `TrafficRulesPipeline` Protobuf 配置动态组装
- 新增规则只需实现 `TrafficRule` 基类并在配置中添加

### 5.2 统一决策接口

- 所有规则通过 `PathDecision` 设置障碍物决策
- 决策类型：`STOP`、`YIELD`、`IGNORE`、`FOLLOW`、`OVERTAKE` 等
- 虚拟障碍物使用唯一 ID 前缀避免冲突

### 5.3 执行顺序

- 规则按 `rule_list_` 中的顺序串行执行
- 后执行的规则可以覆盖先执行的规则的决策
- 配置文件控制规则的启用/禁用和执行顺序