# Planners 规划器函数级源码解析

本文聚焦 `modules/planning/planners/` 目录，按函数级粒度拆解 Apollo 规划模块的 **4 种规划器实现**：PublicRoadPlanner（公共道路规划器）、RTKReplayPlanner（RTK 回放规划器）、LatticePlanner（格栅规划器）、NaviPlanner（导航规划器）。其中 PublicRoadPlanner 和 RTKReplayPlanner 在本文详细展开，LatticePlanner 和 NaviPlanner 已有独立文档。

## 1. 模块定位

`planners/` 是规划模块的**核心规划算法层**，位于 `PlanningComponent → PlanningBase → Planner` 调用链的末端。每个规划器以 Cyber 插件形式注册，由 `PlanningBase` 在运行时通过 `PluginManager` 动态加载。

```
PlanningComponent
    │
    ├── OnLanePlanning / NaviPlanning (PlanningBase 派生)
    │       │
    │       └── Planner (插件接口)
    │               ├── PublicRoadPlanner   ← 本文重点
    │               ├── RTKReplayPlanner    ← 本文重点
    │               ├── LatticePlanner      → 见 lattice-planner.md
    │               └── NaviPlanner         → 见 navi-planner.md
    │
    └── ScenarioManager (PublicRoadPlanner 内部)
            └── Scenario[] (场景插件列表)
```

## 2. 目录结构

```
modules/planning/planners/
├── public_road/
│   ├── public_road_planner.h / .cc    # 公共道路规划器
│   ├── public_road_planner_test.cc
│   ├── scenario_manager.h / .cc       # 场景管理器
│   └── proto/
│       └── planner_config.proto       # PlannerPublicRoadConfig
├── rtk/
│   ├── rtk_replay_planner.h / .cc     # RTK 回放规划器
│   └── rtk_replay_planner_test.cc
├── lattice/                           # → 见 lattice-planner.md
│   ├── lattice_planner.h / .cc
│   ├── behavior/
│   └── trajectory_generation/
└── navi/                              # → 见 navi-planner.md
    ├── navi_planner.h / .cc
    └── decider/
```

## 3. PublicRoadPlanner — 公共道路规划器

源码：`planners/public_road/public_road_planner.h:48` 与 `public_road_planner.cc:28`

### 3.1 类定义与继承关系

```cpp
class PublicRoadPlanner : public PlannerWithReferenceLine {
 public:
  virtual ~PublicRoadPlanner() = default;
  void Stop() override {}
  std::string Name() override { return "PUBLIC_ROAD"; }
  common::Status Init(const std::shared_ptr<DependencyInjector>& injector,
                      const std::string& config_path = "") override;
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override;
  void Reset(Frame* frame) override { scenario_manager_.Reset(frame); }
 private:
  ScenarioManager scenario_manager_;
  PlannerPublicRoadConfig config_;
  Scenario* scenario_ = nullptr;
};
```

**继承链**：`PublicRoadPlanner → PlannerWithReferenceLine → Planner`

**插件注册**：
```cpp
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PublicRoadPlanner, Planner)
```

### 3.2 `Init` — 初始化

```cpp
Status PublicRoadPlanner::Init(
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_path) {
  Planner::Init(injector, config_path);
  LoadConfig<PlannerPublicRoadConfig>(config_path, &config_);
  scenario_manager_.Init(injector, config_);
  return Status::OK();
}
```

**执行步骤**：

1. 调用基类 `Planner::Init()` 保存 `injector` 和 `config_path`
2. 从 `config_path` 加载 `PlannerPublicRoadConfig` protobuf 配置
3. 初始化 `ScenarioManager`，传入依赖注入器和配置

### 3.3 `Plan` — 主规划入口

```cpp
Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory);
```

**执行步骤**：

1. **场景更新**：调用 `scenario_manager_.Update(planning_start_point, frame)` 判断是否需要切换场景
2. **获取当前场景**：`scenario_ = scenario_manager_.mutable_scenario()`
3. **空场景检查**：若 `scenario_` 为空，返回 `PLANNING_ERROR`
4. **场景处理**：调用 `scenario_->Process(planning_start_point, frame)` 执行当前场景的规划逻辑
5. **调试记录**：若 `FLAGS_enable_record_debug` 为 true，将场景名、阶段名、消息写入 `ADCTrajectory` 的 debug 字段
6. **场景状态处理**：
   - `STATUS_DONE`：场景完成，再次调用 `Update` 切换到下一个场景
   - `STATUS_UNKNOWN`：返回错误
   - 其他：返回 OK

**关键设计**：PublicRoadPlanner 本身不做轨迹规划，而是将规划逻辑委托给当前活跃的 `Scenario`。它的核心职责是**场景调度**。

### 3.4 `Reset` — 重置

```cpp
void Reset(Frame* frame) override { scenario_manager_.Reset(frame); }
```

直接委托给 `ScenarioManager::Reset`。

## 4. ScenarioManager — 场景管理器

源码：`planners/public_road/scenario_manager.h:39` 与 `scenario_manager.cc:33`

### 4.1 类定义

```cpp
class ScenarioManager final {
 public:
  bool Init(const std::shared_ptr<DependencyInjector>& injector,
            const PlannerPublicRoadConfig& planner_config);
  Scenario* mutable_scenario() { return current_scenario_.get(); }
  DependencyInjector* injector() { return injector_.get(); }
  void Update(const common::TrajectoryPoint& ego_point, Frame* frame);
  void Reset(Frame* frame);
 private:
  std::shared_ptr<DependencyInjector> injector_;
  std::shared_ptr<Scenario> current_scenario_;
  std::shared_ptr<Scenario> default_scenario_type_;
  std::vector<std::shared_ptr<Scenario>> scenario_list_;
  bool init_ = false;
};
```

### 4.2 成员变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `injector_` | `shared_ptr<DependencyInjector>` | 依赖注入器 |
| `current_scenario_` | `shared_ptr<Scenario>` | 当前活跃场景 |
| `default_scenario_type_` | `shared_ptr<Scenario>` | 默认场景（LANE_FOLLOW） |
| `scenario_list_` | `vector<shared_ptr<Scenario>>` | 所有已注册场景列表 |
| `init_` | `bool` | 初始化标志，防止重复初始化 |

### 4.3 `Init` — 初始化场景列表

```cpp
bool ScenarioManager::Init(const std::shared_ptr<DependencyInjector>& injector,
                           const PlannerPublicRoadConfig& planner_config);
```

**执行步骤**：

1. **防重入**：若 `init_` 为 true 直接返回
2. **保存注入器**：`injector_ = injector`
3. **遍历配置中的场景声明**：对每个 `planner_config.scenario(i)`：
   - 通过 `PluginManager::CreateInstance<Scenario>()` 创建场景插件实例
   - 使用 `ConfigUtil::GetFullPlanningClassName()` 将短类名转为全限定名
   - 调用 `scenario->Init(injector_, name)` 初始化场景
   - 加入 `scenario_list_`
   - 若场景名为 `"LANE_FOLLOW"`，设为 `default_scenario_type_`
4. **设置初始场景**：`current_scenario_ = default_scenario_type_`
5. **标记已初始化**：`init_ = true`

### 4.4 `Update` — 场景切换决策

```cpp
void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             Frame* frame);
```

**执行步骤**：

1. 遍历 `scenario_list_`（按配置顺序，即优先级从高到低）
2. **优先级保护**：若当前遍历到的场景就是 `current_scenario_` 且状态为 `STATUS_PROCESSING`，直接返回（不打断正在执行的场景）
3. **可转移检查**：调用 `scenario->IsTransferable(current_scenario_.get(), *frame)` 判断是否满足切换条件
4. **执行切换**：
   - 调用 `current_scenario_->Exit(frame)` 退出当前场景
   - 更新 `current_scenario_` 为新场景
   - 调用 `current_scenario_->Reset()` 重置新场景状态
   - 调用 `current_scenario_->Enter(frame)` 进入新场景
5. 找到第一个可转移的场景后立即返回

**关键设计**：场景列表的顺序决定优先级。配置中靠前的场景优先级更高。正在处理中的场景不会被打断。

### 4.5 `Reset` — 重置到默认场景

```cpp
void ScenarioManager::Reset(Frame* frame);
```

**执行步骤**：

1. 若 `current_scenario_` 非空，调用 `Exit(frame)` 退出
2. 重置 `default_scenario_type_`
3. 将 `current_scenario_` 设回默认场景

## 5. RTKReplayPlanner — RTK 回放规划器

源码：`planners/rtk/rtk_replay_planner.h:44` 与 `rtk_replay_planner.cc:35`

### 5.1 类定义

```cpp
class RTKReplayPlanner : public PlannerWithReferenceLine {
 public:
  virtual ~RTKReplayPlanner() = default;
  std::string Name() override { return "RTK"; }
  Status Init(const std::shared_ptr<DependencyInjector>& injector,
              const std::string& config_path = "") override;
  void Stop() override {}
  Status Plan(const TrajectoryPoint& planning_init_point, Frame* frame,
              ADCTrajectory* ptr_computed_trajectory) override;
  Status PlanOnReferenceLine(const TrajectoryPoint& planning_init_point,
                             Frame* frame,
                             ReferenceLineInfo* reference_line_info) override;
  void ReadTrajectoryFile(const std::string& filename);
 private:
  std::uint32_t QueryPositionMatchedPoint(
      const TrajectoryPoint& start_point,
      const std::vector<TrajectoryPoint>& trajectory) const;
  std::vector<TrajectoryPoint> complete_rtk_trajectory_;
};
```

**设计意图**：RTKReplayPlanner 不做实时规划，而是从预录制的 RTK 轨迹文件中读取完整轨迹，根据车辆当前位置截取前方一段作为规划输出。适用于调试、测试和轨迹回放场景。

### 5.2 成员变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `complete_rtk_trajectory_` | `vector<TrajectoryPoint>` | 从文件读取的完整 RTK 轨迹 |

### 5.3 `Init` — 初始化

```cpp
Status RTKReplayPlanner::Init(
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_path);
```

**执行步骤**：

1. 调用基类 `Planner::Init()`
2. 调用 `ReadTrajectoryFile(FLAGS_rtk_trajectory_filename)` 从 gflag 指定的文件加载轨迹

### 5.4 `ReadTrajectoryFile` — 读取轨迹文件

```cpp
void RTKReplayPlanner::ReadTrajectoryFile(const std::string& filename);
```

**执行步骤**：

1. 清空 `complete_rtk_trajectory_`
2. 打开文件，跳过首行（表头）
3. 逐行解析，以 `\t` 或空格分隔
4. 每行至少 11 个字段，按顺序解析为 `TrajectoryPoint`：

| 索引 | 字段 | 对应 protobuf 字段 |
|------|------|-------------------|
| 0 | x | `path_point.x` |
| 1 | y | `path_point.y` |
| 2 | z | `path_point.z` |
| 3 | v | `v`（速度） |
| 4 | a | `a`（加速度） |
| 5 | kappa | `path_point.kappa`（曲率） |
| 6 | dkappa | `path_point.dkappa`（曲率变化率） |
| 7 | relative_time | `relative_time` |
| 8 | theta | `path_point.theta`（航向角） |
| 9 | （未使用） | — |
| 10 | s | `path_point.s`（累积弧长） |

### 5.5 `Plan` — 主规划入口

```cpp
Status RTKReplayPlanner::Plan(const TrajectoryPoint& planning_start_point,
                              Frame* frame,
                              ADCTrajectory* ptr_computed_trajectory);
```

**执行步骤**：

1. **换道优先**：在 `reference_line_info` 列表中查找 `IsChangeLanePath()` 为 true 的参考线
2. 若找到换道参考线：
   - 调用 `PlanOnReferenceLine()` 在该参考线上规划
   - 检查轨迹是否可行驶且长度超过 `FLAGS_change_lane_min_length`
3. 若换道规划失败或 `FLAGS_prioritize_change_lane` 为 false：
   - 遍历所有非换道参考线，逐一调用 `PlanOnReferenceLine()`

### 5.6 `PlanOnReferenceLine` — 单参考线规划

```cpp
Status RTKReplayPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame*,
    ReferenceLineInfo* reference_line_info);
```

**执行步骤**：

1. **有效性检查**：轨迹为空或少于 2 点时返回错误
2. **位置匹配**：调用 `QueryPositionMatchedPoint()` 找到当前位置在完整轨迹中的最近点索引
3. **截取前方轨迹**：从匹配点开始，截取 `FLAGS_rtk_trajectory_forward` 个点
4. **时间归零**：将截取段的 `relative_time` 减去起始点时间，使输出轨迹从 t=0 开始
5. **尾部填充**：若截取点数不足 `FLAGS_rtk_trajectory_forward`，复制最后一个点并递增时间戳（步长 `FLAGS_rtk_trajectory_resolution`）
6. **设置轨迹**：调用 `reference_line_info->SetTrajectory(DiscretizedTrajectory(trajectory_points))`

### 5.7 `QueryPositionMatchedPoint` — 最近点查询

```cpp
std::uint32_t RTKReplayPlanner::QueryPositionMatchedPoint(
    const TrajectoryPoint& start_point,
    const std::vector<TrajectoryPoint>& trajectory) const;
```

**算法**：线性遍历整条轨迹，计算每个点到 `start_point` 的欧氏距离平方，返回距离最小的点的索引。

**复杂度**：O(n)，n 为轨迹点数。

## 6. 规划器对比

| 特性 | PublicRoadPlanner | RTKReplayPlanner | LatticePlanner | NaviPlanner |
|------|------------------|------------------|----------------|-------------|
| 规划方式 | 场景委托 | 轨迹回放 | 采样+评估 | 导航线跟踪 |
| 是否需要地图 | 是 | 否 | 是 | 否（仅导航线） |
| 场景支持 | 多场景切换 | 无 | 无 | 无 |
| 适用场景 | 量产公共道路 | 调试/测试 | 结构化道路 | 高速公路 |
| 实时规划 | 是（委托场景） | 否（回放） | 是 | 是 |
| 插件名 | `PUBLIC_ROAD` | `RTK` | `LATTICE` | `NAVI` |

## 7. 配置与启动

### 7.1 规划器选择

在 `planning_base/conf/planning_config.pb.txt` 中通过 `planner_type` 字段选择：

```protobuf
planner_type: PUBLIC_ROAD
```

### 7.2 PublicRoadPlanner 场景配置

`planners/public_road/conf/planner_config.pb.txt`：

```protobuf
scenario {
  name: "LANE_FOLLOW"
  type: "LaneFollowScenario"
}
scenario {
  name: "PULL_OVER"
  type: "PullOverScenario"
}
scenario {
  name: "TRAFFIC_LIGHT_PROTECTED"
  type: "TrafficLightProtectedScenario"
}
# ... 更多场景
```

场景按声明顺序决定优先级，`LANE_FOLLOW` 作为默认场景。

### 7.3 RTK 相关 gflags

| Flag | 默认值 | 说明 |
|------|--------|------|
| `rtk_trajectory_filename` | — | RTK 轨迹文件路径 |
| `rtk_trajectory_forward` | 800 | 前向截取点数 |
| `rtk_trajectory_resolution` | 0.01 | 尾部填充时间步长（秒） |
| `prioritize_change_lane` | false | 是否优先换道规划 |
| `change_lane_min_length` | — | 换道轨迹最小长度 |

## 8. 场景生命周期

```
ScenarioManager::Init
    │
    ├── 创建所有 Scenario 插件实例
    ├── 调用每个 Scenario::Init()
    └── 设置 current_scenario_ = LANE_FOLLOW

ScenarioManager::Update (每个规划周期调用)
    │
    ├── 遍历 scenario_list_
    ├── 跳过正在处理中的当前场景
    ├── 检查 IsTransferable() 条件
    └── 若满足：Exit → Reset → Enter

Scenario::Process (由 PublicRoadPlanner::Plan 调用)
    │
    ├── 执行当前 Stage 的 Task 列表
    ├── 返回 ScenarioResult
    └── STATUS_DONE 时触发再次 Update
```

## 9. 插件注册机制

所有规划器通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册：

```cpp
// PublicRoadPlanner
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PublicRoadPlanner, Planner)

// RTKReplayPlanner
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RTKReplayPlanner, Planner)
```

运行时由 `PlanningBase` 通过以下方式加载：

```cpp
auto planner = PluginManager::Instance()->CreateInstance<Planner>(
    "apollo::planning::" + planner_type_name);
```
