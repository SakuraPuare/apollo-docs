---
title: "planning_interface_base 接口层"
---

# planning_interface_base 接口层

> Apollo Planning 模块通过 `planning_interface_base` 定义了所有可插拔组件（Planner、Scenario、Stage、Task、TrafficRule）的抽象基类与契约，使具体实现可以以 CyberRT 插件的形式被动态注册、配置化调度。

## 模块定位

`planning_interface_base` 是 Planning 模块中唯一承担"接口声明"职责的库。它不实现任何具体业务逻辑，而是：

1. **定义基类**：为 Planner / Scenario / Stage / Task / TrafficRule 五类可扩展对象提供抽象基类
2. **规定数据契约**：明确每个基类 `Init` / `Process` / `Execute` 等虚函数的输入输出（`TrajectoryPoint`、`Frame`、`ReferenceLineInfo`、`ADCTrajectory` 等）
3. **串联生命周期**：在 `Scenario` 与 `Stage` 基类中实现了状态机骨架（stage pipeline + task pipeline），具体插件只需实现 `Process` 即可
4. **集成 CLASS_LOADER 与 PluginManager**：所有基类都通过 `apollo::cyber::plugin_manager::PluginManager` 按类名动态加载、按类名定位配置路径

### 为什么要单独有 interface_base

Planning 模块在 Apollo 6.0 之后全面插件化，目录被拆分成三层：

```
planning_base/              基础数学、参考线、通用数据结构（无业务逻辑）
planning_interface_base/    插件抽象基类（本文档的主题）
planners/ scenarios/        具体实现（每个实现编译为独立 .so 插件）
tasks/ traffic_rules/
```

这种分层让框架与实现解耦：

- **编译解耦**：`planning_component` 只依赖 `planning_interface_base`，无需在构建期知道有哪些 Planner / Scenario 存在
- **运行时动态加载**：`CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏在 `.so` 被 `dlopen` 时向全局 `PluginManager` 自注册，框架通过类名（如 `"apollo::planning::PublicRoadPlanner"`）反射创建实例
- **配置驱动调度**：Stage 的 task 列表、Scenario 的 stage 列表、TrafficDecider 的 rule 列表均通过 protobuf 配置声明（`ScenarioPipeline` / `StagePipeline` / `TrafficRulesPipeline`），框架按配置顺序依次执行，新增场景或任务无需修改框架代码

### 与 cyber_plugin_manager / class_loader 的集成点

`planning_interface_base` 的每个基类都使用 CyberRT 插件系统的两个核心 API：

| 使用场景 | API | 集成位置 |
|---------|-----|---------|
| 按类名创建插件实例 | `PluginManager::Instance()->CreateInstance<BaseType>(class_name)` | `Scenario::CreateStage`（`scenario_base/scenario.cc:158`）、`Stage::Init`（`scenario_base/stage.cc:62`）、`TrafficDecider::Init`（`traffic_rules_base/traffic_decider.cc:49`） |
| 定位插件默认配置路径 | `PluginManager::Instance()->GetPluginConfPath<BaseType>(class_name, rel_path)` | `Planner::LoadConfig`（`planner_base/planner.h:91`）、`Scenario::Init`（`scenario_base/scenario.cc:66`）、`Task::Init`（`task_base/task.cc:53`）、`TrafficRule::Init`（`traffic_rules_base/traffic_rule.cc:42`） |
| 定位插件根目录 | `PluginManager::Instance()->GetPluginClassHomePath<Scenario>(class_name)` | `Scenario::Init`（`scenario_base/scenario.cc:60`） |

底层 `class_loader_manager.h` 负责 `.so` 加载；`PluginManager` 只是 `ClassLoaderManager` 的高层封装，额外维护了「类名 → 插件包路径 → 配置文件路径」的映射关系。

## planner_base

> 头文件：`modules/planning/planning_interface_base/planner_base/planner.h`

`planner_base` 只包含一个头文件，声明了规划器基类 `Planner` 和带参考线的子基类 `PlannerWithReferenceLine`。它定义了"一个 Planner 应当如何被初始化、被调用、被停止"。

### 类 `Planner`

`Planner` 是所有规划器的根基类（`planner.h:45`）。

```cpp
class Planner {
 public:
  virtual ~Planner() = default;

  virtual std::string Name() = 0;

  virtual apollo::common::Status Init(
      const std::shared_ptr<DependencyInjector>& injector,
      const std::string& config_path = "");

  virtual apollo::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ADCTrajectory* ptr_computed_trajectory) = 0;

  virtual void Stop() = 0;

  virtual void Reset(Frame* frame) {}

  template <typename T>
  bool LoadConfig(const std::string& custom_config_path, T* config);

 protected:
  std::shared_ptr<DependencyInjector> injector_ = nullptr;
};
```

#### 虚函数清单与语义

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `std::string Name()` | `planner.h:52` | 纯虚。返回规划器的可读名称，如 `"PUBLIC_ROAD"`、`"LATTICE"`。用于日志、监控、`PlanningContext` 状态标记 |
| `Status Init(const std::shared_ptr<DependencyInjector>&, const std::string& = "")` | `planner.h:54` | 非纯虚，默认实现只保存 `injector_`。子类可以 override 读取自己的配置（如 `PublicRoadPlanner::Init` 会初始化 `ScenarioManager`）。返回 `Status::OK()` 表示初始化成功 |
| `Status Plan(const TrajectoryPoint&, Frame*, ADCTrajectory*)` | `planner.h:67` | 纯虚。一帧规划的入口；`planning_init_point` 是拼接后的规划起点，`frame` 是当前帧的完整上下文（含参考线、障碍物、历史轨迹），`ptr_computed_trajectory` 是输出轨迹。返回非 OK 则由上层 `OnLanePlanning` 触发 fallback |
| `void Stop()` | `planner.h:71` | 纯虚。进程关闭时调用，用于释放线程、清理异步任务 |
| `void Reset(Frame*)` | `planner.h:73` | 非纯虚，默认空实现。重规划（replan）时由上层触发，清除缓存的内部状态（如 `PublicRoadPlanner` 会调用 `scenario_manager_.Reset(frame)`） |

#### 模板方法 `LoadConfig`

`Planner::LoadConfig`（`planner.h:82-96`）是一个模板成员函数，它的实现利用 RTTI（`abi::__cxa_demangle`）与 `PluginManager` 反查当前插件的默认配置路径：

```cpp
template <typename T>
bool Planner::LoadConfig(const std::string& custom_config_path, T* config) {
  std::string config_path = custom_config_path;
  if ("" == config_path) {
    int status;
    std::string class_name =
        abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
    config_path = apollo::cyber::plugin_manager::PluginManager::Instance()
                      ->GetPluginConfPath<Planner>(
                          class_name, "conf/planner_config.pb.txt");
  }
  return apollo::cyber::common::LoadConfig<T>(config_path, config);
}
```

子类调用 `LoadConfig("", &config_)` 即可加载与自己类名同路径下的 `conf/planner_config.pb.txt`，不用硬编码路径。

### 类 `PlannerWithReferenceLine`

`PlannerWithReferenceLine`（`planner.h:98-118`）继承自 `Planner`，为结构化道路规划器提供了"按参考线规划"的钩子：

```cpp
class PlannerWithReferenceLine : public Planner {
 public:
  virtual ~PlannerWithReferenceLine() = default;

  virtual apollo::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) {
    CHECK_NOTNULL(frame);
    return apollo::common::Status::OK();
  }
};
```

它并未 override `Plan`；子类通常在自己的 `Plan` 实现里遍历 `frame->mutable_reference_line_info()`，逐条调用 `PlanOnReferenceLine`。`PublicRoadPlanner`、`LatticePlanner`、`NaviPlanner` 都继承自此类（`planners/public_road/public_road_planner.h:48`、`planners/lattice/lattice_planner.h`、`planners/navi/navi_planner.h`）。

### 数据契约

| 类型 | 角色 | 来源 |
|------|------|------|
| `common::TrajectoryPoint` | 规划起点（位置、速度、加速度、航向、曲率、相对时间） | 由 `OnLanePlanning` 的轨迹拼接逻辑计算 |
| `Frame*` | 当前帧完整上下文，包含 `LocalView`、参考线、障碍物、EgoInfo、OpenSpaceInfo 等 | 由 `OnLanePlanning::InitFrame` 构造 |
| `ReferenceLineInfo*` | 单条参考线及其派生产物（PathData、SpeedData、Decision） | 由 `Frame` 持有，通常一帧有 1~2 条 |
| `ADCTrajectory*` | 输出轨迹消息（`modules/common_msgs/planning_msgs/planning.proto`） | 规划器填充，由 `PlanningBase` 发布到 `/apollo/planning` |

### 插件注册示例

```cpp
// planners/public_road/public_road_planner.h:80
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PublicRoadPlanner,
                                     Planner)

// planners/lattice/lattice_planner.h:72
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LatticePlanner, Planner)

// planners/navi/navi_planner.h:126
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::NaviPlanner, Planner)

// planners/rtk/rtk_replay_planner.h:93
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RTKReplayPlanner, Planner)
```

宏的作用是将具体类注册到 `PluginManager`，键为完整限定类名；框架通过 `PluginManager::CreateInstance<Planner>("apollo::planning::PublicRoadPlanner")` 反射创建对象。

## scenario_base

> 头文件：`modules/planning/planning_interface_base/scenario_base/`

`scenario_base` 定义了驾驶场景（Scenario）与场景阶段（Stage）的状态机骨架，以及若干可复用的"蠕动 / 巡航"阶段基类。

### 文件结构

```
scenario_base/
├── scenario.h / scenario.cc              # Scenario 基类 + ScenarioContext
├── stage.h / stage.cc                    # Stage 基类 + task pipeline 执行器
├── process_result.h / process_result.cc  # StageResult / ScenarioResult / 状态枚举
├── base_stage_creep.h / .cc              # 蠕动通过路口/停止线的共享逻辑
├── base_stage_cruise.h / .cc             # 通过路口后检查是否应退出的共享逻辑
├── traffic_light_base/
│   ├── base_stage_traffic_light_creep.h / .cc
│   └── base_stage_traffic_light_cruise.h / .cc
└── proto/
    ├── scenario_pipeline.proto           # StagePipeline / ScenarioPipeline
    └── creep_stage.proto                 # CreepStageConfig
```

### 类 `ScenarioContext`

`ScenarioContext`（`scenario.h:44-47`）是一个空基结构体，用于让每个 Scenario 子类定义自己的运行时上下文。

```cpp
struct ScenarioContext {
 public:
  ScenarioContext() {}
};
```

具体场景继承此结构体以携带自己的状态，例如 `ValetParkingContext`（`scenarios/valet_parking/valet_parking_scenario.h:36`）：

```cpp
struct ValetParkingContext : public ScenarioContext {
  ScenarioValetParkingConfig scenario_config;
  std::string target_parking_spot_id;
  bool pre_stop_rightaway_flag = false;
  hdmap::MapPathPoint pre_stop_rightaway_point;
};
```

`Stage` 通过 `Stage::GetContextAs<T>()`（`stage.h:67`）取回当前 Scenario 的上下文指针，实现跨 Stage 的数据共享。

### 类 `Scenario`

`Scenario`（`scenario.h:52-119`）是所有驾驶场景的基类。

```cpp
class Scenario {
 public:
  Scenario();
  virtual ~Scenario() = default;

  virtual bool Init(std::shared_ptr<DependencyInjector> injector,
                    const std::string& name);

  virtual ScenarioContext* GetContext() = 0;

  virtual bool IsTransferable(const Scenario* other_scenario,
                              const Frame& frame) { return false; }

  virtual ScenarioResult Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame);

  virtual bool Exit(Frame* frame) { return true; }

  virtual bool Enter(Frame* frame) { return true; }

  std::shared_ptr<Stage> CreateStage(const StagePipeline& stage_pipeline);

  const ScenarioStatusType& GetStatus() const;
  const std::string GetStage() const;
  const std::string& GetMsg() const;
  const std::string& Name() const;
  void Reset();

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  ScenarioResult scenario_result_;
  std::shared_ptr<Stage> current_stage_;
  std::unordered_map<std::string, const StagePipeline*> stage_pipeline_map_;
  std::string msg_;
  std::shared_ptr<DependencyInjector> injector_;
  std::string config_path_;
  std::string config_dir_;
  std::string name_;
  ScenarioPipeline scenario_pipeline_config_;
};
```

#### 虚函数清单

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `bool Init(std::shared_ptr<DependencyInjector>, const std::string& name)` | `scenario.h:58` / `scenario.cc:43` | 非纯虚。默认实现：①保存 `injector_`、`name_`；②向 `PlanningContext` 写入当前场景名；③通过 `PluginManager` 推导 `config_dir_`、`config_path_`；④读取 `conf/pipeline.pb.txt`（`ScenarioPipeline`）到 `scenario_pipeline_config_`；⑤把每个 `StagePipeline` 放进 `stage_pipeline_map_`。子类通常先调用基类再读取自己的 `conf/scenario_conf.pb.txt` |
| `ScenarioContext* GetContext()` | `scenario.h:64` | 纯虚。返回派生类具体的 `XxxContext*`，供 Stage 获取 |
| `bool IsTransferable(const Scenario* other_scenario, const Frame&)` | `scenario.h:70` | 非纯虚，默认返回 `false`。定义"从 `other_scenario` 切换到自己"的转入条件；`ScenarioManager` 按优先级遍历候选场景依次调用此函数 |
| `ScenarioResult Process(const TrajectoryPoint&, Frame*)` | `scenario.h:75` / `scenario.cc:86` | 非纯虚。驱动内部 Stage 状态机（详见下节）。通常不需要 override |
| `bool Enter(Frame*)` | `scenario.h:80` | 非纯虚，默认返回 `true`。切入该场景时调用，子类可在此初始化上下文 |
| `bool Exit(Frame*)` | `scenario.h:78` | 非纯虚，默认返回 `true`。切出该场景时调用，子类可在此清理状态 |

#### 非虚函数

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `std::shared_ptr<Stage> CreateStage(const StagePipeline&)` | `scenario.h:88` / `scenario.cc:155` | 通过 `PluginManager::CreateInstance<Stage>(full_class_name)` 创建 Stage 实例并调用其 `Init` |
| `const ScenarioStatusType& GetStatus()` | `scenario.h:90` | 返回当前场景执行状态 |
| `const std::string GetStage()` | `scenario.h:94` / `scenario.cc:170` | 返回 `current_stage_` 的名字，未创建则返回空串 |
| `const std::string& GetMsg()` | `scenario.h:96` | 返回调试消息 |
| `const std::string& Name()` | `scenario.h:98` | 返回场景名 |
| `void Reset()` | `scenario.h:103` / `scenario.cc:174` | 重置场景：清空 `scenario_result_`、`current_stage_`。切入场景前由 `ScenarioManager` 调用 |
| `template<T> bool LoadConfig(T*)` | `scenario.h:122` | 从 `config_path_` 加载 proto 配置 |

#### `Scenario::Process` 的状态机语义

`Process`（`scenario.cc:86-153`）是 Scenario 状态机的心脏，它的执行流程：

1. **冷启动**：若 `current_stage_ == nullptr`，用 `stage_pipeline_map_` 中的第一个 Stage 调用 `CreateStage`；创建失败则 `ScenarioResult.stage_status = ERROR`
2. **哨兵**：若 `current_stage_->Name().empty()`（被 `FinishScenario` 标记），直接返回 `STATUS_DONE`
3. **分发**：调用 `current_stage_->Process(planning_init_point, frame)` 得到 `StageResult`
4. **状态转移**：根据 `ret.GetStageStatus()`：

   | `StageStatusType` | 对应 `ScenarioStatusType` | 动作 |
   |-------------------|---------------------------|------|
   | `ERROR` | `STATUS_UNKNOWN` | 日志 + 维持当前 Stage |
   | `RUNNING` | `STATUS_PROCESSING` | 维持当前 Stage |
   | `FINISHED` | 见下 | 读取 `current_stage_->NextStage()` 决定后续 |
   | 其他 | `STATUS_UNKNOWN` | 异常分支 |

5. **Stage 切换**：`FINISHED` 时，若 `next_stage` 与当前同名则继续；若 `next_stage` 为空则 `STATUS_DONE`（场景结束）；若 `next_stage` 不在 `stage_pipeline_map_` 则 `STATUS_UNKNOWN`；否则 `CreateStage` 切换

### 类 `Stage`

`Stage`（`stage.h:46-96`）是场景阶段的基类。

```cpp
class Stage {
 public:
  Stage();
  virtual ~Stage() = default;

  virtual bool Init(const StagePipeline& config,
                    const std::shared_ptr<DependencyInjector>& injector,
                    const std::string& config_dir, void* context);

  virtual StageResult Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  const std::string& Name() const;

  template <typename T>
  T* GetContextAs() const { return static_cast<T*>(context_); }

  const std::string& NextStage() const { return next_stage_; }

 protected:
  StageResult ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);
  StageResult ExecuteTaskOnReferenceLineForOnlineLearning(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);
  StageResult ExecuteTaskOnOpenSpace(Frame* frame);

  virtual StageResult FinishScenario();

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                       const std::string& name, const double time_diff_ms);

  std::vector<std::shared_ptr<Task>> task_list_;
  std::shared_ptr<Task> fallback_task_;
  std::string next_stage_;
  void* context_;
  std::shared_ptr<DependencyInjector> injector_;
  StagePipeline pipeline_config_;

 private:
  std::string name_;
};
```

#### 虚函数清单

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `bool Init(const StagePipeline&, const std::shared_ptr<DependencyInjector>&, const std::string& config_dir, void* context)` | `stage.h:50` / `stage.cc:43` | 非纯虚。默认实现：①保存 pipeline、`injector`、`context`、`name`；②向 `PlanningContext` 写入当前 Stage 名；③按 `pipeline_config_.task()` 逐个调用 `PluginManager::CreateInstance<Task>` 并 `task_ptr->Init`，塞入 `task_list_`；④创建 `fallback_task_`（若未配置则默认用 `"FastStopTrajectoryFallback"`）。将 `next_stage_` 初始化为当前 Stage 名（防止未显式 SetNextStage 时死循环后退） |
| `StageResult Process(const TrajectoryPoint&, Frame*)` | `stage.h:61` | 纯虚。阶段业务逻辑入口；子类通常先做本阶段判定，再调用保护的 `ExecuteTaskOnReferenceLine` 等 task 执行器 |
| `StageResult FinishScenario()` | `stage.h:82` / `stage.cc:258` | 非纯虚。子类在阶段结束时调用：把 `next_stage_ = ""`，返回 `StageStatusType::FINISHED`。Scenario 检测到 `next_stage_` 空串即判定整个场景结束 |

#### 非虚 / 保护函数

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `StageResult ExecuteTaskOnReferenceLine(const TrajectoryPoint&, Frame*)` | `stage.h:74` / `stage.cc:101` | 按 `task_list_` 顺序遍历每条可行驶参考线，依次调用 `task->Execute(frame, &reference_line_info)`；任一 task 失败则调用 `fallback_task_->Execute`。成功后用 `CombinePathAndSpeedProfile` 合成 `DiscretizedTrajectory` 并写回 `reference_line_info` |
| `StageResult ExecuteTaskOnReferenceLineForOnlineLearning(...)` | `stage.h:77` / `stage.cc:162` | 学习型规划专用：只取第一条参考线，用 `AdjustTrajectoryWhichStartsFromCurrentPos` 裁切模型输出的未来轨迹 |
| `StageResult ExecuteTaskOnOpenSpace(Frame*)` | `stage.h:80` / `stage.cc:206` | 开放空间（泊车）专用：依次 `task->Execute(frame)`（单参数版本），结束后根据 `open_space_info().fallback_flag()` 选择 `fallback_trajectory` 或 `chosen_partitioned_trajectory` |
| `void RecordDebugInfo(ReferenceLineInfo*, const std::string& name, double)` | `stage.h:84` / `stage.cc:263` | 将每个 task 耗时写入 `reference_line_info->mutable_latency_stats()`，供 PnC monitor 展示 |
| `const std::string& Name() const` | `stage.h:64` / `stage.cc:99` | 返回 Stage 配置中的 `name`（非类名） |
| `const std::string& NextStage()` | `stage.h:71` | 读取当前期望跳转到的下一 Stage 名 |
| `template<T> T* GetContextAs() const` | `stage.h:67` | 将 `void* context_`（由 `Scenario::GetContext()` 传入）强转为具体 Context 类型 |

#### Stage 生命周期

1. **创建**：`Scenario::Process` 中 `CreateStage` → `PluginManager::CreateInstance<Stage>` → 调用 `Stage::Init`
2. **Init 阶段装配 Task**：读取 `StagePipeline` 中的 `task` 列表，通过 `PluginManager` 逐个加载为 `Task` 插件，调用 `Task::Init` 传入 `task_config_dir`（由 Stage 名转成下划线路径）
3. **每帧驱动**：Scenario 每调用一次 `Stage::Process`，Stage 内部依次执行 task 链；子类通过 `next_stage_ = "..."` 申请切换，或通过 `FinishScenario()` 申请结束
4. **切换**：Scenario 读取 `NextStage()`；若变化则销毁当前 Stage，创建新 Stage
5. **结束**：`FinishScenario` 将 `next_stage_` 置空，Scenario 下一帧检测到后进入 `STATUS_DONE`

### `process_result.h` — 状态枚举与结果

文件 `scenario_base/process_result.h` 定义了场景/阶段的状态枚举与结果封装。

#### 枚举

```cpp
enum class StageStatusType {
  ERROR = 1,
  READY = 2,
  RUNNING = 3,
  FINISHED = 4,
};

enum class ScenarioStatusType {
  STATUS_UNKNOWN = 0,
  STATUS_PROCESSING = 1,
  STATUS_DONE = 2,
};
```

#### 类 `StageResult`

位置：`process_result.h:52-99`

| 函数 | 语义 |
|------|------|
| `StageResult(StageStatusType = READY, common::Status = OK())` | 构造 |
| `const common::Status& GetTaskStatus() const` | 取内层 task status（用于判断业务层错误） |
| `const StageStatusType& GetStageStatus() const` | 取 stage 状态枚举 |
| `const StageResult& SetTaskStatus(const common::Status&)` | 设置 task status |
| `const StageResult& SetStageStatus(StageStatusType)` | 设置 stage 状态 |
| `const StageResult& SetStageStatus(StageStatusType, const std::string& message)` | 设置 stage 状态并附带消息 |
| `bool HasError() const` | stage_status 是否为 `ERROR` |
| `bool IsTaskError() const` | task_status 是否非 OK |

#### 类 `ScenarioResult`

位置：`process_result.h:106-156`

| 函数 | 语义 |
|------|------|
| `ScenarioResult(ScenarioStatusType = UNKNOWN, StageStatusType = READY, common::Status = OK())` | 构造 |
| `const common::Status& GetTaskStatus()` | 透传给内含的 `StageResult` |
| `const StageStatusType& GetStageStatus()` | 透传 |
| `const ScenarioStatusType& GetScenarioStatus()` | 取场景级状态 |
| `const ScenarioResult& SetStageResult(const StageResult&)` | 用 StageResult 覆盖 |
| `const ScenarioResult& SetStageResult(StageStatusType, const std::string&)` | 构造一个新 StageResult 并覆盖 |
| `const ScenarioResult& SetScenarioStatus(ScenarioStatusType)` | 直接设置场景状态 |

### `StagePipeline` / `ScenarioPipeline` 配置

位置：`scenario_base/proto/scenario_pipeline.proto`

```proto
message StagePipeline {
  required string name = 1;            // Stage 别名，也是配置子目录名
  required string type = 2;            // Stage 插件类名（类限定名）
  optional bool enabled = 3;           // 是否启用
  repeated PluginDeclareInfo task = 4; // Stage 内的 Task 序列
  optional PluginDeclareInfo fallback_task = 5; // 失败兜底 Task
}

message ScenarioPipeline {
  repeated StagePipeline stage = 1;    // Scenario 的 Stage 序列
}
```

`Scenario::Init` 读取 `conf/pipeline.pb.txt` 反序列化为 `ScenarioPipeline`；`Stage::Init` 接受其中一个 `StagePipeline` 并据此加载 Task。

### 辅助基类：`BaseStageCreep` / `BaseStageCruise`

`base_stage_creep.h:41` 定义了通过停止线前蠕动（Creep）的通用逻辑：

```cpp
class BaseStageCreep : public Stage {
 public:
  common::Status ProcessCreep(Frame* frame,
                              ReferenceLineInfo* reference_line_info) const;

  bool CheckCreepDone(const Frame& frame,
                      const ReferenceLineInfo& reference_line_info,
                      const double stop_sign_overlap_end_s,
                      const double wait_time_sec, const double timeout_sec);

  double GetCreepFinishS(double overlap_end_s, const Frame& frame,
                         const ReferenceLineInfo& reference_line_info) const;

 private:
  double GetCreepTargetS(double overlap_end_s, const Frame& frame,
                         const ReferenceLineInfo& reference_line_info) const;

  virtual const CreepStageConfig& GetCreepStageConfig() const = 0;

  virtual bool GetOverlapStopInfo(Frame* frame,
                                  ReferenceLineInfo* reference_line_info,
                                  double* overlap_end_s,
                                  std::string* overlap_id) const = 0;

  static constexpr const char* CREEP_VO_ID_PREFIX_ = "CREEP_";
};
```

| 函数 | 语义（`base_stage_creep.cc`） |
|------|-----------------------------|
| `ProcessCreep` | `base_stage_creep.cc:41`。获取当前 overlap 终点 s，用 `util::BuildStopDecision` 在 `CREEP_<overlap_id>` 虚拟障碍物上设置蠕动目标停止点 |
| `CheckCreepDone` | `base_stage_creep.cc:79`。若已越过 `GetCreepFinishS` 或等待超时，则检查所有动态障碍物的 ST 边界是否足够远；连续 5 帧「全部远离」则判定蠕动完成，并将计数器写回 `PlanningContext.creep_decider()`（多车场景安全的做法） |
| `GetCreepTargetS` | `base_stage_creep.cc:65`。目标 s = `overlap_end_s + 4.0` |
| `GetCreepFinishS` | `base_stage_creep.cc:72`。完成判定 s = `GetCreepTargetS - 2.0` |
| `GetCreepStageConfig()` | 纯虚。子类返回自己 Scenario 配置里的 `CreepStageConfig`（`proto/creep_stage.proto`） |
| `GetOverlapStopInfo(...)` | 纯虚。子类根据 overlap 类型（信号灯 / 停止标志等）返回 overlap 终点 s 与 id |

`CreepStageConfig`（`proto/creep_stage.proto`）字段：

```proto
message CreepStageConfig {
  optional double min_boundary_t = 1 [default = 6.0];       // 障碍物 st 边界最小关注 t（秒）
  optional double ignore_max_st_min_t = 2 [default = 0.1];  // 忽略同向运动障碍的最大 min_t
  optional double ignore_min_st_min_s = 3 [default = 15.0]; // 忽略同向障碍的最小 min_s（米）
}
```

`base_stage_cruise.h:36` 定义了穿过路口后判定"是否已驶离"的 `BaseStageCruise`：

```cpp
class BaseStageCruise : public Stage {
 public:
  bool CheckDone(const Frame& frame, const PlanningContext* context,
                 const bool right_of_way_status);

 private:
  virtual hdmap::PathOverlap* GetTrafficSignOverlap(
      const ReferenceLineInfo& reference_line_info,
      const PlanningContext* context) const = 0;
};
```

`CheckDone`（`base_stage_cruise.cc:34`）的逻辑：

- 若参考线上没有 `junction_overlaps`，则查交通标志 overlap；若 ADC 后边缘越过 overlap 终点 ≥ 20 m，判定已通过
- 否则检查 ADC 是否仍在路口内；不在则判定完成
- 所有分支都会调用 `SetJunctionRightOfWay` 更新路权状态

### 信号灯专用阶段：`traffic_light_base/`

`BaseStageTrafficLightCreep`（`base_stage_traffic_light_creep.h:31`）继承自 `BaseStageCreep`，它实现了 `GetOverlapStopInfo`：从 `PlanningContext.traffic_light().current_traffic_light_overlap_id()` 取当前信号灯 overlap，定位 `reference_line_info` 上的 `SIGNAL` 类型 overlap，返回其 end_s（`base_stage_traffic_light_creep.cc:28`）。

`BaseStageTrafficLightCruise`（`base_stage_traffic_light_cruise.h:31`）继承自 `BaseStageCruise`，它实现了 `GetTrafficSignOverlap` 返回信号灯 overlap。

这两个类是典型的"复用 creep / cruise 通用逻辑、只 override 与信号灯相关的 overlap 查询"的例子，`scenarios/traffic_light_protected/` 和 `scenarios/traffic_light_unprotected_*/` 在此之上构建具体的 Stage。

### 插件注册示例

Scenario、Stage 都通过同一个宏注册：

```cpp
// scenarios/valet_parking/valet_parking_scenario.h:71
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ValetParkingScenario,
                                     Scenario)

// scenarios/valet_parking/stage_parking.h:43
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StageParking, Stage)

// scenarios/traffic_light_unprotected_right_turn/stage_creep.h:56
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::TrafficLightUnprotectedRightTurnStageCreep, Stage)
```

宏第二个参数是基类类型（`Scenario` 或 `Stage`），框架依赖它来分类 `CreateInstance<T>`。

## task_base

> 头文件：`modules/planning/planning_interface_base/task_base/`

`task_base` 定义了 Planning 中最小可插拔计算单元 `Task` 及其四种典型子分类。

### 类 `Task`

`Task`（`task.h:37-64`）是所有规划任务的根基类。

```cpp
class Task {
 public:
  Task();
  virtual ~Task() = default;

  const std::string& Name() const;

  virtual bool Init(const std::string& config_dir, const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector);

  virtual common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

  virtual common::Status Execute(Frame* frame);

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  Frame* frame_;
  ReferenceLineInfo* reference_line_info_;
  std::shared_ptr<DependencyInjector> injector_;
  std::string config_path_;
  std::string default_config_path_;
  std::string name_;
};
```

#### 虚函数清单

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `bool Init(const std::string& config_dir, const std::string& name, const std::shared_ptr<DependencyInjector>&)` | `task.h:45` / `task.cc:40` | 非纯虚。默认实现：①保存 `injector_`、`name_`；②用 `config_dir + "/" + ConfigUtil::TransformToPathName(name) + ".pb.txt"` 组合出 Stage 级覆盖配置路径 `config_path_`；③通过 `PluginManager` + RTTI 取出插件默认配置 `default_config_path_`（`conf/default_conf.pb.txt`）。子类在重写 `Init` 时通常先调用基类再调 `LoadConfig(&config_)` |
| `common::Status Execute(Frame*, ReferenceLineInfo*)` | `task.h:48` / `task.cc:60` | 非纯虚。默认实现仅保存 `frame_`、`reference_line_info_` 指针并返回 OK。子类（或 `Decider` / `PathGeneration` / `SpeedOptimizer` 等中间基类）override 此函数并在调用基类后再分发到 `Process` |
| `common::Status Execute(Frame*)` | `task.h:51` / `task.cc:66` | 非纯虚。开放空间（泊车）任务用此重载，只保存 `frame_`。两种 `Execute` 共存是为了兼容结构化道路（需要 reference line）和开放空间（不需要）两种调用方式 |

#### 模板方法 `LoadConfig`

`LoadConfig`（`task.h:66-70`）调用 `ConfigUtil::LoadMergedConfig`，以 Stage 级覆盖文件合并插件默认配置：

```cpp
template <typename T>
bool Task::LoadConfig(T* config) {
  return ConfigUtil::LoadMergedConfig(default_config_path_, config_path_,
                                      config);
}
```

这允许同一个 Task 插件在不同 Stage 中携带不同参数。

### 子基类 `Decider`

`Decider`（`decider.h:32-50`）用于"只产出决策、不直接修改 path/speed"的任务。

```cpp
class Decider : public Task {
 public:
  virtual ~Decider() = default;

  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  apollo::common::Status Execute(Frame* frame) override;

 protected:
  virtual apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) {
    return apollo::common::Status::OK();
  }

  virtual apollo::common::Status Process(Frame* frame) {
    return apollo::common::Status::OK();
  }
};
```

`Decider::Execute` 的实现（`decider.cc:28`）：

```cpp
Status Decider::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(frame, reference_line_info);
}

Status Decider::Execute(Frame* frame) {
  Task::Execute(frame);
  return Process(frame);
}
```

子类实现 `Process` 即可，不必再操心参数保存与基类分发。典型实现：`tasks/path_decider/path_decider.h`、`tasks/st_bounds_decider/st_bounds_decider.h`、`tasks/rule_based_stop_decider/rule_based_stop_decider.h`。

### 子基类 `PathGeneration`

`PathGeneration`（`path_generation.h:31-82`）用于"产出 Frenet / Cartesian 路径"的任务。

```cpp
class PathGeneration : public Task {
 public:
  virtual ~PathGeneration() = default;

  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;
  apollo::common::Status Execute(Frame* frame) override;

 protected:
  virtual apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info);
  virtual apollo::common::Status Process(Frame* frame);

  void GetStartPointSLState();

  void RecordDebugInfo(const PathBound& path_boundaries,
                       const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);
  void RecordDebugInfo(const PathData& path_data, const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);

  bool GetSLBoundary(const PathData& path_data, int point_index,
                     const ReferenceLineInfo* reference_line_info,
                     SLBoundary* const sl_boundary);

  SLState init_sl_state_;
};
```

#### 关键函数

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `Execute(Frame*, ReferenceLineInfo*)` | `path_generation.cc:31` | 调用 `Task::Execute` 再调 `Process` |
| `Execute(Frame*)` | `path_generation.cc:37` | 开放空间重载 |
| `Process(...)` | 默认返回 OK | 子类必须 override |
| `GetStartPointSLState()` | `path_generation.cc:107` | 将 `frame->PlanningStartPoint()` 投影到参考线 Frenet 坐标系，结果写入 `init_sl_state_`；若 `FLAGS_use_front_axe_center_in_path_planning` 为真，会先把起点从后轴中心平移到前轴中心 |
| `RecordDebugInfo(const PathBound&, ...)` | `path_generation.cc:42` | 把左右 path boundary 转成两条 `FrenetFramePath` 写入 debug proto，供 DreamView 可视化 |
| `RecordDebugInfo(const PathData&, ...)` | `path_generation.cc:97` | 把候选路径写入 debug proto |
| `GetSLBoundary(...)` | `path_generation.cc:134` | 基于车辆几何（`VehicleConfigHelper` 查 length、width、back_edge_to_center）构造 `Box2d`，再调用 `ReferenceLine::GetSLBoundary` 得到 SL 边界 |

### 子基类 `SpeedOptimizer`

`SpeedOptimizer`（`speed_optimizer.h:31-45`）是速度规划任务的基类。

```cpp
class SpeedOptimizer : public Task {
 public:
  virtual ~SpeedOptimizer() = default;

  common::Status Execute(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

 protected:
  virtual common::Status Process(const PathData& path_data,
                                 const common::TrajectoryPoint& init_point,
                                 SpeedData* const speed_data) = 0;

  void RecordDebugInfo(const SpeedData& speed_data);
  void RecordDebugInfo(const SpeedData& speed_data,
                       planning_internal::STGraphDebug* st_graph_debug);
};
```

`Execute` 的实现（`speed_optimizer.cc:32`）：

```cpp
Status SpeedOptimizer::Execute(Frame* frame,
                               ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = Process(reference_line_info->path_data(),
                     frame->PlanningStartPoint(),
                     reference_line_info->mutable_speed_data());
  RecordDebugInfo(reference_line_info->speed_data());
  return ret;
}
```

与 `Decider` / `PathGeneration` 不同，`SpeedOptimizer::Process` 是**纯虚**，并且签名直接暴露 `PathData`、起点、`SpeedData*` 三个参数，隐藏了 `Frame` / `ReferenceLineInfo` 细节。`RecordDebugInfo` 会把速度曲线塞进 `ReferenceLineInfo::debug()`，第二个重载可额外写 ST 图用于调试。

### 子基类 `TrajectoryOptimizer`

`TrajectoryOptimizer`（`trajectory_optimizer.h:34-42`）是开放空间（泊车等）轨迹优化任务的基类。

```cpp
class TrajectoryOptimizer : public Task {
 public:
  virtual ~TrajectoryOptimizer() = default;

  apollo::common::Status Execute(Frame* frame) override;

 protected:
  virtual apollo::common::Status Process() = 0;
};
```

`Execute` 的实现（`trajectory_optimizer.cc:30`）只调用 `Task::Execute(frame)` 再调 `Process()`；子类的 `Process` 不接收参数，直接通过成员 `frame_` 读取开放空间信息，典型用于 `OpenSpaceTrajectoryProvider` 等。

### 子基类 `TrajectoryFallbackTask`

`TrajectoryFallbackTask`（`trajectory_fallback_task.h:31-56`）是 Stage 级兜底任务的基类。

```cpp
class TrajectoryFallbackTask : public Task {
 public:
  common::Status Execute(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

 private:
  virtual void GenerateFallbackPath(Frame* frame,
                                    ReferenceLineInfo* reference_line_info);

  virtual SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                          const double stop_distance = 0.0) = 0;

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,
                                   PathData* path_data);

  bool RetrieveLastFramePathProfile(
      const ReferenceLineInfo* reference_line_info, const Frame* frame,
      PathData* path_data);

  void AmendSpeedDataForControl(SpeedData* speed_data_ptr);
};
```

`Execute`（`trajectory_fallback_task.cc:35`）的执行语义：

1. 先调用 `GenerateFallbackPath`：若 `PathData` 为空，就调 `GenerateFallbackPathProfile` 生成兜底路径并给 `ReferenceLineInfo` 加 `kPathOptimizationFallbackCost = 2e4`，`trajectory_type = PATH_FALLBACK`；否则尝试复用上一帧路径或当前帧的 `self` 候选路径
2. 若 `SpeedData` 为空，调用子类实现的纯虚 `GenerateFallbackSpeed` 生成速度曲线，并用 `AmendSpeedDataForControl` 平滑减速以便控制模块跟踪
3. 若非 PATH_FALLBACK，打上 `SPEED_FALLBACK` 并加 `kSpeedOptimizationFallbackCost = 2e4`

`Stage::Init` 在配置未声明 `fallback_task` 时，默认会加载插件 `"FastStopTrajectoryFallback"` 作为兜底（`stage.cc:77-81`）。

### 辅助工具：`lane_change_util` / `path_util` / `st_gap_estimator`

`task_base/common/` 下还有若干**非插件**的静态工具类，供 Task 实现复用。它们没有 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN`，也不继承 `Task`，只是以函数库形式暴露：

| 头文件 | 作用 |
|--------|------|
| `common/lane_change_util/lane_change_util.h` | 变道决策所需的几何/几何与时序通用函数 |
| `common/path_util/path_assessment_decider_util.h` | 路径评估与决策通用函数（代价、可行性判定） |
| `common/path_util/path_bounds_decider_util.h` | 路径边界（PathBound）构造与裁剪 |
| `common/path_util/path_optimizer_util.h` | 路径优化共享的数据变换（SL ↔ XY、边界 shrink） |
| `utils/st_gap_estimator.h` | 安全跟车、让行、超车距离估计 |

`StGapEstimator`（`utils/st_gap_estimator.h:25`）为全静态类：

```cpp
class StGapEstimator {
 public:
  StGapEstimator() = delete;
  virtual ~StGapEstimator() = delete;

  static double EstimateSafeOvertakingGap();
  static double EstimateSafeFollowingGap(const double target_obs_speed);
  static double EstimateSafeYieldingGap();
  static double EstimateProperOvertakingGap(const double target_obs_speed,
                                            const double adc_speed);
  static double EstimateProperFollowingGap(const double adc_speed);
  static double EstimateProperYieldingGap();
};
```

### 插件注册示例

```cpp
// tasks/path_decider/path_decider.h:60
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PathDecider, Task)

// tasks/st_bounds_decider/st_bounds_decider.h:96
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::STBoundsDecider, Task)

// tasks/reuse_path/reuse_path.h:80
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ReusePath, Task)

// tasks/lane_change_path/lane_change_path.h:108
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneChangePath, Task)

// tasks/pull_over_path/pull_over_path.h:107
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PullOverPath, Task)
```

所有 Task 子类（无论 `Decider`、`PathGeneration`、`SpeedOptimizer` 还是 `TrajectoryOptimizer`）都以基类 `Task` 作为宏的第二参数 —— `Stage::Init` 在创建时按 `CreateInstance<Task>` 反射，不区分更细的子基类。

## traffic_rules_base

> 头文件：`modules/planning/planning_interface_base/traffic_rules_base/`

`traffic_rules_base` 定义了交通规则插件 `TrafficRule` 基类，以及将多个规则串成一条执行流水线的 `TrafficDecider` 宿主类。

### 类 `TrafficRule`

`TrafficRule`（`traffic_rule.h:37-61`）是所有交通规则插件的基类。

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
  template <typename T>
  bool LoadConfig(T* config);

  std::shared_ptr<DependencyInjector> injector_;
  std::string config_path_;
  std::string name_;
};
```

#### 虚函数清单

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `bool Init(const std::string& name, const std::shared_ptr<DependencyInjector>&)` | `traffic_rule.h:43` / `traffic_rule.cc:32` | 非纯虚。默认实现：①保存 `injector_`、`name_`；②通过 RTTI + `PluginManager` 推导 `config_path_` = `<plugin>/conf/default_conf.pb.txt`。子类 override 时通常先调用基类再 `LoadConfig(&config_)` |
| `common::Status ApplyRule(Frame* const, ReferenceLineInfo* const)` | `traffic_rule.h:46` | 纯虚。每条参考线上调用一次；规则根据 `frame` / `reference_line_info` 为相关障碍物（信号灯、停止标志虚拟墙等）设置纵向决策，或在参考线上增加虚拟停止墙 |
| `void Reset()` | `traffic_rule.h:49` | 纯虚。每帧开始时由 `TrafficDecider::Execute` 调用，用于清除规则上一帧缓存的状态 |

#### 非虚

| 函数 | 语义 |
|------|------|
| `std::string Getname()` | 返回规则名 |
| `template<T> bool LoadConfig(T*)` | 加载 `config_path_` 对应的 proto 配置 |

#### `TrafficRule` 子类的典型 override

以 `TrafficLight` 为例（`traffic_rules/traffic_light/traffic_light.h:32`）：

```cpp
class TrafficLight : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~TrafficLight() = default;
  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  void Reset() override {}

 private:
  TrafficLightConfig config_;
  ...
};
```

规则基本模式：`Init` 读取配置，`ApplyRule` 扫描 overlap 或 obstacle 并 `BuildStopDecision`，`Reset` 清空成员缓存。

### 类 `TrafficDecider`

`TrafficDecider`（`traffic_decider.h:47-60`）是在 `OnLanePlanning` 中把一组 `TrafficRule` 插件按 pipeline 顺序执行的宿主。

```cpp
class TrafficDecider {
 public:
  TrafficDecider() = default;
  bool Init(const std::shared_ptr<DependencyInjector>& injector);
  virtual ~TrafficDecider() = default;
  apollo::common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

 private:
  bool init_ = false;
  void BuildPlanningTarget(ReferenceLineInfo* reference_line_info);
  std::vector<std::shared_ptr<TrafficRule>> rule_list_;
  TrafficRulesPipeline rule_pipeline_;
};
```

#### 函数清单

| 函数 | 签名位置 | 语义 |
|------|---------|------|
| `bool Init(const std::shared_ptr<DependencyInjector>&)` | `traffic_decider.h:50` / `traffic_decider.cc:34` | ①防重入 `init_` flag；②从 `FLAGS_traffic_rule_config_filename` 加载 `TrafficRulesPipeline`；③对 pipeline 中每条 rule 用 `PluginManager::CreateInstance<TrafficRule>` 创建并 `rule->Init(name, injector)`；④push 进 `rule_list_` |
| `common::Status Execute(Frame*, ReferenceLineInfo*)` | `traffic_decider.h:52` / `traffic_decider.cc:105` | 对每条规则调用 `rule->Reset()` 再 `rule->ApplyRule(frame, reference_line_info)`；最后调用 `BuildPlanningTarget` |
| `void BuildPlanningTarget(ReferenceLineInfo*)` | `traffic_decider.h:57` / `traffic_decider.cc:64` | 遍历所有虚拟障碍物，找最近的带 `stop` 纵向决策的虚障；根据 `StopReasonCode` 区分 `HARD` / `SOFT` 停车类型，最终通过 `SetLatticeStopPoint` 告诉 Planner 当前帧的纵向目标 |

识别为 `HARD` 停车的 `StopReasonCode`：`DESTINATION`、`CROSSWALK`、`STOP_SIGN`、`YIELD_SIGN`、`CREEPER`、`REFERENCE_END`、`SIGNAL`；`YELLOW_SIGNAL` 识别为 `SOFT`。

### `TrafficRulesPipeline` 配置

位置：`traffic_rules_base/proto/traffic_rules_pipeline.proto`

```proto
message TrafficRulesPipeline {
  repeated PluginDeclareInfo rule = 1;
}
```

每个 `PluginDeclareInfo` 包含 `name`（别名）与 `type`（类名）。该文件通过 gflag `FLAGS_traffic_rule_config_filename` 指定路径（默认见 `planning_component/conf/traffic_rule_config.pb.txt`）。

### 插件注册示例

所有内置交通规则都通过同一宏注册：

```cpp
// traffic_rules/traffic_light/traffic_light.h:50
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::TrafficLight, TrafficRule)

// traffic_rules/stop_sign/stop_sign.h:52
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StopSign, TrafficRule)

// traffic_rules/destination/destination.h:53
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::Destination, TrafficRule)

// traffic_rules/crosswalk/crosswalk.h:60
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::Crosswalk, TrafficRule)

// traffic_rules/yield_sign/yield_sign.h:53
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::YieldSign, TrafficRule)

// traffic_rules/keepclear/keep_clear.h:61
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::KeepClear, TrafficRule)

// traffic_rules/rerouting/rerouting.h:56
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::Rerouting, TrafficRule)

// traffic_rules/reference_line_end/reference_line_end.h:50
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ReferenceLineEnd,
                                     TrafficRule)

// traffic_rules/backside_vehicle/backside_vehicle.h:60
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::BacksideVehicle,
                                     TrafficRule)

// traffic_rules/speed_setting/speed_setting.h:44
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::SpeedSetting, TrafficRule)
```

## 插件注册机制汇总

### 宏 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN`

宏由 `cyber/plugin_manager/plugin_manager.h` 提供，语义等价于：

```
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::Xxx, BaseType)
```

它在编译期把 `Xxx` 类型作为 `BaseType` 的派生插件登记到插件库的静态注册表；当 `.so` 被 `PluginManager::LoadPlugin("xxx_plugins.xml")` 加载时，注册表被合并到全局 `PluginManager` 单例。之后：

```cpp
auto ptr = PluginManager::Instance()->CreateInstance<BaseType>(
    "apollo::planning::Xxx");
```

即可得到 `std::shared_ptr<BaseType>`（底层用 `dynamic_cast`）。

### Planning 五类插件的基类标记

| 基类（宏第二参数） | 定义位置 | 使用者 |
|----------------|---------|--------|
| `Planner` | `planning_interface_base/planner_base/planner.h:45` | `PublicRoadPlanner`、`LatticePlanner`、`NaviPlanner`、`RTKReplayPlanner` |
| `Scenario` | `planning_interface_base/scenario_base/scenario.h:52` | `ValetParkingScenario`、`EmergencyStopScenario`、`PullOverScenario`、`TrafficLightProtectedScenario` 等 |
| `Stage` | `planning_interface_base/scenario_base/stage.h:46` | 每个 Scenario 下的所有 Stage |
| `Task` | `planning_interface_base/task_base/task.h:37` | 所有 `tasks/` 子目录下的任务（Decider / PathGeneration / SpeedOptimizer 子类均以 `Task` 注册） |
| `TrafficRule` | `planning_interface_base/traffic_rules_base/traffic_rule.h:37` | 所有 `traffic_rules/` 子目录下的规则 |

### 一次完整的反射链条

以"加载并运行 `ValetParkingScenario` 的 `StageParking`"为例：

1. `OnLanePlanning` 在 `Init` 时读取全局场景配置，得到场景类名 `"apollo::planning::ValetParkingScenario"`
2. `ScenarioManager` 调 `PluginManager::CreateInstance<Scenario>(class_name)` 创建场景
3. `Scenario::Init` 通过 RTTI 取 `class_name`，调 `PluginManager::GetPluginConfPath<Scenario>` 定位 `conf/pipeline.pb.txt`
4. `Scenario::Process` 在首次执行时读取 `StagePipeline[0].type`（如 `"apollo::planning::StageParking"`）
5. `Scenario::CreateStage` 调 `PluginManager::CreateInstance<Stage>` 创建 Stage
6. `Stage::Init` 读 `StagePipeline.task` 列表，对每个 `type` 调 `PluginManager::CreateInstance<Task>` 创建 Task
7. `Task::Init` 再次通过 RTTI 推导 `default_config_path_`，并用 `ConfigUtil::LoadMergedConfig` 把 Stage 级覆盖 + 插件默认配置合并载入
8. 每帧 `Stage::Process` 调 `ExecuteTaskOn*` 驱动 task chain

整条链路中，框架代码（`PlanningBase` / `Scenario` / `Stage`）对具体实现零依赖；所有耦合点都集中在配置文件里的类名字符串。这正是 `planning_interface_base` 的价值：**把抽象稳定在接口层，把变更局限在插件层。**
