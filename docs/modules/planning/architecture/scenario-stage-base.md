# Scenario & Stage 基类架构

> 源码位置：`modules/planning/planning_interface_base/scenario_base/`

## 模块定位

`Scenario` 和 `Stage` 是 Apollo Planning 的核心抽象层，定义了场景驱动的规划框架：
- **Scenario**：管理场景生命周期和阶段切换
- **Stage**：执行具体规划逻辑（Task 流水线）

所有具体场景（LaneFollow、StopSign、PullOver 等）均继承这两个基类。

---

## 一、Scenario 基类

### 类声明

```cpp
struct ScenarioContext { };  // 场景上下文基类

class Scenario {
 public:
  virtual bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name);
  virtual ScenarioContext* GetContext() = 0;
  virtual bool IsTransferable(const Scenario* other, const Frame& frame) { return false; }
  virtual ScenarioResult Process(const TrajectoryPoint& init_point, Frame* frame);
  virtual bool Exit(Frame* frame) { return true; }
  virtual bool Enter(Frame* frame) { return true; }
  std::shared_ptr<Stage> CreateStage(const StagePipeline& pipeline);
  const ScenarioStatusType& GetStatus() const;
  const std::string GetStage() const;
  void Reset();
 protected:
  template <typename T> bool LoadConfig(T* config);
  ScenarioResult scenario_result_;
  std::shared_ptr<Stage> current_stage_;
  std::unordered_map<std::string, const StagePipeline*> stage_pipeline_map_;
};
```

### Init()

```cpp
bool Scenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  // 1. 设置 PlanningContext 中的 scenario_type
  scenario->set_scenario_type(name_);
  // 2. 通过 PluginManager 获取配置路径
  config_dir_ = PluginManager::GetPluginClassHomePath<Scenario>(class_name) + "/conf";
  config_path_ = PluginManager::GetPluginConfPath<Scenario>(..., "conf/scenario_conf.pb.txt");
  // 3. 加载 pipeline.pb.txt（阶段流水线配置）
  GetProtoFromFile(pipeline_config_path, &scenario_pipeline_config_);
  // 4. 建立 stage name → config 映射
  for (const auto& stage : scenario_pipeline_config_.stage())
    stage_pipeline_map_[stage.name()] = &stage;
}
```

- `scenario.cc:L43-L84`

### Process() — 场景主循环

```cpp
ScenarioResult Scenario::Process(const TrajectoryPoint& init_point, Frame* frame) {
  // 首次调用：创建第一个 Stage
  if (current_stage_ == nullptr)
    current_stage_ = CreateStage(*stage_pipeline_map_[first_stage_name]);

  // Stage name 为空 → 场景完成
  if (current_stage_->Name().empty()) return STATUS_DONE;

  // 执行当前 Stage
  auto ret = current_stage_->Process(init_point, frame);

  switch (ret.GetStageStatus()) {
    case ERROR:      → STATUS_UNKNOWN
    case RUNNING:    → STATUS_PROCESSING
    case FINISHED:   → 切换到 NextStage 或 STATUS_DONE
  }
}
```

- 阶段切换逻辑：`Stage::NextStage()` 返回下一阶段名
- 空字符串表示场景结束
- `scenario.cc:L86-L153`

### CreateStage()

```cpp
std::shared_ptr<Stage> Scenario::CreateStage(const StagePipeline& pipeline) {
  auto stage_ptr = PluginManager::CreateInstance<Stage>(GetFullPlanningClassName(type));
  stage_ptr->Init(pipeline, injector_, config_dir_, GetContext());
  return stage_ptr;
}
```

- 通过插件系统动态创建 Stage 实例
- `scenario.cc:L155-L168`

---

## 二、Stage 基类

### 类声明

```cpp
class Stage {
 public:
  virtual bool Init(const StagePipeline& config, const shared_ptr<DependencyInjector>&,
                    const string& config_dir, void* context);
  virtual StageResult Process(const TrajectoryPoint& init_point, Frame* frame) = 0;
  const string& Name() const;
  template <typename T> T* GetContextAs() const;
  const string& NextStage() const { return next_stage_; }
 protected:
  StageResult ExecuteTaskOnReferenceLine(const TrajectoryPoint&, Frame*);
  StageResult ExecuteTaskOnOpenSpace(Frame*);
  virtual StageResult FinishScenario();
  vector<shared_ptr<Task>> task_list_;
  shared_ptr<Task> fallback_task_;
  string next_stage_;
  void* context_;
};
```

### Init()

```cpp
bool Stage::Init(const StagePipeline& config, ...) {
  // 1. 设置 PlanningContext 中的 stage_type
  planning_status->mutable_scenario()->set_stage_type(name_);
  // 2. 加载 Task 插件列表
  for (int i = 0; i < pipeline_config_.task_size(); ++i) {
    auto task_ptr = PluginManager::CreateInstance<Task>(task_type);
    task_ptr->Init(task_config_dir, task.name(), injector);
    task_list_.push_back(task_ptr);
  }
  // 3. 加载 fallback task（默认 FastStopTrajectoryFallback）
  fallback_task_ = PluginManager::CreateInstance<Task>(fallback_task_type);
}
```

- `stage.cc:L43-L97`

### ExecuteTaskOnReferenceLine() — 参考线规划

```cpp
StageResult Stage::ExecuteTaskOnReferenceLine(const TrajectoryPoint& start, Frame* frame) {
  for (auto& rli : *frame->mutable_reference_line_info()) {
    if (!rli.IsDrivable() || rli.IsChangeLanePath()) { skip; }
    // 依次执行所有 Task
    for (auto task : task_list_) {
      ret = task->Execute(frame, &rli);
      RecordDebugInfo(&rli, task->Name(), time_diff_ms);
      if (!ret.ok()) break;
    }
    // Task 失败时执行 fallback
    if (!ret.ok()) fallback_task_->Execute(frame, &rli);
    // 合并路径和速度剖面为轨迹
    rli.CombinePathAndSpeedProfile(..., &trajectory);
    rli.SetTrajectory(trajectory);
    return stage_result;
  }
}
```

- 只处理第一条可驾驶的非变道参考线
- Task 失败时自动执行 fallback（紧急停车轨迹）
- `stage.cc:L101-L159`

### ExecuteTaskOnOpenSpace() — 开放空间规划

```cpp
StageResult Stage::ExecuteTaskOnOpenSpace(Frame* frame) {
  for (auto task : task_list_) {
    ret = task->Execute(frame);  // 无 ReferenceLineInfo
    if (!ret.ok()) break;
  }
  // 从 open_space_info 获取轨迹
  auto& trajectory = frame->open_space_info().chosen_partitioned_trajectory().first;
  auto& gear = frame->open_space_info().chosen_partitioned_trajectory().second;
  PublishableTrajectory publishable_trajectory(now, trajectory);
  *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) = ...;
}
```

- 用于泊车、PullOver 重试等开放空间场景
- `stage.cc:L220-L256`

### FinishScenario()

```cpp
StageResult Stage::FinishScenario() {
  next_stage_ = "";  // 空字符串触发 Scenario 结束
  return StageResult(StageStatusType::FINISHED);
}
```

- `stage.cc:L258-L261`

---

## 生命周期总结

```
Planner::Plan()
  → Scenario::Process()
    → Stage::Process()          [纯虚函数，子类实现]
      → ExecuteTaskOnReferenceLine() 或 ExecuteTaskOnOpenSpace()
        → Task::Execute() × N
        → fallback_task_->Execute() [仅在失败时]
      → CombinePathAndSpeedProfile()
    → 检查 StageStatus → 切换 Stage 或结束 Scenario
```

## 调用关系

- **上游**：`PublicRoadPlanner::Plan()` 调用 `Scenario::Process()`
- **下游**：`Task::Execute()` 执行具体规划算法
- **插件系统**：`PluginManager` 动态加载 Scenario/Stage/Task
- **配置**：`pipeline.pb.txt` 定义阶段序列和 Task 列表
