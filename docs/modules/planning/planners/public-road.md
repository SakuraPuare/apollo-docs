---
title: "PublicRoadPlanner 与 ScenarioManager"
---

# PublicRoadPlanner 与 ScenarioManager

> 源码位置：`modules/planning/planners/public_road/`

## 模块定位

PublicRoadPlanner 是 Apollo 默认的公共道路规划器，采用场景驱动架构。它本身不包含规划算法，而是通过 ScenarioManager 选择合适的 Scenario，将规划工作委托给场景和阶段。

---

## 一、Planner 基类

```cpp
class Planner {
 public:
  virtual std::string Name() = 0;
  virtual Status Init(const shared_ptr<DependencyInjector>&, const string& config_path = "");
  virtual Status Plan(const TrajectoryPoint& init_point, Frame*, ADCTrajectory*) = 0;
  virtual void Stop() = 0;
  virtual void Reset(Frame*) {}
 protected:
  shared_ptr<DependencyInjector> injector_;
};

class PlannerWithReferenceLine : public Planner {
 public:
  virtual Status PlanOnReferenceLine(const TrajectoryPoint&, Frame*, ReferenceLineInfo*);
};
```

- `Planner`：纯虚基类，定义 Plan 接口
- `PlannerWithReferenceLine`：增加参考线级别的规划接口
- `planner.h:L45-L118`

---

## 二、PublicRoadPlanner

```cpp
class PublicRoadPlanner : public PlannerWithReferenceLine {
 public:
  string Name() override { return "PUBLIC_ROAD"; }
  Status Init(const shared_ptr<DependencyInjector>&, const string&) override;
  Status Plan(const TrajectoryPoint&, Frame*, ADCTrajectory*) override;
  void Reset(Frame* frame) override { scenario_manager_.Reset(frame); }
 private:
  ScenarioManager scenario_manager_;
  PlannerPublicRoadConfig config_;
  Scenario* scenario_ = nullptr;
};
```

### Init()

```cpp
Status PublicRoadPlanner::Init(const shared_ptr<DependencyInjector>& injector,
                               const string& config_path) {
  Planner::Init(injector, config_path);
  LoadConfig<PlannerPublicRoadConfig>(config_path, &config_);
  scenario_manager_.Init(injector, config_);
  return Status::OK();
}
```

- `public_road_planner.cc:L28-L35`

### Plan() — 核心规划入口

```cpp
Status PublicRoadPlanner::Plan(const TrajectoryPoint& start, Frame* frame,
                               ADCTrajectory* trajectory) {
  // 1. 更新场景（可能切换）
  scenario_manager_.Update(start, frame);
  scenario_ = scenario_manager_.mutable_scenario();

  // 2. 执行当前场景
  auto result = scenario_->Process(start, frame);

  // 3. 记录调试信息
  scenario_debug->set_scenario_plugin_type(scenario_->Name());
  scenario_debug->set_stage_plugin_type(scenario_->GetStage());

  // 4. 场景完成后再次 Update（触发切换到下一场景）
  if (result == STATUS_DONE)
    scenario_manager_.Update(start, frame);
  else if (result == STATUS_UNKNOWN)
    return PLANNING_ERROR;

  return OK;
}
```

- 每帧先 Update 再 Process
- 场景完成后立即再次 Update，确保无缝切换
- `public_road_planner.cc:L37-L66`

---

## 三、ScenarioManager — 场景管理器

```cpp
class ScenarioManager final {
 public:
  bool Init(const shared_ptr<DependencyInjector>&, const PlannerPublicRoadConfig&);
  Scenario* mutable_scenario() { return current_scenario_.get(); }
  void Update(const TrajectoryPoint& ego_point, Frame* frame);
  void Reset(Frame* frame);
 private:
  shared_ptr<Scenario> current_scenario_;
  shared_ptr<Scenario> default_scenario_type_;  // LANE_FOLLOW
  vector<shared_ptr<Scenario>> scenario_list_;
};
```

### Init()

```cpp
bool ScenarioManager::Init(const shared_ptr<DependencyInjector>& injector,
                           const PlannerPublicRoadConfig& config) {
  for (int i = 0; i < config.scenario_size(); i++) {
    auto scenario = PluginManager::CreateInstance<Scenario>(type);
    scenario->Init(injector_, name);
    scenario_list_.push_back(scenario);
    if (name == "LANE_FOLLOW") default_scenario_type_ = scenario;
  }
  current_scenario_ = default_scenario_type_;
}
```

- 从配置加载所有场景插件
- 默认场景为 LANE_FOLLOW
- `scenario_manager.cc:L33-L54`

### Update() — 场景切换逻辑

```cpp
void ScenarioManager::Update(const TrajectoryPoint& ego_point, Frame* frame) {
  for (auto scenario : scenario_list_) {
    // 当前场景正在处理中 → 保持不变（优先级保护）
    if (current == scenario && current->GetStatus() == STATUS_PROCESSING)
      return;
    // 检查是否可以切换到此场景
    if (scenario->IsTransferable(current_scenario_.get(), *frame)) {
      current_scenario_->Exit(frame);
      current_scenario_ = scenario;
      current_scenario_->Reset();
      current_scenario_->Enter(frame);
      return;
    }
  }
}
```

- **优先级规则**：配置中靠前的场景优先级更高
- **切换条件**：`IsTransferable()` 由各场景自行实现
- **正在处理的场景不会被抢占**
- `scenario_manager.cc:L56-L76`

### Reset()

```cpp
void ScenarioManager::Reset(Frame* frame) {
  current_scenario_->Exit(frame);
  default_scenario_type_->Reset();
  current_scenario_ = default_scenario_type_;
}
```

- 强制回到默认场景（LANE_FOLLOW）
- `scenario_manager.cc:L78-L85`

---

## 调用链路

```
PlanningComponent::Proc()
  → PlanningBase::Plan()
    → PublicRoadPlanner::Plan()
      → ScenarioManager::Update()     [场景选择]
      → Scenario::Process()            [场景执行]
        → Stage::Process()             [阶段执行]
          → Task::Execute() × N        [任务流水线]
```

## 场景优先级（典型配置顺序）

1. EmergencyPullOver / EmergencyStop
2. ParkAndGo
3. PullOver
4. StopSign / TrafficLight / YieldSign
5. LaneFollow（默认兜底）
