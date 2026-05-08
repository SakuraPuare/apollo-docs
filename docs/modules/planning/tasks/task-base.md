---
title: "Task 基类体系"
---

# Task 基类体系

> 源码位置：`modules/planning/planning_interface_base/task_base/`

## 模块定位

Task 基类体系定义了 Planning 中所有可执行单元的接口层次。Stage 通过 Task 流水线完成路径规划、速度规划和决策。

### 继承层次

```
Task（顶层基类）
├── PathGeneration（路径生成基类）
│   ├── LaneFollowPath
│   ├── LaneBorrowPath
│   ├── FallbackPath
│   └── ReversePath ...
├── Decider（决策基类）
│   ├── PathDecider
│   ├── SpeedDecider
│   ├── RuleBasedStopDecider
│   └── RssDecider ...
├── SpeedOptimizer（速度优化基类）
│   ├── PiecewiseJerkSpeed
│   └── PiecewiseJerkSpeedNonlinear
└── 直接继承 Task
    ├── ObstacleNudgeDecider
    └── OpenSpace 系列任务
```

---

## 一、Task — 顶层基类

```cpp
class Task {
 public:
  const std::string& Name() const;
  virtual bool Init(const std::string& config_dir, const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector);
  virtual Status Execute(Frame* frame, ReferenceLineInfo* rli);
  virtual Status Execute(Frame* frame);
 protected:
  template <typename T> bool LoadConfig(T* config);
  Frame* frame_;
  ReferenceLineInfo* reference_line_info_;
  std::shared_ptr<DependencyInjector> injector_;
  std::string config_path_;
  std::string default_config_path_;
  std::string name_;
};
```

### Init()

- 设置 `name_`、`injector_`
- 通过 `PluginManager` 获取配置路径
- 子类调用 `LoadConfig\<T\>()` 加载 protobuf 配置

### Execute()

- 保存 `frame_` 和 `reference_line_info_` 指针
- 子类重写此方法或委托给 `Process()`

### LoadConfig\<T\>()

```cpp
template <typename T>
bool Task::LoadConfig(T* config) {
  return ConfigUtil::LoadMergedConfig(default_config_path_, config_path_, config);
}
```

- 合并默认配置和场景特定配置（场景目录下的同名配置覆盖默认值）

---

## 二、PathGeneration — 路径生成基类

```cpp
class PathGeneration : public Task {
 public:
  Status Execute(Frame*, ReferenceLineInfo*) override;
  Status Execute(Frame*) override;
 protected:
  virtual Status Process(Frame*, ReferenceLineInfo*) { return OK; }
  virtual Status Process(Frame*) { return OK; }
  void GetStartPointSLState();
  void RecordDebugInfo(const PathBound&, const string&, ReferenceLineInfo*);
  void RecordDebugInfo(const PathData&, const string&, ReferenceLineInfo*);
  bool GetSLBoundary(const PathData&, int, const ReferenceLineInfo*, SLBoundary*);
  SLState init_sl_state_;
};
```

### Execute()

```cpp
Status PathGeneration::Execute(Frame* frame, ReferenceLineInfo* rli) {
  Task::Execute(frame, rli);
  return Process(frame, rli);  // 委托给子类
}
```

- `path_generation.cc:L31-L35`

### GetStartPointSLState()

```cpp
void PathGeneration::GetStartPointSLState() {
  TrajectoryPoint start = frame_->PlanningStartPoint();
  if (FLAGS_use_front_axe_center_in_path_planning) {
    // 将规划起点从后轴中心转换到前轴中心
    start.x += wheel_base * cos(theta);
    start.y += wheel_base * sin(theta);
  }
  init_sl_state_ = reference_line.ToFrenetFrame(start);
}
```

- 计算规划起点的 SL 坐标（s, ds, l, dl）
- 可选前轴中心模式（`FLAGS_use_front_axe_center_in_path_planning`）
- `path_generation.cc:L107-L132`

### GetSLBoundary()

- 根据路径上某点的位置和车辆尺寸，计算该点处车辆的 SL 包围盒
- 用于路径评估时的碰撞检测
- `path_generation.cc:L134-L170`

### RecordDebugInfo()

- 将路径边界或候选路径写入 debug 数据，供 Dreamview 可视化
- `path_generation.cc:L42-L106`

---

## 三、Decider — 决策基类

```cpp
class Decider : public Task {
 public:
  Status Execute(Frame*, ReferenceLineInfo*) override;
  Status Execute(Frame*) override;
 protected:
  virtual Status Process(Frame*, ReferenceLineInfo*) { return OK; }
  virtual Status Process(Frame*) { return OK; }
};
```

- 结构与 PathGeneration 相同：Execute 委托给 Process
- 语义区别：Decider 不生成路径/速度，而是设置决策标签（nudge/stop/yield）
- 子类包括：PathDecider、SpeedDecider、RuleBasedStopDecider 等

---

## 四、SpeedOptimizer — 速度优化基类

```cpp
class SpeedOptimizer : public Task {
 public:
  Status Execute(Frame*, ReferenceLineInfo*) override;
 protected:
  virtual Status Process(const PathData& path_data,
                         const TrajectoryPoint& init_point,
                         SpeedData* speed_data) = 0;
  void RecordDebugInfo(const SpeedData&);
  void RecordDebugInfo(const SpeedData&, STGraphDebug*);
};
```

### Execute()

```cpp
Status SpeedOptimizer::Execute(Frame* frame, ReferenceLineInfo* rli) {
  Task::Execute(frame, rli);
  auto ret = Process(rli->path_data(), frame->PlanningStartPoint(),
                     rli->mutable_speed_data());
  RecordDebugInfo(rli->speed_data());
  return ret;
}
```

- 自动从 ReferenceLineInfo 提取 path_data 和 speed_data
- 子类只需实现 `Process(path_data, init_point, speed_data)`
- `speed_optimizer.cc:L32-L42`

---

## 调用关系

```
Stage::ExecuteTaskOnReferenceLine()
  → task->Execute(frame, &rli)
    → PathGeneration::Execute → Process()  [路径任务]
    → Decider::Execute → Process()         [决策任务]
    → SpeedOptimizer::Execute → Process()  [速度任务]
    → Task::Execute (直接)                  [其他任务]
```
