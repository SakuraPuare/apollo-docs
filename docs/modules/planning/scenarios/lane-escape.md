---
title: "Lane Escape"
---

# Lane Escape Scenario

> 源码路径: `modules/planning/scenarios/lane_escape/`

## 概述

Lane Escape（车道逃离）场景用于泊车环境中当车辆当前车道被阻塞时，通过倒车退出当前车道，找到一条畅通的前进路径。该场景属于 Apollo 规划模块中的开放式空间规划场景，主要应用于泊车数据中心（`ParkDataCenter`）触发的车道逃离需求。

整体流程分为两个阶段：当车辆未挂倒挡时，生成一段静止轨迹等待换挡；当车辆进入倒挡后，在参考线上规划倒车路径，连续检测到畅通的前进路径后减速停车，完成逃离。

## 核心类

### LaneEscapeContext

场景上下文结构体，继承自 `ScenarioContext`，封装该场景的配置参数：

```cpp
struct LaneEscapeContext : public ScenarioContext {
  ScenarioLaneEscapeConfig scenario_config;
};
```

### LaneEscapeScenario

场景主类，继承自 `Scenario`，负责场景初始化与场景转移判断。通过 Cyber 插件系统注册：

```cpp
class LaneEscapeScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;
  ScenarioContext* GetContext() override { return &context_; }
  bool IsTransferable(const Scenario* other_scenario, const Frame& frame) override;
 private:
  LaneEscapeContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneEscapeScenario, Scenario)
```

### LaneEscapeParkStage

场景执行阶段，继承自 `Stage`，实现倒车轨迹生成与畅通路径检测的核心逻辑。通过 Cyber 插件系统注册：

```cpp
class LaneEscapeParkStage : public Stage {
 public:
  bool Init(const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context);
  StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;
 private:
  StageResult FinishStage();
  bool need_to_stop_ = false;
  int success_path_times_ = 0;
  int stop_check_count_ = 0;
  std::shared_ptr<common::PathPoint> stop_point_ = nullptr;
  ScenarioLaneEscapeConfig scenario_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneEscapeParkStage, Stage)
```

## 核心函数

### LaneEscapeScenario::Init

加载场景配置，初始化场景上下文。调用基类 `Init` 后，通过模板方法 `LoadConfig` 从配置文件中反序列化 `ScenarioLaneEscapeConfig`。

### LaneEscapeScenario::IsTransferable

判断是否应从当前场景转移到 Lane Escape 场景。转移条件按以下顺序检查：

1. 历史记录不为空（`injector_->history()->Size() != 0`）
2. 规划指令包含车道跟随命令（`has_lane_follow_command()`）
3. 参考线信息非空（`!frame.reference_line_info().empty()`）
4. 泊车数据中心标记需要逃离（`ParkDataCenter::Instance()->is_need_escape()`）

当历史记录为空时，会主动重置 `need_escape` 标志为 `false`，防止残留状态导致异常转移。

### LaneEscapeParkStage::Init

从场景上下文复制配置到阶段私有成员 `scenario_config_`。

### LaneEscapeParkStage::Process

阶段主处理函数，执行逻辑分为三个分支：

**分支一：未挂倒挡时**

车辆不在倒挡时，生成一段 5 秒内持续静止的轨迹（位置为当前车辆位置，速度和加速度均为 0），目标挡位设为 `GEAR_REVERSE`，引导车辆切换至倒挡。返回 `RUNNING` 状态继续等待。

**分支二：已挂倒挡，规划倒车路径**

1. 在参考线上计算倒车停止点，距当前车辆位置后方 `escape_reverse_distance` 米处
2. 构建停车决策（`BuildStopDecision`），停车原因为 `STOP_REASON_STOP_SIGN`
3. 调用 `ExecuteTaskOnReferenceLine` 执行参考线上的任务
4. 遍历候选路径，判断是否存在畅通的前进路径：路径标签包含 `regular`、路径长度超过 `forward_path_length`、且前方障碍物距离超过 `forward_path_length`
5. 连续检测到畅通路径次数累加（`success_path_times_`），否则重置为 0

**分支三：已确认畅通路径，减速停车**

当连续畅通路径次数超过 `success_path_count` 时，计算减速度停车距离（基于当前车速和 0.5 m/s^2 的减速度），设置停车点并触发减速。当车速低于 0.05 m/s 且持续 `stop_check_window` 个周期后，调用 `FinishStage` 结束阶段。

### LaneEscapeParkStage::FinishStage

清空下一阶段并重置泊车数据中心的逃离标志，返回 `FINISHED` 状态。

## 配置

配置通过 Protobuf 定义，消息类型为 `ScenarioLaneEscapeConfig`：

```protobuf
message ScenarioLaneEscapeConfig {
  optional double escape_reverse_distance = 1 [default = 5.0];
  optional double escape_stop_distance = 2 [default = 1.0];
  optional int32 success_path_count = 3 [default = 10];
  optional int32 stop_check_window = 6 [default = 10];
  optional double forward_path_length = 7 [default = 15.0];
}
```

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `escape_reverse_distance` | double | 5.0 | 倒车距离（米），在参考线上后退的目标距离 |
| `escape_stop_distance` | double | 1.0 | 最小停车距离（米），减速停车时的距离下限 |
| `success_path_count` | int32 | 10 | 畅通路径连续检测次数阈值，达到后触发停车 |
| `stop_check_window` | int32 | 10 | 停车确认窗口（周期数），车速低于 0.05 m/s 的持续周期数 |
| `forward_path_length` | double | 15.0 | 前方路径长度阈值（米），用于判断路径是否畅通 |

## 调用关系

```text
LaneEscapeScenario::Init
  -> Scenario::Init (基类初始化)
  -> Scenario::LoadConfig<ScenarioLaneEscapeConfig> (加载配置)

LaneEscapeScenario::IsTransferable
  -> ParkDataCenter::is_need_escape (检查逃离标志)

LaneEscapeParkStage::Init
  -> Stage::Init (基类初始化)
  -> GetContextAs<LaneEscapeContext> (获取场景配置)

LaneEscapeParkStage::Process
  -> [未挂倒挡] 生成静止倒车轨迹
  -> [已挂倒挡]
     -> reference_line.GetReferencePoint (计算倒车停止点)
     -> planning::util::BuildStopDecision (构建停车决策)
     -> ExecuteTaskOnReferenceLine (执行参考线任务)
     -> [检测畅通路径] 遍历 candidate_paths 判断路径条件
     -> [减速停车] 计算减速度距离，更新停止点
     -> [停车完成] FinishStage

LaneEscapeParkStage::FinishStage
  -> ParkDataCenter::set_need_escape(false) (重置逃离标志)
```
