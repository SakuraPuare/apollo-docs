---
title: "预测场景管理"
---

# 预测场景管理

> 源码路径: `modules/prediction/scenario/`

## 概述

scenario 模块负责判断自动驾驶车辆当前所处的驾驶场景类型（巡航 CRUISE / 路口 JUNCTION / 未知 UNKNOWN），并根据不同场景提取特征、对障碍物进行优先级排序和交互标记。模块采用分层流水线架构：

1. **特征提取** -- `FeatureExtractor` 从容器管理器中读取定位、地图和规划数据，生成 `EnvironmentFeatures`（自车道、邻车道、前方路口）。
2. **场景分析** -- `ScenarioAnalyzer` 根据环境特征判断场景类型，并构建对应的 `ScenarioFeatures` 子类。
3. **障碍物优先级** -- `ObstaclesPrioritizer` 为障碍物分配 IGNORE / NORMAL / CAUTION 优先级。
4. **交互标记** -- `InteractionFilter` 标记可能与自车交互的障碍物。
5. **路权分析** -- `RightOfWay` 根据车道转向类型设置路权值。

主入口 `ScenarioManager::Run()` 当前编排了步骤 1 和 2；步骤 3-5 作为独立组件由上层调用。

## 核心类

### ScenarioManager

场景管理器，持有当前场景状态，是模块对外的统一入口。

```cpp
// modules/prediction/scenario/scenario_manager.h
class ScenarioManager {
 public:
  void Run(ContainerManager* container_manager);
  const Scenario scenario() const;
 private:
  Scenario current_scenario_;
};
```

`Run()` 执行特征提取和场景分析，将结果存入 `current_scenario_`。

### FeatureExtractor

纯静态工具类，从 `ContainerManager` 中提取环境特征。

```cpp
// modules/prediction/scenario/feature_extractor/feature_extractor.h
class FeatureExtractor {
 public:
  FeatureExtractor() = delete;
  static EnvironmentFeatures ExtractEnvironmentFeatures(
      ContainerManager* container_manager);
 private:
  static void ExtractEgoLaneFeatures(...);
  static void ExtractNeighborLaneFeatures(...);
  static void ExtractFrontJunctionFeatures(...);
  static std::shared_ptr<const hdmap::LaneInfo> GetEgoLane(
      const common::Point3D& position, const double heading);
};
```

`ExtractEnvironmentFeatures` 按以下顺序提取：

- 从 `PoseContainer` 获取自车位姿（位置、朝向、速度）
- 调用 `GetEgoLane` 定位自车所在车道（基于 `PredictionMap::GetMostLikelyCurrentLane`）
- `ExtractEgoLaneFeatures` 获取自车道 ID 和纵向投影 s 值，同时检查左侧逆向车道
- `ExtractNeighborLaneFeatures` 获取左右相邻车道
- `ExtractFrontJunctionFeatures` 从规划轨迹容器获取前方路口信息（仅考虑与信号灯/停止标志有重叠的路口）

### ScenarioAnalyzer

纯静态工具类，根据环境特征判断场景类型并构建对应的场景特征对象。

```cpp
// modules/prediction/scenario/analyzer/scenario_analyzer.h
class ScenarioAnalyzer {
 public:
  ScenarioAnalyzer() = delete;
  static std::shared_ptr<ScenarioFeatures> Analyze(
      const EnvironmentFeatures& environment_features);
};
```

场景判定逻辑：

- 若前方路口距离小于 `FLAGS_junction_distance_threshold`，判定为 `JUNCTION`
- 否则若存在自车道信息，判定为 `CRUISE`
- 其他情况返回 `UNKNOWN`

### ScenarioFeatures / CruiseScenarioFeatures / JunctionScenarioFeatures

场景特征的类继承体系：

```cpp
// 基类
class ScenarioFeatures {
 public:
  const Scenario& scenario() const;
 protected:
  Scenario scenario_;  // protobuf 类型
};

// 巡航场景
class CruiseScenarioFeatures : public ScenarioFeatures {
 public:
  bool IsLaneOfInterest(const std::string& lane_id) const;
  void InsertLaneOfInterest(const std::string& lane_id);
  void BuildCruiseScenarioFeatures(const EnvironmentFeatures&);
 private:
  void SearchForwardAndInsert(const std::string& lane_id,
                              double start_lane_s, double range);
  std::unordered_set<std::string> lane_ids_of_interest_;
};

// 路口场景
class JunctionScenarioFeatures : public ScenarioFeatures {
 public:
  void BuildJunctionScenarioFeatures(const EnvironmentFeatures&);
};
```

`CruiseScenarioFeatures` 向前搜索自车道及左右邻车道各 50m 范围内的后续车道，连同逆向车道一并纳入 `lane_ids_of_interest_` 集合，供后续预测使用。`JunctionScenarioFeatures` 仅记录路口 ID。

### ObstaclesPrioritizer

障碍物优先级排序器，根据自车状态和场景为障碍物分配优先级。

```cpp
// modules/prediction/scenario/prioritization/obstacles_prioritizer.h
class ObstaclesPrioritizer {
 public:
  explicit ObstaclesPrioritizer(
      const std::shared_ptr<ContainerManager>& container_manager);
  void AssignIgnoreLevel();   // 标记扫描区域外的障碍物为 IGNORE
  void AssignCautionLevel();  // 标记近距离交互障碍物为 CAUTION
};
```

`AssignIgnoreLevel` 构建自车前方矩形扫描区域，区域外、不在车道上、非路口附近、非前方行人/骑行者的障碍物标记为 IGNORE。`AssignCautionLevel` 在巡航保持车道、变道、路口、参考线合并/交叉、行人等场景下标记 CAUTION 级别障碍物。

### InteractionFilter

交互标记过滤器，结构与 `ObstaclesPrioritizer` 类似。

```cpp
// modules/prediction/scenario/interaction_filter/interaction_filter.h
class InteractionFilter {
 public:
  explicit InteractionFilter(
      const std::shared_ptr<ContainerManager>& container_manager);
  void AssignInteractiveTag();
};
```

为车辆类型障碍物标记 `INTERACTION` 或 `NONINTERACTION` 标签，覆盖路口、巡航保持车道、变道、参考线合并/交叉等场景。

### RightOfWay

路权分析工具类。

```cpp
// modules/prediction/scenario/right_of_way/right_of_way.h
class RightOfWay {
 public:
  RightOfWay() = delete;
  static void Analyze(ContainerManager* container_manager);
};
```

遍历障碍物的所有车道序列，根据车道转向类型设置 `right_of_way` 值：左转车道设为 -20，右转车道设为 -10，直行保持默认。

## 核心函数

| 函数 | 所属类 | 功能 |
|------|--------|------|
| `ScenarioManager::Run` | ScenarioManager | 编排特征提取与场景分析流水线 |
| `FeatureExtractor::ExtractEnvironmentFeatures` | FeatureExtractor | 从容器提取自车道、邻车道、前方路口环境特征 |
| `ScenarioAnalyzer::Analyze` | ScenarioAnalyzer | 根据环境特征判定场景类型，构建对应 ScenarioFeatures |
| `CruiseScenarioFeatures::BuildCruiseScenarioFeatures` | CruiseScenarioFeatures | 向前搜索 50m 范围内感兴趣车道集合 |
| `JunctionScenarioFeatures::BuildJunctionScenarioFeatures` | JunctionScenarioFeatures | 记录前方路口 ID |
| `ObstaclesPrioritizer::AssignIgnoreLevel` | ObstaclesPrioritizer | 基于扫描区域标记障碍物 IGNORE 优先级 |
| `ObstaclesPrioritizer::AssignCautionLevel` | ObstaclesPrioritizer | 按场景标记障碍物 CAUTION 优先级 |
| `InteractionFilter::AssignInteractiveTag` | InteractionFilter | 标记与自车存在交互关系的障碍物 |
| `RightOfWay::Analyze` | RightOfWay | 根据车道转向类型设置路权值 |

## 配置

场景模块通过 `prediction_gflags.h` 中的 gflags 控制行为，主要配置项：

| 配置项 | 说明 |
|--------|------|
| `FLAGS_junction_distance_threshold` | 前方路口判定距离阈值 |
| `FLAGS_lane_distance_threshold` | 车道搜索距离阈值 |
| `FLAGS_lane_angle_difference_threshold` | 车道角度偏差阈值 |
| `FLAGS_enable_all_junction` | 是否考虑所有路口（否则仅考虑与信号灯/停止标志重叠的路口） |
| `FLAGS_scan_length` / `FLAGS_scan_width` | 障碍物扫描区域尺寸 |
| `FLAGS_caution_distance_threshold` | CAUTION 级别距离阈值 |
| `FLAGS_interaction_distance_threshold` | 交互标记距离阈值 |
| `FLAGS_caution_obs_max_nums` | CAUTION 障碍物最大数量 |
| `FLAGS_interactive_obs_max_nums` | 交互障碍物最大数量 |
| `FLAGS_enable_rank_caution_obstacles` | 是否对 CAUTION 障碍物排序截断 |
| `FLAGS_enable_rank_interactive_obstacles` | 是否对交互障碍物排序截断 |
| `FLAGS_enable_all_pedestrian_caution_in_front` | 是否对前方所有行人标记 CAUTION |

## 调用关系

```text
ScenarioManager::Run()
  |
  +-- FeatureExtractor::ExtractEnvironmentFeatures()
  |     +-- PoseContainer          (获取自车位姿)
  |     +-- PredictionMap          (定位自车道)
  |     +-- ADCTrajectoryContainer (获取前方路口)
  |
  +-- ScenarioAnalyzer::Analyze()
        |
        +-- [CRUISE]  CruiseScenarioFeatures::BuildCruiseScenarioFeatures()
        |               +-- SearchForwardAndInsert() (BFS 向前搜索车道)
        |
        +-- [JUNCTION] JunctionScenarioFeatures::BuildJunctionScenarioFeatures()
        |
        +-- [UNKNOWN]  返回基类 ScenarioFeatures

ObstaclesPrioritizer (由上层独立调用)
  +-- AssignIgnoreLevel()   (扫描区域过滤)
  +-- AssignCautionLevel()  (场景化 CAUTION 标记)

InteractionFilter (由上层独立调用)
  +-- AssignInteractiveTag() (交互标记)

RightOfWay (由上层独立调用)
  +-- Analyze()              (路权设置)
```
