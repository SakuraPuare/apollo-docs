---
title: "泊车开放空间回退决策器"
---

# 泊车开放空间回退决策器

> 源码路径: `modules/planning/tasks/open_space_fallback_decider_park/`

## 概述

`OpenSpaceFallbackDeciderPark` 是开放空间泊车场景下的回退（fallback）决策器，继承自 `Decider` 基类。当开放空间规划器生成的轨迹与障碍物发生碰撞时，该模块负责生成安全的减速停车轨迹，使车辆在碰撞点之前停下来。

该模块区分静态障碍物碰撞和动态障碍物碰撞两种场景，分别采用不同的策略：

- **静态碰撞**：计算减速度，在配置的安全距离内将车辆减速至零；当距离不足时使用最大减速度（-4.0 m/s²）紧急停车。
- **动态碰撞**：基于二次方程求解停车时间，沿原轨迹路径生成减速轨迹，并在停止后追加 20 个零速轨迹点以保持车辆静止。

## 核心类

### OpenSpaceFallbackDeciderPark

```cpp
class OpenSpaceFallbackDeciderPark : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(Frame* frame) override;

  bool IsCollisionFreeTrajectory(
      const TrajGearPair& trajectory_pb,
      const std::vector<std::vector<common::math::Box2d>>&
          predicted_bounding_rectangles,
      const std::vector<Obstacle*>& static_obstacles, int* current_idx,
      int* first_collision_idx, bool& is_collision_with_static_obstacle,
      bool& is_collision_with_dynamic_obstacle);

  bool QuardraticFormulaLowerSolution(const double a, const double b,
                                      const double c, double* sol);

  bool IsCollisionFreeEgoBox();

  void CalculateFallbackTrajectory(
      const TrajGearPair& ChosenPartitionedTrajectory,
      const double& deceleration, const double& relative_time_interval,
      TrajGearPair* traj_gear_pair);

  void CalculateFallbackTrajectoryWithTrajectoryPose(
      const TrajGearPair& ChosenPartitionedTrajectory,
      const apollo::common::TrajectoryPoint start_point,
      const apollo::common::TrajectoryPoint end_point,
      const double& stop_distance,
      const double& relative_time_interval,
      TrajGearPair* traj_gear_pair);

 private:
  OpenSpaceFallBackDeciderParkConfig config_;
  double open_space_fallback_collision_distance_ = 0;
};
```

通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册为 Task 插件，由 CyberRT 插件管理器动态加载。

## 核心函数

### Init

加载 `OpenSpaceFallBackDeciderParkConfig` 配置，初始化 `open_space_fallback_collision_distance_` 为配置的碰撞检测距离。

### Process

主处理流程：

1. **构建碰撞环境**：调用 `OpenSpaceFallbackUtil::BuildPredictedEnvironment` 构建动态障碍物的预测包围盒序列，调用 `OpenSpaceFallbackUtil::BuildStaticObstacleEnvironment` 筛选静态障碍物。
2. **空轨迹处理**：若无已选轨迹，基于当前车速计算减速度，在配置碰撞距离内生成停车轨迹。
3. **碰撞检测**：调用 `OpenSpaceFallbackUtil::IsCollisionFreeTrajectory` 判断轨迹是否与障碍物碰撞，并区分静态/动态碰撞。
4. **静态碰撞处理**：清空优化器输出的所有中间轨迹数据，根据碰撞距离与停车距离的大小关系选择减速度，调用 `CalculateFallbackTrajectory` 或 `CalculateFallbackTrajectoryWithTrajectoryPose` 生成回退轨迹。
5. **动态碰撞处理**：沿原轨迹搜索停车索引，利用 `QuardraticFormulaLowerSolution` 求解停车时间，修改轨迹点的速度和加速度，裁剪停止点之后的轨迹并追加静止点。

### CalculateFallbackTrajectory

根据给定减速度和时间间隔，从当前车辆位置出发，沿当前航向角生成匀减速停车轨迹。根据挡位（前进/倒车）调整方向符号。当当前车速已为零时，生成 20 个原地静止轨迹点。

### CalculateFallbackTrajectoryWithTrajectoryPose

基于起始点和碰撞点的位姿生成回退轨迹。沿两点连线方向以三角形速度曲线（加速到最大速度后减速到零）运动到停止位置，最大速度限制为 0.5 m/s。当停止距离过小时生成原地静止轨迹。

### QuardraticFormulaLowerSolution

求解一元二次方程 `ax² + bx + c = 0`，返回较小解的绝对值，用于动态碰撞场景中计算停车时间。

### IsCollisionFreeEgoBox

基于当前车辆状态构建自车包围盒，遍历所有非虚拟、非动态且尺寸大于 0.2m 的障碍物进行多边形重叠检测。

## 配置

配置通过 Protobuf 消息 `OpenSpaceFallBackDeciderParkConfig` 定义：

```protobuf
message OpenSpaceFallBackDeciderParkConfig {
  optional double open_space_prediction_time_period = 1 [default = 5.0];
  optional double open_space_fallback_collision_distance = 2 [default = 5.0];
  optional double open_space_fallback_stop_distance = 3 [default = 2.0];
  optional double open_space_fallback_collision_time_buffer = 4 [default = 10.0];
  optional bool is_use_trajectory_pose = 5 [default = false];
  optional double distance_to_obs = 6 [default = 1.0];
  optional double collision_check_range = 7 [default = 10.0];
}
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `open_space_prediction_time_period` | double | 5.0 | 障碍物预测时间窗口（秒） |
| `open_space_fallback_collision_distance` | double | 5.0 | 回退碰撞检测距离（米） |
| `open_space_fallback_stop_distance` | double | 2.0 | 回退停车轨迹与障碍物的安全间隔（米） |
| `open_space_fallback_collision_time_buffer` | double | 10.0 | 回退碰撞检测时间缓冲（秒） |
| `is_use_trajectory_pose` | bool | false | 是否使用基于轨迹位姿的回退模式 |
| `distance_to_obs` | double | 1.0 | 停车点距障碍物的最小距离（米） |
| `collision_check_range` | double | 10.0 | 碰撞检测的范围半径（米） |

默认配置值（`conf/default_conf.pb.txt`）：

```text
open_space_prediction_time_period: 5.0
open_space_fallback_collision_distance: 1.0
open_space_fallback_stop_distance: 2.0
open_space_fallback_collision_time_buffer: 5.0
```

## 调用关系

- **上游**：由规划框架通过 Task 插件机制调用，接收 `Frame` 对象中的开放空间信息（`open_space_info`），包括已选分区轨迹、优化器轨迹数据和障碍物列表。
- **下游**：
  - `OpenSpaceFallbackUtil::BuildPredictedEnvironment` -- 构建动态障碍物预测包围盒
  - `OpenSpaceFallbackUtil::BuildStaticObstacleEnvironment` -- 筛选静态障碍物
  - `OpenSpaceFallbackUtil::IsCollisionFreeTrajectory` -- 碰撞检测
  - `common::VehicleConfigHelper` -- 获取车辆参数
- **输出**：将生成的回退轨迹写入 `frame->open_space_info()` 的 `chosen_partitioned_trajectory` 或 `fallback_trajectory` 字段，供下游轨迹执行模块使用。
