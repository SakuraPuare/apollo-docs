---
title: "开放空间轨迹分段"
---

# OpenSpaceTrajectoryPartition

> 源码路径: `modules/planning/tasks/open_space_trajectory_partition/`

## 概述

OpenSpaceTrajectoryPartition 是开放空间规划（Open Space Planning）中的轨迹分段任务。它继承自 `TrajectoryOptimizer`，负责将拼接后的连续轨迹按照挡位（前进/后退）拆分为多段子轨迹，并在执行过程中根据车辆当前位置选择最优轨迹段、管理挡位切换，最终输出当前帧需要跟踪的轨迹。

核心职责：

- **轨迹插值**：对拼接轨迹进行线性插值以提高采样密度
- **轨迹分段**：根据行驶方向将轨迹按前进/后退挡位拆分为多段
- **轨迹匹配**：基于 IoU（交并比）和航向角匹配当前位置与轨迹点
- **挡位切换管理**：在切换到下一段轨迹前插入静止等待轨迹，确保换挡完成
- **故障安全**：当常规匹配失败时，使用 failsafe 搜索跨轨迹匹配最近点

继承关系：

```text
Task → TrajectoryOptimizer → OpenSpaceTrajectoryPartition
```

## 核心类

### OpenSpaceTrajectoryPartition

主类，注册为 Cyber 插件（`CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN`），以 `Task` 类型动态加载。关键成员包括：车辆状态缓存（`ego_x_`/`ego_y_`/`ego_theta_`/`ego_box_`）、匹配参数（`heading_search_range_`/`distance_search_range_`）、轨迹状态（`current_trajectory_index_`/`last_index_`/`last_time_`）以及故障安全标志 `fail_search_fallback_`。内部定义了两个比较器 `pair_comp_` 和 `comp_` 用于优先级队列按 IoU 排序。

```cpp
class OpenSpaceTrajectoryPartition : public TrajectoryOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  void Restart();
 private:
  common::Status Process() override;
  OpenSpaceTrajectoryPartitionConfig config_;
  double ego_theta_, ego_x_, ego_y_, ego_v_;
  canbus::Chassis_GearPosition ego_gear_;
  Box2d ego_box_;
  double heading_search_range_, distance_search_range_;
  size_t current_trajectory_index_;
  int last_index_;
  bool fail_search_fallback_;
};
```

## 核心函数

### Process() — 主处理流程

```cpp
Status OpenSpaceTrajectoryPartition::Process() {
  // 1. 对拼接轨迹插值
  InterpolateTrajectory(stitched, &interpolated);
  // 2. 按挡位分段
  PartitionTrajectory(interpolated, &partitioned);
  // 3. 首次定位：直接使用第一段轨迹
  if (!position_init) { AdjustRelativeTimeAndS(partitioned, 0, 0, ...); return OK; }
  // 4. 检查是否到达当前轨迹段末端
  flag_change_to_next = CheckReachTrajectoryEnd(...);
  // 5. 如果需要换挡，插入换挡等待轨迹
  if (InsertGearShiftTrajectory(...)) return OK;
  // 6. 基于 IoU 在当前轨迹段中找最近点
  for (j in trajectory) {
    if (distance < range && heading_diff < range) closest_point.emplace(j, iou_ratio);
  }
  // 7. 如果匹配失败，生成停车轨迹进入故障安全模式
  if (closest_point.empty()) GenerateStopTrajectory(...);
  // 8. 速度匹配后调整相对时间和距离
  AdjustRelativeTimeAndS(...);
}
```

### InterpolateTrajectory() — 轨迹插值

在相邻点之间均匀插入 `interpolated_pieces_num - 1` 个中间点（线性近似）。若使用迭代锚点平滑器则跳过。

### PartitionTrajectory() — 按挡位分段

遍历轨迹点，通过相邻点行驶方向与路径朝向夹角判断挡位（< 90 度为前进，>= 90 度为后退），挡位变化时结束当前段并开启新段。每个轨迹点通过 `LoadTrajectoryPoint()` 加载，曲率按 `tan(steer) / wheel_base` 计算。

### CheckReachTrajectoryEnd() — 到达终点检测

同时满足横向偏移、纵向偏移、航向偏差、车速和 IoU 五个阈值条件时，将 `current_trajectory_index_` 推进到下一段。

### InsertGearShiftTrajectory() — 换挡等待轨迹

需要切换挡位时，生成静止等待轨迹（零速点，间隔 `gear_shift_unit_t`），等待 `gear_shift_period_duration` 秒后确认换挡完成。通过 `GenerateGearShiftTrajectory()` 生成。

### UseFailSafeSearch() — 故障安全搜索

遍历所有分段轨迹找到 IoU 最大的轨迹点，优先选择未遍历过的段。通过 `EncodeTrajectory()` 编码轨迹标识、`CheckTrajTraversed()` 检查历史、`UpdateTrajHistory()` 更新记录。

### AdjustRelativeTimeAndS() — 调整相对时间和距离

以当前匹配的最近点为原点，重新计算选定轨迹段和拼接轨迹的相对时间和弧长 s。

## 配置

配置通过 `OpenSpaceTrajectoryPartitionConfig` protobuf 消息定义（`default_conf.pb.txt`）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gear_shift_max_t` | 3.0 s | 换挡等待轨迹最大时长 |
| `gear_shift_unit_t` | 0.02 s | 换挡等待轨迹时间步长 |
| `gear_shift_period_duration` | 2.0 s | 换挡完成所需等待时长 |
| `interpolated_pieces_num` | 10 | 相邻点之间的插值份数 |
| `heading_search_range` | 0.79 rad | 最近点搜索航向角过滤范围 |
| `distance_search_range` | 2.0 m | 最近点搜索距离阈值 |
| `heading_offset_to_midpoint` | 0.79 rad | 到达终点检测航向偏移阈值 |
| `lateral_offset_to_midpoint` | 0.5 m | 到达终点检测横向偏移阈值 |
| `longitudinal_offset_to_midpoint` | 0.4 m | 到达终点检测纵向偏移阈值 |
| `vehicle_box_iou_threshold_to_midpoint` | 0.30 | 到达终点检测 IoU 阈值 |
| `speed_replan_distance` | 2.0 m | 速度重规划距离阈值 |
| `linear_velocity_threshold_on_ego` | 0.1 m/s | 到达终点检测速度阈值 |
| `use_gear_shift_trajectory` | true | 是否启用换挡等待轨迹 |

## 调用关系

```text
OpenSpaceTrajectoryPartition::Process()
  ├── InterpolateTrajectory()          // 插值提高密度
  ├── PartitionTrajectory()            // 按挡位拆分轨迹
  │     └── LoadTrajectoryPoint()      // 逐点加载
  ├── UpdateVehicleInfo()              // 刷新自车状态
  ├── CheckReachTrajectoryEnd()        // 判断是否到达段末端
  ├── InsertGearShiftTrajectory()      // 换挡等待
  │     └── GenerateGearShiftTrajectory()
  ├── [IoU 最近点搜索]                  // 优先级队列匹配
  ├── GenerateStopTrajectory()         // 故障安全停车
  ├── UseFailSafeSearch()              // 跨轨迹故障安全搜索
  │     ├── CheckTrajTraversed()
  │     └── UpdateTrajHistory()
  └── AdjustRelativeTimeAndS()         // 重置时间/距离原点
```

上下游数据流：

```text
OpenSpaceTrajectoryProvider
  → stitched_trajectory_result（拼接轨迹）
    → OpenSpaceTrajectoryPartition::Process()
      → partitioned_trajectories（分段轨迹）
      → chosen_partitioned_trajectory（当前选定轨迹段）
        → 下游控制模块
```
