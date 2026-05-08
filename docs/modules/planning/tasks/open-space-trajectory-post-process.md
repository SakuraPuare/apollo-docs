---
title: "开放空间轨迹后处理"
---

# 开放空间轨迹后处理

> 源码路径: `modules/planning/tasks/open_space_trajectory_post_process/`

## 概述

`OpenSpaceTrajectoryPostProcess` 是开放空间规划模块中的轨迹后处理任务，继承自 `TrajectoryOptimizer`。该模块负责对优化后的开放空间轨迹进行插值、分段、档位切换处理和轨迹跟踪点选择，最终生成可用于控制的离散轨迹。

主要功能包括：

- 轨迹插值加密，提高轨迹点密度
- 按档位（前进/倒车）将轨迹分段
- 档位切换时的静止等待轨迹生成
- 基于 IoU 和距离的最近轨迹点匹配
- 轨迹到达终点检测与换段切换
- 失败安全搜索与轨迹历史管理

## 核心类

### OpenSpaceTrajectoryPostProcess

```cpp
class OpenSpaceTrajectoryPostProcess : public TrajectoryOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  ~OpenSpaceTrajectoryPostProcess() = default;

 private:
  common::Status Process() override;
  // ... 其他私有方法
};
```

继承自 `TrajectoryOptimizer`，通过插件机制注册为 `Task`：

```cpp
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::OpenSpaceTrajectoryPostProcess, Task)
```

## 核心函数

### Process()

主处理函数，执行流程如下：

1. 若优化轨迹为空，直接返回
2. 从上一帧继承分区轨迹和插值轨迹结果
3. 若当前无分区轨迹（首次规划）：
   - 对优化轨迹进行插值
   - 按档位分区轨迹
   - 初始化状态，选择首个分区轨迹
4. 若已有分区轨迹（跟踪阶段）：
   - 更新自车信息
   - 检查是否到达当前轨迹终点，若到达则切换到下一段
   - 检查是否到达目的地，若到达则生成停车轨迹
   - 执行档位切换轨迹插入（如启用）
   - 通过 IoU 最近点匹配选择跟踪点
   - 结合时间匹配与位置匹配进行速度重规划

### InterpolateTrajectory()

对优化轨迹进行线性插值加密。根据配置的 `interpolated_pieces_num` 在每两个相邻轨迹点之间插入中间点，使用 `InterpolateUsingLinearApproximation` 进行线性插值。

### PartitionTrajectory()

按档位将连续轨迹分段。通过计算相邻轨迹点的方向向量与航向角的差值判断前进/倒车，当档位发生变化时创建新的分段。

### CheckReachTrajectoryEnd()

检查是否到达当前轨迹段的终点。使用横向偏移、纵向偏移、航向偏移和车辆包围盒 IoU 等多重条件判断，同时需要车辆已停止（`stop_check_count_` 达到阈值）。

### InsertGearShiftTrajectory()

在档位切换时生成静止等待轨迹。管理档位切换状态机，等待切换完成后才继续执行。

### GenerateGearShiftTrajectory()

生成档位切换期间的静止轨迹，轨迹点位于车辆当前位置，速度为 0，持续时间为 `gear_shift_max_t`。

### GenerateStopTrajectory()

生成停车轨迹，持续 2 秒，速度为 0，带微小减速度。

### UpdateVehicleInfo()

从车辆状态更新自车信息，包括位置、航向、速度、档位和包围盒。同时累计停车检查计数。

### AdjustRelativeTimeAndS()

以最近跟踪点为原点重新计算轨迹的相对时间和弧长。

### CheckArrivePoint()

综合判断车辆是否到达目标点，检查横向偏移、纵向偏移、航向偏差和车辆包围盒 IoU。

## 配置

配置通过 `OpenSpaceTrajectoryPostProcessConfig` protobuf 定义：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `gear_shift_max_t` | double | - | 档位切换轨迹最大时长 |
| `gear_shift_unit_t` | double | - | 档位切换轨迹时间步长 |
| `gear_shift_period_duration` | double | - | 档位切换等待周期 |
| `interpolated_pieces_num` | uint64 | - | 轨迹插值分段数 |
| `heading_search_range` | double | - | 航向搜索范围 |
| `heading_track_range` | double | - | 航向跟踪范围 |
| `distance_search_range` | double | 1.0e-6 | 距离搜索范围 |
| `heading_offset_to_midpoint` | double | - | 终点航向偏移阈值 |
| `lateral_offset_to_midpoint` | double | 0.1 | 终点横向偏移阈值 |
| `longitudinal_offset_to_midpoint` | double | 0.1 | 终点纵向偏移阈值 |
| `vehicle_box_iou_threshold_to_midpoint` | double | 0.95 | 车辆包围盒 IoU 阈值 |
| `use_gear_shift_trajectory` | bool | false | 是否启用档位切换轨迹 |
| `speed_replan_distance` | double | 2.0 | 速度重规划距离阈值 |
| `scale_destination` | double | 1.0 | 目标点阈值缩放系数 |
| `stop_check_window` | uint32 | 3 | 停车检查窗口帧数 |

## 调用关系

- **上游**：由 `frame_->open_space_info()` 获取优化器输出的轨迹数据
- **下游**：将处理后的轨迹写入 `frame_->mutable_open_space_info()`，供控制模块使用
- **依赖**：
  - `TrajectoryOptimizer` — 基类，提供任务框架
  - `VehicleConfigHelper` — 获取车辆参数
  - `FrameHistory` — 获取上一帧行驶信息用于轨迹跟踪
  - `PlanningContext` — 存储开放空间规划状态和轨迹历史
