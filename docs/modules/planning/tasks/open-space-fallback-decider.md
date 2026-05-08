---
title: "OpenSpaceFallbackDecider 开放空间回退决策器"
---

# OpenSpaceFallbackDecider 开放空间回退决策器

> 源码路径：`modules/planning/tasks/open_space_fallback_decider/`

## 概述

`OpenSpaceFallbackDecider` 是规划模块中 `Decider` 的插件实现，用于开放空间（泊车等低速场景）下的回退决策。当已选定的开放空间轨迹存在碰撞风险时，该任务会生成减速停车的回退轨迹；当自车当前位置与静态障碍物重叠时，立即生成原地停车轨迹并设置 `fallback_flag`。

核心处理流程：构建障碍物预测包围盒环境 -> 对当前轨迹进行碰撞检测 -> 若碰撞则计算停车减速度并生成回退轨迹 -> 检测自车当前是否已处于碰撞状态。

## 核心类

### OpenSpaceFallbackDecider

继承自 `Decider`，通过 CyberRT 插件机制注册为 `Task` 类型插件。

```cpp
class OpenSpaceFallbackDecider : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  Status Process(Frame* frame) override;
  void BuildPredictedEnvironment(const std::vector<const Obstacle*>& obstacles,
                                 std::vector<std::vector<common::math::Box2d>>&
                                     predicted_bounding_rectangles);
  bool IsCollisionFreeTrajectory(
      const TrajGearPair& trajectory_pb,
      const std::vector<std::vector<common::math::Box2d>>&
          predicted_bounding_rectangles,
      size_t* current_idx, size_t* first_collision_idx);
  bool QuardraticFormulaLowerSolution(const double a, const double b,
                                      const double c, double* sol);
  bool IsCollisionFreeEgoBox();
  void PathPointNormalizing(double rotate_angle,
                            const common::math::Vec2d& translate_origin,
                            double* x, double* y, double* phi);

 private:
  OpenSpaceFallBackDeciderConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::OpenSpaceFallbackDecider, Task)
```

**源码**：`modules/planning/tasks/open_space_fallback_decider/open_space_fallback_decider.h`

## 核心函数

### OpenSpaceFallbackDecider::Process()

主处理入口，执行完整的回退决策流程。

```cpp
Status OpenSpaceFallbackDecider::Process(Frame* frame) {
  // 若已处于回退状态则跳过
  if (frame_->open_space_info().fallback_flag()) {
    return Status::OK();
  }

  // 1. 构建预测障碍物包围盒环境
  BuildPredictedEnvironment(frame->obstacles(), predicted_bounding_rectangles);

  // 2. 检测当前轨迹是否无碰撞
  if (!IsCollisionFreeTrajectory(..., &fallback_start_index,
                                 &first_collision_index)) {
    // 计算停车距离和减速度，生成回退轨迹
    frame_->mutable_open_space_info()->set_stop_flag(true);
    // ... 构建 fallback_trajectory
  } else {
    frame_->mutable_open_space_info()->set_stop_flag(false);
  }

  // 3. 检测自车当前位置是否与障碍物重叠
  if (!IsCollisionFreeEgoBox()) {
    // 生成原地停车轨迹，设置 fallback_flag = true
  }
}
```

**职责**：碰撞检测与回退轨迹生成
**输入**：`Frame* frame`（规划帧，包含轨迹和障碍物信息）
**输出**：设置 `stop_flag`、`fallback_flag`、写入 `fallback_trajectory`
**关键步骤**：

1. 若已处于 fallback 状态则直接返回
2. 调用 `BuildPredictedEnvironment()` 构建障碍物预测包围盒
3. 对当前选定轨迹执行 `IsCollisionFreeTrajectory()` 碰撞检测
4. 若碰撞：根据碰撞点距离计算减速度（限制最大 +/-4.0 m/s^2），生成减速到零的回退轨迹，并在末尾追加 20 个零速停留点（间隔 0.5s）
5. 最后调用 `IsCollisionFreeEgoBox()` 检测自车当前位置是否碰撞，若碰撞则生成原地停车轨迹

### OpenSpaceFallbackDecider::BuildPredictedEnvironment()

将障碍物按预测时间步展开为包围盒序列。

```cpp
void OpenSpaceFallbackDecider::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles,
    std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles) {
  double relative_time = 0.0;
  while (relative_time < config_.open_space_prediction_time_period()) {
    std::vector<Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles) {
      if (!obstacle->IsVirtual()) {
        TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
        Box2d box = obstacle->GetBoundingBox(point);
        predicted_env.push_back(std::move(box));
      }
    }
    predicted_bounding_rectangles.emplace_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}
```

**职责**：在预测时间窗口内，为每个非虚拟障碍物按时间步生成 `Box2d` 包围盒
**输入**：障碍物列表
**输出**：`predicted_bounding_rectangles`（按时间步索引的包围盒二维数组）

### OpenSpaceFallbackDecider::IsCollisionFreeTrajectory()

对给定轨迹逐点与预测包围盒进行碰撞检测。

```cpp
bool OpenSpaceFallbackDecider::IsCollisionFreeTrajectory(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles,
    size_t* current_index, size_t* first_collision_index) {
  // 从当前时间点开始遍历轨迹
  for (size_t i = *current_index; i < point_size; ++i) {
    // 构建自车包围盒，考虑后轴到中心偏移
    Box2d ego_box({x, y}, ego_theta, ego_length, ego_width);
    ego_box.Shift(shift_vec);

    // 与所有预测时间步的障碍物包围盒进行重叠检测
    for (size_t j = 0; j < predicted_time_horizon; j++) {
      for (const auto& obstacle_box : predicted_bounding_rectangles[j]) {
        if (ego_box.HasOverlap(obstacle_box)) {
          // 在碰撞时间缓冲内则判定为碰撞
          if (std::abs(trajectory_point.relative_time() -
                       j * FLAGS_trajectory_time_resolution) <
              config_.open_space_fallback_collision_time_buffer()) {
            *first_collision_index = i;
            return false;
          }
        }
      }
    }
  }
  return true;
}
```

**职责**：检测轨迹是否与预测障碍物存在碰撞
**输入**：轨迹对（轨迹 + 档位）、预测包围盒序列
**输出**：`current_index`（当前轨迹点索引）、`first_collision_index`（首次碰撞索引）
**返回**：`true` 无碰撞，`false` 有碰撞

### OpenSpaceFallbackDecider::IsCollisionFreeEgoBox()

检测自车当前位置是否与静态障碍物重叠。

```cpp
bool OpenSpaceFallbackDecider::IsCollisionFreeEgoBox() {
  const auto& vehicle_state = frame_->vehicle_state();
  Box2d ego_box({x, y}, heading, ego_length, ego_width);
  Polygon2d ego_polygon = Polygon2d(ego_box);

  for (const Obstacle* obstacle : frame_->obstacles()) {
    // 跳过虚拟、非静态、尺寸过小的障碍物
    if (obstacle->IsVirtual() || !obstacle->IsStatic() ||
        obstacle->Perception().width() < 0.2 ||
        obstacle->Perception().length() < 0.2) {
      continue;
    }
    Polygon2d obstacle_polygon = obstacle->PerceptionPolygon();
    if (ego_polygon.HasOverlap(obstacle_polygon)) {
      return false;
    }
  }
  return true;
}
```

**职责**：检测自车当前位置是否与静态障碍物碰撞
**输入**：自车状态、障碍物列表
**返回**：`true` 无碰撞，`false` 已处于碰撞状态

### OpenSpaceFallbackDecider::QuardraticFormulaLowerSolution()

求解一元二次方程的较小非负根，用于根据运动学公式反算停车时间。

```cpp
bool OpenSpaceFallbackDecider::QuardraticFormulaLowerSolution(
    const double a, const double b, const double c, double* sol) {
  // ax^2 + bx + c = 0，返回较小解的绝对值
  double tmp = b * b - 4 * a * c;
  if (tmp < kEpsilon) return false;
  double sol1 = (-b + std::sqrt(tmp)) / (2.0 * a);
  double sol2 = (-b - std::sqrt(tmp)) / (2.0 * a);
  *sol = std::abs(std::min(sol1, sol2));
  return true;
}
```

**职责**：求解二次方程并返回较小解的绝对值
**输入**：系数 `a`、`b`、`c`
**输出**：解 `sol`
**用途**：在回退轨迹生成中，根据 `s = v*t + 0.5*a*t^2` 反算到达各轨迹点的时间

## 配置

通过 `OpenSpaceFallBackDeciderConfig` protobuf 消息定义，配置文件路径为 `conf/default_conf.pb.txt`。

```protobuf
message OpenSpaceFallBackDeciderConfig {
  optional double open_space_prediction_time_period = 1 [default = 5.0];
  optional double open_space_fallback_collision_distance = 2 [default = 5.0];
  optional double open_space_fallback_stop_distance = 3 [default = 2.0];
  optional double open_space_fallback_collision_time_buffer = 4 [default = 10.0];
}
```

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `open_space_prediction_time_period` | 5.0 s | 障碍物预测时间窗口 |
| `open_space_fallback_collision_distance` | 5.0 m | 回退碰撞检测距离 |
| `open_space_fallback_stop_distance` | 2.0 m | 回退停车轨迹与障碍物的安全间距 |
| `open_space_fallback_collision_time_buffer` | 10.0 s | 碰撞判定时间缓冲 |

代码中还使用以下硬编码参数：

| 参数 | 值 | 说明 |
|------|-----|------|
| `vehicle_max_acc` | 4.0 m/s^2 | 车辆最大加速度 |
| `vehicle_max_dec` | -4.0 m/s^2 | 车辆最大减速度 |
| 停车安全间距 | 1.0 m | 碰撞点前的安全余量 |
| 停留点数量 | 20 个 | 回退轨迹末尾的零速停留点数（间隔 0.5s） |
| 速度上限 | 1.0 m/s | 回退过程中速度绝对值上限 |

## 调用关系

- **父类**：`Decider`（定义决策任务接口）
- **依赖**：`DependencyInjector`（获取车辆状态）、`VehicleConfigHelper`（获取车辆参数）、`Box2d`/`Polygon2d`（几何碰撞检测）
- **调用者**：开放空间规划器通过插件机制在轨迹选定后调用此 Task
- **数据流**：
  - 读取 `frame->open_space_info().chosen_partitioned_trajectory()` 作为输入轨迹
  - 写入 `frame->open_space_info().mutable_fallback_trajectory()` 回退轨迹
  - 设置 `frame->open_space_info().set_stop_flag()` 停车标志
  - 设置 `frame->open_space_info().set_fallback_flag()` 回退标志
