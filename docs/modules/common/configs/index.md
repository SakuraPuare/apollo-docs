---
title: "Configs 配置工具"
---

# Configs 配置工具

> 源码路径：`modules/common/configs/`

## 概述

`configs` 模块提供 Apollo 系统的全局配置管理，核心是 `VehicleConfigHelper` 单例类，负责加载和提供车辆参数配置。此外通过 gflags 定义了地图路径、车辆配置路径、定位参数等全局配置项。规划、控制、感知等模块均依赖此模块获取车辆几何参数。

## 核心类

### VehicleConfigHelper

车辆配置帮助类（单例），提供车辆参数的加载和查询。

```cpp
class VehicleConfigHelper {
 public:
  // 从 FLAGS_vehicle_config_path 加载
  static void Init();

  // 从指定配置对象加载
  static void Init(const VehicleConfig &config);

  // 从指定文件加载
  static void Init(const std::string &config_file);

  // 获取当前车辆配置
  static const VehicleConfig &GetConfig();

  // 安全转弯半径（考虑车辆宽度和前后悬）
  static double MinSafeTurnRadius();

  // 根据路径点获取车辆包围盒
  static common::math::Box2d GetBoundingBox(const common::PathPoint &path_point);

 private:
  static VehicleConfig vehicle_config_;
  static bool is_init_;
  DECLARE_SINGLETON(VehicleConfigHelper)
};
```

**源码**：`modules/common/configs/vehicle_config_helper.h`、`modules/common/configs/vehicle_config_helper.cc`

## 核心函数

### VehicleConfigHelper::Init()

三种重载：无参版本从 `FLAGS_vehicle_config_path` 加载；文件路径版本从指定文件加载；直接传入 `VehicleConfig` 对象。

```cpp
void VehicleConfigHelper::Init() {
  Init(FLAGS_vehicle_config_path);
}

void VehicleConfigHelper::Init(const std::string &config_file) {
  VehicleConfig params;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &params));
  Init(params);
}

void VehicleConfigHelper::Init(const VehicleConfig &vehicle_params) {
  vehicle_config_ = vehicle_params;
  is_init_ = true;
}
```

### VehicleConfigHelper::GetConfig()

懒加载：首次调用时若未初始化则自动从默认路径加载。

```cpp
const VehicleConfig &VehicleConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}
```

### VehicleConfigHelper::MinSafeTurnRadius()

计算车辆以最大转向角转弯时的安全转弯半径，考虑车辆宽度和前后悬长度。

```cpp
double VehicleConfigHelper::MinSafeTurnRadius() {
  const auto &param = vehicle_config_.vehicle_param();
  double lat_edge_to_center =
      std::max(param.left_edge_to_center(), param.right_edge_to_center());
  double lon_edge_to_center =
      std::max(param.front_edge_to_center(), param.back_edge_to_center());
  return std::sqrt(
      (lat_edge_to_center + param.min_turn_radius()) *
          (lat_edge_to_center + param.min_turn_radius()) +
      lon_edge_to_center * lon_edge_to_center);
}
```

**公式**：`AO = sqrt((min_turn_radius + lat_edge)^2 + lon_edge^2)`

### VehicleConfigHelper::GetBoundingBox()

根据路径点（位置 + 航向角）计算车辆的 2D 包围盒。

```cpp
common::math::Box2d VehicleConfigHelper::GetBoundingBox(
    const common::PathPoint &path_point) {
  const auto &vehicle_param = vehicle_config_.vehicle_param();
  math::Vec2d point(path_point.x(), path_point.y());
  return common::math::Box2d(
      point, path_point.theta(),
      vehicle_param.front_edge_to_center(),
      vehicle_param.back_edge_to_center(),
      vehicle_param.width());
}
```

## 全局配置项

### 地图相关

| gflags | 说明 |
|--------|------|
| `map_dir` | 地图目录（包含 base_map、sim_map、routing 等） |
| `base_map_filename` | 基础地图文件名 |
| `sim_map_filename` | 仿真地图文件名 |
| `routing_map_filename` | 路由拓扑图文件名 |
| `default_routing_filename` | 默认路由文件名 |

### 车辆相关

| gflags | 说明 |
|--------|------|
| `vehicle_config_path` | 车辆配置文件路径 |
| `vehicle_model_config_filename` | 车辆模型配置文件名 |
| `half_vehicle_width` | 车辆半宽 |

### 定位相关

| gflags | 说明 |
|--------|------|
| `localization_tf2_frame_id` | TF2 父坐标系 ID |
| `localization_tf2_child_frame_id` | TF2 子坐标系 ID |
| `enable_map_reference_unify` | 是否启用地图参考坐标统一 |

### 其他

| gflags | 说明 |
|--------|------|
| `use_cyber_time` | 是否使用 Cyber 时间 |
| `use_sim_time` | 是否使用仿真时间 |
| `use_navigation_mode` | 是否使用导航模式 |
| `look_forward_time_sec` | 前向预瞄时间（秒） |
| `multithread_run` | 是否多线程运行 |

**源码**：`modules/common/configs/config_gflags.h`

## 调用关系

- **被依赖**：规划（`VehicleConfigHelper::GetConfig()` 获取车辆参数）、控制、感知、定位等所有模块
- **数据源**：`VehicleConfig` protobuf 定义在 `modules/common_msgs/config_msgs/vehicle_config.proto`
