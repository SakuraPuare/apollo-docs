---
title: "车辆状态"
---

# 车辆状态

> 源码路径：`modules/common/vehicle_state/`

## 概述

`vehicle_state` 模块提供车辆当前状态的统一查询接口，融合定位（Localization）和底盘（Chassis）数据，输出包含位置、姿态、速度、加速度、曲率等完整车辆状态信息。规划和控制模块通过 `VehicleStateProvider` 获取车辆当前状态，是规划控制链路的基础数据源。

## 核心类

### VehicleStateProvider

车辆状态提供者，融合定位和底盘数据维护完整的车辆状态。

```cpp
class VehicleStateProvider {
 public:
  // 从定位和底盘消息更新状态
  Status Update(const localization::LocalizationEstimate& localization,
                const canbus::Chassis& chassis);

  // 从 protobuf 文件更新状态（离线模式）
  void Update(const std::string& localization_file,
              const std::string& chassis_file);

  // 位置
  double x() const;
  double y() const;
  double z() const;

  // 姿态
  double heading() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
  double kappa() const;

  // 运动
  double linear_velocity() const;
  double angular_velocity() const;
  double linear_acceleration() const;

  // 底盘
  double gear() const;
  double steering_percentage() const;
  double timestamp() const;

  // 高级查询
  math::Vec2d EstimateFuturePosition(double t) const;
  math::Vec2d ComputeCOMPosition(double rear_to_com_distance) const;

  const VehicleState& vehicle_state() const;
  const localization::Pose& pose() const;
  const localization::Pose& original_pose() const;
};
```

**源码**：`modules/common/vehicle_state/vehicle_state_provider.h`、`modules/common/vehicle_state/vehicle_state_provider.cc`

## 核心函数

### VehicleStateProvider::Update()

融合定位和底盘数据更新车辆状态。

```cpp
Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate &localization,
    const canbus::Chassis &chassis) {
  // 1. 从定位提取位置、姿态、角速度、加速度
  ConstructExceptLinearVelocity(localization);

  // 2. 时间戳：优先 measurement_time，其次 header.timestamp_sec
  vehicle_state_.set_timestamp(localization.measurement_time());

  // 3. 档位：来自 chassis.gear_location()
  vehicle_state_.set_gear(chassis.gear_location());

  // 4. 线速度：来自 chassis.speed_mps()
  //    倒车时取反（除非 FLAGS_reverse_heading_vehicle_state 为 true）
  vehicle_state_.set_linear_velocity(chassis.speed_mps());

  // 5. 方向盘转角：来自 chassis.steering_percentage()
  vehicle_state_.set_steering_percentage(chassis.steering_percentage());

  // 6. 曲率：kappa = angular_velocity / linear_velocity
  //    速度 < 0.1 m/s 时 kappa 设为 0
  vehicle_state_.set_kappa(angular_velocity / linear_velocity);

  return Status::OK();
}
```

**数据来源**：

| 字段 | 来源 | 说明 |
|------|------|------|
| x/y/z | Localization | 位置坐标 |
| heading | Localization | 航向角（四元数转换） |
| roll/pitch/yaw | Localization | 欧拉角 |
| angular_velocity | Localization | 角速度（z 轴） |
| linear_acceleration | Localization | 线加速度（y 轴） |
| linear_velocity | Chassis | 车速（倒车取反） |
| gear | Chassis | 档位 |
| steering_percentage | Chassis | 方向盘转角百分比 |
| kappa | 计算 | angular_velocity / linear_velocity |

### EstimateFuturePosition()

基于恒定角速度模型预测未来 t 秒后的位置。

```cpp
math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  // 角速度接近 0 时：直线运动
  //   dx = 0, dy = v * t
  // 角速度非零时：圆弧运动
  //   dx = -v/w * (1 - cos(w*t))
  //   dy = sin(w*t) * v/w
  // 使用四元数旋转到世界坐标
}
```

### ComputeCOMPosition()

计算质心（Center of Mass）位置，根据后轴到质心距离偏移。

```cpp
math::Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // 仅在前进或后退（由 FLAGS 控制）时偏移
  // 使用四元数旋转偏移向量后加到车辆位置
}
```

## 配置

| gflags | 说明 |
|--------|------|
| `FLAGS_reverse_heading_vehicle_state` | 倒车时是否保持正速度（不取反） |
| `FLAGS_state_transform_to_com_reverse` | 倒车时是否变换到质心坐标 |
| `FLAGS_state_transform_to_com_drive` | 前进时是否变换到质心坐标 |
| `FLAGS_enable_map_reference_unify` | 是否启用地图参考坐标统一 |
| `FLAGS_use_navigation_mode` | 导航模式下跳过定位更新 |

## 调用关系

- **上游**：定位模块发布 `LocalizationEstimate`；底盘模块发布 `Chassis`
- **被依赖**：规划模块（`PlanningComponent`）和控制模块（`ControllerAgent`）调用 `Update()` 后通过 getter 查询车辆状态
- **依赖**：`VehicleState` protobuf、`math::QuaternionToHeading()`、`math::EulerAnglesZXYd`
