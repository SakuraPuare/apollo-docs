---
title: "车辆模型"
---

# Vehicle Model

> 源码路径: `modules/common/vehicle_model/`

## 概述

VehicleModel 提供基于运动学自行车模型的车辆状态预测功能。给定当前车辆状态和预测时间跨度，通过欧拉前向离散化方法推算未来时刻的车辆位置、航向、速度等状态量。该模块假设控制指令（曲率、加速度）在预测时域内保持不变。

目前仅实现了后轴中心运动学自行车模型（`REAR_CENTERED_KINEMATIC_BICYCLE_MODEL`），质心动力学自行车模型和 MLP 模型已定义但尚未支持。

## 核心类

### VehicleModel

无实例化的纯静态工具类，构造函数已删除：

```cpp
class VehicleModel {
 public:
  VehicleModel() = delete;

  static VehicleState Predict(const double predicted_time_horizon,
                              const VehicleState& cur_vehicle_state);

 private:
  static void RearCenteredKinematicBicycleModel(
      const VehicleModelConfig& vehicle_model_config,
      const double predicted_time_horizon,
      const VehicleState& cur_vehicle_state,
      VehicleState* predicted_vehicle_state);
};
```

- 命名空间: `apollo::common`
- 依赖: `VehicleState`（vehicle_state proto）、`VehicleModelConfig`（vehicle_model_config proto）、`VehicleConfigHelper`

## 核心函数

### Predict

```cpp
static VehicleState Predict(const double predicted_time_horizon,
                            const VehicleState& cur_vehicle_state);
```

公共入口函数，执行流程：

1. 通过 `FLAGS_vehicle_model_config_filename` 加载 `VehicleModelConfig` 配置文件
2. 校验模型类型，拒绝 `COM_CENTERED_DYNAMIC_BICYCLE_MODEL` 和 `MLP_MODEL`（尚未实现）
3. 根据模型类型分派到对应的私有预测方法
4. 返回预测后的 `VehicleState`

### RearCenteredKinematicBicycleModel

```cpp
static void RearCenteredKinematicBicycleModel(
    const VehicleModelConfig& vehicle_model_config,
    const double predicted_time_horizon,
    const VehicleState& cur_vehicle_state,
    VehicleState* predicted_vehicle_state);
```

后轴中心运动学自行车模型的核心实现。使用欧拉前向离散化逐步推进状态：

- **时间步长**: 从配置读取 `dt`（默认 0.06s），不超过总预测时域
- **状态更新公式**（每步）:

  ```cpp
  intermidiate_phi = cur_phi + 0.5 * dt * cur_v * kappa;
  next_phi = cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * kappa;
  next_x = cur_x + dt * (cur_v + 0.5 * dt * cur_a) * cos(intermidiate_phi);
  next_y = cur_y + dt * (cur_v + 0.5 * dt * cur_a) * sin(intermidiate_phi);
  next_v = cur_v + dt * cur_a;
  ```

- **假设**: 曲率 `kappa` 和加速度在预测时域内恒定，z 轴位置不变
- **输出**: 设置预测状态的 x、y、z、heading、kappa、linear_velocity、linear_acceleration

## 配置

配置通过 protobuf 文件定义，由 gflag `FLAGS_vehicle_model_config_filename` 指定路径。

### Proto 定义

```protobuf
message VehicleModelConfig {
  enum ModelType {
    REAR_CENTERED_KINEMATIC_BICYCLE_MODEL = 0;
    COM_CENTERED_DYNAMIC_BICYCLE_MODEL = 1;
    MLP_MODEL = 2;
  }
  optional ModelType model_type = 1;
  optional RearCenteredKinematicBicycleModelConfig rc_kinematic_bicycle_model = 2;
  optional ComCenteredDynamicBicycleModelConfig comc_dynamic_bicycle_model = 3;
  optional MlpModelConfig mlp_model = 4;
}

message RearCenteredKinematicBicycleModelConfig {
  optional double dt = 1;
}
```

### 默认配置

```text
model_type: REAR_CENTERED_KINEMATIC_BICYCLE_MODEL
rc_kinematic_bicycle_model: {
    dt: 0.06
}
```

## 调用关系

```text
VehicleModel::Predict()
  |
  +-- cyber::common::GetProtoFromFile()   // 加载配置
  |
  +-- RearCenteredKinematicBicycleModel()  // 运动学自行车模型预测
        |
        +-- 循环: dt 步进 Euler 离散化
              |
              +-- 更新 x, y, phi, v
```

典型调用方为控制模块和预测模块，在需要根据当前车辆状态推算未来轨迹时调用 `VehicleModel::Predict()`。
