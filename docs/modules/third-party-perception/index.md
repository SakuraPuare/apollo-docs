---
title: "第三方感知"
---

# 第三方感知 (Third Party Perception)

> 源码路径: `modules/third_party_perception/`

## 概述

`third_party_perception` 模块为 Apollo 自动驾驶平台提供第三方传感器（Mobileye、Smartereye、Conti/Delphi 雷达）的感知数据接入能力。该模块将第三方设备输出的障碍物和车道线数据转换为 Apollo 统一的 `PerceptionObstacles` 消息格式，供下游规划、预测等模块消费。

模块以 `TimerComponent` 方式运行（默认 100ms 周期，即 10Hz），根据配置文件中的设备类型（`SMARTEREYE` 或 `MOBILEYE`）选择对应的感知处理策略。Mobileye 方案支持摄像头与雷达（Conti 或 Delphi ESR）的数据融合；Smartereye 方案仅处理摄像头障碍物与车道线数据。

## 核心类

### ThirdPartyPerception

- 文件：`third_party_perception_base.h` / `third_party_perception_base.cc`
- 基类，提供通用的生命周期管理和数据回调接口

```cpp
class ThirdPartyPerception {
 public:
  explicit ThirdPartyPerception(apollo::cyber::Node* const node);
  std::string Name() const;
  apollo::common::Status Init();
  apollo::common::Status Start();
  void Stop();
  void OnLocalization(const apollo::localization::LocalizationEstimate& message);
  void OnChassis(const apollo::canbus::Chassis& message);
  virtual bool Process(apollo::perception::PerceptionObstacles* const response);

 protected:
  std::mutex third_party_perception_mutex_;
  apollo::localization::LocalizationEstimate localization_;
  apollo::canbus::Chassis chassis_;
  RadarObstacles current_radar_obstacles_;
  RadarObstacles last_radar_obstacles_;
  std::shared_ptr<apollo::cyber::Node> node_;
};
```

构造函数中自动创建 `LocalizationEstimate` 和 `Chassis` 两个 Reader，分别订阅定位和底盘通道。所有回调通过 `third_party_perception_mutex_` 保证线程安全。

### ThirdPartyPerceptionComponent

- 文件：`third_party_perception_component.h` / `third_party_perception_component.cc`
- 继承自 `apollo::cyber::TimerComponent`，是模块的 Cyber RT 入口组件

```cpp
class ThirdPartyPerceptionComponent final
    : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  std::shared_ptr<apollo::cyber::Writer<apollo::perception::PerceptionObstacles>> writer_;
  std::shared_ptr<ThirdPartyPerception> perception_;
};
```

- `Init()`：根据 `ThirdPartyPerceptionDevice` 配置中的 `device_type` 创建对应的感知实例（`ThirdPartyPerceptionSmartereye` 或 `ThirdPartyPerceptionMobileye`），并创建 `PerceptionObstacles` Writer
- `Proc()`：每周期调用 `perception_->Process()` 填充障碍物数据并发布

### ThirdPartyPerceptionMobileye

- 文件：`third_party_perception_mobileye.h` / `third_party_perception_mobileye.cc`
- 继承自 `ThirdPartyPerception`，实现 Mobileye 摄像头 + 雷达融合方案

```cpp
class ThirdPartyPerceptionMobileye : public ThirdPartyPerception {
 public:
  void OnMobileye(const apollo::drivers::Mobileye& message);
  void OnContiRadar(const apollo::drivers::ContiRadar& message);
  void OnDelphiESR(const apollo::drivers::DelphiESR& message);
  bool Process(apollo::perception::PerceptionObstacles* const response) override;
};
```

构造函数创建三个 Reader，分别订阅 Mobileye、Delphi ESR 和 Conti Radar 通道。`Process()` 将摄像头障碍物与雷达障碍物进行融合后输出。

### ThirdPartyPerceptionSmartereye

- 文件：`third_party_perception_smartereye.h` / `third_party_perception_smartereye.cc`
- 继承自 `ThirdPartyPerception`，实现 Smartereye 方案

```cpp
class ThirdPartyPerceptionSmartereye : public ThirdPartyPerception {
 public:
  void OnSmartereye(const apollo::drivers::SmartereyeObstacles& message);
  void OnSmartereyeLanemark(const apollo::drivers::SmartereyeLanemark& message);
  bool Process(apollo::perception::PerceptionObstacles* const response) override;
};
```

构造函数创建两个 Reader，分别订阅 Smartereye 障碍物和车道线通道。`Process()` 直接输出摄像头检测到的障碍物（含车道线信息）。

## 核心函数

### ThirdPartyPerceptionComponent::Init

```cpp
bool ThirdPartyPerceptionComponent::Init()
```

组件初始化入口：

1. 调用 `GetProtoConfig()` 加载 `ThirdPartyPerceptionDevice` 配置
2. 根据 `device_type` 枚举创建对应的感知实例：
   - `SMARTEREYE` -> `ThirdPartyPerceptionSmartereye`
   - `MOBILEYE` -> `ThirdPartyPerceptionMobileye`
   - 其他 -> 基类 `ThirdPartyPerception`（仅接收定位和底盘数据）
3. 调用 `perception_->Init()` 和 `perception_->Start()` 完成初始化
4. 创建 `PerceptionObstacles` Writer 发布到 `FLAGS_perception_obstacle_topic`

### ThirdPartyPerceptionComponent::Proc

```cpp
bool ThirdPartyPerceptionComponent::Proc()
```

定时触发（100ms 周期）：调用 `perception_->Process()` 获取障碍物数据，然后通过 Writer 发布。

### ThirdPartyPerceptionMobileye::OnMobileye

```cpp
void ThirdPartyPerceptionMobileye::OnMobileye(const Mobileye& message)
```

Mobileye 数据回调：调用 `conversion_mobileye::MobileyeToPerceptionObstacles()` 将 Mobileye 原始数据转换为 `PerceptionObstacles`，存储到 `eye_obstacles_` 成员。转换过程中使用 `localization_` 和 `chassis_` 进行坐标校准。

### ThirdPartyPerceptionMobileye::OnContiRadar / OnDelphiESR

```cpp
void ThirdPartyPerceptionMobileye::OnContiRadar(const ContiRadar& message)
void ThirdPartyPerceptionMobileye::OnDelphiESR(const DelphiESR& message)
```

雷达数据回调：将原始雷达数据转换为 `RadarObstacles`（`conversion_radar` 工具），经 `filter::FilterRadarObstacles()` 过滤后，再转换为 `PerceptionObstacles` 存储到 `radar_obstacles_`。仅当 `FLAGS_enable_radar` 为 `true` 时启用雷达输出。

### ThirdPartyPerceptionMobileye::Process

```cpp
bool ThirdPartyPerceptionMobileye::Process(PerceptionObstacles* const response)
```

融合输出：调用 `fusion::EyeRadarFusion()` 将 `eye_obstacles_` 和 `radar_obstacles_` 融合，填充消息头后输出。融合后清空缓存。

### ThirdPartyPerceptionSmartereye::OnSmartereye

```cpp
void ThirdPartyPerceptionSmartereye::OnSmartereye(
    const apollo::drivers::SmartereyeObstacles& message)
```

Smartereye 数据回调：调用 `conversion_smartereye::SmartereyeToPerceptionObstacles()` 转换数据，同时融合当前车道线和定位/底盘信息。

## 配置

### Protobuf 配置

配置文件路径：`third_party_perception/conf/third_party_perception_component.pb.txt`

```protobuf
device_type: SMARTEREYE
```

对应 proto 定义（`proto/third_party_perception_component.proto`）：

```protobuf
enum ThirdPartyPerceptionDeviceType {
  SMARTEREYE = 0;
  MOBILEYE = 1;
}

message ThirdPartyPerceptionDevice {
  optional ThirdPartyPerceptionDeviceType device_type = 1;
}
```

### GFlags 参数

通过 `conf/third_party_perception.conf` 文件设置：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `third_party_perception_node_name` | `third_party_perception` | 节点名称 |
| `third_party_perception_freq` | `10` | 定时器频率 (Hz) |
| `enable_radar` | `true` | 是否启用雷达障碍物输出 |
| `use_conti_radar` | `true` | `true` 使用 Conti 雷达，`false` 使用 Delphi ESR |
| `mobileye_pos_adjust` | `3.0` | Mobileye 障碍物位置校准偏移量 (m) |
| `smartereye_pos_adjust` | `3.0` | Smartereye 障碍物位置校准偏移量 (m) |
| `radar_pos_adjust` | `3.0` | 雷达障碍物位置校准偏移量 (m) |
| `mobileye_id_offset` | `0` | Mobileye 障碍物 ID 偏移 |
| `smartereye_id_offset` | `0` | Smartereye 障碍物 ID 偏移 |
| `radar_id_offset` | `1000` | 雷达障碍物 ID 偏移 |
| `filter_y_distance` | `7.5` | 雷达 Y 轴过滤距离 (m) |
| `movable_speed_threshold` | `6.7` | 运动物体速度阈值 (m/s) |
| `movable_heading_threshold` | `1.5` | 运动物体航向角差异阈值 (rad) |
| `movable_frames_count_threshold` | `5` | 连续运动帧数阈值 |
| `keep_radar_frames` | `5` | Delphi ESR 保留帧数 |
| `max_mobileye_obstacle_length` | `31.2` | Mobileye 障碍物最大长度 (m) |
| `max_mobileye_obstacle_width` | `12.7` | Mobileye 障碍物最大宽度 (m) |
| `overwrite_mobileye_theta` | `true` | 覆盖 Mobileye 原始角度输出 |
| `default_car_length` | `5.0` | 默认小车包围盒长度 (m) |
| `default_truck_length` | `10.0` | 默认卡车包围盒长度 (m) |
| `default_bike_length` | `2.0` | 默认自行车包围盒长度 (m) |
| `default_ped_length` | `0.5` | 默认行人包围盒长度 (m) |
| `default_height` | `3.0` | 默认包围盒高度 (m) |

### DAG 配置

文件：`third_party_perception/dag/third_party_perception.dag`

```text
module_config {
    module_library : "modules/third_party_perception/libthird_party_perception_component.so"
    timer_components {
        class_name : "ThirdPartyPerceptionComponent"
        config {
            name: "third_party_perception"
            config_file_path: "/apollo/modules/third_party_perception/conf/third_party_perception_component.pb.txt"
            flag_file_path:  "/apollo/modules/third_party_perception/conf/third_party_perception.conf"
            interval: 100
        }
    }
}
```

`interval: 100` 表示组件每 100ms 执行一次 `Proc()` 方法。

## 调用关系

### Mobileye 融合方案数据流

```text
Mobileye (摄像头)          ContiRadar / DelphiESR (雷达)
    │                              │
    ▼                              ▼
OnMobileye()              OnContiRadar() / OnDelphiESR()
    │                              │
    ├── conversion_mobileye        ├── conversion_radar
    │   MobileyeToPerception       │   ContiToRadarObstacles
    │   Obstacles                  │   DelphiToRadarObstacles
    ▼                              │
eye_obstacles_                     ▼
                        filter::FilterRadarObstacles
                                    │
                                    ▼
                           radar_obstacles_

    ┌──────────────────────────────────────┐
    │  ThirdPartyPerceptionComponent::Proc │
    │          (100ms 定时触发)             │
    │                                      │
    │  fusion::EyeRadarFusion(             │
    │      eye_obstacles_,                 │
    │      radar_obstacles_)               │
    │              │                       │
    │              ▼                       │
    │  FillHeader + Writer->Write          │
    └──────────────┬───────────────────────┘
                   │
                   ▼
         /apollo/perception/obstacles
```

### Smartereye 方案数据流

```text
SmartereyeObstacles         SmartereyeLanemark
    │                              │
    ▼                              ▼
OnSmartereye()            OnSmartereyeLanemark()
    │                              │
    ├── conversion_smartereye      │
    │   SmartereyeToPerception     ▼
    │   Obstacles           smartereye_lanemark_
    │   (+ localization, chassis, lanemark)
    ▼
eye_obstacles_

    ┌──────────────────────────────────────┐
    │  ThirdPartyPerceptionComponent::Proc │
    │          (100ms 定时触发)             │
    │                                      │
    │  *response = eye_obstacles_          │
    │  FillHeader + Writer->Write          │
    └──────────────┬───────────────────────┘
                   │
                   ▼
         /apollo/perception/obstacles
```

所有回调函数通过 `third_party_perception_mutex_` 互斥锁保证线程安全，`Process()` 中获取同一把锁后读取缓存数据并发布。
