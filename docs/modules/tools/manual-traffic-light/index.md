---
title: "手动交通灯工具"
---

# 手动交通灯工具

> 源码路径: `modules/tools/manual_traffic_light/manual_traffic_light.cc`

## 概述

手动交通灯工具是一个基于 `TimerComponent` 的调试组件，允许开发者通过键盘输入手动控制交通灯状态（红/绿切换）。该工具会将模拟的交通灯检测结果发布到 `traffic_light_detection` 话题，供下游规划模块使用。

主要功能：

- 订阅定位信息，获取车辆当前位置
- 根据位置从高精地图中查询附近交通灯信号
- 支持按 `c` 键切换红绿灯状态
- 将手动设置的灯色以 `TrafficLightDetection` 消息发布

## 核心类/函数

### ManualTrafficLight

继承自 `apollo::cyber::TimerComponent`，通过 `CYBER_REGISTER_COMPONENT` 宏注册。

```cpp
class ManualTrafficLight final : public apollo::cyber::TimerComponent {
 public:
  bool Init();   // 初始化 reader/writer
  bool Proc();   // 定时回调：查询信号灯、读取键盘、发布消息
};
```

### Init

创建定位消息订阅者和交通灯检测消息发布者：

```cpp
bool Init() {
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic, [this](const auto &localization) {
        OnLocalization(localization);
      });
  traffic_light_detection_writer_ =
      node_->CreateWriter<TrafficLightDetection>(
          FLAGS_traffic_light_detection_topic);
  return true;
}
```

### GetTrafficLightsWithinDistance

根据当前定位在高精地图上查询前方指定距离内的交通灯：

```cpp
bool GetTrafficLightsWithinDistance(
    std::vector<SignalInfoConstPtr> *traffic_lights) {
  auto position = localization_.pose().position();
  int ret = hdmap->GetForwardNearestSignalsOnLane(
      position, FLAGS_traffic_light_distance, traffic_lights);
  return ret == 0;
}
```

### GetKeyBoardColorInput

使用 `poll` 非阻塞读取键盘输入，按 `c` 切换红绿状态：

```cpp
TrafficLight::Color GetKeyBoardColorInput() {
  struct pollfd fd = {STDIN_FILENO, POLLIN, revent};
  switch (poll(&fd, 1, 100)) {
    default:
      char ch = 'x';
      std::cin >> ch;
      if (ch == 'c') {
        is_green_ = !is_green_;
        updated_ = true;
      }
  }
  return is_green_ ? TrafficLight::GREEN : TrafficLight::RED;
}
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--all_lights` | `false` | 设置地图上所有交通灯 |
| `--traffic_light_distance` | `1000.0` | 仅获取该距离内的交通灯（米） |

## 调用关系

```text
TimerComponent::Proc()
  ├── GetAllTrafficLights() 或 GetTrafficLightsWithinDistance()
  │     └── HDMapUtil::BaseMapPtr()->GetForwardNearestSignalsOnLane()
  ├── GetKeyBoardColorInput()
  │     └── poll() + stdin 读取
  ├── CreateTrafficLightDetection()
  │     └── 填充 TrafficLightDetection protobuf
  └── traffic_light_detection_writer_->Write()
```
