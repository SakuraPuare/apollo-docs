---
title: "开放空间重规划决策器"
---

# 开放空间重规划决策器

> 源码路径: `modules/planning/tasks/open_space_replan_decider/`

## 概述

`OpenSpaceReplanDecider` 是开放空间规划（Open Space Planning）中的重规划决策器，负责判断当前帧是否需要触发重新规划。它继承自 `Decider` 基类，作为 Task 插件注册到 Cyber RT 插件管理器中。

该决策器的核心逻辑较为简洁：对比上一帧的开放空间优化轨迹与当前车辆速度，决定是否设置重规划标志位。当上一帧无优化轨迹且车辆处于低速或静止状态时，触发重规划。

## 核心类

### OpenSpaceReplanDecider

```cpp
class OpenSpaceReplanDecider : public Decider {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;

 private:
  apollo::common::Status Process(Frame *frame) override;
  OpenSpaceReplanDeciderConfig config_;
};
```

继承自 `Decider`，通过 `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏注册为 `Task` 类型插件，可在运行时动态加载。

## 核心函数

### Init

```cpp
bool OpenSpaceReplanDecider::Init(
    const std::string &config_dir, const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector);
```

初始化函数，调用基类 `Decider::Init` 完成通用初始化，然后通过 `Decider::LoadConfig` 加载 `OpenSpaceReplanDeciderConfig` 配置到成员变量 `config_`。

### Process

```cpp
apollo::common::Status OpenSpaceReplanDecider::Process(Frame *frame);
```

核心决策逻辑，判断是否需要重规划。判定条件如下：

1. 从 `DependencyInjector` 获取上一帧 `previous_frame`
2. 若 `frame` 或 `previous_frame` 为空，返回错误状态
3. 当以下两个条件同时满足时，设置 `replan_flag = true`：
   - 上一帧的 `optimizer_trajectory_data` 为空（无有效优化轨迹）
   - 当前车辆线速度低于 `max_abs_speed_when_stopped` 阈值（车辆处于低速或静止）
4. 否则设置 `replan_flag = false`

## 配置

Proto 定义文件为 `open_space_replan_decider.proto`，当前 `OpenSpaceReplanDeciderConfig` 消息体为空，不含额外配置参数。默认配置文件 `conf/default_conf.pb.txt` 同样为空。

## 调用关系

```text
Task (基类接口)
  └── Decider (决策器基类)
        └── OpenSpaceReplanDecider (开放空间重规划决策器)
              ├── Init()    ← 由 Task 执行框架调用
              └── Process() ← 由 Task 执行框架在每帧调用
```

`OpenSpaceReplanDecider` 通过 Cyber RT 插件机制注册，由 Planning 模块的 Task 执行框架在每帧规划流程中调用。它读取上一帧的开放空间信息和当前车辆状态，将重规划决策结果写入当前帧的 `open_space_info`，供后续开放空间规划流程使用。
