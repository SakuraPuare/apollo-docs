# Control Submodules 控制子模块函数级源码解析

本文聚焦 `modules/control/control_component/submodules/` 目录，按函数级粒度拆解控制模块的 **4 个 CyberRT 子模块**：预处理器、后处理器、横纵向控制器、MPC 控制器。

## 1. 模块定位

控制模块支持两种运行形态：

- **单体模式**（默认）：`ControlComponent` 串行执行所有步骤
- **子模块模式**（`FLAGS_use_control_submodules=true`）：拆分为独立 CyberRT 组件，通过 channel 级联

子模块模式的 DAG 管线：

```
LocalView ──> PreprocessorSubmodule ──> Preprocessor
                                            │
                    ┌───────────────────────┘
                    ▼
    ┌───────────────────────────────────┐
    │  LatLonControllerSubmodule (模式A) │
    │  MPCControllerSubmodule    (模式B) │
    └───────────────────────────────────┘
                    │
                    ▼
              ControlCommand
                    │
                    ▼
    PostprocessorSubmodule ──> ControlCommand (最终输出)
```

## 2. PreprocessorSubmodule — 预处理器

```cpp
class PreprocessorSubmodule final : public cyber::Component<LocalView> {
 public:
  bool Init() override;
  bool Proc(const shared_ptr<LocalView>& local_view) override;
 private:
  void CheckInput(LocalView*);
  void CheckTimestamp(const LocalView&);
  void ProducePreprocessorStatus(Preprocessor*);
  bool estop_;
  shared_ptr<DependencyInjector> injector_;
  shared_ptr<Reader<LocalView>> local_view_reader_;
  shared_ptr<Writer<Preprocessor>> preprocessor_writer_;
};
```

**职责**：接收原始传感器/规划输入，校验并发布预处理状态。

### 2.1 `Proc` 主流程

1. `CheckInput`：校验输入完整性
   - 检查定位、底盘、轨迹消息是否有效
   - 检查消息时间戳是否过期
2. `CheckTimestamp`：时间戳一致性检查
   - 检查各传感器消息的时间偏差
   - 超过阈值记录告警
3. E-Stop 检查：
   - 检查规划轨迹中的紧急停车标志
   - 设置 `estop_` 标志
4. `ProducePreprocessorStatus`：组装预处理状态消息
   - 填充车辆状态、轨迹信息、E-Stop 状态
   - 通过 `preprocessor_writer_` 发布

### 2.2 成员变量

| 变量 | 说明 |
|------|------|
| `estop_` | 紧急停车标志 |
| `injector_` | 依赖注入器 |
| `latest_replan_trajectory_header_` | 最新重规划轨迹头 |
| `monitor_logger_buffer_` | 监控日志缓冲 |

## 3. LatLonControllerSubmodule — 横纵向控制器

```cpp
class LatLonControllerSubmodule final : public cyber::Component<Preprocessor> {
 public:
  bool Init() override;
  bool Proc(const shared_ptr<Preprocessor>& preprocessor_status) override;
 private:
  void ProduceControlCoreCommand(const LocalView&, ControlCommand*);
  shared_ptr<ControlTask> lateral_controller_;
  shared_ptr<ControlTask> longitudinal_controller_;
  shared_ptr<DependencyInjector> injector_;
  shared_ptr<Writer<ControlCommand>> control_core_writer_;
};
```

**职责**：接收预处理状态，分别执行横向和纵向控制，合成控制指令。

### 3.1 `Init` 初始化

1. 加载配置文件
2. 创建 `DependencyInjector`
3. 通过 `ControlTaskAgent` 创建横向控制器（LatController）和纵向控制器（LonController）
4. 初始化 `control_core_writer_`

### 3.2 `Proc` 主流程

1. 从 `Preprocessor` 提取 `LocalView`
2. `ProduceControlCoreCommand`：
   - 调用 `lateral_controller_->ComputeControlCommand` → 设置 `steering_target`
   - 调用 `longitudinal_controller_->ComputeControlCommand` → 设置 `throttle`/`brake`
   - 合并为统一的 `ControlCommand`
3. E-Stop 处理：
   - 若 `estop_` 为 true，强制设置刹车
4. 通过 `control_core_writer_` 发布

### 3.3 控制器架构

```
Preprocessor → LocalView
       │
       ├── lateral_controller_ (LatController/LQR)
       │       → steering_target, steering_rate
       │
       └── longitudinal_controller_ (LonController/PID)
               → throttle, brake
       │
       └── ControlCommand (合并输出)
```

## 4. MPCControllerSubmodule — MPC 控制器

```cpp
class MPCControllerSubmodule final : public cyber::Component<Preprocessor> {
 public:
  bool Init() override;
  bool Proc(const shared_ptr<Preprocessor>& preprocessor_status) override;
 private:
  void ProduceControlCoreCommand(const LocalView&, ControlCommand*);
  shared_ptr<ControlTask> mpc_controller_;
  shared_ptr<DependencyInjector> injector_;
  shared_ptr<Writer<ControlCommand>> control_core_writer_;
};
```

**职责**：与 `LatLonControllerSubmodule` 接口相同，但使用单一 MPC 控制器联合优化横向和纵向。

### 4.1 与 LatLon 的区别

| 特性 | LatLonControllerSubmodule | MPCControllerSubmodule |
|------|--------------------------|----------------------|
| 控制器数量 | 2 个（横向+纵向） | 1 个（联合 MPC） |
| 控制策略 | 解耦 | 耦合 |
| 优化方法 | LQR + PID | Model Predictive Control |
| 适用场景 | 常规驾驶 | 高精度控制 |

### 4.2 `Proc` 主流程

1. 从 `Preprocessor` 提取 `LocalView`
2. `ProduceControlCoreCommand`：
   - 调用 `mpc_controller_->ComputeControlCommand` → 同时输出 `steering` + `throttle/brake`
3. E-Stop 处理
4. 发布 `ControlCommand`

## 5. PostprocessorSubmodule — 后处理器

```cpp
class PostprocessorSubmodule final : public cyber::Component<ControlCommand> {
 public:
  bool Init() override;
  bool Proc(const shared_ptr<ControlCommand>& control_command) override;
 private:
  shared_ptr<Writer<ControlCommand>> postprocessor_writer_;
};
```

**职责**：接收控制器输出的 `ControlCommand`，发布为最终执行指令。

- 最轻量的子模块
- 仅做消息转发，可扩展为后处理逻辑（如限幅、滤波）

## 6. 子模块配置

### 6.1 DAG 文件

```
# control.dag (单体模式)
module_config {
  module_library: "libcontrol_component.so"
  components {
    class_name: "ControlComponent"
  }
}

# lateral_longitudinal_module.dag (横纵向子模块模式)
module_config {
  module_library: "libcontrol_component.so"
  components {
    class_name: "PreprocessorSubmodule"
  }
  components {
    class_name: "LatLonControllerSubmodule"
  }
  components {
    class_name: "PostprocessorSubmodule"
  }
}

# mpc_module.dag (MPC 子模块模式)
module_config {
  module_library: "libcontrol_component.so"
  components {
    class_name: "PreprocessorSubmodule"
  }
  components {
    class_name: "MPCControllerSubmodule"
  }
  components {
    class_name: "PostprocessorSubmodule"
  }
}
```

### 6.2 Launch 文件

```
# control.launch (单体)
dag: conf/control.dag

# control_lateral_longitudinal_control.launch (横纵向)
dag: conf/lateral_longitudinal_module.dag

# control_mpc_control.launch (MPC)
dag: conf/mpc_module.dag
```

## 7. 与单体模式的对比

| 特性 | 单体模式 | 子模块模式 |
|------|---------|-----------|
| 运行形态 | 单进程单组件 | 多组件通过 channel 级联 |
| 调度 | ControlComponent 内串行 | CyberRT 调度器异步 |
| 灵活性 | 固定流程 | 可替换控制器子模块 |
| 调试 | 一步到位 | 可独立监控每个子模块 |
| 性能 | 低开销 | channel 通信有开销 |
| 默认 | 是 | 否（需配置切换） |