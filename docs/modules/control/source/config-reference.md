# Control Configuration Reference 控制配置参考

本文档梳理 `modules/control/` 下的关键 Protobuf 配置定义，为理解和调优控制模块提供配置参考。

## 1. 模块定位

控制模块的配置分为四层：

1. **算法参数**：PID、Lead-Lag、MRAC 的增益和限幅
2. **控制器配置**：横向 LQR、纵向 PID 的完整参数集
3. **管线配置**：控制器执行顺序
4. **标定表**：速度/加速度 → 油门/刹车的映射

```
ControlPipeline (控制器执行顺序)
    ├── LatBaseLqrControllerConf (横向 LQR)
    │       ├── matrix_q (状态权重)
    │       ├── GainScheduler (增益调度)
    │       ├── LeadlagConf (Lead-Lag)
    │       └── MracConf (MRAC 自适应)
    └── LonBasedPidControllerConf (纵向 PID)
            ├── PidConf (位置 PID)
            ├── PidConf (速度 PID)
            └── LeadlagConf (Lead-Lag)
```

## 2. 算法参数

### 2.1 PidConf — PID 参数

```protobuf
message PidConf {
  optional bool integrator_enable = 1;              // 积分器使能
  optional double integrator_saturation_level = 2;  // 积分饱和限幅
  optional double kp = 3;                           // 比例增益
  optional double ki = 4;                           // 积分增益
  optional double kd = 5;                           // 微分增益
  optional double kaw = 6 [default = 0.0];          // Anti-Windup 增益
  optional double output_saturation_level = 7;      // 输出饱和限幅
}
```

| 参数 | 说明 | 典型值 |
|------|------|--------|
| `kp` | 比例增益 | 速度环: 0.5-2.0, 位置环: 0.1-1.0 |
| `ki` | 积分增益 | 速度环: 0.01-0.1, 位置环: 0.001-0.01 |
| `kd` | 微分增益 | 通常为 0 或很小 |
| `kaw` | Anti-Windup 增益 | 0.0-1.0（BC 模式使用） |
| `integrator_saturation_level` | 积分限幅 | 速度环: 50-200 |
| `output_saturation_level` | 输出限幅 | 根据执行器范围 |

### 2.2 LeadlagConf — Lead-Lag 参数

```protobuf
message LeadlagConf {
  optional double innerstate_saturation_level = 1 [default = 300];
  optional double alpha = 2 [default = 0.1];   // 超前系数
  optional double beta = 3 [default = 1.0];    // 增益系数
  optional double tau = 4 [default = 0.0];     // 滞后系数
}
```

传递函数：`G(s) = (alpha * s + beta) / (s + tau)`

| 参数 | 说明 | 调优方向 |
|------|------|---------|
| `alpha > 0` | 超前补偿 | 增大 → 改善响应速度 |
| `beta` | 增益 | 通常为 1.0 |
| `tau` | 滞后时间常数 | 增大 → 改善稳态精度 |
| `innerstate_saturation_level` | 内部状态限幅 | 防止溢出 |

### 2.3 MracConf — MRAC 自适应参数

```protobuf
message MracConf {
  optional int32 mrac_model_order = 1 [default = 1];  // 模型阶数 (1 或 2)
  optional double reference_time_constant = 2;          // 参考模型时间常数
  optional double reference_natural_frequency = 3;      // 自然频率 (2阶)
  optional double reference_damping_ratio = 4;          // 阻尼比 (2阶)
  repeated double adaption_state_gain = 5;              // 状态自适应增益
  optional double adaption_desired_gain = 6;            // 输入自适应增益
  optional double adaption_nonlinear_gain = 7;          // 非线性自适应增益
  repeated double adaption_matrix_p = 8;                // Lyapunov 方程 P 矩阵
  optional double mrac_saturation_level = 9 [default = 1.0];  // 输出限幅
  repeated double anti_windup_compensation_gain = 10;   // Anti-Windup 补偿增益
  optional double clamping_time_constant = 11;          // 钳位时间常数
}
```

| 参数 | 说明 | 调优方向 |
|------|------|---------|
| `mrac_model_order` | 参考模型阶数 | 1 阶简单快速，2 阶精确 |
| `reference_time_constant` | 时间常数 | 减小 → 更快响应 |
| `reference_natural_frequency` | 自然频率 | 增大 → 更快响应 |
| `reference_damping_ratio` | 阻尼比 | 0.7-1.0（临界阻尼附近） |
| `adaption_state_gain` | 状态自适应增益 | 增大 → 更快适应 |
| `mrac_saturation_level` | 输出限幅 | 防止过大修正 |

### 2.4 GainScheduler — 增益调度

```protobuf
message GainScheduler {
  message GainSchedulerInfo {
    optional double speed = 1;    // 速度阈值
    optional double ratio = 2;    // 增益比例
  }
  repeated GainSchedulerInfo scheduler = 1;
}
```

- 按速度插值调整增益比例
- 低速时增大横向误差权重，高速时增大航向误差权重

## 3. 控制器配置

### 3.1 LatBaseLqrControllerConf — 横向 LQR 配置

```protobuf
message LatBaseLqrControllerConf {
  optional double ts = 1;                           // 采样时间 (0.01s)
  optional int32 preview_window = 2;                // 前瞻窗口
  optional double cf = 3;                           // 前轮侧偏刚度 (N/rad)
  optional double cr = 4;                           // 后轮侧偏刚度 (N/rad)
  optional int32 mass_fl = 5;                       // 前左轮质量
  optional int32 mass_fr = 6;                       // 前右轮质量
  optional int32 mass_rl = 7;                       // 后左轮质量
  optional int32 mass_rr = 8;                       // 后右轮质量
  optional double eps = 9;                          // LQR 收敛阈值
  repeated double matrix_q = 10;                    // 状态权重矩阵 Q
  repeated double reverse_matrix_q = 11;            // 倒车状态权重
  optional int32 cutoff_freq = 12;                  // 滤波器截止频率
  optional int32 mean_filter_window_size = 13;      // 均值滤波窗口
  optional int32 max_iteration = 14;                // LQR 最大迭代次数
  optional double max_lateral_acceleration = 15;    // 最大横向加速度
  optional GainScheduler lat_err_gain_scheduler = 16;   // 横向误差增益调度
  optional GainScheduler heading_err_gain_scheduler = 17; // 航向误差增益调度
  optional LeadlagConf reverse_leadlag_conf = 18;   // 倒车 Lead-Lag
  optional bool enable_reverse_leadlag_compensation = 19;
  optional bool enable_look_ahead_back_control = 20;
  optional double lookahead_station = 21;            // 低速前瞻距离
  optional double lookback_station = 22;             // 低速回看距离
  optional MracConf steer_mrac_conf = 23;            // 转向 MRAC 配置
  optional bool enable_steer_mrac_control = 24;
  optional double lookahead_station_high_speed = 25; // 高速前瞻距离
  optional double lookback_station_high_speed = 26;  // 高速回看距离
  optional double lock_steer_speed = 27 [default = 0.081];  // 锁定方向盘速度
  optional double switch_speed = 36;                 // 低/高速切换速度
  optional double switch_speed_window = 37;          // 切换窗口
  optional double reverse_feedforward_ratio = 38;    // 倒车前馈比例
}
```

**关键参数调优**：

| 参数 | 说明 | 调优方向 |
|------|------|---------|
| `matrix_q` | `[lateral_err, lateral_err_rate, heading_err, heading_err_rate]` 权重 | 增大 → 更紧跟踪 |
| `cf / cr` | 前/后轮侧偏刚度 | 从车辆参数获取 |
| `max_lateral_acceleration` | 横向加速度限幅 | 减小 → 更保守转向 |
| `preview_window` | 前瞻周期数 | 增大 → 改善高速稳定性 |
| `lookahead_station` | 前瞻距离 | 增大 → 更平滑但响应慢 |

### 3.2 LonBasedPidControllerConf — 纵向 PID 配置

```protobuf
message LonBasedPidControllerConf {
  optional double ts = 1;                           // 采样时间
  optional double brake_minimum_action = 2;         // 最小刹车动作
  optional double throttle_minimum_action = 3;      // 最小油门动作
  optional double speed_controller_input_limit = 4; // 速度控制器输入限幅
  optional double station_error_limit = 5;          // 位置误差限幅
  optional double preview_window = 6;               // 前瞻窗口
  optional double standstill_acceleration = 7;      // 驻车加速度
  optional PidConf station_pid_conf = 8;            // 位置 PID
  optional PidConf low_speed_pid_conf = 9;          // 低速 PID
  optional PidConf high_speed_pid_conf = 10;        // 高速 PID
  optional double switch_speed = 11;                // 低/高速切换速度
  optional PidConf reverse_station_pid_conf = 12;   // 倒车位置 PID
  optional PidConf reverse_speed_pid_conf = 13;     // 倒车速度 PID
  optional FilterConf pitch_angle_filter_conf = 14; // 坡度角滤波
  optional LeadlagConf reverse_station_leadlag_conf = 15;  // 倒车位置 Lead-Lag
  optional LeadlagConf reverse_speed_leadlag_conf = 16;    // 倒车速度 Lead-Lag
  optional bool enable_reverse_leadlag_compensation = 18;
  optional double switch_speed_window = 19;         // 切换窗口
  optional bool enable_speed_station_preview = 20;  // 前瞻使能
  optional bool enable_slope_offset = 21;           // 坡度补偿
  optional double max_path_remain_when_stopped = 22; // 停车时最大剩余路径
  optional double pedestrian_stop_time = 27;        // 行人停车等待时间
  optional bool use_vehicle_epb = 36;               // 使用电子驻车
}
```

**PID 配置层次**：

| 层次 | 配置 | 说明 |
|------|------|------|
| 位置外环 | `station_pid_conf` | 位置误差 → 速度参考 |
| 速度内环（低速） | `low_speed_pid_conf` | 低速速度误差 → 加速度 |
| 速度内环（高速） | `high_speed_pid_conf` | 高速速度误差 → 加速度 |
| 倒车位置 | `reverse_station_pid_conf` | 倒车位置误差 |
| 倒车速度 | `reverse_speed_pid_conf` | 倒车速度误差 |

## 4. 管线配置

### 4.1 ControlPipeline — 控制器管线

```protobuf
message ControlPipeline {
  repeated PluginDeclareInfo controller = 1;  // 控制器插件列表
}
```

**典型管线配置**：

```protobuf
# conf/pipeline.pb.txt
controller {
  name: "lat_controller"
  type: "LatController"
}
controller {
  name: "lon_controller"
  type: "LonController"
}
```

### 4.2 PluginDeclareInfo — 插件声明

```protobuf
message PluginDeclareInfo {
  optional string name = 1;  // 插件名称
  optional string type = 2;  // 插件类型（类名）
}
```

## 5. 标定表

### 5.1 CalibrationTable — 标定表

```protobuf
message CalibrationTable {
  repeated CalibrationInfo calibration = 1;
}
message CalibrationInfo {
  optional double speed = 1;                      // 速度 (m/s)
  repeated ControlCommand calibration_point = 2;  // (加速度, 油门/刹车) 对
}
message ControlCommand {
  optional double command = 1;  // 油门正值 / 刹车负值
}
```

- 2D 查表：`(速度, 目标加速度) → 油门/刹车值`
- 使用 `Interpolation2D` 做双线性插值

## 6. 配置加载流程

```
ControlComponent::Init()
    │
    ├── 1. 加载 control.conf (gflags)
    ├── 2. 加载 pipeline.pb.txt (控制器管线)
    ├── 3. 加载 calibration_table.pb.txt (标定表)
    │
    └── ControlTaskAgent::Init()
            │
            ├── 创建 LatController
            │       └── 加载 lat_based_lqr_controller_conf.pb.txt
            │               ├── matrix_q, cf, cr (LQR 参数)
            │               ├── lat_err_gain_scheduler (增益调度)
            │               ├── steer_mrac_conf (MRAC 参数)
            │               └── reverse_leadlag_conf (Lead-Lag)
            │
            └── 创建 LonController
                    └── 加载 lon_based_pid_controller_conf.pb.txt
                            ├── station_pid_conf (位置 PID)
                            ├── low_speed_pid_conf (低速 PID)
                            ├── high_speed_pid_conf (高速 PID)
                            └── calibration_table (标定表)
```

## 7. 配置调优指南

### 7.1 横向控制调优

| 问题 | 参数调整 |
|------|---------|
| 跟踪不紧 | 增大 `matrix_q` 中横向误差权重 |
| 转向抖动 | 减小 `matrix_q` 中横向误差率权重，增大滤波 |
| 高速发散 | 增大 `preview_window`，增大航向误差权重 |
| 低速振荡 | 减小 `kp`，增大 `kd` |
| 倒车不稳 | 调整 `reverse_matrix_q`，启用 `reverse_leadlag_conf` |

### 7.2 纵向控制调优

| 问题 | 参数调整 |
|------|---------|
| 跟车不紧 | 增大 `station_pid_conf.kp` |
| 速度抖动 | 减小 `speed_pid_conf.kp`，增大 `kd` |
| 停车不稳 | 调整 `standstill_acceleration`，启用 EPB |
| 起步顿挫 | 调整 `throttle_minimum_action`，减小 `kp` |
| 坡道溜车 | 启用 `enable_slope_offset`，调整 `use_opposite_slope_compensation` |

### 7.3 常用参数范围

| 参数 | 低速 (0-10 km/h) | 中速 (10-60 km/h) | 高速 (>60 km/h) |
|------|------------------|-------------------|-----------------|
| `matrix_q[0]` (横向误差) | 10-50 | 5-20 | 2-10 |
| `matrix_q[2]` (航向误差) | 5-20 | 10-50 | 20-100 |
| `station_pid.kp` | 0.5-1.0 | 0.3-0.8 | 0.2-0.5 |
| `speed_pid.kp` | 1.0-3.0 | 0.5-2.0 | 0.3-1.0 |