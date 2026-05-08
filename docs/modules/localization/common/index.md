---
title: "定位通用工具"
---

# localization/common

> 源码路径: `modules/localization/common/`

## 概述

`common` 模块为定位子系统提供通用基础组件，包含两部分：

- **GNSS 时间补偿器** (`gnss_compensator`)：当 GNSS 设备返回无效时间戳时，利用 TF 发送时间和历史有效 GNSS 时间进行推算补偿，保证下游模块始终获得可用的时间基准。
- **全局配置标志** (`localization_gflags`)：通过 gflags 集中声明定位模块的全部运行参数，涵盖 GNSS、IMU、LiDAR、NDT、积分导航等多个子模块的配置项。

## 核心类

### LocalizationGnssCompensator

单例类，用于补偿无效 GNSS 数据中的时间戳。

```cpp
class LocalizationGnssCompensator {
 public:
  ~LocalizationGnssCompensator();

  void ProcessCompensation(
      const uint64_t& current_send_tf_time, uint64_t* measurement_time);

 private:
  uint64_t last_send_tf_time_ = 0;
  uint64_t last_valid_gnss_time_ = 0;
  uint64_t last_compensated_gnss_time_ = 0;

  DECLARE_SINGLETON(LocalizationGnssCompensator)
};
```

**私有成员说明：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `last_send_tf_time_` | `uint64_t` | 上一次发送 TF 消息的时间 |
| `last_valid_gnss_time_` | `uint64_t` | 上一次有效 GNSS 数据的时间戳 |
| `last_compensated_gnss_time_` | `uint64_t` | 上一次补偿后的时间戳，用于连续无效场景的累积推算 |

## 核心函数

### LocalizationGnssCompensator::ProcessCompensation

```cpp
void ProcessCompensation(
    const uint64_t& current_send_tf_time, uint64_t* measurement_time);
```

对 GNSS 测量时间戳进行补偿，处理逻辑如下：

1. **首次调用初始化**：若 `last_valid_gnss_time_` 和 `last_send_tf_time_` 均为 0，则将当前测量时间直接记录为有效值。
2. **计算时间增量**：`compensated_delta = current_send_tf_time - last_send_tf_time_`。
3. **判定无效数据**：当增量大于容差阈值（`FLAGS_gps_imu_compensate_ns_tolerance`）且当前测量时间不大于上一次有效 GNSS 时间时，认为当前数据时间戳无效。
4. **执行补偿**：累加增量到 `last_compensated_gnss_time_`，若 `FLAGS_enable_gps_imu_compensate` 为 `true`，则用补偿值替换原始测量时间。
5. **记录有效数据**：若数据有效，更新 `last_valid_gnss_time_` 和 `last_compensated_gnss_time_`。
6. 无论何种情况，最后都会更新 `last_send_tf_time_`。

## 配置

`localization_gflags` 模块通过 gflags 定义了定位子系统的全部运行参数，按功能分组如下：

### 通用基础参数

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `localization_module_name` | string | `"localization"` | 模块名称 |
| `localization_publish_freq` | double | `100` | 定位发布频率 (Hz) |
| `localization_config_file` | string | `"...pb.txt"` | 定位配置文件路径 |
| `enable_watchdog` | bool | `true` | 是否启用看门狗 |
| `enable_gps_timestamp` | bool | `false` | 是否使用 GPS 时间戳作为定位头时间戳 |

### GNSS/IMU 时间补偿参数

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_gps_imu_interprolate` | bool | `true` | 启用 GPS/IMU 插值 |
| `enable_gps_imu_compensate` | bool | `false` | 启用 GPS/IMU 时间戳补偿 |
| `gps_imu_compensate_ns_tolerance` | int32 | `10000000` | 补偿容差（纳秒） |
| `gps_time_delay_tolerance` | double | `1.0` | GPS 消息延迟容差（秒） |
| `gps_imu_timestamp_sec_diff_tolerance` | double | `0.02` | GPS/IMU 时间戳差值容差（秒） |
| `timestamp_sec_tolerance` | double | `1e-7` | 时间戳秒级容差 |

### 地图偏移参数

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `map_offset_x/y/z` | double | `0.0` | 地图坐标偏移量 |

### LiDAR 定位参数

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lidar_localization_mode` | int32 | `2` | 定位模式（0=强度, 1=高程, 2=融合） |
| `lidar_yaw_align_mode` | int32 | `2` | 航向角对齐模式 |
| `lidar_filter_size` | int32 | `17` | 滤波器大小 |
| `lidar_imu_max_delay_time` | double | `0.4` | LiDAR 与 IMU 最大延迟（秒） |
| `lidar_map_coverage_theshold` | double | `0.9` | 地图覆盖阈值 |
| `point_cloud_step` | int32 | `2` | 点云采样步长 |
| `if_use_avx` | bool | `false` | 是否使用 AVX 指令加速 |

### GNSS 天线/IMU 杆臂参数

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_ins_aid_rtk` | bool | `false` | 启用 INS 辅助 RTK |
| `gnss_mode` | int32 | `0` | GNSS 模式（0=最优GNSS位姿, 1=自解算GNSS） |
| `imu_to_ant_offset_x/y/z` | double | `0.0` | IMU 到天线杆臂偏移 |
| `imu_to_ant_offset_ux/uy/uz` | double | `0.0` | 杆臂偏移不确定性 |

### 积分导航参数

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `integ_ins_can_self_align` | bool | `false` | INS 是否自对准 |
| `integ_sins_align_with_vel` | bool | `true` | SINS 是否速度对准 |
| `integ_sins_state_check` | bool | `false` | SINS 状态检查 |
| `integ_sins_state_span_time` | double | `60.0` | SINS 状态跨度时间（秒） |
| `integ_sins_state_pos_std` | double | `1.0` | SINS 位置标准差 |

### NDT 定位参数

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ndt_max_iterations` | int32 | `10` | NDT 匹配最大迭代次数 |
| `online_resolution` | double | `2.0` | 在线点云分辨率 |
| `ndt_target_resolution` | double | `1.0` | 目标分辨率 |
| `ndt_line_search_step_size` | double | `0.1` | 线搜索步长 |
| `ndt_transformation_epsilon` | double | `0.01` | 变换收敛条件 |
| `ndt_filter_size_x/y` | int32 | `48` | NDT 搜索区域尺寸 |
| `ndt_bad_score_count_threshold` | int32 | `10` | 连续差评分计数阈值 |
| `ndt_warnning_ndt_score` | double | `1.0` | NDT 评分警告阈值 |
| `ndt_error_ndt_score` | double | `2.0` | NDT 评分错误阈值 |

### 状态监控阈值

| Flag | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `imu_delay_time_threshold_1/2/3` | double | `0.1/0.05/0.02` | IMU 延迟时间分级阈值（秒） |
| `imu_missing_time_threshold_1/2/3` | double | `0.1/0.05/0.01` | IMU 丢失时间分级阈值（秒） |
| `bestgnsspose_loss_time_threshold` | double | `2.0` | BestGNSS 位姿丢失时间阈值（秒） |
| `lidar_loss_time_threshold` | double | `2.0` | LiDAR 位姿丢失时间阈值（秒） |
| `localization_std_x/y_threshold_1` | double | `0.15` | 定位标准差一级阈值 |
| `localization_std_x/y_threshold_2` | double | `0.3` | 定位标准差二级阈值 |

## 调用关系

```text
localization::LocalizationGnssCompensator::ProcessCompensation
  ├── 读取 FLAGS_gps_imu_compensate_ns_tolerance  (localization_gflags)
  ├── 读取 FLAGS_enable_gps_imu_compensate         (localization_gflags)
  └── 输出 AINFO 日志（补偿详情）
```

`LocalizationGnssCompensator` 作为单例被定位主流程调用，通常在发送 TF 消息时触发 `ProcessCompensation`，确保 GNSS 测量时间戳在异常情况下仍可被下游模块使用。`localization_gflags` 中定义的配置项则被定位模块各个子系统广泛引用，是模块运行时参数的统一入口。
