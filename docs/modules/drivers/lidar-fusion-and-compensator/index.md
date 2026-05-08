---
title: "激光雷达融合与补偿"
---

# 激光雷达融合与补偿模块

> 源码路径: `modules/drivers/lidar_fusion_and_compensator/`

## 概述

`FusionAndCompensatorComponent` 是 Apollo 中负责多激光雷达点云融合与运动补偿的 Cyber 组件。它从多个输入通道接收 `PointCloud` 消息，在等待时间窗口内完成数据对齐后，将所有点云转换到统一的目标坐标系，并根据车辆运动进行运动补偿（平移补偿和可选的旋转补偿），最终将融合后的点云写入输出通道。

核心处理流程如下：

1. 主通道接收到点云后，在 `wait_time_s` 时间窗口内等待其他输入通道数据到达
2. 根据 `max_interval_ms` 判断是否丢弃过期数据
3. 计算所有点云的时间戳区间 `[timestamp_min, timestamp_max]`
4. 通过 TF2 查询静态变换和动态变换
5. 对每个点云执行融合与运动补偿，写入输出通道

## 核心类

### FusionAndCompensatorComponent

继承自 `Component<PointCloud>`，通过 `CYBER_REGISTER_COMPONENT` 宏注册为 Cyber 组件。

**关键成员变量：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `conf_` | `FusionAndCompensatorConfig` | Protobuf 配置对象 |
| `tf2_buffer_ptr_` | `apollo::transform::Buffer*` | TF2 变换查询缓冲区指针 |
| `writer_` | `Writer<PointCloud>` | 融合结果输出 Writer |
| `readers_` | `vector<Reader<PointCloud>>` | 辅助输入通道 Reader 列表 |
| `static_tf_map_` | `map<string, Affine3f>` | 静态坐标变换缓存（雷达坐标系到目标坐标系） |
| `filter_map_` | `map<string, const FilterConfig*>` | 按 frame_id 索引的点云过滤配置 |
| `pool_size_` | `size_t` | 对象池大小，默认 10 |
| `reserved_point_size_` | `size_t` | 预分配点数，默认 500000 |

## 核心函数

### Init

```cpp
bool Init() override;
```

组件初始化入口。加载 Protobuf 配置，获取 TF2 Buffer 单例，根据 `output_channel` 创建 Writer，根据 `input_channel` 列表创建多个 Reader，并按 `frame_id` 建立过滤器索引。

### Proc

```cpp
bool Proc(const std::shared_ptr<PointCloud>& point_cloud) override;
```

消息回调主函数。主通道收到点云后触发，执行以下步骤：

1. 从对象池获取目标点云消息并初始化头部信息
2. 轮询等待辅助 Reader 数据，直到所有通道数据到达或超过 `wait_time_s`
3. 调用 `GetTimestampInterval` 计算全局时间戳区间
4. 依次对每个输入点云调用 `FusionAndCompensator` 进行融合补偿
5. 设置序列号、时间戳等头部字段，写入输出通道

### FusionAndCompensator

```cpp
bool FusionAndCompensator(
    const std::shared_ptr<PointCloud> source,
    const uint64_t& timestamp_min,
    const uint64_t& timestamp_max,
    std::shared_ptr<PointCloud> target);
```

单个点云的融合补偿核心逻辑。首先查找或查询源帧到目标帧的静态变换并缓存，然后查询 `world_frame_id` 到目标帧在 `timestamp_min` 和 `timestamp_max` 时刻的动态变换，最后调用 `MotionCompensation` 完成实际的点云变换。

### MotionCompensation

```cpp
void MotionCompensation(
    const std::shared_ptr<PointCloud>& source,
    const uint64_t& timestamp_min,
    const uint64_t& timestamp_max,
    const Eigen::Affine3d& world_tf_min_time,
    const Eigen::Affine3d& world_tf_max_time,
    const Eigen::Affine3f& lidar_tf,
    std::shared_ptr<PointCloud> target);
```

运动补偿主方法。根据 `rotation_compensation` 配置分为两种路径：

- **旋转补偿模式**：当两次姿态差异超过阈值（四元数标量部分 < `1 - 1e-8`，对应约 0.0003 rad）时，使用球面线性插值（SLERP）对每个点按时间比例进行平移+旋转变换
- **纯平移模式**：仅按时间比例进行线性插值平移补偿

每个点在变换前会先应用 `lidar_tf` 静态变换，并通过 `IsFilteredPoint` 检查是否被过滤。

### IsFilteredPoint

```cpp
bool IsFilteredPoint(const Eigen::Vector3f& point, const std::string& frame_id) const;
```

根据 `filter_config` 中配置的边界框（`min_x/max_x/min_y/max_y/min_z/max_z`）判断目标坐标系下的点是否应被过滤。点落在边界内则返回 `true`（过滤），否则返回 `false`（保留）。

### GetTimestampInterval

```cpp
void GetTimestampInterval(
    const std::vector<std::shared_ptr<PointCloud>>& point_clouds,
    uint64_t* timestamp_min,
    uint64_t* timestamp_max);
```

遍历所有点云中所有点的 `timestamp` 字段，计算全局最小和最大时间戳。

### QueryPoseAffineFromTF2

```cpp
bool QueryPoseAffineFromTF2(
    const uint64_t& timestamp,
    const std::string& target_frame_id,
    const std::string& source_frame_id,
    Eigen::Affine3d* pose);
```

通过 TF2 Buffer 查询指定时间戳下两个坐标系之间的变换关系，返回 `Eigen::Affine3d`（包含平移和旋转）。

## 配置

配置通过 Protobuf 消息 `FusionAndCompensatorConfig` 定义（`proto/config.proto`），示例文件为 `conf/config.pb.txt`。

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_interval_ms` | `uint32` | - | 最大允许的点云时间间隔（毫秒），超过则视为过期 |
| `drop_expired_data` | `bool` | - | 是否丢弃过期数据 |
| `input_channel` | `repeated string` | - | 辅助输入通道列表（不含主通道） |
| `wait_time_s` | `float` | - | 主通道收到数据后的等待时间（秒） |
| `output_channel` | `string` | - | 融合后点云的输出通道 |
| `transform_query_timeout` | `float` | `0.02` | TF2 变换查询超时时间（秒） |
| `world_frame_id` | `string` | `"world"` | 世界坐标系 frame_id |
| `target_frame_id` | `string` | - | 目标坐标系 frame_id，未设置时使用主通道的 frame_id |
| `rotation_compensation` | `bool` | `false` | 是否启用旋转补偿 |
| `translation_compensation` | `bool` | `true` | 是否启用平移补偿 |
| `filter_config` | `repeated FilterConfig` | - | 按 frame_id 配置的点云空间过滤规则 |

`FilterConfig` 消息支持 `frame_id`（必填）以及 `min_x/max_x/min_y/max_y/min_z/max_z`（可选）六个方向的边界过滤。

## 调用关系

```text
Proc
├── Reader::Observe / GetLatestObserved   (轮询获取辅助通道数据)
├── IsExpired                              (判断点云是否过期)
├── GetTimestampInterval                   (计算时间戳区间)
├── FusionAndCompensator                   (单点云融合补偿)
│   ├── QueryPoseAffineFromTF2             (查询静态变换并缓存)
│   ├── QueryPoseAffineFromTF2             (查询动态变换)
│   └── MotionCompensation                 (运动补偿)
│       └── IsFilteredPoint                (空间过滤)
└── Writer::Write                          (输出融合结果)
```

组件通过 Cyber 的 DAG/Launch 配置绑定主输入通道，`Proc` 在每次主通道消息到达时自动触发。辅助通道的 Reader 在 `Init` 阶段创建，由 `Proc` 内部主动轮询。
