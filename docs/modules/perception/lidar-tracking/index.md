---
title: "激光雷达跟踪"
---

# 激光雷达跟踪

> 源码路径: `modules/perception/lidar_tracking/`

## 概述

激光雷达跟踪模块（lidar_tracking）负责对激光雷达感知的分割目标进行多帧关联跟踪。模块作为 Cyber 组件接收上游的 `LidarFrameMessage`，通过多激光雷达融合引擎（MlfEngine）对前景和背景目标分别执行数据关联、状态滤波和轨迹管理，最终输出携带跟踪 ID 和运动状态的 `SensorFrameMessage`。

模块采用插件化架构，核心跟踪引擎、匹配器、滤波器均通过注册器动态加载。跟踪流程包括：目标分离与坐标变换、二部图匹配关联、多级状态滤波（类型、方向、形状、运动）、轨迹结果收集与过期清理。

融合分类器（FusedClassifier）当前已注释禁用，预留了目标类型融合能力。

## 核心类

### LidarTrackingComponent

Cyber 组件入口，继承 `cyber::Component<LidarFrameMessage>`。负责加载配置、初始化多目标跟踪器和融合分类器（当前禁用），接收点云帧消息并调用跟踪流水线。

```cpp
class LidarTrackingComponent : public cyber::Component<LidarFrameMessage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;
 private:
  bool InternalProc(const std::shared_ptr<const LidarFrameMessage>& in_message,
                    const std::shared_ptr<SensorFrameMessage>& out_message);
  BaseMultiTargetTracker* multi_target_tracker_;
  BaseClassifier* fusion_classifier_;
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> writer_;
};
```

### MlfEngine

多激光雷达融合跟踪引擎，继承 `BaseMultiTargetTracker`。管理前景/背景轨迹数据和目标对象，协调匹配器与跟踪器完成每帧跟踪。将目标按前景（模型检测）和背景（聚类）分别处理。

```cpp
class MlfEngine : public BaseMultiTargetTracker {
 protected:
  std::vector<MlfTrackDataPtr> foreground_track_data_;
  std::vector<MlfTrackDataPtr> background_track_data_;
  std::vector<TrackedObjectPtr> foreground_objects_;
  std::vector<TrackedObjectPtr> background_objects_;
  std::unique_ptr<MlfTracker> tracker_;
  std::unique_ptr<MlfTrackObjectMatcher> matcher_;
};
```

### MlfTracker

轨迹状态管理器，维护一组滤波器链（Filter Chain）。对匹配到的轨迹调用所有滤波器的 `UpdateWithObject`，对未匹配轨迹调用 `UpdateWithoutObject`。

```cpp
class MlfTracker {
 protected:
  std::vector<MlfBaseFilter*> filters_;
  int global_track_id_counter_ = 0;
};
```

### MlfTrackObjectMatcher

数据关联匹配器，计算关联代价矩阵后调用二部图匹配算法求解。前台和后台目标使用不同的匹配策略（默认均为 GNN）。

```cpp
class MlfTrackObjectMatcher {
 protected:
  std::unique_ptr<MlfTrackObjectDistance> track_object_distance_;
  BaseBipartiteGraphMatcher* foreground_matcher_;
  BaseBipartiteGraphMatcher* background_matcher_;
  float max_match_distance_ = 4.0f;
};
```

### MlfTrackData

轨迹数据结构，继承 `TrackData`。存储目标历史对象的按传感器分组时间序列、缓存对象、预测状态、可见性时间戳和前后景概率。

```cpp
class MlfTrackData : public TrackData {
 public:
  std::map<std::string, TimedObjects> sensor_history_objects_;
  TimedObjects cached_objects_;
  double consecutive_invisible_time_ = 0.0;
  double latest_visible_time_ = 0.0;
  bool is_current_state_predicted_ = true;
  float foreground_track_prob_ = 0.0;
};
```

### TrackedObject

跟踪目标包装结构，封装原始 `base::ObjectPtr` 以及跟踪过程中产生的位置、速度、方向、形状特征、滤波状态等中间数据。

```cpp
struct TrackedObject {
  base::ObjectPtr object_ptr;
  Eigen::Vector3d center, velocity, direction, size;
  Eigen::Vector3d belief_velocity, belief_acceleration;
  Eigen::Vector3d output_velocity, output_direction, output_center, output_size;
  int track_id = -1;
  double tracking_time = 0.0;
};
```

### 滤波器类（MlfBaseFilter 子类）

| 滤波器 | 职责 |
|--------|------|
| `MlfTypeFilter` | 目标类型融合，支持单帧和时序融合（CCRF） |
| `MlfDirectionFilter` | 朝向滤波，卡尔曼滤波平滑目标方向角 |
| `MlfShapeFilter` | 形状滤波，基于凸包计算目标轮廓 |
| `MlfMotionFilter` | 运动滤波，常加速模型卡尔曼滤波估计速度和加速度 |

### GnnBipartiteGraphMatcher

贪心最近邻（GNN）二部图匹配实现，继承 `BaseBipartiteGraphMatcher`。对代价矩阵贪心选择最小代价配对。

### MlfTrackObjectDistance

轨迹-目标距离度量类。组合多种距离特征（位置、方向、尺寸、点数、直方图等）加权计算最终关联代价。前台/后台目标使用不同的权重配置。

## 核心函数

### LidarTrackingComponent::Init

初始化组件：加载 `LidarTrackingComponentConfig` protobuf 配置，创建输出 Channel Writer，通过注册器实例化并初始化多目标跟踪器。

```cpp
bool LidarTrackingComponent::Init() {
  LidarTrackingComponentConfig comp_config;
  GetProtoConfig(&comp_config);
  writer_ = node_->CreateWriter<SensorFrameMessage>(output_channel_name);
  multi_target_tracker_ = BaseMultiTargetTrackerRegisterer::GetInstanceByName(name);
  multi_target_tracker_->Init(tracker_init_options);
}
```

### LidarTrackingComponent::InternalProc

核心处理流水线：传递元数据（时间戳、序列号、传感器ID）到输出消息；调用跟踪器；将跟踪结果写入输出帧；统计延迟。

```cpp
bool LidarTrackingComponent::InternalProc(
    const std::shared_ptr<const LidarFrameMessage>& in_message,
    const std::shared_ptr<SensorFrameMessage>& out_message) {
  // 传递元数据，检查错误码
  multi_target_tracker_->Track(tracker_options, lidar_frame.get());
  // 收集输出帧：tracked_objects, hdmap, sensor2world_pose, cloud
}
```

### MlfEngine::Track

单帧跟踪主流程，分为六步：

1. **时间戳修正** — 可选使用帧时间戳替代目标检测时间
2. **全局偏移设置** — 首帧时设置世界坐标系到局部坐标系的偏移
3. **目标分离与变换** — 将前景/背景目标变换到局部坐标系
4. **数据关联** — 分别对前景和背景执行二部图匹配
5. **状态滤波** — 仅主传感器触发，对已关联轨迹更新滤波器链
6. **结果收集与清理** — 输出轨迹对象，清除过期轨迹

```cpp
bool MlfEngine::Track(const MultiTargetTrackerOptions& options, LidarFrame* frame) {
  SplitAndTransformToTrackedObjects(objects, sensor_info, timestamp);
  TrackObjectMatchAndAssign(match_options, foreground_objects_, "foreground", &foreground_track_data_);
  TrackObjectMatchAndAssign(match_options, background_objects_, "background", &background_track_data_);
  if (is_main_sensor) {
    TrackStateFilter(foreground_track_data_, timestamp);
    CollectTrackedResult(frame);
  }
  RemoveStaleTrackData("foreground", timestamp, &foreground_track_data_);
}
```

### MlfEngine::SplitAndTransformToTrackedObjects

从对象池批量获取 `TrackedObject`，将原始目标通过 `AttachObject` 变换到局部坐标系。根据 `is_clustered` 标记区分前景（模型检测）和背景（聚类），前台目标可选计算形状特征用于直方图匹配。

### MlfEngine::TrackObjectMatchAndAssign

调用 `MlfTrackObjectMatcher::Match` 完成关联。对匹配上的轨迹-目标对，将目标推入轨迹缓存；对未匹配的目标创建新轨迹。

### MlfEngine::CollectTrackedResult

从轨迹数据收集最终输出。根据轨迹年龄（`age_`）、预测状态（`is_current_state_predicted_`）、前方关键轨迹（`is_front_critical_track_`）和延迟输出策略决定是否输出该轨迹。将轨迹转换为 `base::Object` 写入帧。

### MlfTracker::UpdateTrackDataWithObject

更新轨迹数据：先调用所有滤波器的 `UpdateWithObject`，再将目标推入轨迹历史。

### MlfTrackObjectMatcher::Match

计算代价矩阵（`ComputeAssociateMatrix`），调用二部图匹配器求解，返回匹配对、未匹配轨迹和未匹配目标。

## 配置

### 组件配置

Protobuf 定义位于 `proto/lidar_tracking_component_config.proto`：

```protobuf
message LidarTrackingComponentConfig {
  optional string output_channel_name = 1;
  optional perception.PluginParam multi_target_tracker_param = 2;
  optional perception.PluginParam fusion_classifier_param = 3;
}
```

### 组件配置文件

`conf/lidar_tracking_config.pb.txt`：

```text
output_channel_name: "/perception/inner/PrefusedObjects"
multi_target_tracker_param {
  name: "MlfEngine"
  config_path: "perception/lidar_tracking/data/tracking"
  config_file: "mlf_engine.pb.txt"
}
```

### 引擎配置

`data/tracking/mlf_engine.pb.txt`：

```text
use_histogram_for_match: true
histogram_bin_size: 10
output_predict_objects: false
reserved_invisible_time: 0.3
use_frame_timestamp: true
pub_track_times: 1
```

| 参数 | 含义 |
|------|------|
| `use_histogram_for_match` | 前台目标是否使用形状直方图特征辅助匹配 |
| `reserved_invisible_time` | 目标消失后保留轨迹的时间（秒） |
| `output_predict_objects` | 是否输出纯预测（无观测）的轨迹 |
| `pub_track_times` | 轨迹发布前需要被观测到的最少次数 |

### 匹配器配置

`data/tracking/mlf_track_object_matcher.conf`：

```text
foreground_mathcer_method: "GnnBipartiteGraphMatcher"
background_matcher_method: "GnnBipartiteGraphMatcher"
bound_value: 100
max_match_distance: 3.0
```

### 滤波器链配置

`data/tracking/mlf_tracker.conf`：

```text
filter_name: "MlfTypeFilter"
filter_name: "MlfDirectionFilter"
filter_name: "MlfShapeFilter"
filter_name: "MlfMotionFilter"
```

## 调用关系

```text
LidarTrackingComponent::Proc
  |
  +-- InternalProc
       |
       +-- MlfEngine::Track
            |
            +-- SplitAndTransformToTrackedObjects
            |     |
            |     +-- TrackedObject::AttachObject (坐标变换)
            |     +-- TrackedObject::ComputeShapeFeatures (前台直方图)
            |
            +-- TrackObjectMatchAndAssign (前景)
            |     |
            |     +-- MlfTrackObjectMatcher::Match
            |           |
            |           +-- MlfTrackObjectDistance::ComputeDistance
            |           +-- GnnBipartiteGraphMatcher::Match
            |
            +-- TrackObjectMatchAndAssign (背景)
            |     +-- (同上)
            |
            +-- TrackStateFilter (仅主传感器)
            |     |
            |     +-- MlfTracker::UpdateTrackDataWithObject
            |           |
            |           +-- MlfTypeFilter::UpdateWithObject
            |           +-- MlfDirectionFilter::UpdateWithObject
            |           +-- MlfShapeFilter::UpdateWithObject
            |           +-- MlfMotionFilter::UpdateWithObject
            |           +-- MlfTrackData::PushTrackedObjectToTrack
            |
            +-- CollectTrackedResult
            |     +-- MlfTrackData::ToObject (轨迹转输出)
            |
            +-- RemoveStaleTrackData (清理过期轨迹)
```

**接口继承关系：**

```text
BaseMultiTargetTracker (interface)
  +-- MlfEngine

BaseBipartiteGraphMatcher (interface)
  +-- GnnBipartiteGraphMatcher
  +-- MultiHmBipartiteGraphMatcher

BaseClassifier (interface, 当前禁用)
  +-- FusedClassifier

MlfBaseFilter (interface)
  +-- MlfTypeFilter
  +-- MlfDirectionFilter
  +-- MlfShapeFilter
  +-- MlfMotionFilter
```

**Channel 关系：**

- 输入：`LidarFrameMessage`（来自上游激光雷达检测/分割模块）
- 输出：`/perception/inner/PrefusedObjects`（`SensorFrameMessage`，供下游融合模块使用）
