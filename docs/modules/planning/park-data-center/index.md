---
title: "Park Data Center"
---

# Park Data Center

> 源码路径: `modules/planning/park_data_center/`

## 概述

Park Data Center 是泊车规划模块的数据中心，负责管理跨规划帧的 **Nudge（绕行/微避让）** 状态数据。以单例模式运行，维护当前帧和历史帧的障碍物 nudge 信息，供规划器在决策过程中查询和更新。

模块包含三部分核心逻辑：

- **ParkDataCenter**：单例数据中心，管理 NudgeInfo 的当前帧读写与历史帧存取
- **NudgeInfo / NudgeObstacleInfo**：描述单条参考线上的 nudge 障碍物跟踪状态、SL 多边形、nudge 概率等
- **LaneEscapeUtil**：判断车辆是否需要从当前车道逃逸（绕过静止阻塞障碍物），包含排队检测、空间分析和障碍物稳定性判断

## 核心类

### ParkDataCenter

单例类（`DECLARE_SINGLETON` 宏），管理 nudge 数据的帧级生命周期。内部持有 `NudgeInfoMap`（当前帧，按参考线 key 索引）和 `NudgeInfoIndexedQueue`（历史帧，按序列号索引，容量由 `FLAGS_max_frame_history_num` 控制）。

```cpp
class ParkDataCenter {
public:
    void UpdateHistoryNudgeInfo(uint32_t sequence_num);
    bool GetLastFrameNudgeTrackInfo(uint32_t sequence_num,
            std::size_t reference_line_key,
            std::unordered_map<std::string, NudgeObstacleInfo> *track_info);
    const NudgeInfo &current_nudge_info(std::size_t reference_line_key);
    NudgeInfo *mutable_current_nudge_info(std::size_t reference_line_key);
    void set_need_escape(bool need);
    bool is_need_escape();
private:
    NudgeInfoMap current_nudge_info_data_;
    NudgeInfoIndexedQueue history_nudge_info_data_;
    bool need_escape_ = false;
    DECLARE_SINGLETON(ParkDataCenter)
};
```

### NudgeObstacleInfo

描述单个障碍物的 nudge 跟踪状态，包含原始/扩展多边形、SL 边界、剩余空间（`RemainNudgeSpace`）、nudge 概率和类型（`NudgeType`：`NONE`/`LEFT`/`RIGHT`/`WAITING`）。

```cpp
struct NudgeObstacleInfo {
    Polygon2d origin_polygon, expand_polygon;
    SLBoundary sl_boundary;
    RemainNudgeSpace remain_nudge_space;
    bool is_passable = false;
    double nudge_probability = 0.0, tracking_time = 0.0;
    NudgeType nudge_type = NudgeType::NONE;
    double obs_tracking_time();  // last_track_time - start_track_time
};
```

### NudgeInfo

单条参考线的 nudge 信息集合，管理被跟踪障碍物映射、阻塞 SL 多边形和额外 nudge 关键点。

```cpp
class NudgeInfo {
private:
    bool is_enable_ = false;
    NudgeType nudge_type_ = NudgeType::NONE;
    bool is_merge_block_obs_ = false;
    std::unordered_map<std::string, NudgeObstacleInfo> tracking_nudge_obs_info_;
    std::set<std::string> update_ids_;
    std::vector<SLPolygon> block_sl_polygons_;
    std::vector<std::vector<SLPoint>> extra_nudge_key_points_;
};
```

## 核心函数

### ParkDataCenter::UpdateHistoryNudgeInfo

将当前帧 nudge 数据保存到历史队列，并清空当前帧。

```cpp
void ParkDataCenter::UpdateHistoryNudgeInfo(uint32_t sequence_num);
```

- **职责**：帧切换时保存当前帧 nudge 数据
- **输入**：`sequence_num` — 当前帧序号，作为历史队列键
- **关键步骤**：将 `current_nudge_info_data_` 拷贝到 `history_nudge_info_data_`（IndexedQueue），然后 `clear()`

### ParkDataCenter::GetLastFrameNudgeTrackInfo

查询上一帧中指定参考线的 nudge 障碍物跟踪信息。

```cpp
bool ParkDataCenter::GetLastFrameNudgeTrackInfo(uint32_t sequence_num,
        std::size_t reference_line_key,
        std::unordered_map<std::string, NudgeObstacleInfo> *track_info);
```

- **输入**：`sequence_num` 帧序号；`reference_line_key` 参考线标识
- **输出**：通过 `track_info` 指针返回障碍物 nudge 信息映射；找到返回 `true`

### NudgeInfo::GetInterpolatedNudgeL

在额外 nudge 关键点序列上进行线性插值，获取指定 s 处的横向偏移 l 值。

```cpp
bool NudgeInfo::GetInterpolatedNudgeL(bool is_left_nudge, double s, double* nudge_l) const;
```

- **职责**：根据 s 坐标插值计算 nudge 横向偏移
- **关键步骤**：通过 `lower_bound` 查找 s 所在区间，线性插值得到 `*nudge_l`

### NudgeInfo::IsObsIgnoreNudgeDecision

判断障碍物是否可忽略 nudge 决策。若障碍物不在 `update_ids_` 中且距 nudge 关键点足够远，则忽略。

```cpp
bool NudgeInfo::IsObsIgnoreNudgeDecision(std::string obs_id,
        double obs_start_s, double check_dis) const;
```

### NudgeInfo::SortBlockSLPolygons

按最小 s 坐标对阻塞 SL 多边形排序。

```cpp
void NudgeInfo::SortBlockSLPolygons();
```

### LaneEscapeUtil::IsNeedEscape

核心判断函数：车辆是否需要从当前车道逃逸以绕过阻塞障碍物。

```cpp
static bool IsNeedEscape(ReferenceLineInfo& ref_line_info,
        double min_distance_block_obs_to_junction, bool enable_junction_borrow,
        double passby_min_gap, double passby_kappa_ratio,
        int queue_check_count, int stable_block_count);
```

- **职责**：综合多条件判断是否触发车道逃逸
- **关键步骤**：
  1. 检查车辆静止、路径有效、距终点 > 20m
  2. 获取最近 stop decision 障碍物（`GetClosestStopDecisionObs`）
  3. 检查阻塞障碍物到路口距离
  4. `IsEnoughSpace` 检查通过空间
  5. `IsQueueSence` 排队识别 -- 若为排队则不逃逸
  6. `IsStableBlockObs` 障碍物稳定性 -- 持续阻塞才触发逃逸

### LaneEscapeUtil::IsEnoughSpace

判断阻塞障碍物周围是否有足够空间让车辆通过。

```cpp
static bool IsEnoughSpace(const ReferenceLineInfo& ref_line_info,
        const std::string& blocking_obstacle_id,
        const double &passby_min_gap, const double &passby_kappa_ratio);
```

- **关键步骤**：检查车道边界类型（实线禁止借道）；收集纵向范围内障碍物横向边界；计算最大可通行间隙；比较 `max_gap - ego_width > passby_min_gap + kappa * ratio`

### LaneEscapeUtil::IsQueueSence

判断阻塞场景是否为排队场景。运动车辆或短时间静止车辆视为排队；同一障碍物静止超过 `queue_check_count` 帧后不再视为排队。

```cpp
static bool IsQueueSence(ReferenceLineInfo& ref_line_info,
        const std::string& blocking_obstacle_id, const int &queue_check_count,
        std::pair<std::shared_ptr<Obstacle>, int> &queue_obs_info);
```

### LaneEscapeUtil::IsStableBlockObs

判断阻塞障碍物是否在同一位置持续阻塞超过 `stable_block_count` 帧。同一 ID 持续出现达到阈值后返回 `true`。

```cpp
static bool IsStableBlockObs(ReferenceLineInfo& ref_line_info,
        const std::string& blocking_obstacle_id, const int &stable_block_count,
        std::pair<std::string, int> &stable_block_obs);
```

### LaneEscapeUtil::GetClosestStopDecisionObs

遍历所有障碍物，获取最近的带有 stop 决策的静态非虚拟障碍物。若发现 ID 含 `SWEEPER_NUDGE_` 的障碍物则返回 `false`（清扫车 nudge 场景不触发逃逸）。

```cpp
static bool GetClosestStopDecisionObs(const ReferenceLineInfo& ref_line_info,
        std::string& stop_decision_obs_id, double& min_distance);
```

## 配置

| 参数 | 说明 | 来源 |
|------|------|------|
| `FLAGS_max_frame_history_num` | 历史帧队列最大长度 | `planning_gflags` |
| `FLAGS_min_stop_distance_obstacle` | 最小停车距离 | `planning_gflags` |
| `min_distance_block_obs_to_junction` | 阻塞障碍物到路口最小距离 | 调用方传入 |
| `passby_min_gap` | 通过最小间隙 | 调用方传入 |
| `passby_kappa_ratio` | 曲率补偿系数 | 调用方传入 |
| `queue_check_count` | 排队检测帧数阈值 | 调用方传入 |
| `stable_block_count` | 稳定阻塞帧数阈值 | 调用方传入 |

## 调用关系

```text
Planning Component
  +-- ParkDataCenter (singleton)
  |     +-- current_nudge_info_data_ : NudgeInfoMap (当前帧, 按参考线索引)
  |     |     +-- NudgeInfo
  |     |           +-- tracking_nudge_obs_info_ : NudgeObstacleInfo
  |     |           +-- block_sl_polygons_, extra_nudge_key_points_
  |     +-- history_nudge_info_data_ : NudgeInfoIndexedQueue (历史帧)
  +-- LaneEscapeUtil (static)
        +-- IsNeedEscape() --> GetClosestStopDecisionObs()
        |                  --> IsEnoughSpace()
        |                  --> IsQueueSence()
        |                  --> IsStableBlockObs()
        +-- set_need_escape() / is_need_escape() --> ParkDataCenter
```
