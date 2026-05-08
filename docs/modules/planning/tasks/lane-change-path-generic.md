---
title: "LaneChangePathGeneric 换道路径规划"
---

# LaneChangePathGeneric 换道路径规划

> 源码路径：`modules/planning/tasks/lane_change_path_generic/`

## 概述

`LaneChangePathGeneric` 是规划模块的换道路径生成任务，在参考线为换道路径时生成平滑的换道轨迹。继承 `PathGeneration`，采用"边界决策 → 路径优化 → 评估"三段式流程。与借道（Lane Borrow）不同，换道是完整跨越车道线的行为，需要额外处理换道禁区、安全间距和换道状态机。

## 核心类

### LaneChangePathGeneric

```cpp
class LaneChangePathGeneric : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool DecidePathBounds(std::vector<PathBoundary>* boundary);
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);
  bool AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path);

  void UpdateLaneChangeStatus();
  void GetBoundaryFromLaneChangeForbiddenZone(PathBoundary* path_bound);
  bool GetBoundaryFromNudgeDecision(PathBoundary* path_bound);
  void GetLaneChangeStartPoint(const ReferenceLine& reference_line,
                                double adc_frenet_s, common::math::Vec2d* start_xy);
  bool IsClearToChangeLane(ReferenceLineInfo* reference_line_info);
  void UpdateStatus(double timestamp, ChangeLaneStatus::Status status_code,
                    const std::string& path_id);
  bool HysteresisFilter(double obstacle_distance, double safe_distance,
                        double distance_buffer, bool is_obstacle_blocking);
  void SetPathInfo(PathData* path_data);
  bool CheckLastFrameSucceed(const Frame* last_frame);
  void GetSLPolygons(std::vector<SLPolygon>* polygons);

  LaneChangePathGenericConfig config_;
  bool is_clear_to_change_lane_ = false;
  bool is_exist_lane_change_start_position_ = false;
  common::math::Vec2d lane_change_start_xy_;
};
```

**源码**：`modules/planning/tasks/lane_change_path_generic/lane_change_path_generic.h`

## 核心流程

### Process() — 主入口

```cpp
Status LaneChangePathGeneric::Process(Frame* frame,
                                       ReferenceLineInfo* reference_line_info) {
  UpdateLaneChangeStatus();
  if (!reference_line_info->IsChangeLanePath() || reference_line_info->path_reusable()) {
    return Status::OK();
  }
  if (status != ChangeLaneStatus::IN_CHANGE_LANE) {
    return Status(ErrorCode::PLANNING_ERROR, "Not satisfy lane change conditions");
  }

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) { return error; }
  if (!OptimizePath(..., &candidate_path_data)) { return error; }
  if (!AssessPath(&candidate_path_data, final_path)) { return error; }
  return Status::OK();
}
```

**关键步骤**：

1. `UpdateLaneChangeStatus()` 更新换道状态机
2. 检查是否为换道路径且不可复用
3. 检查状态是否为 `IN_CHANGE_LANE`
4. 执行三段式路径生成

### UpdateLaneChangeStatus() — 换道状态机

管理 `ChangeLaneStatus` 状态转换：

```text
CHANGE_LANE_FINISHED → IN_CHANGE_LANE    (冻结时间过后)
CHANGE_LANE_FAILED   → IN_CHANGE_LANE    (冻结时间过后)
IN_CHANGE_LANE       → CHANGE_LANE_FINISHED (path_id 变化)
IN_CHANGE_LANE       → CHANGE_LANE_FAILED   (上一帧 SPEED_FALLBACK)
```

**冻结时间配置**：

- `change_lane_fail_freeze_time`：失败后冻结时长
- `change_lane_success_freeze_time`：成功后冻结时长

### DecidePathBounds() — 路径边界决策

1. `InitPathBoundary()` 初始化边界
2. `GetBoundaryFromSelfLane()` 基于自车道获取粗边界
3. `ExtendBoundaryByADC()` 根据自车位置扩展边界
4. `UpdatePathBoundaryBySLPolygon()` 根据障碍物 SL 多边形收缩边界
5. `GetBoundaryFromLaneChangeForbiddenZone()` 处理换道禁区
6. `GetBoundaryFromNudgeDecision()` 根据 Nudge 信息调整边界
7. `AddExtraPathBound()` 添加额外边界点

### GetBoundaryFromLaneChangeForbiddenZone() — 换道禁区

当目标车道有障碍物阻塞（`is_clear_to_change_lane_` 为 false）时，在换道起点之前收缩边界，迫使路径在当前车道内行驶直到换道起点。

```cpp
void LaneChangePathGeneric::GetBoundaryFromLaneChangeForbiddenZone(
    PathBoundary* const path_bound) {
  if (is_clear_to_change_lane_) {
    is_exist_lane_change_start_position_ = false;
    return;
  }
  double lane_change_start_s = config_.lane_change_prepare_length() + adc_frenet_s;
  // 在 lane_change_start_s 之前，将边界收缩到自车道内
  for (size_t i = 0; i < path_bound->size(); ++i) {
    if (curr_s > lane_change_start_s) break;
    // 收缩 l_lower / l_upper 到自车道宽度
  }
}
```

### IsClearToChangeLane() — 换道安全检查

遍历所有动态障碍物，检查前方和后方安全间距：

- **同向**：前向安全距离 = max(10m, (v_ego - v_obs) *3s)；后向 = max(10m, (v_obs - v_ego)* 3s)
- **对向**：前向安全距离 = max(50m, (v_ego + v_obs) * 5s)；后向 = 1m

使用 `HysteresisFilter()` 提供 0.5m 滞回缓冲，避免频繁切换阻塞状态。

### OptimizePath() — 路径优化

与借道路径相同，使用 Piecewise Jerk 优化器。区别在于 `UpdatePathRefWithBound()` 而非 `UpdatePathRefWithBoundInSidePassDirection()`，路径参考权重更偏向目标车道中心线。

### AssessPath() — 路径评估

过滤无效路径（`IsValidRegularPath`），设置路径点决策信息（`SetPathInfo`），在接近目的地时裁剪出车道点。换道路径只有一条候选，直接选为最终路径。

### SetPathInfo() — 路径点标注

标记每个路径点的车道状态：

- `IN_LANE`：在车道内（含换道前和换道完成后）
- `OUT_ON_FORWARD_LANE`：跨越车道线期间

### CheckLastFrameSucceed()

检查上一帧是否为 `SPEED_FALLBACK` 类型，若是则说明上一帧失败，不继续换道。

## 辅助函数

| 函数 | 说明 |
|------|------|
| `HysteresisFilter()` | 带滞回的安全距离判断，已阻塞时用 +buffer，未阻塞时用 -buffer |
| `GetLaneChangeStartPoint()` | 根据 `lane_change_prepare_length` 计算换道起始 XY 坐标 |
| `CheckLastFrameSucceed()` | 检查上一帧换道是否成功（非 SPEED_FALLBACK） |
| `GetSLPolygons()` | 获取障碍物 SL 多边形，忽略 Nudge 决策中可忽略的障碍物 |

## 配置

通过 `LaneChangePathGenericConfig` protobuf 加载：

| 字段 | 说明 |
|------|------|
| `lane_change_prepare_length` | 换道准备段长度（米） |
| `change_lane_fail_freeze_time` | 换道失败后冻结时长（秒） |
| `change_lane_success_freeze_time` | 换道成功后冻结时长（秒） |
| `extend_adc_buffer` | 自车边界扩展缓冲（米） |
| `enable_forbidden_zone` | 是否启用换道禁区 |
| `path_optimizer_config` | 路径优化器配置 |

## 调用关系

- **父类**：`PathGeneration`（路径生成任务基类）
- **依赖**：`PathBoundsDeciderUtil`、`PathOptimizerUtil`、`PathAssessmentDeciderUtil`、`ParkDataCenter`
- **被调用**：规划器通过插件机制作为 Task 调用，仅在 `IsChangeLanePath()` 为 true 时执行
