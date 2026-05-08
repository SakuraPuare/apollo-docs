# ObstacleNudgeDecider — 障碍物绕行决策器

> 源码位置：`modules/planning/tasks/obstacle_nudge_decider/`

## 模块定位

ObstacleNudgeDecider 负责判断前方静态障碍物是否需要绕行（nudge），通过概率跟踪机制避免频繁切换绕行状态。核心逻辑委托给 `NudgeCalculation` 类，使用 `ParkDataCenter` 持久化跟踪信息。

---

## 类结构

```cpp
class ObstacleNudgeDecider : public Task {
 public:
  bool Init(...) override;
  Status Execute(Frame*, ReferenceLineInfo*) override;
 private:
  Status Process(Frame*, ReferenceLineInfo*);
  ObstacleNudgeDeciderConfig config_;
  std::unique_ptr<NudgeCalculation> nudge_calc_;
};
```

---

## Process() — 主流程

```cpp
Status ObstacleNudgeDecider::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 更新历史绕行信息
  if (rli->index() == 0 && last_frame != nullptr)
    ParkDataCenter::Instance()->UpdateHistoryNudgeInfo(last_seq_num);

  // 跳过条件：未启用 / 路径可复用
  if (!FLAGS_enable_nudge_decider || !config_.enbale_nugde_static_obs()
      || rli->path_reusable()) return OK;

  // 获取当前是否处于绕行状态
  bool adc_in_nudge_state = planning_context->path_decider().is_in_path_lane_borrow_scenario();
  nudge_calc_->set_in_nudge_state(adc_in_nudge_state);

  // 执行绕行决策
  nudge_calc_->BuildNudgeDecisionWithObs(frame, last_frame, rli);
}
```

- `obstacle_nudge_decider.cc:L59-L82`

---

## NudgeCalculation — 绕行计算核心

### 构造函数

```cpp
NudgeCalculation::NudgeCalculation(const ObstacleNudgeDeciderConfig& config) {
  config_.CopyFrom(config);
  veh_config_.width = vehicle_param().width();
  veh_config_.length = vehicle_param().length();
  veh_config_.front_edge_to_center = vehicle_param().front_edge_to_center();
}
```

### BuildNudgeDecisionWithObs() — 主决策流程

```cpp
void NudgeCalculation::BuildNudgeDecisionWithObs(
    Frame* current_frame, const Frame* last_frame, ReferenceLineInfo* rli) {
  // 1. 计算前向检查距离（速度相关）
  check_forward_dis_ = Clamp(ego_v, 0, 8) / 8.0
      * (max_forward - min_forward) + min_forward;

  // 2. 从上一帧恢复跟踪信息
  ParkDataCenter::GetLastFrameNudgeTrackInfo(...);

  // 3. 遍历所有障碍物
  for (const auto* obstacle : obstacles) {
    if (!IsWithinNudgeScopeObstacle(rli, *obstacle)) continue;
    BuildStaticTrackingObs(rli, *obstacle, nudge_info);
  }

  // 4. 合并相邻障碍物多边形
  MergeObsPolygon(rli, nudge_info);

  // 5. 判断概率是否足够触发绕行
  IsProbEnoughToNudge(nudge_info, is_merge_obs);

  // 6. 计算绕行额外空间
  CalcNudgeExtraSpace(rli, nudge_info);
}
```

- `nudge_calculation.cc:L45-L130`

### CalcObsRemainSpace() — 计算剩余通行空间

```cpp
bool NudgeCalculation::CalcObsRemainSpace(
    ReferenceLineInfo* rli, const Obstacle& obstacle, RemainNudgeSpace* remain) {
  // 计算等效自车宽度（考虑曲率）
  double ego_whole_pass_width = 0.5 * width + CalculateEquivalentEgoWidth(...);

  // 获取车道和道路宽度
  GetLaneWidth(obs_s, &lane_left, &lane_right);
  GetRoadWidth(obs_s, &road_left, &road_right);
  GetLaneBoundaryType(obs_s, &left_type, &right_type);

  // 计算左右剩余空间
  remain->left_lane_remain = lane_left - ego_pass_width - obs_end_l - buffer;
  remain->right_lane_remain = lane_right - ego_pass_width + obs_start_l - buffer;

  // 路牙边界不扩展到道路宽度
  if (left_type == CURB) remain->left_road_remain = remain->left_lane_remain;
  else remain->left_road_remain = road_left - ego_pass_width - obs_end_l - buffer;

  // 可通行判定
  return left_lane_remain > obs_passable_buffer || right_lane_remain > obs_passable_buffer;
}
```

- `nudge_calculation.cc:L223-L278`

### MatchTrackingObsInfo() — 障碍物跟踪匹配

```cpp
bool NudgeCalculation::MatchTrackingObsInfo(
    const Obstacle& obstacle, NudgeInfo* nudge_info, ...) {
  for (auto& [id, info] : tracking_obs_info) {
    // 匹配条件：
    // 1. 跟踪多边形包含当前障碍物
    // 2. 多边形 IoU > 0.6
    if (info.expand_polygon.Contains(obstacle.PerceptionPolygon())
        || info.expand_polygon.ComputeIoU(obstacle.PerceptionPolygon()) > 0.6) {
      // 更新跟踪信息，替换 ID
      return true;
    }
  }
  return false;
}
```

- 解决障碍物 ID 跳变问题（感知重新分配 ID）
- `nudge_calculation.cc:L189-L221`

### MergeObsPolygon() — 合并相邻障碍物

- 找到第一个完全在车道内的障碍物多边形
- 将纵向距离 < 0.8m 的相邻障碍物合并为一个大多边形
- 合并后统一决策是否绕行
- `nudge_calculation.cc:L355-L420`

### IsProbEnoughToNudge() — 概率判定

- 基于跟踪时间累积绕行概率
- 概率超过阈值时触发绕行决策
- 避免对短暂出现的障碍物做出绕行反应

---

## 数据结构

```cpp
struct VehicleConfig {
  double width, length, front_edge_to_center;
};

struct RemainNudgeSpace {
  double left_lane_remain, right_lane_remain;
  double left_road_remain, right_road_remain;
  LaneBoundaryType left_lane_boundary_type, right_lane_boundary_type;
};
```

## 调用关系

- **上游**：Stage Task 流水线，在路径生成之前执行
- **持久化**：`ParkDataCenter` 存储跨帧跟踪信息
- **下游**：影响 `PathBoundsDecider` 的边界计算
- **依赖**：`SLPolygon`、`CalculateEquivalentEgoWidth`、`FLAGS_static_obstacle_nudge_l_buffer`
