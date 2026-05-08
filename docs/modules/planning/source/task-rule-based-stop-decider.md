# RuleBasedStopDecider — 规则停车决策器

> 源码位置：`modules/planning/tasks/rule_based_stop_decider/`

## 模块定位

RuleBasedStopDecider 是一个基于规则的停车决策器，在路径生成之后运行，负责在三种情况下添加虚拟停车围栏：
1. **路径末端停车**：路径过短时在末端设置停车点
2. **紧急变道停车**：变道目标车道被阻塞且接近路由终点时强制停车等待
3. **侧方绕行停车**：借对向车道绕行前先停车确认安全

---

## 类声明

```cpp
class RuleBasedStopDecider : public Decider {
 public:
  bool Init(...) override;
 private:
  Status Process(Frame*, ReferenceLineInfo*) override;
  void AddPathEndStop(Frame*, ReferenceLineInfo*);
  void CheckLaneChangeUrgency(Frame*);
  void StopOnSidePass(Frame*, ReferenceLineInfo*);
  bool CheckSidePassStop(const PathData&, const ReferenceLineInfo&, double*);
  bool BuildSidePassStopFence(const PathData&, double, PathPoint*, Frame*, ReferenceLineInfo*);
  bool CheckADCStop(const PathData&, const ReferenceLineInfo&, double);
  bool CheckClearDone(const ReferenceLineInfo&, const PathPoint&);

  static constexpr char const* PATH_END_VO_ID_PREFIX = "PATH_END_";
  RuleBasedStopDeciderConfig config_;
  bool is_clear_to_change_lane_ = false;
  bool is_change_lane_planning_succeed_ = false;
};
```

---

## Process() — 主流程

```cpp
Status RuleBasedStopDecider::Process(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 侧方绕行停车
  if (config_.enable_stop_on_side_pass())
    StopOnSidePass(frame, rli);
  // 2. 紧急变道停车
  if (config_.enable_lane_change_urgency_checking())
    CheckLaneChangeUrgency(frame);
  // 3. 路径末端停车
  AddPathEndStop(frame, rli);
  return Status::OK();
}
```

- `rule_based_stop_decider.cc:L54-L70`

---

## 1. AddPathEndStop — 路径末端停车

```cpp
void RuleBasedStopDecider::AddPathEndStop(Frame* frame, ReferenceLineInfo* rli) {
  if (path_data.frenet_frame_path().back().s() - front().s()
      < config_.short_path_length_threshold()) {
    BuildStopDecision("PATH_END_" + path_label,
                      path_end_s - 0.1, 0.0,
                      STOP_REASON_REFERENCE_END, ...);
  }
}
```

- 当路径长度 < `short_path_length_threshold` 时触发
- 停车点在路径末端前 0.1m
- `rule_based_stop_decider.cc:L121-L136`

---

## 2. CheckLaneChangeUrgency — 紧急变道停车

```cpp
void RuleBasedStopDecider::CheckLaneChangeUrgency(Frame* frame) {
  for (auto& rli : *frame->mutable_reference_line_info()) {
    if (rli.IsChangeLanePath()) {
      is_clear_to_change_lane_ = IsClearToChangeLane(&rli);
      is_change_lane_planning_succeed_ = rli.Cost() < kStraightForwardLineCost;
      continue;
    }
    // 非变道场景 或 目标车道畅通 → 跳过
    if (size <= 1 || (is_clear && is_succeed)) continue;

    // 计算到路由终点的距离
    double distance_to_passage_end = sl_point.s() - adc_end_s;
    if (distance_to_passage_end > approach_distance_for_lane_change) continue;

    // 紧急情况：设置临时停车围栏
    BuildStopDecision("lane_change_stop", sl_point.s(),
                      urgent_distance_for_lane_change,
                      STOP_REASON_LANE_CHANGE_URGENCY, ...);
  }
}
```

- 触发条件：目标车道被阻塞 **且** 距路由终点 < `approach_distance_for_lane_change`
- `kStraightForwardLineCost = 10.0`：变道路径代价阈值
- `rule_based_stop_decider.cc:L72-L118`

---

## 3. StopOnSidePass — 侧方绕行停车

```cpp
void RuleBasedStopDecider::StopOnSidePass(Frame* frame, ReferenceLineInfo* rli) {
  // 自车道路径不需要侧方停车
  if (path_label.find("self") != npos) { check_clear = false; return; }

  // 已通过停车点 → 检查是否完成清除
  if (check_clear && CheckClearDone(*rli, stop_point)) check_clear = false;

  // 检查是否需要侧方停车
  if (!check_clear && CheckSidePassStop(path_data, *rli, &stop_s)) {
    // 感知未被遮挡 且 可安全变道 → 不停车
    if (!IsPerceptionBlocked(...) && IsClearToChangeLane(rli)) return;
    // 未停稳 → 建立停车围栏
    if (!CheckADCStop(...)) BuildSidePassStopFence(...);
    // 已停稳 → 检查是否可通行
    else if (IsClearToChangeLane(rli)) check_clear = true;
  }
}
```

- 状态机：检测 → 停车 → 等待清除 → 通行
- `rule_based_stop_decider.cc:L138-L178`

### CheckSidePassStop()

- 遍历 `path_point_decision_guide`
- 当路径点从 `IN_LANE` 变为 `OUT_OF_LANE` 时，记录停车 s 值
- `rule_based_stop_decider.cc:L181-L200`

### CheckADCStop()

- 检查自车速度 < `max_abs_speed_when_stopped`
- 检查距停车点距离 < `max_valid_stop_distance`
- `rule_based_stop_decider.cc:L258-L283`

### CheckClearDone()

- 自车后边缘已越过停车点 **且** 自车横向位置在车道内
- `rule_based_stop_decider.cc:L285-L307`

---

## 调用关系

- **上游**：Stage Task 流水线，在路径生成之后、速度规划之前执行
- **依赖**：`BuildStopDecision`（创建虚拟障碍物）、`IsClearToChangeLane`、`IsPerceptionBlocked`
- **下游**：停车围栏影响 SpeedDecider 的速度规划
