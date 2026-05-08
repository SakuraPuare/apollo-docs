---
title: "RssDecider — RSS 安全距离决策器"
---

# RssDecider — RSS 安全距离决策器

> 源码位置：`modules/planning/tasks/rss_decider/`

## 模块定位

RssDecider 实现了 Intel 提出的 RSS（Responsibility-Sensitive Safety）模型，用于计算自车与前方最近障碍物之间的安全纵向距离。当实际距离小于 RSS 安全距离时，标记当前状态为不安全，并可选择清空速度数据触发减速。

### 依赖库

使用 `ad_rss` 第三方库（Intel RSS C++ 实现）：
- `RssSituationExtraction`：从世界模型提取场景
- `RssSituationChecking`：检查场景安全性
- `RssResponseResolving`：生成安全响应
- `RssResponseTransformation`：转换为加速度约束

---

## 类声明

```cpp
struct rss_world_model_struct {
  double front_obs_dist, obs_s_start, obs_s_end, obs_l_start, obs_l_end;
  double obs_speed, ego_v_s_start, ego_v_s_end, ego_v_l_start, ego_v_l_end;
  double lane_leftmost, lane_rightmost, lane_length, lane_width;
  double OR_front_lon_min/max, OR_front_lat_min/max;
  double OR_rear_lon_min/max, OR_rear_lat_min/max;
  double adc_vel, laneSeg_len_min/max, laneSeg_width_min/max;
};

class RssDecider : public Task {
 public:
  bool Init(...) override;
  Status Execute(Frame*, ReferenceLineInfo*) override;
 private:
  Status Process(Frame*, ReferenceLineInfo*);
  void rss_config_default_dynamics(::ad_rss::world::Dynamics*);
  void rss_create_ego_object(::ad_rss::world::Object*, double vel_lon, double vel_lat);
  void rss_create_other_object(::ad_rss::world::Object*, double vel_lon, double vel_lat);
  void rss_dump_world_info(const rss_world_model_struct&);
  RssDeciderConfig config_;
};
```

---

## Process() — 核心流程

```cpp
Status RssDecider::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 1. 找前方最近障碍物
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle->IsVirtual()) continue;
    if (!reference_line.HasOverlap(obstacle->PerceptionBoundingBox())) continue;
    if (obstacle_sl.end_s() <= ego_v_s_start) continue;  // 忽略后方
    double distance = obstacle_sl.start_s() - ego_v_s_end;
    if (distance > 0 && distance < front_obstacle_distance) {
      // 更新最近障碍物信息
    }
  }

  // 2. 无前方障碍物 → 安全，设置默认加速度范围
  if (front_obstacle_distance >= rss_max_front_obstacle_distance) {
    rss_info->set_is_rss_safe(true);
    return Status::OK();
  }

  // 3. 构建 RSS 世界模型
  rss_create_other_object(&leadingObject, nearest_obs_speed, 0);
  rss_create_ego_object(&followingObject, adc_velocity, 0.0);
  // 设置 OccupiedRegion（参数化坐标）
  // 构建 RoadSegment + WorldModel

  // 4. RSS 四步计算
  RssSituationExtraction::extractSituations(worldModel, situationVector);
  RssCheck.checkSituations(situationVector, responseStateVector);
  RssResponse.provideProperResponse(responseStateVector, properResponse);
  RssResponseTransformation::transformProperResponse(..., accelerationRestriction);

  // 5. 计算安全纵向距离
  calculateSafeLongitudinalDistanceSameDirection(leading, following, safeLonDistance);

  // 6. 设置结果
  if (responseStateVector[0].longitudinalState.isSafe) {
    rss_info->set_is_rss_safe(true);
  } else {
    rss_info->set_is_rss_safe(false);
    if (config_.enable_rss_fallback()) {
      reference_line_info->mutable_speed_data()->clear();  // 触发减速
    }
  }
  // 设置加速度约束范围
  rss_info->set_cur_dist_lon(currentLonDistance);
  rss_info->set_rss_safe_dist_lon(safeLonDistance);
  rss_info->set_acc_lon_range_minimum/maximum(...);
  rss_info->set_acc_lat_left/right_range_minimum/maximum(...);
}
```

- `rss_decider.cc:L54-L340`

---

## 辅助函数

### rss_config_default_dynamics()

```cpp
void RssDecider::rss_config_default_dynamics(Dynamics* dynamics) {
  dynamics->alphaLon.accelMax = Acceleration(3.5);
  dynamics->alphaLon.brakeMax = Acceleration(8.0);
  dynamics->alphaLon.brakeMin = Acceleration(1.0);
  dynamics->alphaLon.brakeMinCorrect = Acceleration(1.0);
  dynamics->alphaLat.accelMax = Acceleration(0.2);
  dynamics->alphaLat.brakeMin = Acceleration(0.8);
}
```

- 默认动力学参数：最大加速 3.5m/s²，最大制动 8.0m/s²
- `rss_decider.cc:L342-L352`

### rss_create_ego_object() / rss_create_other_object()

- 创建 RSS Object，设置速度和动力学参数
- ego 响应时间 1s，other 响应时间 2s
- `rss_decider.cc:L354-L394`

---

## 输出

写入 `ReferenceLineInfo::rss_info()`：
- `is_rss_safe`：当前是否 RSS 安全
- `cur_dist_lon`：当前纵向距离
- `rss_safe_dist_lon`：RSS 安全纵向距离
- `acc_lon_range_minimum/maximum`：纵向加速度约束
- `acc_lat_left/right_range_minimum/maximum`：横向加速度约束

## 调用关系

- **上游**：Stage Task 流水线中作为后处理任务
- **触发条件**：需要 path_data 和 speed_data 非空
- **下游影响**：`enable_rss_fallback` 开启时清空 speed_data 触发紧急减速
