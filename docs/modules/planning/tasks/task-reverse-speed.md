---
title: "ReverseSpeed 倒车速度规划"
---

# ReverseSpeed 倒车速度规划

> 源码位置：`modules/planning/tasks/reverse_speed/`

## 模块定位

ReverseSpeed 为倒车场景生成 ST 图数据。它不直接输出速度曲线，而是构建倒车路径上的障碍物 ST 边界和速度限制，供后续速度优化器使用。

继承关系：
```
Task → ReverseSpeed
```

---

## Execute 主流程

```cpp
Status ReverseSpeed::Execute(Frame* frame, ReferenceLineInfo* rli) {
  // 1. 沿倒车路径构建障碍物 ST 边界
  GetSTboundaries(path_data.discretized_path(), frame, &boundaries);

  // 2. 设置速度限制（倒车限速）
  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0, config_.speed_limit());
  speed_limit.AppendSpeedLimit(config_.total_time(), config_.speed_limit());

  // 3. 加载 ST 图数据
  st_graph_data->LoadData(
      boundaries,
      0.0,              // min_s_on_st_boundaries
      init_point,
      speed_limit,
      config_.speed_limit(),  // cruise_speed
      path_length,
      config_.total_time(),
      st_graph_debug);
}
```

---

## GetSTboundaries — 构建倒车 ST 边界

```cpp
void ReverseSpeed::GetSTboundaries(path_data, frame, boundaries) {
  for (obstacle : frame->GetObstacleList()) {
    for (path_point : path_data) {
      // 检查路径点处车辆包围盒是否与障碍物重叠
      if (CheckOverlap(path_point, obstacle->PerceptionPolygon(), l_buffer)) {
        // 计算障碍物在 ST 图上的矩形区域
        low_s = max(0, path_point.s() - back_edge_to_center);
        high_s = min(path_length, path_point.s() + obstacle_length);

        // 构建静态矩形 ST 边界（时间方向覆盖全程）
        lower_points = {(low_s, 0), (low_s, total_time)};
        upper_points = {(high_s, 0), (high_s, total_time)};

        st_boundary = STBoundary::CreateInstance(lower_points, upper_points);
        st_boundary.ExpandByS(s_buffer);
        st_boundary.SetBoundaryType(STOP);

        boundaries->push_back(&st_boundary);
        break;  // 每个障碍物只取第一个重叠点
      }
    }
  }
}
```

特点：
- 障碍物在 ST 图上表现为**静态矩形**（时间方向全覆盖）
- 类型统一设为 `STOP`（倒车时遇到障碍物必须停车）

---

## CheckOverlap — 碰撞检测

```cpp
bool ReverseSpeed::CheckOverlap(path_point, obs_polygon, l_buffer) {
  // 将路径点从后轴中心转换为车辆几何中心
  ego_center = path_point + offset.Rotate(theta);

  // 构建 ADC 包围盒（宽度加上 l_buffer）
  Box2d adc_box(ego_center, theta, length, width + 2*l_buffer);

  // 多边形碰撞检测
  return obs_polygon.HasOverlap(Polygon2d(adc_box));
}
```

---

## 关键配置参数

| 参数 | 说明 |
|------|------|
| `speed_limit` | 倒车速度限制 |
| `total_time` | ST 图总时长 |
| `l_buffer` | 横向碰撞检测缓冲 |
| `s_buffer` | ST 边界纵向膨胀量 |
