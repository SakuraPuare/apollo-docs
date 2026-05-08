---
title: "Open Space 规划子系统"
---

# Open Space 规划子系统

> 源码位置：`modules/planning/tasks/open_space_*/`

## 模块定位

Open Space 规划用于非结构化场景（泊车、掉头、窄道通行），不依赖车道线，而是在自由空间中搜索可行轨迹。它由多个协作任务组成，形成独立的规划流水线。

---

## 任务流水线

```
OpenSpacePreStopDecider → OpenSpaceRoiDecider → OpenSpaceTrajectoryProvider
    → OpenSpaceTrajectoryPartition → OpenSpaceFallbackDecider
```

| 任务 | 职责 |
|------|------|
| `OpenSpacePreStopDecider` | 在进入开放空间前减速停车 |
| `OpenSpaceRoiDecider` | 确定 ROI（感兴趣区域）和障碍物表示 |
| `OpenSpaceTrajectoryProvider` | 生成开放空间轨迹（核心优化） |
| `OpenSpaceTrajectoryPartition` | 将轨迹按档位分段 |
| `OpenSpaceFallbackDecider` | 轨迹执行失败时的兜底处理 |

---

## OpenSpaceRoiDecider — ROI 构建

```cpp
class OpenSpaceRoiDecider : public Decider {
  Status Process(Frame* frame) override;
  bool GetParkingSpot(Frame*, ParkingInfo*);
  bool GetPullOverSpot(Frame*, array<Vec2d,4>*, hdmap::Path*);
  void SetOrigin(const ParkingInfo&, Frame*);
};
```

核心功能：
1. 获取目标停车位（`GetParkingSpot`）或靠边停车点（`GetPullOverSpot`）
2. 设置坐标原点（归一化问题）
3. 构建障碍物的半平面表示（H-representation: Ax ≤ b）
4. 输出：`XYbounds`、`obstacles_A`、`obstacles_b`、`end_pose`

---

## OpenSpaceTrajectoryProvider — 轨迹生成

```cpp
class OpenSpaceTrajectoryProvider : public TrajectoryOptimizer {
  Status Process() override;
  void GenerateTrajectoryThread();  // 异步优化
  unique_ptr<OpenSpaceTrajectoryOptimizer> optimizer_;
};
```

### 异步执行模型

```
主线程：检查数据就绪 → 启动优化线程 → 等待/复用上帧结果
优化线程：HybridA* → DualWarmStart → DistanceApproach → Smoother
```

### OpenSpaceTrajectoryOptimizer — 优化器

```cpp
class OpenSpaceTrajectoryOptimizer {
  Status Plan(stitching_trajectory, end_pose, XYbounds, rotate_angle,
              translate_origin, obstacles_edges_num, obstacles_A, obstacles_b,
              obstacles_vertices_vec, time_latency);
 private:
  unique_ptr<HybridAStar> warm_start_;                    // 粗轨迹搜索
  unique_ptr<DualVariableWarmStartProblem> dual_warm_;    // 对偶变量热启动
  unique_ptr<DistanceApproachProblem> distance_approach_; // 距离优化
  unique_ptr<IterativeAnchoringSmoother> smoother_;       // 迭代平滑
};
```

优化流程：
1. **Hybrid A\*** — 在离散化空间中搜索粗轨迹（考虑车辆运动学）
2. **Dual Variable Warm Start** — 为 NLP 提供对偶变量初始值
3. **Distance Approach** — NLP 优化（最小化与障碍物距离的违反）
4. **Iterative Anchoring Smoother** — 迭代锚点平滑（备选方案）

---

## OpenSpaceTrajectoryPartition — 轨迹分段

```cpp
class OpenSpaceTrajectoryPartition : public TrajectoryOptimizer {
  Status Process() override;
  void PartitionTrajectory(const DiscretizedTrajectory&,
                           vector<TrajGearPair>*);
  bool CheckReachTrajectoryEnd(...);
};
```

功能：
- 将连续轨迹按前进/后退分段（`TrajGearPair` = 轨迹 + 档位）
- 插值使轨迹点等间距
- 检测是否到达当前段终点，切换到下一段
- 防止重复执行已走过的段

---

## OpenSpaceFallbackDecider — 兜底决策

```cpp
class OpenSpaceFallbackDecider : public Decider {
  Status Process(Frame*, ReferenceLineInfo*) override;
};
```

- 检测轨迹执行是否偏离过大
- 触发重规划或生成停车轨迹

---

## OpenSpacePreStopDecider — 预停车

```cpp
class OpenSpacePreStopDecider : public Decider {
  Status Process(Frame*, ReferenceLineInfo*) override;
};
```

- 在进入开放空间场景前，在参考线上设置停车点
- 确保车辆以低速/停止状态进入开放空间规划

---

## 数据结构

### OpenSpaceTrajectoryThreadData

```cpp
struct OpenSpaceTrajectoryThreadData {
  vector<TrajectoryPoint> stitching_trajectory;  // 拼接轨迹
  vector<double> end_pose;                       // 目标位姿 [x, y, θ, v]
  vector<double> XYbounds;                       // 空间边界 [xmin, xmax, ymin, ymax]
  double rotate_angle;                           // 坐标旋转角
  Vec2d translate_origin;                        // 坐标平移原点
  MatrixXi obstacles_edges_num;                  // 每个障碍物的边数
  MatrixXd obstacles_A;                          // 半平面 A 矩阵
  MatrixXd obstacles_b;                          // 半平面 b 向量
  vector<vector<Vec2d>> obstacles_vertices_vec;  // 障碍物顶点
};
```

### ParkingInfo

```cpp
struct ParkingInfo {
  ParkingType parking_type;
  string parking_id;
  vector<Vec2d> corner_points;  // 四角点
  Vec2d center_point;
  bool is_on_left;
};
```

---

## 与结构化规划的关系

| 特性 | 结构化规划 | Open Space 规划 |
|------|-----------|----------------|
| 场景 | 车道内行驶 | 泊车/掉头/窄道 |
| 参考 | 参考线 | 目标位姿 |
| 路径/速度 | 分离规划 | 联合规划（轨迹） |
| 搜索方法 | QP 优化 | Hybrid A* + NLP |
| 档位 | 通常前进 | 前进/后退交替 |
