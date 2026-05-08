---
title: "PathDecider 与 SpeedDecider"
---

# PathDecider 与 SpeedDecider

> 源码位置：`modules/planning/tasks/path_decider/` 和 `modules/planning/tasks/speed_decider/`

## 模块定位

- **PathDecider**：路径规划后，根据路径与障碍物的横向关系生成 IGNORE / NUDGE / STOP 决策
- **SpeedDecider**：速度规划后，根据速度曲线与 ST 边界的位置关系生成 FOLLOW / YIELD / OVERTAKE / STOP 决策

两者共同完成从"几何规划结果"到"语义决策"的转换，决策结果供下游任务和调试使用。

---

## 一、PathDecider

### 类声明

```cpp
class PathDecider : public Task {
 private:
  Status Process(const ReferenceLineInfo*, const PathData&, PathDecision*);
  bool MakeObjectDecision(const PathData&, const string& blocking_id, PathDecision*);
  bool MakeStaticObstacleDecision(const PathData&, const string&, PathDecision*);
  bool IgnoreBackwardObstacle(PathDecision*);
  ObjectStop GenerateObjectStopDecision(const Obstacle&) const;
};
```

### Process() 主流程

```cpp
Status PathDecider::Process(const ReferenceLineInfo* rli,
                            const PathData& path_data,
                            PathDecision* path_decision) {
  // 1. 更新阻塞障碍物计数器（滞后判断）
  if (blocking_obstacle != nullptr) {
    counter = min(counter + 1, 10);   // 连续阻塞计数
  } else {
    counter = max(counter - 1, -10);  // 连续无阻塞计数
  }

  // 2. 对所有障碍物做决策
  MakeObjectDecision(path_data, blocking_id, path_decision);
}
```

### MakeStaticObstacleDecision() — 核心决策逻辑

对每个静态障碍物，根据其 SL 边界与路径的横向关系：

```
lateral_radius = half_width + lateral_ignore_buffer

if 障碍物不在路径 s 范围内:
  → IGNORE（纵向 + 横向）

if 障碍物横向距离 > lateral_radius:
  → IGNORE（横向太远）

if 障碍物与路径横向重叠 < min_nudge_l:
  → STOP（无法绕行，必须停车）

else:
  if 障碍物在路径左侧 → LEFT_NUDGE
  if 障碍物在路径右侧 → RIGHT_NUDGE
```

- `path_decider.cc:L129-L256`

### IgnoreBackwardObstacle()

```cpp
bool PathDecider::IgnoreBackwardObstacle(PathDecision* path_decision) {
  // 忽略所有在 ADC 后方的动态障碍物
  if (obstacle.end_s() < adc_start_s)
    → IGNORE（纵向）
}
```

---

## 二、SpeedDecider

### 类声明

```cpp
class SpeedDecider : public Task {
 private:
  enum STLocation { ABOVE = 1, BELOW = 2, CROSS = 3 };
  STLocation GetSTLocation(PathDecision*, SpeedData&, STBoundary&) const;
  Status MakeObjectDecision(const SpeedData&, PathDecision*) const;
  bool CreateStopDecision(...) const;
  bool CreateFollowDecision(...) const;
  bool CreateYieldDecision(...) const;
  bool CreateOvertakeDecision(...) const;
};
```

### MakeObjectDecision() — 核心决策逻辑

对每个有 ST 边界的障碍物：

```
1. 获取速度曲线与 ST 边界的位置关系
   location = GetSTLocation(speed_profile, st_boundary)

2. 根据位置关系生成决策：
   - ABOVE（速度曲线在 ST 边界上方）→ OVERTAKE（超车）
   - BELOW（速度曲线在 ST 边界下方）→ YIELD / FOLLOW / STOP
   - CROSS（速度曲线穿过 ST 边界）→ 异常情况
```

### GetSTLocation()

判断速度曲线相对于 ST 边界的位置：

- 遍历速度曲线每个点 (t, s)
- 与 ST 边界的上下界比较
- 统计 ABOVE/BELOW 的点数，确定整体位置关系

### BELOW 情况的细分决策

```
if 障碍物是 KEEP_CLEAR:
  → 检查是否可通过 / 是否被阻塞

if CheckIsFollow(obstacle):
  if IsFollowTooClose():
    → STOP
  else:
    → FOLLOW（跟车）

else:
  → YIELD（让行）
```

### CheckIsFollow()

```cpp
bool CheckIsFollow(const Obstacle& obstacle, const STBoundary& boundary) const {
  // 判断条件：
  // 1. 障碍物在 ADC 前方
  // 2. 障碍物与 ADC 同向行驶
  // 3. ST 边界从 t=0 开始（一直在前方）
}
```

### 决策生成函数

| 函数 | 决策类型 | 关键参数 |
|------|---------|---------|
| `CreateFollowDecision` | FOLLOW | 跟车距离 = `EstimateProperFollowGap(speed)` |
| `CreateYieldDecision` | YIELD | 让行距离 = 配置值 |
| `CreateOvertakeDecision` | OVERTAKE | 超车间距 = `EstimateProperOvertakingGap(...)` |
| `CreateStopDecision` | STOP | 停车距离 = `MinRadiusStopDistance` |

---

## 任务执行顺序

```
LaneFollowPath → PathDecider → SpeedBoundsDecider → PiecewiseJerkSpeed → SpeedDecider
     路径生成      路径决策        ST边界构建           速度优化           速度决策
```

PathDecider 在路径生成后运行，SpeedDecider 在速度优化后运行。
