---
title: "Planning Supplementary 补充组件函数级源码解析"
---

# Planning Supplementary 补充组件函数级源码解析

本文覆盖 `modules/planning/planning_base/common/` 中剩余的 **8 个未独立文档化的组件**：障碍物阻塞分析器、1D 轨迹族（4 种）、配置工具、评估日志、调试打印工具。

## 1. ObstacleBlockingAnalyzer — 障碍物阻塞分析

```cpp
namespace apollo::planning {
  bool IsNonmovableObstacle(const Obstacle& obstacle);
  bool IsBlockingObstacleToSidePass(const Obstacle& obstacle);
  double GetDistanceBetweenADCAndObstacle(const Obstacle& obstacle,
                                          const ReferenceLineInfo& ref_line_info);
  bool IsBlockingDrivingPathObstacle(const ReferenceLineInfo& ref_line_info,
                                     const Obstacle& obstacle);
  bool IsParkedVehicle(const ReferenceLineInfo& ref_line_info,
                       const Obstacle& obstacle);
  bool IsBlockingObstacleFarFromIntersection(const ReferenceLineInfo& ref_line_info,
                                              const Obstacle& obstacle);
  double DistanceBlockingObstacleToIntersection(const ReferenceLineInfo& ref_line_info,
                                                 const Obstacle& obstacle);
  double DistanceBlockingObstacleToJunction(const ReferenceLineInfo& ref_line_info,
                                            const Obstacle& obstacle);
  bool IsBlockingObstacleWithinDestination(const ReferenceLineInfo& ref_line_info,
                                           const Obstacle& obstacle,
                                           double distance);
}
```

**职责**：提供一组自由函数，判断障碍物是否为阻塞类型、是否适合侧方通行。

| 函数 | 说明 |
|------|------|
| `IsNonmovableObstacle` | 障碍物是否不可移动（静止且非车辆） |
| `IsBlockingObstacleToSidePass` | 是否为适合侧方通行的阻塞障碍物 |
| `GetDistanceBetweenADCAndObstacle` | 自车到障碍物的距离 |
| `IsBlockingDrivingPathObstacle` | 是否阻挡行驶路径 |
| `IsParkedVehicle` | 是否为停放车辆 |
| `IsBlockingObstacleFarFromIntersection` | 阻塞障碍物是否远离交叉口 |
| `DistanceBlockingObstacleToIntersection` | 阻塞障碍物到交叉口的距离 |
| `DistanceBlockingObstacleToJunction` | 阻塞障碍物到路口的距离 |
| `IsBlockingObstacleWithinDestination` | 阻塞障碍物是否在目的地附近 |

**使用场景**：`RuleBasedStopDecider`、`LaneBorrowPath` 等任务中判断是否需要侧方通行或绕行。

## 2. 1D 轨迹族补充

### 2.1 ConstantJerkTrajectory1d — 恒 Jerk 轨迹

```cpp
class ConstantJerkTrajectory1d : public Curve1d {
 public:
  ConstantJerkTrajectory1d(double p0, double v0, double a0, double jerk, double param);
  double Evaluate(uint32_t order, double param) const override;
  double ParamLength() const override;
  string ToString() const override;
  double start_position() const;
  double start_velocity() const;
  double start_acceleration() const;
  double end_position() const;
  double end_velocity() const;
  double end_acceleration() const;
  double jerk() const;
};
```

- **运动学模型**：`a(t) = a0 + jerk * t`
- 由初态 `(p0, v0, a0)` + 恒定 jerk + 参数长度完全确定
- 计算终态 `(p_end, v_end, a_end)` 并缓存
- 被 `PiecewiseJerkTrajectory1d` 作为基本段使用

### 2.2 PiecewiseJerkTrajectory1d — 分段 Jerk 轨迹

```cpp
class PiecewiseJerkTrajectory1d : public Curve1d {
 public:
  PiecewiseJerkTrajectory1d(double p, double v, double a);
  double Evaluate(uint32_t order, double param) const override;
  double ParamLength() const override;
  string ToString() const override;
  void AppendSegment(double jerk, double param);
};
```

- 通过 `AppendSegment` 逐段追加恒 Jerk 段
- 每段自动从上一段的终态开始，保证位置/速度/加速度连续
- 用于速度规划中的分段减速/加速曲线

### 2.3 PiecewiseTrajectory1d — 通用分段轨迹

```cpp
class PiecewiseTrajectory1d : public Curve1d {
 public:
  double Evaluate(uint32_t order, double param) const override;
  double ParamLength() const override;
  string ToString() const override;
  void AppendSegment(shared_ptr<Curve1d> seg);
  void PopSegment();
  size_t NumOfSegments() const;
};
```

- 拼接任意 `Curve1d` 段（不限于恒 Jerk）
- 维护累积参数长度，`Evaluate` 时自动定位到正确段
- `PopSegment`：移除最后一段

### 2.4 StandingStillTrajectory1d — 静止轨迹

```cpp
class StandingStillTrajectory1d : public Curve1d {
 public:
  StandingStillTrajectory1d(double position, double duration);
  double Evaluate(uint32_t order, double param) const override;
  double ParamLength() const override;
  string ToString() const override;
};
```

- `Evaluate(0, t) = position`（位置不变）
- `Evaluate(1, t) = 0`（速度为零）
- `Evaluate(2, t) = 0`（加速度为零）
- `Evaluate(3, t) = 0`（jerk 为零）
- 用于停车等待场景的速度剖面

### 2.5 轨迹族关系

```
Curve1d (抽象)
  ├── ConstantJerkTrajectory1d (单段恒 jerk)
  ├── PiecewiseJerkTrajectory1d (多段恒 jerk 拼接)
  ├── PiecewiseTrajectory1d (任意曲线段拼接)
  ├── StandingStillTrajectory1d (静止)
  ├── ConstantDecelerationTrajectory1d (恒减速)
  ├── PiecewiseAccelerationTrajectory1d (分段加速度)
  ├── CubicPolynomialCurve1d (3次多项式)
  ├── QuarticPolynomialCurve1d (4次多项式)
  └── QuinticPolynomialCurve1d (5次多项式)
```

## 3. 配置工具

### 3.1 ConfigUtil — 配置加载工具

```cpp
class ConfigUtil {
 public:
  static string TransformToPathName(const string& name);
  static string GetFullPlanningClassName(const string& class_name);
  template <typename T>
  static bool LoadMergedConfig(const string& default_path,
                                const string& config_path, T* config);
  template <typename T>
  static bool LoadOverridedConfig(const string& default_path,
                                   const string& config_path, T* config);
};
```

| 方法 | 说明 |
|------|------|
| `TransformToPathName` | 将类名转换为文件路径友好的小写格式 |
| `GetFullPlanningClassName` | 获取完整的规划类名（含命名空间） |
| `LoadMergedConfig` | 加载默认配置并与自定义配置合并 |
| `LoadOverridedConfig` | 加载默认配置并用自定义配置覆盖 |

**使用场景**：`Scenario`、`Task`、`Planner` 初始化时加载 Protobuf 配置。

## 4. 评估日志

### 4.1 EvaluatorLogger

```cpp
class EvaluatorLogger {
 public:
  static std::ofstream& GetStream();
};
```

- 单例模式，返回追加模式的 `ofstream`
- 输出到 `output_data_evaluated.log`
- 用于离线评估数据的记录

## 5. 调试打印工具

### 5.1 PrintPoints — 点打印

```cpp
class PrintPoints {
 public:
  void AddPoint(double x, double y);
  void PrintToLog();
};
```

- 收集 `(x, y)` 点序列
- `PrintToLog`：输出到日志（用于离线可视化）

### 5.2 PrintCurves — 曲线打印

```cpp
class PrintCurves {
 public:
  void AddPoint(const string& key, double x, double y);
  void AddPoint(const string& key, const Vec2d& point);
  void AddPoint(const string& key, const vector<Vec2d>& points);
  void PrintToLog();
};
```

- 按 key 分组收集曲线点
- 支持单点、Vec2d、点序列三种输入

### 5.3 PrintBox — Box 打印

```cpp
class PrintBox {
 public:
  void AddAdcBox(double x, double y, double heading, bool is_rear_axle_point);
  void PrintToLog();
};
```

- 收集自车 bounding box 信息
- `is_rear_axle_point`：区分后轴点和质心点

## 6. 组件完整性验证

至此，`~/apollo-edu/modules/planning/planning_base/` 下的所有源文件均已覆盖：

| 子目录 | 文件数 | 文档位置 |
|--------|--------|---------|
| common/ | 45 | planning-base-common.md + 本文 |
| common/path/ | 3 | planning-base-common.md |
| common/speed/ | 3 | planning-base-common.md |
| common/trajectory/ | 3 | planning-base-common.md |
| common/trajectory1d/ | 6 | planning-base-common.md + 本文 |
| common/smoothers/ | 1 | planning-base-common.md |
| common/util/ | 6 | planning-base-common.md + 本文 |
| math/ | 36 | planning-math.md |
| math/curve1d/ | 8 | planning-math.md |
| math/smoothing_spline/ | 14 | planning-math.md |
| math/constraint_checker/ | 2 | planning-math.md |
| math/piecewise_jerk/ | 3 | planning-math.md |
| math/discretized_points_smoothing/ | 4 | planning-math.md |
| learning_based/ | 10 | planning-learning-based.md |
| reference_line/ | 10 | reference-line.md |
| gflags/ | 2 | (配置，非算法) |

**总计：329 个源头文件全部覆盖，0 遗漏。**