---
title: "RTK Planner"
---

# RTK Planner

> 源码路径: `modules/planning/planners/rtk/`

## 概述

RTK Replay Planner 是 Apollo 规划模块中最基础的规划器之一，用于**回放预先录制的 RTK 轨迹**。它从轨迹文件中读取一条完整的参考轨迹，根据车辆当前位置在该轨迹上找到最近匹配点，然后截取前方一段轨迹作为规划输出。该规划器主要用于离线测试和回放场景，不涉及实时路径搜索或优化算法。

继承关系：

- `Planner`（纯虚基类）
  - `PlannerWithReferenceLine`（增加参考线维度的规划接口）
    - `RTKReplayPlanner`（RTK 回放规划器）

## 核心类

```cpp
class RTKReplayPlanner : public PlannerWithReferenceLine {
 public:
  std::string Name() override { return "RTK"; }

  apollo::common::Status Init(
      const std::shared_ptr<DependencyInjector>& injector,
      const std::string& config_path = "") override;

  apollo::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ADCTrajectory* ptr_computed_trajectory) override;

  apollo::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;

  void ReadTrajectoryFile(const std::string& filename);

 private:
  std::uint32_t QueryPositionMatchedPoint(
      const common::TrajectoryPoint& start_point,
      const std::vector<common::TrajectoryPoint>& trajectory) const;

  std::vector<common::TrajectoryPoint> complete_rtk_trajectory_;
};
```

`RTKReplayPlanner` 通过 Cyber RT 插件机制注册为 `Planner` 插件。核心数据成员 `complete_rtk_trajectory_` 存储从文件中读取的完整轨迹点序列。

## 核心函数

### Init

```cpp
Status RTKReplayPlanner::Init(
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_path) {
  Planner::Init(injector, config_path);
  ReadTrajectoryFile(FLAGS_rtk_trajectory_filename);
  return Status::OK();
}
```

- **职责**: 初始化规划器，加载预录轨迹文件
- **输入**: `injector` 依赖注入器、`config_path` 配置路径
- **输出**: `Status` 初始化状态
- **关键步骤**:
  1. 调用父类 `Planner::Init` 完成通用初始化
  2. 通过全局 flag `FLAGS_rtk_trajectory_filename` 获取轨迹文件路径
  3. 调用 `ReadTrajectoryFile` 解析文件内容到 `complete_rtk_trajectory_`

### Plan

- **职责**: 主规划入口，管理车道变更与普通车道的规划优先级
- **输入**: `planning_start_point` 起始轨迹点、`frame` 当前规划帧
- **输出**: `Status` 规划状态
- **关键步骤**:
  1. 优先查找换道参考线（`IsChangeLanePath`），先对换道路径调用 `PlanOnReferenceLine`
  2. 判断换道规划是否成功（可驾驶 + 换道 + 轨迹长度大于 `FLAGS_change_lane_min_length`）
  3. 若换道规划失败或未启用换道优先（`FLAGS_prioritize_change_lane`），遍历所有非换道参考线逐条规划

### PlanOnReferenceLine

```cpp
Status RTKReplayPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame*,
    ReferenceLineInfo* reference_line_info) {
  // 校验轨迹有效性
  if (complete_rtk_trajectory_.empty() || complete_rtk_trajectory_.size() < 2) {
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 1. 查找最近匹配点
  std::uint32_t matched_index =
      QueryPositionMatchedPoint(planning_init_point, complete_rtk_trajectory_);
  // 2. 截取子轨迹段 [matched_index, matched_index + forward_buffer)
  std::uint32_t end_index = std::min(
      complete_rtk_trajectory_.size(), matched_index + forward_buffer);
  std::vector<TrajectoryPoint> trajectory_points(
      complete_rtk_trajectory_.begin() + matched_index,
      complete_rtk_trajectory_.begin() + end_index);
  // 3. 归零相对时间
  double zero_time = complete_rtk_trajectory_[matched_index].relative_time();
  for (auto& pt : trajectory_points) {
    pt.set_relative_time(pt.relative_time() - zero_time);
  }
  // 4. 点数不足时复制末尾点填充
  while (trajectory_points.size() < FLAGS_rtk_trajectory_forward) { ... }
  reference_line_info->SetTrajectory(DiscretizedTrajectory(trajectory_points));
  return Status::OK();
}
```

- **职责**: 在单条参考线上执行轨迹回放规划
- **输入**: `planning_init_point` 当前车辆位置、`reference_line_info` 目标参考线
- **输出**: `Status` 规划状态，轨迹写入 `reference_line_info`
- **关键步骤**:
  1. 校验 `complete_rtk_trajectory_` 非空且至少有 2 个点
  2. 调用 `QueryPositionMatchedPoint` 找到离当前车辆最近的轨迹点索引
  3. 截取从 `matched_index` 到 `matched_index + forward_buffer` 的子轨迹段
  4. 将相对时间归零（以匹配点时间为基准）
  5. 若点数不足，复制最后一个点并递增时间戳（步长 `FLAGS_rtk_trajectory_resolution`）填充

### ReadTrajectoryFile

```cpp
void RTKReplayPlanner::ReadTrajectoryFile(const std::string& filename) {
  std::ifstream file_in(filename.c_str());
  std::string line;
  getline(file_in, line);  // 跳过表头
  while (true) {
    getline(file_in, line);
    if (line == "") { break; }
    const std::vector<std::string> tokens =
        absl::StrSplit(line, absl::ByAnyChar("\t "));
    // 解析 11 列数据到 TrajectoryPoint
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(std::stod(tokens[0]));
    point.mutable_path_point()->set_y(std::stod(tokens[1]));
    point.mutable_path_point()->set_z(std::stod(tokens[2]));
    point.set_v(std::stod(tokens[3]));
    point.set_a(std::stod(tokens[4]));
    point.mutable_path_point()->set_kappa(std::stod(tokens[5]));
    point.mutable_path_point()->set_dkappa(std::stod(tokens[6]));
    point.set_relative_time(std::stod(tokens[7]));
    point.mutable_path_point()->set_theta(std::stod(tokens[8]));
    point.mutable_path_point()->set_s(std::stod(tokens[10]));
    complete_rtk_trajectory_.push_back(std::move(point));
  }
}
```

- **职责**: 解析 RTK 轨迹文件，加载到内存
- **输入**: `filename` 轨迹文件路径（TSV/空格分隔）
- **输出**: 无（结果写入 `complete_rtk_trajectory_`）
- **关键步骤**:
  1. 清空已有轨迹数据，跳过首行表头
  2. 逐行按 Tab 或空格分割，要求至少 11 列
  3. 解析各字段：x/y/z（位置）、v（速度）、a（加速度）、kappa/dkappa（曲率及其变化率）、relative_time（相对时间）、theta（航向角）、s（弧长）

轨迹文件格式（按列索引）：

| 索引 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|------|---|---|---|---|---|---|---|---|---|---|---|
| 字段 | x | y | z | v | a | kappa | dkappa | time | theta | (未使用) | s |

### QueryPositionMatchedPoint

- **职责**: 在预录轨迹中线性搜索离当前车辆位置最近的点（欧氏距离平方最小）
- **输入**: `start_point` 当前车辆位置、`trajectory` 完整轨迹
- **输出**: 最近点的索引 `uint32_t`
- **关键步骤**: 遍历所有轨迹点，使用 `(dx*dx + dy*dy)` 仅基于 x/y 坐标计算距离，记录最小值索引

## 调用关系

```text
Cyber RT Plugin Manager
  └── RTKReplayPlanner::Init()
        ├── Planner::Init()
        └── ReadTrajectoryFile(FLAGS_rtk_trajectory_filename)

Planning Pipeline
  └── RTKReplayPlanner::Plan()
        ├── [优先] 查找换道参考线 → PlanOnReferenceLine()
        │     ├── QueryPositionMatchedPoint()
        │     ├── 截取子轨迹段 + 归零时间 + 填充末尾点
        │     └── SetTrajectory()
        └── [回退] 遍历非换道参考线 → PlanOnReferenceLine()
              └── (同上)

PlannerWithReferenceLine  ← 父类
  └── RTKReplayPlanner
        ├── complete_rtk_trajectory_  ← 存储完整预录轨迹
        └── QueryPositionMatchedPoint()  ← 内部最近点查找
```
