# 开放空间规划与泊车数据中心（Open Space Planning & Park Data Center）

> 本文档面向已熟悉 **Hybrid A\*** 搜索、**Reed-Shepp** 曲线、**OBCA / DL-IAPS** 等泊车与非结构化规划算法的读者，逐函数剖析 `modules/planning/planning_open_space/` 与 `modules/planning/park_data_center/` 两个模块的代码组织、关键类接口与实现细节。
>
> 文中所有源码引用均使用 `modules/planning/planning_open_space/子路径/文件名.cc:行号` 形式，行号对应 `apollo-edu` 当前代码基线。

[[toc]]

## 1. 模块定位与调用关系

### 1.1 与 lane-based 规划的区别

Apollo 对**结构化道路**（有车道、有参考线、可 Frenet 投影）使用以下栈：

- `planners/public_road_planner` 选择 `Scenario → Stage → Task`；
- 路径生成在 Frenet `(s, l)` 下进行，代表任务为 `lane_follow_path`、`piecewise_jerk_path_optimizer`；
- 速度规划使用 ST 图 / DP + QP，代表任务为 `piecewise_jerk_speed_optimizer`。

而**开放空间**（停车场、掉头、狭窄路面、大曲率弯道）不存在连续车道，也难以定义一条稳定的参考线：车辆需要**前进+倒车切换**、**大曲率转向**、**任意航向**才能到达目标位姿，此时把路径和速度一起建模成 `SE(2)` 上的非线性最优控制问题（Optimal Trajectory Problem on `SE(2)`）更合适。

典型调用场景：

| 场景 | Scenario 名称 | 典型任务流水线 |
| --- | --- | --- |
| 代客泊车（泊入 / 泊出） | `valet_parking_park` | `OpenSpaceRoiDeciderPark` → `OpenSpaceTrajectoryOptimizerPark` → `OpenSpaceFallbackDecider` |
| 紧急掉门、U-turn | `emergency_pull_over`（历史版本）、`pull_over_park` | Hybrid A\* + OBCA |
| 大曲率弯道 | `large_curvature` | `OpenSpaceTrajectoryOptimizerPark` |
| 非结构化道路（园区、乡道） | 定制 Scenario | 同上 |

`planning_open_space/` 提供算法库（搜索 + 平滑），本身**不是**一个 Task；上层 Task（`open_space_trajectory_provider`、`open_space_trajectory_optimizer_park`、`open_space_roi_decider*`）负责组装 ROI、调用库、回写 `Frame::open_space_info()`。

### 1.2 两阶段求解器架构

```
┌────────────────────────┐   warm_start (xWS, uWS)   ┌─────────────────────────┐
│  Coarse Trajectory Gen │ ────────────────────────▶ │   Trajectory Smoother    │
│   (Hybrid A* + RS)     │                           │ OBCA / IA / DL-IAPS 等  │
└────────────────────────┘                           └─────────────────────────┘
         ▲                                                     │
         │                                                     ▼
  ROI / XYbounds / 障碍物                         smoothed DiscretizedTrajectory
```

- **粗解**（Warm Start）：`coarse_trajectory_generator/hybrid_a_star.*` 用 Hybrid A\* 在 `(x, y, φ)` 空间搜索一条运动学可行、无碰撞的离散路径，再用 `GenerateSpeedAcceleration` / `GenerateSCurveSpeedAcceleration` 反推速度、加速度与 steer。
- **精解**（Smoother）：两条并列路线——
  1. **DL-IAPS**：`iterative_anchoring_smoother.*`，路径-速度解耦，FEM 位姿偏差平滑 + Piecewise-Jerk 速度；
  2. **OBCA**（含多变体）：`distance_approach_problem.*` + `distance_approach_ipopt*_interface.*`，把障碍物写成对偶形式 `λ, μ`，送入 IPOPT 联合优化 `(x, u, t, λ, μ)`。
- **对偶变量热启动**：`dual_variable_warm_start_problem.*` 先用 OSQP/IPOPT 解一个小型对偶问题，给 OBCA 的 `λ, μ` 提供初值以加速主解。

### 1.3 `park_data_center` 的角色

`park_data_center/` 与 `planning_open_space/` **不是**同一个层级。它是一个**按参考线 key 索引的全局单例**，在泊车/近场场景的 `lane_borrow`、`nudge`、`lane_change` 等任务间共享"下一帧需要保留的绕行决策（NudgeInfo）"。在开放空间栈里，它并不直接参与 Hybrid A\* / OBCA 求解，而是辅助**前置的路径 / 速度决策**（如绕行避障、借道）。源码里真正的"目标车位、parking_spot、停车状态机"是放在 `planning_base/common/open_space_info.h` 的 `OpenSpaceInfo` 以及 `scenarios/valet_parking_park/` 里的 StageContext，`park_data_center` 仅承担 NudgeInfo 缓存职责。

## 2. `planning_open_space/` 目录结构

```
planning_open_space/
├── coarse_trajectory_generator/   # Hybrid A* + Reed-Shepp + 2D 启发
├── trajectory_smoother/           # OBCA 的 7 个 IPOPT 变体 + 4 个对偶热启 + 迭代锚定
├── utils/                         # ROI 构建、优化器公用几何变换
├── tools/                         # 命令行 wrapper（Python 对接 / 调试）
└── proto/                         # PlannerOpenSpaceConfig proto
```

顶层 `BUILD` 把 `coarse_trajectory_generator` + `trajectory_smoother` + `utils` 编成 `//modules/planning/planning_open_space:apollo_planning_open_space` 静态库；上层 Task 通过依赖这个库拿到 `HybridAStar`、`DistanceApproachProblem` 等类。

## 3. `coarse_trajectory_generator/` — 粗解生成

本目录提供 Hybrid A\* 的全部组件：节点 `Node3d`、2D 启发栅格 `GridSearch` + `Node2d`、Reed-Shepp 解析扩展 `ReedShepp`，以及主搜索器 `HybridAStar`。

### 3.1 `Node3d` — 三维搜索节点

**文件**：`coarse_trajectory_generator/node3d.h` / `node3d.cc`

`Node3d` 把 `SE(2)` 连续状态 `(x, y, φ)` 同时保存**连续值**和**离散栅格索引**，后者用于 hash 去重与 close-set 判定。

#### 3.1.1 公有接口

```cpp
class Node3d {
public:
    Node3d(double x, double y, double phi);                      // 最小构造（仅调试）
    Node3d(double x, double y, double phi,                       // 单点+config 版
           const std::vector<double>& XYbounds,
           const PlannerOpenSpaceConfig& open_space_conf);
    Node3d(const std::vector<double>& traversed_x,               // 轨迹段版
           const std::vector<double>& traversed_y,
           const std::vector<double>& traversed_phi,
           const std::vector<double>& XYbounds,
           const PlannerOpenSpaceConfig& open_space_conf);
    // park generic —— 只接 WarmStartConfig（用于独立 warm-start 场景）
    Node3d(double x, double y, double phi,
           const std::vector<double>& XYbounds,
           const WarmStartConfig& warm_start_conf);
    Node3d(const std::vector<double>& traversed_x,
           const std::vector<double>& traversed_y,
           const std::vector<double>& traversed_phi,
           const std::vector<double>& XYbounds,
           const WarmStartConfig& warm_start_conf);

    static apollo::common::math::Box2d GetBoundingBox(
        const common::VehicleParam& vehicle_param_,
        double x, double y, double phi);

    double GetCost() const;         // = traj_cost_ + heuristic_cost_
    double GetTrajCost() const;
    double GetHeuCost() const;
    int    GetGridX()   const;
    int    GetGridY()   const;
    double GetX()       const;
    double GetY()       const;
    double GetPhi()     const;
    const std::string& GetIndex()   const;   // "x_grid_y_grid_phi_grid"
    size_t GetStepSize()            const;
    bool   GetDirec()               const;   // true = 前进, false = 倒车
    double GetSteer()               const;
    std::shared_ptr<Node3d> GetPreNode() const;
    const std::vector<double>& GetXs()   const;
    const std::vector<double>& GetYs()   const;
    const std::vector<double>& GetPhis() const;
    const double& GetTravelDist()        const;

    void SetPre(std::shared_ptr<Node3d> pre_node);
    void SetDirec(bool direction);
    void SetTrajCost(double cost);
    void SetHeuCost(double cost);
    void SetSteer(double steering);
    void SetTravelDist(double dist);
    bool operator==(const Node3d& right) const;   // 以 index_ 比对
};
```

#### 3.1.2 关键字段语义

| 字段 | 含义 |
| --- | --- |
| `x_, y_, phi_` | 当前节点的**连续**位姿，等价于 `traversed_x/y/phi_.back()` |
| `traversed_x/y/phi_` | 从父节点展开到本节点所经过的全部离散采样点（长度 = `step_size_`） |
| `x_grid_, y_grid_, phi_grid_` | `(x, y, φ)` 分别除以 `xy_grid_resolution` / `phi_grid_resolution` 得到的整型栅格下标 |
| `index_` | `absl::StrCat(x_grid, "_", y_grid, "_", phi_grid)`，open/close set 的 key |
| `step_size_` | 本节点内部包含多少条采样点，一次 Hybrid A\* 展开通常 `step_size = ceil(arc_length/step_size_)` |
| `traj_cost_` | 从起点到本节点累计的 g-cost |
| `heuristic_cost_` | 本节点到终点的 h-cost（来自 DP map） |
| `direction_` / `steering_` / `travel_distance_` | 本节点沿哪个齿轮（前/倒）、以哪个固定舵角、在当前齿轮下累计走了多远（切齿清零） |
| `pre_node_` | 父节点指针，用于回溯路径 |

#### 3.1.3 离散化

```cpp
// node3d.cc:50
x_grid_   = (x  - XYbounds[0]) / warm_start_conf.xy_grid_resolution();
y_grid_   = (y  - XYbounds[2]) / warm_start_conf.xy_grid_resolution();
phi_grid_ = (φ  - (-π))        / warm_start_conf.phi_grid_resolution();
```

`XYbounds` 顺序固定为 `[xmin, xmax, ymin, ymax]`，断言 `CHECK_EQ(XYbounds.size(), 4U)`。φ 离散以 `-π` 为零点。

#### 3.1.4 `GetBoundingBox`（`node3d.cc:138`）

用 `VehicleParam` 把**后轴中心** `(x, y)` 处的位姿旋转成一个**车身中心**的 `Box2d`：

```cpp
double shift_distance = ego_length/2 - back_edge_to_center;     // 车身中心相对后轴的纵向偏移
Box2d ego_box(
    {x + shift_distance * cos(phi), y + shift_distance * sin(phi)},
    phi, ego_length, ego_width);
```

该 box 被 `HybridAStar::ValidityCheck` 用作碰撞检测的 ADC 包络，也可被 smoother 复用。

### 3.2 `GridSearch` — 2D holonomic-with-obstacles 启发

**文件**：`coarse_trajectory_generator/grid_search.h` / `grid_search.cc`

Hybrid A\* 的启发 `h(n)` 必须是 admissible，最常用就是"不考虑运动学但考虑障碍物的二维最短距离"。Apollo 用经典 8 邻域 A\* + DP 在全 ROI 上预计算一张 `dp_map_`，搜索期间 O(1) 查表。

#### 3.2.1 `Node2d`（`grid_search.h:42`）

- 栅格下标 `(grid_x_, grid_y_)`；
- `path_cost_`（累计 g）、`heuristic_`（到目标）、`cost_ = path_cost + heuristic`；
- `distance_to_obstacle_`：该栅格到最近障碍 / 软边界的距离，用于 ESDF 软代价；
- `pre_node_` 父指针；
- 静态 `CalcIndex(x, y, xy_resolution, XYbounds)` 生成 `"grid_x_grid_y"` 字符串。

#### 3.2.2 `GridSearch` 公有接口

```cpp
class GridSearch {
public:
    explicit GridSearch(const PlannerOpenSpaceConfig& open_space_conf);
    explicit GridSearch(const WarmStartConfig& warm_start_conf);   // park generic

    // 普通 A*：从 (sx,sy) 搜到 (ex,ey)
    bool GenerateAStarPath(
        double sx, double sy, double ex, double ey,
        const std::vector<double>& XYbounds,
        const std::vector<std::vector<common::math::LineSegment2d>>& obstacles_linesegments_vec,
        GridAStartResult* result);

    // DP Map：从终点反向 Dijkstra 覆盖整张图，给 Hybrid A* 查表用
    bool GenerateDpMap(
        double ex, double ey,
        const std::vector<double>& XYbounds,
        const std::vector<std::vector<common::math::LineSegment2d>>& obstacles_linesegments_vec,
        const std::vector<std::vector<common::math::LineSegment2d>>& soft_boundary_linesegments_vec = {{}});

    // 查表
    double CheckDpMap(double sx, double sy);            // 返回启发值，不可达返回 +inf
    double GetObstacleDistance(double x, double y);     // 返回 ESDF 到最近障碍的距离
};
```

#### 3.2.3 关键私有方法

- `EuclidDistance(x1, y1, x2, y2)`（`grid_search.cc:40`）：欧氏距离，用于启发。
- `CheckConstraints(node)`（`grid_search.cc:44`）：检查栅格越界 + 障碍物线段距离是否大于 `node_radius_`。注意这里把 `grid_x, grid_y` 当作"点"传给 `DistanceTo`，所以与 `obstacles_linesegments_vec_` 的线段单位必须一致。
- `GenerateNextNodes(node)`（`grid_search.cc:120`）：8 邻域展开，直边代价 `+1`，对角线 `+√2`。
- `AddSoftCost(soft_boundary)`（`grid_search.cc:66`）：把车位边线（`parking_space_boundary`）按 `node_radius_` 采样，在 `dp_map_` 上用 `ExtendNode` 以 BFS 方式**降低**对应栅格的 `distance_to_obstacle_`，使得 Hybrid A\* 的轨迹代价对"压车位线"施加惩罚。仅在 `use_esdf_` 打开时生效，范围由 `esdf_range_` 限制。
- `ExtendNode(x, y, origin_index, close_set)`（`grid_search.cc:90`）：递归 4 邻域扩散 ESDF，命中 `esdf_range_` 即停。
- `LoadGridAStarResult(result)`（`grid_search.cc:285`）：把 `final_node_` 沿 `pre_node_` 回溯，填充 `result->x/y` 并反转顺序，`path_cost` 乘回 `xy_grid_resolution_` 变成米。

#### 3.2.4 `GenerateDpMap` 实现要点

文件：`grid_search.cc:213` 起。

1. 以**终点** `end_node` 为起点做 Dijkstra（启发恒为 0，退化为 BFS+代价），确保 `dp_map_[k].cost` 等于从 `k` 走到终点的最短栅格距离。
2. `CheckConstraints` 过滤越界与障碍；
3. 弹出即关闭：`dp_map_.emplace(current_node->GetIndex(), current_node)`；
4. 新节点入队或松弛：
   ```cpp
   if (open_set[idx]->GetCost() > next_node->GetCost()) {
       open_set[idx]->SetCost(next_node->GetCost());
       open_set[idx]->SetPreNode(current_node);
   }
   ```
5. 结束后若 `use_esdf_` 打开，调用 `AddSoftCost` 对软边界做 ESDF。

`CheckDpMap` 返回 `dp_map_[idx]->GetCost() * xy_grid_resolution_`，正是 `HybridAStar::HoloObstacleHeuristic` 使用的 h-cost。

### 3.3 `ReedShepp` — 解析扩展

**文件**：`coarse_trajectory_generator/reeds_shepp_path.h` / `reeds_shepp_path.cc`（共 1512 行）

Reed-Shepp（RS）曲线枚举了**最短前进+倒车**路径的 48 种组合（3–5 段 L/R/S 原语），Apollo 归并成 9 类模板 `SCS / CSC / CCC / CCCC / CCSC / CCSCC` 及若干子变体。RS 的作用：

1. 在每个 Hybrid A\* 迭代中，尝试从当前节点用 RS 曲线直达终点（`AnalyticExpansion`），一旦 RS 路径无碰撞即**闭式接管**剩余路径，极大加速搜索；
2. 选择最小代价 RS 作为候选完整解，代价函数与 Hybrid A\* 的 g-cost 对齐（前进/倒车/切齿/转向/短段惩罚）。

#### 3.3.1 核心数据结构

```cpp
struct ReedSheppPath {
    std::vector<double> segs_lengths;   // 每段的带符号长度（负号=倒车）
    std::vector<char>   segs_types;     // 每段类型，'L'/'R'/'S'
    double total_length = 0.0;
    double cost = 0.0;
    std::vector<double> x, y, phi;      // 插值后的离散轨迹
    std::vector<bool>   gear;           // 每个插值点的档位（true=前进）
    double radius = 1.0;                // 曲线半径缩放系数
};

struct RSPParam {                        // 解析公式的中间结果
    bool flag = false;                   // 是否存在解
    double t, u, v;                      // 三段参数
};
```

#### 3.3.2 公有接口

```cpp
class ReedShepp {
public:
    ReedShepp(const common::VehicleParam& vehicle_param,
              const PlannerOpenSpaceConfig& open_space_conf);
    ReedShepp(const common::VehicleParam& vehicle_param,
              const WarmStartConfig& warm_start_conf);          // park generic

    // 从所有模板里挑选代价最小者，写回 optimal_path
    bool ShortestRSP(
        const std::shared_ptr<Node3d> start_node,
        const std::shared_ptr<Node3d> end_node,
        std::shared_ptr<ReedSheppPath> optimal_path);

    // 外部可调的两个开关
    bool   reeds_sheep_last_straight_ = false;   // 终点前强制加一段直线，便于垂直泊入
    bool   is_vertical_               = false;
    double max_kappa_;                           // 当前允许的最大曲率
};
```

#### 3.3.3 代价函数（`reeds_shepp_path.cc:80` 起）

对 `all_possible_paths` 逐条：

```
for each segment j:
    if 弧段 (L/R):
        cost += |len_j|       * traj_steer_penalty_ / max_kappa_ * radius
        if 连续弧段切向（L↔R）:
            cost += atan(wheel_base/steering_radius*2) * traj_steer_change_penalty_
        if |len_j|/max_kappa_*radius < traj_expected_shortest_length_:
            cost += traj_short_length_penalty_
    else (S):
        cost += |len_j|       * (forward ? traj_forward_penalty_ : traj_back_penalty_) / max_kappa_
        if |len_j|/max_kappa_ < traj_expected_shortest_length_:
            cost += traj_short_length_penalty_
    if 相邻段方向反号:
        cost += traj_gear_switch_penalty_
```

初始段档位与当前节点档位不一致时还要额外加一次 `traj_gear_switch_penalty_`（`reeds_shepp_path.cc:127`）。

#### 3.3.4 模板函数一览

以下全部声明在 `reeds_shepp_path.h`，实现在 `reeds_shepp_path.cc`。每个 `CxC`-类把 `(x, y, φ)` 目标位姿代入对应公式，若返回 `flag=true` 则用 `SetRSP` 把三段拼成一条候选 RS 曲线。

| 成员函数 | 功能 |
| --- | --- |
| `GenerateRSPs`（`:183`） | 串行调度所有模板 |
| `GenerateRSPPar`（`:1124`） | OpenMP 并行版 |
| `GenerateRSP`（`:202`） | 依次调用 `SCS / CSC / CCC / CCCC / CCSC / CCSCC` 并转换对称 |
| `SCS`、`CS`、`CSC`、`CCC`、`CCCC`、`CCSC`、`CCSCC` | 7 大模板，各自生成 2–8 条候选 |
| `LSL / LSR / LRL / SLS / LS / LRLRn / LRLRp / LRSR / LRSL / LRSLR` | 9 个几何闭式 RSPParam 解 |
| `calc_tau_omega(u,v,xi,eta,φ)`（`:58`） | 多次在 `CCSC`/`LRSR`/`LRSLR` 中用到的 τ, ω 求解子例程 |
| `SetRSP / SetRSPPar` | 把 `(lengths, types)` 写入 `ReedSheppPath`，更新 `total_length`、`radius` |
| `GenerateLocalConfigurations`（`:977`） | 按 `step_size` 对选中 RS 逐段离散化，写入 `x, y, phi, gear` |
| `Interpolation`（`:1053`） | 单段 L/R/S 的闭式插值 |

`GenerateLocalConfigurations` 完成后会校验终点误差 `|x-x_end| + |y-y_end| < 1e-3`，否则 `ShortestRSP` 返回失败。

### 3.4 `HybridAStar` — 主搜索器

**文件**：`coarse_trajectory_generator/hybrid_a_star.h` / `hybrid_a_star.cc`（共 903 行）

#### 3.4.1 结果结构

```cpp
struct HybridAStartResult {
    std::vector<double> x, y, phi;       // 路径（SE(2)）
    std::vector<double> v, a, steer;     // 速度/加速度/舵角
    std::vector<double> accumulated_s;   // 累计弧长
};
```

#### 3.4.2 公有接口

```cpp
class HybridAStar {
public:
    explicit HybridAStar(const PlannerOpenSpaceConfig& open_space_conf);
    explicit HybridAStar(const WarmStartConfig& warm_start_conf);   // park generic
    virtual ~HybridAStar() = default;

    // 主入口（park generic 版）
    bool Plan(
        double sx, double sy, double sphi,
        double ex, double ey, double ephi,
        const std::vector<double>& XYbounds,
        const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
        HybridAStartResult* result,
        const std::vector<std::vector<common::math::Vec2d>>& soft_boundary_vertices_vec = {},
        bool reeds_sheep_last_straight = false,
        bool is_vertical = false,
        double kappa_ratio = 1.0);

    // 解出的 result 可以按齿轮切换位置拆成多段子轨迹
    bool TrajectoryPartition(
        const HybridAStartResult& result,
        std::vector<HybridAStartResult>* partitioned_result);
};
```

#### 3.4.3 构造函数里的关键超参

`hybrid_a_star.cc:72` 起是 `WarmStartConfig` 版构造函数，会把配置抽到成员变量。核心参数：

| 成员 | 来源 | 含义 |
| --- | --- | --- |
| `next_node_num_` | `next_node_num` | 每一步展开多少个候选节点（前进段 + 倒车段，通常为 10） |
| `max_steer_angle_` | `vehicle_param.max_steer_angle() / steer_ratio * traj_kappa_contraint_ratio()` | 搜索时使用的最大前轮转角；`traj_kappa_contraint_ratio` 默认 0.7，给后续平滑留裕度 |
| `max_kappa_` | `tan(max_steer_angle_)/wheel_base` | 最大曲率 |
| `step_size_` | `step_size` | 同一段内的等间距采样步长 |
| `xy_grid_resolution_` | `xy_grid_resolution` | `Node3d` 的 xy 栅格分辨率 |
| `arc_length_` | `phi_grid_resolution * wheel_base / tan(max_steer*2/(n/2-1))`，下界 `√2 * xy_grid_resolution` | 单次展开的弧长，保证能跨出栅格 |
| `traj_*_penalty_` | warm_start_config | 代价函数权重（前进/倒车/切齿/转向/转向变化/短段） |
| `soft_boundary_penalty_` | `soft_boundary_penalty` | 压软边界的惩罚权重，除以 ESDF 距离 |
| `traj_expected_shortest_length_` / `traj_short_length_penalty_` | 同名配置 | 约束最短子段长度，防止平滑器无法处理过短段 |

`PlannerOpenSpaceConfig` 版构造函数（`hybrid_a_star.cc:38`）几乎相同，但使用 `warm_start_config()` 子消息，且从 `iterative_anchoring_smoother_config` 里读取速度/加速度/jerk 相关字段（用于 `GenerateSCurveSpeedAcceleration`）。

#### 3.4.4 `Plan()`（`hybrid_a_star.cc:697`）— 主搜索循环

步骤（按源码顺序）：

1. **重置**：根据 `kappa_ratio` 重新设置 `max_steer_angle_ / max_kappa_`；清空 `open_set_/close_set_/open_pq_/final_node_`；刷新 `ReedShepp` 的 `reeds_sheep_last_straight_ / is_vertical_ / max_kappa_`。
2. **障碍物格式转换**：把 `obstacles_vertices_vec`、`soft_boundary_vertices_vec` 从顶点列转为线段列，分别存到 `obstacles_linesegments_vec_` 与 `soft_boundary_linesegments_vec_`。
3. **构造起终点节点**：`start_node_.reset(new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_warm_start_config_))`，终点同理；对两者都做 `ValidityCheck`，冲突即整体失败。
4. **启发生成**：`grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_, obstacles_linesegments_vec_, soft_boundary_linesegments_vec_)`，耗时最大。
5. **主循环**（`hybrid_a_star.cc:807`）：
   ```
   while (open_pq_ 非空) && (size < kMaxNodeNum=200000)
          && (available_result_num < desired_explored_num)
          && (explored_node_num < max_explored_num):
       current = open_pq_.pop();
       if AnalyticExpansion(current, &final) succeeds:
           keep the min-cost final node;
           available_result_num++;
       explored_node_num++;
       close_set_.insert(current->index);
       if time 超过 astar_max_search_time 且已有候选: break;
       for i in [0, next_node_num_):
           next = Next_node_generator(current, i);
           if next==null || next ∈ close_set: continue;
           if !ValidityCheck(next): continue;
           if next ∉ open_set:
               CalculateNodeCost(current, next);
               open_pq_.emplace(next, next->GetCost());
   ```
6. **回溯**：`GetResult(result)` 从 `final_node_` 向 `pre_node_` 回溯，得到 `x/y/phi` 序列。
7. **时空化**：`GetTemporalProfile(result)` 先按齿轮 `TrajectoryPartition` 拆段，再对每段调用 `GenerateSCurveSpeedAcceleration`（若 `FLAGS_use_s_curve_speed_smooth`）或 `GenerateSpeedAcceleration`，最后拼成一条带 `v/a/steer` 的轨迹。

#### 3.4.5 `Next_node_generator(current, i)`（`hybrid_a_star.cc:224`）— 运动学展开

- `i ∈ [0, next_node_num_/2)`：前进段，`steering = -max_steer + (2*max_steer/(N/2-1))*i`，`traveled_distance = +step_size_`；
- `i ∈ [N/2, N)`：倒车段，同 `steering` 枚举但 `traveled_distance = -step_size_`。

积分自行车模型：

```cpp
for (size_t k = 0; k < arc_length_ / step_size_; ++k) {
    next_phi = last_phi + traveled_distance / wheel_base * tan(steering);
    next_x   = last_x + traveled_distance * cos((last_phi + next_phi) / 2.0);
    next_y   = last_y + traveled_distance * sin((last_phi + next_phi) / 2.0);
    ...
}
```

这里使用的是**中点法**（`(last_phi + next_phi)/2`），比单纯欧拉法偏差更小。超出 `XYbounds_` 的直接返回 `nullptr`。末尾为 `next_node` 设置：

- `SetPre(current_node)`；
- `SetDirec(traveled_distance > 0)`；
- `SetSteer(steering)`；
- `SetTravelDist`：若与父节点同齿轮，累加 `arc_length * dir`；否则重置成当次长度（切齿清零）。

#### 3.4.6 `CalculateNodeCost / TrajCost / HoloObstacleHeuristic`

- `CalculateNodeCost`（`:277`）：`next->traj_cost = current->traj_cost + TrajCost(current, next)`；`next->heuristic_cost = HoloObstacleHeuristic(next)`。
- `TrajCost`（`:285`）：
  - 按齿轮加 `traj_forward_penalty_ * len` 或 `traj_back_penalty_ * len`；
  - 齿轮切换加 `traj_gear_switch_penalty_`；如果上一段 `TravelDist` 短于 `traj_expected_shortest_length_`，再加 `traj_short_length_penalty_`；
  - `traj_steer_penalty_ * |steer|`；
  - `traj_steer_change_penalty_ * |steer - current.steer|`；
  - 软边界：`soft_boundary_penalty_ / (GetObstacleDistance(x, y) + 1e-6)`。
- `HoloObstacleHeuristic`（`:322`）：直接返回 `grid_a_star_heuristic_generator_->CheckDpMap(next->x, next->y)`。

#### 3.4.7 `ValidityCheck / RSPCheck / RSPLengthCheck`

- `ValidityCheck(node)`（`:151`）：遍历 `node->traversed_x/y/phi_`（第 0 个跳过，除非是 start/end），检查是否越 `XYbounds_`；把每个采样位姿转成 `Box2d`（`Node3d::GetBoundingBox`），与每条障碍物线段做 `HasOverlap` 测试。
- `RSPCheck(reeds_shepp_to_end)`（`:124`）：把整条 RS 曲线包装成一个 `Node3d`（以 `traversed_x/y/phi` 方式），复用 `ValidityCheck` + `RSPLengthCheck`。
- `RSPLengthCheck`（`:134`）：对每个子段，弧段长度换算成世界距离 `|len|/max_kappa * radius`（'S' 段是 `|len|/max_kappa`），若小于 `0.1m` 则拒绝——避免 smoother 处理微段。

#### 3.4.8 `AnalyticExpansion / LoadRSPinCS`

- `AnalyticExpansion(current, &candidate)`（`:109`）：`ReedShepp::ShortestRSP(current, end_node_, rs)`；`RSPCheck(rs)` 通过后调用 `LoadRSPinCS`。
- `LoadRSPinCS(rs, current)`（`:194`）：把整条 RS 曲线打包成一个"终点节点"，其 `traversed_*` 就是 RS 的 `x/y/phi`，`traj_cost = current->traj_cost + rs->cost`，`pre_node = current`。

#### 3.4.9 `GetResult` / `GetTemporalProfile` / `TrajectoryPartition`

- `GetResult(result)`（`:326`）：沿 `pre_node_` 逐节点反推，反转每段，最后整体反转得到正序 `x/y/phi`，再调 `GetTemporalProfile`。
- `GetTemporalProfile(result)`（`:673`）：调 `TrajectoryPartition` 拆段，每段独立生成速度/加速度，然后重新拼接为 `stitched_result`（段间去掉最后一个点避免重复）。
- `TrajectoryPartition(result, partitioned_result)`（`:607`）：按车头朝向与移动方向夹角是否 `> π/2` 判断档位翻转；档位变化即起新段。
- `GenerateSpeedAcceleration(result)`（`:389`）：从离散 `(x, y)` 数值微分反推 `v`，两端置零；再反推 `a` 和 `steer = atan((φ_{i+1}-φ_i)*wheel_base / step_size)`，倒车段符号反转。
- `GenerateSCurveSpeedAcceleration(result)`（`:431`）：基于 `PiecewiseJerkSpeedProblem` 做最小时间 S-curve 速度规划，用 `acc_weight_ / jerk_weight_ / kappa_penalty_weight_ / ref_s_weight_ / ref_v_weight_` 作为目标权重。

#### 3.4.10 `GetSoftBoundaryCost`（`:208`）

对 RS 曲线的每个采样点查询 `GetObstacleDistance`，累加 `soft_boundary_penalty_ / (dist + 1e-6)` 作为总软代价，目前仅打日志，未并入 g-cost。

## 4. `trajectory_smoother/` — 精解优化

此目录提供**两条互补的平滑链路**：迭代锚定（DL-IAPS 风格）与距离逼近（OBCA 的多个变体），以及把障碍物对偶变量 `λ, μ` 预热的 `dual_variable_warm_start_*` 系列。

```
trajectory_smoother/
├── iterative_anchoring_smoother.{h,cc}              # DL-IAPS (FEM + PJS)
├── distance_approach_problem.{h,cc}                 # OBCA 入口封装
├── distance_approach_interface.h                    # 纯虚基类（IPOPT::TNLP）
├── distance_approach_ipopt_interface.{h,cc}         # 变体 0: 基础 OBCA
├── distance_approach_ipopt_cuda_interface.{h,cc}    # 变体 1: CUDA
├── distance_approach_ipopt_fixed_ts_interface.{h,cc}        # 变体 2: 固定采样时间
├── distance_approach_ipopt_fixed_dual_interface.{h,cc}      # 变体 3: 固定对偶变量
├── distance_approach_ipopt_relax_end_interface.{h,cc}       # 变体 4: 放松终态
├── distance_approach_ipopt_relax_end_slack_interface.{h,cc} # 变体 5: 放松终态+松弛
├── dual_variable_warm_start_problem.{h,cc}          # 对偶热启入口
├── dual_variable_warm_start_ipopt_interface.{h,cc}          # Mode IPOPT
├── dual_variable_warm_start_ipopt_qp_interface.{h,cc}       # Mode IPOPTQP
├── dual_variable_warm_start_osqp_interface.{h,cc}           # Mode OSQP（默认）
├── dual_variable_warm_start_slack_osqp_interface.{h,cc}     # Mode SLACKQP
└── planning_block.h
```

### 4.1 OBCA 问题形式（回顾）

给定离散状态 $x_k = (x, y, \varphi, v)$、控制 $u_k = (\delta, a)$、时间缩放 $t$，OBCA 要解：

$$
\min_{x, u, t, \lambda, \mu}\ \sum_k \ell(x_k, u_k, \Delta u_k) + w_t \sum_k t_k^2
$$

s.t. 运动学：
$$
x_{k+1} = f(x_k, u_k, t_k \cdot t_s)
$$
端点：$x_0, x_F$ 固定（relax_end 变体中放松为不等式）；
控制物理约束 $|\delta|, |a| \le \bar{\cdot}$、速率约束 $|\dot\delta|, |\dot a| \le \bar{\cdot}$；
**OBCA 对偶形式避障**：对每个障碍 $m$，其 H-表示 $A_m x \ge b_m$（$A_m \in \mathbb{R}^{n_m \times 2}$），引入 $\lambda_{m,k} \ge 0$、$\mu_{m,k} \ge 0$，满足：

$$
\begin{aligned}
-g^\top \mu_{m,k} + (A_m T(x_k) - b_m)^\top \lambda_{m,k} &\ge d_{\min} \\
\lVert G^\top \mu_{m,k} + R(\varphi_k)^\top A_m^\top \lambda_{m,k} \rVert_2 &\le 1 \\
\lambda \ge 0,\ \mu \ge 0
\end{aligned}
$$

其中 $G, g$ 是车辆多边形（Apollo 这里是矩形）的 H-表示，`R(φ)` 是旋转矩阵，`T(x) = (x,y)`。$d_{\min}$ 即 `min_safety_distance_`。

### 4.2 `DistanceApproachProblem`（入口封装）

**文件**：`trajectory_smoother/distance_approach_problem.h` / `.cc`

```cpp
class DistanceApproachProblem {
public:
    explicit DistanceApproachProblem(const PlannerOpenSpaceConfig& planner_open_space_config);
    explicit DistanceApproachProblem(const DistanceApproachConfig& distance_approach_config);  // park generic

    bool Solve(
        const Eigen::MatrixXd& x0,                 // 4x1 初始状态
        const Eigen::MatrixXd& xF,                 // 4x1 终态
        const Eigen::MatrixXd& last_time_u,        // 2x1 上一帧控制（平滑过渡）
        size_t horizon, double ts,                 // 时域 N 与名义时间步 Δt
        const Eigen::MatrixXd& ego,                // 4x1 车辆尺寸(l, w, shift_x, shift_y)
        const Eigen::MatrixXd& xWS,                // 4x(N+1) warm-start 状态
        const Eigen::MatrixXd& uWS,                // 2xN warm-start 控制
        const Eigen::MatrixXd& l_warm_up,          // λ 热启
        const Eigen::MatrixXd& n_warm_up,          // μ 热启
        const Eigen::MatrixXd& s_warm_up,          // slack 热启（RELAX_END_SLACK 专用）
        const std::vector<double>& XYbounds,
        size_t obstacles_num,
        const Eigen::MatrixXi& obstacles_edges_num,
        const Eigen::MatrixXd& obstacles_A,
        const Eigen::MatrixXd& obstacles_b,
        Eigen::MatrixXd* state_result,
        Eigen::MatrixXd* control_result,
        Eigen::MatrixXd* time_result,
        Eigen::MatrixXd* dual_l_result,
        Eigen::MatrixXd* dual_n_result);
};
```

`Solve` 内部根据 `distance_approach_config_.distance_approach_mode()` 实例化六种 `DistanceApproachInterface` 之一（`distance_approach_problem.cc:67` 起的 if-else 链）：

| Mode (enum `DistanceApproachMode`) | 对应 Interface | 特点 |
| --- | --- | --- |
| `DISTANCE_APPROACH_IPOPT` = 0 | `DistanceApproachIPOPTInterface` | 基础 OBCA，时间项可变，ADOL-C 自动微分 |
| `DISTANCE_APPROACH_IPOPT_CUDA` = 1 | `DistanceApproachIPOPTCUDAInterface` | CUDA 加速雅可比 |
| `DISTANCE_APPROACH_IPOPT_FIXED_TS` = 2 | `DistanceApproachIPOPTFixedTsInterface` | 固定 Δt，去掉 t 决策变量 |
| `DISTANCE_APPROACH_IPOPT_FIXED_DUAL` = 3 | `DistanceApproachIPOPTFixedDualInterface` | 冻结 λ/μ 仅优化 (x, u, t) |
| `DISTANCE_APPROACH_IPOPT_RELAX_END` = 4 | `DistanceApproachIPOPTRelaxEndInterface` | 终态放松成不等式，防止不可行 |
| `DISTANCE_APPROACH_IPOPT_RELAX_END_SLACK` = 5 | `DistanceApproachIPOPTRelaxEndSlackInterface` | RELAX_END + slack 变量，进一步兜底 |

组装完 `Ipopt::SmartPtr<TNLP>` 后：

- `app = IpoptApplicationFactory()`；按 `ipopt_config` 设置 `print_level / mumps_mem_percent / mumps_pivtol / max_iter / tol / acceptable_constr_viol_tol / min_hessian_perturbation / jacobian_regularization_value / print_timing_statistics / alpha_for_y / recalc_y / mu_init`；
- `app->Initialize() → app->OptimizeTNLP(problem)`；
- 解算成功或达到 acceptable 后通过 `ptop->get_optimization_results(...)` 回填 `state/control/time/dual_l/dual_n`；
- 失败则通过 `failure_status` 查表 (`Infeasible_Problem_Detected / Maximum_Iterations_Exceeded / ...`) 打印 `Solver failure case: ...`。

### 4.3 `DistanceApproachInterface` 基类

**文件**：`trajectory_smoother/distance_approach_interface.h`

继承 `Ipopt::TNLP`（Twice-differentiable NLP），纯虚方法对应 IPOPT 的回调协议：

| 虚函数 | 作用 |
| --- | --- |
| `get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style)` | 返回决策变量数、约束数、Jacobian/Hessian 非零数 |
| `get_bounds_info(n, x_l, x_u, m, g_l, g_u)` | 决策变量与约束上下界 |
| `get_starting_point(n, init_x, x, init_z, z_L, z_U, m, init_lambda, lambda)` | 初值（`xWS, uWS, l_warm_up, n_warm_up` 拼接） |
| `eval_f(n, x, new_x, obj_value)` | 目标值 |
| `eval_grad_f(n, x, new_x, grad_f)` | 目标梯度（ADOL-C 生成 tape 后回放） |
| `eval_g(n, x, new_x, m, g)` | 约束残差 |
| `check_g(n, x, m, g)` | debug：逐条检查约束可行性 |
| `eval_jac_g / eval_jac_g_ser` | 约束 Jacobian（稀疏结构 + 数值），后者串行回退 |
| `eval_h` | Lagrangian Hessian |
| `finalize_solution(status, ...)` | 解算结束时保存 `x, z_L, z_U, lambda, g` |
| `get_optimization_results(state, control, time, dual_l, dual_n)` | 把保存的解切片成 Eigen 矩阵 |

`tag_f / tag_g / tag_L` 为 ADOL-C 的三条 tape ID。

### 4.4 `DistanceApproachIPOPTInterface`（主实现）

**文件**：`trajectory_smoother/distance_approach_ipopt_interface.h` / `.cc`（C++ 近 2000 行）

#### 4.4.1 决策变量布局

`num_of_variables_ = n_state + n_control + n_time + n_lambda + n_miu`，各段起始下标保存在 `state_start_index_ / control_start_index_ / time_start_index_ / l_start_index_ / n_start_index_`。

- 状态段：`4 * (horizon+1)`（x, y, φ, v）；
- 控制段：`2 * horizon`（δ, a）；
- 时间段：`horizon`（每段的时间缩放 `t_k ∈ [min_time_sample_scaling_, max_time_sample_scaling_]`）；
- λ 段：`obstacles_edges_sum_ * (horizon+1)`；
- μ 段：`4 * obstacles_num_ * (horizon+1)`（车辆多边形 4 条边）。

#### 4.4.2 构造函数参数

```cpp
DistanceApproachIPOPTInterface(
    size_t horizon, double ts,
    const Eigen::MatrixXd& ego,                 // {length, width, shift_x, shift_y}
    const Eigen::MatrixXd& xWS,                 // 4x(N+1) 热启状态
    const Eigen::MatrixXd& uWS,                 // 2xN 热启控制
    const Eigen::MatrixXd& l_warm_up,           // λ_warm
    const Eigen::MatrixXd& n_warm_up,           // μ_warm
    const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xf,
    const Eigen::MatrixXd& last_time_u,
    const std::vector<double>& XYbounds,
    const Eigen::MatrixXi& obstacles_edges_num,
    size_t obstacles_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const PlannerOpenSpaceConfig& planner_open_space_config);
// park generic: 最后一个参数换成 DistanceApproachConfig
```

构造体从 `planner_open_space_config_.distance_approach_config()` 读取十余个权重/约束：`weight_state_x_y_phi_v_`、`weight_input_steer_ / weight_input_a_`、`weight_rate_steer_ / weight_rate_a_`、`weight_stitching_*`、`weight_first_order_time_ / weight_second_order_time_`、`min_safety_distance_ / max_safety_distance_`、`max_steer_angle_ / max_speed_forward_ / max_speed_reverse_ / max_acceleration_forward_ / max_acceleration_reverse_`、`min_time_sample_scaling_ / max_time_sample_scaling_`、`max_steer_rate_`、`max_lambda_ / max_miu_`、`enable_jacobian_ad_`。

#### 4.4.3 目标函数（实现在 `eval_obj` 模板，`distance_approach_ipopt_interface.cc`）

概念形式（对所有 `k`）：

```
J = Σ  [ w_x*(x_k - x_ref)^2 + w_y*(...)^2 + w_φ*(...)^2 + w_v*(v_k)^2
       + w_δ*δ_k^2 + w_a*a_k^2
       + w_Δδ*(δ_{k+1}-δ_k)^2 + w_Δa*(a_{k+1}-a_k)^2
       + w_t1*t_k + w_t2*t_k^2 ]
  + w_stitching_δ*(δ_0 - last_u_δ)^2 + w_stitching_a*(a_0 - last_u_a)^2
```

`x_ref` 取 `xWS.col(k)`，使优化不大幅偏离 warm-start。

#### 4.4.4 约束（`eval_constraints` 模板）

1. **运动学离散**（Runge-Kutta 一阶自行车）：`x_{k+1} - x_k - t_k * ts * f(x_k, u_k, u_{k+1}) = 0`；
2. **端点**：`x_0 = x0`，`x_N = xf`；
3. **控制上下界**：`|δ| ≤ max_steer_angle_`，`−max_reverse_a ≤ a ≤ max_forward_a`；
4. **速率约束**：`|δ_{k+1}−δ_k|/(t_k*ts) ≤ max_steer_rate_`；
5. **OBCA 对偶**：对每对 (obstacle m, time k) 施加上一节所述三组不等式+等式（使用障碍物的 `A_m`, `b_m`, `edges_num_m`）；
6. **XY bounds**：`XYbounds[0] ≤ x_k ≤ XYbounds[1]`、`XYbounds[2] ≤ y_k ≤ XYbounds[3]`；
7. **λ, μ 上下界**：`0 ≤ λ ≤ max_lambda_`、`0 ≤ μ ≤ max_miu_`。

#### 4.4.5 ADOL-C 自动微分

- `generate_tapes(n, m, nnz_jac_g, nnz_h_lag)`：一次性记录 `f, g, L` 的 tape；
- `eval_jac_g / eval_h` 通过 `options_g / options_L` 的 `ColPack` 做颜色分组稀疏提取；
- 成员 `obj_lam`, `rind_g/cind_g/jacval`, `rind_L/cind_L/hessval`, `nnz_jac / nnz_L` 存储稀疏结构；
- 若 `enable_jacobian_ad_=false`，改用手写 `eval_jac_g_ser`。

### 4.5 其他五个 OBCA 变体

所有变体继承 `DistanceApproachInterface`，构造签名与 `DistanceApproachIPOPTInterface` 基本一致，差异集中在 `eval_f / eval_g / get_bounds_info`。

- **CUDA 版**（`distance_approach_ipopt_cuda_interface.*`，2859 行）：用 CUDA kernel 加速雅可比/Hessian；对外 API 与基础版完全相同。
- **FixedTs 版**（`distance_approach_ipopt_fixed_ts_interface.*`）：`time_start_index_` 段从决策变量里移除，`t_k ≡ 1`；去掉时间约束，显著减小 NLP 规模，适合对终点时间不敏感的场景。
- **FixedDual 版**（`distance_approach_ipopt_fixed_dual_interface.*`）：冻结 `l_warm_up / n_warm_up`，对偶变量不参与优化，仅优化 `(x, u, t)`；收敛更快但牺牲最优性。
- **RelaxEnd 版**（`distance_approach_ipopt_relax_end_interface.*`）：把 `x_N = xf` 替换成 `|x_N − xf|^2 ≤ ε`（或加入目标），防止 warm-start 偏差导致问题不可行。`weight_end_state` 控制末态偏离的二次惩罚。
- **RelaxEndSlack 版**（`distance_approach_ipopt_relax_end_slack_interface.*`）：在 RelaxEnd 的基础上，把每个 OBCA 不等式改成 `... + s ≥ d_min`，`s ≥ 0` 作为 slack 变量；`weight_slack` 控制 slack 代价；构造函数多了 `s_warm_up`。

### 4.6 `DualVariableWarmStartProblem`

**文件**：`trajectory_smoother/dual_variable_warm_start_problem.h` / `.cc`（含 4 种 interface 共约 2200 行）

```cpp
class DualVariableWarmStartProblem {
public:
    explicit DualVariableWarmStartProblem(const PlannerOpenSpaceConfig& planner_open_space_config);
    explicit DualVariableWarmStartProblem(const DualVariableWarmStartConfig& cfg);   // park generic

    bool Solve(
        size_t horizon, double ts,
        const Eigen::MatrixXd& ego,
        size_t obstacles_num,
        const Eigen::MatrixXi& obstacles_edges_num,
        const Eigen::MatrixXd& obstacles_A,
        const Eigen::MatrixXd& obstacles_b,
        const Eigen::MatrixXd& xWS,                // 给定轨迹
        Eigen::MatrixXd* l_warm_up,                // 输出 λ 初值
        Eigen::MatrixXd* n_warm_up,                // 输出 μ 初值
        Eigen::MatrixXd* s_warm_up);               // 输出 slack 初值（SLACKQP 模式）
};
```

目标：给定已固定的 `xWS`，对每一时刻 `k` 与每个障碍 `m`，最小化 `β * (violation^2) + weight_d * (λ,μ 的 L2)`，让 `λ, μ` 尽量满足 OBCA 不等式，再把解作为主 OBCA 问题的热启。按 `qp_format` 分发：

| qp_format (`DualWarmUpMode`) | Interface | 特点 |
| --- | --- | --- |
| `IPOPT` = 0 | `DualVariableWarmStartIPOPTInterface` | 通用非线性 |
| `IPOPTQP` = 1 | `DualVariableWarmStartIPOPTQPInterface` | QP 形式，用 IPOPT 求解 |
| `OSQP` = 2（默认） | `DualVariableWarmStartOSQPInterface` | 直接用 OSQP，最快 |
| `SLACKQP` = 4 | `DualVariableWarmStartSlackOSQPInterface` | 带 slack，避免严格不可行 |
| `DEBUG` = 3 | 仅日志 | 调试用 |

#### 4.6.1 `DualVariableWarmStartOSQPInterface`（默认）

头文件：`trajectory_smoother/dual_variable_warm_start_osqp_interface.h`。核心方法：

- `assemble_P(P_data, P_indices, P_indptr)`：CSC 格式装配二次项矩阵 `P`；
- `assemble_constraint(A_data, A_indices, A_indptr)`：装配约束矩阵 `A`；
- `assembleA(r, c, P_data, P_indices, P_indptr)`：把 P/A 的密集形式再展平；
- `optimize()`：调用 OSQP solver；
- `check_solution(l_warm_up, n_warm_up)`：可选地把解代回检查约束满足度；
- `get_optimization_results(l_warm_up, n_warm_up)`：拷贝到 Eigen 输出。

私有状态包括 `osqp_config_`（`alpha / eps_abs / eps_rel / max_iter / polish / osqp_debug_log`）、`lambda_horizon_ = obstacles_edges_sum_ * (horizon+1)`、`miu_horizon_ = 4 * obstacles_num_ * (horizon+1)`。

### 4.7 `IterativeAnchoringSmoother`

**文件**：`trajectory_smoother/iterative_anchoring_smoother.h` / `.cc`（786 行）

对应论文：**DL-IAPS and PJSO: A Path/Speed Decoupled Trajectory Optimization**（Jinyun 等, 2020）。**路径**层用 FEM 位姿偏差平滑（`FemPosDeviationSmoother`）；**速度**层用 Piecewise-Jerk Speed。相比 OBCA，收敛快、鲁棒性好、但严格路径/速度解耦。

#### 4.7.1 公有接口

```cpp
class IterativeAnchoringSmoother {
public:
    explicit IterativeAnchoringSmoother(const PlannerOpenSpaceConfig& planner_open_space_config);
    explicit IterativeAnchoringSmoother(const IterativeAnchoringConfig& iterative_anchoring_config);

    bool Smooth(
        const Eigen::MatrixXd& xWS,                                 // 4x(N+1) warm-start 状态
        double init_a, double init_v,
        const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
        DiscretizedTrajectory* discretized_trajectory);
};
```

入口期望 `xWS` 是**单段**（`HybridAStar::TrajectoryPartition` 拆段后的其中一段），这也是为何上层 `open_space_trajectory_optimizer.cc` 会对每段分别调 `Smooth`。

#### 4.7.2 `Smooth()` 主流程（`iterative_anchoring_smoother.cc:68`）

1. `gear_ = CheckGear(xWS)`：根据起始位姿与下一点方向向量夹角 <π/2 判定是否前进；
2. 障碍顶点列 → 线段列 `obstacles_linesegments_vec_`；
3. 把 `xWS` 累积弧长，写入 `warm_start_path` (`DiscretizedPath`)；校验累积弧长 > 0.1；
4. 以 `interpolated_delta_s`（默认 0.1m）重新均匀插值，得到 `interpolated_warm_start_point2ds`；若点数 < 6，关闭 `enforce_initial_kappa_`；
5. `AdjustStartEndHeading` 微调首末点，保证 `heading = atan2(dy,dx)` 与 warm-start 完全一致（用到 `xWS(2,0)` 与 `xWS(2,cols-1)`）；
6. `SetPathProfile(point2d, raw_path_points)`：离散点阶差分算 theta / kappa，得到 `DiscretizedPath`；
7. `GenerateInitialBounds(path_points, bounds)`：根据 `iterative_anchoring_config_.default_bound()` 或估计得到每点的平滑半径；
8. `CheckCollisionAvoidance(path_points, colliding_point_index)`：用 `ADC box vs 障碍线段` 判碰撞，记录冲撞点；
9. `SmoothPath(raw, bounds, smoothed, box)`：主体平滑。内部迭代调用 `FemPosDeviationSmoother`（由 `fem_pos_deviation_smoother_config` 配置），每轮检查碰撞，失败时 `AdjustPathBounds` 压缩 bounds，或在园区版加入 `point_box` 软约束；每轮按 `collision_decrease_ratio` 缩小 bounds；
10. `SmoothSpeed(init_a, init_v, path_length, smoothed_speeds)`：调 `PiecewiseJerkSpeedProblem` 在 `s-t` 平面最小化 `w_a*a² + w_jerk*j² + w_kappa*(kappa*v²)² + ...`；
11. `CombinePathAndSpeed`：把 `path_points` 和 `speed_data` 按时间合成 `DiscretizedTrajectory`；
12. `AdjustPathAndSpeedByGear`：倒车段把 `v, a` 置负、`theta` 加 π。

#### 4.7.3 其他私有方法

- `AdjustStartEndHeading`（`:234`）：调整首点 / 末点的相邻点坐标，使其差分方向等于目标 heading；
- `ReAnchoring(colliding, path)`（`:271`）：对每个冲撞点做若干次高斯扰动重采样，最多 `reanchoring_trails_num_` 次；
- `GenerateInitialBounds`（`:382`）：按 `default_bound` 或估计的方法生成每点的横向自由度；
- `SmoothPath`（`:414`）：核心 FEM 平滑循环；
- `CheckCollisionAvoidance`（`:477`）：对平滑后的 `path_points` 逐点做 `Box2d` vs 障碍线段的 `HasOverlap`；
- `AdjustPathBounds`（`:527`）：对冲突点缩小 bounds；
- `SetPathProfile`（`:549`）：用 `DiscretePointsMath` 填充 `theta/kappa/s`；
- `CheckGear`（`:582`）：判断档位（返回 true 表前进）；
- `SmoothSpeed`（`:590`）：Piecewise-Jerk 速度优化，参数取自 `iterative_anchoring_config_.max_forward_v / max_reverse_v / max_forward_acc / max_reverse_acc / max_acc_jerk`；
- `CombinePathAndSpeed`（`:668`）：按时间离散 `t_k = k*delta_t_`，查表 `path.Evaluate(speed.s(t_k))`；
- `AdjustPathAndSpeedByGear`（`:716`）：倒车段整体取反；
- `GenerateStopProfileFromPolynomial` / `IsValidPolynomialProfile`（`:732` / `:757`）：基于五次多项式构造 **Stop** 速度曲线，作为极端情况的备选；
- `CalcHeadings`（`:769`）：中心差分估计离散点航向。

#### 4.7.4 park-generic 扩展

`box_vec_` / `box_vec_vec_` 保存当前平滑"锚定框"序列；`SmoothPath(..., point_box)` 额外接受 `point_box` 软约束（例如车位线），驱动 FEM 把每个路径点钉在对应的凸盒子内。

### 4.8 `planning_block.h`

仅声明 `DEFINE_PLANNING_BLOCK` / `ADEBUG_PLANNING_BLOCK` 等宏，用于把调试信息写入 `PlanningInternal` proto，方便 dreamview 回放。无业务逻辑。

## 5. `utils/` — 几何与 ROI 工具

`utils/` 里放的是**静态工具类**，不持有状态，被上层 Task（`open_space_roi_decider*`、`open_space_trajectory_optimizer*`）大量复用。

### 5.1 `OpenSpaceRoiUtil`

**文件**：`utils/open_space_roi_util.h` / `.cc`（346 行）

#### 5.1.1 公有接口

```cpp
class OpenSpaceRoiUtil {
public:
    // 主入口：把 open_space_info 里的 obstacles_vertices_vec 转成 Ax≥b 超平面表示
    static bool FormulateBoundaryConstraints(OpenSpaceInfo* open_space_info);

    // ROI 的 XY 外接矩形（顺序 xmin, xmax, ymin, ymax）
    static void GetRoiXYBoundary(
        const std::vector<std::vector<common::math::Vec2d>>& roi_parking_boundary,
        std::vector<double>* XYBoundary);

    // 三个重载的坐标变换：把 world 坐标平移+旋转到 origin/heading 参考系
    static void TransformByOriginPoint(
        const common::math::Vec2d& origin_point, double heading,
        std::vector<std::vector<common::math::Vec2d>>* roi_parking_boundary);
    static void TransformByOriginPoint(
        const common::math::Vec2d& origin_point, double heading,
        std::vector<common::math::Vec2d>* roi_parking_boundary);
    static void TransformByOriginPoint(
        const common::math::Vec2d& origin_point, double heading,
        common::math::Vec2d* point);

    // 针对整个 OpenSpaceInfo 做整体变换：同时把 obstacles_vertices_vec、end_pose、ROI_xy_boundary 都处理
    static void TransformByOriginPoint(
        const Vec2d& origin_point, const double& origin_heading,
        OpenSpaceInfo* open_space_info);

    // 从 frame 里筛出需要考虑的障碍物，顶点按顺时针闭合写入 roi_parking_boundary
    static bool LoadObstacles(
        double filtering_distance, double perception_obstacle_buffer,
        const std::vector<double>& xy_boundary,
        Frame* frame,
        std::vector<std::vector<common::math::Vec2d>>* roi_parking_boundary);

    // 判断某个 obstacle 是否可以直接丢弃
    static bool FilterOutObstacle(
        double filtering_distance, const Frame& frame,
        const std::vector<double>& xy_boundary, const Obstacle& obstacle);

    // 把 obstacles_vertices_vec 转成 Ax≥b 矩阵对
    static bool LoadObstacleInHyperPlanes(OpenSpaceInfo* open_space_info);
    static bool GetHyperPlanes(
        const size_t& obstacles_num,
        const Eigen::MatrixXi& obstacles_edges_num,
        const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
        Eigen::MatrixXd* A_all, Eigen::MatrixXd* b_all);

    // 多边形顶点顺时针/逆时针判定与调整
    static bool IsPolygonClockwise(const std::vector<Vec2d>& polygon);
    static bool AdjustPointsOrderToClockwise(std::vector<Vec2d>* polygon);

    // 把泊车位顶点按"左上→右上→右下→左下"的顺序重排，参考 nearby_path 的方向
    static bool UpdateParkingPointsOrder(
        const apollo::hdmap::Path& nearby_path, std::vector<Vec2d>* points);
};
```

#### 5.1.2 `FormulateBoundaryConstraints` 细节（`open_space_roi_util.cc:36`）

1. 拿 `open_space_info->mutable_obstacles_vertices_vec()` 引用；
2. 为每个障碍物设置 `obstacles_edges_num(i, 0) = vertices.size() - 1`（顶点闭合后的边数）；
3. 写回 `obstacles_num = obstacles_vertices_vec.size()`；
4. 调 `LoadObstacleInHyperPlanes` 把每个障碍多边形转成 `Ax ≥ b`。

#### 5.1.3 `GetHyperPlanes`（`:194`）— 顶点 → 超平面

对每个障碍，顺序读取相邻顶点 `v1, v2`：

- 若 `|v1.x − v2.x| < ε`（竖直边）：`A = (±1, 0), b = ±v1.x`，符号根据 `v2.y < v1.y` 决定。
- 若 `|v1.y − v2.y| < ε`（水平边）：`A = (0, ±1), b = ±v1.y`。
- 一般情形：用 `[v1.x 1; v2.x 1]^{-1} * [v1.y; v2.y]` 解出直线 `y = a*x + b`，转成 `(−a, 1) · p = b` 或 `(a, −1) · p = −b`，方向由 `v1.x < v2.x` 决定。

输出 `A_all ∈ R^{Σn_m × 2}`、`b_all ∈ R^{Σn_m × 1}`，直接喂给 OBCA 的约束装配。

#### 5.1.4 `LoadObstacles`（`:100`）— 感知障碍 → ROI

- 遍历 `frame->GetObstacleList()`；
- `FilterOutObstacle` 过滤：虚拟障碍（`IsVirtual`）、中心超出 XY 框、或到自车+终点都远于 `filtering_distance` 的全部丢弃；
- 对剩下的每个障碍物取 `PerceptionBoundingBox()`，`LongitudinalExtend / LateralExtend(perception_obstacle_buffer)` 扩一圈；
- `GetAllCorners()` 返回逆时针顶点，反转成顺时针，末尾 push `front()` 实现闭环；
- 追加到 `roi_parking_boundary`。

#### 5.1.5 `UpdateParkingPointsOrder`（`:313`）

泊车位必须以"靠近自车"的顶点作为起点，并按顺时针重排：

1. `AdjustPointsOrderToClockwise`；
2. 对每个顶点 `p_i` 投影到 `nearby_path`，求 `(s, l)`；选择使 `|s|+2|l|` 最小的下标 `min_index`；
3. 若 `sum_l < 0`（车位在参考线右侧）：从 `min_index` 起循环重排；否则按反向重排，保持"左上→右上→右下→左下"的规范。

### 5.2 `OpenSpaceTrajectoryOptimizerUtil`

**文件**：`utils/open_space_trajectory_optimizer_util.h` / `.cc`（标注 park generic）

```cpp
class OpenSpaceTrajectoryOptimizerUtil {
public:
    // 检查 bounding_box 是否与任一障碍线段相交（复用自 HybridAStar::ValidityCheck 的思路）
    static bool BoxOverlapObstacles(
        const common::math::Box2d& bounding_box,
        const std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec);

    // 为平滑器生成"锚定盒"：对每个路径点沿 heading 法向扩展，直到碰到障碍为止
    static void GeneratePointBox(
        const std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec,
        const std::vector<std::pair<double, double>>& result,
        std::vector<std::vector<common::math::Vec2d>>& box_vec);

    // 把 (x, y, phi) 从 world 坐标系变换到本地参考系（减 origin、旋转 -rotate_angle）
    static void PathPointNormalizing(
        double rotate_angle, const common::math::Vec2d& translate_origin,
        double* x, double* y, double* phi);

    // PathPointNormalizing 的逆变换
    static void PathPointDeNormalizing(
        double rotate_angle, const common::math::Vec2d& translate_origin,
        double* x, double* y, double* phi);
};
```

`PathPointNormalizing` / `PathPointDeNormalizing` 用于把自车/障碍物统一归一化到泊车位局部坐标系下再求解，解完之后再变换回世界系；相当于 OBCA 中的"坐标归一化"步骤，可以显著改善数值条件。

## 6. `tools/` — 命令行 / 绑定调试

- `tools/hybrid_a_star_wrapper.cc`：把 `HybridAStar::Plan` 包成 C ABI，供 Python ctypes 调用，用于离线调参。
- `tools/distance_approach_problem_wrapper.cc`：同理包装 `DistanceApproachProblem::Solve`。
- `tools/open_space_roi_wrapper.cc`：包装 `OpenSpaceRoiUtil` 的几何函数，用于在脚本里验证 ROI 提取结果。

三者均只做一层 `extern "C"` 转发，没有独立算法。

## 7. `park_data_center/` — 泊车/近场任务共享的 NudgeInfo 缓存

**目录下全部文件**：

```
park_data_center/
├── BUILD
├── cyberfile.xml
├── nudge_info.h
├── nudge_info.cc
├── park_data_center.h
├── park_data_center.cc
└── park_data_center_test.cc
```

### 7.1 模块职责

与文档预期略有差异——`park_data_center` **不是**泊车的目标车位/状态机存储，而是一组**按参考线 key 索引的"绕行信息"（Nudge）缓存**。在 `obstacle_nudge_decider / lane_borrow_path_generic / lane_change_path_generic` 等 Task 之间共享以下信息：

- 哪些障碍物正在被跟踪（`tracking_nudge_obs_info_`）；
- 当前帧每个障碍的绕行概率 `nudge_probability`、累计 tracking 时长；
- 绕行左右方向（`NudgeType::LEFT/RIGHT/WAITING/NONE`）；
- 是否合并多障碍块 (`is_merge_block_obs_`)；
- 绕行 key points `extra_nudge_key_points_` 作为路径生成器的软锚点。

这些数据跨帧、跨 Task，所以要一个**单例**来缓存；Apollo 用 `IndexedQueue` 保存历史 N 帧（`FLAGS_max_frame_history_num`）以便下一帧做障碍物跟踪匹配。

### 7.2 `RemainNudgeSpace` 结构

```cpp
// nudge_info.h:46
struct RemainNudgeSpace {
    double left_lane_remain  = std::numeric_limits<double>::min();
    double left_road_remain  = std::numeric_limits<double>::min();
    double right_lane_remain = std::numeric_limits<double>::min();
    double right_road_remain = std::numeric_limits<double>::min();
    hdmap::LaneBoundaryType::Type left_lane_boundary_type  = hdmap::LaneBoundaryType::CURB;
    hdmap::LaneBoundaryType::Type right_lane_boundary_type = hdmap::LaneBoundaryType::CURB;
};
```

描述障碍物左右两侧剩余空间（车道 vs 路面），以及两侧车道线类型（路缘 / 白线 / 黄线等），用于决策"能不能绕"。

### 7.3 `NudgeType` 枚举

```cpp
enum class NudgeType {
    NONE = 0, LEFT = 1, RIGHT = 2, WAITING = 3,
};
```

`WAITING` 表示还在观察、暂不执行绕行。

### 7.4 `NudgeObstacleInfo` 结构

```cpp
// nudge_info.h:62
struct NudgeObstacleInfo {
    Polygon2d origin_polygon;            // 感知原始多边形
    Polygon2d expand_polygon;            // 向外扩展后的多边形（用于决策）
    SLBoundary sl_boundary;              // 在参考线 SL 下的边界
    RemainNudgeSpace remain_nudge_space;
    bool   is_passable       = false;
    bool   is_all_on_lane    = false;
    double nudge_probability = 0.0;
    double tracking_time     = 0.0;
    double start_track_time  = std::numeric_limits<double>::max();
    double last_track_time   = 0;
    NudgeType nudge_type     = NudgeType::NONE;

    double obs_tracking_time()  { return last_track_time - start_track_time; }
    void   Clear()              { nudge_probability = 0; tracking_time = 0; nudge_type = NudgeType::NONE; }
    void   PrintPolygonInfo(std::string obs_id);   // 把多边形写入 print_curve 供 dreamview 显示
};
```

### 7.5 `NudgeInfo` 类

**文件**：`park_data_center/nudge_info.h` / `.cc`

一条参考线对应一份 `NudgeInfo`，包含"跟踪中的障碍列表 + 阻挡点多边形 + 绕行锚点"。

#### 7.5.1 公有接口

```cpp
class NudgeInfo {
public:
    NudgeInfo() = default;

    // 开关
    void set_enable(bool is_enable);
    bool is_enable() const;
    void set_is_merge_block_obs(bool is_merge);
    bool is_merge_block_obs() const;

    // 跟踪障碍表（按 id 索引）
    const std::unordered_map<std::string, NudgeObstacleInfo>& tracking_nudge_obs_info() const;
    std::unordered_map<std::string, NudgeObstacleInfo>* mutable_tracking_nudge_obs_info();

    // 阻挡多边形（在 SL 下）
    const std::vector<SLPolygon>& block_sl_polygons() const;
    std::vector<SLPolygon>* mutable_block_sl_polygons();

    // 本帧更新过的 obs_id 集合
    const std::set<std::string>& update_ids() const;
    std::set<std::string>* mutable_update_ids();

    // 绕行左右锚点（extra_nudge_key_points_[0] = 左侧 SL 点, [1] = 右侧 SL 点）
    const std::vector<std::vector<SLPoint>>& extra_nudge_key_points() const;
    std::vector<std::vector<SLPoint>>* mutable_extra_nudge_key_points();

    // 查询
    bool IsObsIgnoreNudgeDecision(std::string obs_id, double obs_start_s, double check_dis) const;
    bool NeedCheckObsCollision(std::string obs_id) const;
    void GetFirstBlockSLPolygon(std::string obs_id, SLPolygon* obs_sl_polygon) const;
    void SortBlockSLPolygons();
    void PrintDebugInfo();
    bool GetInterpolatedNudgeL(bool is_left_nudge, double s, double* nudge_l) const;

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

#### 7.5.2 关键查询方法实现

- `NeedCheckObsCollision(obs_id)`（`nudge_info.cc:72`）：若 `update_ids_` 包含 `obs_id` 则返回 true，代表该帧需要对它做碰撞重检。
- `IsObsIgnoreNudgeDecision(obs_id, obs_start_s, check_dis)`（`:84`）：若该障碍**没有**在本帧更新过，且它距离上一次绕行锚点 `extra_nudge_key_points_[0][1].s()` 还有 `> check_dis`，则可以忽略；额外条件：`extra_nudge_key_points_` 至少有两层 > 2 个点。
- `GetFirstBlockSLPolygon(obs_id, out)`（`:101`）：把跟当前 `obs_id` 匹配的第一个阻挡 SL 多边形取出，前提是 `is_merge_block_obs_` 打开。
- `SortBlockSLPolygons()`（`:111`）：按 `MinS()` 升序排列，确保从近到远处理。
- `GetInterpolatedNudgeL(is_left_nudge, s, nudge_l)`（`:135`）：在 `extra_nudge_key_points_[0 或 1]`（左/右）中按 `s` 做线性插值，返回对应的 `l`。超出两端时 clamp 到端点。这个方法被 `lane_borrow_path_generic` 用作路径生成的软锚点。

### 7.6 `ParkDataCenter` 单例

**文件**：`park_data_center/park_data_center.h` / `.cc`

```cpp
using NudgeInfoMap = std::map<std::size_t, NudgeInfo>;
using NudgeInfoIndexedQueue = IndexedQueue<uint32_t, NudgeInfoMap>;

class ParkDataCenter {
public:
    ~ParkDataCenter();

    // 把当前帧 NudgeInfoMap 入队历史（key = sequence_num），并清空 current
    void UpdateHistoryNudgeInfo(uint32_t sequence_num);

    // 根据历史 sequence_num 与参考线 key 查前一帧的跟踪信息
    bool GetLastFrameNudgeTrackInfo(
        uint32_t sequence_num,
        std::size_t reference_line_key,
        std::unordered_map<std::string, NudgeObstacleInfo>* track_info);

    // 当前帧该参考线上的 NudgeInfo（不存在则懒构造）
    const NudgeInfo& current_nudge_info(std::size_t reference_line_key);
    NudgeInfo*       mutable_current_nudge_info(std::size_t reference_line_key);

private:
    static bool is_init_;
    NudgeInfoMap current_nudge_info_data_;
    NudgeInfoIndexedQueue history_nudge_info_data_
        = NudgeInfoIndexedQueue(FLAGS_max_frame_history_num);

    DECLARE_SINGLETON(ParkDataCenter)
};
```

#### 7.6.1 典型生命周期

1. 帧开始：`obstacle_nudge_decider` 调 `ParkDataCenter::Instance()->mutable_current_nudge_info(ref_line_key)` 拿引用，填写 tracking、block 多边形、extra key points；
2. 后续 Task：`lane_borrow_path_generic` / `lane_change_path_generic` 通过 `current_nudge_info(ref_line_key)` 只读消费这些数据，驱动路径生成器在 `GetInterpolatedNudgeL` 给出的 `l` 目标附近生成曲线；
3. 帧结束：`obstacle_nudge_decider::Process` 末尾调 `ParkDataCenter::Instance()->UpdateHistoryNudgeInfo(frame->SequenceNum())`，把 `current` map 入队、清空，下一帧从空白状态开始。

`IndexedQueue<uint32_t, NudgeInfoMap>` 内部实现在 `planning_base/common/indexed_queue.h:34`，是一个"容量上限 `FLAGS_max_frame_history_num` 的 FIFO 环形缓存"。`GetLastFrameNudgeTrackInfo` 本质就是调 `history_nudge_info_data_.Find(sequence_num)`。

#### 7.6.2 与 `planning_open_space` 的关系

两个模块**没有**直接调用关系：

- `planning_open_space` 的 Hybrid A\* / OBCA 只接受 `xWS, uWS, XYbounds, obstacles_A/b` 等显式参数，不访问 `ParkDataCenter`；
- `ParkDataCenter` 的 NudgeInfo 只被 `tasks/obstacle_nudge_decider`、`tasks/lane_borrow_path_generic`、`tasks/lane_change_path_generic` 使用。

它们的共同角色是服务**泊车/近场**场景：`valet_parking_park` Scenario 里可能同时激活"路径绕行(Nudge) → Hybrid A\* 回正 → OBCA 精解"的流水线，两个模块各司其职。

### 7.7 `park_data_center_test.cc`

只有 30 行的 gtest，验证 `ParkDataCenter::Instance()` 单例行为与 `UpdateHistoryNudgeInfo` 顺序，不涉及算法逻辑。

## 8. 算法要点总结

本节把散落在各文件的算法细节集中在一处，便于对照源码理解。

### 8.1 Hybrid A\* 的离散化

**状态空间**：`(x, y, φ) ∈ R^2 × S^1`。

- `xy_grid_resolution_`：默认 `0.2 m`，`X Y` 栅格分辨率；
- `phi_grid_resolution_`：默认 `0.05 rad ≈ 2.86°`，航向栅格分辨率；
- **索引**：`index = "x_grid_y_grid_phi_grid"`，`phi_grid_` 用 `-π` 作零点。

**运动基元**（`Next_node_generator`）：

- 展开个数 `next_node_num_`（默认 10）= 前进 5 条 + 倒车 5 条；
- 舵角 `steering ∈ {−max_steer, ..., +max_steer}` 线性离散；
- 弧长 `arc_length_` 公式见 3.4.3，保证跨栅格；
- 中点法积分 `φ' = dist / wheel_base * tan(steer)`。

**启发**：`h(n) = GridSearch::CheckDpMap(n.x, n.y)`，即"二维避障最短距离 × `xy_grid_resolution_`"。这满足**admissible**，因为它忽略运动学只取几何下界。

**解析扩展（Reed-Shepp）**：每迭代一次就尝试 `AnalyticExpansion`——这是 Hybrid A\* 比普通 A\* 快数量级的关键。一旦某次 RS 曲线无碰撞，就把整条 RS 接在当前节点后，直接生成 `final_node_` 候选。

**代价函数**：见 3.4.6，惩罚前进/倒车、切齿、转向、转向变化、短段、软边界。

**提前终止**：`astar_max_search_time` 限定搜索总时间，`desired_explored_num` 限制候选解个数；一旦超过即以当前最优 final_node 返回。

### 8.2 Reed-Shepp 的 48 种组合

Reeds & Shepp (1990) 证明：任意两点之间的最短前进+倒车 Dubins 曲线属于以下几类：

| 类型 | 段数 | 对应 Apollo 函数 |
| --- | --- | --- |
| `SCS`（直-弧-直） | 3 | `SCS` |
| `CS`（弧-直） | 2 | `CS` |
| `CSC` | 3 | `CSC` |
| `CCC` | 3 | `CCC` |
| `CCCC` | 4 | `CCCC` |
| `CCSC` | 4 | `CCSC` |
| `CCSCC` | 5 | `CCSCC` |

每个模板下还有若干对称变种（y ↔ −y, φ ↔ −φ, 时间反演等），合计 48 条候选。Apollo 对每条用 `GenerateLocalConfigurations` + `Interpolation` 离散化，再用前述代价函数选最小 cost。

### 8.3 OBCA 对偶公式

对每个障碍 `m`（凸多边形，H-形式 `A_m p ≥ b_m`），车辆多边形以 `G p ≥ g`（矩形 4 条边）描述。OBCA 原始可行性：车辆凸包与障碍凸包不交。其 KKT 对偶：
$$
\exists\ \lambda_m \ge 0,\ \mu_m \ge 0:\quad
G^\top \mu_m + R(\varphi)^\top A_m^\top \lambda_m = 0,\quad
-g^\top \mu_m + (A_m T - b_m)^\top \lambda_m > 0
$$

Apollo 把第一条等式松弛成"范数 ≤ 1"（避免 $\lambda, \mu$ 同时缩放到无穷），第二条加入 `min_safety_distance_` 得到最终三组约束（见 4.1）。决策变量 `λ ∈ R^{n_m × N+1}`、`μ ∈ R^{4 × M × (N+1)}`（4 为车辆边数）。维度庞大是 OBCA 慢的主因——因此需要 `DualVariableWarmStartProblem` 先给一个接近可行的初值。

### 8.4 DL-IAPS 路径-速度解耦

IterativeAnchoringSmoother 对应论文：

> Zhou J, He R, Wang Y et al. **DL-IAPS and PJSO**: A Path/Speed Decoupled Trajectory Optimization and its Application in Autonomous Driving. arXiv:2009.11135.

核心思想：

1. **路径**：FEM (Finite Element Method) Position Deviation Smoother 以 `x, y` 为 FEM 节点，目标 `Σ |x_{i+1} − 2x_i + x_{i−1}|^2`（二阶差分 ≈ 离散曲率）+ `Σ |x_i − x_ref_i|^2`（锚定），约束每个点在一个半径 `bounds[i]` 的自由圆内。求解器：`FemPosDeviationSmoother`（QP）。
2. **速度**：Piecewise Jerk Speed，决策变量 `(s_i, v_i, a_i)`，代价 `w_a a² + w_jerk j² + w_kappa (κ v²)²`，QP 形式。
3. **碰撞回退**：每轮路径平滑后做碰撞检查，冲撞点对应的 `bounds[i]` 乘以 `collision_decrease_ratio`（默认 0.9）再求解，直到收敛。

### 8.5 常用数值技巧

- **ADOL-C 自动微分**：Apollo 的 OBCA 用 `adouble` 重载做一次 `tag_f / tag_g / tag_L` tape 记录，之后的 Jacobian/Hessian 直接 `ColPack` 稀疏提取；切换到手写梯度需要关闭 `enable_jacobian_ad_`。
- **坐标归一化**：`PathPointNormalizing` 把 warm-start 搬到车位局部系，使 `(x, y)` 的数量级在 `10^1`，IPOPT 数值稳定性显著提升。
- **DP Map 缓存**：全局只算一次，Hybrid A\* 每次启发查表 O(1)。
- **ESDF 软边界**：`AddSoftCost` 递归 4 邻域扩散，把距离信息写进 `dp_map_[idx].distance_to_obstacle_`，给搜索施加"软斥力"，使粗解不压车位线。

## 9. `proto/planner_open_space_config.proto` — 配置字段完整索引

**文件**：`proto/planner_open_space_config.proto`

### 9.1 顶层 `PlannerOpenSpaceConfig`

| 字段 | 类型 | 默认 | 作用 |
| --- | --- | --- | --- |
| `roi_config` | `ROIConfig` | — | ROI 提取配置 |
| `warm_start_config` | `WarmStartConfig` | — | Hybrid A\* / Reed-Shepp 配置 |
| `dual_variable_warm_start_config` | `DualVariableWarmStartConfig` | — | 对偶变量热启 |
| `distance_approach_config` | `DistanceApproachConfig` | — | OBCA 主求解配置 |
| `iterative_anchoring_smoother_config` | `IterativeAnchoringConfig` | — | DL-IAPS 配置 |
| `delta_t` | float | 1.0 | 速度规划采样步长 |
| `near_destination_threshold` | double | 0.001 | 判定到达终点的阈值 |
| `enable_linear_interpolation` | bool | false | 是否对输出做线性插值 |
| `is_near_destination_theta_threshold` | double | 0.05 | 终点航向误差阈值 |

### 9.2 `ROIConfig`

| 字段 | 默认 | 作用 |
| --- | --- | --- |
| `roi_longitudinal_range_start` | 10.0 | ROI 向后延伸米 |
| `roi_longitudinal_range_end` | 10.0 | ROI 向前延伸米 |
| `parking_start_range` | 7.0 | 感知车位的阈值范围 |
| `parking_inwards` | false | 是否为倒库式泊入 |

### 9.3 `WarmStartConfig`（Hybrid A\* 核心）

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `xy_grid_resolution` | 0.2 | xy 栅格分辨率（米） |
| `phi_grid_resolution` | 0.05 | 航向栅格分辨率（rad） |
| `next_node_num` | 10 | 每步展开候选数（含前进/倒车） |
| `step_size` | 0.5 | 展开弧段内部采样步长 |
| `traj_forward_penalty` | 0.0 | 前进代价 |
| `traj_back_penalty` | 0.0 | 倒车代价 |
| `traj_gear_switch_penalty` | 10.0 | 切齿代价 |
| `traj_steer_penalty` | 100.0 | 转向幅度代价 |
| `traj_steer_change_penalty` | 10.0 | 转向变化代价 |
| `traj_short_length_penalty` | 10.0 | 短段代价 |
| `grid_a_star_xy_resolution` | 0.1 | 2D 启发栅格分辨率 |
| `node_radius` | 0.5 | 2D 启发的碰撞半径 |
| `s_curve_config` | `SpeedOptimizerConfig` | S-curve 速度权重 |
| `traj_kappa_contraint_ratio` | 0.7 | 最大曲率缩放（给平滑留裕度） |
| `max_explored_num` | 10000 | 搜索节点上限 |
| `desired_explored_num` | 10000 | 达到多少候选解即停 |
| `traj_expected_shortest_length` | 1.0 | 短段阈值（米） |
| `astar_max_search_time` | 5.0 | 搜索时间上限（秒） |
| `esdf_range` | 2.0 | ESDF 扩散最大距离 |
| `soft_boundary_penalty` | 2.0 | 软边界惩罚权重 |
| `use_esdf` | true | 是否启用 ESDF |
| `traj_straight_length` | 1.0 | RS 终段直线长度 |

`SpeedOptimizerConfig`：`acc_weight = 1.0 / jerk_weight = 10.0 / kappa_penalty_weight = 1000.0 / ref_s_weight = 10.0 / ref_v_weight = 10.0`。

### 9.4 `DualVariableWarmStartConfig`

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `weight_d` | 1.0 | λ/μ L2 权重 |
| `ipopt_config` | `IpoptConfig` | IPOPT 参数 |
| `qp_format` | — | `DualWarmUpMode`：IPOPT / IPOPTQP / OSQP / DEBUG / SLACKQP |
| `min_safety_distance` | 0.0 | OBCA 最小安全距 |
| `debug_osqp` | false | OSQP 调试日志 |
| `beta` | 1.0 | 违反代价权重 |
| `osqp_config` | `OSQPConfig` | OSQP 参数 |

### 9.5 `DistanceApproachConfig`（OBCA 主求解）

常用字段（完整列表见 proto 文件 `planner_open_space_config.proto:137`）：

| 字段 | 说明 |
| --- | --- |
| `weight_steer / weight_a / weight_steer_rate / weight_a_rate` | 输入与变化率权重 |
| `weight_x / weight_y / weight_phi / weight_v` | 状态跟踪 warm-start 的权重 |
| `weight_steer_stitching / weight_a_stitching` | 与上一帧控制的拼接代价 |
| `weight_first_order_time / weight_second_order_time` | 时间正则化 |
| `min_safety_distance` / `max_safety_distance` | OBCA 的 `d_min, d_max` |
| `max_speed_forward / max_speed_reverse` | 速度约束 |
| `max_acceleration_forward / max_acceleration_reverse` | 加速度约束 |
| `min_time_sample_scaling / max_time_sample_scaling` | 时间缩放区间 |
| `use_fix_time` | 是否固定 Δt（等价于 FixedTs 变体） |
| `ipopt_config` | `IpoptConfig` |
| `enable_constraint_check / enable_hand_derivative / enable_derivative_check / enable_initial_final_check` | 调试开关 |
| `distance_approach_mode` | `DistanceApproachMode` 枚举 |
| `enable_jacobian_ad` | 是否启用 ADOL-C |
| `enable_check_initial_state` | 初态检查 |
| `weight_end_state` | RelaxEnd 的末态代价 |
| `weight_slack` | RelaxEndSlack 的 slack 代价 |

### 9.6 `IpoptConfig` / `IpoptSolverConfig`

两个 message 字段基本一致（`IpoptSolverConfig` 是 park generic 版的重命名）：`ipopt_print_level / mumps_mem_percent / mumps_pivtol / ipopt_max_iter / ipopt_tol / ipopt_acceptable_constr_viol_tol / ipopt_min_hessian_perturbation / ipopt_jacobian_regularization_value / ipopt_print_timing_statistics / ipopt_alpha_for_y / ipopt_recalc_y / ipopt_mu_init (default 0.1)`。

### 9.7 `OSQPConfig`

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `alpha` | 1.0 | Over-relaxation |
| `eps_abs` | 1e-3 | 绝对容差 |
| `eps_rel` | 1e-3 | 相对容差 |
| `max_iter` | 10000 | 最大迭代 |
| `polish` | true | 是否做 polish |
| `osqp_debug_log` | false | 调试日志 |

### 9.8 `IterativeAnchoringConfig`（DL-IAPS）

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `interpolated_delta_s` | 0.1 | 锚点插值步长（米） |
| `reanchoring_trails_num` | 50 | 重锚定尝试次数 |
| `reanchoring_pos_stddev` | 0.25 | 重锚定位置扰动 |
| `reanchoring_length_stddev` | 1.0 | 重锚定长度扰动 |
| `estimate_bound` | false | 是否估计 bounds |
| `default_bound` | 2.0 | 默认 bounds 半径 |
| `vehicle_shortest_dimension` | 1.04 | 车辆最窄尺寸 |
| `fem_pos_deviation_smoother_config` | `FemPosDeviationSmootherConfig` | FEM 平滑器参数 |
| `collision_decrease_ratio` | 0.9 | bounds 缩放比 |
| `max_forward_v / max_reverse_v` | 2.0 / 2.0 | 速度上限 |
| `max_forward_acc / max_reverse_acc` | 3.0 / 2.0 | 加速度上限 |
| `max_acc_jerk` | 4.0 | Jerk 上限 |
| `delta_t` | 0.2 | 时间离散步长 |
| `s_curve_config` | — | S-curve 权重 |

### 9.9 enum 一览

```proto
enum DualWarmUpMode         { IPOPT=0; IPOPTQP=1; OSQP=2; DEBUG=3; SLACKQP=4; }
enum DistanceApproachMode   {
    DISTANCE_APPROACH_IPOPT=0;
    DISTANCE_APPROACH_IPOPT_CUDA=1;
    DISTANCE_APPROACH_IPOPT_FIXED_TS=2;
    DISTANCE_APPROACH_IPOPT_FIXED_DUAL=3;
    DISTANCE_APPROACH_IPOPT_RELAX_END=4;
    DISTANCE_APPROACH_IPOPT_RELAX_END_SLACK=5;
}
enum DualVariableMode       { DUAL_VARIABLE_IPOPT=0; DUAL_VARIABLE_IPOPTQP=1; DUAL_VARIABLE_OSQP=2; DUAL_VARIABLE_DEBUG=3; }
```

## 10. 调用链参考：从 Task 到本模块

虽然 Task 不属于本次文档范围，以下简要列举让读者理解上下文：

1. `tasks/open_space_roi_decider*` 从 `Frame::open_space_info` 取出 `parking_spot_id` 与地图路径，通过 `OpenSpaceRoiUtil::LoadObstacles / UpdateParkingPointsOrder / TransformByOriginPoint / FormulateBoundaryConstraints` 得到 `XYbounds / obstacles_vertices_vec / obstacles_A,b / end_pose`。
2. `tasks/open_space_trajectory_provider` 的 `OpenSpaceTrajectoryOptimizer`（文件：`tasks/open_space_trajectory_provider/open_space_trajectory_optimizer.h:46`）：
   - 持有 `warm_start_ (HybridAStar)`、`distance_approach_ (DistanceApproachProblem)`、`dual_variable_warm_start_ (DualVariableWarmStartProblem)`、`iterative_anchoring_smoother_ (IterativeAnchoringSmoother)`；
   - `Plan()` 调 `warm_start_->Plan(...)` → `GenerateDecoupledTraj`（走 IA 通道） 或 `GenerateDistanceApproachTraj`（走 OBCA 通道，先 `dual_variable_warm_start_->Solve` 再 `distance_approach_->Solve`）；
   - 结果回写到 `optimized_trajectory_`，上层 Scenario 再拼回 stitching trajectory 发到控制。
3. `tasks/open_space_trajectory_optimizer_park/optimizer.*` 是 park generic 版入口，绕开 `PlannerOpenSpaceConfig`、直接用 `WarmStartConfig / DistanceApproachConfig / IterativeAnchoringConfig`。
4. `tasks/open_space_fallback_decider` 在 OBCA / IA 失败时兜底。

`park_data_center` 与上述链路**并行**存在，服务于 `tasks/obstacle_nudge_decider`、`tasks/lane_borrow_path_generic`、`tasks/lane_change_path_generic`，专用于近场绕行决策。

---

**参考源码位置一览（绝对路径）**：

- `modules/planning/planning_open_space/coarse_trajectory_generator/{hybrid_a_star, node3d, grid_search, reeds_shepp_path}.{h,cc}`
- `modules/planning/planning_open_space/trajectory_smoother/{distance_approach_problem, distance_approach_interface, distance_approach_ipopt[_cuda|_fixed_ts|_fixed_dual|_relax_end|_relax_end_slack]_interface, dual_variable_warm_start_{problem,ipopt_interface,ipopt_qp_interface,osqp_interface,slack_osqp_interface}, iterative_anchoring_smoother, planning_block}.{h,cc}`
- `modules/planning/planning_open_space/utils/{open_space_roi_util, open_space_trajectory_optimizer_util}.{h,cc}`
- `modules/planning/planning_open_space/proto/planner_open_space_config.proto`
- `modules/planning/park_data_center/{park_data_center, nudge_info}.{h,cc}`





