# Planning Util 工具函数库函数级源码解析

本文聚焦 `modules/planning/planning_base/common/util/` 目录，按函数级粒度拆解规划模块的 **6 个工具文件**：停车决策构建、配置加载、车辆状态校验、交通元素检查、坐标变换、调试可视化。

## 1. 模块定位

`util/` 是规划模块的**公共工具函数库**，为场景机（Scenario）、交通规则（TrafficRule）、规划任务（Task）提供无状态的辅助函数。

```
Scenario / TrafficRule / Task
    │
    ├── util::BuildStopDecision()     ← 停车决策
    ├── util::IsVehicleStateValid()   ← 状态校验
    ├── util::CheckStopSignOnReferenceLine() ← 交通元素
    ├── util::GetADCStopDeceleration()       ← 减速计算
    ├── util::WorldCoordToObjCoord()  ← 坐标变换
    ├── ConfigUtil::LoadMergedConfig() ← 配置加载
    ├── PrintCurves / PrintBox        ← 调试输出
    └── EvaluatorLogger               ← 评估日志
```

## 2. 目录结构

```
planning_base/common/util/
├── common.h / .cc           # BuildStopDecision 停车决策构建
├── config_util.h / .cc      # ConfigUtil 配置加载工具
├── evaluator_logger.h       # EvaluatorLogger 评估日志单例
├── math_util.h / .cc        # 坐标变换工具
├── print_debug_info.h / .cc # PrintPoints/PrintCurves/PrintBox 调试
└── util.h / .cc             # 车辆状态校验、交通元素检查、几何计算
```

## 3. BuildStopDecision — 停车决策构建

源码：`common.h` 与 `common.cc`

### 3.1 重载一：基于参考线 s 坐标

```cpp
int BuildStopDecision(const std::string& stop_wall_id,
                      const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag,
                      Frame* const frame,
                      ReferenceLineInfo* const reference_line_info,
                      double stop_wall_width = 4.0);
```

**参数说明**：

| 参数 | 说明 |
|------|------|
| `stop_wall_id` | 虚拟停车墙的唯一 ID |
| `stop_line_s` | 停车线在参考线上的 s 坐标 |
| `stop_distance` | 车头到停车线的安全距离 |
| `stop_reason_code` | 停车原因枚举（红灯/停车标志/行人等） |
| `wait_for_obstacles` | 需要等待的障碍物 ID 列表 |
| `decision_tag` | 决策标签（用于调试追踪） |
| `stop_wall_width` | 虚拟墙宽度，默认 4.0m |

**执行步骤**：

1. **边界检查**：验证 `stop_line_s` 在参考线 `[0, Length()]` 范围内
2. **创建虚拟障碍物**：调用 `frame->CreateStopObstacle()` 在 `stop_line_s` 处创建虚拟停车墙
3. **添加到参考线**：`reference_line_info->AddObstacle(obstacle)`
4. **计算停车点**：`stop_s = stop_line_s - stop_distance`，获取该点的坐标和航向
5. **构建 Stop 决策**：填充 `ObjectDecisionType::stop`（原因码、距离、航向、坐标、等待列表）
6. **注册决策**：`path_decision->AddLongitudinalDecision(decision_tag, stop_wall_id, stop)`

**返回值**：`0` 成功，`-1` 创建障碍物失败

### 3.2 重载二：基于车道 ID + 车道 s

```cpp
int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id,
                      const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag,
                      Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);
```

**与重载一的区别**：
- 通过 `lane_id + lane_s` 定位停车点（而非直接给参考线 s）
- 额外检查停车墙中心是否在车道上（`reference_line.IsOnLane`）
- 从障碍物的 SL 边界 `start_s` 减去 `stop_distance` 计算停车点

## 4. ConfigUtil — 配置加载工具

源码：`config_util.h` 与 `config_util.cc`

### 4.1 `TransformToPathName`

```cpp
static std::string TransformToPathName(const std::string& name);
```

将类名转为全小写路径名。用于从类名推导配置文件路径。

### 4.2 `GetFullPlanningClassName`

```cpp
static std::string GetFullPlanningClassName(const std::string& class_name);
```

**逻辑**：若 `class_name` 已包含 `::`，直接返回；否则拼接 `"apollo::planning::" + class_name`。

**用途**：`PluginManager::CreateInstance<T>()` 需要全限定类名。

### 4.3 `LoadMergedConfig<T>`

```cpp
template <typename T>
static bool LoadMergedConfig(const std::string& default_config_path,
                             const std::string& config_path, T* config);
```

**执行步骤**：
1. 加载默认配置到 `config`
2. 加载用户配置到 `spcific_config`（若不存在则跳过）
3. `config->MergeFrom(spcific_config)`：用户配置覆盖默认值

**语义**：protobuf MergeFrom — 用户只需声明想覆盖的字段。

### 4.4 `LoadOverridedConfig<T>`

```cpp
template <typename T>
static bool LoadOverridedConfig(const std::string& default_config_path,
                                const std::string& config_path, T* config);
```

**执行步骤**：
1. 尝试加载用户配置，成功则直接返回
2. 失败则回退到默认配置

**语义**：用户配置完全替代默认配置（非合并）。

## 5. 车辆状态与交通元素检查

源码：`util.h` 与 `util.cc`

### 5.1 `IsVehicleStateValid`

```cpp
bool IsVehicleStateValid(const VehicleState& vehicle_state);
```

检查车辆状态的 7 个关键字段是否为 NaN：`x, y, z, heading, kappa, linear_velocity, linear_acceleration`。任一为 NaN 返回 false。

### 5.2 `IsDifferentRouting`

```cpp
bool IsDifferentRouting(const PlanningCommand& first,
                        const PlanningCommand& second);
```

通过比较 header 的 `sequence_num` 和 `module_name` 判断两条规划命令是否来自不同的路由请求。

### 5.3 `GetADCStopDeceleration`

```cpp
double GetADCStopDeceleration(VehicleStateProvider* vehicle_state,
                              double adc_front_edge_s, double stop_line_s);
```

**计算公式**：`a = v² / (2 * stop_distance)`

**特殊处理**：
- 车速低于 `max_abs_speed_when_stopped` → 返回 0（已停车）
- `stop_distance < 1e-5` → 返回 `double::max`（需紧急制动）

### 5.4 `CheckStopSignOnReferenceLine`

```cpp
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& info,
                                  const std::string& stop_sign_overlap_id);
```

在参考线的 `stop_sign_overlaps()` 中查找指定 ID 的停车标志是否仍存在。

### 5.5 `CheckTrafficLightOnReferenceLine`

```cpp
bool CheckTrafficLightOnReferenceLine(const ReferenceLineInfo& info,
                                      const std::string& traffic_light_overlap_id);
```

在参考线的 `signal_overlaps()` 中查找指定 ID 的交通灯是否仍存在。

### 5.6 `CheckInsideJunction`

```cpp
bool CheckInsideJunction(const ReferenceLineInfo& reference_line_info);
```

**逻辑**：
1. 获取 ADC 前边缘 s 坐标处的 junction overlap
2. 计算 ADC 后边缘到 junction 终点的距离
3. 若距离 < `kIntersectionPassDist`（2.0m），认为仍在路口内

### 5.7 `GetFilesByPath`

```cpp
void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files);
```

递归遍历目录，收集所有常规文件路径。

### 5.8 `CalculateEquivalentEgoWidth`

```cpp
double CalculateEquivalentEgoWidth(const ReferenceLineInfo& info,
                                   double s, bool* is_left);
```

计算车辆在弯道中的**等效占道宽度**。弯道中车辆实际扫过的横向范围大于车宽，此函数基于前后轴的曲率差异计算等效宽度，用于路径边界约束。

### 5.9 弧形边界计算函数族

```cpp
bool left_arc_bound_with_heading(double delta_x, double r, double heading, double* result);
bool right_arc_bound_with_heading(double delta_x, double r, double heading, double* result);
bool left_arc_bound_with_heading_with_reverse_kappa(...);
bool right_arc_bound_with_heading_with_reverse_kappa(...);
```

计算给定半径圆弧在指定横向偏移 `delta_x` 处的纵向边界值。用于路径边界的几何约束计算。

## 6. MathUtil — 坐标变换

源码：`math_util.h` 与 `math_util.cc`

### 6.1 `WorldCoordToObjCoord`

```cpp
std::pair<double, double> WorldCoordToObjCoord(
    std::pair<double, double> input_world_coord,
    std::pair<double, double> obj_world_coord,
    double obj_world_angle);
```

**算法**：
1. 计算世界坐标差：`(x_diff, y_diff)`
2. 计算极坐标：`rho = sqrt(x² + y²)`, `theta = atan2(y, x) - obj_angle`
3. 转回直角坐标：`(cos(theta) * rho, sin(theta) * rho)`

**用途**：将世界坐标系中的点转换到障碍物局部坐标系。

### 6.2 `WorldAngleToObjAngle`

```cpp
double WorldAngleToObjAngle(double input_world_angle, double obj_world_angle);
```

将世界坐标系角度转换为障碍物局部坐标系角度，结果归一化到 `[-π, π]`。

## 7. PrintDebugInfo — 调试可视化

源码：`print_debug_info.h` 与 `print_debug_info.cc`

### 7.1 PrintPoints

```cpp
class PrintPoints {
  void set_id(std::string id);
  void AddPoint(double x, double y);
  void PrintToLog();
};
```

收集二维点序列，通过 `AINFO` 输出格式化字符串 `print_<id>:(x1, y1);(x2, y2);...`。受 `FLAGS_enable_print_curve` 控制。

### 7.2 PrintCurves

```cpp
class PrintCurves {
  void AddPoint(std::string key, double x, double y);
  void AddPoint(std::string key, const Vec2d& point);
  void AddPoint(std::string key, const std::vector<Vec2d>& points);
  void PrintToLog();
};
```

按 key 分组管理多条曲线，内部使用 `map<string, PrintPoints>`。

### 7.3 PrintBox

```cpp
class PrintBox {
  void AddAdcBox(double x, double y, double heading, bool is_rear_axle_point = true);
  void PrintToLog();
};
```

记录车辆包围盒（x, y, heading, length, width）。若输入为后轴中心点，自动转换为几何中心。

## 8. EvaluatorLogger — 评估日志

源码：`evaluator_logger.h`

```cpp
class EvaluatorLogger {
 public:
  static std::ofstream& GetStream();
};
```

**设计**：Meyer's Singleton，返回追加模式的文件流，路径为 `FLAGS_planning_data_dir + "/output_data_evaluated.log"`。用于学习型规划组件输出评估数据。
