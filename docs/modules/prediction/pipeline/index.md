---
title: "预测流水线"
---

# Pipeline

> 源码路径: `modules/prediction/pipeline/`

## 概述

预测流水线模块负责将高精地图（HD Map）中的矢量化地理要素转换为 `VectorNet` 模型所需的特征向量。核心思想是将道路、车道边界、交叉路口、人行横道等地图元素编码为等间距采样的折线（polyline），每条折线由起点、终点坐标以及属性标签组成，并在目标障碍物的局部坐标系下进行旋转变换，最终输出可用于深度学习模型训练或推理的特征数据。

该目录包含四个独立的可执行程序：

- **VectorNet 核心类**（`vector_net.h` / `vector_net.cc`）— 特征提取的核心逻辑
- **records_to_offline_data**（`records_to_offline_data.cc`）— 从录制包批量生成训练数据
- **vector_net_feature**（`vector_net_feature.cc`）— 单次 VectorNet 特征查询工具
- **vector_net_offline_data**（`vector_net_offline_data.cc`）— 批量离线特征提取工具

## 核心类

### VectorNet

定义在 `vector_net.h`，是整个流水线的核心类，负责从 HD Map 中提取各类地理要素并转换为特征向量。

```cpp
class VectorNet {
 public:
  VectorNet() { apollo::hdmap::HDMapUtil::ReloadMaps(); }
  ~VectorNet() = default;

  bool query(const common::PointENU& center_point, const double obstacle_phi,
             FeatureVector* const feature_ptr, PidVector* const p_id_ptr);

  bool offline_query(const double obstacle_x, const double obstacle_y,
                     const double obstacle_phi);

  bool offline_query(const double obstacle_x, const double obstacle_y,
                     const double obstacle_phi, const std::string file_name);
};
```

关键类型别名：

```cpp
using FeatureVector = std::vector<std::vector<std::vector<double>>>;
using PidVector = std::vector<std::vector<double>>;
```

`FeatureVector` 为三层嵌套结构：外层为折线集合，中层为单条折线内的向量序列，内层为单个向量的特征元素（起点 x/y、终点 x/y、保留位、属性值、边界类型、索引）。`PidVector` 记录每条折线的最小坐标标识（p_id_x, p_id_y）。

### 枚举类型

```cpp
enum ATTRIBUTE_TYPE {
  ROAD, LANE_UNKOWN, LANE_DOTTED_YELLOW, LANE_DOTTED_WHITE,
  LANE_SOLID_YELLOW, LANE_SOLID_WHITE, LANE_DOUBLE_YELLOW,
  LANE_CURB, JUNCTION, CROSSWALK,
};

enum BOUNDARY_TYPE {
  UNKNOW, NORMAL, LEFT_BOUNDARY, RIGHT_BOUNDARY,
};
```

`ATTRIBUTE_TYPE` 对应地图要素类型，`BOUNDARY_TYPE` 对应边界位置关系。两者分别通过 `attribute_map` 和 `boundary_map` 映射为浮点数值，用于特征编码。

## 核心函数

### GetOnePolyline

```cpp
template <typename Points>
void GetOnePolyline(const Points& points, double* start_length,
                    const common::PointENU& center_point,
                    const double obstacle_phi, ATTRIBUTE_TYPE attr_type,
                    BOUNDARY_TYPE bound_type, const int count,
                    std::vector<std::vector<double>>* const one_polyline,
                    std::vector<double>* const one_p_id);
```

将一组有序点转换为等间距采样的折线向量。核心处理流程：

1. 计算输入点序列的累积弧长 `s`
2. 以 `FLAGS_point_distance` 为间隔，通过线性插值（`lerp`）在弧长上等间距采样
3. 将采样点从世界坐标转换到以障碍物为中心、朝向为 `obstacle_phi` 的局部坐标系（旋转 `PI/2 - phi`）
4. 对每对相邻采样点生成特征向量 `[d_s_x, d_s_y, d_e_x, d_e_y, 0, 0, attr, bound, count]`
5. 同时维护每条折线的最小坐标 `p_id`

### query

```cpp
bool query(const common::PointENU& center_point, const double obstacle_phi,
           FeatureVector* const feature_ptr, PidVector* const p_id_ptr);
```

主查询接口，依次调用以下四个子函数提取各类地图要素：

| 函数 | 提取要素 | 说明 |
|------|----------|------|
| `GetRoads` | 道路边界 | 遍历 `FLAGS_road_distance` 范围内道路的外边界多边形，区分左/右/普通边界 |
| `GetLanes` | 车道边界 | 先通过 `GetLaneQueue` 将离散车道段按前后继关系串联为队列，再分别提取左/右边界折线 |
| `GetJunctions` | 交叉路口 | 提取路口多边形轮廓 |
| `GetCrosswalks` | 人行横道 | 提取人行横道多边形轮廓 |

### GetLaneQueue

```cpp
void GetLaneQueue(
    const std::vector<hdmap::LaneInfoConstPtr>& lanes,
    std::vector<std::deque<hdmap::LaneInfoConstPtr>>* const lane_deque_ptr);
```

将查询范围内的离散车道段按拓扑关系（前驱/后继）串联为有序队列，确保跨多段的车道边界可以作为连续折线处理。

### offline_query

```cpp
bool offline_query(const double obstacle_x, const double obstacle_y,
                   const double obstacle_phi, const std::string file_name);
```

离线特征提取接口。内部调用 `query` 获取特征后，将结果序列化为 `VectorNetFeature` protobuf 并写入 ASCII 文件，便于后续训练流程使用。

## 配置

模块行为受以下 gflags 控制：

| Flag | 用途 |
|------|------|
| `FLAGS_point_distance` | 折线等间距采样的间隔距离 |
| `FLAGS_road_distance` | 查询中心点周围的道路要素搜索半径 |
| `FLAGS_prediction_target_file` | 默认离线特征输出文件路径 |
| `FLAGS_prediction_target_dir` | 批量离线特征输出目录 |
| `FLAGS_world_coordinate_file` | 世界坐标输入文件（Protobuf 格式） |
| `FLAGS_prediction_offline_bags` | 离线录制包路径列表（冒号分隔） |
| `FLAGS_prediction_conf_file` | 预测模块配置文件路径 |

## 调用关系

```text
records_to_offline_data.cc (main)
  └─ GenerateDataForLearning()
       ├─ ContainerManager / EvaluatorManager / PredictorManager 初始化
       └─ MessageProcess::ProcessOfflineData()  // 逐包处理录制数据

vector_net_feature.cc (main)
  └─ VectorNet::offline_query(x, y, phi)       // 单次查询，输出到默认文件
       └─ VectorNet::query()
            ├─ GetRoads()
            │    └─ GetOnePolyline()
            ├─ GetLanes()
            │    ├─ GetLaneQueue()
            │    └─ GetOnePolyline()
            ├─ GetJunctions()
            │    └─ GetOnePolyline()
            └─ GetCrosswalks()
                 └─ GetOnePolyline()

vector_net_offline_data.cc (main)
  └─ 读取 WorldCoord protobuf
       └─ VectorNet::offline_query(x, y, phi, file_name)  // 逐条记录批量输出
```
