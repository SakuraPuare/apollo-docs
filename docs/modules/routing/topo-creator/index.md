---
title: "拓扑图生成器"
---

# Topo Creator

> 源码路径：`modules/routing/topo_creator/`

## 概述

Topo Creator 是一个离线工具，负责将高精地图（HD Map）转换为路由拓扑图（Routing Topo Graph）。它读取 base map 文件，为每条可行驶车道创建拓扑节点（Node），并根据车道间的前后继关系和左右邻接关系创建拓扑边（Edge），最终将生成的拓扑图序列化为 protobuf 二进制文件和文本文件，供在线路由模块使用。

## 核心类

### GraphCreator

拓扑图生成的核心类，负责协调整个图的构建流程。

```cpp
class GraphCreator {
 public:
  GraphCreator(const std::string& base_map_file_path,
               const std::string& dump_topo_file_path,
               const RoutingConfig& routing_conf);
  ~GraphCreator() = default;
  bool Create();

 private:
  void InitForbiddenLanes();
  std::string GetEdgeID(const std::string& from_id, const std::string& to_id);
  void AddEdge(
      const Node& from_node,
      const ::google::protobuf::RepeatedPtrField<hdmap::Id>& to_node_vec,
      const Edge::DirectionType& type);
  static bool IsValidUTurn(const hdmap::Lane& lane, const double radius);

 private:
  std::string base_map_file_path_;
  std::string dump_topo_file_path_;
  hdmap::Map pbmap_;
  Graph graph_;
  std::unordered_map<std::string, int> node_index_map_;
  std::unordered_map<std::string, std::string> road_id_map_;
  std::unordered_set<std::string> showed_edge_id_set_;
  std::unordered_set<std::string> forbidden_lane_id_set_;
  const RoutingConfig& routing_conf_;
};
```

### node_creator 命名空间

```cpp
namespace node_creator {
void GetPbNode(const hdmap::Lane& lane, const std::string& road_id,
               const RoutingConfig& routingconfig, Node* const node);
}
```

### edge_creator 命名空间

```cpp
namespace edge_creator {
void GetPbEdge(const Node& node_from, const Node& node_to,
               const Edge::DirectionType& type,
               const RoutingConfig& routingconfig, Edge* pb_edge);
}
```

## 核心函数

### GraphCreator::Create

主流程函数，执行以下步骤：

1. 加载 base map（支持 protobuf 和 OpenDrive XML 格式）
2. 建立 lane_id 到 road_id 的映射
3. 调用 `InitForbiddenLanes()` 过滤非 `CITY_DRIVING` 类型车道
4. 遍历所有车道，过滤掉禁止车道和转弯半径不足的 U-Turn 车道，为合法车道创建 Node
5. 再次遍历车道，根据前后继关系创建 FORWARD 边，根据左右邻接关系创建 LEFT/RIGHT 边
6. 将拓扑图序列化为 `.bin` 和 `.txt` 文件

```cpp
bool GraphCreator::Create() {
  // 加载地图（支持 .xml OpenDrive 或 protobuf 格式）
  if (absl::EndsWith(base_map_file_path_, ".xml")) {
    hdmap::adapter::OpendriveAdapter::LoadData(base_map_file_path_, &pbmap_);
  } else {
    cyber::common::GetProtoFromFile(base_map_file_path_, &pbmap_);
  }
  // 构建 road_id 映射
  for (const auto& road : pbmap_.road()) { ... }
  // 创建节点
  InitForbiddenLanes();
  for (const auto& lane : pbmap_.lane()) {
    node_creator::GetPbNode(lane, road_id, routing_conf_, graph_.add_node());
  }
  // 创建边
  for (const auto& lane : pbmap_.lane()) {
    AddEdge(from_node, lane.successor_id(), Edge::FORWARD);
    AddEdge(from_node, lane.left_neighbor_forward_lane_id(), Edge::LEFT);
    AddEdge(from_node, lane.right_neighbor_forward_lane_id(), Edge::RIGHT);
  }
  // 输出 bin 和 txt 文件
  cyber::common::SetProtoToBinaryFile(graph_, bin_file);
  cyber::common::SetProtoToASCIIFile(graph_, txt_file);
}
```

### node_creator::GetPbNode

为单条车道创建拓扑节点，包含两部分逻辑：

- **InitNodeInfo**：设置 lane_id、road_id、中心线、长度，并根据左右车道线类型（虚线允许变道）计算 `left_out` / `right_out` 可变道区间。若车道属于 junction 且无左右邻居则标记为 `is_virtual`。
- **InitNodeCost**：基于车道长度和限速计算通行代价，对左转、右转、U-Turn 分别施加额外惩罚。

```cpp
void InitNodeCost(const Lane& lane, const RoutingConfig& routing_config,
                  Node* const node) {
  double speed_limit = lane.has_speed_limit() ? lane.speed_limit()
                                              : routing_config.base_speed();
  double ratio = speed_limit >= routing_config.base_speed()
                     ? std::sqrt(routing_config.base_speed() / speed_limit)
                     : 1.0;
  double cost = lane_length * ratio;
  // 转弯惩罚
  if (lane.turn() == Lane::LEFT_TURN)
    cost += routing_config.left_turn_penalty();
  else if (lane.turn() == Lane::RIGHT_TURN)
    cost += routing_config.right_turn_penalty();
  else if (lane.turn() == Lane::U_TURN)
    cost += routing_config.uturn_penalty();
  node->set_cost(cost);
}
```

### edge_creator::GetPbEdge

创建拓扑边并计算变道代价。FORWARD 边代价为 0；LEFT/RIGHT 边根据可变道区间长度与 `base_changing_length` 的比值计算惩罚系数：

```cpp
void GetPbEdge(const Node& node_from, const Node& node_to,
               const Edge::DirectionType& type,
               const RoutingConfig& routing_config, Edge* edge) {
  edge->set_from_lane_id(node_from.lane_id());
  edge->set_to_lane_id(node_to.lane_id());
  edge->set_direction_type(type);
  edge->set_cost(0.0);
  if (type == Edge::LEFT || type == Edge::RIGHT) {
    // 可变道区间越短，惩罚越大（ratio^-1.5）
    double ratio = std::pow(
        changing_area_length / routing_config.base_changing_length(), -1.5);
    edge->set_cost(routing_config.change_penalty() * ratio);
  }
}
```

### GraphCreator::IsValidUTurn

静态方法，通过车道中心线的起点、中点和终点三点拟合圆弧，判断 U-Turn 车道的转弯半径是否满足车辆最小转弯半径要求。

## 调用关系

**上游调用者：**

- `topo_creator.cc`（main 函数）：加载 `RoutingConfig`，获取 base_map 和 routing_map 路径，实例化 `GraphCreator` 并调用 `Create()`

**内部调用：**

- `GraphCreator::Create()` 调用 `node_creator::GetPbNode()` 创建节点
- `GraphCreator::AddEdge()` 调用 `edge_creator::GetPbEdge()` 创建边
- `GraphCreator::Create()` 调用 `InitForbiddenLanes()` 和 `IsValidUTurn()` 进行过滤

**下游依赖：**

- 生成的拓扑图文件被 `modules/routing/core/` 中的在线路由模块加载使用
- 依赖 `modules/map/hdmap/` 提供的高精地图数据
- 依赖 `modules/common/configs/vehicle_config_helper.h` 获取车辆最小转弯半径
