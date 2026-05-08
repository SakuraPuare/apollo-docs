---
title: "路由拓扑图"
---

# Graph

> 源码路径：`modules/routing/graph/`

## 概述

Graph 模块是路由系统的拓扑图数据结构层，负责将 HD Map 生成的拓扑文件加载为内存中的有向图结构。它提供节点（车道）、边（车道连接关系）、S 坐标范围等核心抽象，供路由搜索算法（A* 等）使用。同时支持通过黑名单区域将原始节点拆分为子节点，构建 SubTopoGraph 以实现动态避障路由。

## 核心类

### NodeSRange

表示节点上的一段纵向 S 坐标范围。

```cpp
class NodeSRange {
 public:
  NodeSRange(double s1, double s2);
  bool IsValid() const;
  double StartS() const;
  double EndS() const;
  double Length() const;
  bool IsEnoughForChangeLane() const;
  bool MergeRangeOverlap(const NodeSRange& other);
  void SetStartS(double start_s);
  void SetEndS(double end_s);
};
```

### TopoNode

拓扑图节点，对应一条车道（Lane）。包含车道 ID、所属道路 ID、长度、代价、中心曲线，以及入边/出边集合。

```cpp
class TopoNode {
 public:
  explicit TopoNode(const Node& node);
  TopoNode(const TopoNode* topo_node, const NodeSRange& range); // 子节点构造

  const std::string& LaneId() const;
  const std::string& RoadId() const;
  double Length() const;
  double Cost() const;
  bool IsVirtual() const;

  const std::unordered_set<const TopoEdge*>& OutToSucEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToLeftEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToRightEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromPreEdge() const;

  const TopoEdge* GetOutEdgeTo(const TopoNode* to_node) const;
  void AddInEdge(const TopoEdge* edge);
  void AddOutEdge(const TopoEdge* edge);
};
```

### TopoEdge

拓扑图边，表示两个车道节点之间的连接关系（前向/左换道/右换道）。

```cpp
enum TopoEdgeType { TET_FORWARD, TET_LEFT, TET_RIGHT };

class TopoEdge {
 public:
  TopoEdge(const Edge& edge, const TopoNode* from_node, const TopoNode* to_node);
  double Cost() const;
  TopoEdgeType Type() const;
  const TopoNode* FromNode() const;
  const TopoNode* ToNode() const;
};
```

### TopoGraph

顶层拓扑图容器，从 protobuf Graph 对象加载所有节点和边。

```cpp
class TopoGraph {
 public:
  bool LoadGraph(const Graph& graph);
  const TopoNode* GetNode(const std::string& id) const;
  void GetNodesByRoadId(const std::string& road_id,
                        std::unordered_set<const TopoNode*>* node_in_road) const;
  const std::string& MapVersion() const;
  const std::string& MapDistrict() const;
};
```

### TopoRangeManager

管理节点上的黑名单 S 范围（不可通行区域），支持添加、排序合并操作。

```cpp
class TopoRangeManager {
 public:
  void Add(const TopoNode* node, double start_s, double end_s);
  void SortAndMerge();
  const std::vector<NodeSRange>* Find(const TopoNode* node) const;
  void Clear();
};
```

### SubTopoGraph

子拓扑图，根据黑名单范围将原始节点拆分为多个子节点，并重建边连接关系。

```cpp
class SubTopoGraph {
 public:
  SubTopoGraph(const std::unordered_map<const TopoNode*,
               std::vector<NodeSRange>>& black_map);
  void GetSubInEdgesIntoSubGraph(const TopoEdge* edge,
      std::unordered_set<const TopoEdge*>* sub_edges) const;
  void GetSubOutEdgesIntoSubGraph(const TopoEdge* edge,
      std::unordered_set<const TopoEdge*>* sub_edges) const;
  const TopoNode* GetSubNodeWithS(const TopoNode* topo_node, double s) const;
};
```

### NodeWithRange

继承自 NodeSRange，将 TopoNode 指针与 S 范围绑定，用于子图中表示节点片段。

```cpp
class NodeWithRange : public NodeSRange {
 public:
  NodeWithRange(const TopoNode* node, double start_s, double end_s);
  const TopoNode* GetTopoNode() const;
  const std::string& LaneId() const;
  const std::string& RoadId() const;
};
```

## 核心函数

### TopoGraph::LoadGraph()

**职责**：从 protobuf Graph 对象加载完整拓扑图。
**关键步骤**：

1. 清空已有数据，读取地图版本和区域信息
2. 调用 `LoadNodes()` 遍历所有 Node，创建 TopoNode 并建立 lane_id 到索引的映射
3. 调用 `LoadEdges()` 遍历所有 Edge，根据 from/to lane_id 查找对应节点，创建 TopoEdge 并注册到节点的入边/出边集合

### SubTopoGraph 构造函数

**职责**：根据黑名单范围拆分节点并重建子图。
**关键步骤**：

1. 对每个黑名单节点，计算有效通行范围（排除黑名单区间）
2. 调用 `InitSubNodeByValidRange()` 为每段有效范围创建子节点
3. 调用 `InitSubEdge()` 为子节点重建前向边和换道边
4. 调用 `AddPotentialEdge()` 补充跨子节点的潜在换道边

### range_utils.h 二分查找工具

**职责**：在已排序的 NodeSRange 向量中高效定位包含指定 S 值的区间。
**关键步骤**：

1. `BinarySearchForSLarger()` — 查找 StartS 大于等于目标值的区间
2. `BinarySearchForSSmaller()` — 查找 EndS 小于等于目标值的区间
3. `BinarySearchCheckValidSIndex()` — 验证找到的索引是否真正包含目标 S 值（容差 0.02m）

## 调用关系

**被调用方**（上游）：

- `routing/core/Navigator` 持有 TopoGraph 实例，在路由请求时加载并查询节点
- `routing/strategy/AStarStrategy` 在搜索过程中遍历 TopoNode 的出边集合
- `routing/core/BlackListRangeGenerator` 使用 TopoRangeManager 收集黑名单范围，传入 SubTopoGraph

**调用方**（下游）：

- 依赖 `modules/routing/proto/topo_graph.pb.h` 提供的 Graph/Node/Edge protobuf 消息
- 依赖 `modules/common/math` 和 `hdmap::Curve` 提供几何数据
