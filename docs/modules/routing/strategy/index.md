---
title: "路由搜索策略"
---

# Routing Strategy

> 源码路径：`modules/routing/strategy/`

## 概述

Strategy 模块定义了路由图上的路径搜索策略接口及其 A\* 实现。Navigator 调用该模块在拓扑子图上搜索从起点到终点的最优路径，支持换道决策。

## 核心类

### Strategy

抽象基类，定义搜索接口。

```cpp
class Strategy {
 public:
  virtual ~Strategy() {}
  virtual bool Search(const TopoGraph* graph, const SubTopoGraph* sub_graph,
                      const TopoNode* src_node, const TopoNode* dest_node,
                      std::vector<NodeWithRange>* const result_nodes) = 0;
};
```

### AStarStrategy

A\* 算法实现，支持换道搜索。

```cpp
class AStarStrategy : public Strategy {
 public:
  explicit AStarStrategy(bool enable_change);
  bool Search(const TopoGraph* graph, const SubTopoGraph* sub_graph,
              const TopoNode* src_node, const TopoNode* dest_node,
              std::vector<NodeWithRange>* const result_nodes) override;
 private:
  void Clear();
  double HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node);
  double GetResidualS(const TopoNode* node);
  double GetResidualS(const TopoEdge* edge, const TopoNode* to_node);

  bool change_lane_enabled_;
  std::unordered_set<const TopoNode*> open_set_;
  std::unordered_set<const TopoNode*> closed_set_;
  std::unordered_map<const TopoNode*, const TopoNode*> came_from_;
  std::unordered_map<const TopoNode*, double> g_score_;
  std::unordered_map<const TopoNode*, double> enter_s_;
};
```

## 核心函数

### AStarStrategy::Search()

```cpp
bool AStarStrategy::Search(const TopoGraph* graph,
                           const SubTopoGraph* sub_graph,
                           const TopoNode* src_node,
                           const TopoNode* dest_node,
                           std::vector<NodeWithRange>* const result_nodes) {
  Clear();
  open_set_.insert(src_node);
  g_score_[src_node] = 0.0;
  enter_s_[src_node] = src_node->StartS();
  SearchNode src_search_node(src_node);
  src_search_node.f = HeuristicCost(src_node, dest_node);
  std::priority_queue<SearchNode> open_set_detail;
  open_set_detail.push(src_search_node);
  // ... A* 主循环：取 f 最小节点展开邻居 ...
}
```

**职责**：在拓扑子图上执行 A\* 搜索，找到从 src_node 到 dest_node 的最短路径

**输入**：

- `graph` — 完整拓扑图
- `sub_graph` — 当前请求的子拓扑图（含黑名单约束）
- `src_node` / `dest_node` — 起终点拓扑节点

**输出**：`result_nodes` 填充路径上的节点及其 s 范围，返回是否搜索成功

**关键步骤**：

1. 初始化 open set，将起点加入优先队列
2. 循环取 f 值最小的节点，若为终点则回溯路径
3. 对当前节点的所有出边邻居计算 g_score（累计代价）
4. 若换道使能，同时考虑换道边（检查剩余 s 是否满足 `FLAGS_min_length_for_lane_change`）
5. 搜索完成后调用 `AdjustLaneChange` 优化换道位置

### AStarStrategy::HeuristicCost()

```cpp
double AStarStrategy::HeuristicCost(const TopoNode* src_node,
                                    const TopoNode* dest_node) {
  const auto& src_point = src_node->AnchorPoint();
  const auto& dest_point = dest_node->AnchorPoint();
  double distance = std::fabs(src_point.x() - dest_point.x()) +
                    std::fabs(src_point.y() - dest_point.y());
  return distance;
}
```

**职责**：计算启发式代价（曼哈顿距离）

### AStarStrategy::GetResidualS()

**职责**：计算节点或换道边的剩余可行驶距离，用于判断是否有足够空间完成换道

**关键逻辑**：从 `enter_s_` 记录的进入位置开始，计算到节点末端（含后继同车道节点）的距离

## 辅助函数

### SearchNode（内部结构体）

优先队列元素，按 f 值升序排列（重载 `operator<` 为大于比较以实现最小堆）。

### AdjustLaneChange()

路径后处理：将换道动作向前或向后调整到更大的车道段上执行，确保换道发生在有足够空间的位置。

## 调用关系

- **被调用方**：`Navigator::SearchRoute()` 创建 `AStarStrategy` 实例并调用 `Search()`
- **依赖**：`TopoGraph`、`SubTopoGraph`、`TopoNode`、`TopoEdge`（来自 `routing/graph/`）
- **配置依赖**：`FLAGS_min_length_for_lane_change`（来自 `routing/common/routing_gflags`）
