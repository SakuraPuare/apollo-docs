---
title: "路由核心逻辑"
---

# Core

> 源码路径：`modules/routing/core/`

## 概述

Core 模块是 Routing 的核心搜索引擎，负责接收 RoutingRequest、在拓扑图上执行 A* 搜索、生成黑名单区域以避开禁行路段，并将搜索结果转换为包含 Passage Region 的 RoutingResponse。

## 核心类

### Navigator

路由导航器，整合黑名单生成与结果生成，对外提供 `SearchRoute` 接口。

```cpp
class Navigator {
 public:
  explicit Navigator(const std::string& topo_file_path);
  bool IsReady() const;
  bool SearchRoute(const routing::RoutingRequest& request,
                   routing::RoutingResponse* const response);
 private:
  std::unique_ptr<TopoGraph> graph_;
  TopoRangeManager topo_range_manager_;
  std::unique_ptr<BlackListRangeGenerator> black_list_generator_;
  std::unique_ptr<ResultGenerator> result_generator_;
};
```

### BlackListRangeGenerator

根据 RoutingRequest 中的黑名单车道/道路以及起终点信息，生成需要屏蔽的拓扑范围。

```cpp
class BlackListRangeGenerator {
 public:
  void GenerateBlackMapFromRequest(const routing::RoutingRequest& request,
                                   const TopoGraph* graph,
                                   TopoRangeManager* const range_manager) const;
  void AddBlackMapFromTerminal(const TopoNode* src_node,
                               const TopoNode* dest_node, double start_s,
                               double end_s,
                               TopoRangeManager* const range_manager) const;
};
```

### ResultGenerator

将搜索得到的 NodeWithRange 序列转换为 RoutingResponse 中的 RoadSegment 和 Passage 结构。

```cpp
class ResultGenerator {
 public:
  bool GeneratePassageRegion(const std::string& map_version,
                             const routing::RoutingRequest& request,
                             const std::vector<NodeWithRange>& nodes,
                             const TopoRangeManager& range_manager,
                             routing::RoutingResponse* const result);
};
```

## 核心函数

### Navigator::SearchRoute()

**职责**：处理一次完整的路由搜索请求，返回 RoutingResponse。

**关键步骤**：

1. 校验请求中的 waypoint 和 blacklisted_lane 是否存在于拓扑图中
2. 调用 `Init()` 解析途经点并生成全局黑名单
3. 调用 `SearchRouteByStrategy()` 执行分段 A* 搜索
4. 设置结果首尾节点的精确 s 值
5. 调用 `ResultGenerator::GeneratePassageRegion()` 生成最终响应

### Navigator::SearchRouteByStrategy()

**职责**：对相邻 waypoint 逐段执行 A* 搜索并合并结果。

**关键步骤**：

1. 创建 `AStarStrategy` 实例（根据 `FLAGS_enable_change_lane_in_result` 配置）
2. 对每对相邻 waypoint，调用 `AddBlackMapFromTerminal()` 生成局部黑名单
3. 构建 `SubTopoGraph`，获取起终点子节点
4. 执行 `strategy_ptr->Search()` 得到当前段路径
5. 调用 `MergeRoute()` 合并所有段的结果，确保连续性

### BlackListRangeGenerator::GenerateBlackMapFromRequest()

**职责**：从请求中提取黑名单车道和道路，加入 TopoRangeManager。

**关键步骤**：

1. 遍历 `blacklisted_lane`，按 lane id 查找节点并添加对应 s 范围
2. 遍历 `blacklisted_road`，获取该 road 下所有节点并全段屏蔽
3. 调用 `SortAndMerge()` 合并重叠区间

### BlackListRangeGenerator::AddBlackMapFromTerminal()

**职责**：根据起终点位置，屏蔽起点之前和终点之后的区域（含平行车道）。

**关键步骤**：

1. 对 start_s 向后偏移 `S_GAP_FOR_BLACK`（0.01m），屏蔽起点前方区域
2. 递归查找起点的出方向平行车道，按比例屏蔽
3. 对 end_s 向前偏移，屏蔽终点后方区域
4. 递归查找终点的入方向平行车道，按比例屏蔽

### ResultGenerator::GeneratePassageRegion()

**职责**：将路径节点序列转换为 RoadSegment/Passage 结构。

**关键步骤**：

1. 调用 `ExtractBasicPassages()` 按换道边分割节点为多个 Passage
2. 调用 `ExtendPassages()` 向前后扩展可换道区域
3. 调用 `CreateRoadSegments()` 按换道可达性划分 RoadSegment

## 配置

- `FLAGS_enable_change_lane_in_result`：是否在搜索结果中允许换道（影响 A* 策略）
- 拓扑文件路径：构造 Navigator 时传入，通常为 `routing_map.bin`

## 调用关系

**上游调用者**：

- `RoutingComponent` 创建 Navigator 实例并调用 `SearchRoute()`

**内部调用链**：

- `Navigator` → `BlackListRangeGenerator`（生成黑名单）
- `Navigator` → `AStarStrategy`（执行图搜索）
- `Navigator` → `ResultGenerator`（生成响应）

**下游依赖**：

- `modules/routing/graph/`：TopoGraph、SubTopoGraph、TopoRangeManager、NodeWithRange
- `modules/routing/strategy/`：AStarStrategy
