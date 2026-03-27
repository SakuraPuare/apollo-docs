# Routing 模块

> 基于拓扑地图的全局路径搜索模块

## 模块职责

Routing 模块负责根据用户给定的起点、终点（及可选途经点），在高精地图的拓扑图上执行全局路径搜索，输出一条由车道级别 passage 组成的行驶路线（`RoutingResponse`）。它是 Apollo 自动驾驶系统中 Planning 模块的上游依赖——Planning 在 Routing 给出的全局路线基础上进行局部轨迹规划。

模块的核心工作可以分为两个阶段：

1. **离线阶段（topo_creator）**：读取 HD Map 的 base_map，生成拓扑图文件（`routing_map.*`），包含节点（lane）和边（lane 间的连接关系及变道关系）。
2. **在线阶段（routing 服务）**：加载拓扑图，接收 `RoutingRequest`，使用 A\* 算法搜索最优路径，生成 `RoutingResponse`。

### 通道（Channel）

| 方向 | Channel | 消息类型 |
|------|---------|---------|
| 输入 | `/apollo/raw_routing_request` | `RoutingRequest` |
| 输出 | `/apollo/raw_routing_response` | `RoutingResponse` |
| 输出 | `/apollo/raw_rrouting_response_history` | `RoutingResponse`（定时重发） |

---

## 核心类与接口

### 组件层

| 类 | 文件 | 职责 |
|----|------|------|
| `RoutingComponent` | `routing_component.h/cc` | Cyber RT Component 入口，订阅 `RoutingRequest`，调用 `Routing::Process()`，发布 `RoutingResponse`。同时通过定时器周期性重发最近一次路由结果。 |
| `Routing` | `routing.h/cc` | 业务门面类。初始化时加载 HD Map 和 `Navigator`；处理请求时先通过 `FillLaneInfoIfMissing()` 补全 waypoint 的 lane 信息（支持仅给坐标的场景），再委托 `Navigator::SearchRoute()` 执行搜索。当 waypoint 落在多条重叠 lane 上时，会生成多个候选请求并选取最短路径。 |

### 核心搜索层

| 类 | 文件 | 职责 |
|----|------|------|
| `Navigator` | `core/navigator.h/cc` | 路径搜索的调度器。持有 `TopoGraph`、`BlackListRangeGenerator`、`ResultGenerator` 和搜索策略。`SearchRoute()` 的流程：验证请求 → 生成黑名单 → 构建 SubTopoGraph → 分段 A\* 搜索 → 合并结果 → 生成 passage region。 |
| `BlackListRangeGenerator` | `core/black_list_range_generator.h/cc` | 根据 `RoutingRequest` 中的 `blacklisted_lane` / `blacklisted_road` 以及起终点位置，生成需要屏蔽的 lane 范围（`TopoRangeManager`），用于在搜索前裁剪图。 |
| `ResultGenerator` | `core/result_generator.h/cc` | 将 A\* 搜索得到的 `NodeWithRange` 序列转换为 `RoutingResponse`。核心逻辑：提取基本 passage → 扩展相邻可变道 passage → 划分 RoadSegment。 |

### 策略层

| 类 | 文件 | 职责 |
|----|------|------|
| `Strategy` | `strategy/strategy.h` | 搜索策略的纯虚基类，定义 `Search(graph, sub_graph, src, dest, result)` 接口。 |
| `AStarStrategy` | `strategy/a_star_strategy.h/cc` | A\* 算法的具体实现。支持变道搜索（`enable_change`）。搜索完成后执行变道位置的前后调整优化。 |

### 图数据结构层

| 类 | 文件 | 职责 |
|----|------|------|
| `TopoGraph` | `graph/topo_graph.h/cc` | 全局拓扑图。从 protobuf `Graph` 加载节点和边，提供按 lane_id / road_id 查询节点的能力。 |
| `TopoNode` | `graph/topo_node.h/cc` | 拓扑节点，对应一条 lane。存储 lane 的长度、代价、中心线、左右可变道范围，以及所有入边/出边（按方向分类：前向、左变道、右变道）。支持子节点模式（`IsSubNode()`），用于黑名单裁剪后的分段表示。 |
| `TopoEdge` | `graph/topo_node.h/cc` | 拓扑边，连接两个 `TopoNode`。类型为 `TET_FORWARD`（前向连接）、`TET_LEFT`（左变道）或 `TET_RIGHT`（右变道），携带通行代价。 |
| `SubTopoGraph` | `graph/sub_topo_graph.h/cc` | 子拓扑图。根据黑名单范围将被屏蔽的 lane 拆分为多个子节点，重建子节点间的边关系，使 A\* 搜索能绕过被屏蔽区域。 |
| `NodeSRange` | `graph/topo_range.h` | 表示 lane 上的一段 s 范围 `[start_s, end_s]`，支持合并重叠区间、判断是否足够变道等操作。 |
| `NodeWithRange` | `graph/node_with_range.h` | 继承 `NodeSRange`，绑定一个 `TopoNode` 指针，表示"某条 lane 上的某一段"，是搜索结果的基本单元。 |
| `TopoRangeManager` | `graph/topo_range_manager.h/cc` | 管理多个 `TopoNode` 到 `NodeSRange` 列表的映射，提供添加、排序合并、查询功能。用于存储黑名单范围。 |

### 离线拓扑图构建

| 类/命名空间 | 文件 | 职责 |
|------------|------|------|
| `GraphCreator` | `topo_creator/graph_creator.h/cc` | 读取 HD Map base_map，遍历所有 `CITY_DRIVING` 类型的 lane，创建拓扑节点和边，输出 protobuf 格式的 `Graph`。过滤非城市道路 lane，校验 U-turn 的最小转弯半径。 |
| `node_creator` | `topo_creator/node_creator.h/cc` | 为每条 lane 生成 `Node` protobuf：设置 lane_id、road_id、长度、中心线、左右可变道范围、代价（基于速度限制和转弯惩罚）。 |
| `edge_creator` | `topo_creator/edge_creator.h/cc` | 为 lane 间的连接关系生成 `Edge` protobuf：前向边代价为 0，变道边代价与可变道区域长度成反比（区域越短，惩罚越大）。 |

---

## 算法概述

### A\* 路径搜索算法

Routing 模块使用经典的 A\* 算法在拓扑图上搜索从起点到终点的最优路径。以下是算法的详细说明。

#### 图的抽象

- **节点（Node）**：每条 HD Map lane 对应一个 `TopoNode`，携带通行代价 `cost`
- **边（Edge）**：lane 之间的连接关系对应 `TopoEdge`，分为三种类型：
  - `TET_FORWARD`：前向连接（同一 road 内的前后 lane 衔接，或路口内的连接 lane）
  - `TET_LEFT`：左变道
  - `TET_RIGHT`：右变道
- **邻居代价**：`GetCostToNeighbor(edge) = edge->Cost() + edge->ToNode()->Cost()`

#### 代价函数设计

**节点代价**（`node_creator.cc`）：

```
node_cost = lane_length * speed_ratio + turn_penalty
```

- `speed_ratio`：当 lane 限速 >= 基准速度时，`sqrt(base_speed / speed_limit)`（鼓励走高速路段）；否则为 1.0
- `turn_penalty`：左转 +50m、右转 +20m、U-turn +100m（等效距离惩罚）

**边代价**（`edge_creator.cc`）：

- 前向边：代价为 0
- 变道边：`change_penalty * ratio`，其中 `ratio = (changing_area_length / base_changing_length) ^ (-1.5)`。可变道区域越短，惩罚越大，避免在短距离内强制变道

**默认配置值**（`routing_config.pb.txt`）：

| 参数 | 值 | 含义 |
|------|----|------|
| `base_speed` | 4.167 m/s | 基准速度（约 15 km/h） |
| `left_turn_penalty` | 50.0 m | 左转等效距离惩罚 |
| `right_turn_penalty` | 20.0 m | 右转等效距离惩罚 |
| `uturn_penalty` | 100.0 m | U-turn 等效距离惩罚 |
| `change_penalty` | 500.0 m | 变道基础惩罚 |
| `base_changing_length` | 50.0 m | 变道基准长度 |

#### 启发函数

```cpp
double AStarStrategy::HeuristicCost(const TopoNode* src_node,
                                     const TopoNode* dest_node)
```

使用欧几里得距离作为启发函数——计算当前节点 `AnchorPoint` 到目标节点 `AnchorPoint` 的直线距离。这是一个可接受的（admissible）启发函数，保证 A\* 找到最优解。

#### 搜索流程

```
1. 初始化 open_set，将 src_node 加入，g_score[src] = 0
2. 循环：
   a. 从 open_set 中取出 f 值最小的节点 current（使用最小堆）
   b. 若 current == dest_node，回溯 came_from_ 构建路径，返回成功
   c. 将 current 移入 closed_set
   d. 遍历 current 的所有出边（通过 SubTopoGraph 获取，已排除黑名单区域）：
      - 跳过已在 closed_set 中的邻居
      - 若变道未启用（change_lane_enabled_ == false），跳过左/右变道边
      - 若变道边的剩余可行驶距离不足（GetResidualS），跳过
      - 计算 tentative_g = g_score[current] + GetCostToNeighbor(edge)
      - 若 tentative_g < g_score[neighbor]，更新：
        · came_from_[neighbor] = current
        · g_score[neighbor] = tentative_g
        · enter_s_[neighbor] = 记录进入该 lane 的 s 值
        · f = tentative_g + HeuristicCost(neighbor, dest)
        · 将 neighbor 加入 open_set
3. open_set 为空仍未到达终点，返回失败
```

#### 变道优化（后处理）

A\* 搜索完成后，`AStarStrategy` 会执行两步变道位置调整：

1. **`AdjustLaneChangeBackward`**：从路径末尾向前扫描，若发现变道点可以提前（即上游 lane 也能到达目标 lane），则将变道位置前移，使变道发生在更早的位置，给车辆更多准备距离。

2. **`AdjustLaneChangeForward`**：从路径开头向后扫描，若发现变道点可以延后（即下游 lane 也能从源 lane 到达），则将变道位置后移，避免不必要的提前变道。

#### 分段搜索

`Navigator::SearchRouteByStrategy()` 支持多个 waypoint。对于 N 个 waypoint，执行 N-1 次 A\* 搜索（每对相邻 waypoint 之间搜索一次），然后通过 `MergeRoute()` 合并结果，确保相邻段在衔接处的 lane 一致。

---

## Routing Graph 的构建

### 离线构建流程

拓扑图由 `topo_creator` 工具离线生成，流程如下：

```
HD Map (base_map.bin / .xml)
        │
        ▼
   GraphCreator::Create()
        │
        ├── 1. 加载地图，初始化 forbidden lanes（非 CITY_DRIVING 类型）
        │
        ├── 2. 遍历所有 lane：
        │      ├── 跳过 forbidden lane
        │      ├── 调用 node_creator::GetPbNode() 生成 Node protobuf
        │      │     ├── 设置 lane_id, road_id, length, central_curve
        │      │     ├── 解析左右边界线类型，生成可变道范围（left_out, right_out）
        │      │     │   （仅 DOTTED_YELLOW / DOTTED_WHITE 允许变道）
        │      │     ├── 标记 is_virtual（路口内无相邻 lane 的为 virtual）
        │      │     └── 计算节点代价（基于速度限制 + 转弯惩罚）
        │      └── 记录 lane_id → node_index 映射
        │
        ├── 3. 遍历所有 lane，创建边：
        │      ├── FORWARD 边：successor lane 连接
        │      ├── LEFT 边：左邻 lane 连接（需边界线允许穿越 + U-turn 半径校验）
        │      └── RIGHT 边：右邻 lane 连接（同上）
        │      边代价由 edge_creator::GetPbEdge() 计算
        │
        └── 4. 序列化 Graph protobuf → routing_map 文件
```

### 在线加载流程

```
routing_map 文件
      │
      ▼
TopoGraph::LoadGraph()
      │
      ├── LoadNodes()：为每个 Node protobuf 创建 TopoNode 对象
      │     └── TopoNode::Init()：解析左右可变道范围，查找 anchor point
      │
      └── LoadEdges()：为每个 Edge protobuf 创建 TopoEdge 对象
            └── 建立 from_node ↔ to_node 的双向引用（AddOutEdge / AddInEdge）
```

### SubTopoGraph（子图裁剪）

每次搜索请求到来时，`Navigator` 会根据黑名单范围构建 `SubTopoGraph`：

1. 对每个被黑名单覆盖的 `TopoNode`，计算有效通行范围（去除黑名单区间）
2. 将有效范围拆分为多个子节点（`SubNode`），每个子节点对应 lane 上的一段可通行区间
3. 在子节点之间重建前向边和变道边
4. 搜索时，A\* 通过 `SubTopoGraph::GetSubInEdgesIntoSubGraph()` / `GetSubOutEdgesIntoSubGraph()` 获取经过裁剪的边集合

---

## 数据流

```
                    ┌─────────────────────────────────────────────┐
                    │              RoutingComponent                │
                    │  (Cyber RT Component, 订阅/发布消息)          │
                    └──────────────────┬──────────────────────────┘
                                       │
                              RoutingRequest
                                       │
                                       ▼
                    ┌─────────────────────────────────────────────┐
                    │                 Routing                      │
                    │  FillLaneInfoIfMissing()                     │
                    │  - 通过 HDMap 补全 waypoint 的 lane_id 和 s   │
                    │  - 处理 lane 重叠，生成多个候选请求             │
                    └──────────────────┬──────────────────────────┘
                                       │
                            多个候选 RoutingRequest
                                       │
                                       ▼
                    ┌─────────────────────────────────────────────┐
                    │               Navigator                      │
                    │                                              │
                    │  1. Init()                                   │
                    │     ├── GetWayNodes(): 将 waypoint 映射到     │
                    │     │   TopoGraph 中的 TopoNode               │
                    │     ├── BlackListRangeGenerator               │
                    │     │   ::GenerateBlackMapFromRequest()       │
                    │     └── AddBlackMapFromTerminal()             │
                    │                                              │
                    │  2. SearchRouteByStrategy()                   │
                    │     ├── 构建 SubTopoGraph（裁剪黑名单区域）     │
                    │     ├── 对每对相邻 waypoint 执行 A* 搜索       │
                    │     └── MergeRoute() 合并分段结果              │
                    │                                              │
                    │  3. ResultGenerator                           │
                    │     ::GeneratePassageRegion()                 │
                    │     ├── ExtractBasicPassages(): 按变道点切分    │
                    │     ├── ExtendPassages(): 扩展可变道 passage   │
                    │     └── CreateRoadSegments(): 生成最终响应     │
                    └──────────────────┬──────────────────────────┘
                                       │
                              RoutingResponse
                                       │
                                       ▼
                    ┌─────────────────────────────────────────────┐
                    │              RoutingComponent                │
                    │  发布到 /apollo/raw_routing_response          │
                    └─────────────────────────────────────────────┘
```

### RoutingResponse 结构

搜索结果经 `ResultGenerator` 处理后，输出的 `RoutingResponse` 包含：

- **RoadSegment**：一段连续的道路区间，包含一个或多个 Passage
- **Passage**：一组可互相变道的平行 lane 段，标记变道方向（LEFT / RIGHT / FORWARD）
- **LaneSegment**：单条 lane 上的一段 `[start_s, end_s]`

这种层次结构使下游 Planning 模块能够清晰地知道在哪些区间需要变道、可以变道到哪些 lane。

---

## 配置方式

### 配置文件

**`conf/routing_config.pb.txt`**（protobuf 文本格式，对应 `RoutingConfig` message）：

```protobuf
base_speed: 4.167
left_turn_penalty: 50.0
right_turn_penalty: 20.0
uturn_penalty: 100.0
change_penalty: 500.0
base_changing_length: 50.0
topic_config {
  routing_response_topic: "/apollo/raw_routing_response"
  routing_response_history_topic: "/apollo/raw_rrouting_response_history"
}
```

**`conf/routing.conf`**（gflags 命令行参数）：

```
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--routing_conf_file=/apollo/modules/routing/conf/routing_config.pb.txt
--use_road_id=false
--min_length_for_lane_change=1.0
--enable_change_lane_in_result
```

### 关键参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `base_speed` | double | 4.167 m/s | 节点代价计算的基准速度 |
| `left_turn_penalty` | double | 50.0 m | 左转惩罚（等效距离） |
| `right_turn_penalty` | double | 20.0 m | 右转惩罚 |
| `uturn_penalty` | double | 100.0 m | U-turn 惩罚 |
| `change_penalty` | double | 500.0 m | 变道基础惩罚 |
| `base_changing_length` | double | 50.0 m | 变道惩罚的基准长度 |
| `min_length_for_lane_change` | double | 1.0 m | 变道所需的最小 lane 长度 |
| `enable_change_lane_in_result` | bool | true | 搜索结果中是否包含变道操作 |
| `routing_response_history_interval_ms` | uint32 | 1000 ms | 历史路由结果重发间隔 |

### DAG 配置

`dag/routing.dag` 定义了 Cyber RT 的组件加载方式：

- 动态库：`modules/routing/librouting_component.so`
- 组件类：`RoutingComponent`
- 输入 channel：`/apollo/raw_routing_request`（队列深度 10）

### 启动方式

```bash
# 通过 Cyber RT launch 启动
cyber_launch start modules/routing/launch/routing.launch

# 或直接通过 mainboard 启动
mainboard -d modules/routing/dag/routing.dag
```
