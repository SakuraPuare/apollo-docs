---
title: "地图工具"
---
# 地图工具

> 源码路径: `modules/map/tools/`

## 概述

`map/tools` 模块提供地图数据的离线处理与质量检测工具集，分为两部分：

- **独立 CLI 工具**（7 个）：地图格式转换、坐标查询、四元数转换、默认路径点刷新等
- **map\_datachecker 子系统**：基于 gRPC 的采集数据质量校验框架（Client-Server 架构），用于验证 GNSS 位姿、通道数据和采集圈数

## 核心类

### MapUtil

`map_xysl.cc` 中定义的地图查询工具类，封装 HD Map 的坐标转换与元素查询：

```cpp
class MapUtil {
 public:
  // 根据元素类型和 ID 获取地图对象（ClearArea/Crosswalk/Junction/Lane/Signal 等）
  GET_ELEMENT_BY_ID(Lane);
  // XY 坐标 -> 最近车道的 (lane_id, s, l, heading)
  int PointToSL(const PointENU &point, std::string *lane_id,
                double *s, double *l, double *heading) const;
  // SL 坐标 -> XY 坐标
  int SLToPoint(LaneInfoConstPtr lane_ptr, double s, double l,
                PointENU *point, double *heading) const;
  // 投影点到指定车道
  int LaneProjection(const Vec2d &vec2d, const std::string &lane_id,
                     double *s, double *l, double *heading) const;
};
```

### Alignment

静态对齐与八字绕行的抽象基类（`alignment.h`）：

```cpp
class Alignment {
 public:
  virtual ErrorCode Process(const std::vector<FramePose>& poses) = 0;
  virtual void Reset() = 0;
  double GetProgress() const;
  bool IsGoodPose(const std::vector<FramePose>& poses, int pose_index);
};
```

派生类 `StaticAlign`（`static_align.h`）执行静止对齐，`EightRoute`（`eight_route.h`）执行八字绕行对齐。

### AlignmentAgent\<T\>

对齐任务的 gRPC Agent 模板类（`alignment_agent.h`），管理对齐线程的生命周期：

```cpp
template <typename ALIGNMENT_TYPE, typename REQUEST_TYPE, typename RESPONSE_TYPE>
class AlignmentAgent {
 public:
  grpc::Status ProcessGrpcRequest(grpc::ServerContext *context,
                                  REQUEST_TYPE *request, RESPONSE_TYPE *response);
  int AlignmentStart(REQUEST_TYPE *request, RESPONSE_TYPE *response);
  int AlignmentCheck(REQUEST_TYPE *request, RESPONSE_TYPE *response);
  int AlignmentStop(REQUEST_TYPE *request, RESPONSE_TYPE *response);
};
```

具体实例化：`STATIC_ALIGN_AGENT_TYPE` 和 `EIGHT_ROUTE_AGENT_TYPE`。

### MapDataCheckerAgent

gRPC 服务端主类（`worker_agent.h`），实现 `CollectionCheckerService::Service`：

```cpp
class MapDataCheckerAgent final : public CollectionCheckerService::Service {
 public:
  grpc::Status ServiceChannelVerify(...);
  grpc::Status ServiceStaticAlign(...);
  grpc::Status ServiceEightRoute(...);
  grpc::Status ServiceLoopsVerify(...);
};
```

内部持有 `PoseCollectionAgent`、`ChannelVerifyAgent`、`AlignmentAgent`、`LoopsVerifyAgent` 四个子 Agent。

### 其他关键类

| 类 | 文件 | 职责 |
|---|---|---|
| `FramePose` | `server/common.h` | 单帧位姿数据结构，含时间戳、平移、四元数、GNSS 状态等 |
| `JsonConf` | `server/common.h` | 校验参数配置，含 topic 列表、对齐阈值、圈数阈值等 |
| `PoseCollectionAgent` | `server/pose_collection_agent.h` | 订阅 GNSS 最佳定位，收集并转换位姿 |
| `ChannelVerifyAgent` | `server/channel_verify_agent.h` | 检查录制文件中的 topic 完整性和帧率 |
| `LoopsVerifyAgent` | `server/loops_verify_agent.h` | 验证采集数据是否满足圈数要求 |
| `Client` | `client/client.h` | 客户端，按阶段驱动校验流程 |
| `Mapdatachecker` | `server/worker.h` | Server 启停与报告管理 |

## 核心函数

### 独立 CLI 工具

| 入口 | 文件 | 功能 |
|---|---|---|
| `main` | `map_tool.cc` | OpenDrive XML -> Protobuf txt/bin |
| `main` | `map_xysl.cc` | 坐标查询：`--xy_to_sl`、`--sl_to_xy`、`--xy_to_lane`、`--lane_to_lane` |
| `main` | `bin_map_generator.cc` | base\_map.txt -> base\_map.bin |
| `main` | `proto_map_generator.cc` | 对地图坐标施加 XY 偏移后输出 |
| `main` | `sim_map_generator.cc` | 按角度和距离降采样生成 sim\_map |
| `main` | `quaternion_euler.cc` | 四元数 -> 欧拉角及航向角 |
| `RefreshDefaultEndPoint` | `refresh_default_end_way_point.cc` | 地图更新后刷新默认终点 |

### map\_datachecker 子系统

| 函数 | 文件 | 功能 |
| `Alignment::Process` | `server/static_align.cc` | 处理位姿序列，判断静态对齐进度 |
| `Alignment::Process` | `server/eight_route.cc` | 处理位姿序列，判断八字绕行对齐进度 |
| `ChannelVerify::Verify` | `server/channel_verify.cc` | 校验录制文件 topic 完整性和帧率 |
| `LapsChecker::Check` | `server/laps_checker.cc` | 检测采集轨迹的圈数是否达标 |
| `PoseCollection::OnBestgnsspos` | `server/pose_collection.cc` | 回调处理 GNSS 最佳定位消息 |
| `Client::Run` | `client/client.cc` | 客户端主循环：RecordCheck -> StaticAlign -> EightRoute -> DataCollect -> LoopsCheck -> Clean |
| `ParseJson` | `server/common.cc` | 解析 JSON 配置文件为 `JsonConf` |

## 配置

map\_datachecker 通过 JSON 配置文件控制校验参数（由 `JsonConf` 结构体解析）：

| 参数 | 类型 | 说明 |
|---|---|---|
| `topic_list` | `vector<pair<string,double>>` | 需校验的 topic 及期望帧率 |
| `use_system_time` | `bool` | 是否使用系统时间 |
| `topic_rate_tolerance` | `double` | 帧率容差 |
| `solution_status` | `unsigned int` | GNSS 解算状态要求 |
| `position_type_range` | `set<unsigned int>` | 允许的定位类型范围 |
| `diff_age_range` | `pair<float,float>` | 差分龄期范围 |
| `local_std_upper_limit` | `double` | 本地标准差上限 |
| `static_align_duration` | `double` | 静态对齐所需持续时间（秒） |
| `static_align_dist_thresh` | `double` | 静态对齐允许的最大运动距离 |
| `eight_angle` | `double` | 八字绕行相邻帧角度阈值（度） |
| `eight_duration` | `double` | 八字绕行所需持续时间（秒） |
| `laps_frames_thresh` | `int` | 多圈采集最小帧数阈值 |
| `laps_alpha_err_thresh` | `double` | 同轨迹相邻位姿角度误差上限（度） |
| `laps_number` | `size_t` | 需校验的圈数 |
| `laps_rate_thresh` | `double` | 采集圈校验所需点比例阈值 |

独立 CLI 工具通过 gflags 配置，常用参数：

- `--map_dir`：地图目录（`hdmap_util.h` 定义）
- `--output_dir`：输出目录
- `--x`, `--y`, `--s`, `--l`：坐标参数
- `--lane`：车道 ID
- `--angle_threshold`、`--downsample_distance`：sim\_map 降采样参数

## 调用关系

### 独立 CLI 工具

```text
map_tool.cc          --> OpendriveAdapter::LoadData() --> SetProtoToASCIIFile / SetProtoToBinaryFile
bin_map_generator.cc --> GetProtoFromFile()           --> SetProtoToBinaryFile
proto_map_generator.cc --> GetProtoFromFile()         --> ShiftMap() --> SetProtoToASCIIFile / SetProtoToBinaryFile
sim_map_generator.cc --> GetProtoFromFile / OpendriveAdapter --> DownsampleByAngle --> DownsampleByDistance --> OutputMap
map_xysl.cc          --> HDMapUtil::BaseMap()          --> GetNearestLane / GetLaneById / GetProjection
refresh_default_end_way_point.cc --> HDMapUtil::BaseMap() --> GetNearestLane --> GetSmoothPoint
```

### map\_datachecker 子系统

```text
Server:
  main (worker.cc)
    --> Mapdatachecker::Init / Start
        --> MapDataCheckerAgent (gRPC Service)
            --> PoseCollectionAgent  --> PoseCollection  (订阅 GNSS topic)
            --> AlignmentAgent<StaticAlign>             (静态对齐检查)
            --> AlignmentAgent<EightRoute>              (八字绕行检查)
            --> ChannelVerifyAgent  --> ChannelVerify   (通道完整性检查)
            --> LoopsVerifyAgent    --> LapsChecker      (圈数校验)
Client:
  main (client.cc)
    --> Client::Run()
        --> RecordCheckStage    (检查录制状态)
        --> StaticAlignStage    (静态对齐)
        --> EightRouteStage     (八字绕行)
        --> DataCollectStage    (数据采集)
        --> LoopsCheckStage     (圈数校验)
        --> CleanStage          (清理)
```

### 通信协议

Client 与 Server 通过 gRPC (`CollectionCheckerService`) 通信，支持四类服务：

- `ServiceChannelVerify`：通道数据校验
- `ServiceStaticAlign`：静态对齐
- `ServiceEightRoute`：八字绕行对齐
- `ServiceLoopsVerify`：圈数校验

每个服务支持 `START`、`CHECK`、`STOP` 三种命令类型。
