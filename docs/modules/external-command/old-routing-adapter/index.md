---
title: "旧路由适配器"
---

# Old Routing Adapter

> 源码路径：`modules/external_command/old_routing_adapter/`

## 概述

旧路由请求适配器，将 Apollo 旧版 `RoutingRequest` 消息转换为新版外部命令（`LaneFollowCommand` 或 `ValetParkingCommand`），实现向后兼容。

## 核心类

### OldRoutingAdapter

```cpp
class OldRoutingAdapter : public cyber::Component<> {
 public:
  bool Init() override;
 private:
  void OnRoutingRequest(
      const std::shared_ptr<routing::RoutingRequest>& routing_request);
  template <typename T>
  void CopyRoutingRequest(
      const std::shared_ptr<routing::RoutingRequest>& routing_request,
      int start_index, int end_index, T* message);
  void Convert(const routing::LaneWaypoint& lane_way_point,
               external_command::Pose* pose) const;
  double GetNearestHeading(const std::string& lane_id, double x, double y) const;
};
```

## 核心函数

### OldRoutingAdapter::OnRoutingRequest()

**职责**：接收旧版 `RoutingRequest`，根据是否包含 `parking_id` 决定转换为 `LaneFollowCommand` 或 `ValetParkingCommand`

**关键步骤**：

1. 判断请求中是否有 `parking_info.parking_space_id`
2. 有泊车 ID → 构建 `ValetParkingCommand` 发送到泊车服务
3. 无泊车 ID → 构建 `LaneFollowCommand` 发送到车道跟随服务

### OldRoutingAdapter::Convert()

**职责**：将 `LaneWaypoint`（lane_id + s）转换为 `Pose`（x, y, heading），通过 HDMap 查询坐标

## 调用关系

- **输入**：订阅旧版 `RoutingRequest` 话题
- **输出**：通过 Service Client 调用 `LaneFollowCommandProcessor` 或 `ValetParkingCommandProcessor`
- **依赖**：HDMap（坐标转换）
