---
title: ServiceDiscovery 服务发现模块
---

# ServiceDiscovery 服务发现模块

## 1. 模块职责概述

ServiceDiscovery 是 Apollo Cyber RT 的服务发现子系统，负责在分布式环境中自动发现和管理拓扑元素（Node、Channel、Service）。它将整个系统的通信拓扑建模为一个有向图：

- Node 是 Writer/Reader/Server/Client 的容器，对应图中的顶点
- Channel 是 Writer 到 Reader 的边
- Service 是 Server 到 Client 的边

模块基于 eProsima Fast-RTPS 的 Participant Discovery Protocol 实现跨进程、跨主机的自动发现，无需中心化注册中心。

## 2. 核心类/接口说明

### 2.1 TopologyManager（单例）

拓扑管理的顶层入口，持有三个子管理器，并通过 RTPS Participant 监听其他进程的加入/离开。

```cpp
class TopologyManager {
public:
  using ChangeFunc = std::function<void(const ChangeMsg&)>;
  using ChangeConnection = base::Connection<const ChangeMsg&>;

  void Shutdown();

  // 注册拓扑变更监听器
  ChangeConnection AddChangeListener(const ChangeFunc& func);
  void RemoveChangeListener(const ChangeConnection& conn);

  // 获取三个子管理器
  NodeManagerPtr& node_manager();
  ChannelManagerPtr& channel_manager();
  ServiceManagerPtr& service_manager();

private:
  bool Init();
  bool CreateParticipant();
  void OnParticipantChange(const PartInfo& info);
};
```

初始化流程：
1. 创建 RTPS Participant，名称格式为 `hostname+pid`，监听端口 `11511`
2. 注册 `ParticipantListener` 监听其他 Participant 的发现/移除事件
3. 依次初始化 NodeManager、ChannelManager、ServiceManager，各自通过 `StartDiscovery` 创建独立的 RTPS Publisher/Subscriber

当检测到远程 Participant 离开时，`OnParticipantChange` 会通知三个子管理器执行 `OnTopoModuleLeave`，清理该进程注册的所有角色。

### 2.2 Manager（基类）

所有子管理器的抽象基类，封装了 RTPS 发布/订阅、角色注册/注销、变更通知的通用逻辑。

```cpp
class Manager {
public:
  bool StartDiscovery(RtpsParticipant* participant);
  void StopDiscovery();
  virtual void Shutdown();

  // 注册/注销拓扑角色
  bool Join(const RoleAttributes& attr, RoleType role, bool need_publish = true);
  bool Leave(const RoleAttributes& attr, RoleType role);

  // 监听变更
  ChangeConnection AddChangeListener(const ChangeFunc& func);
  void RemoveChangeListener(const ChangeConnection& conn);

  // 当远程进程离开时的清理回调
  virtual void OnTopoModuleLeave(const std::string& host_name, int process_id) = 0;

protected:
  virtual bool Check(const RoleAttributes& attr) = 0;   // 校验属性合法性
  virtual void Dispose(const ChangeMsg& msg) = 0;        // 处理变更消息
  virtual bool NeedPublish(const ChangeMsg& msg) const;   // 是否需要广播

  void Convert(const RoleAttributes& attr, RoleType role, OperateType opt, ChangeMsg* msg);
  void Notify(const ChangeMsg& msg);    // 触发信号通知监听者
  bool Publish(const ChangeMsg& msg);   // 通过 RTPS 广播变更
  void OnRemoteChange(const std::string& msg_str);  // 接收远程变更
  bool IsFromSameProcess(const ChangeMsg& msg);      // 过滤本进程消息
};
```

每个子管理器拥有独立的 RTPS Topic（`channel_name_`），用于广播各自类型的拓扑变更。消息序列化使用 `UnderlayMessage` 封装 protobuf 的 `ChangeMsg`。

`OnRemoteChange` 接收到远程消息后，会先过滤掉来自同一进程的消息（避免重复处理），再调用子类的 `Dispose` 方法。

### 2.3 NodeManager

管理 Node 的注册与发现，使用 `SingleValueWarehouse` 存储（每个 node_id 对应唯一 Node）。

```cpp
class NodeManager : public Manager {
public:
  bool HasNode(const std::string& node_name);
  void GetNodes(RoleAttrVec* nodes);

private:
  void DisposeJoin(const ChangeMsg& msg);   // 处理节点加入
  void DisposeLeave(const ChangeMsg& msg);  // 处理节点离开
  NodeWarehouse nodes_;  // SingleValueWarehouse
};
```

- 广播 Topic：`node_change_broadcast`
- 允许角色：`ROLE_NODE`
- 变更类型：`CHANGE_NODE`
- 重复节点检测：如果发现同名 Node 且属于本进程，会触发 `AsyncShutdown()` 终止进程

### 2.4 ChannelManager

管理 Channel 上的 Writer 和 Reader，维护节点间的数据流拓扑图。

```cpp
class ChannelManager : public Manager {
public:
  void GetChannelNames(std::vector<std::string>* channels);
  void GetProtoDesc(const std::string& channel_name, std::string* proto_desc);
  void GetMsgType(const std::string& channel_name, std::string* msg_type);

  bool HasWriter(const std::string& channel_name);
  void GetWriters(RoleAttrVec* writers);
  void GetWritersOfNode(const std::string& node_name, RoleAttrVec* writers);
  void GetWritersOfChannel(const std::string& channel_name, RoleAttrVec* writers);

  bool HasReader(const std::string& channel_name);
  void GetReaders(RoleAttrVec* readers);
  void GetReadersOfNode(const std::string& node_name, RoleAttrVec* readers);
  void GetReadersOfChannel(const std::string& channel_name, RoleAttrVec* readers);

  // 拓扑图查询
  void GetUpstreamOfNode(const std::string& node_name, RoleAttrVec* upstream_nodes);
  void GetDownstreamOfNode(const std::string& node_name, RoleAttrVec* downstream_nodes);
  FlowDirection GetFlowDirection(const std::string& lhs_node_name, const std::string& rhs_node_name);

  bool IsMessageTypeMatching(const std::string& lhs, const std::string& rhs);

private:
  Graph node_graph_;                    // 节点间数据流有向图
  WriterWarehouse node_writers_;        // key: node_id, MultiValueWarehouse
  ReaderWarehouse node_readers_;        // key: node_id, MultiValueWarehouse
  WriterWarehouse channel_writers_;     // key: channel_id, MultiValueWarehouse
  ReaderWarehouse channel_readers_;     // key: channel_id, MultiValueWarehouse
};
```

- 广播 Topic：`channel_change_broadcast`
- 允许角色：`ROLE_WRITER`、`ROLE_READER`
- 变更类型：`CHANGE_CHANNEL`
- 使用四个 `MultiValueWarehouse` 分别按 node_id 和 channel_id 索引 Writer/Reader
- 维护 `Graph` 有向图，Writer 所在 Node 为边的 src，Reader 所在 Node 为边的 dst，Channel 为边的 value
- `ScanMessageType` 在 Join 时检查新加入角色的消息类型是否与已有角色匹配
- `RawMessage` 和 `PyMessageWrap` 被加入豁免列表，可与任意类型匹配

### 2.5 ServiceManager

管理 Service 的 Server 和 Client。

```cpp
class ServiceManager : public Manager {
public:
  bool HasService(const std::string& service_name);
  void GetServers(RoleAttrVec* servers);
  void GetClients(const std::string& service_name, RoleAttrVec* clients);

private:
  ServerWarehouse servers_;   // SingleValueWarehouse, key: service_id
  ClientWarehouse clients_;   // MultiValueWarehouse, key: service_id
};
```

- 广播 Topic：`service_change_broadcast`
- 允许角色：`ROLE_SERVER`、`ROLE_CLIENT`
- 变更类型：`CHANGE_SERVICE`
- Server 使用 `SingleValueWarehouse`（每个 service_id 只有一个 Server）
- Client 使用 `MultiValueWarehouse`（一个 Service 可有多个 Client）

### 2.6 容器层

#### WarehouseBase

角色存储的抽象接口，定义了 `Add`、`Remove`、`Search`、`GetAllRoles` 等纯虚方法。支持按 `uint64_t` key 或 `RoleAttributes` 属性匹配进行查询。

#### SingleValueWarehouse

基于 `std::unordered_map<uint64_t, RolePtr>` 实现，每个 key 只存储一个角色。使用 `AtomicRWLock` 保证线程安全。适用于 Node（node_id 唯一）和 Server（service_id 唯一）。

#### MultiValueWarehouse

基于 `std::unordered_multimap<uint64_t, RolePtr>` 实现，每个 key 可存储多个角色。适用于 Writer/Reader（同一 channel 可有多个）和 Client（同一 service 可有多个）。

#### Graph

有向图实现，用于 ChannelManager 中建模节点间的数据流关系。

```cpp
enum FlowDirection { UNREACHABLE, UPSTREAM, DOWNSTREAM };

class Graph {
public:
  void Insert(const Edge& e);
  void Delete(const Edge& e);
  uint32_t GetNumOfEdge();
  FlowDirection GetDirectionOf(const Vertice& lhs, const Vertice& rhs);
};
```

- `Vertice`：以 node_name 为值
- `Edge`：以 channel_name 为值，src 为 Writer 所在 Node，dst 为 Reader 所在 Node
- `GetDirectionOf` 使用 BFS（`LevelTraverse`）判断两个节点间的数据流方向

### 2.7 角色模型

```cpp
class RoleBase {
public:
  virtual bool Match(const proto::RoleAttributes& target_attr) const;
  bool IsEarlierThan(const RoleBase& other) const;
  const proto::RoleAttributes& attributes() const;
};

class RoleWriter : public RoleBase { /* 按 channel_id 匹配 */ };
class RoleServer : public RoleBase { /* 按 service_id 匹配 */ };

using RoleNode = RoleBase;
using RoleReader = RoleWriter;
using RoleClient = RoleServer;
```

### 2.8 通信层

#### ParticipantListener

继承 `eprosima::fastrtps::ParticipantListener`，监听 RTPS Participant 的发现/移除/丢弃事件，回调 `TopologyManager::OnParticipantChange`。

#### SubscriberListener

继承 `eprosima::fastrtps::SubscriberListener`，监听 RTPS Topic 上的新消息，回调 `Manager::OnRemoteChange`。

## 3. 数据流描述

### 3.1 角色注册流程（Join）

```
应用层调用 Node::CreateWriter/CreateReader/CreateService/CreateClient
  → 对应 Manager::Join(attr, role)
    → Check(attr)                    // 校验属性
    → Convert(attr, role, OPT_JOIN)  // 构造 ChangeMsg
    → Dispose(msg)                   // 本地处理（存入 Warehouse）
    → Publish(msg)                   // 通过 RTPS 广播给其他进程
```

### 3.2 远程变更接收流程

```
RTPS Subscriber 收到消息
  → SubscriberListener::onNewDataMessage
    → Manager::OnRemoteChange(msg_str)
      → ParseFromString → ChangeMsg
      → IsFromSameProcess? → 过滤本进程消息
      → Check(msg.role_attr())
      → Dispose(msg)                // 本地处理
```

### 3.3 进程离开清理流程

```
RTPS 检测到远程 Participant 离开
  → ParticipantListener::onParticipantDiscovery
    → TopologyManager::OnParticipantChange
      → Convert(info) → ChangeMsg(OPT_LEAVE)
      → NodeManager::OnTopoModuleLeave(host_name, pid)
      → ChannelManager::OnTopoModuleLeave(host_name, pid)
      → ServiceManager::OnTopoModuleLeave(host_name, pid)
        → 按 host_name + process_id 搜索并移除所有相关角色
        → 逐个 Notify 监听者
```

## 4. 配置方式

ServiceDiscovery 模块的配置主要通过以下方式：

- RTPS Participant 端口：硬编码为 `11511`（见 `TopologyManager::CreateParticipant`）
- Participant 名称：自动生成，格式为 `hostname+pid`
- QoS 配置：各 Manager 的 Publisher/Subscriber 使用 `QOS_PROFILE_TOPO_CHANGE` 配置
- 广播 Topic 名称：
  - NodeManager: `node_change_broadcast`
  - ChannelManager: `channel_change_broadcast`
  - ServiceManager: `service_change_broadcast`

## 5. Proto 定义

### topology_change.proto

```protobuf
enum ChangeType {
  CHANGE_NODE = 1;
  CHANGE_CHANNEL = 2;
  CHANGE_SERVICE = 3;
  CHANGE_PARTICIPANT = 4;
}

enum OperateType {
  OPT_JOIN = 1;
  OPT_LEAVE = 2;
}

enum RoleType {
  ROLE_NODE = 1;
  ROLE_WRITER = 2;
  ROLE_READER = 3;
  ROLE_SERVER = 4;
  ROLE_CLIENT = 5;
  ROLE_PARTICIPANT = 6;
}

message ChangeMsg {
  optional uint64 timestamp = 1;
  optional ChangeType change_type = 2;
  optional OperateType operate_type = 3;
  optional RoleType role_type = 4;
  optional RoleAttributes role_attr = 5;
}
```

### role_attributes.proto

```protobuf
message RoleAttributes {
  optional string host_name = 1;
  optional string host_ip = 2;
  optional int32 process_id = 3;
  optional string node_name = 4;
  optional uint64 node_id = 5;
  optional string channel_name = 6;
  optional uint64 channel_id = 7;
  optional string message_type = 8;
  optional bytes proto_desc = 9;
  optional uint64 id = 10;
  optional QosProfile qos_profile = 11;
  optional SocketAddr socket_addr = 12;
  optional string service_name = 13;
  optional uint64 service_id = 14;
}
```

## 6. 与其他模块的关系

| 模块 | 关系 |
|------|------|
| `cyber/transport/rtps/` | 底层依赖，提供 RTPS Participant、Publisher、Subscriber 的封装 |
| `cyber/node/` | Node 创建 Writer/Reader/Service/Client 时调用 Manager::Join 注册拓扑 |
| `cyber/service/` | Service/Client 的创建依赖 ServiceManager 进行服务发现 |
| `cyber/parameter/` | ParameterServer/Client 基于 Service/Client 模式，间接依赖 ServiceManager |
| `cyber/base/signal.h` | 提供 Signal/Connection 机制，用于拓扑变更的观察者模式 |
| `cyber/common/global_data.h` | 提供 HostName、ProcessId、RegisterNode/RegisterChannel 等全局数据 |
