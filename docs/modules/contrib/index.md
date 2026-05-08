---
title: "社区贡献模块"
---

# Community Contributed Modules

> 源码路径: `modules/contrib/`

## 概述

`contrib` 模块汇集社区贡献的扩展功能，主要包含以下子模块：

- **cyber_bridge** — 基于 Boost.Asio 的 TCP 桥接服务，允许外部客户端（如 LG SVL 模拟器）通过自定义二进制协议接入 CyberRT 的发布/订阅系统。
- **lgsvl_msgs** — 为 LG SVL 模拟器定义的 Protobuf 消息集，包含 2D/3D 目标检测消息格式。
- **elo** — 百度自车定位（Ego Localization）模块，融合 GNSS、视觉传感器和高精地图实现厘米级定位（需单独下载）。
- **e2e** — 端到端驾驶模块（需单独下载）。

核心可运行组件为 `cyber_bridge`，其余子模块以独立构建或外部下载方式提供。

## 核心类

### Server

TCP 服务端，监听端口 9090（可通过 `--port` gflag 配置），负责接受客户端连接并分发给 `Client` 实例。

```cpp
// server.h
class Server : public std::enable_shared_from_this<Server> {
 public:
  explicit Server(Node* node);
  void run();

 private:
  Node& node;
  Clients clients;
  boost::asio::io_service io;
  boost::asio::signal_set signals;
  boost::asio::ip::tcp::acceptor acceptor;

  void stop(const boost::system::error_code& error, int signal_number);
  void begin_accept();
  void end_accept(const boost::system::error_code& ec);
};
```

### Client

单个 TCP 客户端连接的处理器。接收二进制协议帧，根据操作码分发到对应处理函数，并将 CyberRT 消息回传给客户端。

```cpp
// client.h
class Client : public std::enable_shared_from_this<Client> {
 public:
  Client(Node* node, Clients* clients, boost::asio::ip::tcp::socket socket);
  void start();
  void stop();
  void publish(const std::string& channel, const std::string& msg);

 private:
  Node& node;
  Clients& clients;
  boost::asio::ip::tcp::socket socket;

  void handle_register_desc();
  void handle_add_writer();
  void handle_add_reader();
  void handle_publish();
};
```

### Clients

客户端连接池，管理所有活跃的 `Client` 实例的生命周期。

```cpp
// clients.h
class Clients {
 public:
  void start(std::shared_ptr<Client> client);
  void stop(std::shared_ptr<Client> client);
  void stop_all();

 private:
  std::unordered_set<std::shared_ptr<Client>> clients;
};
```

### Node

桥接层核心类，将 TCP 客户端的操作映射到 CyberRT 的 `Reader`/`Writer` 节点，使用 `PyMessageWrap` 作为通用消息载体。

```cpp
// node.h
class Node {
 public:
  void remove(std::shared_ptr<Client> client);
  void add_reader(const std::string& channel, const std::string& type,
                  std::shared_ptr<Client> client);
  void add_writer(const std::string& channel, const std::string& type,
                  std::shared_ptr<Client> client);
  void publish(const std::string& channel, const std::string& data);

 private:
  std::unique_ptr<apollo::cyber::Node> node;
  Writers writers;  // channel -> Writer 映射
  Readers readers;  // channel -> Reader 映射
};
```

## 核心函数

### bridge.cc — main

程序入口，初始化 CyberRT，创建 `Node` 和 `Server`，启动事件循环。

```cpp
int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cyber::Init(argv[0]);
  Node node;
  auto server = std::make_shared<Server>(&node);
  server->run();
  apollo::cyber::Clear();
}
```

### Client::handle_read

异步读取回调，从字节流中解析操作码并分发：

| 操作码 | 常量 | 说明 |
|--------|------|------|
| 1 | `OP_REGISTER_DESC` | 注册 Protobuf 描述符 |
| 2 | `OP_ADD_READER` | 创建 CyberRT Reader 订阅 |
| 3 | `OP_ADD_WRITER` | 创建 CyberRT Writer 发布 |
| 4 | `OP_PUBLISH` | 通过 Writer 发布消息 |

### Client::publish

将 CyberRT 收到的消息序列化为二进制帧，异步写回 TCP 客户端。使用双缓冲（`writing` + `pending`）避免锁竞争，`pending` 上限为 1GB。

### Node::add_reader / Node::add_writer

- `add_reader`：若 channel 已有 Reader 则复用，否则创建新的 CyberRT Reader 并注册回调，回调中将消息广播给该 channel 上所有客户端。
- `add_writer`：若 channel 已有 Writer 则复用，否则从 `ProtobufFactory` 获取描述符并创建 CyberRT Writer。

## 配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--port` | int32 | 9090 | TCP 监听端口 |

通过 gflags 在启动时指定，例如：

```bash
./cyber_bridge --port=9090
```

### lgsvl_msgs 消息定义

Protobuf 包名 `apollo.contrib.lgsvl_msgs`，定义以下消息：

| 消息 | 说明 |
|------|------|
| `Detection2D` | 2D 检测结果：header、id、label、score、2D 边界框、速度 |
| `Detection2DArray` | 2D 检测结果数组 |
| `Detection3D` | 3D 检测结果：header、id、label、score、3D 边界框（含位姿和尺寸）、速度 |
| `Detection3DArray` | 3D 检测结果数组 |

## 调用关系

```text
main()
  |
  +-> Node()                        // 创建 CyberRT 节点 "bridge"
  +-> Server(&node)
        |
        +-> run()
              |
              +-> begin_accept()    // 异步等待 TCP 连接
              |     |
              |     +-> end_accept()
              |           |
              |           +-> Client(&node, &clients, socket)
              |           +-> clients.start(client)
              |                 |
              |                 +-> client->start()   // 启动异步读取
              |                       |
              |                       +-> handle_read()
              |                             |
              |                             +-> handle_register_desc()  // 注册 proto 描述符
              |                             +-> handle_add_reader()     // 订阅 channel
              |                             |     +-> node.add_reader()
              |                             |           +-> cyber::Reader 回调 -> client->publish()
              |                             +-> handle_add_writer()     // 发布 channel
              |                             |     +-> node.add_writer()
              |                             +-> handle_publish()        // 发布消息
              |                                   +-> node.publish()
              |                                         +-> writer->Write()
              |
              +-> io.run()          // Boost.Asio 事件循环
```

外部客户端通过 TCP 连接到 cyber_bridge，依次发送注册描述符、添加 Reader/Writer、发布消息等操作码，bridge 将这些操作转换为 CyberRT 的发布/订阅调用，实现外部系统与 Apollo CyberRT 的通信桥接。
