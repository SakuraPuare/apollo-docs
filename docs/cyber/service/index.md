---
title: "Service"
---

# Service

> 源码路径: `cyber/service/`

## 概述

`service` 模块实现了 Cyber RT 的 **服务通信（Service/Client）** 模式，一种基于请求-响应的同步/异步 RPC 机制。与发布-订阅不同，服务通信要求调用方发送请求后等待服务方返回响应。

模块由四个头文件组成（所有模板代码内联于头文件中，无独立 `.cc`）：

| 文件 | 职责 |
|------|------|
| `service_base.h` | Service 基类，持有服务名称，定义 `destroy()` 纯虚接口 |
| `client_base.h` | Client 基类，持有服务名称，提供服务发现等待逻辑 |
| `service.h` | 模板化 Service 实现，接收请求、回调后返回响应 |
| `client.h` | 模板化 Client 实现，同步/异步发送请求并获取响应 |

底层通信使用 RTPS 传输层，请求和响应走独立通道（`service_name + SRV_CHANNEL_REQ_SUFFIX` / `SRV_CHANNEL_RES_SUFFIX`）。

## 核心类

### ClientBase / ServiceBase

基类分别管理客户端/服务端的服务名称生命周期。`ClientBase` 额外提供 `WaitForServiceNanoseconds` 以 5ms 步长轮询服务发现，等待目标 Service 上线。

```cpp
// ClientBase 核心接口
explicit ClientBase(const std::string& service_name);
virtual void Destroy() = 0;
virtual bool ServiceIsReady() const = 0;
bool WaitForServiceNanoseconds(std::chrono::nanoseconds time_out);

// ServiceBase 核心接口
explicit ServiceBase(const std::string& service_name);
virtual void destroy() = 0;
const std::string& service_name() const;
```

### Client\<Request, Response\>

模板化客户端，继承 `ClientBase`。一个 Client 实例只能请求一个 Service。构造时自动生成请求/响应通道名。关键类型别名：

```cpp
using SharedRequest  = std::shared_ptr<Request>;
using SharedResponse = std::shared_ptr<Response>;
using SharedFuture   = std::shared_future<SharedResponse>;
using CallbackType   = std::function<void(SharedFuture)>;
```

内部通过 `pending_requests_`（`unordered_map<uint64_t, tuple<promise, callback, future>>`）跟踪未完成的请求，以序列号匹配响应。

### Service\<Request, Response\>

模板化服务端，继承 `ServiceBase`。内部维护 `std::list<function<void()>>` 任务队列和独立处理线程，收到请求后异步入队、串行处理。

```cpp
using ServiceCallback = std::function<void(const std::shared_ptr<Request>&,
                                           std::shared_ptr<Response>&)>;
```

## 核心函数

### Client::Init

```cpp
bool Client<Request, Response>::Init();
```

- **职责**: 初始化请求发送器和响应接收器
- **关键步骤**: 创建 `request_channel_` 的 RTPS Transmitter -> 绑定 `HandleResponse` 回调 -> 创建 `response_channel_` 的 RTPS Receiver
- **输出**: 成功返回 `true`，任一创建失败返回 `false`

### Client::SendRequest（同步）

```cpp
SharedResponse SendRequest(SharedRequest request,
                           const std::chrono::seconds& timeout_s = std::chrono::seconds(5));
```

- **职责**: 同步发送请求并阻塞等待响应
- **关键步骤**: 调用 `AsyncSendRequest` 获取 future -> `future.wait_for(timeout_s)` -> 超时返回 `nullptr`
- **输出**: `SharedResponse` 或 `nullptr`（超时/未初始化）

### Client::AsyncSendRequest（异步，带回调）

```cpp
SharedFuture AsyncSendRequest(SharedRequest request, CallbackType&& cb);
```

- **职责**: 异步发送请求，响应到达后回调通知
- **关键步骤**: 递增 `sequence_number_` -> `Transmit()` 发送请求 -> 创建 promise/future 存入 `pending_requests_`
- **输出**: `SharedFuture`

### Client::HandleResponse（私有）

```cpp
void HandleResponse(const std::shared_ptr<Response>& response,
                    const transport::MessageInfo& request_header);
```

- **职责**: 响应到达后匹配序列号、唤醒等待方
- **关键步骤**: 校验 `spare_id == writer_id_` -> 按 `seq_num` 查找 `pending_requests_` -> `promise->set_value(response)` -> 执行回调 -> 移除条目

### Client::WaitForService

```cpp
template <typename RatioT = std::milli>
bool WaitForService(std::chrono::duration<int64_t, RatioT> timeout);
```

- **职责**: 阻塞等待目标 Service 可用或超时
- **实现**: 委托 `WaitForServiceNanoseconds`，以 5ms 步长通过 `TopologyManager::service_manager()->HasService()` 轮询

### Service::Init

```cpp
bool Service<Request, Response>::Init();
```

- **职责**: 初始化响应发送器、请求接收器及内部处理线程
- **关键步骤**: 创建 `response_channel_` 的 RTPS Transmitter -> 创建 `request_channel_` 的 RTPS Receiver（到达时 `Enqueue`） -> 启动 `Process` 线程

### Service::HandleRequest（私有）

```cpp
void HandleRequest(const std::shared_ptr<Request>& request,
                   const transport::MessageInfo& message_info);
```

- **职责**: 处理单个请求 — 调用用户回调并发送响应
- **关键步骤**: 加锁 `service_handle_request_mutex_` -> 创建 Response -> 调用 `service_callback_(request, response)` -> 复制 `message_info` 并设 `sender_id` -> `SendResponse()`

### Service::Enqueue / Process（私有）

```cpp
void Enqueue(std::function<void()>&& task);
void Process();
```

- **职责**: 生产者-消费者任务队列
- **Enqueue**: 加锁追加到 `tasks_` 尾部 -> `notify_one`
- **Process**: 循环 `condition_.wait` 取出队首任务执行；`inited_ == false` 时退出

### Service::destroy

```cpp
void destroy();
```

- **职责**: 停止服务释放资源
- **关键步骤**: `inited_ = false` -> 清空队列 -> `notify_all` -> `join` 处理线程

## 配置

本模块无独立配置文件，相关参数如下：

| 配置项 | 来源 | 说明 |
|--------|------|------|
| QoS Profile | `QosProfileConf::QOS_PROFILE_SERVICES_DEFAULT` | 请求/响应通道默认 QoS |
| 传输模式 | `proto::OptionalMode::RTPS` | 所有通道使用 RTPS |
| 服务发现轮询间隔 | 5ms（硬编码） | `WaitForServiceNanoseconds` 步长 |
| 同步请求超时 | 5 秒（默认参数） | `SendRequest` 的 `timeout_s` |

## 调用关系

```text
用户代码
  |
  +---> Client::Init() / Service::Init()
  |       |-- CreateTransmitter<RTPS>()   // 请求/响应发送器
  |       +-- CreateReceiver<RTPS>()      // 请求/响应接收器
  |
  +---> Client::SendRequest() / AsyncSendRequest()
  |       |-- Transmitter::Transmit()     // 通过 RTPS 通道发送
  |       +-- pending_requests_[seq]      // 注册 promise 等待
  |
  +---> [RTPS 网络传输]
  |
  +---> Service::HandleRequest()（由 Process 线程从队列取出执行）
  |       |-- service_callback_(req, resp) // 用户回调
  |       +-- Transmitter::Transmit()      // 发送响应
  |
  +---> [RTPS 网络传输]
  |
  +---> Client::HandleResponse()
          |-- promise.set_value(resp)     // 唤醒等待方
          +-- callback(future)            // 执行回调
```

Service 端通过独立线程和条件变量队列保证请求串行处理，Client 端通过 `promise`/`future` 机制支持同步阻塞和异步回调两种调用方式。

