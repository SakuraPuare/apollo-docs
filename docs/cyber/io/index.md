---
title: IO 异步 I/O 模块
---

# IO 异步 I/O 模块

> 源码路径: `cyber/io/`

## 概述

IO 模块是 Cyber RT 的异步网络 I/O 基础设施，基于 Linux `epoll` 实现，为协程（CRoutine）提供非阻塞的 socket 读写能力。模块的核心设计思想是：当协程执行 I/O 操作遇到 `EAGAIN` 时，不阻塞线程，而是通过 `Poller` 注册 epoll 事件，将协程挂起为 `IO_WAIT` 状态；当 epoll 事件就绪时，回调唤醒协程继续执行。

模块由三层组成：

- **数据结构层**：`PollRequest` / `PollResponse` / `PollCtrlParam` 定义 I/O 事件的请求、响应与控制参数
- **调度层**：`Poller`（单例）封装 epoll 主循环，管理 fd 注册/注销，驱动事件分发
- **应用层**：`PollHandler` 将协程与 epoll 事件绑定；`Session` 封装 POSIX socket API，提供协程友好的网络操作接口

## 核心类

### PollRequest / PollResponse / PollCtrlParam

`poll_data.h` 中定义的三个 POD 结构体，描述 epoll 事件的完整生命周期。

```cpp
struct PollResponse {
  explicit PollResponse(uint32_t e = 0) : events(e);
  uint32_t events;           // epoll 返回的就绪事件掩码
};

struct PollRequest {
  int fd = -1;                                          // 目标文件描述符
  uint32_t events = 0;                                  // 关注的 epoll 事件类型
  int timeout_ms = -1;                                  // 超时（<0 永不超时，>0 毫秒数）
  std::function<void(const PollResponse&)> callback;    // 事件就绪回调
};

struct PollCtrlParam {
  int operation;    // EPOLL_CTL_ADD / MOD / DEL
  int fd;
  epoll_event event;
};
```

### Poller

全局单例，维护一个 epoll 实例和专用轮询线程，负责所有 fd 的事件监听与分发。

```cpp
class Poller {
 public:
  using RequestPtr = std::shared_ptr<PollRequest>;
  using RequestMap = std::unordered_map<int, RequestPtr>;
  using CtrlParamMap = std::unordered_map<int, PollCtrlParam>;

  void Shutdown();
  bool Register(const PollRequest& req);    // 注册 / 更新 fd 监听
  bool Unregister(const PollRequest& req);  // 注销 fd 监听

 private:
  void Init();
  void Clear();
  void Poll(int timeout_ms);          // 执行一次 epoll_wait 并分发回调
  void ThreadFunc();                  // 轮询线程主循环
  void HandleChanges();              // 应用待处理的 epoll_ctl 变更
  int GetTimeoutMs();                // 计算最近超时
  void Notify();                     // 通过 pipe 唤醒 epoll_wait

  DECLARE_SINGLETON(Poller)
};
```

### PollHandler

每个需要异步 I/O 的协程拥有一个 `PollHandler`，负责将协程挂起/唤醒与 epoll 事件关联。

```cpp
class PollHandler {
 public:
  explicit PollHandler(int fd);
  bool Block(int timeout_ms, bool is_read);   // 挂起当前协程等待 I/O
  bool Unblock();                              // 取消等待
  int fd() const;
  void set_fd(int fd);

 private:
  bool Check(int timeout_ms);                 // 校验参数与协程上下文
  void Fill(int timeout_ms, bool is_read);    // 填充 PollRequest
  void ResponseCallback(const PollResponse& rsp);  // 唤醒协程
};
```

### Session

对 POSIX socket API 的协程安全封装，内部持有 `PollHandler`，所有读写操作在 `EAGAIN` 时自动挂起协程等待。

```cpp
class Session {
 public:
  using SessionPtr = std::shared_ptr<Session>;

  Session();
  explicit Session(int fd);

  int Socket(int domain, int type, int protocol);
  int Listen(int backlog);
  int Bind(const struct sockaddr *addr, socklen_t addrlen);
  SessionPtr Accept(struct sockaddr *addr, socklen_t *addrlen);
  int Connect(const struct sockaddr *addr, socklen_t addrlen);
  int Close();

  ssize_t Recv(void *buf, size_t len, int flags, int timeout_ms = -1);
  ssize_t RecvFrom(void *buf, size_t len, int flags,
                   struct sockaddr *src_addr, socklen_t *addrlen,
                   int timeout_ms = -1);
  ssize_t Send(const void *buf, size_t len, int flags, int timeout_ms = -1);
  ssize_t SendTo(const void *buf, size_t len, int flags,
                 const struct sockaddr *dest_addr, socklen_t addrlen,
                 int timeout_ms = -1);
  ssize_t Read(void *buf, size_t count, int timeout_ms = -1);
  ssize_t Write(const void *buf, size_t count, int timeout_ms = -1);

  int fd() const;
};
```

## 核心函数

### Poller::Register

注册或更新一个 fd 到 epoll 监听集合。

- **职责**：将 `PollRequest` 转换为 `PollCtrlParam`，存入请求映射表，通知轮询线程
- **输入**：`const PollRequest& req`（fd、事件掩码、超时、回调）
- **输出**：`bool`，注册成功返回 `true`
- **关键步骤**：
  1. 校验 fd 和 callback 有效性
  2. 写锁保护下，根据 fd 是否已存在决定 `EPOLL_CTL_ADD` 或 `EPOLL_CTL_MOD`
  3. 存储请求到 `requests_` 映射，控制参数到 `ctrl_params_` 映射
  4. 调用 `Notify()` 唤醒 epoll_wait

```cpp
bool Poller::Register(const PollRequest& req) {
  if (is_shutdown_.load()) { return false; }
  if (req.fd < 0 || req.callback == nullptr) { return false; }

  PollCtrlParam ctrl_param{};
  ctrl_param.fd = req.fd;
  ctrl_param.event.data.fd = req.fd;
  ctrl_param.event.events = req.events;

  {
    WriteLockGuard<AtomicRWLock> lck(poll_data_lock_);
    if (requests_.count(req.fd) == 0) {
      ctrl_param.operation = EPOLL_CTL_ADD;
      requests_[req.fd] = std::make_shared<PollRequest>();
    } else {
      ctrl_param.operation = EPOLL_CTL_MOD;
    }
    *requests_[req.fd] = req;
    ctrl_params_[ctrl_param.fd] = ctrl_param;
  }

  Notify();
  return true;
}
```

### Poller::Poll

执行一次 `epoll_wait`，处理超时，分发回调。

- **职责**：等待 epoll 事件就绪，对每个就绪 fd 调用其注册的回调
- **输入**：`int timeout_ms`（本次 epoll_wait 的超时毫秒数）
- **输出**：无（通过回调通知）
- **关键步骤**：
  1. 调用 `epoll_wait` 获取就绪事件
  2. 遍历所有请求，递减有超时的请求计时器，超时到期的生成空 `PollResponse`
  3. 合并 epoll 就绪事件到响应表
  4. 对每个响应调用对应 `request->callback`

```cpp
void Poller::Poll(int timeout_ms) {
  epoll_event evt[kPollSize];
  auto before_time_ns = Time::Now().ToNanosecond();
  int ready_num = epoll_wait(epoll_fd_, evt, kPollSize, timeout_ms);
  auto after_time_ns = Time::Now().ToNanosecond();
  int interval_ms =
      static_cast<int>((after_time_ns - before_time_ns) / 1000000);
  if (interval_ms == 0) { interval_ms = 1; }

  std::unordered_map<int, PollResponse> responses;
  {
    ReadLockGuard<AtomicRWLock> lck(poll_data_lock_);
    for (auto& item : requests_) {
      auto& request = item.second;
      if (ctrl_params_.count(request->fd) != 0) { continue; }
      if (request->timeout_ms > 0) {
        request->timeout_ms -= interval_ms;
        if (request->timeout_ms < 0) { request->timeout_ms = 0; }
      }
      if (request->timeout_ms == 0) {
        responses[item.first] = PollResponse();
        request->timeout_ms = -1;
      }
    }
  }

  if (ready_num > 0) {
    for (int i = 0; i < ready_num; ++i) {
      responses[evt[i].data.fd] = PollResponse(evt[i].events);
    }
  }

  for (auto& item : responses) {
    ReadLockGuard<AtomicRWLock> lck(poll_data_lock_);
    auto search = requests_.find(item.first);
    if (search != requests_.end()) {
      search->second->timeout_ms = -1;
      search->second->callback(item.second);
    }
  }
}
```

### Poller::ThreadFunc

轮询线程主循环，屏蔽所有信号，持续调用 `HandleChanges` + `GetTimeoutMs` + `Poll`。

```cpp
void Poller::ThreadFunc() {
  sigset_t signal_set;
  sigfillset(&signal_set);
  pthread_sigmask(SIG_BLOCK, &signal_set, nullptr);

  while (!is_shutdown_.load()) {
    HandleChanges();
    int timeout_ms = GetTimeoutMs();
    Poll(timeout_ms);
  }
}
```

### PollHandler::Block

将当前协程挂起，等待指定 fd 的读/写事件就绪。

- **职责**：构建 `PollRequest`，注册到 `Poller`，挂起协程，事件就绪后检查结果
- **输入**：`int timeout_ms`（超时毫秒数），`bool is_read`（`true` 等读，`false` 等写）
- **输出**：`bool`，目标事件就绪返回 `true`，超时返回 `false`
- **关键步骤**：
  1. `Check()` 校验 timeout、fd、当前协程上下文
  2. 原子交换 `is_blocking_` 防止重入
  3. `Fill()` 填充 `PollRequest`（设置 `EPOLLET | EPOLLONESHOT`，根据读写添加 `EPOLLIN`/`EPOLLOUT`）
  4. 注册到 `Poller::Instance()->Register()`
  5. `routine_->Yield(RoutineState::IO_WAIT)` 挂起协程
  6. 恢复后检查 `response_.events` 是否包含目标事件

```cpp
bool PollHandler::Block(int timeout_ms, bool is_read) {
  if (!Check(timeout_ms)) { return false; }
  if (is_blocking_.exchange(true)) { return false; }

  Fill(timeout_ms, is_read);
  if (!Poller::Instance()->Register(request_)) {
    is_blocking_.store(false);
    return false;
  }

  routine_->Yield(RoutineState::IO_WAIT);

  bool result = false;
  uint32_t target_events = is_read ? EPOLLIN : EPOLLOUT;
  if (response_.events & target_events) { result = true; }
  is_blocking_.store(false);
  return result;
}
```

### PollHandler::ResponseCallback

epoll 事件就绪时由 `Poller` 回调，保存响应并唤醒挂起的协程。

- **职责**：存储事件结果，通知调度器恢复协程
- **输入**：`const PollResponse& rsp`
- **输出**：无
- **关键步骤**：
  1. 检查 `is_blocking_` 和 `routine_` 有效性
  2. 保存 `response_`
  3. 若协程处于 `IO_WAIT`，调用 `scheduler::Instance()->NotifyTask()`

```cpp
void PollHandler::ResponseCallback(const PollResponse& rsp) {
  if (!is_blocking_.load() || routine_ == nullptr) { return; }
  response_ = rsp;
  if (routine_->state() == RoutineState::IO_WAIT) {
    scheduler::Instance()->NotifyTask(routine_->id());
  }
}
```

### Session::Accept

协程友好的 accept 封装，在无连接可接受时挂起协程等待。

- **职责**：调用 `accept4`，遇到 `EAGAIN` 时通过 `PollHandler::Block` 挂起等待新连接
- **输入**：`struct sockaddr *addr`，`socklen_t *addrlen`
- **输出**：`SessionPtr`，新连接的 Session；失败返回 `nullptr`
- **关键步骤**：
  1. 调用 `accept4(fd_, addr, addrlen, SOCK_NONBLOCK)`
  2. 若返回 `-1` 且 `errno == EAGAIN`，调用 `poll_handler_->Block(-1, true)` 挂起等待
  3. 唤醒后重试 `accept4`
  4. 成功则构造新的 `Session(sock_fd)` 返回

```cpp
auto Session::Accept(struct sockaddr *addr, socklen_t *addrlen) -> SessionPtr {
  ACHECK(fd_ != -1);
  int sock_fd = accept4(fd_, addr, addrlen, SOCK_NONBLOCK);
  while (sock_fd == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    poll_handler_->Block(-1, true);
    sock_fd = accept4(fd_, addr, addrlen, SOCK_NONBLOCK);
  }
  if (sock_fd == -1) { return nullptr; }
  return std::make_shared<Session>(sock_fd);
}
```

### Session::Connect

协程友好的 connect 封装，非阻塞连接后等待可写事件确认连接完成。

- **职责**：发起非阻塞连接，挂起等待连接结果
- **输入**：`const struct sockaddr *addr`，`socklen_t addrlen`
- **输出**：`int`，成功返回 `0`，失败返回 `-1`
- **关键步骤**：
  1. 调用 `connect()`
  2. 若返回 `-1` 且 `errno == EINPROGRESS`，调用 `poll_handler_->Block(-1, false)` 等待可写
  3. 唤醒后用 `getsockopt(SO_ERROR)` 检查连接结果

```cpp
int Session::Connect(const struct sockaddr *addr, socklen_t addrlen) {
  ACHECK(fd_ != -1);
  int optval;
  socklen_t optlen = sizeof(optval);
  int res = connect(fd_, addr, addrlen);
  if (res == -1 && errno == EINPROGRESS) {
    poll_handler_->Block(-1, false);
    getsockopt(fd_, SOL_SOCKET, SO_ERROR,
               reinterpret_cast<void *>(&optval), &optlen);
    if (optval == 0) { res = 0; } else { errno = optval; }
  }
  return res;
}
```

### Session::Recv / Send / Read / Write

通用的读写封装，模式一致：先尝试系统调用，`EAGAIN` 时挂起等待就绪后重试。

- **职责**：协程安全的 I/O 读写，支持超时控制
- **timeout_ms 语义**：`< 0` 持续重试直到成功；`== 0` 仅尝试一次；`> 0` 在超时内重试
- **关键步骤**：
  1. 调用系统调用（`recv` / `send` / `read` / `write`）
  2. 若 `timeout_ms == 0`，直接返回结果
  3. 循环：`EAGAIN` 时 `poll_handler_->Block()` 挂起等待，唤醒后重试
  4. `timeout_ms > 0` 时仅重试一次（Block 超时后 break）

```cpp
ssize_t Session::Recv(void *buf, size_t len, int flags, int timeout_ms) {
  ACHECK(buf != nullptr);
  ACHECK(fd_ != -1);
  ssize_t nbytes = recv(fd_, buf, len, flags);
  if (timeout_ms == 0) { return nbytes; }
  while (nbytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (poll_handler_->Block(timeout_ms, true)) {
      nbytes = recv(fd_, buf, len, flags);
    }
    if (timeout_ms > 0) { break; }
  }
  return nbytes;
}
```

## 配置

| 常量 | 值 | 说明 |
|---|---|---|
| `kPollSize` | 32 | `epoll_create` 初始大小 & 每次 `epoll_wait` 最大事件数 |
| `kPollTimeoutMs` | 100 | 无活跃请求时的默认 `epoll_wait` 超时（毫秒） |

所有 socket 创建时自动附加 `SOCK_NONBLOCK` 标志，无需手动配置。epoll 事件使用 `EPOLLET`（边缘触发）+ `EPOLLONESHOT`（单次触发），确保同一 fd 不会被多个协程同时处理。

## 调用关系

```text
Session (用户层)
  |-- Socket / Listen / Bind / Connect / Accept / Close
  |-- Recv / Send / Read / Write / RecvFrom / SendTo
        |
        v
PollHandler (协程桥接层)
  |-- Block()   --> Poller::Register() --> 协程 Yield(IO_WAIT)
  |-- Unblock() --> Poller::Unregister()
  |-- ResponseCallback() <--> scheduler::NotifyTask()
        |
        v
Poller (epoll 调度层，单例)
  |-- ThreadFunc() 主循环
  |     |-- HandleChanges()  --> epoll_ctl(ADD/MOD/DEL)
  |     |-- GetTimeoutMs()
  |     |-- Poll()           --> epoll_wait --> 回调分发
  |-- Notify()  --> pipe 写入唤醒 epoll_wait
```

协程调用 `Session::Recv` 时，若数据未就绪（`EAGAIN`），`PollHandler::Block` 将请求注册到 `Poller` 并挂起协程。`Poller` 的专用线程通过 `epoll_wait` 检测到 fd 可读后，回调 `PollHandler::ResponseCallback`，由调度器唤醒协程继续执行。
