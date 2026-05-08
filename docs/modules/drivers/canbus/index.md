---
title: "Canbus CAN 总线驱动"
---

# Canbus CAN 总线驱动

> 源码路径：`modules/drivers/canbus/`

## 概述

`canbus` 驱动模块是 Apollo 与车辆 CAN 总线通信的基础设施，负责通过 CAN 卡硬件收发 CAN 报文，并将原始报文解析为上层可使用的 protobuf 消息。模块采用模板化设计，`SensorCanbus<T>` 可适配不同类型的 CAN 传感器（如 Mobileye、底盘等）。同时包含 `CanSender` 用于向车辆发送控制指令。

## 架构

```text
CAN 总线
    │
    ▼
CanClient (CAN 卡硬件接口)
    │
    ├── CanReceiver<T>     ← 接收 CAN 报文
    │     └── MessageManager<T>  ← 解析报文为 protobuf
    │
    ├── CanSender<T>       ← 发送 CAN 报文
    │
    └── SensorCanbus<T>    ← 组件驱动
          ├── 定时发布 (sensor_freq > 0)
          └── 事件触发发布 (sensor_freq == 0)
```

## 核心类

### SensorCanbus\<SensorType\>

CAN 传感器驱动组件模板，负责初始化 CAN 卡、接收/解析报文、发布消息。

```cpp
template <typename SensorType>
class SensorCanbus : public cyber::Component<> {
 public:
  bool Init() override;

 private:
  bool Start();
  void PublishSensorData();
  void OnTimer();
  void DataTrigger();
  bool OnError(const std::string& error_msg);
  void RegisterCanClients();

  SensorCanbusConf canbus_conf_;
  std::unique_ptr<CanClient> can_client_;
  CanReceiver<SensorType> can_receiver_;
  std::unique_ptr<MessageManager<SensorType>> sensor_message_manager_;
  std::shared_ptr<Writer<SensorType>> sensor_writer_;
  std::unique_ptr<cyber::Timer> timer_;
};
```

**源码**：`modules/drivers/canbus/sensor_canbus.h`

### CanClient

CAN 卡硬件抽象接口，通过工厂模式创建具体实现。

```cpp
class CanClient {
 public:
  virtual ErrorCode Init(const CANCardParameter& param) = 0;
  virtual ErrorCode Start() = 0;
  virtual ErrorCode Stop() = 0;
  virtual ErrorCode Send(const std::vector<CanFrame>& frames) = 0;
  virtual ErrorCode Receive(std::vector<CanFrame>* frames) = 0;
};
```

**源码**：`modules/drivers/canbus/can_client/can_client.h`

### CanClientFactory

CAN 客户端工厂，根据配置创建对应的 `CanClient` 实例。

**源码**：`modules/drivers/canbus/can_client/can_client_factory.h`

### CanReceiver\<SensorType\>

CAN 报文接收器，从 `CanClient` 读取报文并通过 `MessageManager` 解析。

```cpp
template <typename SensorType>
class CanReceiver {
 public:
  ErrorCode Init(CanClient* client, MessageManager<SensorType>* manager,
                 bool enable_log);
  ErrorCode Start();
  ErrorCode Stop();
};
```

**源码**：`modules/drivers/canbus/can_comm/can_receiver.h`

### CanSender\<SensorType\>

CAN 报文发送器，将 protobuf 消息编码为 CAN 报文并发送。

**源码**：`modules/drivers/canbus/can_comm/can_sender.h`

### MessageManager\<SensorType\>

消息管理器，维护 CAN ID 到解析器的映射，将原始 CAN 报文解析为 protobuf 消息。

**源码**：`modules/drivers/canbus/can_comm/message_manager.h`

## 核心函数

### SensorCanbus::Init()

```cpp
template <typename SensorType>
bool SensorCanbus<SensorType>::Init() {
  // 1. 加载配置
  GetProtoFromFile(config_file_path_, &canbus_conf_);

  // 2. 创建 CAN 客户端
  can_client_ = CanClientFactory::Instance()
      ->CreateCANClient(canbus_conf_.can_card_parameter());

  // 3. 创建消息管理器
  sensor_message_manager_.reset(new MessageManager<SensorType>());

  // 4. 初始化接收器
  can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                     canbus_conf_.enable_receiver_log());

  // 5. 创建 Writer
  sensor_writer_ = node_->CreateWriter<SensorType>(FLAGS_sensor_node_name);

  // 6. 启动
  return Start();
}
```

### SensorCanbus::Start()

```cpp
template <typename SensorType>
bool SensorCanbus<SensorType>::Start() {
  can_client_->Start();      // 启动 CAN 卡硬件
  can_receiver_.Start();     // 启动接收线程

  if (FLAGS_sensor_freq > 0) {
    // 定时模式：按频率周期发布
    timer_.reset(new cyber::Timer(duration_ms, [this]() { OnTimer(); }));
    timer_->Start();
  } else {
    // 事件触发模式：有新数据时发布
    thread_.reset(new std::thread([this]() { DataTrigger(); }));
  }
}
```

### SensorCanbus::DataTrigger()

事件触发发布模式，通过条件变量等待新数据到达后立即发布。

```cpp
template <typename SensorType>
void SensorCanbus<SensorType>::DataTrigger() {
  std::condition_variable* cvar = sensor_message_manager_->GetMutableCVar();
  while (data_trigger_running_) {
    cvar->wait(lock);
    PublishSensorData();
    sensor_message_manager_->ClearSensorData();
  }
}
```

## 子模块

| 子目录 | 说明 |
|--------|------|
| `can_client/` | CAN 卡硬件接口与工厂 |
| `can_comm/` | CAN 收发器与消息管理器 |
| `common/` | CAN 通用工具 |
| `proto/` | 配置 protobuf |

## 配置

通过 `SensorCanbusConf` protobuf 加载：

| 字段 | 类型 | 说明 |
|------|------|------|
| can_card_parameter | CANCardParameter | CAN 卡硬件参数 |
| enable_receiver_log | bool | 是否启用接收日志 |

gflags：

| 标志位 | 说明 |
|--------|------|
| `FLAGS_sensor_node_name` | 传感器节点名称 |
| `FLAGS_sensor_freq` | 发布频率（Hz），0 为事件触发模式 |

## 调用关系

- **上游**：CAN 总线硬件
- **下游**：发布 `SensorType` 消息（如 `Chassis`、`Mobileye` 等）到 CyberRT 通道
- **依赖**：`CanClient`（硬件驱动）、`MessageManager`（报文解析）
- **被依赖**：`canbus-vehicle` 中各车型模块使用此驱动
