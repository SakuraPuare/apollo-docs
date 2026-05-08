---
title: "外部命令处理组件"
---

# External Command Process Component

> 源码路径：`modules/external_command/process_component/`

## 概述

外部命令处理组件是外部命令系统的入口 CyberRT Component。通过插件机制动态加载配置中指定的各类 `CommandProcessorBase` 实例，并提供统一的命令状态查询服务。

## 核心类

### ExternalCommandProcessComponent

```cpp
class ExternalCommandProcessComponent : public cyber::Component<> {
 public:
  bool Init() override;
 private:
  std::vector<std::shared_ptr<CommandProcessorBase>> command_processors_;
  std::shared_ptr<cyber::Service<CommandStatusRequest, CommandStatus>>
      command_status_service_;
};
```

## 核心函数

### ExternalCommandProcessComponent::Init()

```cpp
bool ExternalCommandProcessComponent::Init() {
  ProcessComponentConfig config;
  GetProtoConfig(&config);
  const auto& plugin_manager = cyber::plugin_manager::PluginManager::Instance();
  for (const auto& processor_class_name : config.processor()) {
    command_processors_.emplace_back(
        plugin_manager->CreateInstance<CommandProcessorBase>(
            processor_class_name));
    command_processors_.back()->Init(node_);
  }
  command_status_service_ = node_->CreateService<CommandStatusRequest, CommandStatus>(
      config.output_command_status_name(),
      [this](const auto& request, auto& response) {
        for (const auto& processor : command_processors_) {
          if (processor->GetCommandStatus(request->command_id(), response.get())) {
            return;
          }
        }
        response->set_status(CommandStatusType::UNKNOWN);
      });
  return true;
}
```

**职责**：初始化所有命令处理器并注册状态查询服务

**关键步骤**：

1. 读取 `ProcessComponentConfig` 配置，获取需要加载的处理器类名列表
2. 通过 `PluginManager` 动态创建各 `CommandProcessorBase` 实例并调用 `Init()`
3. 注册 `CommandStatusRequest` → `CommandStatus` 服务，遍历所有处理器查询命令状态

## 配置

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `processor` | repeated string | 要加载的命令处理器插件类名 |
| `output_command_status_name` | string | 状态查询服务名称 |

## 调用关系

- **被调用方**：CyberRT 框架启动时加载此 Component
- **管理**：动态加载 `LaneFollowCommandProcessor`、`ActionCommandProcessor` 等插件
- **依赖**：`cyber::plugin_manager::PluginManager`、`CommandProcessorBase`
