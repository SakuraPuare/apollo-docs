---
title: Mainboard - 主程序入口
---

# Mainboard - 主程序入口

## 模块职责概述

Mainboard 是 Apollo Cyber 框架的主进程入口程序，负责解析命令行参数、加载 DAG 配置文件、通过 ClassLoader 动态加载组件共享库并初始化所有组件。它是将 DAG 描述的模块拓扑转化为运行时进程的核心启动器。

## 核心类/接口说明

### main() 入口函数

`mainboard.cc` 中的 `main()` 函数定义了完整的启动和关闭流程：

```cpp
int main(int argc, char** argv) {
  // 1. 解析命令行参数
  ModuleArgument module_args;
  module_args.ParseArgument(argc, argv);

  // 2. 初始化 Cyber 框架
  apollo::cyber::Init(argv[0], dag_info);

  // 3. 注册信号处理（SIGTERM）
  std::signal(SIGTERM, [](int sig) {
    apollo::cyber::OnShutdown(sig);
    // 停止性能分析（如果启用）
  });

  // 4. 加载并初始化所有模块
  ModuleController controller(module_args);
  if (!controller.Init()) {
    controller.Clear();
    return -1;
  }

  // 5. 启动性能分析（如果启用）
  // 6. 等待关闭信号
  apollo::cyber::WaitForShutdown();

  // 7. 清理资源
  controller.Clear();
  return 0;
}
```

### ModuleArgument

命令行参数解析器，支持 DAG 配置文件、进程组、调度策略、插件和性能分析等参数。

```cpp
class ModuleArgument {
 public:
  void ParseArgument(int argc, char* const argv[]);
  void DisplayUsage();

  const std::string& GetBinaryName() const;
  const std::string& GetProcessGroup() const;
  const std::string& GetSchedName() const;
  const std::list<std::string>& GetDAGConfList() const;
  const std::list<std::string>& GetPluginDescriptionList() const;
  const bool GetEnableCpuprofile() const;
  const bool GetEnableHeapprofile() const;
  const bool& GetDisablePluginsAutoLoad() const;
};
```

支持的命令行参数：

| 参数 | 短选项 | 说明 |
|------|--------|------|
| `--dag_conf` | `-d` | DAG 配置文件路径（可多次指定） |
| `--process_name` | `-p` | 进程组名称 |
| `--sched_name` | `-s` | 调度策略名称 |
| `--plugin` | - | 插件描述文件路径 |
| `--disable_plugin_autoload` | - | 禁用插件自动加载 |
| `--cpuprofile` | `-c` | 启用 gperftools CPU 分析 |
| `--profile_filename` | `-o` | CPU 分析输出文件名 |
| `--heapprofile` | `-H` | 启用 gperftools 堆内存分析 |
| `--heapprofile_filename` | `-O` | 堆分析输出文件名 |
| `--help` | `-h` | 显示帮助信息 |

使用示例：

```bash
mainboard -d /path/to/perception.dag -d /path/to/planning.dag \
          -p perception_planning -s CYBER_DEFAULT
```

### ModuleController

模块控制器，负责 DAG 文件解析、共享库加载和组件初始化。

```cpp
class ModuleController {
 public:
  explicit ModuleController(const ModuleArgument& args);

  bool Init();    // 调用 LoadAll()
  bool LoadAll(); // 加载所有 DAG 中定义的模块
  void Clear();   // 关闭所有组件并卸载库

 private:
  bool LoadModule(const std::string& path);
  bool LoadModule(const DagConfig& dag_config);
  int GetComponentNum(const std::string& path);

  ModuleArgument args_;
  class_loader::ClassLoaderManager class_loader_manager_;
  std::vector<std::shared_ptr<ComponentBase>> component_list_;
};
```

## 数据流描述

### 启动流程

```
mainboard 进程启动
  │
  ├─ ModuleArgument::ParseArgument()
  │    解析 -d, -p, -s 等命令行参数
  │    设置 GlobalData 的 ProcessGroup 和 SchedName
  │
  ├─ cyber::Init()
  │    初始化日志、调度器、Transport 等基础设施
  │
  ├─ ModuleController::Init() → LoadAll()
  │    │
  │    ├─ 加载插件
  │    │    ├─ 加载命令行指定的插件描述文件
  │    │    └─ LoadInstalledPlugins()（除非 --disable_plugin_autoload）
  │    │
  │    ├─ 解析 DAG 配置文件
  │    │    ├─ 通过 APOLLO_DAG_PATH 环境变量查找 DAG 文件
  │    │    ├─ 统计组件总数（用于调度器资源分配）
  │    │    └─ 设置 GlobalData::SetComponentNums()
  │    │
  │    └─ 逐个加载模块 LoadModule(dag_config)
  │         │
  │         ├─ 通过 APOLLO_LIB_PATH 查找共享库
  │         ├─ ClassLoaderManager::LoadLibrary() 加载 .so
  │         ├─ 遍历 components 配置
  │         │    ├─ CreateClassObj<ComponentBase>(class_name)
  │         │    └─ base->Initialize(component_config)
  │         └─ 遍历 timer_components 配置
  │              ├─ CreateClassObj<ComponentBase>(class_name)
  │              └─ base->Initialize(timer_component_config)
  │
  ├─ cyber::WaitForShutdown()
  │    阻塞等待 SIGTERM 或 cyber::Shutdown()
  │
  └─ ModuleController::Clear()
       ├─ 逐个调用 component->Shutdown()
       └─ ClassLoaderManager::UnloadAllLibrary()
```

### DAG 配置文件格式

DAG 文件使用 protobuf text format，由 `dag_conf.proto` 定义：

```protobuf
message DagConfig {
  repeated ModuleConfig module_config = 1;
}

message ModuleConfig {
  optional string module_library = 1;        // 共享库路径
  repeated ComponentInfo components = 2;      // 普通组件列表
  repeated TimerComponentInfo timer_components = 3; // 定时器组件列表
}

message ComponentInfo {
  optional string class_name = 1;            // 类名（需与注册宏一致）
  optional ComponentConfig config = 2;        // 组件配置
}

message TimerComponentInfo {
  optional string class_name = 1;
  optional TimerComponentConfig config = 2;
}
```

DAG 配置示例：

```
module_config {
  module_library: "lib/libperception_component.so"
  components {
    class_name: "apollo::perception::PerceptionComponent"
    config {
      name: "perception"
      config_file_path: "conf/perception.conf"
      readers {
        channel: "/apollo/sensor/lidar/pointcloud"
      }
    }
  }
}
```

组件配置（`ComponentConfig`）：

```protobuf
message ComponentConfig {
  optional string name = 1;              // 组件名称
  optional string config_file_path = 2;  // 配置文件路径
  optional string flag_file_path = 3;    // gflag 文件路径
  repeated ReaderOption readers = 4;     // Reader 配置列表
}

message TimerComponentConfig {
  optional string name = 1;
  optional string config_file_path = 2;
  optional string flag_file_path = 3;
  optional uint32 interval = 4;          // 定时间隔（毫秒）
}
```

## 配置方式

### 环境变量

| 环境变量 | 说明 |
|----------|------|
| `APOLLO_DAG_PATH` | DAG 配置文件搜索路径 |
| `APOLLO_LIB_PATH` | 模块共享库搜索路径 |

### 默认值

| 参数 | 默认值 |
|------|--------|
| `process_group` | `mainboard_default` |
| `sched_name` | `CYBER_DEFAULT` |
| CPU profile 文件名 | `{process_group}_cpu.prof` |
| Heap profile 文件名 | `{process_group}_mem.prof` |

## 与其他模块的关系

- **ClassLoader**：`ModuleController` 内部持有 `ClassLoaderManager`，用于加载共享库和创建组件实例
- **PluginManager**：在 `LoadAll()` 中调用 `PluginManager::Instance()->LoadPlugin()` 和 `LoadInstalledPlugins()` 加载插件
- **Component**：所有被加载的组件必须继承 `ComponentBase`，并通过 `CLASS_LOADER_REGISTER_CLASS` 宏注册
- **Scheduler**：mainboard 通过 `-s` 参数指定调度策略，组件总数影响调度器的线程池大小
- **Init/State**：使用 `cyber::Init()` 初始化框架，`cyber::WaitForShutdown()` 等待退出信号
- **proto/dag_conf.proto**：定义了 DAG 配置文件的结构（`DagConfig`、`ModuleConfig`、`ComponentInfo`）
- **proto/component_conf.proto**：定义了组件配置结构（`ComponentConfig`、`TimerComponentConfig`、`ReaderOption`）
- **gperftools**：可选集成 CPU profiler 和 heap profiler，用于性能分析
