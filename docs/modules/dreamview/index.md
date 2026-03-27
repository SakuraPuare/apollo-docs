# Dreamview 可视化模块

Apollo Dreamview 是 Apollo 自动驾驶平台的 Web 可视化与人机交互（HMI）模块。它提供了一个基于浏览器的实时可视化界面，用于监控自动驾驶车辆的运行状态、调试算法输出、回放数据记录以及管理各功能模块的启停。Apollo 目前维护两个版本：经典版 **Dreamview** 和新一代 **Dreamview Plus**。

## 模块职责

### Dreamview（经典版）

经典版 Dreamview 位于 `modules/dreamview/`，承担以下核心职责：

- **仿真世界可视化**：通过 `SimulationWorldUpdater` 和 `SimulationWorldService` 将车辆位姿、规划轨迹、预测障碍物、决策信息、交通灯状态等聚合为 `SimulationWorld` protobuf 消息，经 WebSocket 推送至前端进行 3D 渲染。
- **高精地图服务**：`MapService` 加载 HDMap，根据车辆当前位置和可视半径（`FLAGS_sim_map_radius`）动态检索地图元素（车道线、路口、人行横道等），序列化后发送给前端。
- **点云可视化**：`PointCloudUpdater` 订阅激光雷达点云 Cyber 通道，进行坐标变换和体素滤波后，以二进制格式推送至前端。
- **感知相机画面**：`PerceptionCameraUpdater` 订阅相机图像通道，结合定位和外参标定信息，将图像与 2D 检测框叠加后推送至前端。
- **HMI 控制面板**：`HMI` / `HMIWorker` 管理模块启停、模式切换、地图切换、车辆配置切换、录制控制等操作，并将 `HMIStatus` 状态持续广播给前端。
- **仿真控制**：`SimControlManager` 提供 Sim Control 模式，支持加载动态模型、重置车辆位姿等仿真操作。
- **插件管理**：`PluginManager` 支持通过 Cyber 通道与外部插件通信，扩展 Dreamview 功能。

### Dreamview Plus（新一代）

Dreamview Plus 位于 `modules/dreamview_plus/`，在经典版基础上进行了架构升级：

- **统一的 Updater 管理体系**：引入 `UpdaterBase` → `UpdaterWithChannelsBase` 抽象基类和 `UpdaterManager` 管理器，所有数据流（simworld、camera、pointcloud、map、obstacle、hmistatus、channelsinfo）统一注册和管理，支持按需订阅/取消订阅。
- **SocketManager 订阅调度**：`SocketManager` 作为前端与后端之间的订阅调度中心，处理前端的 Subscribe/UnSubscribe 请求，动态管理数据流的启停和通道分配。
- **多通道数据源支持**：`UpdaterWithChannelsBase` 支持同一数据类型来自多个 Cyber 通道（如多个激光雷达、多个相机），前端可按通道名独立订阅。
- **障碍物独立更新器**：新增 `ObstacleUpdater`，独立于 SimulationWorld 直接订阅感知障碍物通道，提供更细粒度的障碍物可视化数据。
- **通道信息更新器**：`ChannelsUpdater` 订阅任意 Cyber 通道的原始消息，支持前端动态查看通道数据。
- **DvPlugin 插件系统**：`DvPluginBase` / `DvPluginManager` 提供基于 Cyber PluginManager 的插件扩展机制，插件可注册自定义 WebSocket Handler、HTTP Handler 和 Updater。
- **前端全面重构**：采用 React 18 + TypeScript + Lerna monorepo 架构，引入 Three.js 3D 渲染引擎、Web Worker 多线程解码、RxJS 响应式数据流管理。

## 核心类与接口

### 后端核心类

#### `Dreamview`（主入口类）

定义于 `modules/dreamview_plus/backend/dreamview.h`，是整个后端的顶层编排类。

```cpp
class Dreamview {
 public:
  apollo::common::Status Init();   // 初始化 CivetServer、WebSocket、各 Updater
  apollo::common::Status Start();  // 启动所有 Updater 和 HMI
  void Stop();                     // 停止所有服务
  void RegisterUpdaters();         // 向 UpdaterManager 注册所有 Updater

 private:
  std::unique_ptr<CivetServer> server_;           // HTTP/WebSocket 服务器
  std::unique_ptr<SimulationWorldUpdater> sim_world_updater_;
  std::unique_ptr<PointCloudUpdater> point_cloud_updater_;
  std::unique_ptr<PerceptionCameraUpdater> perception_camera_updater_;
  std::unique_ptr<MapUpdater> map_updater_;
  std::unique_ptr<ObstacleUpdater> obstacle_updater_;
  std::unique_ptr<ChannelsUpdater> channels_info_updater_;
  std::unique_ptr<HMI> hmi_;
  std::unique_ptr<MapService> map_service_;
  std::unique_ptr<SocketManager> socket_manager_;
  std::unique_ptr<UpdaterManager> updater_manager_;
  std::unique_ptr<DvPluginManager> dv_plugin_manager_;
  // 多个 WebSocketHandler 实例，分别绑定不同 URI
  std::unique_ptr<WebSocketHandler> websocket_;       // /websocket
  std::unique_ptr<WebSocketHandler> sim_world_ws_;    // /simworld
  std::unique_ptr<WebSocketHandler> map_ws_;          // /map
  std::unique_ptr<WebSocketHandler> point_cloud_ws_;  // /pointcloud
  std::unique_ptr<WebSocketHandler> camera_ws_;       // /camera
  std::unique_ptr<WebSocketHandler> obstacle_ws_;     // /obstacle
  std::unique_ptr<WebSocketHandler> hmi_ws_;          // /hmistatus
  std::unique_ptr<WebSocketHandler> plugin_ws_;       // /plugin
  std::unique_ptr<WebSocketHandler> socket_manager_ws_; // 主 WebSocket
  std::unique_ptr<WebSocketHandler> channels_info_ws_;  // /channelsinfo
};
```

`Init()` 方法的关键流程：
1. 初始化 `VehicleConfigHelper`
2. 创建 `CivetServer`，配置监听端口（默认 8888）、WebSocket 超时、SSL 等
3. 实例化 `MapService`、各 `WebSocketHandler`
4. 创建各 Updater 并调用 `RegisterUpdaters()` 注册到 `UpdaterManager`
5. 创建 `SocketManager` 和 `DvPluginManager`
6. 将 WebSocketHandler 绑定到 CivetServer 的对应 URI 路径

#### `WebSocketHandler`

定义于 `modules/dreamview/backend/common/handlers/websocket_handler.h`，继承自 CivetWeb 的 `CivetWebSocketHandler`。

```cpp
class WebSocketHandler : public CivetWebSocketHandler {
 public:
  using MessageHandler = std::function<void(const Json&, Connection*)>;
  using ConnectionReadyHandler = std::function<void(Connection*)>;

  bool BroadcastData(const std::string& data, bool skippable = false);
  bool BroadcastBinaryData(const std::string& data, bool skippable = false);
  bool SendData(Connection* conn, const std::string& data, bool skippable = false);
  void RegisterMessageHandler(std::string type, MessageHandler handler);
  void RegisterConnectionReadyHandler(ConnectionReadyHandler handler);
};
```

核心机制：
- 维护一个 `connections_` 连接池，每个连接有独立的互斥锁防止并发写入
- `message_handlers_` 按消息类型（JSON 中的 `type` 字段）分发处理
- 支持文本和二进制两种数据广播模式
- `skippable` 参数允许在连接繁忙时跳过非关键帧

#### `UpdaterBase` 与 `UpdaterWithChannelsBase`

```cpp
// 所有数据更新器的抽象基类
class UpdaterBase {
 public:
  virtual void StartStream(const double& time_interval_ms,
                           const std::string& channel_name = "",
                           nlohmann::json* subscribe_param = nullptr) = 0;
  virtual void StopStream(const std::string& channel_name = "") = 0;
  virtual void PublishMessage(const std::string& channel_name = "") = 0;
};

// 支持多通道的更新器基类
class UpdaterWithChannelsBase : public UpdaterBase {
 public:
  virtual void GetChannelMsg(std::vector<std::string>* channels) = 0;
  void GetChannelMsgWithFilter(std::vector<std::string>* channels,
                               const std::string& filter_message_type,
                               const std::string& filter_channel,
                               bool reverse_filter_channel = false);
};
```

#### `UpdaterManager`

```cpp
class UpdaterManager {
 public:
  void RegisterUpdater(std::string path_name, UpdaterBase* updater);
  bool Start(const std::string& path_name, const double& time_interval_ms,
             const std::string& channel_name = "",
             nlohmann::json* subscribe_param = nullptr);
  bool Stop(const std::string& path_name, const std::string& channel_name);
  UpdaterBase* GetUpdater(const std::string& path_name);
};
```

`UpdaterManager` 维护一个 `updater_map_`（`path_name → UpdaterBase*`），Dreamview Plus 在 `RegisterUpdaters()` 中注册了以下 Updater：

| path_name      | Updater 类                  | 数据类型                |
|----------------|-----------------------------|------------------------|
| `simworld`     | `SimulationWorldUpdater`    | 仿真世界聚合数据        |
| `hmistatus`    | `HMI`                      | HMI 状态               |
| `camera`       | `PerceptionCameraUpdater`   | 相机图像 + 2D 检测框    |
| `pointcloud`   | `PointCloudUpdater`         | 激光雷达点云            |
| `map`          | `MapUpdater`                | 高精地图元素            |
| `obstacle`     | `ObstacleUpdater`           | 感知障碍物              |
| `channelsinfo` | `ChannelsUpdater`           | 任意 Cyber 通道原始数据  |

#### `SimulationWorldService`

定义于 `modules/dreamview_plus/backend/simulation_world/simulation_world_service.h`，是仿真世界数据的核心聚合服务。

该类通过 Cyber Reader 订阅以下通道数据并聚合到 `SimulationWorld` protobuf 对象中：

- **定位**（`LocalizationEstimate`）：车辆位姿、速度、航向
- **底盘**（`Chassis`）：车速、档位、转向角、油门/刹车
- **感知障碍物**（`PerceptionObstacles`）：障碍物类型、位置、速度、多边形
- **预测**（`PredictionObstacles`）：障碍物预测轨迹
- **规划**（`ADCTrajectory`）：规划轨迹、决策信息、速度规划
- **规划命令**（`PlanningCommand`）：当前执行的规划命令
- **控制**（`ControlCommand`）：控制指令
- **路由**（`RoutingResponse`）：全局路径
- **交通灯**（`TrafficLightDetection`）：交通灯检测结果
- **GPS**（`Gps`）：GPS 原始数据
- **相对地图**（`MapMsg`）：导航模式下的相对地图
- **故事**（`Stories`）：场景故事信息
- **音频检测**（`AudioDetection`）：音频事件
- **任务管理**（`Task`）：任务状态

同时提供 Writer 用于发送路由请求、导航信息、任务命令等。

#### `SocketManager`

定义于 `modules/dreamview_plus/backend/socket_manager/socket_manager.h`，是 Dreamview Plus 的核心订阅调度器。

```cpp
class SocketManager {
 public:
  SocketManager(WebSocketHandler* websocket, UpdaterManager* updater_manager,
                DvPluginManager* dv_plugin_manager);
  void BrocastDataHandlerConf(bool clear_channel_msg = false);

 private:
  bool Subscribe(const Json& json);
  bool UnSubscribe(const Json& json);
  void RefreshChannels(const apollo::cyber::proto::ChangeMsg& change_msg);
};
```

核心功能：
- 接收前端的 Subscribe/UnSubscribe JSON 请求，调用 `UpdaterManager` 启停对应的数据流
- 监听 Cyber 拓扑变化（`ChangeMsg`），当通道增减时增量通知前端
- 管理 `DataHandlerConf`，维护数据类型与 WebSocket 路径、通道的映射关系

#### `MapService`

定义于 `modules/dreamview/backend/common/map_service/map_service.h`，提供高精地图的高层 API。

```cpp
class MapService {
 public:
  explicit MapService(bool use_sim_map = true);
  void CollectMapElementIds(const PointENU& point, double radius, MapElementIds* ids) const;
  hdmap::Map RetrieveMapElements(const MapElementIds& ids) const;
  bool ReloadMap(bool force_reload);
  bool ConstructLaneWayPoint(double x, double y, routing::LaneWaypoint* laneWayPoint) const;
  size_t CalculateMapHash(const MapElementIds& ids) const;
};
```

- 支持 `sim_map`（降采样地图，用于前端显示）和 `hdmap`（完整高精地图）两种模式
- `CollectMapElementIds` 根据车辆位置和半径收集可视范围内的地图元素 ID
- `CalculateMapHash` 用于判断地图是否变化，避免重复推送

#### `HMI` 与 `HMIWorker`

`HMI` 继承自 `UpdaterBase`，作为 HMI 状态的数据更新器，同时封装了 `HMIWorker` 的功能。

`HMIWorker` 是 HMI 操作的实际执行者，主要功能包括：
- 模块启停（`StartModule` / `StopModule`）
- 模式切换（`ChangeMode`）
- 地图切换（`ChangeMap`）
- 车辆配置切换（`ChangeVehicle`）
- 录制控制（`ChangeRecord`）
- 仿真控制（`ChangeScenario`、`ChangeDynamicModel`）
- 状态监控（订阅 `SystemStatus`、`Chassis`、`Localization`）

`HMIWorker` 通过 `StatusUpdateHandler` 回调机制，在状态变化时通知 `HMI`，由 `HMI` 通过 WebSocket 广播给前端。

#### `DvPluginBase` 与 `DvPluginManager`

```cpp
class DvPluginBase {
 public:
  virtual void Init() = 0;
  virtual void Run() = 0;
  virtual std::map<std::string, WebSocketHandler*> GetWebSocketHandlers();
  virtual std::map<std::string, CivetHandler*> GetHandlers();
  virtual std::map<std::string, UpdaterBase*> GetUpdaterHandlers();
  virtual DataHandlerConf GetDataHandlerConf();
};
```

`DvPluginManager` 基于 Cyber 的 `PluginManager` 自动发现和加载插件，将插件注册的 WebSocket Handler、HTTP Handler 和 Updater 集成到 Dreamview Plus 的服务体系中。

### 前端核心架构（Dreamview Plus）

Dreamview Plus 前端采用 Lerna monorepo 架构，主要包含以下子包：

| 包名 | 职责 |
|------|------|
| `dreamview-core` | 核心框架层：WebSocket 管理、数据流订阅、状态管理、Worker 线程池 |
| `dreamview-carviz` | 3D 可视化引擎：基于 Three.js 的车辆、障碍物、轨迹、地图渲染 |
| `dreamview-ui` | UI 组件库 |
| `dreamview-web` | 应用入口，组合各子包 |
| `dreamview-lang` | 国际化 |
| `dreamview-analysis` | 性能分析工具 |

#### `WebSocketManager`

前端 WebSocket 通信的核心管理器，负责：

- 维护主连接（`mainConnection`）和插件连接（`pluginConnection`）
- 通过 `ChildWsWorkerClass` 管理子 WebSocket Worker 线程，每个数据流在独立 Worker 中解码
- 使用 RxJS `BehaviorSubject` 管理元数据（metadata）和连接状态
- 支持 `subscribeToData` / `subscribeToDataWithChannel` 按数据名和通道名订阅
- 通过 `PluginManager` 支持消息处理插件（如 `MapMessageHandlerPlugin`）
- 数据帧率控制（默认 10fps，100ms 间隔）

#### `Carviz`（3D 渲染引擎）

`Carviz.class.ts` 是 3D 场景的核心类，管理 Three.js 场景、相机、渲染器，并协调各渲染组件：

- 障碍物渲染（`obstacles.d.ts`）
- 点云渲染（`pointCloud.d.ts`）
- 地图元素渲染（`map/`）
- 车辆模型渲染（`adc.d.ts`）
- 网格与文本标注（`grid/`、`text.d.ts`）
- 路由编辑（`RoutingEditor.class.ts`）

## 数据流

### 后端数据流

```
Cyber RT 通道                    后端 Updater                WebSocket              前端
─────────────────────────────────────────────────────────────────────────────────────────
/apollo/localization/pose    ┐
/apollo/planning             ├→ SimulationWorldUpdater ──→ /simworld ──→ 3D 场景渲染
/apollo/prediction           │   (聚合为 SimulationWorld)
/apollo/perception/obstacles ┘
/apollo/perception/obstacles ──→ ObstacleUpdater ────────→ /obstacle ──→ 障碍物面板
/apollo/sensor/lidar/*       ──→ PointCloudUpdater ──────→ /pointcloud → 点云渲染
/apollo/sensor/camera/*      ──→ PerceptionCameraUpdater → /camera ───→ 相机画面
HDMap 文件                   ──→ MapUpdater ─────────────→ /map ──────→ 地图渲染
HMIStatus                   ──→ HMI ────────────────────→ /hmistatus → 控制面板
任意通道                     ──→ ChannelsUpdater ────────→ /channelsinfo → 通道浏览
```

### 前端数据流（Dreamview Plus）

```
WebSocket 消息
    │
    ▼
WebSocketManager (主线程)
    │
    ├─→ ChildWsWorker (Web Worker) ──→ Protobuf 解码 ──→ BehaviorSubject
    │                                                        │
    │                                                        ▼
    └─→ PluginManager (消息拦截/转换)                    React 组件订阅
                                                             │
                                                             ▼
                                                    Carviz 3D 渲染 / UI 面板
```

### SocketManager 订阅调度流程

1. 前端通过主 WebSocket 发送 `Subscribe` 请求（包含 `dataName` 和可选 `channelName`）
2. `SocketManager` 解析请求，调用 `UpdaterManager::Start()` 启动对应 Updater
3. Updater 开始从 Cyber 通道读取数据，按设定频率序列化后通过对应 WebSocket 推送
4. 前端发送 `UnSubscribe` 时，`SocketManager` 调用 `UpdaterManager::Stop()` 停止推送

## 配置方式

### 启动配置

DAG 文件（`modules/dreamview_plus/dag/dreamview_plus.dag`）：

```protobuf
module_config {
    module_library : "modules/dreamview_plus/libdreamview_plus.so"
    components {
        class_name : "Dreamview"
        config {
            name: "dreamview_plus"
        }
    }
}
```

Launch 文件（`modules/dreamview_plus/launch/dreamview_plus.launch`）：

```xml
<cyber>
    <module>
        <name>dreamview_plus</name>
        <dag_conf>/apollo/modules/dreamview_plus/dag/dreamview_plus.dag</dag_conf>
        <process_name>dreamview_plus</process_name>
    </module>
</cyber>
```

### 关键 GFlags 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `server_ports` | `8888` | CivetServer HTTP 监听端口 |
| `sim_map_radius` | `200.0` | 前端地图可视半径（米） |
| `dreamview_data_handler_conf_path` | - | 数据处理器配置文件路径 |
| `dv_cpu_profile` | `false` | 启用 CPU 性能分析（gperftools） |
| `dv_heap_profile` | `false` | 启用堆内存分析（gperftools） |

### 前端配置

前端通过 `dreamview-web/config/` 目录下的配置文件管理构建参数，使用 `package.json` 中的 scripts 定义开发和构建命令。
