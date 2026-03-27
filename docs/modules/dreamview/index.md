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

- **统一的 Updater 管理体系**：引入 `UpdaterBase` / `UpdaterWithChannelsBase` 抽象基类和 `UpdaterManager` 管理器，所有数据流（simworld、camera、pointcloud、map、obstacle、hmistatus、channelsinfo）统一注册和管理，支持按需订阅与取消订阅。
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
  std::unique_ptr<WebSocketHandler> websocket_;         // /websocket
  std::unique_ptr<WebSocketHandler> sim_world_ws_;      // /simworld
  std::unique_ptr<WebSocketHandler> map_ws_;            // /map
  std::unique_ptr<WebSocketHandler> point_cloud_ws_;    // /pointcloud
  std::unique_ptr<WebSocketHandler> camera_ws_;         // /camera
  std::unique_ptr<WebSocketHandler> obstacle_ws_;       // /obstacle
  std::unique_ptr<WebSocketHandler> hmi_ws_;            // /hmi
  std::unique_ptr<WebSocketHandler> plugin_ws_;         // /plugin
  std::unique_ptr<WebSocketHandler> socket_manager_ws_; // /socketmanager
  std::unique_ptr<WebSocketHandler> channels_info_ws_;  // /channelsinfo
  std::unique_ptr<ImageHandler> image_;
  std::unique_ptr<ProtoHandler> proto_handler_;
  std::unique_ptr<PluginManager> plugin_manager_;
  std::unique_ptr<cyber::Timer> exit_timer_;
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

`UpdaterManager` 维护一个 `updater_map_`（`path_name` 到 `UpdaterBase*` 的映射），Dreamview Plus 在 `RegisterUpdaters()` 中注册了以下 Updater：

| path_name      | Updater 类                  | 数据类型                |
|----------------|-----------------------------|------------------------|
| `simworld`     | `SimulationWorldUpdater`    | 仿真世界聚合数据        |
| `hmistatus`    | `HMI`                      | HMI 状态               |
| `camera`       | `PerceptionCameraUpdater`   | 相机图像 + 2D 检测框    |
| `pointcloud`   | `PointCloudUpdater`         | 激光雷达点云            |
| `map`          | `MapUpdater`                | 高精地图元素            |
| `obstacle`     | `ObstacleUpdater`           | 感知障碍物              |
| `channelsinfo` | `ChannelsUpdater`           | 任意 Cyber 通道原始数据（注：`ChannelsUpdater` 直接继承 `UpdaterBase` 而非 `UpdaterWithChannelsBase`，通过自身的 RawMessage 机制支持订阅任意通道；其在 `data_handler.conf` 中的 `data_name` 为 `"cyber"`，`channelsinfo` 是 websocket_name/路径） |

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
- **交通灯**（`TrafficLightDetection`）：交通灯检测结果
- **GPS**（`Gps`）：GPS 原始数据
- **相对地图**（`MapMsg`）：导航模式下的相对地图
- **故事**（`Stories`）：场景故事信息
- **音频检测**（`AudioDetection`）：音频事件
- **任务管理**（`Task`）：任务状态
- **导航信息**（`NavigationInfo`）
- **驾驶事件**（`DriveEvent`）
- **监控消息**（`MonitorMessage`）

同时提供 Writer/Client 用于发送路由请求（`LaneFollowCommand`、`ValetParkingCommand`、`ActionCommand`）、导航信息、任务命令和路由响应（`RoutingResponse`）。

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
  void CollectMapElementIds(const PointENU& point, double radius,
                            MapElementIds* ids) const;
  hdmap::Map RetrieveMapElements(const MapElementIds& ids) const;
  bool ReloadMap(bool force_reload);
  bool ConstructLaneWayPoint(double x, double y,
                             routing::LaneWaypoint* laneWayPoint) const;
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
- 录制控制（`ChangeRecord`、`StartDataRecorder`、`StopDataRecorder`）
- 仿真控制（`ChangeDynamicModel`、`ChangeOperation`）
- 状态监控（订阅 `SystemStatus`、`Chassis`、`Localization`）

`HMIWorker` 通过 `StatusUpdateHandler` 回调机制，在状态变化时通知 `HMI`，由 `HMI` 通过 WebSocket 广播给前端。

#### `SimControlManager`

定义于 `modules/dreamview/backend/common/sim_control_manager/sim_control_manager.h`，以单例模式管理仿真控制。

```cpp
class SimControlManager {
 public:
  bool IsEnabled() const;
  nlohmann::json LoadDynamicModels();
  bool AddDynamicModel(const std::string& dynamic_model_name);
  bool ChangeDynamicModel(const std::string& dynamic_model_name);
  bool DeleteDynamicModel(const std::string& dynamic_model_name);
  void Restart(double x, double y, double v = 0.0, double a = 0.0);
  bool Init(bool set_start_point, double start_velocity = 0.0,
            double start_acceleration = 0.0, double start_heading = ...);
  void Start();
  void Stop();
};
```

通过 `DynamicModelFactory` 支持多种动态模型的热加载和切换。

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
| `dreamview-theme` | 主题系统 |
| `dreamview-analysis` | 性能分析工具 |
| `dreamview-log` | 日志系统 |

#### `WebSocketManager`

定义于 `packages/dreamview-core/src/services/WebSocketManager/websocket-manager.service.ts`，是前端与后端通信的核心管理器。

- 维护主连接（`mainConnection`）和插件连接（`pluginConnection`）
- 通过 `ChildWsWorkerClass` 在 Web Worker 中建立子 WebSocket 连接，避免主线程阻塞
- 使用 `DecoderWorkerClass` 在 Worker 中进行 protobuf 解码
- 使用 RxJS `BehaviorSubject` 管理元数据（metadata）和数据流订阅
- 支持 `subscribeToData` / `subscribeToDataWithChannel` 按数据名和通道名订阅
- 通过 `PluginManager` 支持消息处理插件（如 `MapMessageHandlerPlugin`）
- 内置节流控制（`throttleDuration`），默认 10fps（100ms 间隔），根据性能动态调整
- 启动时预加载核心 proto 文件（`hmi_status.proto`、`geometry.proto` 等）

#### `Carviz`（3D 渲染引擎）

定义于 `packages/dreamview-carviz/src/Carviz.class.ts`，是 3D 可视化渲染的核心类。基于 Three.js 构建，管理以下渲染子模块：

| 子模块 | 文件 | 职责 |
|--------|------|------|
| `View` | `render/view.ts` | 视角控制（俯视、跟随、自由视角） |
| `Map` | `render/map/` | 高精地图元素渲染（车道线、路口、停止线等） |
| `Adc` | `render/adc.ts` | 自动驾驶车辆 3D 模型 |
| `Obstacles` | `render/obstacles.ts` | 障碍物立方体、多边形渲染 |
| `PointCloud` | `render/pointCloud.ts` | 激光雷达点云渲染 |
| `Routing` | `render/routing.ts` | 路由路径渲染 |
| `Decision` | `render/decision.ts` | 决策标记（停车、跟随、让行、超车） |
| `Prediction` | `render/prediction.ts` | 预测轨迹渲染 |
| `Planning` | `render/planning.ts` | 规划轨迹渲染 |
| `Gps` | `render/gps.ts` | GPS 位置标记 |
| `Follow` | `render/follow.ts` | 相机跟随逻辑 |
| `Coordinates` | `render/coordinates.ts` | 坐标轴渲染 |
| `Text` | `render/text.ts` | 2D 文本标签（CSS2DRenderer） |

`Carviz` 还集成了交互功能模块：

- `InitiationMarker` / `PathwayMarker`：路由编辑起终点标记
- `CopyMarker`：坐标复制
- `RulerMarker`：测距工具
- `IndoorLocalizationMarker`：室内定位标记
- `RoutingEditor`：路由编辑器

渲染引擎使用的颜色映射体系包括：

- 障碍物类型颜色（行人黄色、自行车青色、车辆绿色等）
- 决策标记颜色（停车红色、跟随绿色、让行粉色、超车蓝色）
- 点云高度颜色映射（按高度从红到紫渐变）

## 数据流

### 后端数据流总览

```
Cyber RT 通道                    后端 Updater                WebSocket         前端
──────────────────────────────────────────────────────────────────────────────────────
/apollo/localization/pose    ┐
/apollo/planning             ├─→ SimulationWorldUpdater ──→ /simworld ──→ 3D 场景渲染
/apollo/prediction           │   (聚合为 SimulationWorld)
/apollo/perception/obstacles ┘
/apollo/perception/obstacles ───→ ObstacleUpdater ────────→ /obstacle ──→ 障碍物面板
/apollo/sensor/lidar/*       ───→ PointCloudUpdater ──────→ /pointcloud → 点云渲染
/apollo/sensor/camera/*      ───→ PerceptionCameraUpdater → /camera ───→ 相机画面
HDMap 文件                   ───→ MapUpdater ─────────────→ /map ──────→ 地图渲染
HMIWorker                   ───→ HMI ────────────────────→ /hmistatus → 控制面板
任意 Cyber 通道              ───→ ChannelsUpdater ────────→ /channelsinfo → 通道浏览
```

### 前端数据流（Dreamview Plus）

```
WebSocket 消息
    |
    v
WebSocketManager (主线程)
    |
    +---> ChildWsWorker (Web Worker) ---> Protobuf 解码 ---> BehaviorSubject
    |                                                            |
    |                                                            v
    +---> PluginManager (消息拦截/转换)                     React 组件订阅
                                                                 |
                                                                 v
                                                        Carviz 3D 渲染 / UI 面板
```

### SimulationWorldService 数据聚合流程

1. 构造时创建多个 `cyber::Reader`，分别订阅定位、底盘、感知、预测、规划、控制、路由、交通灯等通道
2. 每个 Reader 收到消息后缓存最新数据
3. `Update()` 方法被 `SimulationWorldUpdater` 的定时器周期性调用
4. `Update()` 依次调用 `UpdateWithLatestObserved()` 读取各 Reader 最新数据
5. 各 `UpdateSimulationWorld()` 模板特化函数将不同类型的数据写入 `SimulationWorld` 对象
6. 同时更新模块延迟（`UpdateDelays`）和处理延迟（`UpdateLatencies`）
7. 调用 `PopulateMapInfo()` 填充可视范围内的地图元素信息
8. `GetWireFormatString()` 将聚合后的 `SimulationWorld` 序列化为 protobuf wire format

### SocketManager 订阅调度流程

1. 前端通过主 WebSocket（`/socketmanager`）发送 `Subscribe` 请求，包含 `dataName` 和可选的 `channelName`
2. `SocketManager` 解析请求，查找 `DataHandlerConf` 获取对应的 Updater 路径
3. 调用 `UpdaterManager::Start()` 启动对应 Updater 的 `StartStream()`
4. Updater 创建 `cyber::Timer`，按设定频率从 Cyber 通道读取数据，序列化后通过对应 WebSocket 推送
5. 前端发送 `UnSubscribe` 时，`SocketManager` 调用 `UpdaterManager::Stop()` 停止推送
6. `SocketManager` 还监听 Cyber 拓扑变化，当有新通道上线或下线时，通过 `RefreshChannels` 增量更新前端

### 点云数据流

`PointCloudUpdater` 的数据处理管线：

1. 订阅指定的点云 Cyber 通道（如 `/apollo/sensor/lidar128/compensator/PointCloud2`）
2. 收到 `drivers::PointCloud` 消息后，转换为 PCL 点云格式
3. 通过 TF Buffer 获取激光雷达到世界坐标系的变换矩阵
4. 执行坐标变换（`TransformPointCloud`）
5. 可选的体素滤波（`FilterPointCloud`，由 `FLAGS_voxel_filter_size` 控制）
6. 序列化为紧凑的二进制格式
7. 通过 `WebSocketHandler::BroadcastBinaryData()` 推送

### 相机数据流

`PerceptionCameraUpdater` 的数据处理管线：

1. 订阅相机图像通道和对应的感知障碍物通道
2. 订阅定位通道，维护时间戳对齐的定位队列
3. 收到图像后，查找最近时刻的定位数据
4. 通过 TF Buffer 查询相机到车辆坐标系的静态变换
5. 将定位信息和变换矩阵组合为 `CameraUpdate` protobuf
6. 叠加 2D 检测框（BBox2D）信息
7. 通过 WebSocket 推送图像和标注数据

## 配置方式

### 启动方式

Dreamview 和 Dreamview Plus 均以独立二进制进程启动，不使用 DAG 文件。

**经典版 Dreamview 启动**：

```bash
dreamview --flagfile=/apollo/modules/dreamview/conf/dreamview.conf
```

Launch 文件 `modules/dreamview/launch/dreamview.launch`：

```xml
<cyber>
    <module>
        <name>dreamview</name>
        <dag_conf></dag_conf>
        <type>binary</type>
        <process_name>
            dreamview --flagfile=/apollo/modules/dreamview/conf/dreamview.conf
        </process_name>
        <exception_handler>respawn</exception_handler>
    </module>
</cyber>
```

**Dreamview Plus 启动**：

```bash
dreamview_plus --flagfile=/apollo/modules/dreamview_plus/conf/dreamview.conf
```

Launch 文件 `modules/dreamview_plus/launch/dreamview_plus.launch`：

```xml
<cyber>
    <module>
        <name>dreamview_plus</name>
        <dag_conf></dag_conf>
        <type>binary</type>
        <process_name>
            dreamview_plus --flagfile=/apollo/modules/dreamview_plus/conf/dreamview.conf
        </process_name>
        <exception_handler>respawn</exception_handler>
    </module>
</cyber>
```

### GFlags 配置参数

以下列出关键的 GFlags 参数（定义于 `modules/dreamview/backend/common/dreamview_gflags.h`）：

| 参数 | 说明 |
|------|------|
| `server_ports` | CivetServer HTTP 监听端口，Dreamview Plus 默认 `8888` |
| `static_file_dir` | 前端静态文件目录，Dreamview Plus 指向 `frontend/dist` |
| `websocket_timeout_ms` | WebSocket 连接超时时间 |
| `request_timeout_ms` | HTTP 请求超时时间 |
| `ssl_certificate` | SSL 证书路径（可选） |
| `sim_map_radius` | 前端地图可视半径（米） |
| `routing_from_file` | 是否从文件读取初始路由 |
| `voxel_filter_size` | 点云体素滤波尺寸 |
| `voxel_filter_height` | 点云高度滤波阈值 |
| `dreamview_profiling_mode` | 是否启用性能分析模式 |
| `dreamview_profiling_duration` | 性能分析模式运行时长（毫秒） |
| `dv_cpu_profile` | 启用 CPU 性能分析（gperftools，仅 Dreamview Plus） |
| `dv_heap_profile` | 启用堆内存分析（gperftools，仅 Dreamview Plus） |
| `default_hmi_mode` | 默认 HMI 模式，Dreamview Plus 默认 `Default` |
| `data_handler_config_path` | DataHandler 配置文件路径 |
| `plugin_path` | 插件搜索路径 |
| `vehicles_config_path` | 车辆配置文件路径 |
| `maps_data_path` | 地图数据路径 |
| `status_publish_interval` | HMI 状态发布间隔 |
| `monitor_timeout_threshold` | Monitor 超时阈值 |

### Dreamview Plus 配置文件

`modules/dreamview_plus/conf/dreamview.conf` 示例：

```
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--static_file_dir=/apollo/modules/dreamview_plus/frontend/dist
--default_data_collection_config_path=/apollo/modules/dreamview_plus/conf/data_collection_table.pb.txt
--default_preprocess_config_path=/apollo/modules/dreamview_plus/conf/preprocess_table.pb.txt
--data_handler_config_path=/apollo/modules/dreamview_plus/conf/data_handler.conf
--vehicle_data_config_filename=/apollo/modules/dreamview_plus/conf/vehicle_data.pb.txt
--default_hmi_mode=Default
--server_ports=8888
```

### DataHandler 配置

`modules/dreamview_plus/conf/data_handler.conf` 定义了数据处理器与 WebSocket 路径的映射关系：

```protobuf
data_handler_info {
  key: "apollo.dreamview.SimulationWorld"
  value {
    data_name: "simworld"
    msg_type: "apollo.dreamview.SimulationWorld"
    websocket_info {
      websocket_name: "simworld"
      websocket_pipe: "/simworld"
    }
  }
}
data_handler_info {
  key: "apollo.dreamview.CameraUpdate"
  value {
    data_name: "camera"
    msg_type: "apollo.dreamview.CameraUpdate"
    websocket_info {
      websocket_name: "camera"
      websocket_pipe: "/camera"
    }
    different_for_channels: true
  }
}
// ... 点云、HMI状态、地图、障碍物、Cyber通道等
```

`different_for_channels: true` 表示该数据类型支持多通道，前端可按通道名独立订阅。

### HMI 模式配置

`modules/dreamview_plus/conf/hmi_modes/` 目录下包含不同 HMI 模式的配置文件：

| 文件 | 模式 |
|------|------|
| `default.pb.txt` | 默认模式（全功能） |
| `perception.pb.txt` | 感知调试模式 |
| `pnc.pb.txt` | 规划与控制调试模式 |
| `vehicle_test.pb.txt` | 车辆测试模式 |

每个模式文件使用 `HMIMode` protobuf 格式，定义：

- `modules`：可启停的非 Cyber 模块（使用 start/stop command）
- `cyber_modules`：可启停的 Cyber 模块（使用 dag 文件）
- `operations`：支持的操作模式（如 `Record`、`Sim_Control`）
- `default_operation`：默认操作模式

## 前后端架构分析

### 后端架构

后端采用 C++ 实现，基于以下技术栈：

**HTTP/WebSocket 服务器 -- CivetWeb**

- 使用 `CivetServer`（CivetWeb 的 C++ 封装）作为嵌入式 HTTP/WebSocket 服务器
- 同时提供静态文件服务（前端 HTML/JS/CSS）和 WebSocket 服务
- 支持 SSL/TLS、Keep-Alive、TCP_NODELAY 等网络优化

**消息中间件 -- Cyber RT**

- 通过 `cyber::Reader` 订阅各模块的输出通道
- 通过 `cyber::Writer` 发布命令和状态消息
- 通过 `cyber::Client` 发送服务请求（如路由命令）
- 使用 `cyber::Timer` 实现定时推送

**序列化 -- Protocol Buffers + JSON**

- 模块间通信使用 protobuf 二进制格式
- 与前端通信使用 protobuf wire format 或 JSON（通过 `nlohmann::json`）
- 点云等大数据使用自定义二进制格式

**Dreamview 与 Dreamview Plus 后端架构对比**

| 特性 | Dreamview | Dreamview Plus |
|------|-----------|---------------|
| WebSocket 端点 | 5 个（websocket、map、pointcloud、camera、plugin） | 10 个（新增 simworld、hmi、socketmanager、obstacle、channelsinfo） |
| 数据流管理 | 各 Updater 独立管理 | UpdaterManager 统一注册和调度 |
| 前端订阅 | 隐式（连接即订阅） | 显式（Subscribe/UnSubscribe 请求） |
| 插件系统 | PluginManager（Cyber 通道通信） | DvPluginManager（Cyber 插件加载 + WebSocket/HTTP 注册） |
| 性能分析 | 支持 profiling mode | 额外集成 gperftools CPU/Heap profiler |

### 前端架构

**经典版 Dreamview 前端**

- 位于 `modules/dreamview/frontend/`
- 使用 Webpack 构建
- 基于原生 JavaScript / React
- 使用 protobuf.js 进行 proto 解码

**Dreamview Plus 前端**

- 位于 `modules/dreamview_plus/frontend/`
- 技术栈：React 18 + TypeScript + Lerna monorepo
- 构建工具：Webpack 5
- 3D 渲染：Three.js + CSS2DRenderer
- 状态管理：RxJS（BehaviorSubject / Observable）
- 数据通信：WebSocket + Web Worker 多线程
- UI 文档：Storybook
- 代码规范：ESLint + Stylelint + Prettier

**前端多线程架构**

Dreamview Plus 前端通过 Web Worker 实现多线程数据处理：

1. **主线程**：React UI 渲染、Three.js 场景渲染、用户交互处理
2. **ChildWsWorker**：在独立 Worker 中建立子 WebSocket 连接，接收高频数据流（点云、相机）
3. **DecoderWorker**：在 Worker 中执行 protobuf 解码，避免主线程阻塞
4. **WorkerPoolManager**：管理 Worker 线程池，支持任务优先级队列和空闲回收

## 可视化数据源和渲染管线

### 数据源清单

| 数据源 | Cyber 通道 | protobuf 类型 | 前端渲染目标 |
|--------|-----------|---------------|-------------|
| 车辆位姿 | `/apollo/localization/pose` | `LocalizationEstimate` | 车辆 3D 模型位置和朝向 |
| 底盘信息 | `/apollo/canbus/chassis` | `Chassis` | 车速、档位、方向盘角度显示 |
| 感知障碍物 | `/apollo/perception/obstacles` | `PerceptionObstacles` | 3D 边界框、多边形、类型标签 |
| 预测轨迹 | `/apollo/prediction` | `PredictionObstacles` | 预测轨迹曲线 |
| 规划轨迹 | `/apollo/planning` | `ADCTrajectory` | 规划轨迹曲线、速度曲线 |
| 控制命令 | `/apollo/control` | `ControlCommand` | 转向、油门、刹车指示 |
| 路由路径 | `/apollo/routing_response` | `RoutingResponse` | 全局路由路径渲染 |
| 交通灯 | `/apollo/perception/traffic_light` | `TrafficLightDetection` | 交通灯状态指示 |
| 点云 | `/apollo/sensor/lidar/*/PointCloud2` | `drivers::PointCloud` | 3D 点云散点图 |
| 相机图像 | `/apollo/sensor/camera/*/image` | `drivers::Image` | 2D 相机画面 + 检测框叠加 |
| 高精地图 | HDMap 文件 | `hdmap::Map` | 车道线、路口、人行横道等地图元素 |
| GPS | `/apollo/sensor/gnss/best_pose` | `Gps` | GPS 位置标记 |

### 渲染管线

Dreamview Plus 的 3D 渲染管线基于 Three.js，整体流程如下：

**1. 场景初始化**

```
Carviz 构造
    |
    +---> THREE.Scene (场景)
    +---> THREE.PerspectiveCamera (透视相机)
    +---> THREE.WebGLRenderer (WebGL 渲染器)
    +---> CSS2DRenderer (2D 标签渲染器)
    +---> OrbitControls (轨道控制器)
    +---> 各渲染子模块初始化
```

**2. 数据更新循环**

```
WebSocketManager 推送数据
    |
    v
数据解码 (Worker)
    |
    v
React 组件接收 (BehaviorSubject)
    |
    v
Carviz 数据更新方法
    |
    +---> map.update(mapData)         // 更新地图元素
    +---> adc.update(adcData)         // 更新车辆位姿
    +---> obstacles.update(objData)   // 更新障碍物
    +---> planning.update(planData)   // 更新规划轨迹
    +---> prediction.update(predData) // 更新预测轨迹
    +---> decision.update(decData)    // 更新决策标记
    +---> pointCloud.update(pcData)   // 更新点云
    +---> routing.update(routeData)   // 更新路由
    +---> gps.update(gpsData)         // 更新 GPS 标记
```

**3. 渲染帧**

```
Carviz.render()
    |
    +---> view.setView()           // 更新相机视角（跟随车辆）
    +---> renderer.render(scene, camera)  // WebGL 渲染
    +---> CSS2DRenderer.render(scene, camera) // 2D 标签渲染
    |
    +---> 性能统计
          - render calls (渲染调用次数)
          - triangles (三角面片数)
          - frame count (帧计数)
```

**4. 障碍物渲染细节**

障碍物按类型使用不同颜色：

- 行人（PEDESTRIAN）：黄色 `#ffea00`
- 自行车（BICYCLE）：青色 `#00dceb`
- 车辆（VEHICLE）：绿色 `#00ff3c`
- 虚拟障碍物（VIRTUAL）：深红 `#800000`
- CIPV：橙色 `#ff9966`
- 锥桶（TRAFFICCONE）：橙红 `#e1601c`
- 未知（UNKNOWN）：紫色 `#a020f0`

**5. 点云渲染细节**

点云按高度使用彩虹色映射：

- 0.5m: 红色
- 1.0m: 橙色
- 1.5m: 黄色
- 2.0m: 绿色
- 2.5m: 蓝色
- 3.0m: 靛色
- 10.0m+: 紫色

**6. 决策标记渲染**

- STOP（停车）：红色 `#ff3030`
- FOLLOW（跟随）：绿色 `#1ad061`
- YIELD（让行）：粉色 `#ff30f7`
- OVERTAKE（超车）：蓝色 `#30a5ff`

### 前端经典版渲染管线

经典版 Dreamview 前端（`modules/dreamview/frontend/`）使用 Webpack 构建，渲染组件位于 `src/renderer/` 目录，采用类似的 Three.js 渲染架构但实现相对简单，没有 Web Worker 多线程和 RxJS 响应式管理。

## 源码目录结构

```
modules/dreamview/                          # 经典版 Dreamview
    backend/
        dreamview.h / dreamview.cc          # 主入口类
        common/
            handlers/                       # WebSocket/HTTP 处理器
                websocket_handler.h/.cc
                image_handler.h/.cc
                proto_handler.h/.cc
            map_service/                    # 高精地图服务
                map_service.h/.cc
            sim_control_manager/            # 仿真控制管理
            plugins/                        # 插件管理
            dreamview_gflags.h/.cc          # GFlags 定义
            teleop/                         # 远程操控（条件编译）
        hmi/                                # HMI 控制
        simulation_world/                   # 仿真世界服务
        perception_camera_updater/          # 相机更新器
        point_cloud/                        # 点云更新器
    frontend/                               # 前端（Webpack + JS）
    conf/                                   # 配置文件
    launch/                                 # Launch 文件
    proto/                                  # Proto 定义

modules/dreamview_plus/                     # Dreamview Plus
    backend/
        dreamview.h / dreamview.cc          # 主入口类（增强版）
        simulation_world/                   # 仿真世界服务和更新器
        perception_camera_updater/          # 相机更新器（多通道）
        point_cloud/                        # 点云更新器（多通道）
        obstacle_updater/                   # 障碍物更新器（新增）
        channels_updater/                   # 通道信息更新器（新增）
        map/                                # 地图更新器（新增）
        hmi/                                # HMI（继承 UpdaterBase）
        socket_manager/                     # 订阅调度管理器（新增）
        updater/                            # Updater 基类和管理器（新增）
            updater_base.h/.cc
            updater_with_channels_base.h/.cc
            updater_manager.h/.cc
        dv_plugin/                          # DvPlugin 插件系统（新增）
            dv_plugin_base.h/.cc
            dv_plugin_manager.h/.cc
        record_player/                      # 录制回放
    frontend/                               # 前端（React 18 + TS + Lerna）
        packages/
            dreamview-core/                 # 核心框架
            dreamview-carviz/               # 3D 渲染引擎
            dreamview-ui/                   # UI 组件库
            dreamview-web/                  # 应用入口
            dreamview-lang/                 # 国际化
            dreamview-theme/                # 主题
            dreamview-analysis/             # 性能分析
            dreamview-log/                  # 日志
    conf/                                   # 配置文件
        hmi_modes/                          # HMI 模式配置
        data_handler.conf                   # DataHandler 配置
    launch/                                 # Launch 文件
    proto/                                  # Proto 定义
        data_handler.proto                  # DataHandler 协议定义
        obstacle.proto                      # 障碍物协议定义
```
