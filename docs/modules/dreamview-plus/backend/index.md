---
title: "DreamView Plus 后端"
---

# DreamView Plus Backend

> 源码路径：`modules/dreamview_plus/backend/`

## 概述

DreamView Plus 后端服务，基于 CivetServer（HTTP/WebSocket 服务器）提供可视化界面的数据服务。管理多个 WebSocket 通道，实时推送仿真世界、障碍物、点云、地图、摄像头等数据到前端。

## 核心类

### Dreamview

```cpp
class Dreamview {
 public:
  apollo::common::Status Init();
  apollo::common::Status Start();
  void Stop();
  void RegisterUpdaters();
 private:
  std::unique_ptr<CivetServer> server_;
  std::unique_ptr<SimulationWorldUpdater> sim_world_updater_;
  std::unique_ptr<ObstacleUpdater> obstacle_updater_;
  std::unique_ptr<PointCloudUpdater> point_cloud_updater_;
  std::unique_ptr<PerceptionCameraUpdater> perception_camera_updater_;
  std::unique_ptr<MapUpdater> map_updater_;
  std::unique_ptr<ChannelsUpdater> channels_info_updater_;
  std::unique_ptr<HMI> hmi_;
  std::unique_ptr<MapService> map_service_;
  std::unique_ptr<DvPluginManager> dv_plugin_manager_;
  std::unique_ptr<SocketManager> socket_manager_;
  std::unique_ptr<UpdaterManager> updater_manager_;
};
```

## 子模块

| 子模块 | 目录 | 职责 |
| ------ | ---- | ---- |
| `simulation_world` | `simulation_world/` | 仿真世界数据聚合与推送 |
| `obstacle_updater` | `obstacle_updater/` | 障碍物数据更新 |
| `point_cloud` | `point_cloud/` | 点云数据处理与推送 |
| `perception_camera_updater` | `perception_camera_updater/` | 感知摄像头图像推送 |
| `map` | `map/` | 地图数据更新 |
| `hmi` | `hmi/` | 人机交互管理 |
| `channels_updater` | `channels_updater/` | 通道信息更新 |
| `dv_plugin` | `dv_plugin/` | 插件管理 |
| `socket_manager` | `socket_manager/` | WebSocket 连接管理 |

## 调用关系

- **入口**：`dreamview.cc` 中 `Init()` → `Start()` 启动 HTTP/WS 服务
- **依赖**：CivetServer（HTTP 服务器）、CyberRT（数据订阅）
- **前端**：通过 WebSocket 向 DreamView Plus 前端推送实时数据
