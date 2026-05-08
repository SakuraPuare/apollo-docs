---
title: "V2X 代理"
---

# V2X 代理

> 源码路径: `modules/v2x/v2x_proxy/`

## 概述

V2X Proxy 是车辆与路侧设备（RSU/OBU）之间的通信代理模块。通过 gRPC 接口与 OBU 设备通信，接收路侧信号灯（SPAT）、路侧信息（RSI）和障碍物数据，经过处理后发布到 Cyber RT 通道供感知和规划模块使用。同时将车辆状态信息上报给 OBU。

## 核心类

### V2xProxy

代理主类，管理 OBU/OS 接口、定时器和多个工作线程。

```cpp
class V2xProxy {
 public:
  explicit V2xProxy(std::shared_ptr<::apollo::hdmap::HDMap> hdmap = nullptr);
  ~V2xProxy();
  bool InitFlag();
  void stop();
  bool GetRsuListFromFile(const std::string& filename,
                          std::set<std::string>* whitelist);

 private:
  // 从 OBU 接收信号灯数据（独立线程循环运行）
  void RecvTrafficlight();
  // 接收规划消息
  void RecvOsPlanning();
  // 定时上报车辆状态到 OBU
  void OnV2xCarStatusTimer();

  std::unique_ptr<::apollo::cyber::Node> node_;
  std::unique_ptr<OsInterFace> os_interface_;          // Cyber RT 通道接口
  std::unique_ptr<ObuInterFaceGrpcImpl> obu_interface_; // gRPC OBU 接口
  std::unique_ptr<::apollo::cyber::Timer> v2x_car_status_timer_;
  std::unique_ptr<::apollo::cyber::Timer> obu_status_timer_;
  std::unique_ptr<::apollo::cyber::Timer> rsu_whitelist_timer_;

  // 工作线程
  std::unique_ptr<std::thread> recv_thread_;     // 信号灯接收
  std::unique_ptr<std::thread> planning_thread_; // 规划消息处理
  std::unique_ptr<std::thread> rsi_thread_;      // RSI 接收
  std::unique_ptr<std::thread> obs_thread_;      // 障碍物接收

  std::shared_ptr<InternalData> internal_;
  std::shared_ptr<::apollo::hdmap::HDMap> hdmap_;
  std::set<std::string> rsu_list_;  // RSU 白名单
};
```

### InternalData

内部数据处理类，负责信号灯和规划消息的核心逻辑。

```cpp
class InternalData final {
 public:
  InternalData();
  void reset();

  // 处理 OBU 信号灯数据，结合 HDMap 生成 OS 信号灯
  bool ProcTrafficlight(const std::shared_ptr<::apollo::hdmap::HDMap>& hdmap,
                        const ObuLight* x2v_traffic_light,
                        const std::string& junction_id, bool flag_u_turn,
                        double distance, double check_time,
                        std::shared_ptr<OSLight>* os_light);

  // 处理规划消息，生成感知信号灯检测结果
  bool ProcPlanningMessage(
      const ::apollo::planning::ADCTrajectory* planning_msg,
      const OSLight* last_os_light,
      std::shared_ptr<::apollo::perception::TrafficLightDetection>* res_light);
};
```

### utils 工具函数

```cpp
namespace utils {
// 查找 HDMap 中起止车道之间的所有道路 ID
bool FindAllRoadId(const std::shared_ptr<HDMap>& hdmap,
                   const LaneInfoConstPtr& start, const LaneInfoConstPtr& end,
                   size_t max_road_count, std::unordered_set<std::string>* result);

// 检查车辆是否在指定道路集合中
bool CheckCarInSet(const std::shared_ptr<HDMap>& hdmap,
                   const std::unordered_set<std::string>& id_set,
                   const LaneInfoConstPtr& car_laneinfo, size_t max_lane_count);

// 获取 RSU 信息（位置匹配、航向角过滤、白名单校验）
bool GetRsuInfo(const std::shared_ptr<HDMap>& hdmap,
                const OSLocation& os_location,
                const std::set<std::string>& rsu_whitelist,
                double distance, double max_heading_difference,
                std::shared_ptr<CarStatus>* v2x_car_status,
                std::string* out_junction_id, double* out_heading);

// 获取信号灯下一颜色
OSLightColor GetNextColor(OSLightColor color);
// 去重信号灯
void UniqueOslight(OSLight* os_light);
}  // namespace utils
```

## 调用关系

```text
V2xProxy (代理主程序，Cyber RT Node: "v2x_proxy")
├── 构造函数
│   ├── 加载 HDMap
│   ├── 创建 OsInterFace（Cyber RT 读写器）
│   ├── 创建 ObuInterFaceGrpcImpl（gRPC 客户端/服务端）
│   └── 启动工作线程（recv_thread_, planning_thread_, rsi_thread_, obs_thread_）
├── RecvTrafficlight()（循环线程）
│   ├── ObuInterFace::GetV2xTrafficLightFromObu() → gRPC 获取信号灯
│   ├── InternalData::ProcTrafficlight() → 处理并关联 HDMap
│   └── OsInterFace::SendV2xTrafficLightToOs() → 发布到 Cyber RT
├── OnV2xCarStatusTimer()（定时回调）
│   ├── utils::GetRsuInfo() → 获取当前 RSU 信息
│   └── ObuInterFace::SendCarStatusToObu() → 上报车辆状态
└── rsu_whitelist_timer_ → 定时刷新 RSU 白名单文件
```
