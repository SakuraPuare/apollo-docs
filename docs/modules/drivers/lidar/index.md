---
title: "激光雷达驱动"
---

# Lidar Driver Module

> 源码路径: `modules/drivers/lidar/`

## 概述

激光雷达驱动模块负责从多种品牌激光雷达硬件接收原始数据，解析为标准化点云 (`PointCloud`)，并通过 Cyber RT 通道发布。模块采用工厂模式统一管理不同品牌的驱动实现，支持 Velodyne、禾赛 (Hesai)、速腾 (Robosense)、Livox、镭神 (LSLidar)、华镭 (HSLidar)、Seyond、万集 (Vanjee) 等激光雷达设备。

模块包含三个子系统：驱动层（工厂模式创建品牌专属驱动，负责硬件通信与原始数据采集）、补偿器（基于 TF2 位姿查询做运动补偿）、融合器（将主/辅激光雷达点云合并为统一帧）。核心代码位于 `common/`（基类与工厂）、`compensator/`（运动补偿）、`fusion/`（主辅融合），各品牌驱动分布在 `velodyne/`、`hslidar/`、`rslidar/`、`livox/`、`lslidar/`、`seyond/`、`vanjeelidar/` 子目录中。

## 核心类

### LidarDriverComponent

顶层 Cyber RT 组件，是整个激光雷达模块的入口。继承自 `apollo::cyber::Component<>`，在 `Init()` 中加载配置、创建工厂、实例化具体品牌驱动。

```cpp
class LidarDriverComponent : public ::apollo::cyber::Component<> {
 public:
  bool Init() override;
 private:
  std::shared_ptr<LidarDriverFactory> lidar_factory_;
  apollo::drivers::lidar::config conf_;
  std::shared_ptr<::apollo::cyber::Node> node_;
  std::unique_ptr<LidarDriver> driver_;
};
```

### LidarDriver

所有品牌驱动的抽象基类，定义统一的初始化接口。各厂商驱动继承此类并实现 `Init()`。

```cpp
class LidarDriver {
 public:
  explicit LidarDriver(const std::shared_ptr<::apollo::cyber::Node>& node);
  virtual ~LidarDriver() = default;
  virtual bool Init() = 0;
 protected:
  std::shared_ptr<cyber::Node> node_;
};
```

### LidarDriverFactory

单例工厂类，基于 `apollo::common::util::Factory` 实现品牌枚举到驱动创建函数的映射。通过 `RegisterLidarClients()` 注册所有已知品牌，通过 `CreateLidarDriver()` 按配置创建对应驱动实例。

```cpp
class LidarDriverFactory
    : public apollo::common::util::Factory<
          LidarParameter::LidarBrand, LidarDriver, ...> {
 public:
  void RegisterLidarClients();
  std::unique_ptr<LidarDriver> CreateLidarDriver(
      const std::shared_ptr<::apollo::cyber::Node>& node,
      const apollo::drivers::lidar::config& parameter);
};
```

### LidarComponentBase / LidarComponentBaseImpl

厂商级别组件的模板基类。`LidarComponentBaseImpl<ScanType>` 提供 Scan 通道读写、PointCloud 分配与发布、ArenaQueue 共享内存支持等通用能力；`LidarComponentBase<ScanType>` 在其基础上封装 `InitBase()` 流程（初始化 Packet 和 Converter）。

```cpp
template <typename ScanType>
class LidarComponentBase : public LidarComponentBaseImpl<ScanType> {
 public:
  virtual bool Init() = 0;
  virtual void ReadScanCallback(
      const std::shared_ptr<ScanType>& scan_message) = 0;
  bool InitBase(const LidarConfigBase& lidar_config_base) override;
  std::shared_ptr<PointCloud> AllocatePointCloud();
  bool WritePointCloud(const std::shared_ptr<PointCloud>& point_cloud);
};
```

### Compensator

运动补偿器，通过 TF2 查询起止时刻的位姿，对点云中每个点做线性插值补偿。由 `CompensatorComponent` 包装为 Cyber RT 组件运行。

```cpp
class Compensator {
 public:
  explicit Compensator(const CompensatorConfig& config);
  bool MotionCompensation(
      const std::shared_ptr<const PointCloud>& msg,
      std::shared_ptr<PointCloud> msg_compensated);
 private:
  bool QueryPoseAffineFromTF2(const uint64_t& timestamp,
                               void* pose,
                               const std::string& child_frame_id);
  void GetTimestampInterval(const std::shared_ptr<const PointCloud>& msg,
                            uint64_t* timestamp_min,
                            uint64_t* timestamp_max);
};
```

### PriSecFusionComponent

主辅激光雷达融合组件。订阅多路点云通道，通过 TF2 坐标变换将辅雷达点云变换到主雷达坐标系后合并输出。

```cpp
class PriSecFusionComponent : public Component<PointCloud> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<PointCloud>& point_cloud) override;
 private:
  bool Fusion(std::shared_ptr<PointCloud> target,
              std::shared_ptr<PointCloud> source);
  bool QueryPoseAffine(const std::string& target_frame_id,
                       const std::string& source_frame_id,
                       Eigen::Affine3d* pose);
};
```

### SyncBuffering

线程安全的对象缓冲池模板类，通过原子索引实现无锁循环分配，支持自定义分配器和清理函数，用于 PointCloud 等高频对象的内存复用。关键接口：`SetBufferSize()`、`Init()`、`AllocateElement()`。

## 核心函数

| 函数 | 所属类 | 说明 |
| --- | --- | --- |
| `LidarDriverComponent::Init()` | LidarDriverComponent | 加载 Protobuf 配置，创建 Cyber 节点，通过工厂实例化品牌驱动并初始化 |
| `LidarDriverFactory::RegisterLidarClients()` | LidarDriverFactory | 向工厂注册所有品牌（Velodyne/Hesai/Robosense 等）的创建函数 |
| `LidarDriverFactory::CreateLidarDriver()` | LidarDriverFactory | 根据配置中的品牌枚举创建对应的 `LidarDriver` 子类实例 |
| `LidarComponentBaseImpl::InitConverter()` | LidarComponentBaseImpl | 创建 PointCloud Writer 和 Scan Reader，设置 frame_id 等参数 |
| `LidarComponentBaseImpl::InitPacket()` | LidarComponentBaseImpl | 当数据源为在线雷达时，创建 Scan Writer 用于发送原始扫描数据 |
| `LidarComponentBaseImpl::WritePointCloud()` | LidarComponentBaseImpl | 设置 header（frame_id、序列号、时间戳）后发布 PointCloud |
| `LidarComponentBaseImpl::AllocatePointCloud()` | LidarComponentBaseImpl | 从 Writer 获取预分配的 PointCloud 对象并预留点空间 |
| `Compensator::MotionCompensation()` | Compensator | 获取点云时间范围，查询 TF2 位姿，对点云逐点做运动补偿插值 |
| `PriSecFusionComponent::Fusion()` | PriSecFusionComponent | 查询主辅雷达间坐标变换，将源点云变换后追加到目标点云 |

## 配置

配置通过 Protobuf 定义，入口消息为 `config`，位于 `modules/drivers/lidar/proto/config.proto`。

```protobuf
message config {
    optional LidarParameter.LidarBrand brand = 1;  // 品牌选择
    optional apollo.drivers.hesai.Config hesai = 2; // 禾赛专属配置
    optional apollo.drivers.velodyne.Config velodyne = 3; // Velodyne 专属配置
}
```

品牌枚举 `LidarBrand`（VELODYNE/HESAI/ROBOSENSE）定义于 `lidar_parameter.proto`。

厂商组件共用基础配置 `LidarConfigBase`（`common/proto/lidar_config_base.proto`），关键字段如下：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `scan_channel` | string | Scan 数据通道名 |
| `point_cloud_channel` | string | PointCloud 输出通道名 |
| `frame_id` | string | 坐标系标识 |
| `source_type` | SourceType | `ONLINE_LIDAR`（在线直连）或 `RAW_PACKET`（回放） |
| `default_reserved_points_num` | uint32 | 预分配点数，默认 50000 |

配置文件示例位于 `modules/drivers/lidar/conf/lidar_config.pb.txt`，可同时配置多品牌参数，运行时根据 `brand` 字段选择对应配置块。

## 调用关系

```text
LidarDriverComponent::Init()
  ├── GetProtoConfig(&conf_)                 // 加载配置
  ├── cyber::CreateNode("drivers_lidar")     // 创建 Cyber 节点
  ├── LidarDriverFactory::RegisterLidarClients()  // 注册品牌
  └── LidarDriverFactory::CreateLidarDriver(node, conf_)
        └── 返回具体品牌 LidarDriver 子类
              └── LidarDriver::Init()         // 品牌驱动初始化

厂商组件 (如 VelodyneConvertComponent, RslidarComponent 等)
  继承 LidarComponentBase<ScanType>
    ├── InitBase(config)
    │     ├── InitPacket()   // 创建 Scan Writer（在线模式）
    │     └── InitConverter() // 创建 PointCloud Writer + Scan Reader
    ├── ReadScanCallback()    // 接收原始扫描，解析为点云
    ├── AllocatePointCloud()  // 获取预分配 PointCloud
    └── WritePointCloud()     // 发布标准化点云

CompensatorComponent::Proc(point_cloud)
  └── Compensator::MotionCompensation(msg, msg_compensated)
        ├── GetTimestampInterval()
        └── QueryPoseAffineFromTF2() + 逐点插值

PriSecFusionComponent::Proc(point_cloud)
  └── Fusion(target, source)
        ├── QueryPoseAffine()   // TF2 坐标变换
        └── AppendPointCloud()  // 点云追加合并
```
