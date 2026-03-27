# Transform 坐标变换模块

> Apollo 坐标系变换服务 —— 管理传感器之间的静态/动态坐标变换关系，为感知、定位等模块提供统一的坐标查询接口。

## 模块职责

Transform 模块负责 Apollo 自动驾驶系统中各传感器坐标系之间的变换管理，核心职责包括：

1. **静态坐标变换发布**：从 YAML 外参标定文件中读取传感器之间的固定位姿关系（外参），以 Protobuf 消息形式发布到 `/tf_static` 通道
2. **变换缓存与查询**：订阅 `/tf` 和 `/tf_static` 通道，将变换数据缓存在 tf2 变换树中，供其他模块按帧 ID 和时间戳查询任意两个坐标系之间的变换
3. **变换广播**：为其他模块提供便捷的 `TransformBroadcaster` 接口，将动态变换发布到 `/tf` 通道

典型的坐标系层级关系如下（以默认配置为例）：

```
localization
├── novatel
│   └── velodyne64
│       ├── front_6mm
│       ├── front_12mm
│       └── radar_front
└── imu
```

## 核心类与接口

### BufferInterface（抽象接口）

定义在 `buffer_interface.h`，是坐标变换查询的纯虚接口，提供两类核心方法：

| 方法 | 说明 |
|------|------|
| `lookupTransform(target, source, time, timeout)` | 查询两个坐标系之间在指定时刻的变换 |
| `lookupTransform(target, target_time, source, source_time, fixed_frame, timeout)` | 通过固定参考帧查询不同时刻两个坐标系之间的变换 |
| `canTransform(target, source, time, timeout, errstr)` | 检测两个坐标系之间的变换是否可用 |
| `canTransform(target, target_time, source, source_time, fixed_frame, timeout, errstr)` | 通过固定参考帧检测变换是否可用 |
| `transform(in, out, target_frame, timeout)` | 模板方法，将数据从源坐标系变换到目标坐标系 |

`BufferInterface` 还提供了多个 `transform()` 模板重载，支持预分配输出、不同类型转换、以及通过固定帧的高级变换，内部调用 `tf2::doTransform` 和 `tf2::convert` 完成实际计算。

### Buffer（单例变换缓存）

定义在 `buffer.h` / `buffer.cc`，同时继承 `BufferInterface` 和 `tf2::BufferCore`，是模块的核心运行时组件。采用 `DECLARE_SINGLETON` 宏实现单例模式。

**初始化流程（`Init()`）：**

1. 创建名为 `transform_listener_<timestamp>` 的 Cyber 节点
2. 订阅 `/tf` 通道（动态变换）
3. 订阅 `/tf_static` 通道（静态变换，使用 `QOS_PROFILE_TF_STATIC` 保证可靠传输）

**消息回调（`SubscriptionCallbackImpl()`）：**

- 检测时间回跳：若当前时间早于上次更新时间，清空缓存并重新加载静态变换
- 将 Cyber Protobuf 消息转换为 `geometry_msgs::TransformStamped`（tf2 内部格式）
- 静态变换额外缓存到 `static_msgs_` 向量中
- 调用 `tf2::BufferCore::setTransform()` 写入变换树

**查询实现：**

- `lookupTransform()`：将 `cyber::Time` 转为 `tf2::Time`，委托 `tf2::BufferCore::lookupTransform()` 查询，再将结果从 tf2 格式转回 Cyber Protobuf 格式
- `canTransform()`：在超时时间内轮询 `tf2::BufferCore::canTransform()`，每 3ms 重试一次；仿真模式下会主动推进时钟
- `GetLatestStaticTF()`：从 `static_msgs_` 中反向查找指定帧对的最新静态变换

### StaticTransformComponent（静态变换组件）

定义在 `static_transform_component.h` / `static_transform_component.cc`，继承 `cyber::Component<>`，通过 `CYBER_REGISTER_COMPONENT` 宏注册为 Cyber 组件。

**初始化流程（`Init()`）：**

1. 从 Protobuf 文本配置文件加载 `apollo::static_transform::Conf`
2. 创建 `/tf_static` 通道的 Writer（使用 `QOS_PROFILE_TF_STATIC`）
3. 调用 `SendTransforms()` 一次性发布所有静态变换

**YAML 解析（`ParseFromYaml()`）：**

从外参标定 YAML 文件中提取以下字段：

```yaml
header:
  frame_id: "parent_frame"
child_frame_id: "child_frame"
transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:       # 四元数
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

**去重逻辑（`SendTransform()`）：**

发布前按 `child_frame_id` 去重 —— 若已存在相同子帧的变换则覆盖，否则追加。最终通过 Writer 将 `TransformStampeds` 消息写入通道。

### TransformBroadcaster（变换广播器）

定义在 `transform_broadcaster.h` / `transform_broadcaster.cc`，为其他模块提供发布动态变换的便捷接口。

- 构造时接收 `cyber::Node` 共享指针，创建 `/tf` 通道的 Writer
- 提供单条和批量 `SendTransform()` 方法

## 坐标系定义与变换算法

### 坐标变换原理

Transform 模块基于 tf2 库实现坐标变换，核心数学表示为刚体变换（Rigid Transformation），由平移向量和旋转四元数组成：

**变换表示：**

$$T_{parent \leftarrow child} = (t, q)$$

其中 $t = (x, y, z)$ 为平移向量，$q = (qx, qy, qz, qw)$ 为单位四元数表示的旋转。

**变换树查询：**

tf2 内部维护一棵以帧 ID 为节点的有向树。查询任意两帧之间的变换时：

1. 从 source_frame 沿树向上找到与 target_frame 的最近公共祖先（LCA）
2. 将 source → LCA 路径上的变换依次复合
3. 将 LCA → target 路径上的变换取逆后依次复合
4. 最终得到 $T_{target \leftarrow source}$

**时间插值：**

对于动态变换，tf2 在缓存窗口内按时间戳进行线性插值（平移分量）和球面线性插值 SLERP（旋转四元数），以获取任意时刻的变换。

**固定帧变换：**

高级查询 `lookupTransform(target, target_time, source, source_time, fixed_frame)` 用于处理不同时刻的变换：

$$T_{target \leftarrow source} = T_{target \leftarrow fixed}^{t_1} \cdot T_{fixed \leftarrow source}^{t_2}$$

即先在 $t_2$ 时刻从 source 变换到 fixed_frame，再在 $t_1$ 时刻从 fixed_frame 变换到 target。

### 消息格式转换

模块内部存在两套数据格式的转换：

| 方向 | 源格式 | 目标格式 | 实现位置 |
|------|--------|----------|----------|
| Cyber → tf2 | `apollo::transform::TransformStamped` (Protobuf) | `geometry_msgs::TransformStamped` | `Buffer::SubscriptionCallbackImpl()` |
| tf2 → Cyber | `geometry_msgs::TransformStamped` | `apollo::transform::TransformStamped` (Protobuf) | `Buffer::TF2MsgToCyber()` |

时间戳转换：Cyber 使用秒（`double`），tf2 使用纳秒（`uint64_t`），转换因子为 $10^9$。

## 数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                     配置加载阶段（启动时）                         │
│                                                                 │
│  static_transform_conf.pb.txt                                   │
│         │                                                       │
│         ▼                                                       │
│  StaticTransformComponent::Init()                               │
│         │                                                       │
│         ▼                                                       │
│  ParseFromYaml() ← 各传感器外参 YAML 文件                        │
│         │                                                       │
│         ▼                                                       │
│  Writer::Write() ──► /tf_static 通道                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                     运行时查询阶段                                │
│                                                                 │
│  /tf 通道 ─────────┐                                            │
│                     ▼                                           │
│  /tf_static 通道 ─► Buffer::SubscriptionCallbackImpl()          │
│                     │                                           │
│                     ▼                                           │
│              tf2::BufferCore（变换树缓存）                        │
│                     │                                           │
│                     ▼                                           │
│  其他模块调用 Buffer::Instance()->lookupTransform()              │
│              Buffer::Instance()->canTransform()                  │
│              Buffer::Instance()->transform()                     │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                     动态变换发布                                  │
│                                                                 │
│  其他模块（如 localization）                                      │
│         │                                                       │
│         ▼                                                       │
│  TransformBroadcaster::SendTransform()                          │
│         │                                                       │
│         ▼                                                       │
│  Writer::Write() ──► /tf 通道                                   │
└─────────────────────────────────────────────────────────────────┘
```

### 通道说明

| 通道名 | 消息类型 | 方向 | QoS | 说明 |
|--------|----------|------|-----|------|
| `/tf` | `TransformStampeds` | 输入/输出 | 默认 | 动态变换（如车辆实时位姿） |
| `/tf_static` | `TransformStampeds` | 输入/输出 | `QOS_PROFILE_TF_STATIC`（可靠传输） | 静态变换（传感器外参） |

## 配置方式

### 1. 静态变换配置（Protobuf 文本格式）

文件路径：`modules/transform/conf/static_transform_conf.pb.txt`

```protobuf
extrinsic_file {
    frame_id: "novatel"
    child_frame_id: "velodyne64"
    file_path: "/apollo/modules/drivers/lidar/velodyne/params/velodyne64_novatel_extrinsics.yaml"
    enable: true
}

extrinsic_file {
    frame_id: "localization"
    child_frame_id: "novatel"
    file_path: "/apollo/modules/localization/msf/params/novatel_localization_extrinsics.yaml"
    enable: true
}
```

每个 `extrinsic_file` 条目包含：

| 字段 | 类型 | 说明 |
|------|------|------|
| `frame_id` | string | 父坐标系 ID |
| `child_frame_id` | string | 子坐标系 ID |
| `file_path` | string | 外参 YAML 文件的绝对路径 |
| `enable` | bool | 是否启用该变换 |

### 2. 外参 YAML 文件格式

各传感器的外参标定结果以 YAML 格式存储，结构如下：

```yaml
header:
  frame_id: "novatel"
child_frame_id: "velodyne64"
transform:
  translation:
    x: 0.0
    y: 1.77
    z: 1.1
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

其中 `rotation` 为四元数表示（x, y, z, w），`translation` 为米制平移量。

### 3. DAG 配置

文件路径：`modules/transform/dag/static_transform.dag`

```
module_config {
    module_library : "modules/transform/libstatic_transform_component.so"
    components {
        class_name : "StaticTransformComponent"
        config {
            name : "static_transform"
            config_file_path: "/apollo/modules/transform/conf/static_transform_conf.pb.txt"
        }
    }
}
```

### 4. 启动方式

```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

### 5. 依赖项

| 依赖 | 说明 |
|------|------|
| `cyber` | Apollo 通信框架 |
| `tf2` | 坐标变换核心库（第三方） |
| `common-msgs` | Protobuf 消息定义（`TransformStamped` 等） |
| `yaml-cpp` | YAML 解析库 |
| `absl` | Google Abseil 基础库 |

### 6. 其他模块使用示例

其他模块查询坐标变换的典型用法：

```cpp
#include "modules/transform/buffer.h"

// 获取 Buffer 单例
auto tf_buffer = apollo::transform::Buffer::Instance();

// 查询 velodyne64 到 localization 坐标系的变换
auto transform = tf_buffer->lookupTransform(
    "localization", "velodyne64", cyber::Time(0));

// 检查变换是否可用
std::string err;
bool ok = tf_buffer->canTransform(
    "localization", "velodyne64", cyber::Time(0), 0.1f, &err);
```
