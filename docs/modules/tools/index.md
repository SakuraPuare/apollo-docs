---
title: "模块工具"
---

# Tools 模块

> 源码路径: `modules/tools/`

## 概述

`modules/tools/` 是 Apollo 自动驾驶系统的开发与调试工具集，包含 40 余个子工具，覆盖数据录制回放、3D 可视化、传感器标定、地图生成、信号模拟、记录分析等场景。工具以两种技术栈实现：

- **C++/Qt 可视化工具**：基于 Cyber RT 和 Qt OpenGL 实现 3D 点云、雷达、图像渲染，核心在 `visualizer/` 子目录
- **Python 脚本工具**：基于 Cyber RT Python API 和 Protobuf 实现数据录制、回放、分析、地图处理等，分布在 `common/`、`record_play/`、`record_analyzer/`、`navigator/` 等子目录

此外，`manual_traffic_light/` 提供 C++ 实现的 Cyber RT 组件，用于在仿真环境中手动控制交通灯信号。

## 核心类

### ManualTrafficLight

C++ Cyber RT 定时器组件，用于在无感知模块时手动发送交通灯检测结果。订阅定位消息获取当前位置，查询高精地图获取前方交通灯，通过键盘输入切换红/绿灯状态，发布 `TrafficLightDetection` 消息。

```cpp
class ManualTrafficLight final : public apollo::cyber::TimerComponent {
 public:
  bool Init();
  bool Proc();
 private:
  bool GetAllTrafficLights(std::vector<SignalInfoConstPtr> *traffic_lights);
  bool GetTrafficLightsWithinDistance(std::vector<SignalInfoConstPtr> *traffic_lights);
  bool CreateTrafficLightDetection(
      const std::vector<SignalInfoConstPtr> &signals,
      TrafficLight::Color color, TrafficLightDetection *detection);
  TrafficLight::Color GetKeyBoardColorInput();
};
```

### RenderableObject

Qt OpenGL 可渲染对象基类，管理 VAO/VBO 和 Shader Program 的生命周期。所有 3D 可视化对象（点云、雷达、网格等）均继承此类。

```cpp
class RenderableObject : protected QOpenGLFunctions {
 public:
  bool Init(std::shared_ptr<QOpenGLShaderProgram>& shaderProgram);
  void Destroy();
  void Render(const QMatrix4x4* mvp = nullptr);
 protected:
  virtual bool FillVertexBuffer(GLfloat* pBuffer) = 0;
  virtual GLenum GetPrimitiveType() const = 0;
};
```

### PointCloud / RadarPoints

继承 `RenderableObject` 的具体渲染类。`PointCloud` 将 protobuf 点云数据转为 GL_POINTS 渲染；`RadarPoints` 将雷达障碍物数据渲染为彩色点，支持自定义颜色。

```cpp
class PointCloud : public RenderableObject {
 public:
  bool FillData(const std::shared_ptr<const apollo::drivers::PointCloud>& pData);
  GLenum GetPrimitiveType() const override { return GL_POINTS; }
};

class RadarPoints : public RenderableObject {
 public:
  bool FillData(const std::shared_ptr<const apollo::drivers::RadarObstacles>& pData);
  void set_color(const QColor& color);
};
```

### AbstractCamera 体系

相机抽象基类，支持透视/正交两种投影模式，管理位置、姿态（yaw/pitch/roll）和投影矩阵。`FreeCamera` 实现自由移动（Walk/Strafe/Lift），`TargetCamera` 实现绕目标点旋转。

```cpp
class AbstractCamera {
 public:
  enum class CameraMode { PerspectiveMode, OrthoMode };
  virtual void UpdateWorld() = 0;
  void SetUpProjection(float fovInDegrees, float nearPlaneWidth,
                       float nearPlaneHeight, float near, float far);
};
```

### SceneViewer / MainWindow

`SceneViewer` 是 Qt OpenGL 控件，管理场景渲染、相机切换（FREE/TARGET）和鼠标交互。`MainWindow` 是主窗口，集成点云/图像/雷达的通道订阅与可视化。二者通过 `CyberChannReader<T>` 模板类订阅 Cyber RT 通道。

### PbMessageManager

Python 消息管理器，维护 topic 到 protobuf 类型的映射表，支持按 topic 或自动猜测解析 protobuf 文件。

```python
class PbMessageManager:
    def get_msg_meta_by_topic(self, topic) -> MessageType
    def get_msg_meta_by_name(self, name) -> MessageType
    def parse_topic_file(self, topic, filename)
    def parse_file(self, filename) -> (MessageType, message)
```

### Logger

Python 日志工厂类，单例模式管理命名 logger，支持文件和标准输出，日志文件按小时轮转。

```python
class Logger:
    @staticmethod
    def config(log_file, use_stdout, log_level)
    @staticmethod
    def get_logger(tag) -> logging.Logger
```

## 核心函数

### proto_utils.py

Protobuf 文件读写工具函数集。

```python
def write_pb_to_text_file(topic_pb, file_path)    # 写 protobuf 到文本文件
def get_pb_from_text_file(filename, pb_value)       # 从文本文件读 protobuf
def get_pb_from_bin_file(filename, pb_value)        # 从二进制文件读 protobuf
def get_pb_from_file(filename, pb_value)            # 自动尝试二进制和文本模式
def flatten(pb_value, selectors)                    # 按选择器路径展平 protobuf 字段
```

### file_utils.py

```python
def list_files(dir_path)                # 递归列出目录下所有文件
def getInputDirDataSize(path)           # 计算目录下所有文件总大小
```

## 配置

### ManualTrafficLight

通过 `manual_traffic_light.conf` 和命令行 flag 控制行为：

| Flag | 默认值 | 说明 |
|------|--------|------|
| `all_lights` | `false` | 是否获取地图中全部交通灯 |
| `traffic_light_distance` | `1000.0` | 获取前方指定距离内的交通灯（米） |

启动配置通过 `manual_traffic_light.dag` 和 `manual_traffic_light.launch` 定义 Cyber RT 组件拓扑。

### Visualizer

Qt 可视化工具依赖 Shader 文件进行渲染，存储在 `visualizer/` 目录下，通过 `RenderableObject::CreateShaderProgram()` 加载。

## 调用关系

```text
Cyber RT Framework
  |
  +-- ManualTrafficLight (TimerComponent)
  |     |-- Reader<LocalizationEstimate>   <-- /apollo/localization/pose
  |     |-- HDMapUtil (GetForwardNearestSignalsOnLane)
  |     +-- Writer<TrafficLightDetection>  --> /apollo/perception/traffic_light
  |
  +-- Visualizer (Qt Application)
        |-- MainWindow
        |     |-- CyberChannReader<PointCloud/Image/RadarObstacles>
        |     +-- SceneViewer
        |           |-- AbstractCamera {FreeCamera, TargetCamera}
        |           +-- RenderableObject {PointCloud, RadarPoints, Grid, Plane}
        +-- Python Tools (common/)
              |-- PbMessageManager -> proto_utils
              |-- Logger
              +-- file_utils

Python 工具链:
  record_play/    -- 录制 RTK 数据到 Cyber RT
  record_analyzer/ -- 分析 planning/control 模块表现
  navigator/       -- 路线录制、平滑、可视化
  replay/          -- 回放录制文件
```

