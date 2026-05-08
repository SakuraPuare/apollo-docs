---
title: "可视化工具 (Visualizer)"
---

# 可视化工具 (Visualizer)

> 源码路径: `modules/tools/visualizer/`

## 概述

Visualizer 是 Apollo 提供的基于 Qt/OpenGL 的可视化调试工具。它通过 CyberRT 订阅各模块发布的通道数据（如相机图像、点云等），并以图形化方式实时展示。主要特性：

- 基于 OpenGL 3.3+ 的 3D 渲染引擎
- 自由相机（FreeCamera）支持平移、旋转浏览场景
- 动态发现 CyberRT 拓扑变化，自动添加新通道
- 支持多视口视频图像显示（FixedAspectRatioWidget）
- 网格地面辅助参考（Grid）

## 核心类/函数

### main（入口）

检测 OpenGL Shading Language 版本（需 >= 3.3），初始化 CyberRT，创建主窗口并监听拓扑变化：

```cpp
int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  // 检查 GLSL 版本
  apollo::cyber::Init(argv[0]);
  MainWindow w;
  auto topologyCallback =
      [&w](const apollo::cyber::proto::ChangeMsg& change_msg) {
        w.TopologyChanged(change_msg);
      };
  channelManager->AddChangeListener(topologyCallback);
  w.show();
  return a.exec();
}
```

### AbstractCamera

3D 相机基类，管理投影矩阵和模型视图矩阵，支持透视/正交两种模式：

```cpp
class AbstractCamera {
 public:
  enum class CameraMode { PerspectiveMode, OrthoMode };
  virtual void UpdateWorld(void) = 0;
  void SetUpProjection(float fovInDegrees, float nearPlaneWidth,
                       float nearPlaneHeight, float near, float far);
  void UpdateProjection(void);  // 根据 CameraMode 构建投影矩阵
 protected:
  QVector3D position_;   // 相机位置
  QVector3D attitude_;   // yaw, pitch, roll（度）
  QMatrix4x4 projection_mat_;
  QMatrix4x4 model_view_mat_;
};
```

### FreeCamera

继承 `AbstractCamera`，实现自由漫游相机，支持沿视线方向行走、侧移和升降：

```cpp
class FreeCamera : public AbstractCamera {
 public:
  virtual void UpdateWorld(void);
  void Walk(const float dt) { translation_ += (look_ * dt); }
  void Starfe(const float dt) { translation_ += (right_ * dt); }
  void Lift(const float dt) { translation_ += (up_ * dt); }
 private:
  QVector3D translation_;
};
```

### Grid

继承 `RenderableObject`，使用 `GL_LINES` 绘制地面网格，可配置颜色和单元格数量：

```cpp
class Grid : public RenderableObject {
 public:
  explicit Grid(int cellCountBySide = 1);
  GLenum GetPrimitiveType(void) const { return GL_LINES; }
  void SetCellCount(int cellCount);
 protected:
  virtual bool FillVertexBuffer(GLfloat* pBuffer);
 private:
  QColor grid_color_;
};
```

### CyberChannReader\<T\>

模板类，封装 CyberRT 通道订阅逻辑，提供回调安装和通道打开的统一接口：

```cpp
template <typename T>
class CyberChannReader {
 public:
  bool InstallCallbackAndOpen(CyberChannelCallback<T> channelCallback,
                              const std::string& channelName,
                              const std::string& nodeName);
  void CloseChannel(void);
 private:
  bool CreateChannel(const std::string& channelName,
                     const std::string& nodeName);
  std::shared_ptr<apollo::cyber::Reader<T>> channel_reader_;
  std::shared_ptr<apollo::cyber::Node> channel_node_;
};
```

### FixedAspectRatioWidget

继承 `QWidget`，用于以固定宽高比显示视频图像，支持双击聚焦和右键菜单：

```cpp
class FixedAspectRatioWidget : public QWidget {
  Q_OBJECT
 public:
  void SetupDynamicTexture(const std::shared_ptr<Texture>& textureObj);
  void StartOrStopUpdate(bool b);
 signals:
  void focusOnThis(FixedAspectRatioWidget*);
};
```

## 调用关系

```text
main()
  ├── QApplication 初始化 + OpenGL 版本检查
  ├── apollo::cyber::Init()
  ├── MainWindow
  │     ├── TopologyChanged() ← ChannelManager 拓扑回调
  │     ├── AddNewWriter()    ← 发现新通道写入者
  │     └── FixedAspectRatioWidget (视频视口)
  │           └── VideoImgViewer → Texture
  ├── FreeCamera → AbstractCamera
  │     └── UpdateWorld() / Walk() / Starfe() / Lift()
  ├── Grid → RenderableObject
  │     └── FillVertexBuffer() + GL_LINES 渲染
  └── CyberChannReader<T>
        └── CreateChannel() → cyber::CreateNode() + CreateReader()
```
