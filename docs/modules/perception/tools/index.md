---
title: "感知工具"
---

# Tools

> 源码路径: `modules/perception/tools/`

## 概述

`perception/tools` 模块为感知系统提供离线开发与调试工具集，包含四个子模块：

- **common** -- 共享基础工具：KITTI 标签加载、图像读写、检测结果保存、相机可视化
- **exporter** -- Cyber 消息导出工具，将录制数据中的图像和点云连同位姿一并导出到文件
- **offline** -- 离线可视化框架，含坐标变换服务（TransformServer）、完整 2D/3D 可视化器、键盘交互标定
- **offline_camera_detection** -- 离线相机检测主程序，串联 common 工具和检测器进行批量推理与可视化

## 核心类

### MsgExporter

```cpp
// modules/perception/tools/exporter/msg_exporter.h
namespace apollo::perception::msg {
class MsgExporter {
 public:
  MsgExporter(std::shared_ptr<apollo::cyber::Node> node,
              const std::vector<std::string> channels,
              const std::vector<std::string> child_frame_ids);
 private:
  void ImageMessageHandler(const std::shared_ptr<const ImgMsg>& img_msg,
                           const std::string& channel,
                           const std::string& child_frame_id,
                           const std::string& folder);
  void PointCloudMessageHandler(const std::shared_ptr<const PcMsg>& cloud_msg,
                                const std::string& channel,
                                const std::string& child_frame_id,
                                const std::string& folder);
  bool SavePointCloud(const pcl::PointCloud<PCLPointXYZIT>& point_cloud,
                      double timestamp, const std::string& folder);
  bool SaveImage(const unsigned char* color_image,
                 const unsigned char* range_image, std::size_t width,
                 std::size_t height, double timestamp,
                 const std::string& folder);
  bool QuerySensorToWorldPose(double timestamp,
                              const std::string& child_frame_id,
                              Eigen::Matrix4d* pose);
  bool QueryPose(double timestamp, const std::string& frame_id,
                 const std::string& child_frame_id, Eigen::Matrix4d* pose);
  bool SavePose(const Eigen::Matrix4d& pose, double timestamp,
                const std::string& folder);
};
}
```

根据配置文件绑定 Cyber 通道，在回调中将图像保存为 JPG、点云保存为 PCD，同时通过 `tf2_buffer` 查询传感器到世界坐标系的变换并保存 `.pose` 文件。

### TransformServer

```cpp
// modules/perception/tools/offline/transform_server.h
namespace apollo::perception::camera {
class TransformServer {
 public:
  bool Init(const std::vector<std::string> &camera_names,
            const std::string &params_path);
  bool AddTransform(const std::string &child_frame_id,
                    const std::string &frame_id,
                    const Eigen::Affine3d &transform);
  bool QueryTransform(const std::string &child_frame_id,
                      const std::string &frame_id, Eigen::Affine3d *transform);
  bool LoadFromFile(const std::string &tf_input, float frequency = 200.0f);
  bool QueryPos(double timestamp, Eigen::Affine3d *pose);
};
}
```

以图结构管理多传感器坐标系之间的变换关系。`Init` 从 YAML 加载激光雷达高度及各相机外参，通过 `AddTransform` 构建双向边；`QueryTransform` 使用 DFS 搜索任意两个坐标系间的变换链路。`LoadFromFile` 和 `QueryPos` 支持从文件加载时序位姿并按时间戳查询。

### Visualizer（offline）

```cpp
// modules/perception/tools/offline/visualizer.h
namespace apollo::perception::camera {
class Visualizer {
 public:
  bool Init(const std::vector<std::string> &camera_names,
            TransformServer *tf_server);
  bool Init_all_info_single_camera(
      const std::vector<std::string> &camera_names,
      const std::string &visual_camera,
      const EigenMap<std::string, Eigen::Matrix3f> &intrinsic_map,
      const EigenMap<std::string, Eigen::Matrix4d> &extrinsic_map,
      const Eigen::Matrix4d &ex_lidar2imu, double pitch_adj,
      double yaw_adj, double roll_adj, int image_height, int image_width);
  void ShowResult(const cv::Mat &img, const CameraFrame &frame);
  void ShowResult_all_info_single_camera(
      const cv::Mat &img, const CameraFrame &frame,
      const base::MotionBufferPtr motion_buffer,
      const Eigen::Affine3d &world2camera);
  bool key_handler(const std::string &camera_name, int key);
  bool save_manual_calibration_parameter(const std::string &camera_name,
                                         double pitch_adj_degree,
                                         double yaw_adj_degree,
                                         double roll_adj_degree);
};
}
```

离线全流程可视化器，支持：

- 2D 检测框与 3D 包围盒在图像上的投影绘制
- 鸟瞰图（BEV）上绘制障碍物、车道线、虚拟自车道、轨迹
- 单应矩阵（Homography）实现图像-地面坐标互转
- 键盘交互手动标定（Pitch/Yaw/Roll 微调并保存外参到 YAML）

## 核心函数

### common 子模块

| 函数 | 说明 |
|------|------|
| `LoadKittiLabel(frame, kitti_path, dist_type)` | 读取 KITTI 格式标签文件，解析物体类型、3D 尺寸、中心位置等信息填入 `frame->detected_objects`，支持 `H-from-h` 和 `H-on-h` 两种距离计算方式 |
| `FillImage(frame, file_name)` | 从文件读取图像并填入 `frame->data_provider` |
| `RecoveryImage(frame, cv_img)` | 从 `frame->data_provider` 恢复 OpenCV Mat |
| `GetFileListFromFile(file_name, file_list)` | 从文本文件逐行读取文件名列表 |
| `SaveCameraDetectionResult(frame, file_name)` | 将检测结果保存为扩展 KITTI 格式文本文件 |
| `CameraVisualization(frame, file_name)` | 将检测结果（2D 框、遮挡信息）绘制到图像上并保存为 JPG |

### exporter 子模块

| 函数 | 说明 |
|------|------|
| `config_parser(config_file, channels, child_frame_ids)` | 解析配置文件，每行格式为 `channel child_frame_id enable_flag` |
| `MsgExporter::ImageMessageHandler` | 接收图像消息，查询位姿后保存图像和 `.pose` 文件 |
| `MsgExporter::PointCloudMessageHandler` | 接收点云消息，过滤 NaN 和离群值后保存为 PCD |
| `MsgExporter::QuerySensorToWorldPose` | 组合 sensor-to-novatel 外参与 novatel-to-world 位姿，得到传感器到世界坐标系的完整变换 |

### offline 子模块

| 函数 | 说明 |
|------|------|
| `TransformServer::Init` | 从 YAML 配置加载激光雷达高度、激光雷达-导航设备外参、各相机外参，构建坐标变换图 |
| `TransformServer::QueryTransform` | DFS 搜索坐标变换图，计算任意两坐标系间的复合变换 |
| `Visualizer::Init_all_info_single_camera` | 初始化全部相机参数（内参、外参、IMU 变换），计算 Homography 矩阵和视野（FOV）参数 |
| `Visualizer::adjust_angles` | 根据 Pitch/Yaw/Roll 调整量重新计算相机到车体的 Homography |
| `Visualizer::Draw2Dand3D_all_info_single_camera` | 在图像上绘制 3D 包围盒、2D 框、车道线、虚拟自车道、轨迹，在 BEV 图上绘制对应元素 |
| `Visualizer::key_handler` | 处理键盘输入，切换可视化选项或在标定模式下微调外参角度 |
| `Visualizer::save_manual_calibration_parameter` | 将手动标定后的旋转角转为四元数，写回相机外参 YAML 文件 |
| `Visualizer::image2ground / ground2image` | 通过 Homography 矩阵实现图像像素坐标与地面坐标互转 |

## 配置

### MsgExporter 配置文件

`exporter` 使用纯文本配置文件，每行格式：

```text
<channel> <child_frame_id> <enable_flag>
```

示例运行：

```bash
./msg_exporter export_msg.config
cyber_recorder play -f record_name.record
```

### TransformServer 配置

从指定目录加载 YAML 文件：

- `velodyne128_height.yaml` -- 激光雷达安装高度
- `velodyne128_novatel_extrinsics.yaml` -- 激光雷达到导航设备的外参
- `<camera_name>_extrinsics.yaml` -- 各相机外参（旋转四元数 + 平移向量）

### offline_camera_detection 命令行参数

```cpp
DEFINE_int32(height, 1080, "image height");
DEFINE_int32(width, 1920, "image width");
DEFINE_int32(gpu_id, 0, "gpu id");
DEFINE_string(dest_dir, "./data/output", "output dir");
DEFINE_string(dist_type, "", "dist pred type: H-on-h, H-from-h");
DEFINE_string(kitti_dir, "", "pre-detected obstacles (skip Detect)");
DEFINE_string(root_dir, "...", "image root dir");
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(config_path, "perception/camera_detection_multi_stage/data", "config path");
DEFINE_string(config_file, "yolox3d.pb.txt", "config file");
DEFINE_string(camera_name, "front_6mm", "camera name");
DEFINE_string(detector_name, "Yolox3DObstacleDetector", "detector name");
```

## 调用关系

```text
offline_camera_detection (main)
  |
  +-- InitDataProvider()
  +-- BaseObstacleDetector::Init / Detect
  +-- common::LoadKittiLabel()      -- 或跳过检测直接加载标签
  +-- common::FillImage()           -- 读取图像
  +-- common::SaveCameraDetectionResult()  -- 保存结果
  +-- common::CameraVisualization()        -- 可视化输出

msg_exporter_main (main)
  |
  +-- config_parser()
  +-- MsgExporter(channel, child_frame_ids)
       +-- CreateReader<Image> -> ImageMessageHandler
       |    +-- QuerySensorToWorldPose -> QueryPose (tf2_buffer)
       |    +-- SaveImage / SavePose
       +-- CreateReader<PointCloud> -> PointCloudMessageHandler
            +-- QuerySensorToWorldPose
            +-- SavePointCloud / SavePose

offline::Visualizer
  +-- TransformServer::Init (加载 YAML 外参)
  +-- Init_all_info_single_camera (计算 Homography)
  +-- ShowResult_all_info_single_camera
       +-- Draw2Dand3D_all_info_single_camera (2D/3D 绘制)
       +-- DrawTrajectories (轨迹绘制)
  +-- key_handler -> adjust_angles -> save_manual_calibration_parameter
       +-- euler_to_quaternion -> save_extrinsic_in_yaml
```
