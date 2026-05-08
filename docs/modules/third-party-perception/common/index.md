---
title: "第三方感知公共库"
---

# Third Party Perception Common

> 源码路径：`modules/third_party_perception/common/`

## 概述

第三方感知模块的公共库，提供全局配置参数和几何工具函数，用于将第三方传感器（Mobileye、雷达等）数据转换为 Apollo 感知障碍物格式。

## 核心文件

### third_party_perception_gflags

配置参数定义，包括：

| 参数类别 | 示例参数 | 说明 |
| -------- | -------- | ---- |
| 传感器校准 | `mobileye_pos_adjust`, `radar_pos_adjust` | 位置偏移校准 |
| ID 偏移 | `mobileye_id_offset`, `radar_id_offset` | 避免 ID 冲突 |
| 默认尺寸 | `default_car_length/width`, `default_truck_length` | 创建包围盒 |
| 雷达过滤 | `filter_y_distance`, `movable_speed_threshold` | 过滤无效目标 |

### third_party_perception_util

几何工具函数集：

```cpp
double GetAngleFromQuaternion(const Quaternion quaternion);
void FillPerceptionPolygon(PerceptionObstacle* obstacle, ...);
double GetDefaultObjectLength(const int object_type);
double GetDefaultObjectWidth(const int object_type);
Point3D SLtoXY(const double x, const double y, const double theta);
double Distance(const Point3D& point1, const Point3D& point2);
double Speed(const Point3D& point);
double GetNearestLaneHeading(const PointENU& point_enu);
double GetLateralDistanceToNearestLane(const Point3D& point);
double HeadingDifference(const double theta1, const double theta2);
```

**主要功能**：

- 四元数转角度
- 根据中心点和尺寸填充多边形
- 按物体类型获取默认长宽
- SL 坐标到 XY 坐标转换
- 距离/速度计算
- 最近车道朝向查询

## 调用关系

- **被调用方**：`conversion_mobileye`、`conversion_radar`、`conversion_smartereye` 转换工具
- **依赖**：HDMap（车道朝向查询）
