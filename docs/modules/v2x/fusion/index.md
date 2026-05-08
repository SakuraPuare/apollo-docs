---
title: "V2X 融合"
---

# V2X 融合

> 源码路径: `modules/v2x/fusion/`

## 概述

V2X 融合模块负责将车载感知障碍物与 V2X 通信获取的远端障碍物进行数据融合。基于 KM（Kuhn-Munkres）匈牙利匹配算法计算关联矩阵，通过距离和类型评分完成目标关联，输出融合后的障碍物列表供下游规划模块使用。

## 核心类

### V2XFusionComponent

Cyber RT 组件，接收感知障碍物消息并触发融合流程。

```cpp
class V2XFusionComponent
    : public apollo::cyber::Component<PerceptionObstacles> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<PerceptionObstacles>& perception_obstacles)
      override;

 private:
  // 执行 V2X 消息融合处理
  bool V2XMessageFusionProcess(
      const std::shared_ptr<PerceptionObstacles>& perception_obstacles);
  // 将融合结果序列化为 PerceptionObstacles 消息
  void SerializeMsg(const std::vector<base::Object>& objects,
                    std::shared_ptr<PerceptionObstacles> obstacles);

  Fusion fusion_;
  std::shared_ptr<cyber::Reader<LocalizationEstimate>> localization_reader_;
  std::shared_ptr<cyber::Reader<V2XObstacles>> v2x_obstacles_reader_;
  std::shared_ptr<cyber::Writer<PerceptionObstacles>>
      perception_fusion_obstacles_writer_;
};
```

### Fusion

核心融合算法类，执行多源目标关联与融合。

```cpp
class Fusion {
 public:
  bool Init();
  // 处理多组输入目标列表，执行融合
  bool Proc(const std::vector<std::vector<base::Object>>& input_objectlists,
            double timestamp);
  // 获取融合后的目标列表
  std::vector<base::Object>& get_fused_objects();

  // 将新资源与已有融合结果合并
  bool CombineNewResource(
      const std::vector<base::Object>& new_objects,
      std::vector<base::Object>* fused_objects,
      std::vector<std::vector<base::Object>>* fusion_result);
  // 删除冗余目标
  int DeleteRedundant(std::vector<base::Object>* objects);

 private:
  // 计算关联矩阵（基于距离和类型评分）
  bool ComputeAssociateMatrix(const std::vector<base::Object>& in1_objects,
                              const std::vector<base::Object>& in2_objects,
                              Eigen::MatrixXf* association_mat);
  // 距离评分
  bool CheckDisScore(const base::Object& in1, const base::Object& in2,
                     double* score);
  // 类型评分
  bool CheckTypeScore(const base::Object& in1, const base::Object& in2,
                      double* score);

  KMkernal km_matcher_;  // KM 匈牙利匹配器
  fusion::ScoreParams score_params_;
};
```

### FTConfigManager

单例配置管理器，从 protobuf 文件加载融合参数。

```cpp
PLUGIN_PARAMS(FusionParams, FLAGS_fusion_conf_file, fusion::FusionParams)

class FTConfigManager {
 public:
  DECLARE_SINGLETON(FTConfigManager)
 public:
  FusionParams fusion_params_;
};
```

## 调用关系

```text
V2XFusionComponent (Cyber RT 组件)
├── Init() → 初始化 Fusion、订阅 Localization 和 V2XObstacles
├── Proc() → 接收 PerceptionObstacles 触发融合
│   ├── V2XMessageFusionProcess()
│   │   ├── Fusion::Proc() → 执行多源融合
│   │   │   ├── CombineNewResource() → 合并新目标
│   │   │   │   ├── ComputeAssociateMatrix() → 计算关联矩阵
│   │   │   │   └── KMkernal → 匈牙利匹配
│   │   │   └── DeleteRedundant() → 去重
│   │   └── GetV2xFusionObjects() → 提取融合结果
│   └── SerializeMsg() → 序列化并发布融合障碍物
└── FTConfigManager → 提供融合参数配置
```
