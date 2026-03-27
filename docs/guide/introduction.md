# 简介

Apollo 是百度推出的开源自动驾驶平台，提供了完整的自动驾驶软件栈，包括感知、预测、规划、控制等核心模块，以及 Cyber 实时中间件框架。

## 版本

本文档基于 Apollo 9.0 源码编写。

## 项目结构

```
Apollo/
├── cyber/          # Cyber 中间件框架
├── modules/        # 应用模块
│   ├── perception/     # 感知
│   ├── prediction/     # 预测
│   ├── planning/       # 规划
│   ├── control/        # 控制
│   ├── localization/   # 定位
│   ├── routing/        # 路由
│   ├── dreamview/      # 可视化
│   └── ...
├── third_party/    # 第三方依赖
├── tools/          # 构建工具
└── scripts/        # 脚本
```

## 技术栈

- 语言：C++（主要）、Python
- 构建系统：Bazel
- 通信：Cyber RT（自研中间件）
- GPU：CUDA 11.8
- 深度学习：PyTorch / TensorFlow 2
