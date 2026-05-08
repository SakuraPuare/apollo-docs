---
title: Apollo Docs
layout: home
hero:
  name: Apollo
  text: 自动驾驶开放平台
  tagline: 基于 Apollo 11.0 源码的深度技术解析 — 架构、模块、数据流全覆盖
  image:
    src: /logo.png
    alt: Apollo Logo
  actions:
    - theme: brand
      text: 快速开始
      link: /guide/getting-started
    - theme: alt
      text: 架构概览
      link: /guide/architecture
    - theme: alt
      text: GitHub
      link: https://github.com/ShuYingJiYu/apollo-docs

features:
  - icon: 🧠
    title: CyberRT 框架
    details: 高性能实时中间件 — Node、Component、Transport、Scheduler 等核心机制源码解析
    link: /cyber/
    linkText: 查看框架文档

  - icon: 🗺️
    title: Planning 规划
    details: 场景决策、路径规划、速度规划全链路 — Planner / Scenario / Task 三层架构逐函数解析
    link: /modules/planning/
    linkText: 查看规划模块

  - icon: 🎮
    title: Control 控制
    details: PID / LQR / MPC 横纵向控制器，将规划轨迹转化为方向盘与油门指令
    link: /modules/control/
    linkText: 查看控制模块

  - icon: 👁️
    title: Perception 感知
    details: 多传感器融合、LiDAR 检测、Camera 检测、红绿灯识别等核心算法
    link: /modules/perception/
    linkText: 查看感知模块

  - icon: 🔮
    title: Prediction 预测
    details: 障碍物轨迹预测、意图识别，为规划模块提供未来态势感知
    link: /modules/prediction/
    linkText: 查看预测模块

  - icon: 📍
    title: Localization 定位
    details: MSF 多传感器融合定位、NDT 点云匹配、RTK 组合导航
    link: /modules/localization/
    linkText: 查看定位模块
---
