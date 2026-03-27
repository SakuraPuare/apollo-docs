---
layout: home
hero:
  name: Apollo
  text: 自动驾驶开放平台
  tagline: 基于 Apollo 11.0 源码的深度技术解析 — 架构、模块、数据流、部署全覆盖
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
      text: 文档源码
      link: https://github.com/SakuraPuare/apollo-docs

features:
  - icon: 🧠
    title: Cyber RT 框架
    details: 高性能实时中间件 — Node、Component、Transport、Scheduler 等核心机制深度解析
    link: /cyber/
    linkText: 查看框架文档

  - icon: 👁️
    title: 感知 Perception
    details: 多传感器融合、目标检测、语义分割、红绿灯识别等核心算法与模型架构
    link: /modules/perception/
    linkText: 查看感知模块

  - icon: 🔮
    title: 预测 Prediction
    details: 障碍物轨迹预测、意图识别，为规划模块提供未来态势感知
    link: /modules/prediction/
    linkText: 查看预测模块

  - icon: 🗺️
    title: 规划 Planning
    details: 参考线平滑、决策逻辑、EM Planner / Lattice Planner 等路径与速度规划
    link: /modules/planning/
    linkText: 查看规划模块

  - icon: 🎮
    title: 控制 Control
    details: PID / LQR / MPC 横纵向控制器，将规划轨迹转化为方向盘与油门指令
    link: /modules/control/
    linkText: 查看控制模块

  - icon: 📍
    title: 定位与地图
    details: 多传感器融合定位、高精地图加载与查询、坐标变换体系
    link: /modules/localization/
    linkText: 查看定位模块

  - icon: 🖥️
    title: Dreamview 可视化
    details: Web 端实时可视化平台，支持场景回放、模块调试、仿真联调
    link: /modules/dreamview/
    linkText: 查看 Dreamview

  - icon: 🔧
    title: 构建与工具链
    details: Bazel 构建系统、自定义规则、Docker 开发环境、脚本工具链
    link: /guide/build-system
    linkText: 查看构建文档

  - icon: 🚗
    title: 硬件与通信
    details: CAN 总线、驱动适配、V2X 车路协同、Bridge 桥接等硬件接口层
    link: /modules/drivers/
    linkText: 查看驱动模块

  - icon: 🛡️
    title: 安全与监控
    details: Guardian 安全守护、Monitor 系统监控、功能安全与降级策略
    link: /modules/guardian/
    linkText: 查看安全模块

  - icon: 🚀
    title: 部署与运行
    details: 启动流程、车辆适配、仿真回放、数据采集全链路部署指南
    link: /guide/startup-flow
    linkText: 查看部署文档

  - icon: 📊
    title: 数据与配置
    details: Proto 消息定义、跨模块数据流、配置体系、标定与数据管理
    link: /guide/data-flow
    linkText: 查看数据流文档
---
