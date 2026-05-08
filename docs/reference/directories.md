---
title: "目录结构参考"
---
# 目录结构参考

# third_party

第三方依赖库，通过符号链接指向 Apollo 包管理系统中的预编译包。

对应源码目录：`~/apollo-edu/third_party/`

## 主要依赖

| 包名 | 用途 |
|------|------|
| `eigen3` | 线性代数库 |
| `protobuf` | 序列化框架 |
| `grpc` | RPC 通信 |
| `gtest` | 单元测试框架 |
| `pcl` | 点云处理 |
| `yaml_cpp` | YAML 配置解析 |
| `nlohmann_json` | JSON 处理 |
| `civetweb` | 轻量 HTTP 服务 |

# data

运行时数据目录，包含标定数据、地图数据、日志与本地 KV 存储。

对应源码目录：`~/apollo-edu/data/`

## 目录结构

| 子目录 | 说明 |
|--------|------|
| `calibration_data/` | 传感器标定参数 |
| `map_data/` | 高精地图数据 |
| `log/` | 运行日志 |
| `kv_db.sqlite` | 本地键值存储 |

# dev

开发与构建工具链，包含 Bazel 构建配置、安装脚本与构建辅助工具。

对应源码目录：`~/apollo-edu/dev/`

## 目录结构

| 子目录 | 说明 |
|--------|------|
| `bazel/` | Bazel 构建规则与配置 |
| `buildtool/` | 构建辅助工具 |
| `install/` | 安装与部署脚本 |
