---
title: "集成测试"
---

# Integration Tests

> 源码路径：`modules/third_party_perception/integration_tests/`

## 概述

第三方感知模块的集成测试框架，提供测试基类用于验证传感器数据转换的端到端正确性。

## 核心文件

### third_party_perception_test_base

测试基类，为第三方感知模块的集成测试提供公共初始化和数据加载能力。

## 调用关系

- **测试对象**：`third_party_perception/tools/` 中的转换工具
