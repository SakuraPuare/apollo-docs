---
title: 贡献指南
---

# 贡献指南

欢迎参与 Apollo 开源自动驾驶平台的建设！无论你是经验丰富的开发者还是刚接触自动驾驶领域的新人，我们都非常欢迎你的贡献。本文档将帮助你了解如何参与 Apollo 项目。

## 项目愿景与目标

Apollo 是百度推出的高性能、灵活架构的开源自动驾驶平台，旨在加速自动驾驶车辆的开发、测试和部署。项目的核心理念是：

- 构建一个充满活力的自动驾驶生态系统，提供全面、安全、可靠的解决方案
- 让每个生态成员专注于自身擅长的领域，避免重复造轮子，从而大幅提升创新速度
- 通过开源代码和开放能力，让任何人都可以使用、修改和再分发 Apollo 的组件
- 形成"软件部署 → 数据采集 → 系统迭代"的良性循环，加速自动驾驶技术的成熟

Apollo 欢迎所有形式的技术和非技术贡献。高质量的数据是驱动创新的燃料，也是最有价值的贡献之一。

## 治理模型

Apollo 采用开放治理模型，社区角色分为以下几个层级：

### 维护者（Maintainers）

- 由百度 Apollo 核心团队成员担任
- 负责项目的整体架构决策、版本发布和长期路线图
- 拥有代码仓库的合并权限
- 负责阐明行为准则标准，并对违规行为采取纠正措施
- 为保障架构完整性、系统可靠性和快速演进，百度在必要时会行使领导权推动重要决策

### 提交者（Committers）

- 在特定模块有持续高质量贡献的开发者
- 拥有特定模块的代码审查权限
- 协助维护者进行代码审查和技术讨论
- 由维护者根据贡献记录提名

> （此角色基于开源社区通行实践描述，Apollo 仓库中暂未有正式文档定义）

### 贡献者（Contributors）

- 任何向项目提交过被合并的 Pull Request 的人
- 包括代码、文档、测试、Bug 报告等各类贡献
- 所有贡献者需同意 [Apollo 个人贡献者许可协议（CLA）](https://gist.githubusercontent.com/startcode/f5ccf8887bfc7727a0ae05bf0d601e30/raw/029a11300e987e34a29a9d247ac30caa7f6741a7/Apollo_Individual_Contributor_License_Agreement)

## 贡献方式

### 代码贡献

这是最直接的贡献方式。你可以：

- 修复已知 Bug
- 实现新功能
- 优化现有模块的性能
- 查找标记为 ["help wanted"](https://github.com/ApolloAuto/apollo/issues?utf8=%E2%9C%93&q=label%3A%22Type%3A+Help+wanted%22+) 的 Issue，这些是适合新贡献者入手的任务

### 文档贡献

- 改进现有文档的准确性和可读性
- 补充缺失的 API 文档和使用教程
- 翻译文档（中英文互译）
- Apollo 使用 Doxygen 生成 API 文档，可通过 `bash apollo.sh doc generate` 生成

### 问题报告

- 在 [GitHub Issues](https://github.com/ApolloAuto/apollo/issues) 提交 Bug 报告或功能请求
- Bug 报告请包含：问题描述、复现步骤、期望行为、系统环境信息和截图
- 功能请求请描述：要解决的问题、期望的方案、考虑过的替代方案

### 社区支持

- 在 Issue 中回答其他用户的问题
- 参与技术讨论和方案评审
- 分享使用经验和最佳实践

## 开发环境搭建

### 系统要求

- 8 核以上处理器，16GB 以上内存
- 推荐 NVIDIA Turing 架构 GPU 或 AMD GFX9/RDNA/CDNA GPU
- 支持 Ubuntu 18.04、20.04、22.04
- NVIDIA 驱动版本 520.61.05 及以上
- Docker-CE 19.03 及以上
- NVIDIA Container Toolkit

### 环境搭建步骤

1. Fork 并克隆仓库

```bash
git clone https://github.com/<你的用户名>/apollo.git
cd apollo
```

2. 启动开发容器

```bash
bash docker/scripts/dev_start.sh
```

3. 进入容器

```bash
bash docker/scripts/dev_into.sh
```

4. 构建项目

```bash
bash apollo.sh build
```

### 技术栈概览

| 技术 | 说明 |
|------|------|
| C++ | 主要开发语言 |
| Python | 工具和脚本 |
| Bazel | 构建系统 |
| CyberRT | 自研实时通信中间件 |
| CUDA 11.8 | GPU 加速 |
| PyTorch / TensorFlow 2 | 深度学习框架 |

## 贡献工作流

### 第一步：签署 CLA

在提交任何贡献之前，请先签署 [Apollo 个人贡献者许可协议](https://gist.githubusercontent.com/startcode/f5ccf8887bfc7727a0ae05bf0d601e30/raw/029a11300e987e34a29a9d247ac30caa7f6741a7/Apollo_Individual_Contributor_License_Agreement)。

### 第二步：选择任务

- 浏览 [Issues 列表](https://github.com/ApolloAuto/apollo/issues)，特别关注 "help wanted" 标签
- 如果你正在处理某个 Issue，请留言告知其他人，避免重复工作
- 如果你有新的想法，先创建 Issue 讨论，获得社区反馈后再开始编码

### 第三步：创建分支并开发

```bash
# 从最新的 master 分支创建你的工作分支
git checkout master
git pull upstream master
git checkout -b feature/your-feature-name
```

### 第四步：编写代码

请遵循以下规范：

**许可证头部**：每个新文件顶部必须包含 Apache 2.0 许可证声明。参考示例：
- C++ 文件：`modules/common/util/util.h`
- Python 文件：`modules/tools/vehicle_calibration/process.py`
- Bash 文件：`scripts/apollo_base.sh`

**编码风格**：
- C/C++：遵循 [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- Python：遵循 [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)，可使用 `yapf -i --style='{based_on_style: google}' foo.py` 格式化
- BUILD 文件：使用 `bash apollo.sh format path/to/BUILD/files` 格式化
- 更多实践请参考 Apollo Best Coding Practice 文档

**单元测试**：
- 所有代码贡献必须包含对应的单元测试
- 测试文件命名以 `_test.cc` 结尾
- BUILD 文件中的测试目标以 `test` 结尾
- 运行全部测试：`bash apollo.sh test`

### 第五步：编写提交信息

提交信息的第一行应为一句话的变更摘要，之后可以添加段落详细说明变更内容。如果修复了某个 Issue，请在提交信息中引用 Issue 编号。示例：

```
Control: Replaced algorithm A with algorithm B in modules/control.

Algorithm B is faster than A because it uses binary search. The runtime is
reduced from O(N) to O(log(N)).

Fixes #1234
```

### 第六步：提交前检查

在创建 Pull Request 之前，请确保你的变更通过所有检查：

```bash
# 一键运行构建、测试和代码风格检查
bash apollo.sh check
```

这等同于依次执行：

```bash
bash apollo.sh build   # 构建
bash apollo.sh test    # 测试
bash apollo.sh lint    # 代码风格检查
```

### 第七步：创建 Pull Request

1. 将你的分支推送到你 Fork 的仓库
2. 在 GitHub 上创建 Pull Request，目标分支为 `master`
3. 填写清晰的 PR 描述，说明变更内容和原因
4. 关联相关的 Issue
5. 等待 CI 检查通过和代码审查

## 代码审查流程

所有提交到 Apollo 的代码都需要经过代码审查：

1. 提交 PR 后，CI 系统会自动运行构建和测试
2. 至少一名维护者或提交者会审查你的代码
3. 审查者可能会提出修改建议，请积极回应并更新代码
4. 所有审查意见解决后，维护者会合并你的 PR

审查关注点包括：

- 代码是否符合编码规范
- 是否包含充分的单元测试
- 是否有清晰的文档和注释
- 架构设计是否合理
- 是否存在性能或安全隐患

::: tip 提示
如果你的代码逻辑不够直观，建议以清晰高效的方式实现，并提供充分的注释和文档。代码注释请遵循 Doxygen 格式规范。
:::

## 行为准则

Apollo 社区致力于营造一个开放、友好的环境。我们承诺：无论参与者的年龄、体型、身体状况、种族、性别认同、经验水平、国籍、外貌、宗教或性取向如何，都能在项目和社区中享有免于骚扰的自由。

### 我们鼓励的行为

- 使用友好和包容性的语言
- 尊重不同的观点和经历
- 得体地接受建设性批评
- 关注对社区有益的事情
- 友善地对待社区其他成员

### 不可接受的行为

- 使用与性有关的言语或图像，以及不受欢迎的性骚扰
- 捣乱、煽动、造谣，或含有侮辱/贬损的评论
- 公开或私下的骚扰
- 未经许可发布他人的个人信息
- 个人或政治攻击
- 其他可以被合理地认为不适合专业环境的行为

### 冲突解决

1. 直接与冲突当事人私下沟通，最好是实时沟通
2. 如果不行，请求第三方出面调解
3. 如果仍无法解决，向项目团队举报：apollo-beijing@baidu.com

完整的行为准则请参阅项目根目录下的 `CODE_OF_CONDUCT_cn.md`。

## 社区渠道

- [GitHub Issues](https://github.com/ApolloAuto/apollo/issues) — 提交问题和 Bug 报告
- [GitHub Pull Requests](https://github.com/ApolloAuto/apollo/pulls) — 代码贡献和审查
- [Twitter @apolloplatform](https://twitter.com/apolloplatform) — 项目动态
- [YouTube](https://www.youtube.com/channel/UC8wR_NX_NShUTSSqIaEUY9Q) — 技术视频和教程
- [Medium Blog](https://www.medium.com/apollo-auto) — 技术博客
- [Newsletter](http://eepurl.com/c-mLSz) — 订阅项目通讯
- [Apollo 官网](http://apollo.auto) — 商务合作和更多信息
- 商务合作邮箱：apollopartner@baidu.com

## 许可证

Apollo 基于 [Apache 2.0 许可证](https://github.com/ApolloAuto/apollo/blob/master/LICENSE) 开源。所有贡献的代码将以相同许可证发布。
