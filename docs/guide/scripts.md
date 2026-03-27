---
title: 脚本与工具链
---

# 脚本与工具链

## 概述

Apollo 的构建、测试与开发操作均通过 `apollo.sh` 统一入口脚本进行调度。该脚本位于项目根目录，内部通过 `scripts/apollo.bashrc` 加载环境，再将具体任务分发给 `scripts/` 目录下的各专项脚本。

```
apollo.sh
└── scripts/
    ├── apollo.bashrc        # Shell 环境初始化
    ├── common.bashrc        # 通用 Bash 工具函数
    ├── apollo_base.sh       # 核心公共函数库
    ├── apollo_build.sh      # 构建
    ├── apollo_test.sh       # 单元测试
    ├── apollo_coverage.sh   # 覆盖率测试
    ├── apollo_lint.sh       # 代码风格检查
    ├── apollo_format.sh     # 代码格式化
    ├── apollo_config.sh     # 构建环境配置
    ├── apollo_clean.sh      # 清理构建产物
    ├── apollo_release.sh    # 二进制发布包构建
    ├── apollo_ci.sh         # CI 流水线
    ├── apollo_docs.sh       # Doxygen 文档生成
    └── apollo_buildify.sh   # Bazel BUILD 文件格式化
```

> **注意**：Docker 构建依赖安装脚本位于 `docker/build/installers/`，而非 `scripts/` 目录下。

## 使用方法

所有命令均通过根目录的 `apollo.sh` 调用：

```bash
bash apollo.sh <command> [module]
```

### 配置

| 命令 | 说明 |
|------|------|
| `config` | 交互式或非交互式配置 Bazel 构建环境 |

### 构建

| 命令 | 说明 |
|------|------|
| `build [module]` | 构建 cyber 或指定模块（不指定则构建全部） |
| `build_dbg [module]` | Debug 模式构建 |
| `build_opt [module]` | 优化模式构建 |
| `build_cpu [module]` | 仅 CPU 构建 |
| `build_gpu [module]` | GPU 构建 |
| `build_nvidia [module]` | 针对 NVIDIA GPU 的构建 |
| `build_amd [module]` | 针对 AMD GPU 的构建 |
| `build_opt_gpu [module]` | 优化模式 GPU 构建 |
| `build_opt_gpu_pnc [module]` | 优化模式 GPU PNC 构建 |
| `build_opt_nvidia [module]` | 优化模式 NVIDIA GPU 构建 |
| `build_opt_amd [module]` | 优化模式 AMD GPU 构建 |
| `build_pnc [module]` | PNC 模块构建 |
| `build_pkg [module]` | 包管理模式构建 |
| `build_pkg_dbg [module]` | 包管理模式 Debug 构建 |
| `build_pkg_opt [module]` | 包管理模式优化构建 |
| `build_pkg_opt_gpu [module]` | 包管理模式优化 GPU 构建 |
| `build_fe` | 构建 Dreamview 前端 |
| `build_teleop` | 启用 teleop 支持的构建 |
| `build_prof` | 启用性能分析支持的构建 |

### 测试与检查

| 命令 | 说明 |
|------|------|
| `test [module]` | 运行单元测试 |
| `coverage [module]` | 运行覆盖率测试并生成报告 |
| `lint` | C++ 代码风格检查（cpplint） |
| `check` | 依次执行构建、测试、lint |

### 格式化

| 命令 | 说明 |
|------|------|
| `format` | 格式化 C++、Python、Bazel、Shell 文件 |
| `buildify` | 修复 Bazel BUILD 文件格式 |

### 其他

| 命令 | 说明 |
|------|------|
| `install_dv_plugins` | 安装 Dreamview 插件 |
| `doc` | 生成 Doxygen 文档 |
| `release` | 构建二进制发布包 |
| `clean` | 清理 Bazel 输出目录与日志 |
| `usage` | 显示帮助信息 |

### 调度逻辑

`apollo.sh` 内部通过 `case` 语句将命令分发给对应脚本：

```
config              → apollo_config.sh
build / build_*     → apollo_build.sh（携带对应配置参数）
build_pkg*          → apollo_build_pkg.sh（包管理模式构建）
test                → apollo_test.sh
coverage            → apollo_coverage.sh
lint                → apollo_lint.sh
format              → apollo_format.sh
buildify            → apollo_buildify.sh
doc                 → apollo_docs.sh
release             → apollo_release.sh
clean               → apollo_clean.sh
check               → 依次调用 build、test、lint
cibuild/citest/cilint → apollo_ci.sh
```

## 配置选项

### 环境变量

| 变量 | 说明 |
|------|------|
| `APOLLO_ROOT_DIR` | Apollo 项目根目录 |
| `APOLLO_CACHE_DIR` | 缓存目录 |
| `APOLLO_IN_DOCKER` | 是否在 Docker 环境中运行（布尔值） |
| `APOLLO_VERSION` | 版本字符串，格式为 `branch-timestamp-sha1` |
| `USE_GPU_HOST` | 宿主机 GPU 支持开关 |
| `USE_GPU_TARGET` | 构建目标 GPU 支持开关 |
| `GPU_PLATFORM` | GPU 平台类型（`NVIDIA` / `AMD`） |
| `USE_ESD_CAN` | ESD CAN 库支持开关 |
| `STAGE` | 构建阶段（`dev` / `prod`） |

### apollo_config.sh 配置项

运行 `bash apollo.sh config` 后可配置以下选项，结果保存至 `.apollo.bazelrc`。支持交互模式（`-i`）和非交互模式（`-n`），底层通过 `tools/bootstrap.py` 执行配置：

- GPU 支持开关
- GPU 平台选择（NVIDIA / AMD）
- ESD CAN 支持开关

### apollo_base.sh 核心函数

`apollo_base.sh` 是所有构建脚本共用的函数库，提供以下能力：

| 函数 | 说明 |
|------|------|
| `site_restore()` | 将工作区恢复至干净状态 |
| `env_prepare()` | 准备构建环境，安装依赖 |
| `setup_gpu_support()` | 检测并配置 GPU（NVIDIA/AMD）（注：实际定义在 `apollo.bashrc` 中） |
| `run_module()` | 通过 `cyber_launch` 启动模块 |
| `parse_cmdline_arguments()` | 解析构建命令行参数 |
| `run_bazel()` | 以优化的并发参数执行 Bazel 构建/测试/覆盖率 |
| `record_bag_env_log()` | 录制数据包时记录 Git 状态与环境信息 |

## 常见问题

**Q：构建时提示找不到 GPU，如何处理？**

先运行 `bash apollo.sh config` 重新配置 GPU 支持，确认 `GPU_PLATFORM` 与实际硬件匹配。若宿主机无 GPU，使用 `build_cpu` 命令进行纯 CPU 构建。

**Q：如何只构建某个模块而非全部？**

在命令后附加模块名即可，例如：

```bash
bash apollo.sh build planning
```

不指定模块名时默认构建所有模块。

**Q：`check` 命令与分别执行 `build`、`test`、`lint` 有何区别？**

`check` 会按顺序依次执行三者，任意一步失败则终止后续步骤，适合提交前的完整验证。

**Q：`format` 支持哪些语言？**

`apollo_format.sh` 支持以下格式化工具：

- C++：`clang-format`
- Python：`yapf`（通过 `scripts/yapf.sh`）
- Bazel：`buildifier`
- Shell：`shfmt`
- Markdown

**Q：如何清理构建缓存？**

```bash
bash apollo.sh clean
```

该命令会删除 Bazel 输出目录并清理日志文件。若需彻底重置，可结合 `config` 重新生成 `.apollo.bazelrc`。

**Q：在非 Docker 环境下能否运行构建脚本？**

Apollo 的构建环境依赖特定的系统依赖，官方推荐在 Docker 容器内运行。`APOLLO_IN_DOCKER` 变量为 `false` 时，部分脚本会给出警告，但不强制阻止执行。
