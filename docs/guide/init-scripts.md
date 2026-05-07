# Init Scripts 开发环境脚本解析

本文档解析 `~/apollo-edu/` 下的开发脚本：`init.sh` 环境初始化（16 个函数）、`setup.sh` 快速配置、`kill_all.sh` 进程管理。

## 1. 脚本总览

| 脚本 | 行数 | 说明 |
|------|------|------|
| `init.sh` | 466 | 完整环境初始化（16 个函数） |
| `setup.sh` | 16 | 快速配置入口 |
| `kill_all.sh` | 5 | 进程清理 |

## 2. init.sh 函数清单

### 2.1 工具函数

| 函数 | 说明 |
|------|------|
| `check_in_code()` | 检测是否在 VS Code/Cursor 终端（拒绝编译） |
| `get_cpus()` | 返回编译 CPU 核心数（硬编码 24） |
| `mkill()` | 按进程名强制 kill -9 |
| `collect_changed_module_paths()` | 收集 git 修改文件的 Bazel 模块路径 |
| `collect_changed_top_modules()` | 收集修改文件的顶层模块名 |

### 2.2 编译函数

| 函数 | 说明 |
|------|------|
| `b()` | 增量编译：自动检测修改模块，`buildtool build --dbg` |
| `ba()` | 全量编译：编译所有修改过的顶层模块 |

两种编译后端：
- **buildtool 模式**（新版）：`buildtool build --dbg -p <modules>`
- **apollo.sh 模式**（旧版）：`./apollo.sh build_dbg --jobs=24 --cpus=24 <modules>`

### 2.3 Alias 快捷命令

| Alias | 说明 |
|-------|------|
| `apollo_planning` | 启动规划模块（mainboard） |
| `apollo_routing` | 启动路由模块 |
| `apollo_external` | 启动外部命令模块 |
| `dv_start` | 启动 Dreamview+ |
| `dbg` | GDB 调试最新 core dump |
| `pack` | 打包规划模块为 tar.gz |
| `format` | clang-format 格式化代码 |
| `format_cached` | 格式化已暂存文件 |
| `format_unstaged` | 格式化未暂存文件 |

### 2.4 清理与工具

| 函数 | 说明 |
|------|------|
| `clean()` | 删除空目录、断链、日志、core dump，重置 profile |
| `mcp()` | rsync 安全复制（支持进度和断点续传） |

### 2.5 安装函数

| 函数 | 说明 |
|------|------|
| `install_fd()` | 安装 fd 文件查找工具 (v10.2.0) |
| `install_lsb()` | 伪装 lsb_release 为 Ubuntu 18.04（兼容性） |
| `install_uname()` | 伪装 uname 为 Ubuntu 18.04 内核（兼容性） |
| `build_node()` | 从源码编译 Node.js v18.x |

### 2.6 配置函数

| 函数 | 说明 |
|------|------|
| `change_apt_source()` | 更换 APT 源为中科大镜像 |
| `install_packages()` | 安装 clang-format、git、openssh-server |
| `configure_ssh()` | 修改 SSH 端口为 223，配置公钥 |
| `configure_git()` | 设置 Git 用户名和邮箱 |
| `install_node()` | 通过 nvm 安装 Node.js v22 + Claude Code |

### 2.7 主初始化函数

| 函数 | 步骤 |
|------|------|
| `init()` | 完整初始化：apt 源 → SSH → Git → 基础包 → Node.js → fd → lsb 伪装 → uname 伪装 |
| `init10()` | 精简初始化：跳过 lsb/uname 伪装 |

## 3. setup.sh

```bash
#!/bin/bash
source init.sh
init
```

- 一行命令完成完整环境初始化

## 4. kill_all.sh

```bash
#!/bin/bash
mkill mainboard
mkill planning
mkill control
mkill routing
```

- 杀死所有 Apollo 相关进程

## 5. 开发工作流

```
修改代码 → b (增量编译) → format (格式化) → apollo_planning (测试)
                                              ↓
                                        dbg (调试 core dump)
                                              ↓
                                        pack (打包提交)
```

## 6. 兼容性伪装说明

`install_lsb()` 和 `install_uname()` 将系统信息伪装为 Ubuntu 18.04，原因是 Apollo 构建系统（Bazel）在启动时会检查系统版本，非 18.04 环境会报错。伪装脚本支持所有标准参数，行为与原版一致，仅返回固定值。