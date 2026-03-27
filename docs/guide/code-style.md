---
title: 代码规范指南
description: Apollo 项目代码风格、命名约定、Lint 工具链与最佳实践的完整参考
---

# 代码规范指南

Apollo 项目在 C++、Python、Shell、Protobuf 和 Bazel BUILD 文件上均有明确的代码风格要求。本文档基于源码中的实际配置文件和代码样例，系统性地梳理这些规范。

## 总体原则

- C/C++ 遵循 [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- Python 遵循 [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
- 所有文本文件使用 UTF-8 编码、2 空格缩进（前端项目为 4 空格）、LF 换行
- 每行不超过 80 个字符（由 `CPPLINT.cfg` 中 `linelength=80` 强制）
- 代码注释必须使用英文

## EditorConfig 配置

项目根目录的 `.editorconfig` 定义了基础格式：

```ini
[*]
charset = utf-8
indent_style = space
indent_size = 2
insert_final_newline = true
trim_trailing_whitespace = true

[*.{sh,bash,bashrc}]
shell_variant = bash
switch_case_indent = true
space_redirects = true
binary_next_line = true

[Makefile]
indent_style = tab
```

前端项目 (`modules/dreamview_plus/frontend`) 使用 4 空格缩进。

## C++ 代码规范

### Clang-Format 配置

项目使用基于 Google 风格的 `.clang-format`，关键设置：

```yaml
BasedOnStyle: Google
Language: Cpp
Cpp11BracedListStyle: true
Standard: Cpp11
CommentPragmas: '^ NOLINT'
```

头文件包含顺序通过 `IncludeCategories` 精确控制，优先级从低到高：

| 优先级 | 类别 | 示例 |
|--------|------|------|
| 0 | 对应的主头文件 | `"modules/common/filters/digital_filter.h"` |
| 1 | C 标准库 | `<string.h>`, `<sys/types.h>` |
| 2 | C++ 标准库 | `<vector>`, `<string>` |
| 3 | 系统/第三方库 | `<cuda.h>`, `<tinyxml2.h>` |
| 4 | 测试库 | `"gtest/gtest.h"` |
| 5 | 其他 | 未分类头文件 |
| 6 | Protobuf 生成文件 | `*.pb.h` |
| 7 | Apollo 内部库 | `"cyber/..."`, `"modules/..."` |

运行格式化：

```bash
# 格式化指定文件或目录
bash apollo.sh format -c path/to/cpp/files

# 或直接调用脚本
scripts/clang_format.sh path/to/cpp/dirs/or/files
```

### CPPLINT 配置

`CPPLINT.cfg` 位于项目根目录，核心规则：

```ini
set noparent
filter=-build/c++11
filter=-build/include_alpha,+build/include_order
filter=+build/include_what_you_use
filter=-build/header_guard
filter=+runtime/printf,+runtime/printf_format
filter=-runtime/references
linelength=80
includeorder=standardcfirst
```

要点：
- 禁用了 `build/c++11` 警告（允许 C++11 特性）
- 禁用了 `build/header_guard` 警告（推荐使用 `#pragma once`）
- 禁用了 `runtime/references`（允许非 const 引用参数）
- 头文件包含顺序要求 C 标准库优先（`standardcfirst`）

### 命名规范

从源码中总结的 C++ 命名约定（与 Google C++ Style 一致）：

```cpp
// 命名空间：全小写，用下划线分隔
namespace apollo {
namespace common {

// 类名：大驼峰（PascalCase）
class DigitalFilter { ... };       // common::DigitalFilter
class VehicleConfigHelper { ... }; // common::VehicleConfigHelper

namespace math {
class AABox2d { ... };             // common::math::AABox2d
}  // namespace math
}  // namespace common

namespace cyber {
class AtomicRWLock { ... };        // cyber::AtomicRWLock
}  // namespace cyber

// 公有方法：大驼峰
double Filter(const double x_insert);
void GetAllCorners(std::vector<Vec2d>* const corners) const;
bool IsPointIn(const Vec2d& point) const;
double DistanceTo(const Vec2d& point) const;
static void Init();

// setter 方法：小写下划线（set_ 前缀）
void set_denominators(const std::vector<double>& denominators);
void set_dead_zone(const double deadzone);

// getter 方法：小写下划线（无 get_ 前缀）
const std::vector<double>& denominators() const;
double dead_zone() const;
double length() const;

// 私有成员变量：小写下划线 + 尾部下划线
std::deque<double> x_values_;
double dead_zone_ = 0.0;
bool write_first_ = true;

// 局部变量：小写下划线
double y_insert = 0.0;
uint32_t retry_times = 0;

// 常量：k 前缀 + 大驼峰
const double kDoubleEpsilon = 1.0e-6;

// 静态常量成员：全大写下划线
static const int32_t RW_LOCK_FREE = 0;
static const int32_t WRITE_EXCLUSIVE = -1;
static const uint32_t MAX_RETRY_TIMES = 5;

}  // namespace math
}  // namespace common
}  // namespace apollo
```

### 函数签名约定

```cpp
// 输入对象参数：const 引用（保证非空）
// 输入标量参数：值传递（更好的局部性和性能）
// 输出参数：裸指针（调用方负责保证有效性）
void FooBar(const InputObjectType& input1,
            const InputScalaType input2,
            OutputType* output1);

// 单一返回值：利用 RVO 机制避免拷贝
OutputType FooBar(const InputType& input);
```

### 头文件保护

`modules/` 目录下的代码推荐使用 `#pragma once`，而 `cyber/` 框架代码主要使用传统 include guard：

```cpp
// 推荐
#pragma once

// 旧风格（路径全大写 + 下划线）
#ifndef CYBER_BASE_ATOMIC_RW_LOCK_H_
#define CYBER_BASE_ATOMIC_RW_LOCK_H_
// ...
#endif  // CYBER_BASE_ATOMIC_RW_LOCK_H_
```

### 文件头许可证

每个源文件必须包含 Apache 2.0 许可证头：

```cpp
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
```

### Doxygen 注释

公有 API 使用 Doxygen 风格注释：

```cpp
/**
 * @file
 * @brief Defines the DigitalFilter class.
 */

/**
 * @class DigitalFilter
 * @brief The DigitalFilter class is used to pass signals with a frequency
 * lower than a certain cutoff frequency.
 */

/**
 * @brief Processes a new measurement with the filter.
 * @param x_insert The new input to be processed by the filter.
 */
double Filter(const double x_insert);
```

## Python 代码规范

### Flake8 配置

`tox.ini` 中定义了 Flake8 规则：

```ini
[flake8]
ignore = D203, W503, E203
max-complexity = 10
import-order-style = google
```

- `D203`：忽略 docstring 前空行要求
- `W503`：忽略二元运算符前换行警告
- `E203`：忽略切片操作前空格警告
- 圈复杂度上限为 10
- import 顺序遵循 Google 风格

### 格式化工具

使用 `yapf` 进行 Python 代码格式化：

```bash
# 格式化单个文件
yapf -i --style='{based_on_style: google}' foo.py

# 通过 apollo.sh 格式化
bash apollo.sh format -p path/to/python/files
```

### Python 命名与文件头

```python
#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2020 The Apollo Authors. All Rights Reserved.
# ... (Apache 2.0 License)
# ****************************************************************************

# 模块级常量：全大写下划线
RESPEAKER_CHANNEL = "/apollo/sensor/microphone"
WAV_SAVING_PATH = "/tmp"

# 函数名：小写下划线
def save_to_wave(frames, filepath, sample_width, sample_rate, n_channels=1):
    """Save frame to file.wave"""
    pass

# 变量名：小写下划线
sample_width = 0
sample_rate = 0
```

## Protobuf 规范

```protobuf
syntax = "proto2";

// 包名：小写点分隔，与目录结构对应
package apollo.audio;

// 消息名：大驼峰
message TopicConf {
  // 字段名：小写下划线 + 序号
  optional string audio_data_topic_name = 1;
  optional string audio_detection_topic_name = 2;
}

message AudioConf {
  optional TopicConf topic_conf = 1;
  optional string respeaker_extrinsics_path = 2;
}
```

Protobuf 文件同样通过 `clang-format` 格式化。

## Shell 脚本规范

- 使用 `shellcheck` 进行静态检查
- 使用 `shfmt` 进行格式化
- 文件头使用 `#!/usr/bin/env bash` 或 `#! /usr/bin/env bash`
- 许可证头使用 `#` 注释风格

```bash
bash apollo.sh lint --sh    # 运行 shellcheck
bash apollo.sh format -s path/to/shell/files  # 运行 shfmt
```

## Bazel BUILD 文件规范

### 结构约定

每个 BUILD 文件末尾必须调用 `cpplint()`，确保 C++ 源码被 lint 覆盖：

```python
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_package")

package(default_visibility = ["//visibility:public"])

# 一个 target 最多包含一个 .h 和一个 .cc
apollo_cc_library(
    name = "foobar",
    hdrs = ["foobar.h"],
    srcs = ["foobar.cc"],
    deps = [
        "//cyber",
        "//modules/common/math",
    ],
)

# 测试文件以 _test.cc 结尾，target 名以 _test 结尾
apollo_cc_test(
    name = "foobar_test",
    size = "small",
    srcs = ["foobar_test.cc"],
    deps = [
        ":foobar",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

# 必须放在 BUILD 文件末尾
cpplint()
```

### 依赖管理

只列出直接依赖，不要列出传递依赖：

```python
# sandwich.h 包含 bread.h，bread.h 包含 flour.h
apollo_cc_library(
    name = "sandwich",
    hdrs = ["sandwich.h"],
    srcs = ["sandwich.cc"],
    deps = [
        ":bread",
        # 不要添加 ":flour" —— 它是 bread 的传递依赖
    ],
)
```

格式化 BUILD 文件：

```bash
bash apollo.sh format path/to/BUILD/files
# 或使用 buildifier
buildifier -lint=fix path/to/BUILD
```

## Lint 工具链总览

| 语言 | Lint 工具 | 格式化工具 | 运行命令 |
|------|-----------|------------|----------|
| C/C++ | cpplint | clang-format | `bash apollo.sh lint --cpp` |
| Python | flake8 | yapf (Google style) | `bash apollo.sh lint --py` |
| Shell | shellcheck | shfmt | `bash apollo.sh lint --sh` |
| Bazel | buildifier | buildifier | `bash apollo.sh format -b` |
| Protobuf | - | clang-format | `bash apollo.sh format -c` |
| Markdown | - | prettier | `bash apollo.sh format -m` |

一键运行所有检查：

```bash
# 编译 + 测试 + lint
bash apollo.sh check

# 仅 lint（默认 C++）
bash apollo.sh lint

# lint 全部语言
bash apollo.sh lint --all

# 格式化全部
bash apollo.sh format -a path/to/dir
```

## 最佳实践

1. 提交 PR 前务必在本地运行 `bash apollo.sh check`，确保编译、测试和 lint 全部通过

2. 尽可能使用 `const` 修饰变量和函数：
   ```cpp
   const size_t current_size = vec.size();
   const std::string& name() const;
   ```

3. 使用 C++ 头文件而非 C 头文件：
   ```cpp
   // 推荐
   #include <ctime>
   #include <cmath>
   #include <cstdio>

   // 避免
   #include <time.h>
   #include <math.h>
   #include <stdio.h>
   ```

4. 只包含必需的头文件，不多也不少。通过 `clang-format` 自动修正包含顺序

5. 遵循 DRY 原则。重复使用的长路径名应设置别名：
   ```cpp
   // 使用两次以上时，设置短别名
   using apollo::common::util::Type;

   // 多次访问 protobuf 嵌套字段时，保存共同前缀
   const auto& field_2 = a_proto.field_1().field_2();
   ```

6. 单元测试文件与源文件同目录，命名为 `*_test.cc`：
   ```
   foobar.h
   foobar.cc
   foobar_test.cc
   ```

7. 文件名使用小写下划线风格：`digital_filter.h`、`vehicle_config_helper.cc`、`atomic_rw_lock.h`
