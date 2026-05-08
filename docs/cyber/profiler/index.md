---
title: "Profiler 性能分析器"
---

# Profiler

> 源码路径: `cyber/profiler/`

## 概述

profiler 模块为 Cyber RT 提供**协程级别的代码块计时分析**能力。用户通过 `PERF_BLOCK` / `PERF_BLOCK_END` / `PERF_FUNCTION` 宏在代码中插入测量点，框架自动记录每个代码块的起止时间（纳秒精度）、嵌套深度，并在协程执行完毕后将该协程的完整调用帧数据写入日志文件。

核心设计要点：

- **协程隔离**：`BlockManager` 使用 `thread_local` 存储 `routine_frame_map_`，按协程名隔离各自的 `Frame` 数据，互不干扰
- **RAFI 自动结束**：`Block` 析构时自动调用 `EndBlock()`，确保异常退出也能正确记录
- **按需编译**：通过 `ENABLE_PROFILER` 宏控制，未定义时所有测量宏展开为空操作，零开销
- **栈式嵌套**：`Frame` 内部使用 `std::stack` 管理 `Block` 指针，天然支持函数嵌套调用的层级记录

## 核心类

### Block — 计时块

表示一个被测量的代码区域，记录名称、嵌套深度和起止时间戳。

```cpp
// cyber/profiler/block.h
class Block {
 public:
  using time_point = std::chrono::time_point<std::chrono::steady_clock>;

  Block();
  explicit Block(const std::string& name);
  virtual ~Block();  // 析构时自动 EndBlock()

  void Start();
  void End();

  const std::string& name() const { return name_; }
  std::uint32_t depth() const { return depth_; }
  void set_depth(std::uint32_t depth) { depth_ = depth; }

  std::uint64_t duration() const;           // 纳秒
  std::uint64_t begin_time_since_epoch() const;
  std::uint64_t end_time_since_epoch() const;
  bool finished() const { return end_time_ > begin_time_; }

 private:
  std::string name_;
  std::uint32_t depth_;
  time_point begin_time_;
  time_point end_time_;
};
```

关键实现细节：析构函数中检查 `finished()` 状态，若未结束则自动调用 `BlockManager::Instance()->EndBlock()`，保证 RAII 语义。

### BlockManager — 块管理器（单例）

协调 `Block` 的生命周期，按协程名管理独立的 `Frame` 实例。

```cpp
// cyber/profiler/block_manager.h
class BlockManager {
 public:
  using RoutineName = std::string;
  using RoutineFrameMap = std::unordered_map<RoutineName, Frame>;

  void StartBlock(Block* block);
  void EndBlock();

 private:
  std::string GetRoutineName();
  Frame* GetRoutineFrame();

  static thread_local RoutineFrameMap routine_frame_map_;
  DECLARE_SINGLETON(BlockManager)
};
```

`routine_frame_map_` 声明为 `thread_local`，每个线程拥有独立副本，避免跨线程竞争。

### Frame — 调用帧

以栈结构管理一次协程执行中产生的所有 `Block`，支持完整帧的序列化输出。

```cpp
// cyber/profiler/frame.h
class Frame {
 public:
  void Push(Block* block);
  Block* Top();
  void Pop();

  bool DumpToFile(const std::string& coroutine_name);
  void Clear();

  std::uint32_t size() const { return stack_.size(); }
  bool finished() const { return stack_.empty(); }

 private:
  std::stack<Block*> stack_;     // 当前活跃 Block 栈
  std::list<Block> storage_;     // 已完成 Block 的副本存储
};
```

`Pop()` 时将 `Block` 的值拷贝移入 `storage_`，确保栈弹出后数据仍然可用。

## 核心函数

### PERF_BLOCK / PERF_FUNCTION — 用户宏

```cpp
// cyber/profiler/profiler.h
#define PERF_BLOCK(name, ...)                                    \
  apollo::cyber::profiler::Block UNIQUE_NAME(__LINE__)(name);    \
  apollo::cyber::profiler::BlockManager::Instance()->StartBlock( \
      &UNIQUE_NAME(__LINE__));

#define PERF_BLOCK_END \
  apollo::cyber::profiler::BlockManager::Instance()->EndBlock();

#define PERF_FUNCTION(...) PERF_BLOCK(AFUNC, ## __VA_ARGS__)
```

| 宏 | 职责 | 输入 | 输出 |
|---|---|---|---|
| `PERF_BLOCK(name)` | 在当前位置创建一个命名 `Block` 并注册到管理器 | `name`: 代码块名称 | 无（RAII 管理） |
| `PERF_BLOCK_END` | 手动结束当前栈顶 `Block` | 无 | 无 |
| `PERF_FUNCTION()` | 自动以函数签名作为名称创建 `Block` | 无（或自定义名） | 无（RAII 管理） |

关键步骤：

1. 宏展开时通过 `__LINE__` 生成唯一变量名，避免同名冲突
2. `PERF_FUNCTION` 使用 `__PRETTY_FUNCTION__`（GCC/Clang）或 `__func__` 作为块名称
3. 未定义 `ENABLE_PROFILER` 时，所有宏展开为空，编译器完全消除相关代码

### BlockManager::StartBlock — 开始计时

```cpp
void BlockManager::StartBlock(Block* block) {
  Frame* frame_ptr = GetRoutineFrame();
  if (frame_ptr == nullptr || block == nullptr)
    return;
  frame_ptr->Push(block);
  block->set_depth(frame_ptr->size());
  block->Start();
}
```

| 项目 | 说明 |
|---|---|
| 职责 | 将 Block 压入当前协程的 Frame 栈并启动计时 |
| 输入 | `block`: 待启动的 Block 指针 |
| 输出 | 无 |
| 关键步骤 | 获取当前协程的 Frame -> 压栈 -> 设置嵌套深度 -> 记录起始时间 |

### BlockManager::EndBlock — 结束计时

```cpp
void BlockManager::EndBlock() {
  Frame* frame_ptr = GetRoutineFrame();
  if (frame_ptr == nullptr || frame_ptr->finished())
    return;

  Block* block = frame_ptr->Top();
  block->End();
  frame_ptr->Pop();

  if (frame_ptr->finished()) {
    const std::string& routine_name = GetRoutineName();
    frame_ptr->DumpToFile(routine_name);
    frame_ptr->Clear();
  }
}
```

| 项目 | 说明 |
|---|---|
| 职责 | 结束栈顶 Block 并在协程帧清空时触发日志转储 |
| 输入 | 无（从当前协程 Frame 栈顶取 Block） |
| 输出 | 无 |
| 关键步骤 | 获取栈顶 Block -> 记录结束时间 -> 弹栈 -> 若栈为空则 DumpToFile 并 Clear |

### Frame::DumpToFile — 导出帧数据

```cpp
bool Frame::DumpToFile(const std::string& routine_name) {
  ALOG_MODULE(kModuleName, INFO) << "Frame : " << routine_name;
  for (const Block& block : storage_) {
    ALOG_MODULE(kModuleName, INFO)
        << block.depth() << "," << block.name() << "," << block.duration()
        << "," << block.begin_time_since_epoch()
        << "," << block.end_time_since_epoch();
  }
  return true;
}
```

| 项目 | 说明 |
|---|---|
| 职责 | 将一帧中所有已完成 Block 的计时数据写入 perf 模块日志 |
| 输入 | `routine_name`: 协程名称 |
| 输出 | `true` 表示写入成功 |
| 关键步骤 | 输出帧头 -> 遍历 storage_ 逐行输出 CSV 格式（深度, 名称, 耗时ns, 起始时间ns, 结束时间ns） |

## 配置

| 配置项 | 位置 | 类型 | 说明 |
|---|---|---|---|
| `ENABLE_PROFILER` | 编译宏 | bool | 启用/禁用整个 profiler 模块；未定义时所有宏为空操作 |
| 日志模块名 `"perf"` | `frame.cc` | 编译常量 | `DumpToFile` 使用 `ALOG_MODULE("perf", ...)` 输出，日志写入 perf 模块对应的日志文件 |

## 调用关系

```text
用户代码
  |
  | PERF_BLOCK(name) / PERF_FUNCTION()
  v
Block 构造 + BlockManager::StartBlock(block)
  |
  | 1. GetRoutineFrame()  -->  从 thread_local map 获取当前协程的 Frame
  | 2. Frame::Push(block)  -->  压入调用栈
  | 3. block->Start()      -->  记录开始时间
  |
  | ... 业务代码执行 ...
  |
  v
PERF_BLOCK_END / Block 析构
  |
  v
BlockManager::EndBlock()
  |
  | 1. Frame::Top()        -->  取栈顶 Block
  | 2. block->End()        -->  记录结束时间
  | 3. Frame::Pop()        -->  弹栈，Block 移入 storage_
  | 4. 若 Frame::finished():
  |    a. Frame::DumpToFile(routine_name)  -->  写入 CSV 日志
  |    b. Frame::Clear()                  -->  清空 storage_
```
