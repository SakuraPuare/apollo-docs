---
title: "Context 上下文键值存储"
---

# Context

> 源码路径：`cyber/context/`

## 概述

Context 提供一个类型擦除的键值存储容器，用于在组件间传递任意类型的共享数据。支持线程安全（`SafeSet`/`SafeGet`）和非线程安全（`Set`/`Get`）两套接口。

## 核心类

### Context

> 源码文件：`cyber/context/context.h`

```cpp
class Context {
 public:
  Context() = default;

  template <typename T>
  void Set(const std::string &key, const std::shared_ptr<T> &value);

  template <typename T>
  void Set(const std::string &key, const T &value);

  template <typename T>
  std::shared_ptr<T> Get(const std::string &key) const;

  template <typename T>
  void SafeSet(const std::string &key, const std::shared_ptr<T> &value);

  template <typename T>
  void SafeSet(const std::string &key, const T &value);

  template <typename T>
  std::shared_ptr<T> SafeGet(const std::string &key) const;

 private:
  std::map<std::string, std::shared_ptr<void>> m_map_;
  std::mutex m_mutex_;
};
```

## 核心函数

### Context::Set()

```cpp
template <typename T>
void Context::Set(const std::string &key, const std::shared_ptr<T> &value) {
  m_map_[key] = value;
}

template <typename T>
void Context::Set(const std::string &key, const T &value) {
  m_map_[key] = std::make_shared<T>(value);
}
```

**职责**：将任意类型值存入 map，通过 `shared_ptr<void>` 实现类型擦除。

### Context::Get()

```cpp
template <typename T>
std::shared_ptr<T> Context::Get(const std::string &key) const {
  auto it = m_map_.find(key);
  if (it != m_map_.end()) {
    return std::static_pointer_cast<T>(it->second);
  }
  return nullptr;
}
```

**职责**：按 key 取值，通过 `static_pointer_cast` 还原类型。key 不存在时返回 nullptr。

### Context::SafeSet() / SafeGet()

```cpp
template <typename T>
void Context::SafeSet(const std::string &key, const std::shared_ptr<T> &value) {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return Set(key, value);
}

template <typename T>
std::shared_ptr<T> Context::SafeGet(const std::string &key) const {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return Get<T>(key);
}
```

**职责**：线程安全版本，通过 `std::mutex` 保护并发访问。

## 调用关系

- **被调用方**：可被任何需要跨组件传递数据的模块使用
- **无外部依赖**：纯头文件实现，仅依赖标准库
