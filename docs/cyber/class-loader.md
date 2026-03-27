---
title: ClassLoader - 类加载与插件管理
---

# ClassLoader - 类加载与插件管理

## 模块职责概述

ClassLoader 模块实现了 Apollo Cyber 框架的动态类加载机制，允许在运行时从共享库（.so 文件）中加载和实例化 C++ 类。它是 Apollo 组件化架构的基石，使得各功能模块可以编译为独立的共享库，由 mainboard 在运行时按需加载。配合 PluginManager 子模块，还支持基于 XML 描述文件的插件发现和懒加载。

## 核心类/接口说明

### ClassLoader

单个共享库的加载器，负责加载 .so 文件并从中创建类实例。

```cpp
class ClassLoader {
 public:
  explicit ClassLoader(const std::string& library_path);
  virtual ~ClassLoader();

  bool IsLibraryLoaded();
  bool LoadLibrary();
  int UnloadLibrary();
  const std::string GetLibraryPath() const;

  template <typename Base>
  std::vector<std::string> GetValidClassNames();

  template <typename Base>
  std::shared_ptr<Base> CreateClassObj(const std::string& class_name);

  template <typename Base>
  bool IsClassValid(const std::string& class_name);
};
```

关键行为：
- 构造时自动调用 `LoadLibrary()` 加载共享库
- `CreateClassObj<Base>()` 通过工厂模式创建类实例，返回 `shared_ptr`，析构时自动递减引用计数
- 使用引用计数管理库的加载/卸载：`loadlib_ref_count_` 跟踪加载次数，`classobj_ref_count_` 跟踪存活对象数
- 当仍有存活对象时，`UnloadLibrary()` 不会真正卸载共享库

### ClassLoaderManager

多库管理器，维护库路径到 ClassLoader 的映射，提供跨库的类查找和实例化能力。

```cpp
class ClassLoaderManager {
 public:
  bool LoadLibrary(const std::string& library_path);
  void UnloadAllLibrary();
  bool IsLibraryValid(const std::string& library_path);

  template <typename Base>
  std::shared_ptr<Base> CreateClassObj(const std::string& class_name);

  template <typename Base>
  std::shared_ptr<Base> CreateClassObj(const std::string& class_name,
                                       const std::string& library_path);

  template <typename Base>
  bool IsClassValid(const std::string& class_name);

  template <typename Base>
  std::vector<std::string> GetValidClassNames();

  template <typename Base>
  std::string GetClassValidLibrary(const std::string& class_name);
};
```

关键行为：
- `CreateClassObj(class_name)` 遍历所有已加载的 ClassLoader，找到第一个能创建该类的加载器
- `CreateClassObj(class_name, library_path)` 从指定库中创建类实例
- 内部使用 `std::map<std::string, ClassLoader*>` 维护映射关系

### CLASS_LOADER_REGISTER_CLASS 宏

类注册宏，在共享库加载时自动将类注册到工厂系统。

```cpp
#define CLASS_LOADER_REGISTER_CLASS(Derived, Base) \
  CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, __COUNTER__)
```

展开后生成一个静态全局对象，其构造函数调用 `utility::RegisterClass<Derived, Base>()`，利用 C++ 静态初始化机制在 `dlopen()` 时自动执行注册。

### SharedLibrary

对 `dlopen` / `dlsym` / `dlclose` 的封装，提供跨平台的共享库操作接口。

```cpp
class SharedLibrary {
 public:
  enum Flags { SHLIB_GLOBAL = 1, SHLIB_LOCAL = 2 };

  void Load(const std::string& path);
  void Load(const std::string& path, int flags);
  void Unload();
  bool IsLoaded();
  bool HasSymbol(const std::string& name);
  void* GetSymbol(const std::string& name);
  const std::string& GetPath() const;
};
```

### utility 命名空间

底层工厂注册和类创建的核心实现。

```cpp
namespace utility {
  // 注册类到全局工厂映射
  template <typename Derived, typename Base>
  void RegisterClass(const std::string& class_name,
                     const std::string& base_class_name);

  // 从工厂创建类实例
  template <typename Base>
  Base* CreateClassObj(const std::string& class_name, ClassLoader* loader);

  // 获取某个 ClassLoader 拥有的所有有效类名
  template <typename Base>
  std::vector<std::string> GetValidClassNames(ClassLoader* loader);

  // 库加载/卸载
  bool LoadLibrary(const std::string& library_path, ClassLoader* loader);
  void UnloadLibrary(const std::string& library_path, ClassLoader* loader);
}
```

核心数据结构：
- `BaseToClassFactoryMapMap`：`map<base_class_typeid, map<class_name, AbstractClassFactoryBase*>>`，二级映射，先按基类类型再按派生类名索引工厂对象
- `LibPathSharedLibVector`：已加载共享库的路径和 SharedLibrary 指针列表

### ClassFactory 体系

```cpp
// 工厂基类
class AbstractClassFactoryBase {
  void AddOwnedClassLoader(ClassLoader* loader);
  bool IsOwnedBy(const ClassLoader* loader);
  // ...
};

// 类型化工厂
template <typename Base>
class AbstractClassFactory : public AbstractClassFactoryBase {
  virtual Base* CreateObj() const = 0;
};

// 具体工厂
template <typename ClassObject, typename Base>
class ClassFactory : public AbstractClassFactory<Base> {
  Base* CreateObj() const { return new ClassObject; }
};
```

每个工厂对象记录了拥有它的 ClassLoader 列表，确保只有加载了对应库的 ClassLoader 才能创建该类的实例。

## PluginManager（插件管理器）

PluginManager 是 ClassLoader 的上层封装，提供基于 XML 描述文件的插件发现、注册和懒加载能力。

```cpp
class PluginManager {
 public:
  static PluginManager* Instance();  // 单例

  bool LoadPlugin(const std::string& plugin_description_file_path);
  bool LoadInstalledPlugins();

  template <typename Base>
  std::shared_ptr<Base> CreateInstance(const std::string& derived_class);

  template <typename Base>
  std::vector<std::string> GetDerivedClassNameByBaseClass();
};
```

### PluginDescription

插件描述信息，从 XML 文件解析而来。

```cpp
class PluginDescription {
 public:
  std::string name_;
  std::string description_path_;
  std::string actual_library_path_;
  std::map<std::string, std::string> class_name_base_class_name_map_;

  bool ParseFromIndexFile(const std::string& file_path);
  bool ParseFromDescriptionFile(const std::string& file_path);
};
```

插件描述 XML 格式示例：

```xml
<library path="lib/libmy_plugin.so">
  <class type="MyDerivedClass" base_class="MyBaseClass"/>
</library>
```

### 插件发现机制

1. 通过环境变量 `APOLLO_PLUGIN_INDEX_PATH` 指定插件索引目录（支持 `:` 分隔多路径）
2. 索引目录下的每个文件是一个插件索引，内容为插件描述文件的路径
3. 插件描述文件（XML）定义了库路径和类映射关系
4. `LoadInstalledPlugins()` 自动扫描索引目录并加载所有插件
5. `CreateInstance<Base>()` 支持懒加载：首次创建时才真正加载共享库

## 数据流描述

### 类注册流程

```
dlopen() 加载共享库
  → 执行静态初始化
  → CLASS_LOADER_REGISTER_CLASS 宏生成的静态对象构造
  → utility::RegisterClass<Derived, Base>()
  → 创建 ClassFactory 对象
  → 注册到全局 BaseToClassFactoryMapMap
```

### 类实例化流程

```
ClassLoaderManager::CreateClassObj<Base>(class_name)
  → 遍历所有 ClassLoader
  → ClassLoader::IsClassValid<Base>(class_name)
  → ClassLoader::CreateClassObj<Base>(class_name)
    → utility::CreateClassObj<Base>(class_name, this)
    → 从 ClassClassFactoryMap 查找工厂
    → factory->CreateObj() (new Derived)
    → 包装为 shared_ptr（自定义 deleter 递减引用计数）
```

### 插件懒加载流程

```
PluginManager::CreateInstance<Base>(class_name)
  → 检查 ClassLoaderManager 是否已能创建该类
  → 若不能，查找 plugin_class_plugin_name_map_
  → 找到对应插件描述，加载其共享库
  → 再次通过 ClassLoaderManager 创建实例
```

## 配置方式

### 环境变量

| 环境变量 | 说明 |
|----------|------|
| `APOLLO_PLUGIN_INDEX_PATH` | 插件索引目录路径（`:` 分隔） |
| `APOLLO_PLUGIN_DESCRIPTION_PATH` | 插件描述文件搜索路径 |
| `APOLLO_PLUGIN_LIB_PATH` | 插件共享库搜索路径 |
| `APOLLO_LIB_PATH` | 模块共享库搜索路径 |

### 注册宏

在组件实现文件末尾使用：

```cpp
// 标准组件注册
CLASS_LOADER_REGISTER_CLASS(MyComponent, apollo::cyber::ComponentBase)

// 插件注册（等价于 CLASS_LOADER_REGISTER_CLASS）
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(MyPlugin, MyPluginBase)
```

## 与其他模块的关系

- **Mainboard**：`ModuleController` 使用 `ClassLoaderManager` 加载 DAG 配置中指定的共享库，并通过 `CreateClassObj<ComponentBase>()` 实例化组件
- **Component**：所有 Cyber 组件（`Component`、`TimerComponent`）通过 `CLASS_LOADER_REGISTER_CLASS` 宏注册到 ClassLoader 系统
- **PluginManager**：在 ClassLoader 之上提供插件发现和懒加载能力，mainboard 启动时可自动加载已安装的插件
