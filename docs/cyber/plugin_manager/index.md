---
title: "Plugin Manager"
---

# Plugin Manager

> 源码路径: `cyber/plugin_manager/`

## 概述

Plugin Manager 是 Cyber RT 的插件管理框架，提供插件的发现、注册、懒加载和实例化能力。它基于 Class Loader 机制，通过 XML 描述文件定义插件元数据（共享库路径、派生类与基类映射），并在运行时按需加载共享库、创建插件实例。

核心设计特点：

- **懒加载**：启动时仅解析插件索引文件建立映射表，首次使用时才加载共享库
- **索引 + 描述两级文件体系**：索引文件指向描述文件路径，描述文件（XML）包含库路径和类注册信息
- **环境变量驱动的路径搜索**：通过 `APOLLO_PLUGIN_INDEX_PATH`、`APOLLO_PLUGIN_DESCRIPTION_PATH`、`APOLLO_PLUGIN_LIB_PATH` 等环境变量定位插件文件
- **单例模式**：全局唯一 `PluginManager` 实例管理所有插件

## 核心类

### PluginDescription

插件元数据描述类，负责解析插件索引文件和 XML 描述文件，提取插件名称、共享库路径以及类注册信息。

```cpp
class PluginDescription {
 public:
  std::string name_;
  std::string description_index_path_;
  std::string description_path_;
  std::string actual_description_path_;
  std::string library_path_;
  std::string actual_library_path_;
  std::map<std::string, std::string> class_name_base_class_name_map_;

  bool ParseFromIndexFile(const std::string& file_path);
  bool ParseFromDescriptionFile(const std::string& file_path);
};
```

### PluginManager

插件管理器单例，协调插件的发现、加载、实例化全流程。内部持有 `ClassLoaderManager` 完成底层的动态库加载和类实例化。

```cpp
class PluginManager {
 private:
  apollo::cyber::class_loader::ClassLoaderManager class_loader_manager_;
  std::map<std::string, std::shared_ptr<PluginDescription>> plugin_description_map_;
  std::map<std::string, bool> plugin_loaded_map_;
  std::map<std::pair<std::string, std::string>, std::string> plugin_class_plugin_name_map_;
  static PluginManager* instance_;
};
```

### CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN 宏

插件注册宏，将派生类与基类绑定，实际委托给 `CLASS_LOADER_REGISTER_CLASS`：

```cpp
#define CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(name, base) \
  CLASS_LOADER_REGISTER_CLASS(name, base)
```

## 核心函数

### PluginDescription::ParseFromIndexFile

解析插件索引文件，读取其中存储的描述文件路径，再调用 `ParseFromDescriptionFile` 完成完整解析。

- **输入**：`file_path` -- 索引文件路径
- **输出**：`bool` -- 解析是否成功
- **关键步骤**：
  1. 以文件名作为插件名称（`GetFileName`）
  2. 读取索引文件内容作为描述文件路径
  3. 委托 `ParseFromDescriptionFile` 解析 XML 描述

### PluginDescription::ParseFromDescriptionFile

解析插件 XML 描述文件，提取共享库路径和类注册映射。

- **输入**：`file_path` -- 描述文件路径
- **输出**：`bool` -- 解析是否成功
- **关键步骤**：
  1. 通过环境变量 `APOLLO_PLUGIN_DESCRIPTION_PATH` 解析描述文件的实际路径
  2. 使用 tinyxml2 解析 XML，从根元素 `path` 属性获取库路径
  3. 遍历 `<class>` 子元素，提取 `type`（派生类）和 `base_class`（基类）属性
  4. 通过环境变量 `APOLLO_PLUGIN_LIB_PATH` 解析共享库的实际路径

### PluginManager::LoadInstalledPlugins

加载系统中已安装的所有插件，是插件自动发现的入口。

- **输入**：无
- **输出**：`bool` -- 始终返回 `true`
- **关键步骤**：
  1. 若环境变量 `APOLLO_PLUGIN_SEARCH_IN_BAZEL_OUTPUT=1`，扫描 `$APOLLO_ROOT_DIR/bazel-bin` 下的 `cyber_plugin_index` 目录（开发模式）
  2. 读取 `APOLLO_PLUGIN_INDEX_PATH` 环境变量（冒号分隔的多路径）
  3. 若未设置，回退到 `$APOLLO_DISTRIBUTION_HOME/share/cyber_plugin_index`
  4. 逐个路径调用 `FindPluginIndexAndLoad` 加载插件索引

### PluginManager::FindPluginIndexAndLoad

扫描指定目录下的所有插件索引文件，解析并注册插件（懒加载，不立即加载库）。

- **输入**：`plugin_index_path` -- 索引文件目录
- **输出**：`bool` -- 是否全部成功
- **关键步骤**：
  1. 使用 `FindPathByPattern` 列出目录下所有常规文件
  2. 对每个索引文件，构造 `PluginDescription` 并调用 `ParseFromIndexFile`
  3. 将解析结果写入 `plugin_description_map_` 和 `plugin_class_plugin_name_map_`
  4. 标记 `plugin_loaded_map_` 为 `false`（未加载库，仅注册元数据）

### PluginManager::LoadPlugin

从描述文件路径显式加载一个插件（立即加载共享库）。

- **输入**：`plugin_description_file_path` -- 描述文件路径
- **输出**：`bool` -- 加载是否成功
- **关键步骤**：
  1. 构造 `PluginDescription` 并调用 `ParseFromDescriptionFile`
  2. 检查是否已加载（按名称去重）
  3. 调用 `LoadLibrary` 加载共享库
  4. 注册到三个内部映射表

### PluginManager::CreateInstance\<Base\>

创建指定派生类的插件实例，是插件使用的主要入口。

- **输入**：`derived_class` -- 派生类名称
- **输出**：`std::shared_ptr<Base>` -- 插件实例指针，失败返回 `nullptr`
- **关键步骤**：
  1. 调用 `CheckAndLoadPluginLibrary` 确保共享库已加载
  2. 委托 `ClassLoaderManager::CreateClassObj` 创建实例

### PluginManager::CheckAndLoadPluginLibrary\<Base\>

检查插件库是否已加载，若未加载则触发懒加载。

- **输入**：`class_name` -- 派生类名称
- **输出**：`bool` -- 加载是否成功
- **关键步骤**：
  1. 调用 `IsLibraryLoaded` 检查加载状态
  2. 若未加载，通过 `plugin_class_plugin_name_map_` 查找插件名称
  3. 从 `plugin_description_map_` 获取描述信息，调用 `LoadLibrary` 加载

### PluginManager::GetPluginClassHomePath\<Base\>

获取插件类对应的描述文件所在目录路径（去掉 `share/` 前缀后的相对路径）。

- **输入**：`class_name` -- 派生类名称
- **输出**：`std::string` -- 插件 Home 路径，未找到返回空串

### PluginManager::GetPluginConfPath\<Base\>

获取插件配置文件的完整路径，结合 `APOLLO_CONF_PATH` 环境变量解析。

- **输入**：`class_name` -- 派生类名称；`conf_name` -- 配置文件名
- **输出**：`std::string` -- 配置文件完整路径

### PluginManager::GetDerivedClassNameByBaseClass\<Base\>

根据基类类型获取所有已注册的派生类名称列表。

- **输入**：无（通过模板参数 `Base` 推导基类）
- **输出**：`std::vector<std::string>` -- 派生类名称列表

## 配置

| 环境变量 | 说明 | 默认值 |
|---|---|---|
| `APOLLO_PLUGIN_INDEX_PATH` | 插件索引目录路径，冒号分隔多路径 | `$APOLLO_DISTRIBUTION_HOME/share/cyber_plugin_index` |
| `APOLLO_PLUGIN_DESCRIPTION_PATH` | 描述文件搜索路径前缀 | 无 |
| `APOLLO_PLUGIN_LIB_PATH` | 共享库搜索路径前缀 | 无 |
| `APOLLO_CONF_PATH` | 配置文件搜索路径前缀 | 无 |
| `APOLLO_DISTRIBUTION_HOME` | Apollo 安装根目录 | 无 |
| `APOLLO_ROOT_DIR` | Apollo 源码根目录（开发模式） | 无 |
| `APOLLO_PLUGIN_SEARCH_IN_BAZEL_OUTPUT` | 设为 `1` 时扫描 `bazel-bin` 下插件 | 未设置 |

## 调用关系

```text
LoadInstalledPlugins()
  |
  +-- FindPlunginIndexPath()           // 扫描 cyber_plugin_index 目录
  +-- FindPluginIndexAndLoad(dir)      // 逐目录加载索引
        |
        +-- PluginDescription::ParseFromIndexFile()
              |
              +-- PluginDescription::ParseFromDescriptionFile()
                    |
                    +-- tinyxml2 解析 XML
                    +-- GetFilePathWithEnv() 解析路径

LoadPlugin(desc_file)
  |
  +-- PluginDescription::ParseFromDescriptionFile()
  +-- LoadLibrary()
        |
        +-- ClassLoaderManager::LoadLibrary()

CreateInstance<Base>(class_name)       // 用户侧 API
  |
  +-- CheckAndLoadPluginLibrary<Base>()
        |
        +-- IsLibraryLoaded<Base>()
        +-- LoadLibrary()              // 懒加载
  +-- ClassLoaderManager::CreateClassObj<Base>()
```
