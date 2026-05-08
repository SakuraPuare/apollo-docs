# FeatureOutput 学习数据输出

> 源码位置：`modules/planning/planning_base/common/feature_output.h/.cc`

## 模块定位

FeatureOutput 是一个静态工具类，负责将规划过程中的学习数据帧（`LearningDataFrame`）收集并序列化到磁盘文件。用于离线训练数据采集，支持学习型规划器的数据管道。

## 类声明

```cpp
class FeatureOutput {
 public:
  FeatureOutput() = delete;  // 纯静态类
  static void Close();
  static void Clear();
  static bool Ready();
  static void InsertLearningDataFrame(const std::string& record_filename,
                                      const LearningDataFrame& learning_data_frame);
  static void InsertPlanningResult();
  static LearningDataFrame* GetLatestLearningDataFrame();
  static void WriteLearningData(const std::string& record_file);
  static void WriteRemainderiLearningData(const std::string& record_file);
  static int SizeOfLearningData();
 private:
  static LearningData learning_data_;
  static int learning_data_file_index_;
};
```

## 方法详解

### Clear() / Close()

```cpp
void FeatureOutput::Clear() {
  learning_data_.Clear();
  learning_data_file_index_ = 0;
}
void FeatureOutput::Close() { Clear(); }
```

- 重置内部状态，清空累积的学习数据
- `feature_output.cc:L30-L35`

### Ready()

```cpp
bool FeatureOutput::Ready() {
  Clear();
  return true;
}
```

- 清空后返回就绪状态
- `feature_output.cc:L37-L40`

### InsertLearningDataFrame()

```cpp
void FeatureOutput::InsertLearningDataFrame(
    const std::string& record_file,
    const LearningDataFrame& learning_data_frame) {
  learning_data_.add_learning_data_frame()->CopyFrom(learning_data_frame);
  if (learning_data_.learning_data_frame_size() >=
      FLAGS_learning_data_frame_num_per_file) {
    WriteLearningData(record_file);
  }
}
```

- 将一帧学习数据追加到内部缓冲
- 当帧数达到 `FLAGS_learning_data_frame_num_per_file` 时自动写入文件
- `feature_output.cc:L42-L52`

### GetLatestLearningDataFrame()

```cpp
LearningDataFrame* FeatureOutput::GetLatestLearningDataFrame() {
  const int size = learning_data_.learning_data_frame_size();
  return size > 0 ? learning_data_.mutable_learning_data_frame(size - 1) : nullptr;
}
```

- 返回最新一帧的可变指针，用于后续补充数据
- `feature_output.cc:L54-L58`

### WriteLearningData()

```cpp
void FeatureOutput::WriteLearningData(const std::string& record_file) {
  std::string src_file_name = record_file.substr(record_file.find_last_of("/") + 1);
  src_file_name = src_file_name.empty() ? "00000" : src_file_name;
  const std::string dest_file = absl::StrCat(
      FLAGS_planning_data_dir, "/", src_file_name, ".",
      learning_data_file_index_, ".bin");
  cyber::common::SetProtoToBinaryFile(learning_data_, dest_file);
  learning_data_.Clear();
  learning_data_file_index_++;
}
```

- 输出路径：`{FLAGS_planning_data_dir}/{record_filename}.{index}.bin`
- 使用 protobuf 二进制格式序列化
- 写入后清空缓冲并递增文件索引
- `feature_output.cc:L62-L73`

### WriteRemainderiLearningData()

```cpp
void FeatureOutput::WriteRemainderiLearningData(const std::string& record_file) {
  if (learning_data_.learning_data_frame_size() > 0) {
    WriteLearningData(record_file);
  }
}
```

- 将缓冲中剩余的不足一批的数据强制写出
- 通常在录制结束时调用
- `feature_output.cc:L75-L80`

## 配置项

| 参数 | 说明 |
|------|------|
| `FLAGS_learning_data_frame_num_per_file` | 每个文件包含的帧数 |
| `FLAGS_planning_data_dir` | 学习数据输出目录 |

## 数据流

```
PlanningComponent::RunOnce()
  └── LearningModelInferenceTask / FeatureGenerator
        └── FeatureOutput::InsertLearningDataFrame()
              └── 累积到阈值 → WriteLearningData() → .bin 文件
```
