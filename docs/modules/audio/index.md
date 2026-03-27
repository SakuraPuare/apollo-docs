---
title: Audio 音频模块
description: Apollo 自动驾驶平台音频处理模块，通过麦克风阵列检测紧急车辆警报声，输出警报器开关状态、声源方向、移动状态（靠近/远离），为自动驾驶决策提供听觉感知能力。
---

# Audio 音频模块

## 模块职责

Audio 模块是 Apollo 自动驾驶平台的听觉感知模块，专注于检测紧急车辆（如救护车、消防车、警车）发出的警报声。模块接收来自车载麦克风阵列的原始音频数据，通过三个独立的检测子系统协同工作：

- **SirenDetection（警报声检测）**：基于 PyTorch 深度学习模型判断是否存在警报声
- **DirectionDetection（方向检测）**：基于 GCC-PHAT 算法估算声源方向和位置
- **MovingDetection（移动检测）**：基于频谱分析判断声源是靠近、远离还是静止

检测结果通过 Cyber RT 发布到 `/apollo/audio_detection` 话题，供规划和决策模块使用。当检测到活动的紧急车辆时，Dreamview 上也会显示警报提示。

## 目录结构

```
modules/audio/
├── audio_component.cc/h           # Cyber 组件入口
├── BUILD                          # 构建规则
├── cyberfile.xml                  # 包管理配置
├── common/
│   ├── audio_gflags.cc/h          # 命令行参数定义
│   ├── audio_info.cc/h            # 音频信号缓存与管理
│   └── message_process.cc/h       # 消息处理流水线
├── conf/
│   ├── audio.conf                 # gflags 配置文件
│   ├── audio_conf.pb.txt          # Protobuf 配置文件
│   └── respeaker_extrinsics.yaml  # 麦克风外参标定
├── dag/
│   └── audio.dag                  # DAG 启动配置
├── inference/
│   ├── direction_detection.cc/h   # 声源方向检测
│   ├── moving_detection.cc/h      # 声源移动状态检测
│   ├── siren_detection.cc/h       # 警报声识别（深度学习）
│   ├── moving_detection_test.cc   # 移动检测单元测试
│   └── siren_detection_test.cc    # 警报检测单元测试
├── launch/
│   └── audio.launch               # cyber_launch 启动文件
├── proto/
│   └── audio_conf.proto           # 配置结构定义
└── tools/
    ├── audio_offline_processing.cc # 离线音频处理工具
    └── audiosaver.py               # 音频保存工具
```

## 核心类与接口

### AudioComponent（Cyber 组件）

模块的入口组件，继承自 `cyber::Component<AudioData>`，注册为 Cyber RT 组件。

```cpp
class AudioComponent
    : public cyber::Component<apollo::drivers::microphone::config::AudioData> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<AudioData>&) override;

 private:
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>> localization_reader_;
  std::shared_ptr<cyber::Writer<AudioDetection>> audio_writer_;
  AudioInfo audio_info_;
  DirectionDetection direction_detection_;
  MovingDetection moving_detection_;
  SirenDetection siren_detection_;
};
```

- `Init()`：加载配置文件，创建定位订阅器和检测结果发布器
- `Proc()`：每收到一帧麦克风数据，调用 `MessageProcess::OnMicrophone()` 执行完整检测流水线，将结果发布到输出话题

### MessageProcess（消息处理流水线）

静态工具类，编排三个检测子系统的调用顺序。

```cpp
class MessageProcess {
 public:
  static void OnMicrophone(
      const AudioData& audio_data,
      const std::string& respeaker_extrinsics_file,
      AudioInfo* audio_info,
      DirectionDetection* direction_detection,
      MovingDetection* moving_detection,
      SirenDetection* siren_detection,
      AudioDetection* audio_detection);
};
```

处理流程：
1. 将原始音频数据插入 `AudioInfo` 缓存
2. 调用 `DirectionDetection::EstimateSoundSource()` 估算声源位置和角度
3. 调用 `SirenDetection::Evaluate()` 判断是否为警报声（需要 72000 个采样点，约 3 秒 @24kHz）
4. 调用 `MovingDetection::Detect()` 判断声源移动状态

### AudioInfo（音频信号管理）

管理多通道音频信号的缓存，维护一个滑动窗口。

```cpp
class AudioInfo {
 public:
  void Insert(const AudioData& audio_data);
  std::vector<std::vector<double>> GetSignals(const int signal_length);

 private:
  std::vector<std::deque<double>> signals_;  // 每个通道一个双端队列
};
```

- `Insert()`：解析 `AudioData` 中的 RAW 类型通道数据，将 16-bit PCM 采样值转为 `double` 存入对应通道队列
- `GetSignals()`：从各通道队列尾部取指定长度的信号，供检测算法使用
- 缓存时长由 `--cache_signal_time` 控制（默认 3 秒）

### SirenDetection（警报声检测）

基于 PyTorch JIT 模型的警报声二分类器。

```cpp
class SirenDetection {
 public:
  SirenDetection();
  bool Evaluate(const std::vector<std::vector<double>>& signals);

 private:
  void LoadModel();
  torch::jit::script::Module torch_model_;
  torch::Device device_;
};
```

- 构造时自动加载模型，优先使用 CUDA（如可用），否则使用 CPU
- `Evaluate()`：接收 4 通道 x 72000 采样点的信号，归一化到 `[-1, 1]` 后送入模型
- 模型输出每个通道的正/负分数，通过 4 通道多数投票决定最终结果
- 使用 `omp_set_num_threads(1)` 和 `torch::set_num_threads(1)` 限制线程数，避免资源竞争

### DirectionDetection（方向检测）

基于 GCC-PHAT（广义互相关-相位变换）算法的声源定位。

```cpp
class DirectionDetection {
 public:
  std::pair<Point3D, double> EstimateSoundSource(
      std::vector<std::vector<double>>&& channels_vec,
      const std::string& respeaker_extrinsic_file,
      const int sample_rate, const double mic_distance);

 private:
  double EstimateDirection(std::vector<std::vector<double>>&& channels_vec,
                           const int sample_rate, const double mic_distance);
  double GccPhat(const torch::Tensor& sig, const torch::Tensor& refsig,
                 int fs, double max_tau, int interp);
};
```

算法流程：
1. 对麦克风阵列的对角通道对（ch0-ch2、ch1-ch3）分别计算 GCC-PHAT 互相关
2. 根据时延差（tau）和麦克风间距计算到达角（theta）
3. 综合两对通道的角度估计，得到最优方向（度数转弧度）
4. 通过麦克风外参矩阵（`respeaker_extrinsics.yaml`）将声源方向从麦克风坐标系转换到 IMU 坐标系
5. 假设声源距离 50m，输出声源的三维位置和角度

### MovingDetection（移动检测）

基于频谱分析的声源移动状态检测，利用多普勒效应原理。

```cpp
class MovingDetection {
 public:
  MovingResult Detect(const std::vector<std::vector<double>>& signals);
  MovingResult DetectSingleChannel(const std::size_t channel_index,
                                   const std::vector<double>& signal);

 private:
  SignalStat GetSignalStat(const std::vector<std::complex<double>>& fft_results,
                           const int start_frequency);
  MovingResult AnalyzePower(const std::deque<SignalStat>& signal_stats);
  MovingResult AnalyzeTopFrequence(const std::deque<SignalStat>& signal_stats);
};
```

检测逻辑：
1. 对每个通道的信号做 FFT（使用 FFTW3 库）
2. 统计频谱的总功率和峰值频率
3. 维护最近 10 帧的统计历史
4. 通过连续 3 帧的功率/频率变化趋势判断：
   - 功率递增 → `APPROACHING`（靠近）
   - 功率递减 → `DEPARTING`（远离）
   - 频率递增 → `APPROACHING`（多普勒蓝移）
   - 频率递减 → `DEPARTING`（多普勒红移）
5. 多通道投票决定最终结果

## 数据流

```
麦克风阵列
    ↓
/apollo/sensor/microphone (AudioData, 4通道 RAW PCM)
    ↓
AudioComponent.Proc()
    ↓
AudioInfo.Insert() → 信号缓存（滑动窗口，默认3秒）
    ↓
┌─────────────────────────────────────────────────────┐
│              MessageProcess.OnMicrophone()           │
│                                                      │
│  ┌─ DirectionDetection ─┐  ┌─ SirenDetection ──┐   │
│  │ GCC-PHAT 互相关       │  │ PyTorch JIT 模型   │   │
│  │ → 声源位置 + 角度     │  │ → 警报声 开/关     │   │
│  └───────────────────────┘  └───────────────────┘   │
│                                                      │
│  ┌─ MovingDetection ────┐                           │
│  │ FFT 频谱分析          │                           │
│  │ → 靠近/远离/未知      │                           │
│  └───────────────────────┘                           │
└─────────────────────────────────────────────────────┘
    ↓
AudioDetection (position, source_degree, is_siren, moving_result)
    ↓
/apollo/audio_detection → 规划模块 / Dreamview 告警
```

### 输入输出

| 方向 | Channel | 类型 | 说明 |
|------|---------|------|------|
| 输入 | `/apollo/sensor/microphone` | `AudioData` | 麦克风阵列原始数据（4 通道 16-bit PCM） |
| 输入 | `/apollo/localization/pose` | `LocalizationEstimate` | 车辆定位信息 |
| 输出 | `/apollo/audio_detection` | `AudioDetection` | 检测结果：声源位置、角度、警报状态、移动状态 |

## 配置方式

### Protobuf 配置（audio_conf.pb.txt）

```protobuf
topic_conf {
  audio_data_topic_name: "/apollo/sensor/microphone"
  audio_detection_topic_name: "/apollo/audio_detection"
  localization_topic_name: "/apollo/localization/pose"
  audio_event_topic_name: "/apollo/audio_event"
  perception_topic_name: "/apollo/perception/obstacles"
}
respeaker_extrinsics_path: "/apollo/modules/audio/conf/respeaker_extrinsics.yaml"
```

配置结构定义（`audio_conf.proto`）：

```protobuf
message AudioConf {
  optional TopicConf topic_conf = 1;           // 话题名称配置
  optional string respeaker_extrinsics_path = 2; // 麦克风外参文件路径
}
```

### 命令行参数（gflags）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--cache_signal_time` | `int32` | `3` | 信号缓存时长（秒） |
| `--torch_siren_detection_model` | `string` | `/apollo/modules/audio/data/torch_siren_detection_model.pt` | 警报声检测模型路径 |
| `--audio_records_dir` | `string` | `""` | 离线处理的录制文件目录 |
| `--audio_conf_file` | `string` | `/apollo/modules/audio/conf/audio_conf.pb.txt` | 配置文件路径 |

### 麦克风外参标定（respeaker_extrinsics.yaml）

定义麦克风阵列相对于 IMU（novatel）的外参变换，包含旋转四元数和平移向量：

```yaml
child_frame_id: microphone
transform:
  rotation:
    x: 0, y: 0, z: 0, w: 1
  translation:
    x: 0.0, y: 0.68, z: 0.72
```

### 启动方式

使用 mainboard 启动：

```bash
mainboard -d modules/audio/dag/audio.dag
```

使用 cyber_launch 启动：

```bash
cyber_launch start modules/audio/launch/audio.launch
```

### 离线处理工具

`audio_offline_processing` 工具可对已录制的 Cyber Record 文件进行离线音频检测，生成包含 `AudioDetection` 消息的新 Record 文件：

```bash
./audio_offline_processing --audio_records_dir=/path/to/records
```

### 依赖库

| 依赖 | 用途 |
|------|------|
| LibTorch (GPU) | 警报声深度学习推理 |
| Eigen | 坐标变换矩阵运算 |
| FFTW3 | 快速傅里叶变换（移动检测） |
| yaml-cpp | 外参文件解析 |
| OpenMP | 并行计算控制 |
