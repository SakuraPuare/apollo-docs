---
title: "音频推理"
---

# Audio Inference

> 源码路径：`modules/audio/inference/`

## 概述

音频推理模块，包含三个检测器：方向检测（声源定位）、移动检测（接近/远离判断）、警笛检测（紧急车辆识别）。

## 核心类

### DirectionDetection

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

**职责**：使用 GCC-PHAT 算法估计声源方向和位置，通过外参矩阵转换到车辆坐标系

### MovingDetection

```cpp
class MovingDetection {
 public:
  std::vector<std::complex<double>> fft1d(const std::vector<double>& signals);
  MovingResult Detect(const std::vector<std::vector<double>>& signals);
  MovingResult DetectSingleChannel(const std::size_t channel_index,
                                   const std::vector<double>& signal);
};
```

**职责**：通过 FFT 频谱分析判断声源是接近还是远离，基于功率变化和频率偏移

### SirenDetection

```cpp
class SirenDetection {
 public:
  bool Evaluate(const std::vector<std::vector<double>>& signals);
 private:
  void LoadModel();
  torch::jit::script::Module torch_model_;
};
```

**职责**：使用 PyTorch 模型推理判断音频中是否包含警笛声

## 调用关系

- **被调用方**：`MessageProcess::OnMicrophone()` 依次调用三个检测器
- **依赖**：LibTorch（PyTorch C++）、Eigen（矩阵运算）
