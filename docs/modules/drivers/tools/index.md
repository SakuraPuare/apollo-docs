---
title: "驱动工具"
---

# 驱动工具 (Drivers Tools)

> 源码路径: `modules/drivers/tools/`

## 概述

`drivers/tools` 模块提供驱动层的辅助工具组件。目前仅包含 **image_decompress** 子模块，负责将压缩图像消息解码为标准 RGB 图像并发布到 Cyber RT 通道，供下游感知模块消费。

## 核心类

### ImageDecompressComponent

继承自 `cyber::Component<apollo::drivers::CompressedImage>`，是一个 Cyber RT 标准组件，接收压缩图像输入、解码后输出原始图像。

```cpp
class ImageDecompressComponent final
    : public cyber::Component<apollo::drivers::CompressedImage> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::CompressedImage>& image) override;

 private:
  std::shared_ptr<cyber::Writer<apollo::drivers::Image>> writer_;
  Config config_;
};
```

- `writer_` — 用于向输出通道发布解码后的 `apollo::drivers::Image` 消息
- `config_` — 从配置文件加载的 `Config` 对象

## 核心函数

### Init

```cpp
bool ImageDecompressComponent::Init()
```

组件初始化入口：

1. 调用 `GetProtoConfig(&config_)` 加载 protobuf 配置
2. 根据配置中的 `channel_name` 创建 `cyber::Writer<Image>` 写入器

### Proc

```cpp
bool ImageDecompressComponent::Proc(
    const std::shared_ptr<apollo::drivers::CompressedImage>& compressed_image)
```

消息处理回调，核心解码流程：

1. 复制压缩图像的 `header` 和 `measurement_time` 到输出消息
2. 将压缩数据转为 `std::vector<uint8_t>`，通过 `cv::imdecode` 解码为 BGR `cv::Mat`
3. 调用 `cv::cvtColor` 将 BGR 转换为 RGB
4. 设置输出图像的 `width`、`height`、`encoding`（固定 `"rgb8"`）和 `step`（3 * width）
5. 将像素数据写入 Image 消息并通过 `writer_->Write` 发布

## 配置

配置通过 protobuf 定义，文件路径为 `image_decompress/proto/config.proto`：

```protobuf
message Config {
  optional string channel_name = 1;
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `channel_name` | string | 解码后原始图像的输出通道名称 |

## 调用关系

```text
CompressedImage (输入通道)
    │
    ▼
ImageDecompressComponent::Proc
    │
    ├── cv::imdecode      — JPEG/PNG 解码为 cv::Mat
    ├── cv::cvtColor       — BGR → RGB 转换
    │
    ▼
Image (输出通道, channel_name 由配置指定)
```

组件通过 Cyber RT 的 `Component` 框架自动订阅压缩图像通道，`Proc` 回调被触发后执行解码并发布到配置指定的输出通道。
