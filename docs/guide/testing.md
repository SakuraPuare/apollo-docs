---
title: Apollo 测试体系指南
description: 深入解析 Apollo 自动驾驶平台的测试架构，涵盖单元测试、集成测试、仿真测试的编写规范与最佳实践。
---

# Apollo 测试体系指南

## 概述

Apollo 采用 Google Test (gtest) 和 Google Mock (gmock) 作为 C++ 测试框架，配合 Bazel 构建系统管理测试目标。整个测试体系分为三个层次：

- **单元测试** — 针对独立函数和类的细粒度验证，使用 `TEST` 和 `TEST_F` 宏
- **集成测试** — 验证多个模块协同工作的正确性，通常基于 golden file 比对
- **仿真测试** — 通过 Dreamview 的 Sim Control 模块进行场景级回放与验证

测试文件统一以 `_test.cc` 为后缀，与被测源码放在同一目录下。测试数据存放在对应模块的 `testdata/` 目录中。

## 单元测试

### gtest 基本用法

最简单的测试使用 `TEST` 宏，适用于无需共享状态的纯函数测试。以 `modules/common/math/vec2d_test.cc` 为例：

```cpp
#include "modules/common/math/vec2d.h"

#include <cmath>
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(Vec2dTest, NomralCases) {
  Vec2d pt(2, 3);
  EXPECT_NEAR(pt.Length(), std::sqrt(13.0), 1e-5);
  EXPECT_NEAR(pt.LengthSquare(), 13.0, 1e-5);
  EXPECT_NEAR(pt.DistanceTo({0, 0}), std::sqrt(13.0), 1e-5);
  // ... 省略 DistanceSquareTo 等断言 ...
  EXPECT_NEAR(pt.Angle(), std::atan2(3, 2), 1e-5);
  EXPECT_NEAR(pt.CrossProd({4, 5}), -2, 1e-5);
  EXPECT_NEAR(pt.InnerProd({4, 5}), 23, 1e-5);
  EXPECT_EQ(pt.DebugString(), "vec2d ( x = 2  y = 3 )");
}

TEST(Vec2dTest, rotate) {
  Vec2d pt(4, 0);
  auto p1 = pt.rotate(M_PI / 2.0);
  EXPECT_NEAR(p1.x(), 0.0, 1e-5);
  EXPECT_NEAR(p1.y(), 4.0, 1e-5);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
```

关键断言宏：

| 宏 | 用途 |
|---|---|
| `EXPECT_EQ(a, b)` | 精确相等 |
| `EXPECT_NEAR(a, b, tol)` | 浮点近似相等（自动驾驶中最常用） |
| `EXPECT_TRUE(expr)` | 布尔断言 |
| `EXPECT_FLOAT_EQ(a, b)` | 单精度浮点相等（4 ULP 容差） |
| `EXPECT_DOUBLE_EQ(a, b)` | 双精度浮点相等（4 ULP 容差） |

### TEST_F 与测试 Fixture

当多个测试用例需要共享初始化逻辑或成员变量时，使用 `TEST_F` 配合测试 Fixture 类。Fixture 类继承 `::testing::Test`，在 `SetUp()` 中完成初始化，`TearDown()` 中完成清理。

以 `modules/control/control_component/controller_task_base/common/pid_controller_test.cc` 为例：

```cpp
#include "modules/control/control_component/controller_task_base/common/pid_controller.h"

#include "gtest/gtest.h"
#include "modules/control/control_component/proto/pid_conf.pb.h"
#include "cyber/common/file.h"

namespace apollo {
namespace control {

class PidControllerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string controllers_dir =
        "/apollo/modules/control/control_component/testdata/conf/";
    std::string station_pid_conf_file =
        controllers_dir + "station_pid_conf.pb.txt";
    std::string speed_pid_conf_file =
        controllers_dir + "speed_pid_conf.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(
        station_pid_conf_file, &station_pid_conf_));
    ACHECK(cyber::common::GetProtoFromFile(
        speed_pid_conf_file, &speed_pid_conf_));
  }

 protected:
  PidConf station_pid_conf_;
  PidConf speed_pid_conf_;
};

TEST_F(PidControllerTest, StationPidController) {
  PidConf pid_conf = station_pid_conf_;
  PIDController pid_controller;
  pid_controller.Init(pid_conf);
  pid_controller.Reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(0.1, dt), 0.01, 1e-6);
}

}  // namespace control
}  // namespace apollo
```

Fixture 模式的典型应用场景：

- 从 protobuf 文本文件加载配置（如 PID 参数、控制管线配置）
- 初始化 Kalman Filter 等有状态对象的矩阵参数
- 构建 CyberRT 节点和 Reader/Writer 通道

### 高级 Fixture：继承被测类

对于需要访问被测类 protected/private 成员的场景，Apollo 采用多重继承模式。以横向控制器测试为例：

```cpp
class LatControllerTest : public ::testing::Test, LatController {
 public:
  virtual void SetUp() {
    FLAGS_v = 3;
    timestamp_ = Clock::NowInSeconds();
    injector_ = std::make_shared<DependencyInjector>();
  }

  // 暴露 protected 方法供测试调用
  void ComputeLateralErrors(
      const double x, const double y, const double theta,
      const double linear_v, const double angular_v,
      const double linear_a,
      const TrajectoryAnalyzer &trajectory_analyzer,
      SimpleLateralDebug *debug,
      const canbus::Chassis *chassis) {
    LatController::ComputeLateralErrors(
        x, y, theta, linear_v, angular_v, linear_a,
        trajectory_analyzer, debug, chassis);
  }

 protected:
  // 从 protobuf 文件加载测试数据的辅助方法
  LocalizationPb LoadLocalizaionPb(const std::string &filename) {
    LocalizationPb localization_pb;
    ACHECK(cyber::common::GetProtoFromFile(filename, &localization_pb));
    localization_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return localization_pb;
  }

  double timestamp_ = 0.0;
};
```

配合 BUILD 文件中的 `-fno-access-control` 编译选项，可以绕过 C++ 访问控制：

```python
apollo_cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = ["lat_controller_test.cc"],
    copts = ["-fno-access-control"],
    ...
)
```

### Mock 的使用

Apollo 在感知、规划等模块中使用 Mock 函数构造测试输入数据。以雷达检测模块为例，通过 Mock 函数构造传感器原始观测数据：

```cpp
#include "gtest/gtest.h"

// 构造模拟的毫米波雷达观测数据
ContiRadar MockContiObs() {
  ContiRadar raw_obs;
  drivers::ContiRadarObs conti_obs;
  conti_obs.set_clusterortrack(0);
  conti_obs.set_obstacle_id(80);
  conti_obs.set_longitude_dist(20);
  conti_obs.set_lateral_dist(10);
  conti_obs.set_longitude_vel(10);
  conti_obs.set_lateral_vel(5);
  conti_obs.set_rcs(15);
  conti_obs.set_dynprop(0);
  conti_obs.set_probexist(0.8);
  raw_obs.add_contiobs()->CopyFrom(conti_obs);
  return raw_obs;
}

TEST_F(DummyAlgorithmsTest, dummy_test) {
  ContiRadar raw_obs = MockContiObs();
  ContiRadar corrected_obs;

  PreprocessorOptions preprocessor_options;
  preprocessor.Preprocess(raw_obs, preprocessor_options, &corrected_obs);
  EXPECT_EQ(corrected_obs.contiobs_size(), 6);
  EXPECT_EQ(corrected_obs.contiobs(0).obstacle_id(), 80);
}
```

Apollo 中 Mock 的常见模式：

- **Mock 函数** — 构造 protobuf 消息作为模块输入（如上例）
- **Mock 对象** — 在 Fixture 中直接实例化被测类的 Dummy 实现
- **protobuf 文件加载** — 从 `testdata/` 目录读取预录制的真实数据（最常用）

## BUILD 文件中的测试规则

### apollo_cc_test 宏

Apollo 定义了 `apollo_cc_test` 宏来封装 Bazel 原生的 `cc_test` 规则，自动处理动态依赖填充。宏定义位于 `tools/apollo_package.bzl`：

```python
def apollo_cc_test(**kwargs):
    # simple wrap for cc_test
    CC_TEST(**(dynamic_fill_deps(kwargs)))
```

其中 `CC_TEST` 根据构建模式选择原生 `cc_test` 或 `@rules_cc` 的 `cc_test`，`dynamic_fill_deps` 自动补全依赖关系。

### 基本用法

以 `modules/common/math/BUILD` 为例，一个典型的测试规则：

```python
load("//tools:apollo_package.bzl", "apollo_cc_test")

apollo_cc_test(
    name = "vec2d_test",
    size = "small",
    srcs = ["vec2d_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)
```

常用属性说明：

| 属性 | 说明 |
|---|---|
| `name` | 测试目标名称，通常为 `<模块>_test` |
| `size` | 测试规模：`small`（默认，60s 超时）、`medium`（300s）、`large`（900s） |
| `srcs` | 测试源文件列表 |
| `deps` | 依赖项，必须包含 `@com_google_googletest//:gtest_main` 或 `gtest` |
| `data` | 测试数据文件或 filegroup |
| `copts` | 额外编译选项，如 `-fno-access-control` |
| `linkstatic` | 是否静态链接，集成测试通常设为 `True` |
| `linkopts` | 链接选项，如 `-lm` |

### 测试数据的引用

通过 `filegroup` 规则将 `testdata/` 目录打包，然后在测试规则中通过 `data` 属性引用：

```python
# 定义测试数据 filegroup
filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/**",
    ]),
)

# 在测试中引用
apollo_cc_test(
    name = "control_component_test",
    size = "small",
    srcs = ["control_component_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    linkstatic = True,
    deps = [
        ":DO_NOT_IMPORT_control_component",
        "@com_google_googletest//:gtest_main",
    ],
)
```

对于控制器插件测试，也可以直接引用本地目录的 filegroup：

```python
filegroup(
    name = "lateral_controller_test",
    srcs = glob([
        "lateral_controller_test/**",
    ]) + glob(["conf/*"]),
)

apollo_cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = ["lat_controller_test.cc"],
    data = ["lateral_controller_test"],
    deps = [":lat_controller_lib", ...],
)
```

## 测试数据管理

### testdata/ 目录组织

Apollo 的测试数据遵循统一的目录结构：

```
modules/control/control_component/testdata/
├── conf/                          # 配置文件
│   ├── control_conf.pb.txt        # 控制配置
│   ├── pipeline.pb.txt            # 控制管线配置
│   ├── speed_pid_conf.pb.txt      # PID 参数
│   ├── station_pid_conf.pb.txt
│   └── plugins/                   # 插件配置
│       ├── lat_based_lqr_controller/
│       │   ├── conf/controller_conf.pb.txt
│       │   └── plugins.xml
│       └── lon_based_pid_controller/
│           ├── conf/controller_conf.pb.txt
│           └── plugins.xml
├── control_tester/                # 组件级测试输入
│   ├── chassis.pb.txt
│   ├── localization.pb.txt
│   ├── pad_msg.pb.txt
│   └── planning.pb.txt
├── simple_control_test/           # 场景测试数据
│   ├── 1_chassis.pb.txt           # 输入数据（编号前缀）
│   ├── 1_localization.pb.txt
│   ├── 1_planning.pb.txt
│   ├── 1_pad.pb.txt
│   └── result_simple_test_0.pb.txt  # golden file（result_ 前缀）
└── relative_position_test/
    ├── 0_apollo_canbus_chassis.pb.txt
    └── result_simple_left_0.pb.txt
```

命名约定：

- 输入文件：`<编号>_<数据类型>.pb.txt`，如 `1_chassis.pb.txt`
- 结果文件：`result_<测试名>_<编号>.pb.txt`，如 `result_simple_test_0.pb.txt`
- 配置文件：`<配置名>.pb.txt`，如 `speed_pid_conf.pb.txt`

### Protobuf 测试数据

Apollo 使用 protobuf 文本格式（`.pb.txt`）存储测试数据，通过 `cyber::common::GetProtoFromFile` 加载。例如一个底盘数据文件 `chassis.pb.txt`：

```protobuf
engine_started: true
speed_mps: 0
throttle_percentage: 15.04387
brake_percentage: 22.879377
steering_percentage: 1.0212766
driving_mode: COMPLETE_MANUAL
error_code: NO_ERROR
gear_location: GEAR_NEUTRAL
header {
  timestamp_sec: 1494373003.7010145
  module_name: "chassis"
  sequence_num: 718141
}
```

PID 配置文件 `speed_pid_conf.pb.txt`：

```protobuf
integrator_enable: true
integrator_saturation_level: 0.3
output_saturation_level: 3.0
kp: 1.5
ki: 0.5
kd: 0.0
kaw: 1.0
```

加载方式：

```cpp
PidConf speed_pid_conf;
ACHECK(cyber::common::GetProtoFromFile(
    "/apollo/modules/control/control_component/testdata/conf/speed_pid_conf.pb.txt",
    &speed_pid_conf));
```

### Golden File 测试模式

Golden file 测试是 Apollo 集成测试的核心模式。其工作流程：

1. 加载预录制的输入数据（定位、底盘、规划轨迹等）
2. 运行被测模块处理流程
3. 将输出与预存的 golden file（`result_*.pb.txt`）进行 protobuf 级别的比对

```cpp
// control_test_base.cc 中的核心比对逻辑
bool ControlTestBase::test_control(
    const std::string &test_case_name, int case_num) {
  // ... 加载输入数据并执行控制逻辑 ...

  // 加载 golden file
  std::string golden_result_file = "result_" + test_case_name + "_"
      + std::to_string(case_num) + ".pb.txt";
  std::string full_golden_path =
      FLAGS_test_data_dir + golden_result_file;

  ControlCommand golden_result;
  bool load_success = cyber::common::GetProtoFromASCIIFile(
      full_golden_path, &golden_result);

  if (FLAGS_test_update_golden_log) {
    // 更新模式：将当前结果写入 golden file
    cyber::common::SetProtoToASCIIFile(control_command_, tmp_golden_path);
    return false;
  }

  // 比对模式：验证输出与 golden file 一致
  bool same_result =
      common::util::IsProtoEqual(golden_result, control_command_);
  return same_result;
}
```

使用 `RUN_GOLDEN_TEST` 宏简化调用：

```cpp
#define RUN_GOLDEN_TEST                                            \
  {                                                                \
    const ::testing::TestInfo *const test_info =                   \
        ::testing::UnitTest::GetInstance()->current_test_info();   \
    bool run_control_success = test_control(test_info->name(), 0); \
    EXPECT_TRUE(run_control_success);                              \
  }

TEST_F(SimpleControlTest, simple_test) {
  FLAGS_enable_csv_debug = true;
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "1_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  ControlTestBase::SetUp();
  RUN_GOLDEN_TEST;
}
```

更新 golden file 时，设置 `FLAGS_test_update_golden_log = true` 即可将当前输出写入结果文件。

## 集成测试与仿真测试

### 集成测试

Apollo 的集成测试位于各模块的 `integration_tests/` 目录下，通过 `ControlTestBase` 等基类封装完整的模块初始化和数据流转逻辑。

以控制模块集成测试为例，`ControlTestBase` 完成以下工作：

1. 加载控制管线配置（`pipeline.pb.txt`）
2. 初始化依赖注入器和控制任务代理
3. 加载控制器插件（横向 LQR、纵向 PID）
4. 通过 gflags 注入测试数据文件路径
5. 执行控制逻辑并与 golden file 比对

```cpp
// control_test_base.h
class ControlTestBase : public ::testing::Test {
 public:
  static void SetUpTestCase();
  virtual void SetUp();
  bool test_control();
  bool test_control(const std::string &test_case_name, int case_num);
  void LoadControllerPlugin();

 private:
  void trim_control_command(ControlCommand *origin);
  ControlCommand control_command_;
  ControlComponent control_;
  static uint32_t s_seq_num_;
};
```

具体测试类继承 `ControlTestBase`，只需设置数据目录和文件名：

```cpp
class SimpleControlTest : public ControlTestBase {
 public:
  virtual void SetUp() {
    FLAGS_test_data_dir =
        "/apollo/modules/control/control_component/testdata/"
        "simple_control_test/";
  }
};

TEST_F(SimpleControlTest, simple_test) {
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "1_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  ControlTestBase::SetUp();
  RUN_GOLDEN_TEST;
}
```

### 组件级测试

`ControlComponentTest` 展示了更完整的组件级测试模式，通过 CyberRT 的 Reader/Writer 机制模拟真实的消息通信：

```cpp
class ControlComponentTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_pipeline_file =
        "/apollo/modules/control/control_component/testdata/conf/"
        "pipeline.pb.txt";
    FLAGS_is_control_test_mode = true;
    FLAGS_is_control_ut_test_mode = true;
    SetupCyber();
  }

  virtual void TearDown() {
    if (control_component_) {
      control_component_->Shutdown();
    }
  }

 protected:
  std::shared_ptr<Writer<Chassis>> chassis_writer_;
  std::shared_ptr<Writer<LocalizationEstimate>> localization_writer_;
  std::shared_ptr<Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<Writer<PadMessage>> pad_writer_;
  // ...
};
```

### 规划模块集成测试

规划模块的集成测试基于 `PlanningTestBase`（位于 `modules/planning/planning_component/integration_tests/planning_test_base.h`），支持多地图多场景测试。以 Sunnyvale Loop 场景为例：

```cpp
class SunnyvaleLoopTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    FLAGS_use_navigation_mode = false;
    FLAGS_map_dir = "modules/map/data/sunnyvale_loop";
    FLAGS_test_base_map_filename = "base_map_test.bin";
    FLAGS_test_data_dir =
        "modules/planning/planning_base/testdata/sunnyvale_loop_test";
    FLAGS_planning_upper_speed_limit = 12.5;

    ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  }
};

TEST_F(SunnyvaleLoopTest, cruise) {
  std::string seq_num = "1";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}
```

规划集成测试的特点：

- 通过 `FLAGS_map_dir` 指定地图数据，支持不同场景（`sunnyvale_loop`、`garage` 等）
- `ENABLE_RULE` 宏控制交通规则的启用/禁用
- `RUN_GOLDEN_TEST(sub_case_num)` 支持子用例编号
- `RUN_GOLDEN_TEST_DECISION` 变体仅比对决策结果，忽略轨迹点

### Mock 对象模式

Apollo 中 Mock 对象主要用于模拟协议层和插件接口。以 CAN 总线消息管理器测试为例（`modules/drivers/canbus/can_comm/message_manager_test.cc`）：

```cpp
class MockProtocolData
    : public ProtocolData<::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID = 0x111;
  MockProtocolData() {}
};

class MockMessageManager
    : public MessageManager<::apollo::canbus::ChassisDetail> {
 public:
  MockMessageManager() {
    AddRecvProtocolData<MockProtocolData, true>();
    AddSendProtocolData<MockProtocolData, true>();
  }
};

TEST(MessageManagerTest, GetMutableProtocolDataById) {
  uint8_t mock_data = 1;
  MockMessageManager manager;
  manager.Parse(MockProtocolData::ID, &mock_data, 8);
  manager.ResetSendMessages();
  EXPECT_NE(manager.GetMutableProtocolDataById(MockProtocolData::ID),
            nullptr);
}
```

### 仿真测试

Apollo 通过 Dreamview Plus 平台提供场景级仿真测试能力。Sim Control 模块可以：

- 回放预录制的驾驶场景数据
- 模拟车辆动力学响应
- 注入虚拟障碍物和交通参与者

仿真测试通常不在 Bazel 单元测试框架内运行，而是通过 Dreamview 的 Web 界面或命令行工具进行。规划模块的 `testdata/` 目录中包含了完整的场景数据（如 `sunnyvale_loop_test/`），可用于离线仿真验证。

## 运行测试的命令

### 运行单个测试

```bash
# 运行指定测试目标
bazel test //modules/common/math:vec2d_test

# 运行并查看详细输出
bazel test //modules/common/math:vec2d_test --test_output=all

# 运行指定测试用例
bazel test //modules/common/math:vec2d_test \
    --test_filter=Vec2dTest.rotate
```

### 运行模块级测试

```bash
# 运行 math 模块下所有测试
bazel test //modules/common/math/...

# 运行控制模块所有测试
bazel test //modules/control/...

# 运行所有测试（耗时较长）
bazel test //modules/...
```

### 常用选项

```bash
# 设置测试超时
bazel test //modules/control/... --test_timeout=120

# 并行运行测试
bazel test //modules/common/math/... --jobs=8

# 只运行 small 规模的测试
bazel test //modules/... --test_size_filters=small

# 显示测试日志
bazel test //modules/common/math:vec2d_test --test_output=streamed

# 更新 golden file（通过 test_arg 传递 flag）
bazel test //modules/control/control_component/controller_task_base/integration_tests:simple_control_test \
    --test_arg=--test_update_golden_log=true
```

## CI 脚本与测试覆盖率

Apollo 提供了一套完整的 Shell 脚本用于自动化测试和 CI 流程，位于 `scripts/` 目录下。

### 测试脚本

| 脚本 | 用途 | 典型用法 |
|------|------|----------|
| `scripts/apollo_test.sh` | 运行 Bazel 测试 | `bash scripts/apollo_test.sh` |
| `scripts/apollo_coverage.sh` | 运行测试并生成覆盖率报告 | `bash scripts/apollo_coverage.sh` |
| `scripts/apollo_lint.sh` | 代码风格检查（C++/Python/Shell） | `bash scripts/apollo_lint.sh --cpp` |
| `scripts/apollo_ci.sh` | CI 流水线入口 | `bash scripts/apollo_ci.sh` |

#### apollo_test.sh

封装了 `bazel test` 命令，自动检测 CPU/GPU 环境并设置编译选项：

```bash
# 运行全量测试
bash scripts/apollo_test.sh

# 运行指定模块测试
bash scripts/apollo_test.sh //modules/control/...
```

#### apollo_coverage.sh

运行 `bazel coverage` 并通过 `genhtml` 生成 HTML 覆盖率报告：

```bash
bash scripts/apollo_coverage.sh //modules/common/math/...
# 报告输出到 .cache/coverage/ 目录
```

#### apollo_ci.sh

CI 流水线按顺序执行 lint、build、test 三个阶段：

```bash
# 运行完整 CI
bash scripts/apollo_ci.sh

# 只运行测试阶段
bash scripts/apollo_ci.sh test

# 只运行 lint 阶段
bash scripts/apollo_ci.sh lint
```

CI 测试阶段使用 `--config=unit_test` 配置，确保只运行单元测试级别的用例。

#### apollo_lint.sh

支持三种语言的代码风格检查：

- C++：通过 `bazel test --config=cpplint` 运行 cpplint，BUILD 文件中需包含 `cpplint()` 规则
- Python：使用 `flake8` 检查
- Shell：使用 `shellcheck` 检查

```bash
bash scripts/apollo_lint.sh --cpp    # 仅 C++
bash scripts/apollo_lint.sh --py     # 仅 Python
bash scripts/apollo_lint.sh --all    # 全部
```

## 最佳实践

1. **测试文件命名** — 测试文件与被测文件同目录，命名为 `<被测文件>_test.cc`。例如 `pid_controller.cc` 对应 `pid_controller_test.cc`。

2. **命名空间一致** — 测试代码应放在与被测代码相同的命名空间中，便于访问内部类型。

3. **浮点比较用 EXPECT_NEAR** — 自动驾驶涉及大量浮点运算，始终使用 `EXPECT_NEAR` 并指定合理的容差（通常 `1e-5` 到 `1e-6`）。

4. **测试数据使用 protobuf 文本格式** — 便于人工审查和版本控制。通过 `cyber::common::GetProtoFromFile` 加载，`cyber::common::SetProtoToASCIIFile` 写入。

5. **善用 gflags 控制测试行为** — 通过 `DEFINE_string` / `DEFINE_bool` 定义测试专用 flag，在测试中灵活切换数据文件和运行模式。

6. **Golden file 测试保持可更新** — 提供 `FLAGS_test_update_golden_log` 机制，当算法合理变更时可以方便地更新基准结果。

7. **测试规模标注** — 在 BUILD 文件中正确设置 `size` 属性：纯计算测试用 `small`，涉及文件 I/O 的用 `medium`，需要 CyberRT 通信的用 `large`。

8. **集成测试复用基类** — 通过 `ControlTestBase` 等基类封装模块初始化逻辑，具体测试类只需关注数据配置和断言。

9. **清理时间戳等不稳定字段** — 在比对 golden file 前，清除 header 中的时间戳等运行时变化的字段：

```cpp
void TrimControlCommand(ControlCommand* origin) {
  origin->mutable_header()->clear_radar_timestamp();
  origin->mutable_header()->clear_lidar_timestamp();
  origin->mutable_header()->clear_timestamp_sec();
  origin->mutable_header()->clear_camera_timestamp();
}
```

10. **插件测试需要配置加载路径** — 测试控制器插件时，需要通过 `PluginManager` 显式加载插件 XML 配置文件，并设置 `APOLLO_PLUGIN_LIB_PATH` 环境变量指向编译产物目录。
