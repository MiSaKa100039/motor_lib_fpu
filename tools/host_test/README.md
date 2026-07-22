# tools/host_test/ -- Host 单元测试基础设施

本目录当前已建立最小 HostTest 骨架: `CMakeLists.txt`、`MotorTestRunner.h` 和
`test_main.cpp` 已可作为后续用例入口。受当前 Windows 环境缺少 PC ABI C++ 编译器限制,
本机尚不能生成可执行的 host test。

## 为什么需要 host test

Phase 2 已新增大量纯逻辑代码, 均依赖当时手写推演 + VOFA 真硬件验证:

- `MotorAPI::writeSetpoint` 字段落地 (TORQUE/VELOCITY/POSITION/IMPEDANCE)
- `computeIqReference` 位置环分支 (pos_err → pid_position + vel_ff → speed_ref → pid_speed)
- `Motor_StreamFSM` ACTIVE → HELD 超时转换 / SUSPENDED_BY_FAULT / SUSPENDED_BY_USER
- `MotorCommandGuard::validateModeSelection` POSITION/IMPEDANCE 入口门控
- `MotorFeatureRegistry::supportsMode` BuildCfg 各能力开关裁切

以上均存在 `// TODO HostTest` 标签。真硬件一次跑只能验证一条产品形态, 且 VOFA
看不到 FSM 内部状态切换细节, 极易漏 bug (Phase 2.5 已发现 `MotorAPI::start()`
漏 POSITION 入口分派, 即编译过但跑不起来的典型)。

## 引入 host test 的前置条件

###一、需要一台 host 编译器 (PC 可执行)

当前已检测: 项目仅有 `D:\SoftWare\STM32CubeCLT_1.21.0\st-arm-clang\bin\starm-clang++`
(ARM Cortex-M ABI, 编出的 `.elf` 不能在 Windows 上跑) 和 `cmake` / `ninja`。

**必须在 PC 上安装一个真正的 host ABI 编译器, 三选一:**

| 选项 | 安装命令 | 大小 | 备注 |
|---|---|---|---|
| LLVM MinGW (UCRT) | `winget install MartinStorsjo.LLVM-MinGW.UCRT` 或 GitHub release | ~150MB | 推荐, 与 CLion 兼容好, 与现有工具链不冲突 |
| LLVM 官方 clang | `winget install LLVM.LLVM` | ~500MB | 输出 exe 在 Windows 原生跑, 工具链现代化 |
| MSVC build tools | `winget install Microsoft.VisualStudio.2022.BuildTools` (含 C++ 工作负载) | 数 GB | 大头, 集成 VS / CLion 都行 |

安装后, 在 CLion 巋试置添加一个 toolchain 选该编译器即可。

###二、决定测试 framework

Phase 2.5 Plan 已决策: **自写极简 runner (~100 行 C++)**, 不引第三方依赖。
当前/目标路径结构:
```
tools/host_test/
├── CMakeLists.txt              # 已建: host runner 入口
├── MotorTestRunner.h           # 已建: TEST / ASSERT_TRUE / ASSERT_EQ / ASSERT_NEAR / run_all_tests
├── test_main.cpp               # 已建: main 跑所有 TEST_xxx
├── MotorTestAccess.h           # 待建: MOTOR_LIB_HOST_TEST 宏开关下加 ctx_ 等只读 getter
├── Motor_HAL_Mock.{cpp,h}     # 待建: MotorHAL_t 16 个指针全 no-op 实现
├── MockSensor.{cpp,h}          # 待建: 内含 mechanical_angle / speed_rpm 的可控 sensor
├── test_setpoint.cpp           # writeSetpoint 各 mode 字段落地
├── test_position_loop.cpp      # 位置环方向校正
├── test_stream_fsm.cpp         # HELD 状态转换
├── test_command_guard.cpp      # POSITION 入口门控
├── test_feature_registry.cpp   # supportsMode BuildCfg
├── test_position_start.cpp     # MotorAPI::start() POSITION 入口回归
└── test_fifo_playback.cpp      # append/trigger/abort/FIFO 状态
```

###三、CMake HostTest preset

`CMakePresets.json` 追加一个 HostTest preset (binaryDir=`build/HostTest`, toolchain=host g++),
与现有 Debug preset 互不干扰。

## 各 TODO HostTest 位置一览 (代码内 5 处)

| 文件 | 测试关注点 | BuildCfg 闸控 |
|---|---|---|
| `Lib/Motor/Core/FSM/StreamFSM/Motor_StreamFSM.cpp` | ACTIVE→HELD 超时 + fault/stop 冻结 | HOLD 开启时才编 |
| `Lib/Motor/Core/Runtime/Motor_ManagerRuntime.cpp` 位置环分支 | pos_err → iq_ref 方向校正 + vel_ff 叠加 | POSITION + VEL_FF 开启时才编 |
| `Lib/Motor/Core/Safety/Motor_CommandGuard.cpp` POSITION 入口 | HIGH_RES_ENCODER 通过 / HALL 拒绝 | POSITION 或 IMPEDANCE 开启时才编 |
| `Lib/Motor/Core/Manager/Motor_ManagerAPI.cpp` writeSetpoint | 各 mode 字段落地正确性 | — |
| `Lib/Motor/Core/Feature/Motor_FeatureRegistry.cpp` IMPEDANCE | BuildCfg 开关与 supportsMode 一致性 | IMPEDANCE 开启时才编 |

## Phase 2.6 引入 host test 的执行清单

1. [ ] 安装 LLVM MinGW 或 LLVM clang (winget 命令见上)
2. [ ] CLion 新增 toolchain 配置, 编译器选 host g++/clang++
3. [x] 创建 `tools/host_test/CMakeLists.txt`: 当前先接最小 runner, 后续再接 lib/mocks
4. [x] 写 `MotorTestRunner.h` (~100 行, `TEST(name)`, `ASSERT_EQ/assertN`, registration table, run_all_tests)
5. [ ] 在 `Motor_Manager.h` 加 `#ifdef MOTOR_LIB_HOST_TEST` 包的 `testCtx()` 等 getter
6. [ ] 写 `Motor_HAL_Mock.{cpp,h}`: 16 个函数指针 no-op 填表
7. [ ] 依次编 8 个 test_*.cpp 覆盖 TODO HostTest 各点
8. [ ] 跑通 `./build/HostTest/motor_lib_test.exe`, 0 failure
9. [ ] 在 `AGENTS.md` 追加一条: "修改 lib 任一行后必须跑 HostTest preset 通过"
10. [ ] CI 接入 (可选, 后续)

## 真硬件验证清单 (Phase 2 仍需真电机跑)

host test 仅覆盖纯逻辑层, 以下必须真硬件:

- [ ] 接 AS5600 编码器, 位置环单位阶跃 (pos_ref=0→π) 收敛无超调
- [ ] 速度前馈 vel_ff 给 1000rpm 时, 电机跟随无滞后
- [ ] 堵转保护在位置环下也能正确触发并恢复
- [ ] writeSetpoint 断流后 (模拟断链), BuildCfg HOLD 开启时电机停在原位不动 (HELD 测试)
- [ ] 双电机单 MCU (MAX_INSTANCES=2) 或多 MCU 分布式下, 跑 DualAxisSinDemo, VOFA 应看到 XY 画圆

## 当前状态

- 最小 runner 骨架已建立, 但真实 Motor 用例、HAL mock、测试访问器仍待补。
- `tools/build_motor_matrix.ps1` 已提供构建矩阵检查, 覆盖 FULL/CORE source set 与关键 BuildCfg 组合。
- 当前环境缺少 PC ABI C++ 编译器, `cmake -S tools/host_test -B build/codex-host-test-skeleton -G Ninja`
  会在配置阶段失败; 安装 LLVM MinGW、LLVM clang 或 MSVC Build Tools 后即可继续。
