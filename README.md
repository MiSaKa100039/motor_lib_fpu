# LCA039 GCC/CMake Template

这是一个面向 LCM32F039/LCA039 的独立 GCC 工程。目录和 CMake 使用习惯参考 STM32 工程，但 CPU 参数、启动代码、链接脚本、时钟和调试支持均针对 LCA039。

## 文件来源与可信边界

| 内容 | 来源 | 当前验证 |
|---|---|---|
| `Drivers/CMSIS`、`Drivers/LCM32F039_StdPeriph_Driver` | LCM32F039 SDK 2.0.4 | 从 SDK 原字节迁移，65 个文件迁移前后 SHA-256 一致 |
| `Drivers/.../Source/Templates/arm/lcm32f039_startup.s` | 厂商 ARMASM 启动文件 | 只作为参考，不参与 GCC 构建 |
| `Core/Startup/startup_lcm32f039_gcc.S` | 本工程编写的 GNU 汇编移植 | 中断向量逐项对照 ARMASM；ELF 检查向量地址、栈顶和 Thumb 入口 |
| `Linker/LCM32F039_FLASH.ld` | 本工程编写的 GNU linker script | 固定 64 KiB Flash、8 KiB RAM，并检查向量表、栈顶和 RAM 溢出 |
| `Core/Src/lca039_clock.c` 与生成的配置头 | 本工程实现 | PLL 参数在 CMake 阶段精确求解；运行时带超时、寄存器复核和 RCH 回退 |
| `Core/Src/syscalls.c` | 本工程的 newlib 适配 | 只提供裸机桩函数和受限 `_sbrk`，模板本身不使用动态分配 |
| `Debug/LCM32F039.svd`、Main Flash FLM | 厂商 Pack 0.4.72 | FLM 与 Pack 内文件 SHA-256 一致 |
| CMake、J-Link XML、PowerShell 和 Ozone 工程 | 本工程编写 | 构建矩阵、XML 解析及脚本临时目录安装/卸载已验证 |

GNU 启动文件、链接脚本和时钟代码不是厂商官方 GCC 支持。静态检查和一次成功构建不能替代实板的上电复位、供电边界、温度和长时间运行测试。

## 环境要求

- CMake 3.22 或更新版本
- Ninja
- GNU Arm Embedded Toolchain，`arm-none-eabi-gcc/g++/objcopy/readelf/objdump/size` 位于 PATH

当前验证环境为 GCC 14.3.1、CMake 3.28 和 Ninja 1.11。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

同样支持 `Release` 和 `RelWithDebInfo`。产物位于 `build/<Preset>/`：

- `lca039_template.elf`
- `lca039_template.hex`
- `lca039_template.bin`
- `lca039_template.map`
- `compile_commands.json`

CLion 可以直接读取 `CMakePresets.json`。

## 时钟配置

默认值在 `CMakePresets.json` 中：

```text
LCA039_CLOCK_SOURCE = RCH
LCA039_SYSCLK_HZ    = 72000000
LCA039_OSCH_HZ      = 16000000
LCA039_APB0_DIV     = 1
LCA039_APB1_DIV     = 1
LCA039_VDD_MV       = 3300
```

仓库 preset 使用 72 MHz，以满足 20 kHz 电机 PWM 的计数分辨率；不使用 PLL 时可显式改为 RCH 16 MHz。板级 PWM 周期会根据生成的 `LCA039_SYSCLK_HZ` 自动计算，不再固定假设 72 MHz。

内部 RCH 经 PLL 到 72 MHz：

```powershell
cmake --preset Debug -DLCA039_CLOCK_SOURCE=RCH -DLCA039_SYSCLK_HZ=72000000
cmake --build --preset Debug
```

外部 16 MHz 晶振经 PLL 到 72 MHz：

```powershell
cmake --preset Debug -DLCA039_CLOCK_SOURCE=OSCH -DLCA039_OSCH_HZ=16000000 -DLCA039_SYSCLK_HZ=72000000
cmake --build --preset Debug
```

CMake 会枚举 PLL 的 DM/DN/OD，只接受精确输出，并打印 SYSCLK、PCLK0、PCLK1、VCO、Flash wait state 和 ADC 最小同步分频。无法精确生成时配置失败，例如外部 4 MHz 不能生成 72 MHz，会提示最高可用的 64 MHz。

Flash wait state 自动按以下边界设置：

- SYSCLK < 32 MHz：0
- 32 MHz ≤ SYSCLK < 64 MHz：1
- SYSCLK ≥ 64 MHz：2

64 MHz 及以上会设置 200 µA LDO 驱动和 2.5 V LVR。72 MHz 仅允许在配置的 `LCA039_VDD_MV > 2800` 时生成。

生成的 `build/<Preset>/generated/lca039_clock_config.h` 是构建产物，不应手动修改。

## ADC 和外设时钟

ADC 工作时钟必须不高于 24 MHz。厂商 `ADC_StructInit()` 默认同步 `/1`，因此 PCLK1 为 72 MHz 时必须在调用 `ADC_Init()` 前改成至少 `/4`：

```c
ADC_InitTypeDef adc_init;

ADC_StructInit(&adc_init);
adc_init.ADC_ClkMode = ADC_ClockMode_SynClkDiv4;
ADC_Init(ADC, &adc_init);
```

生成头中的 `LCA039_ADC_SYNC_DIV_MIN` 给出当前 PCLK1 对应的最小同步分频。UART、I2C、SSP 和 WT 还可选择独立时钟源；不要直接套用 STM32 的定时器倍频规则。

## 启动与运行状态

GNU 启动代码完成 `.data` 复制、`.bss` 清零，然后调用厂商 `SystemInit()`、`__libc_init_array()` 和 `main()`。Cortex-M0 不配置 VTOR。

Ozone 可以观察以下变量：

- `g_boot_status`：`.data/.bss` 启动探针；0 表示通过。
- `g_clock_status`：时钟初始化状态；0 表示目标时钟已验证。
- `SystemCoreClock`：根据实际时钟寄存器计算的 HCLK。
- `g_heartbeat`：主循环运行标志。

OSCH、PLL 或时钟切换超时后固件会退回内部 RCH 16 MHz，并通过 `g_clock_status` 保留失败原因。

## 电机配置与两种固件

电机相关代码按功能域组织在 `Platform/Motor`、`BSP/Motor`、`Platform/Communication/VOFA` 和 `BSP/Communication/VOFA`。`Platform/Motor/BuildCfg` 定义编译能力，`MotorCfg` 保存实际硬件/电机参数，`TestCfg` 只控制目标测试。

默认 `hardwareBindingEnabled=false`，所有功率引脚和 ADC 通道均为无效占位；因此任何电机启动请求都会被拒绝并保持桥臂关闭。接入实际功率板前必须填写分流电阻、放大增益、分压、NTC、PWM/Break 引脚和 ADC 通道。

正常固件使用 Debug/Release/RelWithDebInfo preset。独立目标测试固件使用：

```powershell
cmake --preset TargetTest
cmake --build --preset TargetTest
```

主机测试使用：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\Host\Run-HostTests.ps1
```

详细配置、危险测试双重解锁和 Ozone 观察变量参见 [docs/MotorFramework.md](docs/MotorFramework.md)。

## 已知风险与实板验收

厂商驱动保持原字节，因此编译器仍会报告 SDK 内部的可疑缩进和未初始化局部变量警告。模板没有修改这些上游代码；使用对应 ADC、DMA、I2C、UART API 前应单独审查相关警告。

建议实板至少完成：

1. 连续上电/复位，确认 `g_boot_status` 和 `g_clock_status` 始终为 0。
2. 使用 MCO、定时器或外部仪器测量实际主频。
3. 验证所有已使用的中断向量。
4. 在最低/最高供电和目标温度范围内测试 OSCH/PLL。
5. 长时间运行并监控 heartbeat、HardFault、栈余量和外设错误。

J-Link/Ozone 下载和附加调试参见 [Debug/README.md](Debug/README.md)。

## 编码约束

厂商 SDK 文件保持原始编码和字节内容。新增源码与构建文件使用 UTF-8 无 BOM，源码注释使用英文。
