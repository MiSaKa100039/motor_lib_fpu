# J-Link TIM3 ABZ 诊断

本文件夹包含用于排查 TIM3 ABZ 启动问题的只读 J-Link Commander 辅助脚本。

辅助脚本默认使用 `STM32G474RC` 作为 J-Link 设备选择。在此配置下，
SEGGER 的 `STM32G474RB` Flash 加载器仅覆盖前 64KB，导致 ELF `.data`
加载镜像中 `0x08010000` 以上的内容被擦除。

安全注意事项：

- 在功率级可以主动驱动电机时，不要暂停 MCU。
- 在运行基于暂停的转储之前，请将驱动器 EN 拉低或移除母线电源。
- 在运行转储之前关闭 Ozone，因为两个工具使用同一个 J-Link。
- 该脚本不会烧录固件，也不会写入 RAM 或寄存器。

先进行空运行测试：

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\tim3_abz_dump.ps1
```

确认功率级安全后：

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\tim3_abz_dump.ps1 -Run
```

暂停、转储，然后继续运行：

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\tim3_abz_dump.ps1 -Run -Resume
```

烧录完整 ELF 而不启动目标：

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\flash_elf.ps1 -Run
```

如果 SEGGER 报告 Flash 内容已匹配，但 `0x08010000` 以上的地址仍读取为
`0xFFFFFFFF`，请先强制执行全片擦除：

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\flash_elf.ps1 -Run -Erase
```

仅在功率级安全且你有意让固件在烧录后运行时，才添加 `-Go` 参数。

`flash_elf.ps1` 将 ELF 转换为 HEX 和 BIN，然后将连续的 BIN 烧录到
`0x08000000`。这避免了 SEGGER `loadfile` 在此目标上忽略跨越
`0x08010000` 的 `.data` 加载镜像的问题。

脚本会在 `tools\jlink\out` 下创建带时间戳的文件：

- `*.symbols.txt`：用于转储的 ELF 符号地址。
- `*.jlink`：生成的 J-Link Commander 脚本。
- `*.stdout.txt`：J-Link 原始输出。

`BSP_ABZ_Debug_Stage` 取值说明：

```text
10   BSP_ABZ_Init 进入
20   ABZ 上下文已清除，状态已初始化
30   htim3 已绑定；准备在计数器读取使能后读取 TIM3->CNT
40   PrepareTimerIrq 进入
50   StartEncoderAB 即将运行
500  StartEncoderAB 进入
501  TIM 句柄非空
510  HAL_TIM_Encoder_Start(CH1) 即将运行
511  HAL_TIM_Encoder_Start(CH1) 返回
512  HAL_TIM_Encoder_Start(CH2) 即将运行
513  HAL_TIM_Encoder_Start(CH2) 返回
51   StartEncoderAB 返回到 BSP_ABZ_Init
70   A/B 编码器启动成功
90   BSP_ABZ_Init 完成
520  直接寄存器启动路径进入
530  直接寄存器路径即将使能 CC1/CC2
540  直接寄存器路径即将设置 CEN
541  直接寄存器启动完成
```

初步分析指南：

- 如果内核处于 `HardFault_Handler`，请检查转储中的 `CFSR/HFSR/BFAR/MMFAR`
  以及最后一个 `BSP_ABZ_Debug_Stage`。
- 如果在阶段 70/90 之后 `TIM3->CR1.CEN` 或 `TIM3->CCER.CC1E/CC2E`
  被清除，则编码器启动未能保持定时器运行。
- 如果 `GPIOC.MODER` 和 `GPIOC.AFRL/AFRH` 未显示 PC6/PC7/PC8 为 AF2
  模式，则引脚复用不是 TIM3 所期望的配置。
- 如果 `RCC_APB1ENR1.TIM3EN` 被清除，则 TIM3 未获得时钟。
- 如果在 `USER_INIT_STAGE` 从零改变之前 PC 位于 `Error_Handler` 内，
  请先检查 RCC/PWR/FLASH/SysTick 模块；此时 TIM3 尚未运行。
