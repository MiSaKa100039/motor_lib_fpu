# BSP/Motor — 电机驱动硬件层配置指南

## 硬件资源映射

| 硬件 | 通道 | 用途 |
|------|------|------|
| TIM1 CH1/CH1N | — | U 相 H 桥驱动 (高侧 + 低侧互补) |
| TIM1 CH2/CH2N | — | V 相 H 桥驱动 |
| TIM1 CH3/CH3N | — | W 相 H 桥驱动 |
| TIM1 CH4 | — | ADC 触发信号 (PWM 中心点触发) |
| ADC1 Inj Rank1 | CH9 (PC3) | I_U (U 相霍尔传感器) |
| ADC1 Inj Rank2 | CH9 (PC3) | I_Bus (母线电流, 双采样) |
| ADC2 Inj Rank1 | CH1 (PA0) | I_V (V 相霍尔传感器) |
| ADC2 Inj Rank2 | CH1 (PA0) | 双采样 (I_W 由 KCL 计算: Iw = -(Iu+Iv)) |
| ADC2 Reg Rank1 | CH17 (PA4) | V_BUS (母线电压, DMA 循环) |
| ADC2 Reg Rank2 | CH11 (PC5) | NTC1 (温度, DMA 循环) |
| ADC2 Reg Rank3..5 | — | NTC2/3/4 (如需, 在 CubeMX 中新增通道) |
| DMA1 Channel1 | — | ADC2 循环 DMA |
| PB9 | GPIO Out | 调试 IO (示波器测 ISR 频率) |

### NTC 配置说明

| NTC 索引 | ADC 通道 | DMA 缓冲区 | BSP 读取方式 |
|----------|----------|-----------|-------------|
| 0 (NTC1) | CH11 (PC5) | `adc_regular_buffer[1]` | `read_temp_raw(0)` |
| 1 (NTC2) | 需新增 | `adc_regular_buffer[2]` | `read_temp_raw(1)` |
| 2 (NTC3) | 需新增 | `adc_regular_buffer[3]` | `read_temp_raw(2)` |
| 3 (NTC4) | 需新增 | `adc_regular_buffer[4]` | `read_temp_raw(3)` |

**新增 NTC 步骤:**
1. CubeMX → ADC2 常规组 → 增加通道 (Rank 3/4/5)
2. `main.c` `MX_ADC2_Init()`: 修改 `hadc2.Init.NbrOfConversion` 为实际通道数
3. `stm32g4xx_hal_msp.c`: 配置对应 GPIO 为 `GPIO_MODE_ANALOG`
4. `Platform_Motor.cpp`: 设置 `ntc_count` + 对应 `ntc[index]` 参数

### 电流采样: 双霍尔传感器配置

- **U 相**: PC3 → ADC1_IN9 → 注入组 Rank1
- **V 相**: PA0 → ADC2_IN1 → 注入组 Rank1
- **W 相**: 由 KCL 计算 `Iw = -(Iu + Iv)`, 无需 ADC 通道

霍尔传感器 (C6906) 链路:
```
Iu → 霍尔 U → Vout = VREF + Iu × Sensitivity → ADC1_IN9 → raw_ia
Iv → 霍尔 V → Vout = VREF + Iv × Sensitivity → ADC2_IN1 → raw_ib
```
电流换算: `I = (raw - offset) × (1000 × v_ref) / (adc_res × sensitivity_mV_per_A)`

## CubeMX 配置

### 1. TIM1 — 高级定时器 (PWM + ADC 触发)

| 参数 | 值 | 说明 |
|------|-----|------|
| Clock Source | Internal Clock | |
| Channel1/2/3 | **PWM Generation CHx + PWM Generation CHxN** | 互补输出 |
| Channel4 | **PWM Generation No Output** | 仅作触发源 |
| Counter Period | 4250 | ARR 值, 决定 PWM 频率 |
| Auto-reload preload | Enable | |
| Center-aligned mode | **Mode 1 (Center-Aligned 1)** | 三角波计数 |
| 中断 | **TIM1 BRK** | HardFault 时紧急关断 |

**PWM 频率计算:**
```
f_PWM = TIM_CLK / (2 × ARR) = 170MHz / (2 × 4250) ≈ 20kHz
```

中心对称模式 (Center-Aligned) 的优点:
- ADC 在 CNT=CCR4 时触发 → PWM 中心点 → 上下管都导通 → 电流采样窗口最佳
- 等效开关频率翻倍 (谐波能量减半)

### 2. TIM1 — Break / DeadTime

| 参数 | 值 | 说明 |
|------|-----|------|
| BRK Polarity | High | HardFault 触发刹车 |
| BRK Filter | 0 | |
| Automatic Output | Enable | |
| **Dead Time** | **~128 (约 0.75μs)** | 防止上下管同时导通 |

死区时间: `dt = DeadTime / TIM_CLK ≈ 128/170MHz ≈ 0.75μs`

### 3. ADC1 — 电流采样 (注入组, 霍尔 U 相)

| 参数 | 值 |
|------|-----|
| Mode | Independent |
| Scan Conversion | Enable |
| Continuous Conversion | Disable |
| Injected Conversion Mode | External Trigger (TIM1 CH4) |
| **Injected Rank 1** | Channel 9 (PC3, I_U), Sampling Time = 2.5 cycles |
| **Injected Rank 2** | Channel 9 (PC3, I_Bus 预留), Sampling Time = 2.5 cycles |

### 4. ADC2 — 辅助采样 (常规 DMA + 霍尔 V 相注入组)

**常规组 (DMA 循环):**

| 参数 | 值 |
|------|-----|
| DMA | DMA1 Channel1, Circular, HalfWord |
| **Regular Rank 1** | Channel 17 (PA4, V_BUS), Sampling Time = 2.5 cycles |
| **Regular Rank 2** | Channel 11 (PC5, NTC1), Sampling Time = 2.5 cycles |
| **Regular Rank 3+** | 如需多 NTC, 依次增加通道 |

**注入组:**

| 参数 | 值 |
|------|-----|
| **Injected Rank 1** | Channel 1 (PA0, I_V), Sampling Time = 2.5 cycles |
| **Injected Rank 2** | Channel 1 (PA0, 双采样), Sampling Time = 2.5 cycles |
| External Trigger | **Timer 1 Trigger Out** (与 ADC1 同步) |

## PWM 波形验证

调试时使用 `Test_01_PWM_Manual` 测试:
1. 设 `cfg_duty_u = 0.1, cfg_duty_v = 0.1, cfg_duty_w = 0.1`
2. 示波器测量 TIM1 CH1 和 CH1N → 死区时间正确
3. 示波器测量 PB9 → ISR 频率 (应有 ~10kHz 方波)
4. 用 ADC 读寄存器验证采样值在零点附近 (~2048)

## 电流采样校准

Lib 在上电后自动执行 ADC 零点校准 (采集 1000 个样本取平均):
- 校准期间 PWM 输出关闭, 电流理论上为零
- 偏移值存入 `MotorManager::ctx_.offset_ia/ib/ic`
- 之后每次采样都减去偏移量

## ISR 执行时间测量

PB9 引脚在 ISR 入口翻转:
- 翻转频率 = ISR 频率 / 2 = 10kHz (因为每个 ISR 翻转一次)
- 示波器观测 PB9 → 应有 ~10kHz 方波, 确认 ISR 正常运行
- 高电平时间 = ISR 执行时间 (不应超过半个 PWM 周期 25μs)
- 如果 ISR 超时, 会错过下一个 PWM 周期 → ADC 溢出 → 控制失调
