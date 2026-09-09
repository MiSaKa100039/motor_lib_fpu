# BSP/Communication/VOFA — VOFA+ 串口通信硬件层配置指南

## 如何测试 VOFA

### 步骤 1: 调用 Platform 测试函数

在用户代码中初始化一次, 然后在主循环里周期调用:

```cpp
Platform_VOFA_TestFakeData_Init();

while (1)
{
    Platform_VOFA_TestFakeData_Run();
}
```

`Platform_VOFA_TestFakeData_Run()` 内部按 5ms 节拍发送一帧, 不会阻塞主循环。

### 步骤 2: 编译烧录

CLion → Build → Flash → 运行

### 步骤 3: VOFA+ 上位机查看

1. 打开 VOFA+ → 选择 COM 口 (TTL-USB 模块对应端口)
2. 协议: **JustFloat**, 通道数: **4**
3. 波形图应显示正弦/余弦/锯齿/方波

> **注意**: VOFA 测试只验证串口发送链路, 不再依赖 `Tests` 层或 `UserAPP/Test_Dispatch.cpp`。

## 概述

VOFA (Volt/Current/Freq Analyzer) 是一个串口数据可视化工具。
本 BSP 层通过 UART4 DMA TX 将电机运行数据发送到 PC 端 VOFA+ 软件,
在上位机中实时显示电流/电压/转速等波形。

## 硬件连接

| STM32G474 引脚 | 信号 | 连接 |
|---------------|------|------|
| PC10 | UART4_TX | → TTL-USB 模块 RX (如 CH340) |
| PC11 | UART4_RX | → TTL-USB 模块 TX (调试用, 可不接) |
| GND | GND | → TTL-USB 模块 GND |

## CubeMX 配置

### 1. 启用 UART4

| 参数 | 值 | 说明 |
|------|-----|------|
| Mode | **Asynchronous** | 异步串口 |
| Baud Rate | **115200** | 调试用默认值 |
| Word Length | 8 Bits | |
| Parity | None | |
| Stop Bits | 1 | |
| Hardware Flow Control | Disable | |
| Over Sampling | 16 Samples | |

**波特率选择:**

| 波特率 | 有效带宽 | VOFA 帧率 (8ch × 36B) | 适用场景 |
|--------|---------|----------------------|---------|
| 115200 | ~11.5 KB/s | ~300 fps | 低速监控 (温度/母线电压) |
| 921600 | ~92 KB/s | ~2500 fps | 常规调试 (电流/转速波形) |
| 2000000 | ~200 KB/s | ~5500 fps | 高频调试 (HFI/SMO 波形) |

> 修改 `main.c` 中 `MX_UART4_Init()` 的 `BaudRate` 字段即可切换。

### 2. UART4 DMA TX 配置

在 CubeMX UART4 配置页 → DMA Settings → Add:

| 参数 | 值 |
|------|-----|
| DMA Request | **UART4_TX** |
| Channel | **DMA1 Channel3** (ADC1 使用 Channel1, ADC2 使用 Channel2) |
| Direction | **Memory → Peripheral** |
| Priority | Low |
| Mode | **Normal** (非 Circular) |
| Data Width | Byte |
| Increment Mode (Memory) | Enable |

CubeMX 会自动:
- 在 `main.c` 中声明 `DMA_HandleTypeDef hdma_uart4_tx`
- 在 `MX_DMA_Init()` 中添加 NVIC 中断配置
- 在 `stm32g4xx_hal_msp.c` 中配置 `HAL_UART_MspInit` 的 DMA 初始化
- 在 `stm32g4xx_it.c` 中生成 `DMA1_Channel3_IRQHandler`

### 3. stm32g4xx_hal_conf.h 确认

```c
#define HAL_UART_MODULE_ENABLED    // ★ 必须启用
#define HAL_DMA_MODULE_ENABLED     // ★ 必须启用
```

## VOFA+ 上位机配置

1. 打开 VOFA+ 软件
2. 左侧"控件" → 拖入"波形图"
3. 左上角端口选择 → 选择 TTL-USB 的 COM 口
4. 协议设置:
   - 协议: **JustFloat**
   - 通道数: **与代码中发送的 channels 值一致** (如 4 或 8)
5. 点击连接 → 在控制频率下会自动收到数据

## 协议详解

### JustFloat 帧格式 (VOFA+ 默认协议)

```
| float0 | float1 | float2 | ... | floatN-1 | 0x00 0x00 0x80 0x7F |
|←──────────────── 4×N bytes ─────────────→|←── tail (4 bytes) ──→|

帧尾 = IEEE 754 float +inf (0x7F800000, little-endian → 00 00 80 7F)
VOFA+ 通过此标记检测帧边界
```

### 代码调用链

```
ISR (ADC 完成)
  → MotorMonitorData 读取
  → float data[8] = {ia, ib, ic, id, iq, Vbus, rpm, angle}
  → Platform_VOFA_SendFloat(data, 8)
    → VOFA_SendFloat() [Lib]
      → Buffer::writeFrame()        // 双缓冲写入
      → Buffer::getReadyBuffer()    // 取备妥帧
      → VOFA_HardwareTransmit()     // 函数指针
        → BSP_VOFA_Transmit()       // BSP 层
          → HAL_UART_Transmit_DMA() // STM32 HAL, 异步发送
```

## 双缓冲原理

```
    ISR (20kHz)              DMA UART4
    ┌────────┐              ┌────────┐
    │ frame1 │──bufA───────→│ UART   │→ PC
    └────────┘              └────────┘
    ┌────────┐              ┌────────┐     ← bufA 正在 DMA 发送
    │ frame2 │──bufB        │        │     ← bufB 空闲, 可写入
    └────────┘              └────────┘
    ┌────────┐              ┌────────┐
    │ frame3 │──bufA (等待) │ 发送中  │     ← bufB 正在 DMA 发送
    └────────┘              └────────┘     ← frame3 被丢弃 (降采样)
```

- 如果 DMA 未完成, 新帧被丢弃 (不做排队, 保证实时性)
- 降采样比例由上层控制 (每 N 个 tick 发一帧)
- 不使用环形缓冲 → 无累积延迟

## 故障排查

| 问题 | 可能原因 | 检查 |
|------|---------|------|
| VOFA+ 无数据 | UART4 未配置 DMA TX | CubeMX → UART4 → DMA Settings |
| VOFA+ 显示乱码 | 波特率不匹配 | 代码和上位机波特率一致 |
| VOFA+ 波形错误 | 通道数不一致 | 代码 `channels=4` ↔ 上位机设 4 |
| 丢帧严重 | DMA 通道冲突 | UART4_TX 使用 DMA1_Ch3, ADC1/ADC2 分别使用 DMA1_Ch1/Ch2 |
| 发送阻塞 ISR | HAL_UART_Transmit_DMA 返回 BUSY | 降低发送频率或提高波特率 |
