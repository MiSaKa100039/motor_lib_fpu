/*
 * VOFA_Sender.h — VOFA 发送器声明
 *
 * VOFA_Sender 是 Lib VOFA 的核心调度器:
 *   1. 持有双缓冲对象 (g_buffer)
 *   2. 通过函数指针 (VOFA_HardwareTransmit) 调用 BSP 层
 *   3. BSP 层通过 Platform_VOFA_Init() 绑定硬件发送函数
 *
 * 调用链:
 *   Platform_VOFA_SendFloat() → VOFA_SendFloat()
 *     → g_buffer.writeFrame() → g_buffer.getReadyBuffer()
 *     → VOFA_HardwareTransmit() → BSP_VOFA_Transmit()
 *     → HAL_UART_Transmit_DMA()
 */

#pragma once

#include "Communication_Config.h"

#ifdef BSP_VOFA_ENABLE

#include <stdint.h>

namespace Lib_VOFA
{
} // namespace Lib_VOFA

#endif /* BSP_VOFA_ENABLE */
