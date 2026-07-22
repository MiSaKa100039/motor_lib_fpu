/*
 * BSP_VOFA.cpp — VOFA 串口发送 BSP 实现
 *
 * USART3, current CubeMX setting 115200 bps
 *
 * DMA 非阻塞模式 (DMA1_Channel2, Normal):
 *   - ISR 内调用不阻塞, DMA 后台发送
 *   - DMA TC 中断 → HAL_DMA_IRQHandler → HAL_UART_TxCpltCallback → gState 复位
 *   - DMA 忙时丢帧 (JustFloat +inf 帧尾, 上位机自动跳过不完整帧)
 *   - Telemetry channel count/rate must remain below UART throughput
 *
 * 阻塞模式 (已弃用, 仅调试):
 *   - 将下方 #if 0 改为 #if 1
 *   - 绝不可在 ISR 内使用, 会阻塞 3ms 导致控制周期崩溃
 */

#include "BSP_VOFA.h"
#include "main.h"

extern UART_HandleTypeDef huart3;

void BSP_VOFA_Transmit(const uint8_t* data, uint16_t length)
{
    if (length == 0) return;

#if 0

    /*
     * 阻塞模式: 测试用, 简单可靠
     * HAL_UART_Transmit 内部等待每一字节发送完成
     * Do not use blocking transfer inside the control ISR.
     */
    HAL_UART_Transmit(&huart3, (uint8_t*)data, length, 100);
#else
    /*
     * DMA 模式: ISR 安全, 非阻塞
     * DMA 忙时丢帧 → 上位机靠 +inf 帧尾自动跳过
     */
    if (huart3.gState != HAL_UART_STATE_READY)
        return;
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)data, length);
#endif
}
