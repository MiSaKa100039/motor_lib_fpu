/*
 * 文件说明：集中放置芯片中断入口，并把 ADC、TIM1、DMA 等底层 IRQ 分发到 Core/Test/Motor 路径。
 * 中断里只做最小必要分发，不承载复杂业务逻辑。
 */

#include "lcm32f039_it.h"

#include "BSP_Motor.h"
#include "main.h"

#include <stdbool.h>

void Motor_Global_Process_Handler(int motorId);
bool Motor_Global_Process_Handler_TickAccepted(int motorId);

#ifndef CORE_ENABLE_USART1_TX_DMA
#define CORE_ENABLE_USART1_TX_DMA 0
#endif

#if defined(LCA039_FIRMWARE_MODE_TEST) && LCA039_FIRMWARE_MODE_TEST
#include "TargetTest.h"
#endif

void ADC_Handler(void)
{
    if (Core_Motor_HandleAdcIrq())
    {
#if defined(LCA039_FIRMWARE_MODE_TEST) && LCA039_FIRMWARE_MODE_TEST
        TargetTest_FastIrq();
        BSP_Motor_RequestVofaTelemetryFromFastIrq();
#else
        if (Motor_Global_Process_Handler_TickAccepted(0))
        {
            BSP_Motor_RequestVofaTelemetryFromFastIrq();
        }
#endif
    }
}

void TIM1_NON_CC_Handler(void)
{
    Core_Motor_HandleTim1Irq();
}

#if CORE_ENABLE_USART1_TX_DMA
void DMAC_CH0_Handler(void)
{
    /* DMA Channel0 当前只服务 USART1 TX；IRQ 入口只做底层清标志和 busy 释放。 */
    Core_USART1_TxDmaIrqHandler();
}
#endif

#if defined(LCA039_FIRMWARE_MODE_TEST) && LCA039_FIRMWARE_MODE_TEST
void SysTick_Handler(void)
{
    ++g_test_milliseconds;
}
#endif
