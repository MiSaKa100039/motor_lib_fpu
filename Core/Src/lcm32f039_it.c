/*
 * 文件说明：集中放置芯片中断入口，并把 ADC、TIM1、DMA 等底层 IRQ 分发到 Core 硬件路径。
 * 中断里只做最小必要分发，不承载复杂业务逻辑。
 */

#include "lcm32f039_it.h"

#include "BSP_Motor.h"
#include "main.h"
#include "lcm32f039_gpio.h"
#include "lcm32f039_tim.h"

#include <stdbool.h>
#include <sys/types.h>

void Motor_Global_Process_Handler(int motorId);
bool Motor_Global_Process_Handler_TickAccepted(int motorId);

#ifndef CORE_ENABLE_USART1_TX_DMA
#define CORE_ENABLE_USART1_TX_DMA 0
#endif

#if defined(LCA039_FIRMWARE_MODE_TEST) && LCA039_FIRMWARE_MODE_TEST
#include "TargetTest.h"
#endif

static uint32_t TIM1_ReadBreakStatusFlags(void)
{
    uint32_t flags = 0UL;

    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_Break) == SET)
    {
        flags |= TIM_FLAG_Break;
    }

    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_Break2) == SET)
    {
        flags |= TIM_FLAG_Break2;
    }

    return flags;
}

static void TIM1_ClearBreakStatusFlags(uint32_t flags)
{
    if ((flags & TIM_FLAG_Break) != 0UL)
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    }

    if ((flags & TIM_FLAG_Break2) != 0UL)
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_Break2);
    }
}

static uint32_t TIM1_MapBreakStatusToBspFaultFlags(uint32_t flags)
{
    uint32_t fault_flags = BSP_MOTOR_HW_FAULT_NONE;

    if ((flags & TIM_FLAG_Break) != 0UL)
    {
        fault_flags |= BSP_MOTOR_HW_FAULT_TIM1_BREAK;
    }

    if ((flags & TIM_FLAG_Break2) != 0UL)
    {
        fault_flags |= BSP_MOTOR_HW_FAULT_TIM1_BREAK2;
    }

    return fault_flags;
}

void ADC_Handler(void)
{
    /* ADC EOS is the regular-scan sequence boundary and control tick source. */
    GPIOA->BSRR = GPIO_Pin_5;
    if (Core_ADC_HandleConversionIrq())
    {
        if (Motor_Global_Process_Handler_TickAccepted(0))
        {
            BSP_Motor_RequestVofaTelemetryFromFastIrq();
        }
    }
    GPIOA->BSRR = ((uint32_t)GPIO_Pin_5 << 16U);
}

void DMAC_CH1_2_Handler(void)
{
    /* ADC regular DMA is disabled in EOS/FIFO diagnostic mode; only clear stray CH1 flags. */
    GPIOA->BSRR = GPIO_Pin_5;
    (void)Core_ADC_HandleDmaIrq();
    GPIOA->BSRR = ((uint32_t)GPIO_Pin_5 << 16U);
}

void TIM1_NON_CC_Handler(void)
{
    const uint32_t break_flags = TIM1_ReadBreakStatusFlags();
    if (break_flags != 0UL)
    {
        /* Break 输入可能持续有效，先关闭 BIE 再清标志，避免同一故障源形成中断风暴。 */
        TIM_ITConfig(TIM1, TIM_IT_Break, DISABLE);
        TIM1_ClearBreakStatusFlags(break_flags);
        NVIC_ClearPendingIRQ(TIM1_NON_CC_IRQn);
        Core_TIM1_LastBreakFlags = break_flags;
        ++Core_TIM1_BreakIrqCount;
        BSP_Motor_LatchHardwareFaultFromISR(TIM1_MapBreakStatusToBspFaultFlags(break_flags));
    }

    Core_TIM1_HandleUpdateIrq();
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
