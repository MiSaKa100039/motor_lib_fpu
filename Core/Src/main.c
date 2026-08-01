/*
 * 文件说明：负责当前工程的 MCU 级启动入口、Core 硬件 bring-up、TIM1/ADC/USART1 最小组织，
 * 并把用户层初始化与主循环串起来。这里不承载电机控制策略本身。
 */

#include <stdint.h>

#include "Usermain.h"
#include "lca039_clock_config.h"
#include "lcm32f039.h"
#include "lcm32f039_adc.h"
#include "lcm32f039_dma.h"
#include "lcm32f039_gpio.h"
#include "lcm32f039_opa.h"
#include "lcm32f039_rcc.h"
#include "lcm32f039_tim.h"
#include "lcm32f039_uart.h"
#include "main.h"

#define LCA039_DATA_PROBE_VALUE (0x4C434130UL)

#ifndef CORE_ENABLE_USART1_TX_DMA
#define CORE_ENABLE_USART1_TX_DMA 0
#endif

static volatile uint32_t s_data_probe = LCA039_DATA_PROBE_VALUE;
static volatile uint32_t s_bss_probe;

volatile uint32_t g_heartbeat = 0U;
volatile LCA039_ClockStatus g_clock_status = LCA039_CLOCK_STATUS_NOT_CONFIGURED;
volatile LCA039_BootStatus g_boot_status = LCA039_BOOT_STATUS_OK;

volatile uint32_t Core_Motor_Tim1UpdateIrqCount = 0U;
volatile uint32_t Core_Motor_AdcEocIrqCount = 0U;
volatile uint32_t Core_Motor_AdcUnexpectedIrqCount = 0U;
volatile uint16_t Core_Motor_AdcLastRaw = 0U;
volatile uint16_t Core_Motor_AdcTriggerPairRawFirst = 0U;
volatile uint16_t Core_Motor_AdcTriggerPairRawSecond = 0U;
volatile uint16_t Core_ADC_Opa2Raw = 0U;
volatile uint16_t Core_ADC_VbusRaw = 0U;
volatile uint16_t Core_ADC_TempRaw0 = 0U;
volatile uint32_t Core_USART1_TxDmaStartedCount = 0U;
volatile uint32_t Core_USART1_TxDmaCompleteCount = 0U;
volatile uint32_t Core_USART1_TxDmaErrorCount = 0U;
volatile uint32_t Core_USART1_ActualBaudRate = 0U;
volatile uint32_t Core_USART1_BaudErrorPpm = 0U;

typedef enum
{
    CORE_ADC_SCAN_SLOT_OPA2 = 0U,
    CORE_ADC_SCAN_SLOT_VBUS = 1U,
    CORE_ADC_SCAN_SLOT_COUNT = 2U
} Core_AdcScanSlot;

#if CORE_ENABLE_USART1_TX_DMA
typedef enum
{
    CORE_USART1_TX_DMA_IRQ_NONE = 0U,
    CORE_USART1_TX_DMA_IRQ_COMPLETE = 1U,
    CORE_USART1_TX_DMA_IRQ_ERROR = 2U
} Core_USART1_TxDmaIrqStatus;
#endif

static uint8_t s_motor_tim1_initialized = 0U;
static uint8_t s_motor_adc_initialized = 0U;
static uint8_t s_motor_initialized = 0U;
static uint32_t s_tim1_pwm_frequency_hz = 0U;
static uint32_t s_tim1_adc_trigger_frequency_hz = 0U;
static uint16_t s_tim1_pwm_period_counts = 0U;
static uint16_t s_tim1_adc_trigger_compare_counts = 0U;
static volatile uint8_t s_adc_trigger_pair_mode_enabled = 0U;
static volatile uint8_t s_adc_trigger_pair_sample_index = 0U;
static volatile uint8_t s_adc_regular_sample_index = 0U;
static uint16_t s_adc_trigger_pair_first_counts = 0U;
static uint16_t s_adc_trigger_pair_second_counts = 0U;
#if CORE_ENABLE_USART1_TX_DMA
static volatile uint8_t s_usart1_tx_dma_busy = 0U;
static uint32_t s_usart1_target_baud_rate = 0U;
static uint8_t s_usart1_gpio_ready = 0U;
static uint8_t s_usart1_uart_ready = 0U;
static uint8_t s_usart1_dma_ready = 0U;
#endif

static uint32_t motorAdcClockMode(void)
{
    /* ADC 同步分频来自 CMake 生成配置，保证 ADC 时钟不超过芯片允许的 24 MHz。 */
#if LCA039_ADC_SYNC_DIV_MIN == 1
    return ADC_ClockMode_sysnClk;
#elif LCA039_ADC_SYNC_DIV_MIN == 2
    return ADC_ClockMode_SynClkDiv2;
#elif LCA039_ADC_SYNC_DIV_MIN == 4
    return ADC_ClockMode_SynClkDiv4;
#elif LCA039_ADC_SYNC_DIV_MIN == 6
    return ADC_ClockMode_SynClkDiv6;
#elif LCA039_ADC_SYNC_DIV_MIN == 8
    return ADC_ClockMode_SynClkDiv8;
#elif LCA039_ADC_SYNC_DIV_MIN == 10
    return ADC_ClockMode_SynClkDiv10;
#elif LCA039_ADC_SYNC_DIV_MIN == 12
    return ADC_ClockMode_SynClkDiv12;
#elif LCA039_ADC_SYNC_DIV_MIN == 16
    return ADC_ClockMode_SynClkDiv16;
#else
#error "Unsupported generated ADC synchronous divider"
#endif
}

static void updateMotorInitialized(void)
{
    s_motor_initialized = ((s_motor_tim1_initialized != 0U) &&
                           (s_motor_adc_initialized != 0U))
                              ? 1U
                              : 0U;
}

static uint16_t adcTriggerDutyToCompare(float duty)
{
    const uint32_t period = s_tim1_pwm_period_counts;
    float clamped = duty;
    float compare;

    if (period == 0UL) return 0U;
    if (clamped < 0.0f) clamped = 0.0f;
    if (clamped > 1.0f) clamped = 1.0f;

    compare = clamped * (float)period;
    if (compare < 1.0f) compare = 1.0f;
    if (compare > (float)(period - 1UL)) compare = (float)(period - 1UL);
    return (uint16_t)compare;
}

static uint32_t coreEnterCritical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void coreExitCritical(uint32_t primask)
{
    if (primask == 0UL) __enable_irq();
}

static void configureMotorAdcGpio(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AN;
    gpio.GPIO_Speed = GPIO_Speed_Level_1;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AN;
    gpio.GPIO_Speed = GPIO_Speed_Level_1;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOF, &gpio);
}

static void configureMotorOpa2(void)
{
    OPA_InitTypeDef opa;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ANACTRL, ENABLE);
    OPA_StructInit(&opa);
    opa.OPA_Oppselect = OPA2_Oppselect_PF4;
    opa.OPA_Opnselect = OPA2_Opnselect_PF5;
    opa.OPA_Gain = OPA_Gain_20;
    opa.OPA_BiasVoltage = OPA_BiasVoltage_EN;
    opa.OPA_BiasVoltageSource = OPA_OPA_BiasVoltageSource_HALFVRH;
    opa.OPA_FeedbackRes = OPA_FeedbackRes_EN;
    opa.OPA_Opoto_Gpio = OPA_OPOEX_EN;
    opa.OPA_Opoto2_Gpio = OPA_OPOEX2_DISEN;
    opa.OPA_OppRES_short = OPA_OppRES_short_DISEN;
    opa.OPA_OpnRES_short = OPA_OpnRES_short_DISEN;
    OPA_Init(OPA2, &opa);
    OPA_Cmd(OPA2, ENABLE);
}

#if CORE_ENABLE_USART1_TX_DMA
static void configureUsart1Gpio(void)
{
    GPIO_InitTypeDef gpio;

    /* USART1 固定使用 PB14/PB15 AF6，这里只完成引脚复用配置，不引入上层协议语义。 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_6);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_6);
    gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_Level_2;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
    s_usart1_gpio_ready = 1U;
}

static uint8_t updateUsart1BaudDebug(uint32_t baud_rate)
{
    const uint32_t uart_clock_hz = LCA039_PCLK1_HZ;
    uint32_t divider64;
    uint32_t actual_baud_rate;
    uint32_t baud_error;
    uint32_t baud_error_ppm;

    if (baud_rate == 0UL)
    {
        Core_USART1_ActualBaudRate = 0U;
        Core_USART1_BaudErrorPpm = 0U;
        return 0U;
    }

    divider64 = ((uart_clock_hz * 4UL) + (baud_rate / 2UL)) / baud_rate;
    if (divider64 < 64UL)
    {
        Core_USART1_ActualBaudRate = 0U;
        Core_USART1_BaudErrorPpm = 1000000UL;
        return 0U;
    }

    actual_baud_rate = ((uart_clock_hz * 4UL) + (divider64 / 2UL)) / divider64;
    baud_error = (actual_baud_rate > baud_rate)
                     ? (actual_baud_rate - baud_rate)
                     : (baud_rate - actual_baud_rate);
    baud_error_ppm = ((baud_error * 1000000UL) + (baud_rate / 2UL)) / baud_rate;
    Core_USART1_ActualBaudRate = actual_baud_rate;
    Core_USART1_BaudErrorPpm = baud_error_ppm;
    return (baud_error_ppm <= 20000UL) ? 1U : 0U;
}

static void configureUsart1TxDma(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);
    DMA_DeInit(DMA1_Channel0);
    DMA_RemapConfig(DMA1_Channel0, DMA_ReqNum0, DMA_REQ_UART1_TX);
    DMA_ClearITPendingBit(DMA1_IT_TC0);
    DMA_ClearITPendingBit(DMA1_IT_TE0);
    DMA_ITConfig(DMA1_Channel0, DMA1_IT_TC0, ENABLE);

    /* 厂商 SDK 2.0.4 的 DMA_ITConfig TE 分支写错寄存器，这里只在 Core 内做板级修正。 */
    DMA1->MaskErr = 0x101UL;
    NVIC_ClearPendingIRQ(DMAC_CH0_IRQn);
    NVIC_SetPriority(DMAC_CH0_IRQn, 2U);
    NVIC_EnableIRQ(DMAC_CH0_IRQn);
    s_usart1_dma_ready = 1U;
}

static uint8_t startUsart1TxDma(const uint8_t *data, uint16_t length)
{
    DMA_InitTypeDef dma;

    if ((data == 0) || (length == 0U)) return 0U;
    DMA_Cmd(DMA1_Channel0, DISABLE);
    DMA_ClearITPendingBit(DMA1_IT_TC0);
    DMA_ClearITPendingBit(DMA1_IT_TE0);
    DMA_StructInit(&dma);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&UART1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)data;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = length;
    dma.DMA_MSIZE = DMA_MSIZE_1;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Low;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel0, &dma);
    DMA_Cmd(DMA1_Channel0, ENABLE);
    return 1U;
}

static Core_USART1_TxDmaIrqStatus takeUsart1TxDmaIrq(void)
{
    /* 错误优先处理，避免 TC 和 TE 同时置位时误判为发送成功。 */
    if ((DMA_GetITStatus(DMA1_IT_TE0) == SET) ||
        (DMA_GetFlagStatus(DMA1_FLAG_TE0) == SET))
    {
        DMA_ClearITPendingBit(DMA1_IT_TE0);
        DMA_ClearITPendingBit(DMA1_IT_TC0);
        DMA_Cmd(DMA1_Channel0, DISABLE);
        return CORE_USART1_TX_DMA_IRQ_ERROR;
    }
    if ((DMA_GetITStatus(DMA1_IT_TC0) == SET) ||
        (DMA_GetFlagStatus(DMA1_FLAG_TC0) == SET))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC0);
        DMA_Cmd(DMA1_Channel0, DISABLE);
        return CORE_USART1_TX_DMA_IRQ_COMPLETE;
    }
    return CORE_USART1_TX_DMA_IRQ_NONE;
}

static void clearUsart1TxDmaBusy(void)
{
    const uint32_t primask = coreEnterCritical();
    s_usart1_tx_dma_busy = 0U;
    coreExitCritical(primask);
}
#endif

static void resetMotorRuntimeState(void)
{
    /* 运行计数只服务 Core 硬件诊断，重新初始化电机路径时统一清零。 */
    Core_Motor_Tim1UpdateIrqCount = 0U;
    Core_Motor_AdcEocIrqCount = 0U;
    Core_Motor_AdcUnexpectedIrqCount = 0U;
    Core_Motor_AdcLastRaw = 0U;
    Core_Motor_AdcTriggerPairRawFirst = 0U;
    Core_Motor_AdcTriggerPairRawSecond = 0U;
    Core_ADC_Opa2Raw = 0U;
    Core_ADC_VbusRaw = 0U;
    Core_ADC_TempRaw0 = 0U;
    s_adc_trigger_pair_mode_enabled = 0U;
    s_adc_trigger_pair_sample_index = 0U;
    s_adc_regular_sample_index = 0U;
}

    /* ADC 快中断只置位后台发送请求，不在 ISR 中执行串口发送或协议组帧。 */
static void stopAdc(void)
{
    NVIC_DisableIRQ(ADC_IRQn);
    if ((ADC->CR & ADC_CR_ADSTART_Msk) != 0UL)
    {
        ADC_StopOfConversion(ADC);
    }
    if ((ADC->CR & ADC_CR_ADEN_Msk) != 0UL)
    {
        ADC_Cmd(ADC, DISABLE);
    }
}

static void configureMotorAdcChannels(void)
{
    uint8_t channels[16] = {0U};

    channels[CORE_ADC_SCAN_SLOT_OPA2] = ADCIN_2_A;
    channels[CORE_ADC_SCAN_SLOT_VBUS] = ADCIN_1_A;
    /* LC32 CHNUM stores the last sequence index, not the channel count. */
    ADC_ChannelConfig((uint32_t)(CORE_ADC_SCAN_SLOT_COUNT - 1U), channels);
}

static void configureAdc(uint32_t trigger, uint32_t edge)
{
    ADC_InitTypeDef init;

    stopAdc();
    ADC_DeInit(ADC);
    ADC_StructInit(&init);
    init.ADC_ClkMode = motorAdcClockMode();
    init.ADC_Resolution = ADC_Resolution_12b;
    init.ADC_AutoWatiMode = DISABLE;
    init.ADC_DMATRIG_LEVEL = FIFO_DMA_NOEMPTY;
    init.ADC_DMA = DISABLE;
    init.ADC_DiscMode = DISABLE;
    init.ADC_SamecMode = DISABLE;
    init.ADC_ExternalTrigConvEdge = edge;
    init.ADC_ExternalTrigConv = trigger;
    init.ADC_DataAlign = ADC_DataAlign_Right;
    init.ADC_ScanDirection = ADC_ScanDirection_Upward;
    init.ADC_Vref = ADC_Vref_Internal_5V;
    init.ADC_SampleTime = 7UL;
    init.ADC_SampleDelayTime = 0UL;
    init.ADC_AdjustTime = 7UL;
    init.ADC_ConvertTime = 2UL;
    init.ADC_ConvertPulseTime = 0UL;
    ADC_Init(ADC, &init);

    configureMotorAdcChannels();
    s_adc_regular_sample_index = 0U;
    ADC_ClearITPendingBit(ADC, ADC_IT_EOC | ADC_IT_EOS);
    ADC_ITConfig(ADC, ADC_IT_EOC, ENABLE);
    NVIC_ClearPendingIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 0U);
    NVIC_EnableIRQ(ADC_IRQn);
    ADC_Cmd(ADC, ENABLE);
}

static void disablePowerChannels(void)
{
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
}

static void resetTimer(void)
{
    TIM_Cmd(TIM1, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM1, DISABLE);
}

static uint8_t configureUpdateTimer(uint32_t frequencyHz)
{
    /* 测试用更新定时器同样基于 PCLK1，改 APB1 分频后这里会自动换算预分频和周期。 */
    TIM_TimeBaseInitTypeDef timeBase;
    uint32_t totalTicks;
    uint32_t prescaler;
    uint32_t periodCounts;

    if ((frequencyHz == 0UL) || (frequencyHz > LCA039_PCLK1_HZ)) return 0U;
    totalTicks = LCA039_PCLK1_HZ / frequencyHz;
    if ((totalTicks == 0UL) || ((LCA039_PCLK1_HZ % frequencyHz) != 0UL)) return 0U;
    prescaler = (totalTicks - 1UL) / 65535UL;
    periodCounts = totalTicks / (prescaler + 1UL);
    if ((periodCounts == 0UL) || (periodCounts > 65535UL) ||
        ((totalTicks % (prescaler + 1UL)) != 0UL))
    {
        return 0U;
    }

    resetTimer();
    timeBase.TIM_Prescaler = (uint16_t)prescaler;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    timeBase.TIM_Period = periodCounts - 1UL;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBase.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &timeBase);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    NVIC_ClearPendingIRQ(TIM1_NON_CC_IRQn);
    NVIC_SetPriority(TIM1_NON_CC_IRQn, 1U);
    NVIC_EnableIRQ(TIM1_NON_CC_IRQn);
    TIM_Cmd(TIM1, ENABLE);
    return 1U;
}

static uint8_t configureCenterAlignedTriggerTimer(void)
{
    const uint32_t tim1_pwm_frequency_hz = 20000UL;
    uint32_t period_counts;
    uint32_t arr;
    uint32_t adc_trigger_compare;
    TIM_TimeBaseInitTypeDef timeBase;
    TIM_OCInitTypeDef outputCompare;

    /* TIM1 使用中心对齐计数，PWM/ADC 触发频率只在本初始化段内计算，避免在文件头堆放外设配置宏。 */
    if ((tim1_pwm_frequency_hz == 0UL) ||
        ((LCA039_PCLK1_HZ % (2UL * tim1_pwm_frequency_hz)) != 0UL))
    {
        return 0U;
    }
    period_counts = LCA039_PCLK1_HZ / (2UL * tim1_pwm_frequency_hz);
    if ((period_counts == 0UL) || (period_counts > 65535UL)) return 0U;
    arr = period_counts - 1UL;
    if (arr <= 10UL) return 0U;
    adc_trigger_compare = arr - 10UL;

    s_tim1_pwm_frequency_hz = tim1_pwm_frequency_hz;
    s_tim1_adc_trigger_frequency_hz = tim1_pwm_frequency_hz;
    s_tim1_pwm_period_counts = (uint16_t)period_counts;
    s_tim1_adc_trigger_compare_counts = (uint16_t)adc_trigger_compare;
    s_adc_trigger_pair_first_counts = (uint16_t)adc_trigger_compare;
    s_adc_trigger_pair_second_counts = (uint16_t)adc_trigger_compare;

    resetTimer();
    timeBase.TIM_Prescaler = 0U;
    timeBase.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    timeBase.TIM_Period = arr;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBase.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &timeBase);

    TIM_OCStructInit(&outputCompare);
    outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputCompare.TIM_OutputState = TIM_OutputState_Enable;
    outputCompare.TIM_OutputNState = TIM_OutputNState_Disable;
    outputCompare.TIM_Pulse = adc_trigger_compare;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
    outputCompare.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC4Init(TIM1, &outputCompare);
    /*
     * Pair trigger mode rewrites CH4 CCR after the first ADC EOC.
     * CH4 must update immediately; preload would delay the second trigger to the next update.
     */
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    NVIC_DisableIRQ(TIM1_NON_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM1_NON_CC_IRQn);
    disablePowerChannels();

    /* TIM1 CH4 只作为 ADC 触发通道；CH1/CH2/CH3 及互补功率输出默认保持关闭。 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    return 1U;
}
static void stopTimer(void)
{
    if (s_motor_tim1_initialized == 0U) return;
    TIM_Cmd(TIM1, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    NVIC_DisableIRQ(TIM1_NON_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM1_NON_CC_IRQn);
    disablePowerChannels();
}

static uint8_t startTimerAdcPath(void)
{
    /* TIM1 CH4 上升沿触发 ADC，启动采样链路时仍不打开三相功率通道。 */
    stopTimer();
    configureAdc(ADC_ExternalTrigConv_T1_CC4, ADC_ExternalTrigConvEdge_Rising);
    if (configureCenterAlignedTriggerTimer() == 0U) return 0U;
    ADC_StartOfConversion(ADC);
    TIM_Cmd(TIM1, ENABLE);
    return 1U;
}

static uint8_t takeAdcEoc(uint16_t *raw)
{
    uint32_t pending;
    if (raw == 0) return 0U;
    pending = ADC->ISR & ADC->IER;
    if ((pending & ADC_IT_EOC) == 0UL)
    {
        if (pending != 0UL) ADC_ClearITPendingBit(ADC, pending);
        return 0U;
    }
    *raw = (uint16_t)ADC->DR;
    ADC_ClearITPendingBit(ADC, pending | ADC_IT_EOS);
    return 1U;
}

static uint8_t acknowledgeTimerUpdate(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == RESET) return 0U;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    return 1U;
}

void MX_GPIO_Init(void)
{
    configureMotorAdcGpio();
#if CORE_ENABLE_USART1_TX_DMA
    /* 当前 GPIO 初始化只覆盖 USART1 TX/RX；后续 ADC 外部采样引脚也应统一放在这里。 */
    configureUsart1Gpio();
#endif
}

void MX_DMA_Init(void)
{
#if CORE_ENABLE_USART1_TX_DMA
    /* DMA1 Channel0 专用于 USART1 TX，IRQ 入口在 lcm32f039_it.c 中统一分发。 */
    configureUsart1TxDma();
#endif
}

void MX_USART1_UART_Init(void)
{
#if CORE_ENABLE_USART1_TX_DMA
    const uint32_t baud_rate = 921600UL;
    UART_InitTypeDef uart;

    /* USART1 只提供底层串口发送能力，上层帧缓冲和丢帧策略不进入 Core。 */
    /* 初始化段内直接给出目标波特率；UART_Init() 会继续写入 IBRD/FBRD 分频寄存器。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART1, ENABLE);
    RCC_UART1CLKConfig(RCC_UART1CLK_APB1);
    if (updateUsart1BaudDebug(baud_rate) == 0U)
    {
        s_usart1_uart_ready = 0U;
        return;
    }

    UART_StructInit(&uart);
    uart.UART_BaudRate = baud_rate;
    uart.UART_WordLength = UART_WordLength_8b;
    uart.UART_StopBits = UART_StopBits_1;
    uart.UART_Parity = UART_Parity_No;
    uart.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    uart.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
    UART_Init(UART1, &uart);
    UART_FIFOEnable(UART1, ENABLE);
    UART_DMACmd(UART1, UART_DMAReq_Tx, ENABLE);
    UART_Cmd(UART1, ENABLE);
    s_usart1_target_baud_rate = baud_rate;
    s_usart1_uart_ready = 1U;
#endif
}
void MX_TIM1_Init(void)
{
    /* MX_TIM1_Init 只配置 TIM1 计数、CH4 ADC 触发和安全默认态；功率输出不会在这里启动。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM1, ENABLE);
    s_motor_tim1_initialized = 0U;
    if (configureCenterAlignedTriggerTimer() != 0U)
    {
        s_motor_tim1_initialized = 1U;
        TIM_Cmd(TIM1, DISABLE);
        Core_TIM1_DisablePwmOutputs();
    }
    updateMotorInitialized();
}

void MX_ADC_Init(void)
{
    /* MX_ADC_Init 配置 ADC 时钟、触发源和 EOC 中断，采样结果由 ADC IRQ 更新。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC, ENABLE);
    configureMotorAdcGpio();
    configureMotorOpa2();
    configureAdc(ADC_ExternalTrigConv_SoftWare, ADC_ExternalTrigConvEdge_None);
    s_motor_adc_initialized = 1U;
    updateMotorInitialized();
}

bool Core_Motor_HardwareInitialize(void)
{
    resetMotorRuntimeState();
    s_motor_initialized = 0U;
    MX_TIM1_Init();
    MX_ADC_Init();
    return s_motor_initialized != 0U;
}

bool Core_Motor_HardwareStart(void)
{
    return Core_Motor_TestStartTimerAdc();
}

bool Core_Motor_HardwareInitialized(void)
{
    return s_motor_initialized != 0U;
}

bool Core_Motor_HardwareBindingEnabled(void)
{
    return false;
}

static uint8_t cacheRegularAdcSample(uint16_t raw)
{
    const uint8_t slot = s_adc_regular_sample_index;

    Core_Motor_AdcLastRaw = raw;
    if (slot == CORE_ADC_SCAN_SLOT_OPA2)
    {
        Core_ADC_Opa2Raw = raw;
    }
    else if (slot == CORE_ADC_SCAN_SLOT_VBUS)
    {
        Core_ADC_VbusRaw = raw;
    }
    ++Core_Motor_AdcEocIrqCount;

    s_adc_regular_sample_index = (uint8_t)(slot + 1U);
    if (s_adc_regular_sample_index >= CORE_ADC_SCAN_SLOT_COUNT)
    {
        s_adc_regular_sample_index = 0U;
        return 1U;
    }
    return 0U;
}

bool Core_Motor_HandleAdcIrq(void)
{
    /* ADC IRQ 入口只读取原始值、更新计数并触发后台发送标志，控制/观测逻辑留给上层。 */
    uint16_t raw = 0U;
    uint8_t sequence_complete;
    if (takeAdcEoc(&raw) == 0U)
    {
        ++Core_Motor_AdcUnexpectedIrqCount;
        return false;
    }
    sequence_complete = cacheRegularAdcSample(raw);
    if (sequence_complete == 0U)
    {
        return false;
    }
    if (s_adc_trigger_pair_mode_enabled != 0U)
    {
        if (s_adc_trigger_pair_sample_index == 0U)
        {
            Core_Motor_AdcTriggerPairRawFirst = raw;
            s_adc_trigger_pair_sample_index = 1U;
            /*
             * The first EOC only stores raw data and switches CH4 to the second compare.
             * Fast tick is released only after the second EOC completes the pair.
             */
            TIM_SetCompare4(TIM1, s_adc_trigger_pair_second_counts);
            return false;
        }

        Core_Motor_AdcTriggerPairRawSecond = raw;
        s_adc_trigger_pair_sample_index = 0U;
        TIM_SetCompare4(TIM1, s_adc_trigger_pair_first_counts);
        return true;
    }
    Core_Motor_AdcLastRaw = raw;
    Core_Motor_AdcTriggerPairRawFirst = raw;
    Core_Motor_AdcTriggerPairRawSecond = raw;
    return true;
}

void Core_Motor_HandleTim1Irq(void)
{
    if (acknowledgeTimerUpdate() != 0U)
    {
        ++Core_Motor_Tim1UpdateIrqCount;
    }
}

uint16_t Core_ADC_GetLastRaw(void)
{
    return Core_Motor_AdcLastRaw;
}

uint16_t Core_ADC_GetVbusRaw(void)
{
    return Core_ADC_VbusRaw;
}

uint16_t Core_ADC_GetTempRaw(uint8_t index)
{
    (void)index;
    return 0U;
}

void Core_ADC_GetTriggerPairRaw(uint16_t *raw_first, uint16_t *raw_second)
{
    if (raw_first != 0)
    {
        *raw_first = Core_Motor_AdcTriggerPairRawFirst;
    }
    if (raw_second != 0)
    {
        *raw_second = Core_Motor_AdcTriggerPairRawSecond;
    }
}

void Core_TIM1_DisablePwmOutputs(void)
{
    if (s_motor_tim1_initialized != 0U)
    {
        disablePowerChannels();
    }
}

void Core_TIM1_SetPwmCompare(uint16_t phaseU, uint16_t phaseV, uint16_t phaseW)
{
    /* Core 只写 TIM1 比较寄存器，duty 计算和桥臂策略由 BSP/Platform/Lib 决定。 */
    if (s_motor_tim1_initialized == 0U) return;
    TIM_SetCompare1(TIM1, phaseU);
    TIM_SetCompare2(TIM1, phaseV);
    TIM_SetCompare3(TIM1, phaseW);
}

void Core_TIM1_SetAdcTriggerPairDuties(float first_trigger_duty,
                                       float second_trigger_duty)
{
    const uint32_t primask = coreEnterCritical();
    s_adc_trigger_pair_first_counts =
        adcTriggerDutyToCompare(first_trigger_duty);
    s_adc_trigger_pair_second_counts =
        adcTriggerDutyToCompare(second_trigger_duty);
    s_adc_trigger_pair_sample_index = 0U;
    if (s_motor_tim1_initialized != 0U)
    {
        TIM_SetCompare4(TIM1, s_adc_trigger_pair_first_counts);
    }
    coreExitCritical(primask);
}

void Core_ADC_EnableTriggerPairMode(bool enabled)
{
    const uint32_t primask = coreEnterCritical();
    s_adc_trigger_pair_mode_enabled = enabled ? 1U : 0U;
    s_adc_trigger_pair_sample_index = 0U;
    if (!enabled && s_motor_tim1_initialized != 0U)
    {
        TIM_SetCompare4(TIM1, s_tim1_adc_trigger_compare_counts);
    }
    coreExitCritical(primask);
}

bool Core_TIM1_PwmOutputsEnabled(void)
{
    const uint32_t powerChannelMask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                      TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                      TIM_CCER_CC3E | TIM_CCER_CC3NE;
    if (s_motor_tim1_initialized == 0U) return false;
    return (TIM1->CCER & powerChannelMask) != 0UL;
}

uint16_t Core_TIM1_PwmPeriodCounts(void)
{
    return s_tim1_pwm_period_counts;
}

uint16_t Core_TIM1_AdcTriggerCompareCounts(void)
{
    return s_tim1_adc_trigger_compare_counts;
}

uint32_t Core_TIM1_PwmFrequencyHz(void)
{
    return s_tim1_pwm_frequency_hz;
}

uint32_t Core_TIM1_AdcTriggerFrequencyHz(void)
{
    return s_tim1_adc_trigger_frequency_hz;
}
void Core_USART1_TxDmaResetRuntime(void)
{
#if CORE_ENABLE_USART1_TX_DMA
    if (s_usart1_dma_ready != 0U)
    {
        DMA_Cmd(DMA1_Channel0, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC0);
        DMA_ClearITPendingBit(DMA1_IT_TE0);
    }
    clearUsart1TxDmaBusy();
    if (s_usart1_target_baud_rate != 0U)
    {
        (void)updateUsart1BaudDebug(s_usart1_target_baud_rate);
    }
#else
    Core_USART1_ActualBaudRate = 0U;
    Core_USART1_BaudErrorPpm = 0U;
#endif
    Core_USART1_TxDmaStartedCount = 0U;
    Core_USART1_TxDmaCompleteCount = 0U;
    Core_USART1_TxDmaErrorCount = 0U;
}
bool Core_USART1_TxDmaBusy(void)
{
#if CORE_ENABLE_USART1_TX_DMA
    return s_usart1_tx_dma_busy != 0U;
#else
    return false;
#endif
}

bool Core_USART1_TxDmaTryStart(const uint8_t *data, uint16_t length)
{
#if CORE_ENABLE_USART1_TX_DMA
    uint32_t primask;

    if ((data == 0) || (length == 0U) || (s_usart1_gpio_ready == 0U) ||
        (s_usart1_uart_ready == 0U) || (s_usart1_dma_ready == 0U))
    {
        return false;
    }

    primask = coreEnterCritical();
    if (s_usart1_tx_dma_busy != 0U)
    {
        coreExitCritical(primask);
        return false;
    }
    s_usart1_tx_dma_busy = 1U;
    coreExitCritical(primask);

    /* DMA 直接读取调用方缓冲；调用方必须在 busy 清零前保持缓冲内容不变。 */
    if (startUsart1TxDma(data, length) == 0U)
    {
        clearUsart1TxDmaBusy();
        return false;
    }
    ++Core_USART1_TxDmaStartedCount;
    return true;
#else
    (void)data;
    (void)length;
    return false;
#endif
}

void Core_USART1_TxDmaIrqHandler(void)
{
#if CORE_ENABLE_USART1_TX_DMA
    const Core_USART1_TxDmaIrqStatus status = takeUsart1TxDmaIrq();
    if (status == CORE_USART1_TX_DMA_IRQ_COMPLETE)
    {
        ++Core_USART1_TxDmaCompleteCount;
        clearUsart1TxDmaBusy();
    }
    else if (status == CORE_USART1_TX_DMA_IRQ_ERROR)
    {
        ++Core_USART1_TxDmaErrorCount;
        clearUsart1TxDmaBusy();
    }
#endif
}
bool Core_Motor_TestConfigureSysTick(uint32_t frequencyHz)
{
    if ((frequencyHz == 0UL) || (SystemCoreClock < frequencyHz)) return false;
    return SysTick_Config(SystemCoreClock / frequencyHz) == 0UL;
}

bool Core_Motor_TestStartTimerUpdate(uint32_t frequencyHz)
{
    if (s_motor_initialized == 0U) return false;
    stopTimer();
    return configureUpdateTimer(frequencyHz) != 0U;
}

bool Core_Motor_TestStartAdcSoftware(void)
{
    if (s_motor_initialized == 0U) return false;
    stopTimer();
    configureAdc(ADC_ExternalTrigConv_SoftWare, ADC_ExternalTrigConvEdge_None);
    return true;
}

bool Core_Motor_TestTriggerAdcSoftware(void)
{
    if (s_motor_initialized == 0U) return false;
    if ((ADC->CR & ADC_CR_ADSTART_Msk) != 0UL) return false;
    ADC_StartOfConversion(ADC);
    return true;
}

bool Core_Motor_TestStartTimerAdc(void)
{
    if (s_motor_initialized == 0U) return false;
    return startTimerAdcPath() != 0U;
}

void Core_Motor_TestStopTimer(void)
{
    stopTimer();
}

static uint32_t checkMemoryInitialization(void)
{
    uint32_t bootStatus = LCA039_BOOT_STATUS_OK;
    if (s_data_probe != LCA039_DATA_PROBE_VALUE)
    {
        bootStatus |= LCA039_BOOT_STATUS_DATA_INIT_FAILED;
    }
    if (s_bss_probe != 0UL)
    {
        bootStatus |= LCA039_BOOT_STATUS_BSS_INIT_FAILED;
    }
    return bootStatus;
}


int main(void)
{
    uint32_t boot_status = checkMemoryInitialization();

    g_boot_status = (LCA039_BootStatus)boot_status;

    /* 按 CubeMX 风格展开启动顺序，Core 只负责 MCU/外设 bring-up，用户策略从 User_Init() 开始。 */
    g_clock_status = SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_ADC_Init();

    User_Init();

    for (;;)
    {
        ++g_heartbeat;
        User_Loop();
    }
}
