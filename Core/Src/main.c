/*
 * 文件说明：负责当前工程的 MCU 级启动入口、Core 硬件 bring-up、TIM1/ADC/USART1 最小组织，
 * 并把用户层初始化与主循环串起来。这里不承载电机控制策略本身。
 */

#include <stdint.h>

#include "Usermain.h"
#include "lca039_clock_config.h"
#include "lcm32f039.h"
#include "lcm32f039_acmp.h"
#include "lcm32f039_adc.h"
#include "lcm32f039_dac.h"
#include "lcm32f039_dma.h"
#include "lcm32f039_gpio.h"
#include "lcm32f039_opa.h"
#include "lcm32f039_rcc.h"
#include "lcm32f039_tim.h"
#include "lcm32f039_uart.h"
#include "main.h"
#include "lca039_pwm_outputs.h"

#define LCA039_DATA_PROBE_VALUE (0x4C434130UL)

#ifndef CORE_ENABLE_USART1_TX_DMA
#define CORE_ENABLE_USART1_TX_DMA 0
#endif

#define CORE_ENABLE_BUS_OVERCURRENT_ACMP 0

/*
 * Core 板级硬件参数:
 * 这些值直接服务 TIM1 BDTR、DAC0、ACMP1、TIM1 Break 初始化。
 * 当前数值仅为可编译占位，量产前必须按功率级和过流阈值实测修改。
 */
enum
{
    /* 写入 TIM1 BDTR.DTG 的死区编码值；这是硬件编码，不等同于纳秒值。 */
    CORE_TIM1_DEADTIME_TICKS = 0U
};

_Static_assert(CORE_TIM1_DEADTIME_TICKS <= 255U,
               "CORE_TIM1_DEADTIME_TICKS must fit TIM1 BDTR DTG[7:0]");

enum
{
    /*
     * DAC0 输出码，用作 ACMP1 负端比较阈值。
     * 公式:
     *   V_threshold = V_bias +/- I_limit * R_shunt * OPA_gain
     *   DAC_code = round(V_threshold / V_dac_ref * 1023)
     * 当前 ACMP_P=OPA2、ACMP_N=DAC0、CP_IS_POS，默认按 OPA2_OUT > DAC0_OUT 触发；
     * 若实测电流方向相反，应调整 +/- 方向或比较器极性。
     */
    CORE_BUS_OC_DAC_CODE = 800U
};

_Static_assert(CORE_BUS_OC_DAC_CODE <= 0x3FFU,
               "CORE_BUS_OC_DAC_CODE must fit DAC0 10-bit data register");

/* DAC0 参考源选择，必须与 CORE_BUS_OC_DAC_CODE 的计算参考保持一致。 */
static const uint32_t CORE_BUS_OC_DAC_VREF_SELECT = DAC_Vref_Avdd;

#if CORE_ENABLE_BUS_OVERCURRENT_ACMP
/* ACMP1 输出极性，需与 OPA2 电流方向、DAC 阈值方向和 TIM1 Break 有效电平匹配。 */
static const uint32_t CORE_BUS_OC_ACMP_POLARITY = CP_IS_POS;
#endif

/* ACMP1 接入 TIM1 Break 后的有效电平选择，必须与比较器输出极性配套验证。 */
static const uint32_t CORE_BUS_OC_TIM1_BREAK_LEVEL = TIM_InputLevel_High;

#define CORE_ADC_WAIT_TIMEOUT_LOOPS (LCA039_CLOCK_TIMEOUT_LOOPS)
#define CORE_ADC_REGULAR_FIFO_BUFFER_LENGTH (5U)
#define CORE_ADC_DATA_MASK (0x0FFFUL)
#define CORE_ADC_FIFO_DEPTH_POS (16UL)
#define CORE_ADC_FIFO_DEPTH_MASK (0x1FUL << CORE_ADC_FIFO_DEPTH_POS)
#define CORE_ADC_FIFO_STATUS_FLAGS (ADC_IT_F_EMPTY | ADC_IT_F_ALEMPTY | \
                                    ADC_IT_F_HALF | ADC_IT_F_ALF | ADC_IT_F_FULL)
#define CORE_ADC_DMA_CHANNEL_INDEX (1UL)
#define CORE_ADC_DMA_CHANNEL_ENABLE_MASK (1UL << CORE_ADC_DMA_CHANNEL_INDEX)
#define CORE_ADC_DMA_CHANNEL_IRQ_ENABLE_WRITE (0x101UL << CORE_ADC_DMA_CHANNEL_INDEX)
#define CORE_ADC_DMA_CHANNEL_IRQ_DISABLE_WRITE (0x100UL << CORE_ADC_DMA_CHANNEL_INDEX)

static volatile uint32_t s_data_probe = LCA039_DATA_PROBE_VALUE;
static volatile uint32_t s_bss_probe;

volatile uint32_t g_heartbeat = 0U;
volatile LCA039_ClockStatus g_clock_status = LCA039_CLOCK_STATUS_NOT_CONFIGURED;
volatile LCA039_BootStatus g_boot_status = LCA039_BOOT_STATUS_OK;

volatile uint32_t Core_TIM1_UpdateIrqCount = 0U;
volatile uint32_t Core_TIM1_BreakIrqCount = 0U;
volatile uint32_t Core_TIM1_LastBreakFlags = 0U;
volatile uint32_t Core_ADC_PublishedSampleCount = 0U;
volatile uint32_t Core_ADC_UnexpectedIrqCount = 0U;
volatile uint16_t Core_ADC_LastRaw = 0U;
volatile uint16_t Core_ADC_PhaseURaw = 0U;
volatile uint16_t Core_ADC_PhaseVRaw = 0U;
volatile uint16_t Core_ADC_BusCurrentRaw = 0U;
volatile uint16_t Core_ADC_VbusRaw = 0U;
volatile uint16_t Core_ADC_TempRaw0 = 0U;
volatile uint16_t Core_ADC_RegularFifoBuffer[CORE_ADC_REGULAR_FIFO_BUFFER_LENGTH] = {0U};
volatile uint16_t Core_ADC_RegularDmaBuffer[CORE_ADC_REGULAR_FIFO_BUFFER_LENGTH] = {0U};
volatile uint32_t Core_ADC_RegularDmaWords[CORE_ADC_REGULAR_FIFO_BUFFER_LENGTH] = {0UL};
volatile uint32_t Core_ADC_LastPublishedWords[CORE_ADC_REGULAR_FIFO_BUFFER_LENGTH] = {0UL};
volatile uint32_t Core_ADC_SequenceCompleteCount = 0U;
volatile uint32_t Core_ADC_OverrunCount = 0U;
volatile uint32_t Core_ADC_LastIsrFlags = 0U;
volatile uint32_t Core_ADC_LastDmaIrqFlags = 0U;
volatile uint32_t Core_ADC_FifoDepthAtEosLast = 0U;
volatile uint32_t Core_ADC_FifoUnderflowCount = 0U;
volatile uint32_t Core_ADC_FifoExtraCount = 0U;
volatile uint32_t Core_ADC_ReadyTimeoutCount = 0U;
volatile uint32_t Core_ADC_StopTimeoutCount = 0U;
volatile uint32_t Core_ADC_StartTimeoutCount = 0U;
volatile uint32_t Core_ADC_DmaTransferCompleteCount = 0U;
volatile uint32_t Core_ADC_DmaTransferErrorCount = 0U;
volatile uint32_t Core_ADC_DmaPublishedCount = 0U;
volatile uint32_t Core_ADC_DmaRawHighBitsCount = 0U;
volatile uint32_t Core_ADC_DmaMaskedOutOfRangeCount = 0U;
volatile uint32_t Core_ADC_ResyncCount = 0U;
volatile uint32_t Core_ADC_ResyncFailureCount = 0U;
volatile uint32_t Core_ADC_DmaDroppedFirstBlockCount = 0U;
volatile uint32_t Core_ADC_DmaTcSinceResync = 0U;
volatile uint32_t Core_ADC_DmaBlockRearmCount = 0U;
volatile uint32_t Core_ADC_DmaBlockRearmFailureCount = 0U;
volatile uint32_t Core_ADC_DmaTeResyncCount = 0U;
volatile uint32_t Core_USART1_TxDmaStartedCount = 0U;
volatile uint32_t Core_USART1_TxDmaCompleteCount = 0U;
volatile uint32_t Core_USART1_TxDmaErrorCount = 0U;
volatile uint32_t Core_USART1_ActualBaudRate = 0U;
volatile uint32_t Core_USART1_BaudErrorPpm = 0U;

typedef enum
{
    CORE_ADC_SCAN_SLOT_PHASE_U = 0U,
    CORE_ADC_SCAN_SLOT_PHASE_V = 1U,
    CORE_ADC_SCAN_SLOT_BUS_CURRENT = 2U,
    CORE_ADC_SCAN_SLOT_VBUS = 3U,
    CORE_ADC_SCAN_SLOT_TEMP0 = 4U,
    CORE_ADC_SCAN_SLOT_COUNT = 5U
} Core_AdcScanSlot;

typedef enum
{
    CORE_ADC_DMA_IRQ_FLAG_TC = (1UL << 0),
    CORE_ADC_DMA_IRQ_FLAG_TE = (1UL << 1)
} Core_AdcDmaIrqFlag;

#if CORE_ENABLE_USART1_TX_DMA
typedef enum
{
    CORE_USART1_TX_DMA_IRQ_NONE = 0U,
    CORE_USART1_TX_DMA_IRQ_COMPLETE = 1U,
    CORE_USART1_TX_DMA_IRQ_ERROR = 2U
} Core_USART1_TxDmaIrqStatus;
#endif

static uint8_t s_tim1_initialized = 0U;
static uint8_t s_adc_initialized = 0U;
static uint8_t s_opa_initialized = 0U;
static uint8_t s_dac0_initialized = 0U;
static uint8_t s_acmp1_initialized = 0U;
static uint8_t s_peripherals_ready = 0U;
static uint32_t s_tim1_pwm_frequency_hz = 0U;
static uint32_t s_tim1_adc_trigger_frequency_hz = 0U;
static uint16_t s_tim1_pwm_period_counts = 0U;
static uint16_t s_tim1_adc_trigger_compare_counts = 0U;
#if CORE_ENABLE_USART1_TX_DMA
static volatile uint8_t s_usart1_tx_dma_busy = 0U;
static uint32_t s_usart1_target_baud_rate = 0U;
static uint8_t s_usart1_gpio_ready = 0U;
static uint8_t s_usart1_uart_ready = 0U;
static uint8_t s_usart1_dma_ready = 0U;
#endif

static uint32_t motorAdcClockMode(void)
{
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

static void updateCorePeripheralsReady(void)
{
    const uint8_t protection_ready = ((s_dac0_initialized != 0U) && (s_acmp1_initialized != 0U))
                                        ? 1U
                                        : 0U;
    s_peripherals_ready = ((s_tim1_initialized != 0U) &&
                           (s_adc_initialized != 0U) &&
                           (s_opa_initialized != 0U) &&
                           (protection_ready != 0U))
                              ? 1U
                              : 0U;
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

static uint8_t waitForAdcIsrFlag(uint32_t flag)
{
    uint32_t timeout = CORE_ADC_WAIT_TIMEOUT_LOOPS;

    while ((ADC->ISR & flag) == 0UL)
    {
        if (timeout == 0UL)
        {
            return 0U;
        }
        --timeout;
    }
    return 1U;
}

static uint8_t waitForAdcCrClear(uint32_t mask)
{
    uint32_t timeout = CORE_ADC_WAIT_TIMEOUT_LOOPS;

    while ((ADC->CR & mask) != 0UL)
    {
        if (timeout == 0UL)
        {
            return 0U;
        }
        --timeout;
    }
    return 1U;
}

static uint8_t coreAdcStopConversionWithTimeout(void)
{
    if ((ADC->CR & ADC_CR_ADSTART_Msk) == 0UL)
    {
        return 1U;
    }

    ADC->CR |= ADC_CR_ADSTP_Msk;
    if (waitForAdcCrClear(ADC_CR_ADSTP_Msk) == 0U)
    {
        ++Core_ADC_StopTimeoutCount;
        return 0U;
    }
    return 1U;
}

static uint8_t coreAdcDisableWithTimeout(void)
{
    if ((ADC->CR & ADC_CR_ADEN_Msk) == 0UL)
    {
        return 1U;
    }

    if (coreAdcStopConversionWithTimeout() == 0U)
    {
        return 0U;
    }
    ADC->CR_b.ADDIS = 1;
    ADC->ISR_b.ADRDY = 1;
    ADC->CFGR1_b.FIFO_CLR = 1;
    if (waitForAdcCrClear(ADC_CR_ADEN_Msk) == 0U)
    {
        ++Core_ADC_StopTimeoutCount;
        return 0U;
    }
    return 1U;
}

static uint8_t coreAdcEnableWithTimeout(void)
{
    ADC->CR_b.ADEN = 1;
    ADC->TESTCFG_b.PD_ADC = 0;
    ADC->TESTCFG_b.PD_REF = 0;
    ADC->TESTCFG_b.PD_SH2 = 0;
    ADC->TESTCFG_b.PD_SH1 = 0;
    if (waitForAdcIsrFlag(ADC_ISR_ADRDY_Msk) == 0U)
    {
        ++Core_ADC_ReadyTimeoutCount;
        return 0U;
    }
    return 1U;
}

static uint8_t coreAdcStartConversionWithTimeout(void)
{
    if ((ADC->ISR & ADC_ISR_ADRDY_Msk) == 0UL)
    {
        if (coreAdcEnableWithTimeout() == 0U)
        {
            ++Core_ADC_StartTimeoutCount;
            return 0U;
        }
    }
    ADC->CR |= ADC_CR_ADSTART_Msk;
    return 1U;
}

static void configureAnalogPinMux(GPIO_TypeDef *gpio_port,
                                  uint16_t gpio_pin,
                                  uint32_t pin_source,
                                  uint32_t enable_an1,
                                  uint32_t enable_an2)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = gpio_pin;
    gpio.GPIO_Mode = GPIO_Mode_AN;
    gpio.GPIO_Speed = GPIO_Speed_Level_1;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gpio_port, &gpio);
    gpio_ana_mode_set(gpio_port, pin_source, enable_an1, enable_an2);
}

static void configureAnalogInputGpio(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    configureAnalogPinMux(GPIOA, GPIO_Pin_0, GPIO_PinSource0, 1U, 0U);
    configureAnalogPinMux(GPIOA, GPIO_Pin_1, GPIO_PinSource1, 1U, 0U);
    configureAnalogPinMux(GPIOA, GPIO_Pin_2, GPIO_PinSource2, 0U, 1U);
    configureAnalogPinMux(GPIOA, GPIO_Pin_3, GPIO_PinSource3, 0U, 1U);
    configureAnalogPinMux(GPIOA, GPIO_Pin_9, GPIO_PinSource9, 0U, 1U);
    configureAnalogPinMux(GPIOA, GPIO_Pin_10, GPIO_PinSource10, 0U, 1U);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
    configureAnalogPinMux(GPIOF, GPIO_Pin_4, GPIO_PinSource4, 0U, 1U);
    configureAnalogPinMux(GPIOF, GPIO_Pin_5, GPIO_PinSource5, 0U, 1U);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    configureAnalogPinMux(GPIOC, GPIO_Pin_12, GPIO_PinSource12, 1U, 1U);
}

static void configureTim1OutputGpio(void)
{
    GPIO_InitTypeDef gpio;

    /* LCA039BK32GU8 内部 LND32B02 预驱绑定：CH1=U、CH2=V、CH3=W。 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_Level_2;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_3); /* TIM1_CH2N -> LIN2 */

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                    GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_Level_2;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_6); /* TIM1_CH3N -> LIN3 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_2); /* TIM1_CH1N -> LIN1 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_3); /* TIM1_CH2  -> HIN2 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_3); /* TIM1_CH3  -> HIN3 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_5); /* TIM1_CH1  -> HIN1 */
}

static void configureAdcIrqProbeGpio(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* PA5 用作临时 ISR 分段耗时探针，默认保持低电平。 */
    GPIOA->BSRR = ((uint32_t)GPIO_Pin_5 << 16U);
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_Level_2;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    GPIOA->BSRR = ((uint32_t)GPIO_Pin_5 << 16U);
}

static void configureOpaUnit(OPA_TypeDef *opa_unit,
                             uint32_t positive_select,
                             uint32_t negative_select,
                             uint32_t bias_select,
                             uint32_t output_select,
                             uint32_t output2_select)
{
    OPA_InitTypeDef opa;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ANACTRL, ENABLE);
    OPA_StructInit(&opa);
    opa.OPA_Oppselect = positive_select;
    opa.OPA_Opnselect = negative_select;
    opa.OPA_Gain = OPA_Gain_20;
    opa.OPA_BiasVoltage = bias_select;
    opa.OPA_BiasVoltageSource = OPA_OPA_BiasVoltageSource_HALFVRH;
    opa.OPA_FeedbackRes = OPA_FeedbackRes_EN;
    opa.OPA_Opoto_Gpio = output_select;
    opa.OPA_Opoto2_Gpio = output2_select;
    opa.OPA_OppRES_short = OPA_OppRES_short_DISEN;
    opa.OPA_OpnRES_short = OPA_OpnRES_short_DISEN;
    OPA_Init(opa_unit, &opa);
    OPA_Cmd(opa_unit, ENABLE);
}

static void configureOpas(void)
{
    configureOpaUnit(OPA0, OPA0_Oppselect_PA2, OPA0_Opnselect_PA3,
                     OPA_BiasVoltage_EN, OPA_OPOEX_EN, OPA_OPOEX2_DISEN);
    configureOpaUnit(OPA1, OPA1_Oppselect_PA9, OPA1_Opnselect_PA10,
                     OPA_BiasVoltage_EN, OPA_OPOEX_EN, OPA_OPOEX2_DISEN);
    configureOpaUnit(OPA2, OPA2_Oppselect_PF4, OPA2_Opnselect_PF5,
                     OPA_BiasVoltage_EN, OPA_OPOEX_DISEN, OPA_OPOEX2_DISEN);
}

static void configureBusOvercurrentDac(void)
{
    DAC_InitTypeDef dac;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ANACTRL, ENABLE);
    DAC_Cmd(DAC0, DISABLE);
    DAC_StructInit(&dac);
    dac.DAC_Trigger_Source = DAC_Trigger_Software;
    dac.DAC_OutputBuffer = DAC0_OutputBuffer_Enable;
    dac.DAC_Vref_Select = CORE_BUS_OC_DAC_VREF_SELECT;
    DAC_Init(DAC0, &dac);
    DAC_Cmd(DAC0, ENABLE);
    DAC_SetDac_10B_Data(DAC0, DAC_Align_10B_R, (uint16_t)CORE_BUS_OC_DAC_CODE);
    DAC_SoftwareTriggerCmd(DAC0, ENABLE);
}

#if CORE_ENABLE_BUS_OVERCURRENT_ACMP
static void configureBusOvercurrentAcmp(void)
{
    ACMP_InitTypeDef acmp;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ANACTRL, ENABLE);
    ACMP1_Cmd(DISABLE);
    ACMP->ACMP1_CSR = 0UL;
    ACMP->ACMP1_MUXCR = 0UL;
    ACMP->ACMP1_EXTCFG = 0UL;

    acmp.ACMP_P_Select = CP1_PS_CPP1EXT_INPUT;
    acmp.ACMP_N_Select = CP1_NS_DAC0OUT;
    acmp.ACMP_Delay_time = CP_NODelay_time;
    acmp.ACMP_Blanking = CP_NOBLANKING;
    acmp.ACMP_Is = CORE_BUS_OC_ACMP_POLARITY;
    acmp.ACMP_HYSEN = DISABLE;
    acmp.ACMP_FREN = DISABLE;
    acmp.ACMP_BLANKING = DISABLE;
    acmp.CMP_INTENMASK = DISABLE;
    acmp.CMP_SEQ_MODE = DISABLE;
    acmp.TRIG_MODE = CP_TIRG_INASOFT;
    acmp.CHNL_CNT_TIME = CHNL_CNT_TIME_64CLK;
    acmp.EXTSELECT = ENABLE_EXTSELG0;
    acmp.EXTEN_Signal_andmode = DISABLE;
    acmp.EXTEN_MODE_SEL = CP_EXTEN_TRIG_DIS;
    acmp.EXTEN_EDGE_MODE = CP_Edge_model;
    acmp.SYN_MODE = CP_SYN_MODE;
    acmp.TRIG_DELAY_TIME = TRIG_DELAY_TIME_128CLK;
    acmp.TRIG_SOURCE = TRIG_TIM1_CC1;
    acmp.CP_EXTSP = CP_EXTSP_POS;

    ACMP1_Init(&acmp);
    /* P_AMUX_CFG=6 对应 ACMP1 CPP16，即 PC12/OPA2 OP2O。 */
    ACMP1_SEQ_INPUT_SELECT(DISABLE, 6U, 0U);
    SATRT_OFACMP1();
    ACMP1_Cmd(ENABLE);
}

static void configureTim1BreakInputFromAcmp1(void)
{
    TIM1_BreakSourceCfg(TIM1_Break1Source_ACMP1_OUTPUT,
                        CORE_BUS_OC_TIM1_BREAK_LEVEL);
}
#endif

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

static void stopAdcDma(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);
    NVIC_DisableIRQ(DMAC_CH1_2_IRQn);
    DMA_ITConfig(DMA1_Channel1, DMA1_IT_TC1, DISABLE);
    DMA1->MaskErr = CORE_ADC_DMA_CHANNEL_IRQ_DISABLE_WRITE;
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    DMA_ClearITPendingBit(DMA1_IT_TE1);
    NVIC_ClearPendingIRQ(DMAC_CH1_2_IRQn);
}

static uint8_t stopAdc(void)
{
    uint8_t ok = 1U;

    NVIC_DisableIRQ(ADC_IRQn);
    stopAdcDma();
    ADC_ITConfig(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS, DISABLE);
    if ((ADC->CR & ADC_CR_ADSTART_Msk) != 0UL)
    {
        if (coreAdcStopConversionWithTimeout() == 0U)
        {
            ok = 0U;
        }
    }
    if ((ADC->CR & ADC_CR_ADEN_Msk) != 0UL)
    {
        if (coreAdcDisableWithTimeout() == 0U)
        {
            ok = 0U;
        }
    }
    ADC_ClearITPendingBit(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS);
    ADC->CFGR1_b.FIFO_CLR = 1;
    NVIC_ClearPendingIRQ(ADC_IRQn);
    return ok;
}

static void configureAdcRegularScanChannels(void)
{
    uint8_t channels[16] = {0U};

    channels[CORE_ADC_SCAN_SLOT_PHASE_U] = ADCIN_11_B;
    channels[CORE_ADC_SCAN_SLOT_PHASE_V] = ADCIN_12_B;
    channels[CORE_ADC_SCAN_SLOT_BUS_CURRENT] = ADCIN_14_B;
    channels[CORE_ADC_SCAN_SLOT_VBUS] = ADCIN_1_A;
    channels[CORE_ADC_SCAN_SLOT_TEMP0] = ADCIN_0_A;
    /* LC32 CHNUM stores the last sequence index, not the channel count. */
    ADC_ChannelConfig((uint32_t)(CORE_ADC_SCAN_SLOT_COUNT - 1U), channels);
}

static void resetAdcRegularRuntime(void)
{
    uint8_t i;

    Core_ADC_PhaseURaw = 0U;
    Core_ADC_PhaseVRaw = 0U;
    Core_ADC_BusCurrentRaw = 0U;
    Core_ADC_VbusRaw = 0U;
    Core_ADC_TempRaw0 = 0U;
    Core_ADC_SequenceCompleteCount = 0U;
    Core_ADC_OverrunCount = 0U;
    Core_ADC_LastIsrFlags = 0U;
    Core_ADC_LastDmaIrqFlags = 0U;
    Core_ADC_FifoDepthAtEosLast = 0U;
    Core_ADC_FifoUnderflowCount = 0U;
    Core_ADC_FifoExtraCount = 0U;
    Core_ADC_DmaTransferCompleteCount = 0U;
    Core_ADC_DmaTransferErrorCount = 0U;
    Core_ADC_DmaPublishedCount = 0U;
    Core_ADC_DmaRawHighBitsCount = 0U;
    Core_ADC_DmaMaskedOutOfRangeCount = 0U;
    Core_ADC_DmaTcSinceResync = 0U;
    Core_ADC_DmaBlockRearmCount = 0U;
    Core_ADC_DmaBlockRearmFailureCount = 0U;
    Core_ADC_DmaTeResyncCount = 0U;
    for (i = 0U; i < CORE_ADC_REGULAR_FIFO_BUFFER_LENGTH; ++i)
    {
        Core_ADC_RegularFifoBuffer[i] = 0U;
        Core_ADC_RegularDmaBuffer[i] = 0U;
        Core_ADC_RegularDmaWords[i] = 0UL;
        Core_ADC_LastPublishedWords[i] = 0UL;
    }
}

static uint8_t configureAdc(uint32_t trigger, uint32_t edge)
{
    ADC_InitTypeDef init;

    (void)stopAdc();
    ADC_DeInit(ADC);
    resetAdcRegularRuntime();
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
    init.ADC_ConvertTime = 15UL;
    init.ADC_ConvertPulseTime = 0UL;
    if (coreAdcEnableWithTimeout() == 0U)
    {
        ADC_ITConfig(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS, DISABLE);
        stopAdcDma();
        NVIC_DisableIRQ(ADC_IRQn);
        NVIC_ClearPendingIRQ(ADC_IRQn);
        return 0U;
    }
    if ((ADC->CR & ADC_CR_ADSTART_Msk) != 0UL)
    {
        ADC_ITConfig(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS, DISABLE);
        stopAdcDma();
        NVIC_DisableIRQ(ADC_IRQn);
        NVIC_ClearPendingIRQ(ADC_IRQn);
        ++Core_ADC_StartTimeoutCount;
        return 0U;
    }

    ADC_Init(ADC, &init);
    configureAdcRegularScanChannels();
    ADC->CFGR1_b.FIFO_CLR = 1;
    ADC_ClearITPendingBit(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS);
    ADC_ITConfig(ADC, ADC_IT_EOC | CORE_ADC_FIFO_STATUS_FLAGS, DISABLE);
    ADC_ClearITPendingBit(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS);
    ADC_ITConfig(ADC, ADC_IT_EOS, ENABLE);
    NVIC_ClearPendingIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1U);
    NVIC_EnableIRQ(ADC_IRQn);
    return 1U;
}

static void disablePowerChannels(void)
{
    /*
     * 只关闭三相功率桥输出，不能停止 TIM1 counter、CH4 或 ADC；
     * Break 过流后仍依赖 CH4 维持采样 tick 和后台遥测。
     */
    LCA039_PWM_Disable(TIM1);
}

static void enablePowerChannels(void)
{
    LCA039_PWM_Enable(TIM1);
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

static uint32_t coreTim1BreakPolarity(void)
{
    return (CORE_BUS_OC_TIM1_BREAK_LEVEL == TIM_InputLevel_Low)
               ? TIM_BreakPolarity_Low
               : TIM_BreakPolarity_High;
}

static void configureTim1BreakAndDeadTime(void)
{
    TIM_BDTRInitTypeDef bdtr;

    TIM_BDTRStructInit(&bdtr);
    bdtr.TIM_DeadTime = (uint32_t)CORE_TIM1_DEADTIME_TICKS;
    bdtr.TIM_Break1 = TIM_Break_Enable;
    bdtr.TIM_Break1Polarity = coreTim1BreakPolarity();
    bdtr.TIM_AutomaticOutput1 = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &bdtr);
}

static void armTim1BreakInterrupt(void)
{
    /* Break 源可能在阈值占位或上电偏置阶段已有效，先清旧标志再打开一次性锁存中断。 */
    TIM_ITConfig(TIM1, TIM_IT_Break, DISABLE);
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    TIM_ClearFlag(TIM1, TIM_FLAG_Break2);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    NVIC_ClearPendingIRQ(TIM1_NON_CC_IRQn);
    NVIC_SetPriority(TIM1_NON_CC_IRQn, 0U);
    TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
    NVIC_EnableIRQ(TIM1_NON_CC_IRQn);
}

static uint8_t configureCenterAlignedTriggerTimer(void)
{
    const uint32_t tim1_pwm_frequency_hz = 12000UL;
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
    adc_trigger_compare = arr - 1UL;

    s_tim1_pwm_frequency_hz = tim1_pwm_frequency_hz;
    s_tim1_adc_trigger_frequency_hz = tim1_pwm_frequency_hz;
    s_tim1_pwm_period_counts = (uint16_t)period_counts;
    s_tim1_adc_trigger_compare_counts = (uint16_t)adc_trigger_compare;

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
    outputCompare.TIM_OutputNState = TIM_OutputNState_Enable;
    outputCompare.TIM_Pulse = 0U;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
    outputCompare.TIM_OCNPolarity = TIM_OCNPolarity_High;
    outputCompare.TIM_OCIdleState = TIM_OCIdleState_Reset;
    outputCompare.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &outputCompare);
    TIM_OC2Init(TIM1, &outputCompare);
    TIM_OC3Init(TIM1, &outputCompare);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OCStructInit(&outputCompare);
    outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputCompare.TIM_OutputState = TIM_OutputState_Enable;
    outputCompare.TIM_OutputNState = TIM_OutputNState_Disable;
    outputCompare.TIM_Pulse = adc_trigger_compare;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
    outputCompare.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC4Init(TIM1, &outputCompare);
    /* CH4 固定为每个 PWM 周期一次 ADC 触发，不再由 ADC ISR 改写 CCR。 */
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);
    configureTim1BreakAndDeadTime();
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_Break, DISABLE);
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    TIM_ClearFlag(TIM1, TIM_FLAG_Break2);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    NVIC_ClearPendingIRQ(TIM1_NON_CC_IRQn);
    disablePowerChannels();

    /* TIM1 CH4 只作为 ADC 触发通道；CH1/CH2/CH3 及互补功率输出默认保持关闭。 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    return 1U;
}
static void stopTimer(void)
{
    if (s_tim1_initialized == 0U) return;
    TIM_Cmd(TIM1, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_Break, DISABLE);
    NVIC_DisableIRQ(TIM1_NON_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM1_NON_CC_IRQn);
    disablePowerChannels();
}

static uint8_t startTimerAdcPath(void)
{
    /* TIM1 CH4 上升沿触发 ADC，启动采样链路时仍不打开三相功率通道。 */
    stopTimer();
    if (configureAdc(ADC_ExternalTrigConv_T1_CC4, ADC_ExternalTrigConvEdge_Rising) == 0U)
    {
        return 0U;
    }
    if (configureCenterAlignedTriggerTimer() == 0U) return 0U;
    if (coreAdcStartConversionWithTimeout() == 0U)
    {
        return 0U;
    }
    TIM_Cmd(TIM1, ENABLE);
    armTim1BreakInterrupt();
    return 1U;
}

static uint32_t adcFifoDepthFromIsr(uint32_t isr)
{
    return (isr & CORE_ADC_FIFO_DEPTH_MASK) >> CORE_ADC_FIFO_DEPTH_POS;
}

static void mirrorAdcRegularFifoBuffer(void)
{
    uint8_t i;

    for (i = 0U; i < CORE_ADC_REGULAR_FIFO_BUFFER_LENGTH; ++i)
    {
        Core_ADC_RegularDmaBuffer[i] = Core_ADC_RegularFifoBuffer[i];
    }
}

static void publishAdcRegularFifoBuffer(void)
{
    const uint16_t phase_u_raw = Core_ADC_RegularFifoBuffer[CORE_ADC_SCAN_SLOT_PHASE_U];
    const uint16_t phase_v_raw = Core_ADC_RegularFifoBuffer[CORE_ADC_SCAN_SLOT_PHASE_V];
    const uint16_t bus_current_raw =
        Core_ADC_RegularFifoBuffer[CORE_ADC_SCAN_SLOT_BUS_CURRENT];
    const uint16_t vbus_raw = Core_ADC_RegularFifoBuffer[CORE_ADC_SCAN_SLOT_VBUS];
    const uint16_t temp_raw = Core_ADC_RegularFifoBuffer[CORE_ADC_SCAN_SLOT_TEMP0];

    Core_ADC_PhaseURaw = phase_u_raw;
    Core_ADC_PhaseVRaw = phase_v_raw;
    Core_ADC_BusCurrentRaw = bus_current_raw;
    Core_ADC_VbusRaw = vbus_raw;
    Core_ADC_TempRaw0 = temp_raw;
    Core_ADC_LastRaw = temp_raw;
    mirrorAdcRegularFifoBuffer();
    Core_ADC_PublishedSampleCount += CORE_ADC_SCAN_SLOT_COUNT;
    ++Core_ADC_SequenceCompleteCount;
}

static uint16_t adcFifoWordToRaw(uint32_t word)
{
    return (uint16_t)(word & CORE_ADC_DATA_MASK);
}

static void captureAdcRegularFifoWords(void)
{
    uint8_t i;

    for (i = 0U; i < CORE_ADC_SCAN_SLOT_COUNT; ++i)
    {
        const uint32_t word = ADC->DR;
        const uint16_t raw = adcFifoWordToRaw(word);

        if ((word & ~CORE_ADC_DATA_MASK) != 0UL)
        {
            ++Core_ADC_DmaRawHighBitsCount;
        }
        if (raw > (uint16_t)CORE_ADC_DATA_MASK)
        {
            ++Core_ADC_DmaMaskedOutOfRangeCount;
        }

        Core_ADC_RegularFifoBuffer[i] = raw;
        Core_ADC_RegularDmaBuffer[i] = raw;
        Core_ADC_RegularDmaWords[i] = word;
        Core_ADC_LastPublishedWords[i] = word;
    }
}

static uint8_t acknowledgeTimerUpdate(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == RESET) return 0U;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    return 1U;
}

void MX_GPIO_Init(void)
{
    configureAnalogInputGpio();
    configureTim1OutputGpio();
    configureAdcIrqProbeGpio();
#if CORE_ENABLE_USART1_TX_DMA
    /* USART1 引脚和模拟/TIM1 引脚一样，统一在 GPIO 初始化入口完成复用配置。 */
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

void MX_OPA_Init(void)
{
    configureOpas();
    s_opa_initialized = 1U;
    updateCorePeripheralsReady();
}

void MX_DAC_Init(void)
{
    configureBusOvercurrentDac();
    s_dac0_initialized = 1U;
    updateCorePeripheralsReady();
}

void MX_ACMP_Init(void)
{
#if CORE_ENABLE_BUS_OVERCURRENT_ACMP
    configureBusOvercurrentAcmp();
    configureTim1BreakInputFromAcmp1();
#else
    /* Diagnostic build: keep ACMP1 detached from OPA2/PC12 and TIM1 Break. */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ANACTRL, ENABLE);
    ACMP1_Cmd(DISABLE);
    ACMP->ACMP1_CSR = 0UL;
    ACMP->ACMP1_MUXCR = 0UL;
    ACMP->ACMP1_EXTCFG = 0UL;
#endif
    s_acmp1_initialized = 1U;
    updateCorePeripheralsReady();
}

void MX_TIM1_Init(void)
{
    /* MX_TIM1_Init 只配置 TIM1 计数、CH4 ADC 触发和安全默认态；功率输出不会在这里启动。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM1, ENABLE);
    s_tim1_initialized = 0U;
    if (configureCenterAlignedTriggerTimer() != 0U)
    {
        s_tim1_initialized = 1U;
        TIM_Cmd(TIM1, DISABLE);
        Core_TIM1_DisablePwmOutputs();
    }
    updateCorePeripheralsReady();
}

void MX_ADC_Init(void)
{
    /* MX_ADC_Init 配置 ADC 时钟、触发源和 EOS/FIFO 发布路径。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC, ENABLE);
    if (configureAdc(ADC_ExternalTrigConv_SoftWare, ADC_ExternalTrigConvEdge_None) != 0U)
    {
        s_adc_initialized = 1U;
    }
    else
    {
        s_adc_initialized = 0U;
        g_boot_status = (LCA039_BootStatus)(g_boot_status | LCA039_BOOT_STATUS_ADC_INIT_FAILED);
    }
    updateCorePeripheralsReady();
}

bool Core_PeripheralsReady(void)
{
    return s_peripherals_ready != 0U;
}

bool Core_PowerBridgeBindingEnabled(void)
{
    return true;
}

bool Core_ADC_HandleConversionIrq(void)
{
    const uint32_t isr = ADC->ISR;
    const uint32_t clear_flags =
        isr & (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_EOSMP | CORE_ADC_FIFO_STATUS_FLAGS);
    const uint32_t fifo_depth = adcFifoDepthFromIsr(isr);

    Core_ADC_LastIsrFlags = isr;

    if ((isr & ADC_IT_EOS) == 0UL)
    {
        if (clear_flags != 0UL)
        {
            ADC_ClearITPendingBit(ADC, clear_flags);
        }
        ++Core_ADC_UnexpectedIrqCount;
        return false;
    }

    Core_ADC_FifoDepthAtEosLast = fifo_depth;
    if (fifo_depth < CORE_ADC_SCAN_SLOT_COUNT)
    {
        ++Core_ADC_FifoUnderflowCount;
        ++Core_ADC_OverrunCount;
        ADC->CFGR1_b.FIFO_CLR = 1;
        ADC_ClearITPendingBit(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS);
        return false;
    }
    if (fifo_depth > CORE_ADC_SCAN_SLOT_COUNT)
    {
        ++Core_ADC_FifoExtraCount;
        ++Core_ADC_OverrunCount;
        ADC->CFGR1_b.FIFO_CLR = 1;
        ADC_ClearITPendingBit(ADC, ADC_IT_EOC | ADC_IT_EOS | CORE_ADC_FIFO_STATUS_FLAGS);
        return false;
    }

    captureAdcRegularFifoWords();
    publishAdcRegularFifoBuffer();
    if (clear_flags != 0UL)
    {
        ADC_ClearITPendingBit(ADC, clear_flags);
    }
    return true;
}

bool Core_ADC_HandleDmaIrq(void)
{
    const uint8_t transfer_error =
        ((DMA_GetITStatus(DMA1_IT_TE1) == SET) ||
         (DMA_GetFlagStatus(DMA1_FLAG_TE1) == SET))
            ? 1U
            : 0U;
    const uint8_t transfer_complete =
        ((DMA_GetITStatus(DMA1_IT_TC1) == SET) ||
         (DMA_GetFlagStatus(DMA1_FLAG_TC1) == SET))
            ? 1U
            : 0U;
    uint32_t dma_irq_flags = 0UL;

    if (transfer_complete != 0U)
    {
        dma_irq_flags |= CORE_ADC_DMA_IRQ_FLAG_TC;
    }
    if (transfer_error != 0U)
    {
        dma_irq_flags |= CORE_ADC_DMA_IRQ_FLAG_TE;
    }
    Core_ADC_LastDmaIrqFlags = dma_irq_flags;
    DMA_Cmd(DMA1_Channel1, DISABLE);
    if (transfer_error != 0U)
    {
        ++Core_ADC_DmaTransferErrorCount;
        ++Core_ADC_DmaTeResyncCount;
    }
    if (transfer_complete != 0U)
    {
        ++Core_ADC_DmaTransferCompleteCount;
    }
    DMA_ClearITPendingBit(DMA1_IT_TE1);
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    if (dma_irq_flags != 0UL)
    {
        ++Core_ADC_UnexpectedIrqCount;
    }
    return false;
}

void Core_TIM1_HandleUpdateIrq(void)
{
    if (acknowledgeTimerUpdate() != 0U)
    {
        ++Core_TIM1_UpdateIrqCount;
    }
}

uint16_t Core_ADC_GetLastRaw(void)
{
    return Core_ADC_LastRaw;
}

uint16_t Core_ADC_GetPhaseURaw(void)
{
    return Core_ADC_PhaseURaw;
}

uint16_t Core_ADC_GetPhaseVRaw(void)
{
    return Core_ADC_PhaseVRaw;
}

uint16_t Core_ADC_GetBusCurrentRaw(void)
{
    return Core_ADC_BusCurrentRaw;
}

uint16_t Core_ADC_GetVbusRaw(void)
{
    return Core_ADC_VbusRaw;
}

uint16_t Core_ADC_GetTemp0Raw(void)
{
    return Core_ADC_TempRaw0;
}

void Core_TIM1_DisablePwmOutputs(void)
{
    if (s_tim1_initialized != 0U)
    {
        disablePowerChannels();
    }
}

void Core_TIM1_EnablePwmOutputs(void)
{
    if (s_tim1_initialized != 0U)
    {
        enablePowerChannels();
    }
}

void Core_TIM1_SetPwmCompare(uint16_t phaseU, uint16_t phaseV, uint16_t phaseW)
{
    /* Core 只写 TIM1 比较寄存器，duty 计算和桥臂策略由 BSP/Platform/Lib 决定。 */
    if (s_tim1_initialized == 0U) return;
    TIM_SetCompare1(TIM1, phaseU);
    TIM_SetCompare2(TIM1, phaseV);
    TIM_SetCompare3(TIM1, phaseW);
}

void Core_TIM1_BrakeLowOutputs(void)
{
    if (s_tim1_initialized == 0U) return;
    LCA039_PWM_BrakeLow(TIM1, Core_PeripheralsReady() && Core_PowerBridgeBindingEnabled());
}

bool Core_TIM1_PwmOutputsEnabled(void)
{
    const uint32_t powerChannelMask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                      TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                      TIM_CCER_CC3E | TIM_CCER_CC3NE;
    if (s_tim1_initialized == 0U) return false;
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
bool Core_SysTick_ConfigureHz(uint32_t frequencyHz)
{
    if ((frequencyHz == 0UL) || (SystemCoreClock < frequencyHz)) return false;
    return SysTick_Config(SystemCoreClock / frequencyHz) == 0UL;
}

bool Core_TIM1_StartUpdateInterruptHz(uint32_t frequencyHz)
{
    if (s_peripherals_ready == 0U) return false;
    stopTimer();
    return configureUpdateTimer(frequencyHz) != 0U;
}

bool Core_ADC_SelectSoftwareTrigger(void)
{
    if (s_peripherals_ready == 0U) return false;
    stopTimer();
    return configureAdc(ADC_ExternalTrigConv_SoftWare, ADC_ExternalTrigConvEdge_None) != 0U;
}

bool Core_ADC_StartSoftwareConversion(void)
{
    if (s_peripherals_ready == 0U) return false;
    if ((ADC->CR & ADC_CR_ADSTART_Msk) != 0UL) return false;
    return coreAdcStartConversionWithTimeout() != 0U;
}

bool Core_TIM1_StartAdcTriggerPath(void)
{
    if (s_peripherals_ready == 0U) return false;
    return startTimerAdcPath() != 0U;
}

bool Core_TIM1_ResyncAdcTriggerPath(void)
{
    uint8_t ok;

    if (s_peripherals_ready == 0U)
    {
        ++Core_ADC_ResyncFailureCount;
        return false;
    }

    ok = startTimerAdcPath();
    if (ok != 0U)
    {
        ++Core_ADC_ResyncCount;
        return true;
    }

    ++Core_ADC_ResyncFailureCount;
    return false;
}

void Core_TIM1_Stop(void)
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
    MX_OPA_Init();
    MX_DAC_Init();
    MX_ACMP_Init();
    MX_TIM1_Init();
    MX_ADC_Init();

    User_Init();

    for (;;)
    {
        ++g_heartbeat;
        User_Loop();
    }
}
