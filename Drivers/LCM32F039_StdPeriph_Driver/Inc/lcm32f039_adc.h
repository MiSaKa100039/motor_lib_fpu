#ifndef LCM32F039_ADC_H
#define LCM32F039_ADC_H
#include "lcm32f039.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /*--------------------------------------------------------------------------------------------
   adc 12parameters cfg:
  --------------------------------------------------------------------------------------------*/
  typedef struct
  {

    uint32_t ADC_ClkMode; /*  ADC 时钟分频/ADC clock frequency division */

    uint32_t ADC_Resolution; /*  adc采样数据位宽设置 此型号暂时只支持12bit/Adc sampling data bit width setting This model supports only 12 bits for the time being  */

    FunctionalState ADC_AutoWatiMode; /*  自动延迟转换功能删除，默认为DISABLE,禁止配置为ENABLE*/

    FunctionalState ADC_DMA; /*  DMA  */

    FunctionalState ADC_DiscMode; /*  续模式/Discontinuous Conduction mode  */

    FunctionalState ADC_SamecMode; /*  同时采样/simultaneous sampling  */

    uint32_t ADC_ExternalTrigConvEdge; /*   触发边沿or电平设置
                                                 < Selects the external trigger Edge and enables the
                                                 trigger of a regular group. This parameter can be a value
                                                 of @ref ADC_external_trigger_edge_conversion */

    uint32_t ADC_ExternalTrigConv; /*   外部触发源选择
                                                 !< Defines the external trigger used to start the analog
                                                 to digital conversion of regular channels. This parameter
                                                 can be a value of @ref ADC_external_trigger_sources_for_channels_conversion */

    uint32_t ADC_DataAlign;     /*   数据对齐方向
                                                 !< Specifies whether the ADC data alignment is left or right.
                                                 This parameter can be a value of @ref ADC_data_align */
    uint32_t ADC_DMATRIG_LEVEL; /*  置触发 DMA 请求的 FIFO 水平。在 DMAEN 使能前进行配置/Set the FIFO level that triggers the DMA request. This parameter must be configured before DMAEN is enabled  */

    uint32_t ADC_ScanDirection; /*   通道扫描方向设置
                                                 !< Specifies in which direction the channels will be scanned
                                                 in the sequence.
                                                 This parameter can be a value of @ref ADC_Scan_Direction */
    uint32_t ADC_Vref;          /*	ADC-vref select		*/

    uint32_t ADC_SampleTime;
    uint32_t ADC_SampleDelayTime;
    uint32_t ADC_AdjustTime;
    uint32_t ADC_ConvertTime;
    uint32_t ADC_ConvertPulseTime;
  } ADC_InitTypeDef;

/*--------------------------------------------------------------------------------------------
 ADC_CLK_MODE SELECT: ADCCLK<=24mhz ;
 Going out of range can lead to unpredictable errors;
  --------------------------------------------------------------------------------------------*/
//同步时钟 sysclk
#define ADC_ClockMode_sysnClk ((uint32_t)0x00000000) /*!<  与系统时钟同步/Synchronize with the system clock */
#define ADC_ClockMode_SynClkDiv2 (0X01 << 27)        /*!<  Synchronous clock mode divided by 2 */
#define ADC_ClockMode_SynClkDiv4 (0X02 << 27)        /*!<  Synchronous clock mode divided by 4 */
#define ADC_ClockMode_SynClkDiv6 (0X03 << 27)        /*!<  Synchronous clock mode divided by 6 */
#define ADC_ClockMode_SynClkDiv8 (0X04 << 27)        /*!<  Synchronous clock mode divided by 8 */
#define ADC_ClockMode_SynClkDiv10 (0X05 << 27)       /*!<  Synchronous clock mode divided by 10 */
#define ADC_ClockMode_SynClkDiv12 (0X06 << 27)       /*!<  Synchronous clock mode divided by 12 */
#define ADC_ClockMode_SynClkDiv16 (0X07 << 27)       /*!<  Synchronous clock mode divided by 16 */
//异步时钟 pllclk
#define ADC_ClockMode_PLLClk ((uint32_t)0x80000000) /*!< 从pll时钟来，属于异步时钟/From the PLL clock, belongs to the asynchronous clock */
#define ADC_ClockMode_PLLClkDiv2 (0X11 << 27)       /*!<  PLL clock mode divided by 2 */
#define ADC_ClockMode_PLLClkDiv4 (0X12 << 27)       /*!<  PLL clock mode divided by 4 */
#define ADC_ClockMode_PLLClkDiv6 (0X13 << 27)       /*!<  PLL clock mode divided by 6 */
#define ADC_ClockMode_PLLClkDiv8 (0X14 << 27)       /*!<  PLL clock mode divided by 8 */
#define ADC_ClockMode_PLLClkDiv10 (0X15 << 27)      /*!<  PLL clock mode divided by 10 */
#define ADC_ClockMode_PLLClkDiv12 (0X16 << 27)      /*!<  PLL clock mode divided by 12 */
#define ADC_ClockMode_PLLClkDiv16 (0X17 << 27)      /*!<  PLL clock mode divided by 16 */
#define ADC_ClockMode_PLLClkDiv32 (0X18 << 27)      /*!<  PLL clock mode divided by 32 */
#define ADC_ClockMode_PLLClkDiv64 (0X19 << 27)      /*!<  PLL clock mode divided by 64 */
#define ADC_ClockMode_PLLClkDiv128 (0X1A << 27)     /*!<  PLL clock mode divided by 128 */
#define ADC_ClockMode_PLLClkDiv256 (0X1B << 27)     /*!<  PLL clock mode divided by 256 */

/*--------------------------------------------------------------------------------------------
RESLUTION BIT:just 12bit
  --------------------------------------------------------------------------------------------*/
#define ADC_Resolution_12b ((uint32_t)0x00000000)
/****************** RESLUTION BIT **************/
#define ADC_DataAlign_Right ((uint32_t)0x00000000)
#define ADC_DataAlign_Left ((uint32_t)0x00000020)

/*--------------------------------------------------------------------------------------------
scan align  :
  --------------------------------------------------------------------------------------------*/
#define ADC_ScanDirection_Upward ((uint32_t)0x00000000)
#define ADC_ScanDirection_Backward (0x1UL << 2)

/*--------------------------------------------------------------------------------------------
EXT_TRIG_EDGE :
  --------------------------------------------------------------------------------------------*/
#define ADC_ExternalTrigConvEdge_None ((uint32_t)0x00000000)
#define ADC_ExternalTrigConvEdge_Rising (1 << 0)
#define ADC_ExternalTrigConvEdge_Falling (2 << 0)
#define ADC_ExternalTrigConvEdge_RisingFalling (3 << 0)

/*--------------------------------------------------------------------------------------------
EXT_TRIG_SOURCE :
  --------------------------------------------------------------------------------------------*/
#define ADC_ExternalTrigConv_SoftWare ((uint32_t)0x00000000)
/* 5 */
#define ADC_ExternalTrigConv_T1_TRGO ((uint32_t)0x80000000)
#define ADC_ExternalTrigConv_T2_TRGO ((uint32_t)0x80100000)
#define ADC_ExternalTrigConv_T15_TRGO ((uint32_t)0x80200000)
#define ADC_ExternalTrigConv_T3_TRGO ((uint32_t)0x80300000)
/* 4 */
#define ADC_ExternalTrigConv_T1_CC4 ((uint32_t)0x40000000)
#define ADC_ExternalTrigConv_T2_CC4 ((uint32_t)0x40040000)
#define ADC_ExternalTrigConv_T17_CC1 ((uint32_t)0x40080000)
#define ADC_ExternalTrigConv_ACMP0 ((uint32_t)0x400C0000)
/* 3 */
#define ADC_ExternalTrigConv_T1_CC3 ((uint32_t)0x20000000)
#define ADC_ExternalTrigConv_T2_CC3 ((uint32_t)0x20010000)
#define ADC_ExternalTrigConv_T16_CC1 ((uint32_t)0x20020000)
#define ADC_ExternalTrigConv_EXTI11 ((uint32_t)0x20030000)
/* 2 */
#define ADC_ExternalTrigConv_T1_CC2 ((uint32_t)0x10000000)
#define ADC_ExternalTrigConv_T2_CC2 ((uint32_t)0x10004000)
#define ADC_ExternalTrigConv_T1_CC5 ((uint32_t)0x10008000)
#define ADC_ExternalTrigConv_EXTI3 ((uint32_t)0x1000C000)
/* 1 */
#define ADC_ExternalTrigConv_T1_CC1 ((uint32_t)0x08000000)
#define ADC_ExternalTrigConv_T2_CC1 ((uint32_t)0x08001000)
#define ADC_ExternalTrigConv_T15_CC1 ((uint32_t)0x08002000)
#define ADC_ExternalTrigConv_EXTI1 ((uint32_t)0x08003000)

  /*--------------------------------------------------------------------------------------------
atchdog_Channel : SELECT
  --------------------------------------------------------------------------------------------*/
#define ADC_AnalogWatchdog_Channel_0 ((uint32_t)0x00000000)
#define ADC_AnalogWatchdog_Channel_1 ((uint32_t)0x08000000)
#define ADC_AnalogWatchdog_Channel_2 ((uint32_t)0x10000000)
#define ADC_AnalogWatchdog_Channel_3 ((uint32_t)0x18000000)
#define ADC_AnalogWatchdog_Channel_4 ((uint32_t)0x20000000)
#define ADC_AnalogWatchdog_Channel_5 ((uint32_t)0x21000000)
#define ADC_AnalogWatchdog_Channel_6 ((uint32_t)0x30000000)
#define ADC_AnalogWatchdog_Channel_7 ((uint32_t)0x38000000)
#define ADC_AnalogWatchdog_Channel_8 ((uint32_t)0x40000000)
#define ADC_AnalogWatchdog_Channel_9 ((uint32_t)0x48000000)
#define ADC_AnalogWatchdog_Channel_10 ((uint32_t)0x50000000)
#define ADC_AnalogWatchdog_Channel_11 ((uint32_t)0x58000000)
#define ADC_AnalogWatchdog_Channel_12 ((uint32_t)0x60000000)
#define ADC_AnalogWatchdog_Channel_13 ((uint32_t)0x68000000)
#define ADC_AnalogWatchdog_Channel_14 ((uint32_t)0x70000000)
#define ADC_AnalogWatchdog_Channel_15 ((uint32_t)0x78000000)

#define ADC_IT_ADRDY ((uint32_t)0x00000001)
#define ADC_IT_EOSMP ((uint32_t)0x00000002)
#define ADC_IT_EOC ((uint32_t)0x00000004)
#define ADC_IT_EOS ((uint32_t)0x00000008)
#define ADC_IT_AWD ((uint32_t)0x00000080)
#define ADC_IT_F_EMPTY ((uint32_t)0x00000100)
#define ADC_IT_F_ALEMPTY ((uint32_t)0x00000200)
#define ADC_IT_F_HALF ((uint32_t)0x00000400)
#define ADC_IT_F_ALF ((uint32_t)0x00000800)
#define ADC_IT_F_FULL ((uint32_t)0x00001000)

#define IS_ADC_IT(ADC_IT) ((ADC_IT == ADC_IT_ADRDY) ||     \
                           (ADC_IT == ADC_IT_EOSMP) ||     \
                           (ADC_IT == ADC_IT_EOC) ||       \
                           (ADC_IT == ADC_IT_EOS) ||       \
                           (ADC_IT == ADC_IT_AWD) ||       \
                           (ADC_IT == ADC_IT_F_EMPTY) ||   \
                           (ADC_IT == ADC_IT_F_ALEMPTY) || \
                           (ADC_IT == ADC_IT_F_HALF) ||    \
                           (ADC_IT == ADC_IT_F_ALF) ||     \
                           (ADC_IT == ADC_IT_F_FULL))

#define FIFO_DMA_NOEMPTY ((uint32_t)0x00000000)
#define FIFO_DMA_4LEVEL ((uint32_t)0x00000008)
#define FIFO_DMA_8LEVEL ((uint32_t)0x00000010)
#define FIFO_DMA_12LEVEL ((uint32_t)0x00000018)
  /*--------------------------------------------------------------------------------------------
adc-samptime select
  --------------------------------------------------------------------------------------------*/
#define ADC_SampleTime_1Cycles ((uint32_t)0x00000000)
#define ADC_SampleTime_2Cycles ((uint32_t)0x00000001)
#define ADC_SampleTime_3Cycles ((uint32_t)0x00000002)
#define ADC_SampleTime_4Cycles ((uint32_t)0x00000003)
#define ADC_SampleTime_5Cycles ((uint32_t)0x00000004)
#define ADC_SampleTime_6Cycles ((uint32_t)0x00000005)
#define ADC_SampleTime_7Cycles ((uint32_t)0x00000006)
#define ADC_SampleTime_8Cycles ((uint32_t)0x00000007)

#define ADC_ConverTime_10Cycles ((uint32_t)0x00000570)
#define ADC_ConverTime_15Cycles ((uint32_t)0x000007f0)
#define ADC_ConverTime_20Cycles ((uint32_t)0x00000A70)

  /*--------------------------------------------------------------------------------------------
ADC-vref select
  --------------------------------------------------------------------------------------------*/
#define ADC_Vref_Internal_3V3 ((uint32_t)0x00000001)
#define ADC_Vref_Internal_3V3_data ((uint32_t)0x00000000)
#define ADC_Vref_Internal_5V ((uint32_t)0x00000002)
#define ADC_Vref_Internal_5V_data ((uint32_t)0x00000008)

#define ADC_Vref_Externa_Vrh2V5 ((uint32_t)0x00000003)
#define ADC_Vref_Externa_Vrh2V5_data (0x4UL << 16)

#define ADC_Vref_Externa_Vrh4V ((uint32_t)0x00000004)
#define ADC_Vref_Externa_Vrh4V_data (0x6UL << 16)

#define ADC_Vref_Externa_VDD ((uint32_t)0x00000005)
#define ADC_Vref_Externa_VDD_data (0x0UL << 16)

#define ADC_Vref_Externa_IO ((uint32_t)0x00000006)
#define ADC_Vref_Externa_IO_data (0x2UL << 18)

#define ADC_Vref_ExternaIO_VRH_C ((uint32_t)0x00000007)
#define ADC_Vref_ExternaIO_VRH_C_data (0x3UL << 18)
/*--------------------------------------------------------------------------------------------
CHA ADCIN-Channel select
--------------------------------------------------------------------------------------------*/
#define ADCIN_0_A ((uint32_t)0x00)
#define ADCIN_1_A ((uint32_t)0x01)
#define ADCIN_2_A ((uint32_t)0x02)
#define ADCIN_3_A ((uint32_t)0x03)
#define ADCIN_4_A ((uint32_t)0x04)
#define ADCIN_5_A ((uint32_t)0x05)
#define ADCIN_6_A ((uint32_t)0x06)
#define ADCIN_7_A ((uint32_t)0x07)
#define ADCIN_8_B ((uint32_t)0x08)
#define ADCIN_9_B ((uint32_t)0x09)
#define ADCIN_10_B ((uint32_t)0x0A)
#define ADCIN_11_B ((uint32_t)0x0B)
#define ADCIN_12_B ((uint32_t)0x0C)
#define ADCIN_13_B ((uint32_t)0x0D)
#define ADCIN_14_B ((uint32_t)0x0E)
#define ADCIN_15_B ((uint32_t)0x0F)
  /*--------------------------------------------------------------------------------------------
   ADC-Channel select
    --------------------------------------------------------------------------------------------*/
  // #define ADC_Channel_0 ((uint32_t)0x00)
  // #define ADC_Channel_1 ((uint32_t)0x01)
  // #define ADC_Channel_2 ((uint32_t)0x02)
  // #define ADC_Channel_3 ((uint32_t)0x03)
  // #define ADC_Channel_4 ((uint32_t)0x04)
  // #define ADC_Channel_5 ((uint32_t)0x05)
  // #define ADC_Channel_6 ((uint32_t)0x06)
  // #define ADC_Channel_7 ((uint32_t)0x07)
  // #define ADC_Channel_8 ((uint32_t)0x08)
  // #define ADC_Channel_9 ((uint32_t)0x09)
  // #define ADC_Channel_10 ((uint32_t)0x0A)
  // #define ADC_Channel_11 ((uint32_t)0x0B)
  // #define ADC_Channel_12 ((uint32_t)0x0C)
  // #define ADC_Channel_13 ((uint32_t)0x0D)
  // #define ADC_Channel_14 ((uint32_t)0x0E)
  // #define ADC_Channel_15 ((uint32_t)0x0F)

  // #define ADC_Channel_16 ((uint32_t)0x00020000)
  // #define ADC_Channel_17 ((uint32_t)0x00030000)
/*--------------------------------------------------------------------------------------------
 ADC-Inject Channel select
  --------------------------------------------------------------------------------------------*/
#define ADC_InjectChannel_1 ((uint32_t)0x01)
#define ADC_InjectChannel_2 ((uint32_t)0x02)
#define IS_ADC_InjectChannel(Injectx) ((Injectx == ADC_InjectChannel_1) || (Injectx == ADC_InjectChannel_2))

#define ADC_Channel_TempSensor ((uint32_t)ADC_Channel_17)
#define ADC_Channel_Vrefint ((uint32_t)ADC_Channel_16)

#define ADC_FLAG_ADRDY ADC_ISR_ADRDY_Msk
#define ADC_FLAG_EOSMP ADC_ISR_EOSMP_Msk
#define ADC_FLAG_EOC ADC_ISR_EOC_Msk
#define ADC_FLAG_EOSEQ ADC_ISR_EOSEQ_Msk
#define ADC_FLAG_OVR ADC_ISR_OVR_Msk
#define ADC_FLAG_AWD ADC_ISR_AWD_Msk

#define ADC_CFGR2_Con_Pos 0
#define ADC_CFGR2_Con_Msk (0x1fUL << ADC_CFGR1_AWDCH_Pos)

/* =========================================================================================================================== */
/* ================                                            ADC                                            ================ */
/* =========================================================================================================================== */

/* ==========================================================  ISR  ========================================================== */
#define ADC_ISR_FIFO_FULL_Pos (12UL)            /*!< AWD (Bit 7)                                           */
#define ADC_ISR_FIFO_FULL_Msk (0x1000UL)        /*!< AWD (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_FIFO_ALMOST_FULL_Pos (11UL)     /*!< AWD (Bit 7)                                           */
#define ADC_ISR_FIFO_ALMOST_FULL_Msk (0x800UL)  /*!< AWD (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_FIFO_HALF_FULL_Pos (10UL)       /*!< AWD (Bit 7)                                           */
#define ADC_ISR_FIFO_HALF_FULL_Msk (0x400UL)    /*!< AWD (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_FIFO_ALMOST_EMPTY_Pos (9UL)     /*!< AWD (Bit 7)                                           */
#define ADC_ISR_FIFO_ALMOST_EMPTY_Msk (0x200UL) /*!< AWD (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_FIFO_EMPTY_Pos (8UL)            /*!< AWD (Bit 7)                                           */
#define ADC_ISR_FIFO_EMPTY_Msk (0x100UL)        /*!< AWD (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_AWD_Pos (7UL)                   /*!< AWD (Bit 7)                                           */
#define ADC_ISR_AWD_Msk (0x80UL)                /*!< AWD (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_OVR_Pos (4UL)                   /*!< OVR (Bit 4)                                           */
#define ADC_ISR_OVR_Msk (0x10UL)                /*!< OVR (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_EOSEQ_Pos (3UL)                 /*!< EOSEQ (Bit 3)                                         */
#define ADC_ISR_EOSEQ_Msk (0x8UL)               /*!< EOSEQ (Bitfield-Mask: 0x01)                           */
#define ADC_ISR_EOC_Pos (2UL)                   /*!< EOC (Bit 2)                                           */
#define ADC_ISR_EOC_Msk (0x4UL)                 /*!< EOC (Bitfield-Mask: 0x01)                             */
#define ADC_ISR_EOSMP_Pos (1UL)                 /*!< EOSMP (Bit 1)                                         */
#define ADC_ISR_EOSMP_Msk (0x2UL)               /*!< EOSMP (Bitfield-Mask: 0x01)                           */
#define ADC_ISR_ADRDY_Pos (0UL)                 /*!< ADRDY (Bit 0)                                         */
#define ADC_ISR_ADRDY_Msk (0x1UL)               /*!< ADRDY (Bitfield-Mask: 0x01)                           */
/* ==========================================================  IER  ========================================================== */
#define ADC_IER_AWDIE_Pos (7UL)     /*!< AWDIE (Bit 7)                                         */
#define ADC_IER_AWDIE_Msk (0x80UL)  /*!< AWDIE (Bitfield-Mask: 0x01)                           */
#define ADC_IER_OVRIE_Pos (4UL)     /*!< OVRIE (Bit 4)                                         */
#define ADC_IER_OVRIE_Msk (0x10UL)  /*!< OVRIE (Bitfield-Mask: 0x01)                           */
#define ADC_IER_EOSIE_Pos (3UL)     /*!< EOSIE (Bit 3)                                         */
#define ADC_IER_EOSIE_Msk (0x8UL)   /*!< EOSIE (Bitfield-Mask: 0x01)                           */
#define ADC_IER_EOCIE_Pos (2UL)     /*!< EOCIE (Bit 2)                                         */
#define ADC_IER_EOCIE_Msk (0x4UL)   /*!< EOCIE (Bitfield-Mask: 0x01)                           */
#define ADC_IER_EOSMPIE_Pos (1UL)   /*!< EOSMPIE (Bit 1)                                       */
#define ADC_IER_EOSMPIE_Msk (0x2UL) /*!< EOSMPIE (Bitfield-Mask: 0x01)                         */
#define ADC_IER_ADRDYIE_Pos (0UL)   /*!< ADRDYIE (Bit 0)                                       */
#define ADC_IER_ADRDYIE_Msk (0x1UL) /*!< ADRDYIE (Bitfield-Mask: 0x01)                         */
/* ==========================================================  CR  =========================================================== */
#define ADC_CR_ADSTP_Pos (4UL)     /*!< ADSTP (Bit 4)                                         */
#define ADC_CR_ADSTP_Msk (0x10UL)  /*!< ADSTP (Bitfield-Mask: 0x01)                           */
#define ADC_CR_ADSTART_Pos (2UL)   /*!< ADSTART (Bit 2)                                       */
#define ADC_CR_ADSTART_Msk (0x4UL) /*!< ADSTART (Bitfield-Mask: 0x01)                         */
#define ADC_CR_ADDIS_Pos (1UL)     /*!< ADDIS (Bit 1)                                         */
#define ADC_CR_ADDIS_Msk (0x2UL)   /*!< ADDIS (Bitfield-Mask: 0x01)                           */
#define ADC_CR_ADEN_Pos (0UL)      /*!< ADEN (Bit 0)                                          */
#define ADC_CR_ADEN_Msk (0x1UL)    /*!< ADEN (Bitfield-Mask: 0x01)                            */
/* =========================================================  CFGR1  ========================================================= */
#define ADC_CFGR1_AWDCH_Pos (27UL)         /*!< AWDCH (Bit 27)                                        */
#define ADC_CFGR1_AWDCH_Msk (0x78000000UL) /*!< AWDCH (Bitfield-Mask: 0x0f)                           */
#define ADC_CFGR1_AWDEN_Pos (23UL)         /*!< AWDEN (Bit 23)                                        */
#define ADC_CFGR1_AWDEN_Msk (0x800000UL)   /*!< AWDEN (Bitfield-Mask: 0x01)                           */
#define ADC_CFGR1_AWDSGL_Pos (22UL)        /*!< AWDSGL (Bit 22)                                       */
#define ADC_CFGR1_AWDSGL_Msk (0x400000UL)  /*!< AWDSGL (Bitfield-Mask: 0x01)                          */
#define ADC_CFGR1_DISCEN_Pos (16UL)        /*!< DISCEN (Bit 16)                                       */
#define ADC_CFGR1_DISCEN_Msk (0x10000UL)   /*!< DISCEN (Bitfield-Mask: 0x01)                          */
#define ADC_CFGR1_WAIT_Pos (14UL)          /*!< WAIT (Bit 14)                                         */
#define ADC_CFGR1_WAIT_Msk (0x4000UL)      /*!< WAIT (Bitfield-Mask: 0x01)                            */
#define ADC_CFGR1_CONT_Pos (13UL)          /*!< CONT (Bit 13)                                         */
#define ADC_CFGR1_CONT_Msk (0x2000UL)      /*!< CONT (Bitfield-Mask: 0x01)                            */
#define ADC_CFGR1_OVRMOD_Pos (12UL)        /*!< OVRMOD (Bit 12)                                       */
#define ADC_CFGR1_OVRMOD_Msk (0x1000UL)    /*!< OVRMOD (Bitfield-Mask: 0x01)                          */
#define ADC_CFGR1_ALIGN_Pos (5UL)          /*!< ALIGN (Bit 5)                                         */
#define ADC_CFGR1_ALIGN_Msk (0x20UL)       /*!< ALIGN (Bitfield-Mask: 0x01)                           */
#define ADC_CFGR1_SCANDIR_Pos (2UL)        /*!< SCANDIR (Bit 2)                                       */
#define ADC_CFGR1_SCANDIR_Msk (0x4UL)      /*!< SCANDIR (Bitfield-Mask: 0x01)                         */
#define ADC_CFGR1_DMAEN_Pos (0UL)          /*!< DMAEN (Bit 0)                                         */
#define ADC_CFGR1_DMAEN_Msk (0x1UL)        /*!< DMAEN (Bitfield-Mask: 0x01)                           */
/* =========================================================  CFGR2  ========================================================= */
#define ADC_CFGR2_CKMODE_Pos (27UL)         /*!< CKMODE (Bit 27)                                       */
#define ADC_CFGR2_CKMODE_Msk (0xf8000000UL) /*!< CKMODE (Bitfield-Mask: 0x1f)                          */
#define ADC_CFGR2_LP_EN_Pos (10UL)          /*!< LP_EN (Bit 10)                                        */
#define ADC_CFGR2_LP_EN_Msk (0x400UL)       /*!< LP_EN (Bitfield-Mask: 0x01)                           */
#define ADC_CFGR2_LP_Pos (9UL)              /*!< LP (Bit 9)                                            */
#define ADC_CFGR2_LP_Msk (0x200UL)          /*!< LP (Bitfield-Mask: 0x01)                              */
#define ADC_CFGR2_PWR_SET_Pos (3UL)         /*!< PWR_SET (Bit 3)                                       */
#define ADC_CFGR2_PWR_SET_Msk (0x8UL)       /*!< PWR_SET (Bitfield-Mask: 0x01)                         */
#define ADC_CFGR2_PULSE_Pos (2UL)           /*!< PULSE (Bit 2)                                         */
#define ADC_CFGR2_PULSE_Msk (0x4UL)         /*!< PULSE (Bitfield-Mask: 0x01)                           */
#define ADC_CFGR2_EXT_REF_Pos (1UL)         /*!< EXT_REF (Bit 1)                                       */
#define ADC_CFGR2_EXT_REF_Msk (0x2UL)       /*!< EXT_REF (Bitfield-Mask: 0x01)                         */
#define ADC_CFGR2_CON_Pos (0UL)             /*!< CON (Bit 0)                                           */
#define ADC_CFGR2_CON_Msk (0x1UL)           /*!< CON (Bitfield-Mask: 0x01)                             */
/* =========================================================  SMPR  ========================================================== */
#define ADC_SMPR_CONVERT_Pos (7UL)     /*!< CONVERT (Bit 7)                                       */
#define ADC_SMPR_CONVERT_Msk (0xf80UL) /*!< CONVERT (Bitfield-Mask: 0x1f)                         */
#define ADC_SMPR_ADJUST_Pos (4UL)      /*!< ADJUST (Bit 4)                                        */
#define ADC_SMPR_ADJUST_Msk (0x70UL)   /*!< ADJUST (Bitfield-Mask: 0x07)                          */
#define ADC_SMPR_SMP_Pos (0UL)         /*!< SMP (Bit 0)                                           */
#define ADC_SMPR_SMP_Msk (0x7UL)       /*!< SMP (Bitfield-Mask: 0x07)                             */
/* ==========================================================  TR  =========================================================== */
#define ADC_TR_HT_Pos (16UL)        /*!< HT (Bit 16)                                           */
#define ADC_TR_HT_Msk (0xfff0000UL) /*!< HT (Bitfield-Mask: 0xfff)                             */
#define ADC_TR_LT_Pos (0UL)         /*!< LT (Bit 0)                                            */
#define ADC_TR_LT_Msk (0xfffUL)     /*!< LT (Bitfield-Mask: 0xfff)                             */
/* ========================================================  CHSELR  ========================================================= */
#define ADC_CHSELR_CHSELR_Pos (0UL)      /*!< CHSELR (Bit 0)                                        */
#define ADC_CHSELR_CHSELR_Msk (0xffffUL) /*!< CHSELR (Bitfield-Mask: 0xffff)                        */
/* ==========================================================  DR  =========================================================== */
#define ADC_DR_DATA_Pos (0UL)      /*!< DATA (Bit 0)                                          */
#define ADC_DR_DATA_Msk (0xffffUL) /*!< DATA (Bitfield-Mask: 0xffff)                          */
/* =========================================================  TRIM  ========================================================== */
#define ADC_TRIM_VOS_TRIM_Pos (16UL)       /*!< VOS_TRIM (Bit 16)                                     */
#define ADC_TRIM_VOS_TRIM_Msk (0xff0000UL) /*!< VOS_TRIM (Bitfield-Mask: 0xff)                        */
#define ADC_TRIM_GAIN_TRIM_Pos (0UL)       /*!< GAIN_TRIM (Bit 0)                                     */
#define ADC_TRIM_GAIN_TRIM_Msk (0xffUL)    /*!< GAIN_TRIM (Bitfield-Mask: 0xff)                       */
/* ========================================================  EXTCFG  ========================================================= */
#define ADC_EXTCFG_EXTSE_Pos (27UL)          /*!< EXTSE (Bit 27)                                        */
#define ADC_EXTCFG_EXTSE_Msk (0xf8000000UL)  /*!< EXTSE (Bitfield-Mask: 0x1f)                           */
#define ADC_EXTCFG_EXTSELG4_Pos (20UL)       /*!< EXTSELG4 (Bit 20)                                     */
#define ADC_EXTCFG_EXTSELG4_Msk (0x300000UL) /*!< EXTSELG4 (Bitfield-Mask: 0x03)                        */
#define ADC_EXTCFG_EXTSELG3_Pos (18UL)       /*!< EXTSELG3 (Bit 18)                                     */
#define ADC_EXTCFG_EXTSELG3_Msk (0xc0000UL)  /*!< EXTSELG3 (Bitfield-Mask: 0x03)                        */
#define ADC_EXTCFG_EXTSELG2_Pos (16UL)       /*!< EXTSELG2 (Bit 16)                                     */
#define ADC_EXTCFG_EXTSELG2_Msk (0x30000UL)  /*!< EXTSELG2 (Bitfield-Mask: 0x03)                        */
#define ADC_EXTCFG_EXTSELG1_Pos (14UL)       /*!< EXTSELG1 (Bit 14)                                     */
#define ADC_EXTCFG_EXTSELG1_Msk (0xc000UL)   /*!< EXTSELG1 (Bitfield-Mask: 0x03)                        */
#define ADC_EXTCFG_EXTSELG0_Pos (12UL)       /*!< EXTSELG0 (Bit 12)                                     */
#define ADC_EXTCFG_EXTSELG0_Msk (0x3000UL)   /*!< EXTSELG0 (Bitfield-Mask: 0x03)                        */
#define ADC_EXTCFG_EXTDTG_Pos (4UL)          /*!< EXTDTG (Bit 4)                                        */
#define ADC_EXTCFG_EXTDTG_Msk (0xff0UL)      /*!< EXTDTG (Bitfield-Mask: 0xff)                          */
#define ADC_EXTCFG_OP_MODE_Pos (3UL)         /*!< OP_MODE (Bit 3)                                       */
#define ADC_EXTCFG_OP_MODE_Msk (0x8UL)       /*!< OP_MODE (Bitfield-Mask: 0x01)                         */
#define ADC_EXTCFG_EDEG_MODE_Pos (2UL)       /*!< EDEG_MODE (Bit 2)                                     */
#define ADC_EXTCFG_EDEG_MODE_Msk (0x4UL)     /*!< EDEG_MODE (Bitfield-Mask: 0x01)                       */
#define ADC_EXTCFG_EXTEN_Pos (0UL)           /*!< EXTEN (Bit 0)                                         */
#define ADC_EXTCFG_EXTEN_Msk (0x3UL)         /*!< EXTEN (Bitfield-Mask: 0x03)                           */
/* ========================================================  TESTCFG  ======================================================== */
#define ADC_TESTCFG_PD_ADC_Pos (14UL)     /*!< PD_ADC (Bit 14)                                       */
#define ADC_TESTCFG_PD_ADC_Msk (0x4000UL) /*!< PD_ADC (Bitfield-Mask: 0x01)                          */
#define ADC_TESTCFG_PD_REF_Pos (13UL)     /*!< PD_REF (Bit 13)                                       */
#define ADC_TESTCFG_PD_REF_Msk (0x2000UL) /*!< PD_REF (Bitfield-Mask: 0x01)                          */
#define ADC_TESTCFG_PD_SH2_Pos (12UL)     /*!< PD_SH2 (Bit 12)                                       */
#define ADC_TESTCFG_PD_SH2_Msk (0x1000UL) /*!< PD_SH2 (Bitfield-Mask: 0x01)                          */
#define ADC_TESTCFG_PD_SH1_Pos (11UL)     /*!< PD_SH1 (Bit 11)                                       */
#define ADC_TESTCFG_PD_SH1_Msk (0x800UL)  /*!< PD_SH1 (Bitfield-Mask: 0x01)                          */
#define ADC_TESTCFG_RMD_Pos (10UL)        /*!< RMD (Bit 10)                                          */
#define ADC_TESTCFG_RMD_Msk (0x400UL)     /*!< RMD (Bitfield-Mask: 0x01)                             */
#define ADC_TESTCFG_RTA_Pos (9UL)         /*!< RTA (Bit 9)                                           */
#define ADC_TESTCFG_RTA_Msk (0x200UL)     /*!< RTA (Bitfield-Mask: 0x01)                             */
#define ADC_TESTCFG_RT_Pos (6UL)          /*!< RT (Bit 6)                                            */
#define ADC_TESTCFG_RT_Msk (0x1c0UL)      /*!< RT (Bitfield-Mask: 0x07)                              */
#define ADC_TESTCFG_RI_Pos (2UL)          /*!< RI (Bit 2)                                            */
#define ADC_TESTCFG_RI_Msk (0x3cUL)       /*!< RI (Bitfield-Mask: 0x0f)                              */
#define ADC_TESTCFG_TT_Pos (1UL)          /*!< TT (Bit 1)                                            */
#define ADC_TESTCFG_TT_Msk (0x2UL)        /*!< TT (Bitfield-Mask: 0x01)                              */
#define ADC_TESTCFG_TEST_Pos (0UL)        /*!< TEST (Bit 0)                                          */
#define ADC_TESTCFG_TEST_Msk (0x1UL)      /*!< TEST (Bitfield-Mask: 0x01)                            */
                                          /* ========================================================  JCR  ============================================*/
#define ADC_JCR_J2TRIG_Pos (21UL)
#define ADC_JCR_J2TRIG_Msk (0x200000UL)
#define ADC_JCR_J2EN_Pos (16UL)
#define ADC_JCR_J2EN_Msk (0x10000UL)
#define ADC_JCR_J1TRIG_Pos (5UL)
#define ADC_JCR_J1TRIG_Msk (0x20UL)
#define ADC_JCR_J1EN_Pos (0UL)
#define ADC_JCR_J1EN_Msk (0x01UL)
  /*  Function used to set the ADC configuration to the default reset state *****/
  void ADC_DeInit(ADC_TypeDef *ADCx); //重启ad时钟

  /* Initialization and Configuration functions *********************************/
  void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct); //初始化adc 共有12个参数需要配置/To initialize an ADC, you need to configure 12 parameters
  void ADC_StructInit(ADC_InitTypeDef *ADC_InitStruct);              // ADC 参数结构体初值定义/ADC parameter structure initial value definition
  void ADC_Cmd(ADC_TypeDef *ADCx, FunctionalState NewState);         //使能ADC/Enable configuration of the ADC

  /* Power saving functions *****************************************************/
  void ADC_AutoPowerOffCmd(ADC_TypeDef *ADCx, FunctionalState NewState); // adc 自动掉电使能函数/Enable configuration of the Automatic power down

  /* Analog Watchdog configuration functions ************************************/
  void ADC_AnalogWatchdogCmd(ADC_TypeDef *ADCx, FunctionalState NewState);                                   // ADC模拟开门狗/ADC simulates door opening dog electricity
  void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef *ADCx, uint16_t HighThreshold, uint16_t LowThreshold); //模拟看门狗限值/Simulate watchdog limits
  void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef *ADCx, uint32_t ADC_AnalogWatchdog_Channel);        //单通道看门狗选择/Single channel watchdog option
  void ADC_AnalogWatchdogSingleChannelCmd(ADC_TypeDef *ADCx, FunctionalState NewState);                      //单通道看门狗使能/Enable configuration of the single channel guard dog

  /* Temperature Sensor , Vrefint and Vbat management function ******************/
  void ADC_TempSensorCmd(FunctionalState NewState); //温度传感器采样使能/Enable configuration of the temperature sensor sampling
  void ADC_VDDACmd(FunctionalState NewState);       //内部电压采样使能/Enable configuration of the internal voltage sampling

  /* Channels Configuration functions *******************************************/
  void ADC_ChannelConfig(uint32_t NbrOfChannel, uint8_t *addr);
  //通道采样时间设置/Channel sampling time setting

  void ADC_StopOfConversion(ADC_TypeDef *ADCx);       //停止ADC转换/Stop ADC conversion
  void ADC_StartOfConversion(ADC_TypeDef *ADCx);      //开始adc转换/Start ADC conversion
  uint16_t ADC_GetConversionValue(ADC_TypeDef *ADCx); //读取转换结果/Read the conversion result
  void ADC_CloseAdc(ADC_TypeDef *ADCx);               //关闭ADC模块/Shutting down the ADC Module

  void ADC_VrefintSelect(ADC_TypeDef *ADCx, uint32_t ADC_Vref); // ADC参考电压选择/ADC reference voltage selection

  /* Interrupts and flags management functions **********************************/
  void ADC_ITConfig(ADC_TypeDef *ADCx, uint32_t ADC_IT, FunctionalState NewState); // ADC中断使能/Enable configuration of ADC interrupt
  FlagStatus ADC_GetFlagStatus(ADC_TypeDef *ADCx, uint32_t ADC_FLAG);              // ADC状态标志位读取/ADC status flag bit read
  void ADC_ClearFlag(ADC_TypeDef *ADCx, uint32_t ADC_FLAG);                        //清ADC状态标志位/Clears the ADC status flag bit
  ITStatus ADC_GetITStatus(ADC_TypeDef *ADCx, uint32_t ADC_IT);                    //获取中断标志位/Get the interrupt flag bit
  void ADC_ClearITPendingBit(ADC_TypeDef *ADCx, uint32_t ADC_IT);                  //清除中断标志位/Clear the interrupt flag bit
  /*********************************ADC_InjectedChannel*******************/
  void ADC_InjectedChannelConfig(ADC_TypeDef *ADCx, uint8_t Injectx, uint8_t ADC_Channel_IO);
  void ADC_InjectedChannelCmd(ADC_TypeDef *ADCx, uint8_t Injectx, FunctionalState NewState);
  void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef *ADCx, uint8_t Injectx, FunctionalState NewState);
  uint32_t ADC_GetInjected1ConversionValue(ADC_TypeDef *ADCx, uint8_t Injectx);
  int ADC_GetVTStemperature(int ADCValueNow, int Vref);
#ifdef __cplusplus
}
#endif
#endif
