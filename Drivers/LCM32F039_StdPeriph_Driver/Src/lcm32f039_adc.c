/**
  ******************************************************************************
  * @file    lcm32f039_adc.c
  * @author  MCD Application Team
  * @version V2.0.4
  * @date    2025-03-18
  * @brief   This file provides all the adc emulation firmware functions.
  ******************************************************************************/
/**
  ******************************************************************************
  * @attention:
  *
  *
  *
  *
  ******************************************************************************/

#include "lcm32f039_adc.h"
#include "lcm32f039_rcc.h"

/*--------------------------------------------------------------------------------------------
   adc dis,open adc clk again ADC时钟复位使能函数
  --------------------------------------------------------------------------------------------*/
void ADC_DeInit(ADC_TypeDef *ADCx)
{
  if (ADCx == ADC)
  {
    /* Release ADC1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_ADC, ENABLE);
    /* Enable ADC1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_ADC, DISABLE);
  }
}

/*--------------------------------------------------------------------------------------------
  adc enable /ADC使能
  --------------------------------------------------------------------------------------------*/
void ADC_Cmd(ADC_TypeDef *ADCx, FunctionalState State)
{
  if (State != DISABLE)
  {
    /* Set the ADEN bit to Enable the ADC peripheral */
    ADC->CR_b.ADEN = 1;
    ADC->TESTCFG_b.PD_ADC = 0;
    ADC->TESTCFG_b.PD_REF = 0;
    ADC->TESTCFG_b.PD_SH2 = 0;
    ADC->TESTCFG_b.PD_SH1 = 0;
    do
    {
    } while (ADC->ISR_b.ADRDY == 0);
  }
  else
  {
    /* Set the ADDIS to Disable the ADC peripheral */
    ADC->CR_b.ADSTP = 1;
    do
    {
    } while (ADC->CR_b.ADSTP);
    ADC->CR_b.ADDIS = 1;
    ADC->ISR_b.ADRDY = 1;
    ADC->CFGR1_b.FIFO_CLR = 1;
    do
    {
    } while (ADC->CR_b.ADEN);
  }
}
/*--------------------------------------------------------------------------------------------
  adc init:12 parameters/ADC结构体初始化
  --------------------------------------------------------------------------------------------*/
void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct)
{
  uint32_t tmpreg = 0;
  uint32_t tmpreg2 = 0;
  uint32_t tmpregg = 0;
  uint32_t clktmpreg = 0;

  uint32_t AD_SELECT_DATA = 0;
  uint32_t AD_TRIM_temp = 0;

  /* Get the ADCx CFGR value */
  clktmpreg = ADC->CFGR2;

  tmpreg = ADC->CFGR1;
  tmpregg = ADC->EXTCFG;
  tmpreg2 = ADC->SMPR;
  /* Clear SCANDIR, RES[1:0], ALIGN, EXTSEL[2:0], EXTEN[1:0] and CONT bits */
  tmpreg &= 0x00000000;
  tmpregg &= 0x00000000;
  clktmpreg &= 0x07ffffff;
  tmpreg2 &= 0x00000000;
  tmpreg |= (uint32_t)(ADC_InitStruct->ADC_Resolution | \
                ((uint32_t)(ADC_InitStruct->ADC_DMATRIG_LEVEL)) | ((uint32_t)(ADC_InitStruct->ADC_DiscMode) << 16) | \
                    ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ScanDirection);

  tmpregg |= (uint32_t)(ADC_InitStruct->ADC_ExternalTrigConvEdge | ADC_InitStruct->ADC_ExternalTrigConv);

  clktmpreg = (uint32_t)(ADC_InitStruct->ADC_ClkMode | ADC_InitStruct->ADC_SamecMode);
  tmpreg2 |= (uint32_t)((ADC_InitStruct->ADC_AdjustTime) | ((ADC_InitStruct->ADC_SampleDelayTime) << 3) | ((ADC_InitStruct->ADC_SampleTime) << 7) |
                        ((ADC_InitStruct->ADC_ConvertPulseTime) << 10) | ((ADC_InitStruct->ADC_ConvertTime) << 12));
  ADC->CFGR1 = tmpreg;
  ADC->CFGR1 |= ((uint32_t)(ADC_InitStruct->ADC_DMA));
  ADC->EXTCFG = tmpregg;
  ADC->TESTCFG = 0x000000dc;
  ADC->CFGR2 = clktmpreg;
  ADC->SMPR = tmpreg2;
  AD_SELECT_DATA = ADC_InitStruct->ADC_Vref;

  AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5EC)); //

  if (AD_TRIM_temp == 0xffffffff)
  {
    AD_TRIM_temp = 0x00440055;
  }
  else
  {
    AD_TRIM_temp = ADC->TRIM;
  }
  ADC->TRIM = AD_TRIM_temp;

  if (AD_SELECT_DATA == 1) // in vdd 33
  {
    ADC->CFGR2 |= ADC_Vref_Internal_3V3_data;
  }
  else if (AD_SELECT_DATA == 2) // ex vdd 33
  {
    AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A0)); // ADC校准值5V
    if (AD_TRIM_temp != 0xffffffff)
      ADC->TRIM = AD_TRIM_temp;
    ADC->CFGR2 |= ADC_Vref_Internal_5V_data;
  }
  else if (AD_SELECT_DATA == 3)
  {
   AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A4)); // ADC校准值2.5V
   if (AD_TRIM_temp != 0xffffffff)
     ADC->TRIM = AD_TRIM_temp;
     ADC->CFGR2 |= (1 << 1);
     chipctrl_access();
     CHIPCTRL->PWR_CFG |= ADC_Vref_Externa_Vrh2V5_data;
     __dekey();
  }
  else if (AD_SELECT_DATA == 4)
  {
   AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A8)); // ADC校准值4V
   if (AD_TRIM_temp != 0xffffffff)
     ADC->TRIM = AD_TRIM_temp;
     ADC->CFGR2 |= (1 << 1);
     chipctrl_access();
     CHIPCTRL->PWR_CFG |= ADC_Vref_Externa_Vrh4V_data;
     __dekey();
  }
  else if (AD_SELECT_DATA == 5)
  {
   ADC->CFGR2 |= (1 << 1);
   chipctrl_access();
   CHIPCTRL->PWR_CFG |= ADC_Vref_Externa_VDD_data;
   __dekey();
  }
  else if (AD_SELECT_DATA == 6)
  {
     ADC->CFGR2 |= (1 << 1);
     chipctrl_access();
     CHIPCTRL->PWR_CFG |= ADC_Vref_Externa_IO_data;
     __dekey();
  }
  else if (AD_SELECT_DATA == 7)
  {
   AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A4)); // ADC校准值2.5V
   if (AD_TRIM_temp != 0xffffffff)
     ADC->TRIM = AD_TRIM_temp;
     ADC->CFGR2 |= (1 << 1);
     chipctrl_access();
     CHIPCTRL->PWR_CFG |= ADC_Vref_ExternaIO_VRH_C_data;
     __dekey();
  }
}

/*--------------------------------------------------------------------------------------------
   12 parameters for adc
  --------------------------------------------------------------------------------------------*/
void ADC_StructInit(ADC_InitTypeDef *ADC_InitStruct)
{
  /* Reset ADC init structure parameters values */
  /* Initialize the ADC_Resolution member */
  ADC_InitStruct->ADC_ClkMode = ADC_ClockMode_sysnClk; // clk set ：sysclk or pll

  ADC_InitStruct->ADC_Resolution = ADC_Resolution_12b; // 8b or 12b  采样精度12位

  ADC_InitStruct->ADC_AutoWatiMode = DISABLE; // 自动延迟转换模式，只有数据被读走，才能进行下一次转换

  ADC_InitStruct->ADC_DMATRIG_LEVEL = FIFO_DMA_NOEMPTY; // FIFO 非空时，产生一次 DMA 请求

  ADC_InitStruct->ADC_DMA = DISABLE; // cfg1 bit0，ADC dma功能使能

  ADC_InitStruct->ADC_DiscMode = DISABLE; // cfg1 bit16 断续模式，触发一次转换1个 通道

  ADC_InitStruct->ADC_SamecMode = DISABLE; // cfg2 bit0 同时采样模式

  /* Initialize the ADC_ExternalTrigConvEdge member */
  ADC_InitStruct->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // 外部触发

  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO; // 外部触发源

  /* Initialize the ADC_DataAlign member */
  ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right; // 数据右对齐

  /* Initialize the ADC_ScanDirection member */
  ADC_InitStruct->ADC_ScanDirection = ADC_ScanDirection_Upward; // 通道扫描方向

  ADC_InitStruct->ADC_Vref = ADC_Vref_Internal_3V3;
  ADC_InitStruct->ADC_SampleTime = 7;       // 取值范围0-7;AD 采样信号脉冲宽度。默认值为 010，脉冲宽度=(x+1)*CLK
  ADC_InitStruct->ADC_SampleDelayTime = 0;  // 取值范围0-15;AD 采样延迟。同时采样模式下，一次采样周期内两次采样的延迟;
  ADC_InitStruct->ADC_AdjustTime = 7;       // 取值范围0-7;AD 上电准备阶段脉冲宽度。默认值为 010，脉冲宽度=(x+1)*CLK
  ADC_InitStruct->ADC_ConvertTime = 0;      // 取值范围0-15;采样周期
  ADC_InitStruct->ADC_ConvertPulseTime = 0; // 取值范围0-1;启动转换脉冲宽度。默认值为 0，脉冲宽度=(x+3)*CLK
}

/*--------------------------------------------------------------------------------------------
   auto power off mode for adc
  --------------------------------------------------------------------------------------------*/
void ADC_AutoPowerOffCmd(ADC_TypeDef *ADCx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the ADC Automatic Power-Off */
    ADC->CFGR2 |= (2 << 9);
  }
  else
  {
    /* Disable the ADC Automatic Power-Off */
    ADC->CFGR2 &= ~(1 << 10);
  }
}

/********************************************
注入通道1,2，采样通道配置
Injection channel 1,2, sampling channel configuration
 ***********************************************/
void ADC_InjectedChannelConfig(ADC_TypeDef *ADCx, uint8_t Injectx, uint8_t ADC_Channel_IO)
{
  uint32_t tmpreg = 0;
  tmpreg = ADC->CFGR2;
  tmpreg &= 0X01; // 是否同时采样/Whether to sample simultaneously
  assert_param(IS_ADC_InjectChannel(Injectx));
  uint32_t tmpreg1;
  tmpreg1 = ADC->JCR;
  tmpreg1 &= 0xFFDEFFDE;

  if (Injectx == ADC_InjectChannel_1)
  {
    tmpreg1 |= (ADC_Channel_IO << 1);
  }
  if (Injectx == ADC_InjectChannel_2)
  {
    if (tmpreg == 0)
      tmpreg1 |= (ADC_Channel_IO << 17);
    else
    {
      if (ADC_Channel_IO > 7)
      {
        tmpreg1 |= ((ADC_Channel_IO - 8) << 17);
      }
    }
  }
  ADC->JCR = tmpreg1;
}
void ADC_InjectedChannelCmd(ADC_TypeDef *ADCx, uint8_t Injectx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_InjectChannel(Injectx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    if (Injectx == ADC_InjectChannel_1)
      ADCx->JCR |= ADC_JCR_J1EN_Msk;
    if (Injectx == ADC_InjectChannel_2)
      ADCx->JCR |= ADC_JCR_J2EN_Msk;
  }
  else
  {
    if (Injectx == ADC_InjectChannel_1)
      ADCx->JCR &= ~ADC_JCR_J1EN_Msk;
    if (Injectx == ADC_InjectChannel_2)
      ADCx->JCR &= ~ADC_JCR_J2EN_Msk;
  }
}
/****************************************
 * 注入通道x软件触发采样
 * Injection channel X software triggers sampling
 * *********************************************/
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef *ADCx, uint8_t Injectx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_InjectChannel(Injectx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    if (Injectx == ADC_InjectChannel_1)
      ADCx->JCR |= ADC_JCR_J1TRIG_Msk;
    if (Injectx == ADC_InjectChannel_2)
      ADCx->JCR |= ADC_JCR_J2TRIG_Msk;
  }
  else
  {
    if (Injectx == ADC_InjectChannel_1)
      ADCx->JCR &= ~ADC_JCR_J1TRIG_Msk;
    if (Injectx == ADC_InjectChannel_2)
      ADCx->JCR &= ~ADC_JCR_J2TRIG_Msk;
  }
}
/********************************************
注入通道1,2，采样值
Injection channel 1,2, sampling values
 ***********************************************/
uint32_t ADC_GetInjected1ConversionValue(ADC_TypeDef *ADCx, uint8_t Injectx)
{
  uint32_t tmp = 0;
  tmp = ADCx->JDR;
  if (Injectx == ADC_InjectChannel_1)
  {
    tmp = tmp & 0x0000FFFF;
  }
  if (Injectx == ADC_InjectChannel_2)
  {
    tmp = (tmp & 0xFFFF0000) >> 16;
  }
  /* Returns the selected injected channel conversion data value */
  return tmp;
}
/*--------------------------------------------------------------------------------------------
   anglogwatchdog enable
  --------------------------------------------------------------------------------------------*/
void ADC_AnalogWatchdogCmd(ADC_TypeDef *ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  if (NewState != DISABLE)
  {
    /* Enable the ADC Analog Watchdog */
    ADC->CFGR1 |= ADC_CFGR1_AWDEN_Msk;
  }
  else
  {
    /* Disable the ADC Analog Watchdog */
    ADC->CFGR1 &= (uint32_t)~ADC_CFGR1_AWDEN_Msk;
  }
}

/*--------------------------------------------------------------------------------------------
   anglogwatchdog low-high
  --------------------------------------------------------------------------------------------*/
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef *ADCx, uint16_t HighThreshold, uint16_t LowThreshold)
{

  /* Set the ADCx high and low threshold */
  ADC->TR = LowThreshold | ((uint32_t)HighThreshold << 16);
}

/*--------------------------------------------------------------------------------------------
   adc anglogwatchdog for which channel
  --------------------------------------------------------------------------------------------*/
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef *ADCx, uint32_t ADC_AnalogWatchdog_Channel)
{
  uint32_t tmpreg = 0;

  /* Get the old register value */
  tmpreg = ADC->CFGR1;

  /* Clear the Analog watchdog channel select bits */
  tmpreg &= ~(0xf << 27);

  /* Set the Analog watchdog channel */
  tmpreg |= ADC_AnalogWatchdog_Channel;

  /* Store the new register value */
  ADC->CFGR1 = tmpreg;
}

/*--------------------------------------------------------------------------------------------
   adc anglogwatchdog for single or all channel
  --------------------------------------------------------------------------------------------*/
void ADC_AnalogWatchdogSingleChannelCmd(ADC_TypeDef *ADCx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the ADC Analog Watchdog Single Channel */
    ADC->CFGR1 |= ADC_CFGR1_AWDSGL_Msk;
  }
  else
  {
    /* Disable the ADC Analog Watchdog Single Channel */
    ADC->CFGR1 &= (uint32_t)~ADC_CFGR1_AWDSGL_Msk;
  }
}

/*--------------------------------------------------------------------------------------------
  enable adc channel for tempsensor,other channel cant use
  --------------------------------------------------------------------------------------------*/
void ADC_TempSensorCmd(FunctionalState NewState)
{
  /* Check the parameters */
  //  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the temperature sensor channel*/
    ADC->CHSELR = (uint32_t)(1 << 17);
  }
  else
  {
    /* Disable the temperature sensor channel*/
    ADC->CHSELR &= (uint32_t)(~(1 << 17));
  }
}

/*--------------------------------------------------------------------------------------------
  enable adc channel for vrefint,other channel cant use
  --------------------------------------------------------------------------------------------*/
void ADC_VDDACmd(FunctionalState NewState)
{
  /* Check the parameters */
  // assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the Vrefint channel*/
    ADC->CHSELR = (uint32_t)(3 << 16);
  }
  else
  {
    /* Disable the Vrefint channel*/
    ADC->CHSELR &= (uint32_t)(~(3 << 16));
  }
}

/*--------------------------------------------------------------------------------------------
  set adc channel sample
  --------------------------------------------------------------------------------------------*/
void ADC_ChannelConfig(uint32_t NbrOfChannel, uint8_t *ADIN_addr)
{
  uint8_t i;
  ADC->CHSELR_b.CHNUM = NbrOfChannel;
  ADC->CASR = 0;
  ADC->CBSR = 0;
  for (i = 0; i < 8; i++)
  {
    ADC->CASR |= ADIN_addr[i] << (i * 4);
  }
  for (i = 0; i < 8; i++)
  {
    ADC->CBSR |= ADIN_addr[i + 8] << (i * 4);
  }
}

/*--------------------------------------------------------------------------------------------
  STOP ADC：停止当前转换
  --------------------------------------------------------------------------------------------*/
void ADC_StopOfConversion(ADC_TypeDef *ADCx)
{
  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  if (ADC->CR & ADC_CR_ADSTART_Msk)
  {
    ADC->CR |= (uint32_t)ADC_CR_ADSTP_Msk;
  }
  while (ADC->CR & ADC_CR_ADSTP_Msk)
    ;
}

/*--------------------------------------------------------------------------------------------
  START ADC
  --------------------------------------------------------------------------------------------*/
void ADC_StartOfConversion(ADC_TypeDef *ADCx)
{
  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  ADC->CR |= 0x00000001; // ADEN
  while (!(ADC->ISR & ADC_ISR_ADRDY_Msk))
    ;
  ADC->CR |= (uint32_t)ADC_CR_ADSTART_Msk; // ADSTART
}
/*--------------------------------------------------------------------------------------------
  get adc value
  --------------------------------------------------------------------------------------------*/
uint16_t ADC_GetConversionValue(ADC_TypeDef *ADCx)
{
  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  /* Return the selected ADC conversion value */
  while (!(ADC->ISR & ADC_ISR_EOC_Msk))
    ;

  //	ADC->ISR|=ADC_ISR_EOC_Msk;
  return (uint16_t)ADC->DR;

  // return (uint16_t) ADC->DR;
}

/*--------------------------------------------------------------------------------------------
  停止或关闭 ADC模块
  /Stop or shut down the ADC module
  --------------------------------------------------------------------------------------------*/
void ADC_CloseAdc(ADC_TypeDef *ADCx)
{
  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  if (ADC->CR & ADC_CR_ADSTART_Msk)
  {
    ADC->CR |= ADC_CR_ADSTP_Msk;
  }
  while (ADC->CR & ADC_CR_ADSTP_Msk)
    ;
  ADC->CR |= ADC_CR_ADDIS_Msk;
  while (ADC->CR & ADC_CR_ADEN_Msk)
    ;
}

/*--------------------------------------------------------------------------------------------
  使能中断
   /Interrupt enable
  --------------------------------------------------------------------------------------------*/
void ADC_ITConfig(ADC_TypeDef *ADCx, uint32_t ADC_IT, FunctionalState NewState)
{

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC interrupts */
    ADC->IER |= ADC_IT;
  }
  else
  {
    /* Disable the selected ADC interrupts */
    ADC->IER &= (~(uint32_t)ADC_IT);
  }
}

/*--------------------------------------------------------------------------------------------
  读ADC标志状态
  /Read the ADC flag status
  --------------------------------------------------------------------------------------------*/
FlagStatus ADC_GetFlagStatus(ADC_TypeDef *ADCx, uint32_t ADC_FLAG)
{
  FlagStatus bitstatus = RESET;
  uint32_t tmpreg = 0;

  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  // assert_param(IS_ADC_GET_FLAG(ADC_FLAG));

  tmpreg = ADC->ISR;

  /* Check the status of the specified ADC flag */
  if ((tmpreg & ADC_FLAG) != (uint32_t)RESET)
  {
    /* ADC_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* ADC_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the ADC_FLAG status */
  return bitstatus;
}

/*--------------------------------------------------------------------------------------------
  清除 ADC 标志位
  /Clears the ADC flag bit
  --------------------------------------------------------------------------------------------*/
void ADC_ClearFlag(ADC_TypeDef *ADCx, uint32_t ADC_FLAG)
{
  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  // assert_param(IS_ADC_CLEAR_FLAG(ADC_FLAG));

  /* Clear the selected ADC flags */
  if (ADC->ISR & (uint32_t)ADC_FLAG)
  {
    ADC->ISR |= (uint32_t)ADC_FLAG;
  }
}
/*--------------------------------------------------------------------------------------------
  读 ADC 标志
  /Read the ADC logo
  --------------------------------------------------------------------------------------------*/
ITStatus ADC_GetITStatus(ADC_TypeDef *ADCx, uint32_t ADC_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;

  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  // assert_param(IS_ADC_GET_IT(ADC_IT));

  /* Get the ADC_IT enable bit status */
  enablestatus = (uint32_t)(ADC->IER & ADC_IT);

  /* Check the status of the specified ADC interrupt */
  if (((uint32_t)(ADC->ISR & ADC_IT) != (uint32_t)RESET) && (enablestatus != (uint32_t)RESET))
  {
    /* ADC_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* ADC_IT is reset */
    bitstatus = RESET;
  }
  /* Return the ADC_IT status */
  return bitstatus;
}

/*--------------------------------------------------------------------------------------------
 清除 ADC 中断标志位
  /Clears the ADC interrupt flag bit
  --------------------------------------------------------------------------------------------*/
void ADC_ClearITPendingBit(ADC_TypeDef *ADCx, uint32_t ADC_IT)
{
  /* Check the parameters */
  // assert_param(IS_ADC_ALL_PERIPH(ADCx));
  // assert_param(IS_ADC_CLEAR_IT(ADC_IT));

  /* Clear the selected ADC interrupt pending bits */
  ADC->ISR = (uint32_t)ADC_IT;
}

/*--------------------------------------------------------------------------------------------
   ADC 参考电压选择
   /ADC reference voltage selection
  --------------------------------------------------------------------------------------------*/
void ADC_VrefintSelect(ADC_TypeDef *ADCx, uint32_t ADC_Vref)
{
  uint32_t AD_SELECT_DATA = 0;
  uint32_t AD_TRIM_temp = 0;
  uint32_t power = 0;
  uint32_t sys = 0;
  AD_SELECT_DATA = (uint32_t)ADC_Vref;

  AD_TRIM_temp = (*(volatile uint32_t *)(0x1ffff5EC));

  if (AD_TRIM_temp == 0xffffffff)
  {
    AD_TRIM_temp = 0x00440055;
  }
  else
  {
    AD_TRIM_temp = ADC->TRIM;
  }
  ADC->TRIM = AD_TRIM_temp;

  power = CHIPCTRL->PWR_CFG;
  sys = ADC->CFGR2;

  power &= 0xfff0ffff;
  sys &= 0xfffffff5;

  if (AD_SELECT_DATA == 1) // in vdd 33
  {
    sys |= ADC_Vref_Internal_3V3_data;
  }
  else if (AD_SELECT_DATA == 2) // ex vdd 33
  {
    AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A0)); // ADC校准值5V
    if (AD_TRIM_temp != 0xffffffff)
      ADC->TRIM = AD_TRIM_temp;
    sys |= ADC_Vref_Internal_5V_data;
  }
  else if (AD_SELECT_DATA == 3)
  {
     AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A4)); // ADC校准值2.5V
     if (AD_TRIM_temp != 0xffffffff)
       ADC->TRIM = AD_TRIM_temp;
     sys |= (1 << 1);
     power |= ADC_Vref_Externa_Vrh2V5_data;
  }
  else if (AD_SELECT_DATA == 4)
  {
     AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A8)); // ADC校准值4V
     if (AD_TRIM_temp != 0xffffffff)
      ADC->TRIM = AD_TRIM_temp;
     sys |= (1 << 1);
     power |= ADC_Vref_Externa_Vrh4V_data;
  }
  else if (AD_SELECT_DATA == 5)
  {
     sys |= (1 << 1);
     power |= ADC_Vref_Externa_VDD_data;
  }
  else if (AD_SELECT_DATA == 6)
  {
     ADC->CFGR2 |= (1 << 1);
     power |= ADC_Vref_Externa_IO_data;
  }
  else if (AD_SELECT_DATA == 7)
  {
     AD_TRIM_temp = (*(volatile uint32_t *)(0x1FFFF5A4)); // ADC校准值2.5V
     if (AD_TRIM_temp != 0xffffffff)
       ADC->TRIM = AD_TRIM_temp;
       sys |= (1 << 1);
       power |= ADC_Vref_ExternaIO_VRH_C_data;
  }
  else
  {
    sys |= ADC_Vref_Internal_3V3_data;
  }
    chipctrl_access();
    CHIPCTRL->PWR_CFG = power;
    __dekey();
    ADC->CFGR2 = sys;
}

int ADC_GetVTStemperature(int ADCValueNow, int Vref)
{
  int temperatureNormalADC4V_3V3, temperatureHighADC4V_3V3, temperatureNormalADC5V_2V5, temperatureHighADC5V_2V5;
  int slope, temperatureNormal, temperatureHigh, temperatureNormalADC, temperatureHighADC, temperatureNow;
  temperatureNormal = (*(int *)(0x1FFFF580));
  temperatureHigh = (*(int *)(0x1FFFF584));
  temperatureNormalADC4V_3V3 = (*(int *)(0x1FFFF588)); // 4V和3.3V做参考的AD值常温
  temperatureHighADC4V_3V3 = (*(int *)(0x1FFFF58C));   // 4V和3.3V做参考的AD值高温
  temperatureNormalADC5V_2V5 = (*(int *)(0x1FFFF590)); // 5V和2.5V做参考的AD值常温
  temperatureHighADC5V_2V5 = (*(int *)(0x1FFFF594));   // 5V和2.5V做参考的AD值高温
  if (temperatureNormalADC4V_3V3 != 0xFFFFFFFF)
  {
    switch (Vref)
    {
    case 2500:
      temperatureNormalADC = temperatureNormalADC5V_2V5 & 0x0000FFFF; // 低16位参考为2.5V
      temperatureHighADC = temperatureHighADC5V_2V5 & 0x0000FFFF;
      break;
    case 3300:
      temperatureNormalADC = temperatureNormalADC4V_3V3 & 0x0000FFFF; // 低16位参考为3.3V
      temperatureHighADC = temperatureHighADC4V_3V3 & 0x0000FFFF;
      break;
    case 4000:
      temperatureNormalADC = (temperatureNormalADC4V_3V3 & 0x0FFF0000) >> 16; // 高16位参考为4V
      temperatureHighADC = (temperatureHighADC4V_3V3 & 0x0FFF0000) >> 16;
      break;
    case 5000:
      temperatureNormalADC = (temperatureNormalADC5V_2V5 & 0x0FFF0000) >> 16; // 高16位参考为5V
      temperatureHighADC = (temperatureHighADC5V_2V5 & 0x0FFF0000) >> 16;
      break;
    default:
      break;
    }
    slope = ((temperatureHighADC - temperatureNormalADC) * 100) / (temperatureHigh - temperatureNormal);
    temperatureNow = (ADCValueNow - temperatureHighADC) * 10 / slope + temperatureHigh / 10;
  }
  else
  {
    if ((Vref == 4000) || (Vref == 2500))
    {
      temperatureHighADC = (temperatureHighADC4V_3V3 & 0x0FFF0000) >> 16; // 高16位参考为4V
      temperatureNow = (((ADCValueNow * Vref) - (temperatureHighADC * 4000)) >> 12) * 100 / 515 + temperatureHigh / 10;
    }
    else
    {
      temperatureHighADC = temperatureHighADC4V_3V3 & 0x0000FFFF; // 低16位参考为3.3V
      temperatureNow = (((ADCValueNow * Vref) - (temperatureHighADC * 3300)) >> 12) * 100 / 515 + temperatureHigh / 10;
    }
  }
  return temperatureNow;
}
