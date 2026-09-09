/**
  ******************************************************************************
  * @file    lcm32f039_uart.c
  * @author  MCD Application Team
  * @version V2.0.4
  * @date    2025-03-18
  * @brief   This file provides all the uart emulation firmware functions.
  ******************************************************************************/
/**
  ******************************************************************************
  * @attention:
  *
  *
  *
  *
  ******************************************************************************/
#include "LCM32F039.h"
#include "lcm32f039_uart.h"
#include "lcm32f039_rcc.h"

/*--------------------------------------------------------------------------------------------
串口时钟使能：LCM32F039 包含UART0  UART1 UART3
--------------------------------------------------------------------------------------------*/
void UART_DeInit(UART_TypeDef *UARTx)
{

  if (UARTx == UART0)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART0, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART0, DISABLE);
  }
  else if (UARTx == UART1)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART1, DISABLE);
  }
  else
  {
    RCC_APB0PeriphResetCmd(RCC_APB0Periph_UART3, ENABLE);
    RCC_APB0PeriphResetCmd(RCC_APB0Periph_UART3, DISABLE);
  }
}

/*--------------------------------------------------------------------------------------------
串口波特率设置  /Set the UART baud rate
--------------------------------------------------------------------------------------------*/
void UART_BaudRateConfig(UART_TypeDef *UARTx, uint32_t baud)
{

  uint32_t freq;
  RCC_ClocksTypeDef RCC_Clocks;

  uint32_t baud16;
  uint16_t div_int;
  uint32_t div_frac;

  RCC_GetClockFreq(&RCC_Clocks);
  if (UARTx == UART0)
    freq = RCC_Clocks.UART0_Frequency;
  else if (UARTx == UART1)
    freq = RCC_Clocks.UART1_Frequency;
  else if (UARTx == UART3)
    freq = RCC_Clocks.UART3_Frequency;
#ifdef FPGA_K7
  freq = 30000000;
#endif

  baud16 = baud * 16;
  div_int = freq / baud16;
  div_frac = freq - baud16 * div_int;
  div_frac = (div_frac * 64 + baud16 / 2) / baud16;
  UARTx->IBRD = div_int;
  UARTx->FBRD = div_frac;
}

/*--------------------------------------------------------------------------------------------
串口初始化  /UART Initialization
--------------------------------------------------------------------------------------------*/
void UART_Init(UART_TypeDef *UARTx, UART_InitTypeDef *UART_InitStruct)
{
  volatile uint32_t tmpreg = 0;
  UART_BaudRateConfig(UARTx, (UART_InitStruct->UART_BaudRate));
  // frame config
  tmpreg = ((UART_InitStruct->UART_WordLength) | (UART_InitStruct->UART_StopBits) | (UART_InitStruct->UART_Parity));
  UARTx->LCR |= tmpreg;

  tmpreg = UARTx->CR;
  tmpreg &= 0xffff07ff;
  tmpreg = UART_InitStruct->UART_HardwareFlowControl;
  UARTx->CR |= tmpreg;

  tmpreg = UART_InitStruct->UART_Mode;
  if (tmpreg == (UART_Mode_Rx | UART_Mode_Tx))
  {
    UARTx->CR |= UART_Mode_Rx;
    UARTx->CR |= UART_Mode_Tx;
  }
  else if (tmpreg == UART_Mode_Tx)
  {
    UARTx->CR |= UART_Mode_Tx;
    UARTx->CR &= ~UART_Mode_Rx;
  }
  else // receive only
  {
    UARTx->CR |= UART_Mode_Rx;
    UARTx->CR &= ~UART_Mode_Tx;
  }
}

/*--------------------------------------------------------------------------------------------
串口结构参数定义  /UART structure parameter definition
--------------------------------------------------------------------------------------------*/
void UART_StructInit(UART_InitTypeDef *UART_InitStruct)
{
  /* UART_InitStruct members default value */
  UART_InitStruct->UART_BaudRate = 115200;
  UART_InitStruct->UART_WordLength = UART_WordLength_8b;
  UART_InitStruct->UART_StopBits = UART_StopBits_1;
  UART_InitStruct->UART_Parity = UART_Parity_No;
  UART_InitStruct->UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
  UART_InitStruct->UART_HardwareFlowControl = UART_HardwareFlowControl_None;
}

/*--------------------------------------------------------------------------------------------
串口使能  /Enable  UART
--------------------------------------------------------------------------------------------*/
void UART_Cmd(UART_TypeDef *UARTx, FunctionalState NewState)
{
  if (NewState != DISABLE)
    UARTx->CR |= UARTEN_ON;
  else
    UARTx->CR &= UARTEN_OFF;
} /*EditStatus:end*/

/*--------------------------------------------------------------------------------------------
接受发送过程中关闭uart 接受或发送完当前字节  /Disable UART while accepting send    Receives or sends the current byte
--------------------------------------------------------------------------------------------*/
void UART_DirectionModeCmd(UART_TypeDef *UARTx, uint32_t UART_DirectionMode, FunctionalState NewState)
{
  /* Check the parameters */
  // assert_param(IS_UART_DIRECTION(UART_DirectionMode));
  // assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != ENABLE)
  {
    UARTx->CR &= ~UART_DirectionMode;
  }
  else
  {
    UARTx->CR |= UART_DirectionMode;
  }
}

/*--------------------------------------------------------------------------------------------
串口发送函数  UART sending function
--------------------------------------------------------------------------------------------*/
/* Data transfers functions ***************************************************/
void UART_SendData(UART_TypeDef *UARTx, uint8_t Data)
{
  UARTx->DR = Data;
}

/*--------------------------------------------------------------------------------------------
串口接受函数  UART receiver function
--------------------------------------------------------------------------------------------*/
uint16_t UART_ReceiveData(UART_TypeDef *UARTx)
{
  uint32_t tmpreg = 0;
  tmpreg = UARTx->DR;
  return (uint16_t)(tmpreg & 0xff);
}
void UART_SendBreak(UART_TypeDef *UARTx)
{
  /* set and cleared by software */
  UARTx->LCR |= UARTLCR_BRK;
}

/*--------------------------------------------------------------------------------------------
串口fifo深度选择  /UART Fifo depth selection
--------------------------------------------------------------------------------------------*/
void UART_FIFOLevelConfig(UART_TypeDef *UARTx, uint32_t RXIFLSelect, uint32_t TXIFLSelect)
{
  // assert_param(IS_UART_FIFO_SELECT(FIFOSelect));
  // assert_param(IS_UART_FIFO(FIFOSelect));
  uint32_t tmpreg = 0;
  tmpreg = UARTx->IFLS;
  tmpreg &= 0xFFFFFFC0;
  tmpreg |= RXIFLSelect;
  tmpreg |= TXIFLSelect;
  UARTx->IFLS = tmpreg;
} /*EditStatus:end&&check*/

/* IrDA mode functions ********************************************************/
void UART_IrDACmd(UART_TypeDef *UARTx, FunctionalState NewState)
{
  if (NewState != ENABLE)
    UARTx->CR &= UART_SIREN_OFF;
  else
    UARTx->CR |= UART_SIREN_ON;
}; /*  (EditStatus:end)*/

/*--------------------------------------------------------------------------------------------
串口DMA使能  /Enable UART DMA
--------------------------------------------------------------------------------------------*/
/* DMA transfers management functions *****************************************/
void UART_DMACmd(UART_TypeDef *UARTx, uint32_t UART_DMAReq, FunctionalState NewState)
{
  if (UART_DMAReq == UART_DMAReq_Rx)
  {
    (NewState != DISABLE) ? (UARTx->DMACR |= UART_DMAReq_Rx) : (UARTx->DMACR &= ~UART_DMAReq_Rx);
  }
  else
  {
    (NewState != DISABLE) ? (UARTx->DMACR |= UART_DMAReq_Tx) : (UARTx->DMACR &= ~UART_DMAReq_Tx);
  }
} /*EditStatus:end*/

/*--------------------------------------------------------------------------------------------
串口中断使能 /UART  interrupt enable
--------------------------------------------------------------------------------------------*/
/* Interrupts and flags management functions **********************************/
void UART_ITConfig(UART_TypeDef *UARTx, uint32_t UART_IT, FunctionalState NewState)
{
  // assert_param(IS_UART_IT(UART_IT));
  if (NewState == ENABLE)
  {
    UARTx->IMSC |= UART_IT;
  }
  else
  {
    UARTx->IMSC &= ~UART_IT;
  }
}

/*--------------------------------------------------------------------------------------------
串口运行标志位读取  / UART Run flag bit read
--------------------------------------------------------------------------------------------*/
FlagStatus UART_GetFlagStatus(UART_TypeDef *UARTx, uint32_t UART_FLAG)
{
  //  uint32_t temp;
  //  temp=UARTx->FR;
  assert_param(IS_UART_STATUSFLAG(UART_FLAG));
  if (UART_FLAG > (uint32_t)0X800)
  {
    if (UARTx->RSR & (UART_FLAG >> 12))
      return SET;
    else
      return RESET;
  }
  else
  {
    if (UARTx->FR & UART_FLAG)
      return SET;
    else
      return RESET;
  }
}

/*--------------------------------------------------------------------------------------------
屏蔽中断状态位读取  /Mask interrupt status bit reads
--------------------------------------------------------------------------------------------*/
ITStatus UART_GetITStatus(UART_TypeDef *UARTx, uint32_t UART_IT)
{
  if (UARTx->MIS & UART_IT)
    return SET;
  else
    return RESET;
}

/*--------------------------------------------------------------------------------------------
原始中断状态读取  /Raw interrupt state read
--------------------------------------------------------------------------------------------*/
ITStatus UART_GetRawITStatus(UART_TypeDef *UARTx, uint32_t UART_IT)
{
  if (UARTx->RIS & UART_IT)
    return SET;
  else
    return RESET;
}

/*--------------------------------------------------------------------------------------------
清中断标志  /Clear interrupt sign
--------------------------------------------------------------------------------------------*/
void UART_ClearITPendingBit(UART_TypeDef *UARTx, uint32_t UART_IT)
{
  UARTx->ICR |= UART_IT;
}

/*--------------------------------------------------------------------------------------------
串口fifo使能 /Enable UART fifo
--------------------------------------------------------------------------------------------*/
void UART_FIFOEnable(UART_TypeDef *UARTx, uint32_t NewState)
{
  if (NewState != DISABLE)
    UARTx->LCR |= 0X10;
  else // default
    UARTx->LCR &= 0XEF;
}
/*--------------------------------------------------------------------------------------------
仅针对从机模式，主机没有硬件逻辑 LIN模式使能,开启 LIN 接收报头检测（检查 13bit 低电平的报头）。
--------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------
Only in slave mode, the master has no hardware logic    Enable the LIN mode,enable LIN receive header detection (check 13bit low headers).
--------------------------------------------------------------------------------------------*/

void UART_LINCmd(UART_TypeDef *UARTx, FunctionalState NewState)
{
  if (NewState != DISABLE)
    UARTx->CR_b.LINEN = 1;
  else
    UARTx->CR_b.LINEN = 0;
}

void UART_SendLINStart(UART_TypeDef *UARTx, uint32_t baud)
{
  RCC_ClocksTypeDef RCC_Clocks;

  uint32_t freq, count, i;

  RCC_GetClockFreq(&RCC_Clocks);
  freq = RCC_Clocks.SYSCLK_Frequency;

  count = freq * 3 / baud;

  UARTx->LCR_b.BRK = 1; //使用break模拟报头/Use break to simulate the header
  while (i++ < count)
    ;
  UARTx->LCR_b.BRK = 0; //使用break模拟报头/Use break to simulate the header
}
