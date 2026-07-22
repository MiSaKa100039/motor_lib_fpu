/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Platform_MotorFeatures.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint32_t Core_Motor_InjectedAdc1CallbackCount;
#if PLATFORM_MOTOR_ENABLE_TICK_PROFILING
extern volatile uint32_t Core_Motor_InjectedAdc1CallbackRateHz;
extern volatile uint32_t Core_Motor_TickBudgetCycles;
extern volatile uint32_t Core_Motor_TickCoreCyclesLast;
extern volatile uint32_t Core_Motor_TickCoreCyclesMax;
extern volatile uint32_t Core_Motor_TickIsrCyclesLast;
extern volatile uint32_t Core_Motor_TickIsrCyclesMax;
extern volatile uint32_t Core_Motor_TickOverBudgetCount;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Motor_Global_Process_Handler(int motor_id);
void Core_Motor_MarkTelemetryPendingFromIsr(void);
#if PLATFORM_MOTOR_ENABLE_TICK_PROFILING
static void Core_Motor_UpdateInjectedCallbackRate(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if PLATFORM_MOTOR_ENABLE_TICK_PROFILING
static uint32_t g_core_motor_callback_rate_window_start_ms = 0U;
static uint32_t g_core_motor_callback_rate_window_count = 0U;

static void Core_Motor_UpdateInjectedCallbackRate(void)
{
  const uint32_t now_ms = HAL_GetTick();
  if (g_core_motor_callback_rate_window_start_ms == 0U)
  {
    g_core_motor_callback_rate_window_start_ms = now_ms;
    g_core_motor_callback_rate_window_count = 0U;
  }

  ++g_core_motor_callback_rate_window_count;
  const uint32_t elapsed_ms = now_ms - g_core_motor_callback_rate_window_start_ms;
  if (elapsed_ms >= 1000U)
  {
    Core_Motor_InjectedAdc1CallbackRateHz =
        (g_core_motor_callback_rate_window_count * 1000U + elapsed_ms / 2U) / elapsed_ms;
    g_core_motor_callback_rate_window_start_ms = now_ms;
    g_core_motor_callback_rate_window_count = 0U;
  }
}
#endif
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */

    // ========================================================
    // [救命代码] 无论发生什么，立即切断 PWM 输出！
    // ========================================================

    // 1. 强制关闭定时器主输出 (MOE bit)
    // 假设你的电机用的是 TIM1
    if (TIM1) {
      TIM1->BDTR &= ~(TIM_BDTR_MOE); // 直接操作寄存器最快最安全
      TIM1->CR1  &= ~(TIM_CR1_CEN);  // 停止计数
    }

    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance != ADC1)
  {
    return;
  }

#if PLATFORM_MOTOR_ENABLE_TICK_PROFILING
  Core_Motor_UpdateInjectedCallbackRate();
#endif
  ++Core_Motor_InjectedAdc1CallbackCount;

#if PLATFORM_MOTOR_ENABLE_TICK_PROFILING
  const uint32_t isr_begin = DWT->CYCCNT;
  const uint32_t core_begin = DWT->CYCCNT;
  Motor_Global_Process_Handler(0);
  const uint32_t core_cycles = DWT->CYCCNT - core_begin;
  Core_Motor_TickCoreCyclesLast = core_cycles;
  if (core_cycles > Core_Motor_TickCoreCyclesMax)
  {
    Core_Motor_TickCoreCyclesMax = core_cycles;
  }
#else
  Motor_Global_Process_Handler(0);
#endif

  static uint8_t decim = 0U;
  if (++decim >= 40U)
  {
    decim = 0U;
    Core_Motor_MarkTelemetryPendingFromIsr();
  }

#if PLATFORM_MOTOR_ENABLE_TICK_PROFILING
  const uint32_t isr_cycles = DWT->CYCCNT - isr_begin;
  Core_Motor_TickIsrCyclesLast = isr_cycles;
  if (isr_cycles > Core_Motor_TickIsrCyclesMax)
  {
    Core_Motor_TickIsrCyclesMax = isr_cycles;
  }
  if (Core_Motor_TickBudgetCycles > 0U && isr_cycles > Core_Motor_TickBudgetCycles)
  {
    ++Core_Motor_TickOverBudgetCount;
  }
#endif
}
/* USER CODE END 1 */
