#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#include "lca039_clock.h"

#ifndef __cplusplus
#include "lcm32f039.h"
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LCA039_BOOT_STATUS_OK = 0,
    LCA039_BOOT_STATUS_DATA_INIT_FAILED = (1U << 0),
    LCA039_BOOT_STATUS_BSS_INIT_FAILED = (1U << 1),
    LCA039_BOOT_STATUS_APPLICATION_INIT_FAILED = (1U << 2),
    LCA039_BOOT_STATUS_ADC_INIT_FAILED = (1U << 3),
    LCA039_BOOT_STATUS_MOTOR_INIT_FAILED = LCA039_BOOT_STATUS_APPLICATION_INIT_FAILED
} LCA039_BootStatus;

extern volatile uint32_t g_heartbeat;
extern volatile LCA039_ClockStatus g_clock_status;
extern volatile LCA039_BootStatus g_boot_status;

extern volatile uint32_t Core_TIM1_UpdateIrqCount;
extern volatile uint32_t Core_TIM1_BreakIrqCount;
extern volatile uint32_t Core_TIM1_LastBreakFlags;
extern volatile uint32_t Core_ADC_PublishedSampleCount;
extern volatile uint32_t Core_ADC_UnexpectedIrqCount;
extern volatile uint16_t Core_ADC_LastRaw;
extern volatile uint16_t Core_ADC_RegularFifoBuffer[5];
extern volatile uint16_t Core_ADC_RegularDmaBuffer[5];
extern volatile uint32_t Core_ADC_RegularDmaWords[5];
extern volatile uint32_t Core_ADC_LastPublishedWords[5];
extern volatile uint32_t Core_ADC_SequenceCompleteCount;
extern volatile uint32_t Core_ADC_OverrunCount;
extern volatile uint32_t Core_ADC_LastIsrFlags;
extern volatile uint32_t Core_ADC_LastDmaIrqFlags;
extern volatile uint32_t Core_ADC_FifoDepthAtEosLast;
extern volatile uint32_t Core_ADC_FifoUnderflowCount;
extern volatile uint32_t Core_ADC_FifoExtraCount;
extern volatile uint32_t Core_ADC_ReadyTimeoutCount;
extern volatile uint32_t Core_ADC_StopTimeoutCount;
extern volatile uint32_t Core_ADC_StartTimeoutCount;
extern volatile uint32_t Core_ADC_DmaTransferCompleteCount;
extern volatile uint32_t Core_ADC_DmaTransferErrorCount;
extern volatile uint32_t Core_ADC_DmaPublishedCount;
extern volatile uint32_t Core_ADC_DmaRawHighBitsCount;
extern volatile uint32_t Core_ADC_DmaMaskedOutOfRangeCount;
extern volatile uint32_t Core_ADC_ResyncCount;
extern volatile uint32_t Core_ADC_ResyncFailureCount;
extern volatile uint32_t Core_ADC_DmaDroppedFirstBlockCount;
extern volatile uint32_t Core_ADC_DmaTcSinceResync;
extern volatile uint32_t Core_ADC_DmaBlockRearmCount;
extern volatile uint32_t Core_ADC_DmaBlockRearmFailureCount;
extern volatile uint32_t Core_ADC_DmaTeResyncCount;

/* USART1 TX DMA 调试计数，只描述底层发送链路，不包含 VOFA 协议语义。 */
extern volatile uint32_t Core_USART1_TxDmaStartedCount;
extern volatile uint32_t Core_USART1_TxDmaCompleteCount;
extern volatile uint32_t Core_USART1_TxDmaErrorCount;
extern volatile uint32_t Core_USART1_ActualBaudRate;
extern volatile uint32_t Core_USART1_BaudErrorPpm;

/* CubeMX 风格的项目级外设初始化入口，具体寄存器配置由 Core/Src/main.c 内部持有。 */
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);
void MX_OPA_Init(void);
void MX_DAC_Init(void);
void MX_ACMP_Init(void);
void MX_TIM1_Init(void);
void MX_ADC_Init(void);

bool Core_PeripheralsReady(void);
bool Core_PowerBridgeBindingEnabled(void);
bool Core_ADC_HandleConversionIrq(void);
bool Core_ADC_HandleDmaIrq(void);
void Core_TIM1_HandleUpdateIrq(void);
uint16_t Core_ADC_GetLastRaw(void);
uint16_t Core_ADC_GetPhaseURaw(void);
uint16_t Core_ADC_GetPhaseVRaw(void);
uint16_t Core_ADC_GetBusCurrentRaw(void);
uint16_t Core_ADC_GetVbusRaw(void);
uint16_t Core_ADC_GetTemp0Raw(void);

void Core_TIM1_DisablePwmOutputs(void);
void Core_TIM1_EnablePwmOutputs(void);
void Core_TIM1_BrakeLowOutputs(void);
void Core_TIM1_SetPwmCompare(uint16_t phaseU, uint16_t phaseV, uint16_t phaseW);
bool Core_TIM1_PwmOutputsEnabled(void);
uint16_t Core_TIM1_PwmPeriodCounts(void);
uint16_t Core_TIM1_AdcTriggerCompareCounts(void);
uint32_t Core_TIM1_PwmFrequencyHz(void);
uint32_t Core_TIM1_AdcTriggerFrequencyHz(void);

/* DMA busy 期间调用方必须保持 data 指向的发送缓冲有效，Core 不复制协议帧。 */
void Core_USART1_TxDmaResetRuntime(void);
bool Core_USART1_TxDmaBusy(void);
bool Core_USART1_TxDmaTryStart(const uint8_t *data, uint16_t length);
void Core_USART1_TxDmaIrqHandler(void);

bool Core_SysTick_ConfigureHz(uint32_t frequencyHz);
bool Core_TIM1_StartUpdateInterruptHz(uint32_t frequencyHz);
bool Core_ADC_SelectSoftwareTrigger(void);
bool Core_ADC_StartSoftwareConversion(void);
bool Core_TIM1_StartAdcTriggerPath(void);
bool Core_TIM1_ResyncAdcTriggerPath(void);
void Core_TIM1_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
