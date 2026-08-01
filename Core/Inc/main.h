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
    LCA039_BOOT_STATUS_MOTOR_INIT_FAILED = LCA039_BOOT_STATUS_APPLICATION_INIT_FAILED
} LCA039_BootStatus;

extern volatile uint32_t g_heartbeat;
extern volatile LCA039_ClockStatus g_clock_status;
extern volatile LCA039_BootStatus g_boot_status;

extern volatile uint32_t Core_Motor_Tim1UpdateIrqCount;
extern volatile uint32_t Core_Motor_AdcEocIrqCount;
extern volatile uint32_t Core_Motor_AdcUnexpectedIrqCount;
extern volatile uint16_t Core_Motor_AdcLastRaw;
extern volatile uint16_t Core_Motor_AdcTriggerPairRawFirst;
extern volatile uint16_t Core_Motor_AdcTriggerPairRawSecond;

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
void MX_TIM1_Init(void);
void MX_ADC_Init(void);

bool Core_Motor_HardwareInitialize(void);
bool Core_Motor_HardwareStart(void);
bool Core_Motor_HardwareInitialized(void);
bool Core_Motor_HardwareBindingEnabled(void);
bool Core_Motor_HandleAdcIrq(void);
void Core_Motor_HandleTim1Irq(void);
uint16_t Core_ADC_GetLastRaw(void);
uint16_t Core_ADC_GetVbusRaw(void);
uint16_t Core_ADC_GetTempRaw(uint8_t index);
void Core_ADC_GetTriggerPairRaw(uint16_t *raw_first, uint16_t *raw_second);

void Core_TIM1_DisablePwmOutputs(void);
void Core_TIM1_SetPwmCompare(uint16_t phaseU, uint16_t phaseV, uint16_t phaseW);
void Core_TIM1_SetAdcTriggerPairDuties(float first_trigger_duty,
                                       float second_trigger_duty);
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

bool Core_Motor_TestConfigureSysTick(uint32_t frequencyHz);
bool Core_Motor_TestStartTimerUpdate(uint32_t frequencyHz);
bool Core_Motor_TestStartAdcSoftware(void);
bool Core_Motor_TestTriggerAdcSoftware(void);
bool Core_Motor_TestStartTimerAdc(void);
void Core_Motor_TestStopTimer(void);
void Core_ADC_EnableTriggerPairMode(bool enabled);

#ifdef __cplusplus
}
#endif

#endif
