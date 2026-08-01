#ifndef LCM32F039_IT_H
#define LCM32F039_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void ADC_Handler(void);
void TIM1_NON_CC_Handler(void);
void DMAC_CH0_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif