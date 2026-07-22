/*
 * BSP Motor template.
 *
 * Keep CubeMX peripheral initialization, ADC/TIM start sequencing, and HAL
 * interrupt callbacks in Core. This file should only bind MotorHAL_t callbacks
 * to board-level raw IO such as PWM compare registers and ADC sample reads.
 */

#include "BSP_Motor.h"

/* Include the board HAL header from your project. */
// #include "main.h"

/* Core owns regular ADC DMA buffers and hardware startup state. If BSP needs
 * regular ADC samples, read the Core-owned raw buffers or project-specific Core
 * accessors instead of starting DMA here.
 */
// extern TIM_HandleTypeDef htim1;
// extern ADC_HandleTypeDef hadc1;
// extern ADC_HandleTypeDef hadc2;
// extern volatile uint16_t Core_Motor_Adc1RegularBuffer[1];
// extern volatile uint16_t Core_Motor_Adc2RegularBuffer[1];

namespace
{

// static void BSP_SetPWM_Impl(float u, float v, float w)
// {
//     const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, static_cast<uint32_t>(u * period));
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, static_cast<uint32_t>(v * period));
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, static_cast<uint32_t>(w * period));
// }

// static void BSP_PWM_Enable_Impl(void)
// {
//     /* Prefer enabling already-configured power channels and MOE here.
//      * Core should have started the timer/ADC trigger before motor run.
//      */
// }

// static void BSP_PWM_Disable_Impl(void)
// {
//     /* Disable only the power PWM outputs needed by this board. */
// }

// static void BSP_ReadCurrent_Raw_Impl(uint16_t* raw_u, uint16_t* raw_v,
//                                      uint16_t* raw_w, uint16_t* raw_bus)
// {
//     *raw_u = static_cast<uint16_t>(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1));
//     *raw_v = static_cast<uint16_t>(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1));
//     *raw_w = 0U;
//     *raw_bus = 0U;
// }

// static uint16_t BSP_ReadVbus_Raw_Impl(void)
// {
//     return Core_Motor_Adc2RegularBuffer[0];
// }

// static uint16_t BSP_ReadTemp_Raw_Impl(uint8_t index)
// {
//     (void)index;
//     return Core_Motor_Adc1RegularBuffer[0];
// }

// static Lib_Motor::MotorHAL_t bsp_hal_impl =
// {
//     .set_duty = BSP_SetPWM_Impl,
//     .pwm_enable = BSP_PWM_Enable_Impl,
//     .pwm_disable = BSP_PWM_Disable_Impl,
//     .pwm_coast = BSP_PWM_Disable_Impl,
//     .pwm_brake_lowside = nullptr,
//     .brake_resistor_set_duty = nullptr,
//     .brake_resistor_disable = nullptr,
//     .mechanical_brake_engage = nullptr,
//     .mechanical_brake_release = nullptr,
//     .read_currents_raw = BSP_ReadCurrent_Raw_Impl,
//     .phase_current_valid_mask =
//         Lib_Motor::MOTOR_PHASE_CURRENT_U_VALID | Lib_Motor::MOTOR_PHASE_CURRENT_V_VALID,
//     .read_phase_voltages_raw = nullptr,
//     .read_vbus_raw = BSP_ReadVbus_Raw_Impl,
//     .read_temp_raw = BSP_ReadTemp_Raw_Impl,
//     .enter_critical = __disable_irq,
//     .exit_critical = __enable_irq,
//     .flash_read = nullptr,
//     .flash_write = nullptr,
//     .flash_erase = nullptr,
//     .flash_sector_size = 0U,
// };

} // namespace

// const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void)
// {
//     return &bsp_hal_impl;
// }