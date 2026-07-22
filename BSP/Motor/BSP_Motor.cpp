#include "BSP_Motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern volatile uint16_t Core_Motor_Adc1RegularBuffer[1];
extern volatile uint16_t Core_Motor_Adc2RegularBuffer[1];

namespace
{

constexpr uint32_t BSP_MOTOR_POWER_PWM_CCER_MASK =
    (TIM_CCER_CC1E | TIM_CCER_CC1NE |
     TIM_CCER_CC2E | TIM_CCER_CC2NE |
     TIM_CCER_CC3E | TIM_CCER_CC3NE);

void BSP_SetPWM_Impl(float u, float v, float w)
{
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);

    if (u > 1.0f) { u = 1.0f; } else if (u < 0.0f) { u = 0.0f; }
    if (v > 1.0f) { v = 1.0f; } else if (v < 0.0f) { v = 0.0f; }
    if (w > 1.0f) { w = 1.0f; } else if (w < 0.0f) { w = 0.0f; }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, static_cast<uint32_t>(u * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, static_cast<uint32_t>(v * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, static_cast<uint32_t>(w * period));
}

void BSP_PWM_PreEnablePowerChannels_Impl(void)
{
    htim1.Instance->CCER |= BSP_MOTOR_POWER_PWM_CCER_MASK;
}

void BSP_PWM_DisablePowerChannels_Impl(void)
{
    htim1.Instance->CCER &= ~BSP_MOTOR_POWER_PWM_CCER_MASK;
}

void BSP_PWM_Enable_Impl(void)
{
    BSP_PWM_PreEnablePowerChannels_Impl();
    __HAL_TIM_MOE_ENABLE(&htim1);
}

void BSP_PWM_Disable_Impl(void)
{
    BSP_PWM_DisablePowerChannels_Impl();
}

void BSP_PWM_Coast_Impl(void)
{
    BSP_PWM_Disable_Impl();
}

void BSP_PWM_BrakeLow_Impl(void)
{
    BSP_PWM_PreEnablePowerChannels_Impl();
    if ((htim1.Instance->BDTR & TIM_BDTR_MOE) == 0UL)
    {
        __HAL_TIM_MOE_ENABLE(&htim1);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
}

void BSP_ReadCurrent_Raw_Impl(uint16_t* raw_u, uint16_t* raw_v, uint16_t* raw_w, uint16_t* raw_bus)
{
    *raw_u = static_cast<uint16_t>(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1));
    *raw_v = static_cast<uint16_t>(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1));
    *raw_w = 0U;
    *raw_bus = 0U;
}

uint16_t BSP_ReadVbus_Raw_Impl(void)
{
    return Core_Motor_Adc2RegularBuffer[0];
}

uint16_t BSP_ReadTemp_Raw_Impl(uint8_t index)
{
    if (index == 0U)
    {
        return Core_Motor_Adc1RegularBuffer[0];
    }

    return 0U;
}

Lib_Motor::MotorHAL_t bsp_hal_impl =
{
    .set_duty = BSP_SetPWM_Impl,
    .pwm_enable = BSP_PWM_Enable_Impl,
    .pwm_disable = BSP_PWM_Disable_Impl,
    .pwm_coast = BSP_PWM_Coast_Impl,
    .pwm_brake_lowside = BSP_PWM_BrakeLow_Impl,
    .brake_resistor_set_duty = nullptr,
    .brake_resistor_disable = nullptr,
    .mechanical_brake_engage = nullptr,
    .mechanical_brake_release = nullptr,
    .read_currents_raw = BSP_ReadCurrent_Raw_Impl,
    .phase_current_valid_mask =
        Lib_Motor::MOTOR_PHASE_CURRENT_U_VALID | Lib_Motor::MOTOR_PHASE_CURRENT_V_VALID,
    .read_phase_voltages_raw = nullptr,
    .read_vbus_raw = BSP_ReadVbus_Raw_Impl,
    .read_temp_raw = BSP_ReadTemp_Raw_Impl,
    .enter_critical = __disable_irq,
    .exit_critical = __enable_irq,
    .flash_read = nullptr,
    .flash_write = nullptr,
    .flash_erase = nullptr,
    .flash_sector_size = 0U,
};

} // namespace

const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void)
{
    return &bsp_hal_impl;
}