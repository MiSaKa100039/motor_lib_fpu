/* ============================================================
 * BSP Motor 实现模板
 *
 * 用户需要实现 MotorHAL_t 结构体中的所有回调和相电流有效掩码，
 * 然后通过 BSP_Get_HAL_Impl() 返回给 Platform 层绑定。
 *
 * 必须实现的接口与字段：
 *   1. set_duty(float u, float v, float w) — 三相占空比 [0.0, 1.0]
 *   2. pwm_enable()  — 使能 PWM 输出 (MOE=1)
 *   3. pwm_disable() — 禁止 PWM 输出 (MOE=0, Hi-Z)
 *   4. read_currents_raw(...) — 固定按物理 U/V/W 返回 ADC 原始值
 *   5. phase_current_valid_mask — 声明实际安装的相电流采样通道
 *   6. read_vbus_raw() — 读母线电压 ADC 原始值
 *   7. read_temp_raw(index) — 按索引读取温度 ADC 原始值
 *   8. enter_critical() / exit_critical() — 临界区
 *
 * 额外必须实现：
 *   9. BSP_Motor_Hardware_Start() — 启动 ADC、PWM 定时器、使能中断
 *  10. 在 ADC 注入组完成中断中调用 Motor_Global_Process_Handler(motor_id)
 * ============================================================ */

#include "BSP_Motor.h"
#include "Motor_API.h"

/* === 包含你的 HAL 头文件 === */
// #include "main.h"
// #include "tim.h"
// #include "adc.h"

/* ============================================================
 * [0] 硬件句柄声明 (根据你的 CubeMX 配置)
 * ============================================================ */
// extern TIM_HandleTypeDef htim1;
// extern ADC_HandleTypeDef hadc1;
// extern ADC_HandleTypeDef hadc2;
// volatile uint16_t adc_regular_buffer[2];  // DMA 双缓冲

/* ============================================================
 * [1] 接口实现
 * ============================================================ */

// static void BSP_SetPWM_Impl(float u, float v, float w)
// {
//     uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(u * period));
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(v * period));
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(w * period));
// }

// static void BSP_PWM_Enable_Impl(void)
// {
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//     __HAL_TIM_MOE_ENABLE(&htim1);
// }

// static void BSP_PWM_Disable_Impl(void)
// {
//     __HAL_TIM_MOE_DISABLE(&htim1);
//     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
// }

/*
 * Current feedback contract:
 *   - This BSP function must return physical U/V/W currents in fixed positions.
 *   - Map ADC peripheral/channel/rank to U/V/W only in BSP, not in Platform.
 *   - Keep phase_current_valid_mask below consistent with real installed sensors.
 *   - For low-side shunts, trigger ADC only inside a valid low-side conduction window.
 *     Inline shunt/Hall phase sensors retain the same polarity rule but are less
 *     dependent on the PWM low-side conduction window.
 */
// static void BSP_ReadCurrent_Raw_Impl(uint16_t* raw_u, uint16_t* raw_v, uint16_t* raw_w, uint16_t* raw_bus)
// {
//     *raw_u   = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//     *raw_v   = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//     *raw_w   = 0U; // U/V 双采样示例: W 由库按 KCL 重建
//     *raw_bus = 0U; // 未安装母线电流传感器时返回占位值
// }

// static uint16_t BSP_ReadVbus_Raw_Impl(void)
// {
//     return adc_regular_buffer[0];
// }

// static uint16_t BSP_ReadTemp_Raw_Impl(uint8_t index)
// {
//     return adc_regular_buffer[1 + index];
// }

/* ============================================================
 * [2] 接口绑定
 * ============================================================ */

// static Lib_Motor::MotorHAL_t bsp_hal_impl =
// {
//     .set_duty          = BSP_SetPWM_Impl,
//     .pwm_enable        = BSP_PWM_Enable_Impl,
//     .pwm_disable       = BSP_PWM_Disable_Impl,
//     .pwm_coast         = BSP_PWM_Coast_Impl,
//     .pwm_brake_lowside = BSP_PWM_BrakeLow_Impl,
//     .brake_resistor_set_duty = nullptr,
//     .brake_resistor_disable = nullptr,
//     .mechanical_brake_engage = nullptr,
//     .mechanical_brake_release = nullptr,
//     .read_currents_raw = BSP_ReadCurrent_Raw_Impl,
//     // Match the physical phase ADC channels returned above: U/V, U/W, V/W, or ALL.
//     .phase_current_valid_mask =
//         Lib_Motor::MOTOR_PHASE_CURRENT_U_VALID | Lib_Motor::MOTOR_PHASE_CURRENT_V_VALID,
//     .read_vbus_raw     = BSP_ReadVbus_Raw_Impl,
//     .read_temp_raw     = BSP_ReadTemp_Raw_Impl,
//     .enter_critical    = __disable_irq,
//     .exit_critical     = __enable_irq,
// };

// const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void)
// {
//     return &bsp_hal_impl;
// }

/* ============================================================
 * [3] 硬件启动
 * ============================================================ */

// void BSP_Motor_Hardware_Start(void)
// {
//     HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//     HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
//
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
//         __HAL_TIM_GET_AUTORELOAD(&htim1) - 1);
//
//     HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_regular_buffer, 2);
//     HAL_ADCEx_InjectedStart_IT(&hadc1);
//     HAL_ADCEx_InjectedStart(&hadc2);
//
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//     __HAL_TIM_ENABLE(&htim1);
// }

/* ============================================================
 * [4] ADC 注入组完成中断回调
 * ============================================================ */

// extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
// {
//     if (hadc->Instance == ADC1)
//     {
//         Motor_Global_Process_Handler(0);
//     }
// }
