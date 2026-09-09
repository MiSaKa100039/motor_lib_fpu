/*
 * BSP_Motor.h 模板
 *
 * BSP 负责 PWM/ADC/定时器的硬件事实，Platform 仅绑定并填写用户参数。
 */

#pragma once

#include "Motor_HAL_Interface.h"

#ifdef __cplusplus
extern "C" {
#endif

const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void);
void BSP_Motor_Hardware_Start(void);

#ifdef __cplusplus
}
#endif

