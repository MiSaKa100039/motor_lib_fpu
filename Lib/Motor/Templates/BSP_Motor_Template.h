#pragma once

#ifdef __cplusplus

#include "Motor_HAL_Interface.h"

extern "C" {

const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void);

}

#endif