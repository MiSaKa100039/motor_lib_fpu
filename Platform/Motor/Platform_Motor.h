#pragma once

#include "Motor.h"

extern Lib_Motor::MotorAPI Global_Motor_0;

Lib_Motor::Result Platform_Motor_Init(void);
Lib_Motor::Result Platform_Motor_ResetRuntimeAndSampling(void);
const Lib_Motor::MotorIFStartupProfile& Platform_Motor_GetIFStartupProfile(void);
