#pragma once

#include "Motor.h"

/* ===================== 调试/测试变量声明 ===================== */

/*
 * PWM 手动测试模式 (MOTOR_BUILD_ENABLE_DEBUG_PWM_MANUAL):
 *   绕过所有控制环, 直接设定三相占空比。
 *   用于验证: PWM 输出极性、死区、硬件功率级是否正常。
 *   ⚠ 只有确认过线上电压探头可用及限流阈值已降低后才可上功率供电测试。
 */
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
extern volatile float Platform_TestPWM_U;
extern volatile float Platform_TestPWM_V;
extern volatile float Platform_TestPWM_W;
#endif

/*
 * 电流锁定测试 (MOTOR_BUILD_ENABLE_DEBUG_CURRENT_LOCK):
 *   Id/Iq 电流环独立闭环, 电机锁轴不动。
 *   用于验证: 电流采样/方向/增益、Park/Clark 变换、电流 PI 整定。
 *   ⚠ Id=0+Iq=小电流 → 观察锁轴是否稳定, 若有抖动说明角度/采样方向错误。
 */
#if LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
extern volatile float Platform_TestCurrentLockId;
extern volatile float Platform_TestCurrentLockIq;
#endif

/*
 * VF 开环拖动 (MOTOR_BUILD_ENABLE_DEBUG_VF_CONTROL):
 *   固定 duty + 斜坡频率, 开环拖动转子旋转。
 *   用于验证: 霍尔扇区顺序、极对数、旋转方向、速度估算。
 */
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
extern volatile Lib_Motor::MotorVFStartupPhase Platform_TestVFStartupPhases[];
extern Lib_Motor::MotorVFStartupProfile Platform_TestVFStartupProfile;
#endif

