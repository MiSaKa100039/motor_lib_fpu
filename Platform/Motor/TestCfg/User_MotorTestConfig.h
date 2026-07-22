#pragma once

#include "Motor_Features.h"

/*
 * 调试模式只在此文件选择。Usermain、Platform_Motor 和 Platform_MotorTestConfig
 * 都会包含本头文件，因此不能把这些宏仅定义在 Usermain.cpp 中。
 * 正常闭环启动时注释掉全部 USER_MOTOR_TEST_*。
 */
// #define USER_MOTOR_TEST_PWM_MANUAL
// #define USER_MOTOR_TEST_VF_CONTROL
// #define USER_MOTOR_TEST_RL_IDENTIFY
// #define USER_MOTOR_TEST_CURRENT_LOCK
// #define USER_MOTOR_TEST_IF_CONTROL
#define USER_MOTOR_TEST_IF_SMO_OBSERVER
// #define USER_MOTOR_TEST_IF_HFI_OBSERVER 
// #define USER_MOTOR_TEST_HFI_OBSERVER


#if defined(USER_MOTOR_TEST_PWM_MANUAL) + \
    defined(USER_MOTOR_TEST_CURRENT_LOCK) + \
    defined(USER_MOTOR_TEST_VF_CONTROL) + \
    defined(USER_MOTOR_TEST_IF_CONTROL) + \
    defined(USER_MOTOR_TEST_IF_SMO_OBSERVER) + \
    defined(USER_MOTOR_TEST_IF_HFI_OBSERVER) + \
    defined(USER_MOTOR_TEST_HFI_OBSERVER) + \
    defined(USER_MOTOR_TEST_RL_IDENTIFY) > 1
#error "Only one USER_MOTOR_TEST_* option can be selected"
#endif

#if (defined(USER_MOTOR_TEST_PWM_MANUAL) || \
     defined(USER_MOTOR_TEST_CURRENT_LOCK) || \
     defined(USER_MOTOR_TEST_VF_CONTROL) || \
     defined(USER_MOTOR_TEST_IF_CONTROL) || \
     defined(USER_MOTOR_TEST_IF_SMO_OBSERVER) || \
     defined(USER_MOTOR_TEST_IF_HFI_OBSERVER) || \
     defined(USER_MOTOR_TEST_HFI_OBSERVER)) && \
    !LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
#error "Motor debug test requires LIB_MOTOR_ENABLE_DANGEROUS_TEST_API=1 in Platform_MotorFeatures.h"
#endif

#if (defined(USER_MOTOR_TEST_IF_CONTROL) || \
     defined(USER_MOTOR_TEST_IF_SMO_OBSERVER) || \
     defined(USER_MOTOR_TEST_IF_HFI_OBSERVER)) && !LIB_MOTOR_ENABLE_IF_STARTUP
#error "IF debug test requires MOTOR_BUILD_ENABLE_IF_STARTUP in Platform_MotorBuildCfg.h"
#endif

#if defined(USER_MOTOR_TEST_IF_SMO_OBSERVER) && !LIB_MOTOR_ENABLE_SMO
#error "IF+SMO debug test requires MOTOR_BUILD_ENABLE_SMO in Platform_MotorBuildCfg.h"
#endif

#if defined(USER_MOTOR_TEST_IF_HFI_OBSERVER) && !LIB_MOTOR_ENABLE_HFI
#error "IF+HFI debug test requires MOTOR_BUILD_ENABLE_HFI in Platform_MotorBuildCfg.h"
#endif

#if defined(USER_MOTOR_TEST_HFI_OBSERVER) && !LIB_MOTOR_ENABLE_HFI
#error "HFI debug test requires MOTOR_BUILD_ENABLE_HFI in Platform_MotorBuildCfg.h"
#endif

#if defined(USER_MOTOR_TEST_RL_IDENTIFY) && !LIB_MOTOR_ENABLE_AUTO_IDENTIFY
#error "RL identify test requires MOTOR_BUILD_ENABLE_AUTO_IDENTIFY in Platform_MotorBuildCfg.h"
#endif
