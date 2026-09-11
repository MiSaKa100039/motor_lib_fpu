#include "Platform_Motor.h"
#include "BSP_Motor.h"
#include "Platform_MotorFeatures.h"

#include "MotorCfg/Platform_MotorConfig1_Basic.h"
#include "MotorCfg/Platform_MotorConfig2_FeedbackStartup.h"
#include "MotorCfg/Platform_MotorConfig3_Control.h"
#include "MotorCfg/Platform_MotorConfig4_MotionSafety.h"
#include "MotorCfg/Platform_MotorConfig5_BrakeCommand.h"

/* BuildCfg: 数学加速后端宏 */
#include "Platform_MotorBuildCfg.h"
/* FocMath: foc_math_cordic_init (CORDIC 后端时) */
#include "FocMath.h"

Lib_Motor::MotorAPI Global_Motor_0;

namespace
{

bool g_platform_motor_hardware_started = false;

Lib_Motor::Mode Platform_UserMotorTestStartupMode()
{
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
    return Lib_Motor::Mode::DEBUG_PWM_MANUAL;
#elif LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
    return Lib_Motor::Mode::DEBUG_CURRENT_LOCK;
#elif LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    return Lib_Motor::Mode::DEBUG_VF_DRAG;
#elif LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
    return Lib_Motor::Mode::DEBUG_IF_DRAG;
#elif LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
    return Lib_Motor::Mode::DEBUG_IF_SMO_OBSERVER;
#elif LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
    return Lib_Motor::Mode::DEBUG_IF_HFI_OBSERVER;
#elif LIB_MOTOR_ENABLE_DEBUG_HFI_OBSERVER
    return Lib_Motor::Mode::DEBUG_HFI_OBSERVER;
#else
    return Lib_Motor::Mode::NONE;
#endif
}

void Platform_StartMotorHardwareOnce()
{
    if (g_platform_motor_hardware_started)
    {
        return;
    }

    BSP_Motor_Hardware_Start();
    g_platform_motor_hardware_started = true;
}

}

Lib_Motor::Result Platform_Motor_Init(void)
{
    static Lib_Motor::MotorConfig cfg;
    cfg = Lib_Motor::MotorConfig{};

    /* CORDIC 硬件时钟使能 (CubeMX 的 MX_CORDIC_Init 也会开, 此处冗余保障:
     * 即便调用顺序变化, CORDIC 仍可用) */
#if defined(MOTOR_BUILD_ENABLE_CORDIC_ACCEL)
    Lib_Motor::foc_math_cordic_init();
#endif

    cfg.hal = BSP_Get_HAL_Impl();

    Platform_MotorConfig::ApplyBasicConfig(cfg);
    Platform_MotorConfig::ApplyMotionSafetyConfig(cfg);
    Platform_MotorConfig::ApplyFeedbackAndStartupConfig(cfg);
    Platform_MotorConfig::ApplyBrakeAndCommandConfig(cfg);
    Platform_MotorConfig::ApplyControlConfig(cfg);

    /* A selected bench test preloads the target mode during init. CMD=1 still
     * calls the matching debug/calib API to install test parameters and start.
     */
    const Lib_Motor::Mode test_startup_mode = Platform_UserMotorTestStartupMode();
    if (test_startup_mode != Lib_Motor::Mode::NONE)
    {
        cfg.control.startup_target_mode = test_startup_mode;
    }

    BSP_Motor_SetControlFrequencyHz(cfg.control.control_freq_hz);

    const Lib_Motor::Result init_result = Global_Motor_0.init(cfg);
    /*
     * 普通固件只在配置校验成功后启动 TIM/ADC。
     * 配置错误已由 Manager 锁存到 ERROR，继续跑采样链会用未校准 offset
     * 叠加软件过流等二次故障，反而掩盖第一现场。
     */
#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
    if (Global_Motor_0.getState() != Lib_Motor::State::UNINITIALIZED)
#else
    if (init_result == Lib_Motor::Result::Ok)
#endif
    {
        Platform_StartMotorHardwareOnce();
    }
    return init_result;
}

Lib_Motor::Result Platform_Motor_ResetRuntimeAndSampling(void)
{
    if (!BSP_Motor_ResyncSamplingPath())
    {
        g_platform_motor_hardware_started = false;
        return Lib_Motor::Result::Error;
    }

    g_platform_motor_hardware_started = true;
    return Global_Motor_0.reset();
}
