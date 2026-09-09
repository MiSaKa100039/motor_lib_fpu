#include "Motor_FeatureRegistry.h"

namespace Lib_Motor
{

bool MotorFeatureRegistry::supportsStartupSource(StartupSource source)
{
    switch (source)
    {
        case StartupSource::SENSOR:
            return LIB_MOTOR_ENABLE_SENSOR != 0;
        case StartupSource::IF:
            return LIB_MOTOR_ENABLE_IF_STARTUP != 0;
        case StartupSource::HFI:
            return LIB_MOTOR_ENABLE_HFI != 0;
        default:
            return false;
    }
}

bool MotorFeatureRegistry::supportsSteadySource(SteadyAngleSource source)
{
    switch (source)
    {
        case SteadyAngleSource::SENSOR:
            return LIB_MOTOR_ENABLE_SENSOR != 0;
        case SteadyAngleSource::SMO:
            return LIB_MOTOR_ENABLE_SMO != 0;
        case SteadyAngleSource::HFI:
            return LIB_MOTOR_ENABLE_HFI != 0;
        case SteadyAngleSource::NONLINEAR_FLUX:
            return LIB_MOTOR_ENABLE_NONLINEAR_FLUX != 0;
        case SteadyAngleSource::LUENBERGER:
            return LIB_MOTOR_ENABLE_LUENBERGER != 0;
        case SteadyAngleSource::EKF:
            return LIB_MOTOR_ENABLE_EKF != 0;
        default:
            return false;
    }
}

bool MotorFeatureRegistry::supportsRunPolicy(const MotorRunPolicy& policy)
{
    if (!supportsStartupSource(policy.startup_source) ||
        !supportsSteadySource(policy.steady_source))
    {
        return false;
    }

    return true;
}

bool MotorFeatureRegistry::supportsMode(Mode mode)
{
    switch (mode)
    {
        case Mode::NONE:
            return true;

        case Mode::CALIB_RL_IDENTIFY:
            return LIB_MOTOR_ENABLE_AUTO_IDENTIFY != 0;

#if LIB_MOTOR_ENABLE_TORQUE_CONTROL
        case Mode::TORQUE_CONTROL:
            return true;
#endif

#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
        case Mode::VELOCITY_CONTROL:
            return true;
#endif

#if LIB_MOTOR_ENABLE_POSITION_CONTROL
        case Mode::POSITION_CONTROL:
            return true;
#endif

#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
        case Mode::IMPEDANCE_CONTROL:
            /* Phase 3 接通点。Phase 2 BuildCfg 默认关闭, 即使打开也仅占位;
             * 控制环实际行为见 computeIqReference 的占位分支。
             * TODO HostTest: BuildCfg 关 IMPEDANCE 时 supportsMode(IMPEDANCE_CONTROL) 应返回 false,
             *                开启时应返回 true; 当前依赖 compile-time #if 控制, 暂无运行期验证。 */
            return true;
#endif

        case Mode::DEBUG_PWM_MANUAL:
            return LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL != 0;

        case Mode::DEBUG_CURRENT_LOCK:
            return LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK != 0;

        case Mode::DEBUG_VF_DRAG:
            return LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL != 0;

        case Mode::DEBUG_HFI_OBSERVER:
            return LIB_MOTOR_ENABLE_DEBUG_HFI_OBSERVER != 0;

        case Mode::DEBUG_IF_DRAG:
            return LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL != 0;

        case Mode::DEBUG_IF_SMO_OBSERVER:
            return LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER != 0;

        case Mode::DEBUG_IF_HFI_OBSERVER:
            return LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER != 0;

        default:
            return false;
    }
}

bool MotorFeatureRegistry::supportsStallProtection()
{
    return LIB_MOTOR_ENABLE_STALL_PROTECTION != 0;
}

bool MotorFeatureRegistry::supportsDangerousTestApi()
{
    return LIB_MOTOR_ENABLE_DEBUG_ANY != 0;
}

bool MotorFeatureRegistry::supportsBrakeEnergyFsm()
{
    return LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM != 0;
}

bool MotorFeatureRegistry::supportsBatteryRegenPath()
{
    return LIB_MOTOR_ENABLE_BATTERY_REGEN_PATH != 0;
}

bool MotorFeatureRegistry::supportsBrakeResistor()
{
    return LIB_MOTOR_ENABLE_BRAKE_RESISTOR != 0;
}

bool MotorFeatureRegistry::supportsBrakeChopperControl()
{
    return LIB_MOTOR_ENABLE_BRAKE_CHOPPER_CONTROL != 0;
}

bool MotorFeatureRegistry::supportsMechanicalBrake()
{
    return LIB_MOTOR_ENABLE_MECHANICAL_BRAKE != 0;
}

bool MotorFeatureRegistry::isClosedLoopSourceImplemented(SteadyAngleSource source)
{
    switch (source)
    {
        case SteadyAngleSource::SENSOR:
            return LIB_MOTOR_ENABLE_SENSOR != 0;
        case SteadyAngleSource::SMO:
            return LIB_MOTOR_ENABLE_SMO != 0;
        case SteadyAngleSource::HFI:
        case SteadyAngleSource::NONLINEAR_FLUX:
        case SteadyAngleSource::LUENBERGER:
        case SteadyAngleSource::EKF:
        default:
            return false;
    }
}

bool MotorFeatureRegistry::isStartupHandoverImplemented(StartupSource startup,
                                                        SteadyAngleSource steady)
{
    if (!supportsStartupSource(startup) || !supportsSteadySource(steady))
    {
        return false;
    }

    if (startup == StartupSource::SENSOR && steady == SteadyAngleSource::SENSOR)
    {
        return true;
    }
    if (startup == StartupSource::IF && steady == SteadyAngleSource::SMO)
    {
        return true;
    }

    return false;
}

} // namespace Lib_Motor
