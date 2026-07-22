#pragma once

#include "Motor_Config.h"
#include "Platform_MotorFeatures.h"
#include "Platform_MotorBuildCfg.h"

/*
 * Platform_MotorBuildConfig -- 编译能力查询接口
 *
 * 提供 constexpr 函数, 供 ConfigCheck / MotorCfg 在编译期查询当前固件
 * 是否支持某项能力。所有函数均基于 BuildCfg 宏展开, 零运行时开销。
 */

namespace Platform_MotorBuildConfig
{

/*
 * supports(StartupSource) -- 启动角度源是否已编译
 *   SENSOR: 有感启动 (需 MOTOR_BUILD_ENABLE_SENSOR)
 *   IF:     IF 开环启动 (需 MOTOR_BUILD_ENABLE_IF_STARTUP)
 *   HFI:    高频注入启动 (需 MOTOR_BUILD_ENABLE_HFI)
 */
constexpr bool supports(Lib_Motor::StartupSource source)
{
    switch (source)
    {
        case Lib_Motor::StartupSource::SENSOR:
            return LIB_MOTOR_ENABLE_SENSOR != 0;
        case Lib_Motor::StartupSource::IF:
            return LIB_MOTOR_ENABLE_IF_STARTUP != 0;
        case Lib_Motor::StartupSource::HFI:
            return LIB_MOTOR_ENABLE_HFI != 0;
        default:
            return false;
    }
}

/*
 * supports(SteadyAngleSource) -- 稳态角度源是否已编译
 *   SENSOR:         有感 (编码器/HALL)
 *   SMO:            滑模观测器
 *   HFI:            高频注入
 *   NONLINEAR_FLUX: 非线性磁链观测器
 *   LUENBERGER:     龙伯格观测器
 *   EKF:            扩展卡尔曼滤波器
 */
constexpr bool supports(Lib_Motor::SteadyAngleSource source)
{
    switch (source)
    {
        case Lib_Motor::SteadyAngleSource::SENSOR:
            return LIB_MOTOR_ENABLE_SENSOR != 0;
        case Lib_Motor::SteadyAngleSource::SMO:
            return LIB_MOTOR_ENABLE_SMO != 0;
        case Lib_Motor::SteadyAngleSource::HFI:
            return LIB_MOTOR_ENABLE_HFI != 0;
        case Lib_Motor::SteadyAngleSource::NONLINEAR_FLUX:
            return LIB_MOTOR_ENABLE_NONLINEAR_FLUX != 0;
        case Lib_Motor::SteadyAngleSource::LUENBERGER:
            return LIB_MOTOR_ENABLE_LUENBERGER != 0;
        case Lib_Motor::SteadyAngleSource::EKF:
            return LIB_MOTOR_ENABLE_EKF != 0;
        default:
            return false;
    }
}

/*
 * supports(CurrentSenseMode) -- 电流采样模式是否匹配
 * MotorCfg 配置的模式必须与 BuildCfg 编译的模式一致
 */
constexpr bool supports(Lib_Motor::CurrentSenseMode mode)
{
    return static_cast<int>(mode) == MOTOR_BUILD_CURRENT_SENSE_MODE;
}

/*
 * supportsRedundantSensor -- 是否编译了冗余传感器支持
 * 需 MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR 宏开启
 */
constexpr bool supportsRedundantSensor()
{
#ifdef MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    return true;
#else
    return false;
#endif
}

/*
 * supportsOutputSensor -- 是否编译了输出侧传感器支持
 * 需 MOTOR_BUILD_ENABLE_OUTPUT_SENSOR 宏开启
 */
constexpr bool supportsOutputSensor()
{
#ifdef MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    return true;
#else
    return false;
#endif
}

/*
 * supportsBusCurrent -- 是否编译了母线电流采样支持
 * 需 MOTOR_BUILD_HAS_BUS_CURRENT 宏开启
 */
constexpr bool supportsBusCurrent()
{
#ifdef MOTOR_BUILD_HAS_BUS_CURRENT
    return true;
#else
    return false;
#endif
}

/*
 * supportsPhaseVoltage -- 是否编译了相电压采样支持
 * 需 MOTOR_BUILD_HAS_PHASE_VOLTAGE 宏开启
 */
constexpr bool supportsPhaseVoltage()
{
#ifdef MOTOR_BUILD_HAS_PHASE_VOLTAGE
    return true;
#else
    return false;
#endif
}

/*
 * ntcSlots -- 获取编译的 NTC 通道数
 * 返回 MOTOR_BUILD_NTC_SLOTS 的值 (0~4)
 */
constexpr bool supportsStallProtection()
{
    return LIB_MOTOR_ENABLE_STALL_PROTECTION != 0;
}

constexpr bool supportsBrakeEnergyFsm()
{
    return LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM != 0;
}

constexpr bool supportsBatteryRegenPath()
{
    return LIB_MOTOR_ENABLE_BATTERY_REGEN_PATH != 0;
}

constexpr bool supportsBrakeResistor()
{
    return LIB_MOTOR_ENABLE_BRAKE_RESISTOR != 0;
}

constexpr bool supportsBrakeChopperControl()
{
    return LIB_MOTOR_ENABLE_BRAKE_CHOPPER_CONTROL != 0;
}

constexpr bool supportsMechanicalBrake()
{
    return LIB_MOTOR_ENABLE_MECHANICAL_BRAKE != 0;
}

constexpr bool supportsStreamHoldWhenIdle()
{
    return LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE != 0;
}

constexpr bool supportsSetpointFifoPlayback()
{
    return LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK != 0;
}

constexpr int ntcSlots()
{
    return MOTOR_BUILD_NTC_SLOTS;
}

constexpr int maxInstances()
{
    return LIB_MOTOR_MAX_INSTANCES;
}

/* === [13] 控制拓扑 === */
constexpr bool supportsFOCControl()
{
    return LIB_MOTOR_ENABLE_FOC_CONTROL != 0;
}

constexpr bool supportsTrapControl()
{
    return LIB_MOTOR_ENABLE_TRAP_CONTROL != 0;
}

/* === [14] 调试诊断字符串 === */
constexpr bool supportsHumanReadableFaultDetail()
{
    return LIB_MOTOR_ENABLE_HUMAN_READABLE_FAULT_DETAIL != 0;
}

/* === [15] 参数自动辨识 === */
constexpr bool supportsAutoIdentify()
{
    return LIB_MOTOR_ENABLE_AUTO_IDENTIFY != 0;
}

/* === [16] 顺逆风启动 === */
constexpr bool supportsFlyingStart()
{
    return LIB_MOTOR_ENABLE_FLYING_START != 0;
}

} // namespace Platform_MotorBuildConfig
