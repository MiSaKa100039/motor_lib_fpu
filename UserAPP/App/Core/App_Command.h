#pragma once

#include <stdint.h>

#include "App/Core/App_BuildConfig.h"
#include "Motor.h"

namespace UserAPP
{

enum class AppCommandType : uint8_t
{
    NONE = 0,

    START,
    STOP,
    EMERGENCY_STOP,
    CLEAR_FAULT,
    SELECT_MODE,

    SET_TARGET_TORQUE,
    SET_TARGET_SPEED,
    SET_TARGET_POSITION,
    WRITE_SETPOINT,

    SET_REGEN_RATIO,
    SET_ENERGY_BUDGET,

    SYNC_PRELOAD_SEGMENT,
    SYNC_ARM,
    SYNC_TRIGGER,
    SYNC_ABORT,
};

struct AppCommand
{
    AppCommandType type = AppCommandType::NONE;
    uint8_t axis_id = 0U;

    Lib_Motor::Mode mode = Lib_Motor::Mode::NONE;
    Lib_Motor::StopMode stop_mode = Lib_Motor::StopMode::COAST;

    float value0 = 0.0f;
    float value1 = 0.0f;
    float value2 = 0.0f;

    Lib_Motor::MotionSetpoint setpoint;
    Lib_Motor::MotorEnergyBudget energy_budget;
};

} // namespace UserAPP
