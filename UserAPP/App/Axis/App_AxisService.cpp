#include "App/Axis/App_AxisService.h"

namespace UserAPP
{

void AppAxisService::bind(Lib_Motor::MotorAPI* motor)
{
    motor_ = motor;
#if USERAPP_ENABLE_SETPOINT_STREAM
    stream_enabled_ = false;
#endif
#if USERAPP_ENABLE_ENERGY_COORDINATOR
    energy_budget_enabled_ = false;
    regen_ratio_ = 1.0f;
#endif
    last_result_ = Lib_Motor::Result::Ok;
}

bool AppAxisService::available() const
{
    return motor_ != nullptr;
}

Lib_Motor::Result AppAxisService::handleCommand(const AppCommand& cmd)
{
    if (motor_ == nullptr)
    {
        last_result_ = Lib_Motor::Result::InvalidHandle;
        return last_result_;
    }

    switch (cmd.type)
    {
        case AppCommandType::START:
        case AppCommandType::STOP:
        case AppCommandType::EMERGENCY_STOP:
        case AppCommandType::CLEAR_FAULT:
        case AppCommandType::SELECT_MODE:
            return handleMotorCommand(cmd);

        case AppCommandType::SET_TARGET_TORQUE:
        case AppCommandType::SET_TARGET_SPEED:
        case AppCommandType::SET_TARGET_POSITION:
        case AppCommandType::WRITE_SETPOINT:
            return handleMotionCommand(cmd);

        case AppCommandType::SYNC_PRELOAD_SEGMENT:
        case AppCommandType::SYNC_ARM:
        case AppCommandType::SYNC_TRIGGER:
        case AppCommandType::SYNC_ABORT:
            return handleSyncCommand(cmd);

#if USERAPP_ENABLE_ENERGY_COORDINATOR
        case AppCommandType::SET_REGEN_RATIO:
        case AppCommandType::SET_ENERGY_BUDGET:
            return handleEnergyCommand(cmd);
#endif

        default:
            last_result_ = Lib_Motor::Result::NotSupported;
            return last_result_;
    }
}

void AppAxisService::tickMotionStream()
{
#if USERAPP_ENABLE_SETPOINT_STREAM
    if (motor_ == nullptr || !stream_enabled_)
    {
        return;
    }

    last_result_ = motor_->writeSetpoint(latest_setpoint_);
#endif
}

Lib_Motor::State AppAxisService::getState() const
{
    if (motor_ == nullptr)
    {
        return Lib_Motor::State::UNINITIALIZED;
    }
    return motor_->getState();
}

Lib_Motor::Fault AppAxisService::getFault() const
{
    if (motor_ == nullptr)
    {
        return Lib_Motor::Fault::NONE;
    }
    return motor_->getFault();
}

void AppAxisService::getMonitorData(Lib_Motor::MotorMonitorData& out_data) const
{
    if (motor_ == nullptr)
    {
        out_data = {};
        return;
    }
    motor_->getMonitorData(out_data);
}

void AppAxisService::getDebugData(Lib_Motor::MotorDebugData& out_data) const
{
    if (motor_ == nullptr)
    {
        out_data = {};
        return;
    }
    motor_->getDebugData(out_data);
}

Lib_Motor::Result AppAxisService::appendSetpointSeg(
    const Lib_Motor::MotionSetpoint* seg, uint16_t count)
{
    if (motor_ == nullptr)
    {
        last_result_ = Lib_Motor::Result::InvalidHandle;
        return last_result_;
    }

    last_result_ = motor_->appendSetpointSeg(seg, count);
    return last_result_;
}

Lib_Motor::Result AppAxisService::triggerSegPlayback(Lib_Motor::PlaybackTrigger trig)
{
    if (motor_ == nullptr)
    {
        last_result_ = Lib_Motor::Result::InvalidHandle;
        return last_result_;
    }

    last_result_ = motor_->triggerSegPlayback(trig);
    return last_result_;
}

Lib_Motor::Result AppAxisService::abortSegPlayback()
{
    if (motor_ == nullptr)
    {
        last_result_ = Lib_Motor::Result::InvalidHandle;
        return last_result_;
    }

    last_result_ = motor_->abortSegPlayback();
    return last_result_;
}

#if USERAPP_ENABLE_ENERGY_COORDINATOR
void AppAxisService::updateEnergyBudget(const Lib_Motor::MotorEnergyBudget& budget)
{
    latest_energy_budget_ = budget;
    energy_budget_enabled_ = true;
}

void AppAxisService::tickEnergyBudget()
{
    if (motor_ == nullptr || !energy_budget_enabled_)
    {
        return;
    }

    last_result_ = motor_->setEnergyBudget(latest_energy_budget_);
}
#endif

Lib_Motor::Result AppAxisService::handleMotorCommand(const AppCommand& cmd)
{
    switch (cmd.type)
    {
        case AppCommandType::START:
            last_result_ = motor_->start();
            break;

        case AppCommandType::STOP:
#if USERAPP_ENABLE_SETPOINT_STREAM
            stream_enabled_ = false;
#endif
            last_result_ = motor_->stop(cmd.stop_mode);
            break;

        case AppCommandType::EMERGENCY_STOP:
#if USERAPP_ENABLE_SETPOINT_STREAM
            stream_enabled_ = false;
#endif
            last_result_ = motor_->emergencyStop();
            break;

        case AppCommandType::CLEAR_FAULT:
            last_result_ = motor_->clearFault();
            break;

        case AppCommandType::SELECT_MODE:
            last_result_ = motor_->selectMode(cmd.mode);
            break;

        default:
            last_result_ = Lib_Motor::Result::NotSupported;
            break;
    }

    return last_result_;
}

Lib_Motor::Result AppAxisService::handleMotionCommand(const AppCommand& cmd)
{
    switch (cmd.type)
    {
        case AppCommandType::SET_TARGET_TORQUE:
#if USERAPP_ENABLE_SETPOINT_STREAM
            stream_enabled_ = false;
#endif
            last_result_ = motor_->setTargetTorque(cmd.value0);
            break;

        case AppCommandType::SET_TARGET_SPEED:
#if USERAPP_ENABLE_SETPOINT_STREAM
            stream_enabled_ = false;
#endif
            last_result_ = motor_->setTargetSpeed(cmd.value0);
            break;

        case AppCommandType::SET_TARGET_POSITION:
#if USERAPP_ENABLE_SETPOINT_STREAM
            stream_enabled_ = false;
#endif
            last_result_ = motor_->setTargetPosition(cmd.value0);
            break;

        case AppCommandType::WRITE_SETPOINT:
#if USERAPP_ENABLE_SETPOINT_STREAM
            latest_setpoint_ = cmd.setpoint;
            stream_enabled_ = true;
            last_result_ = Lib_Motor::Result::Ok;
#else
            last_result_ = Lib_Motor::Result::NotSupported;
#endif
            break;

        default:
            last_result_ = Lib_Motor::Result::NotSupported;
            break;
    }

    return last_result_;
}

Lib_Motor::Result AppAxisService::handleSyncCommand(const AppCommand& cmd)
{
    switch (cmd.type)
    {
        case AppCommandType::SYNC_PRELOAD_SEGMENT:
            return appendSetpointSeg(&cmd.setpoint, 1U);

        case AppCommandType::SYNC_ARM:
            return triggerSegPlayback(Lib_Motor::PlaybackTrigger::AWAIT_SYNC);

        case AppCommandType::SYNC_TRIGGER:
            return triggerSegPlayback(Lib_Motor::PlaybackTrigger::IMMEDIATE);

        case AppCommandType::SYNC_ABORT:
            return abortSegPlayback();

        default:
            last_result_ = Lib_Motor::Result::NotSupported;
            return last_result_;
    }
}

#if USERAPP_ENABLE_ENERGY_COORDINATOR
Lib_Motor::Result AppAxisService::handleEnergyCommand(const AppCommand& cmd)
{
    switch (cmd.type)
    {
        case AppCommandType::SET_REGEN_RATIO:
            regen_ratio_ = clamp01(cmd.value0);
            last_result_ = Lib_Motor::Result::Ok;
            break;

        case AppCommandType::SET_ENERGY_BUDGET:
            updateEnergyBudget(cmd.energy_budget);
            last_result_ = Lib_Motor::Result::Ok;
            break;

        default:
            last_result_ = Lib_Motor::Result::NotSupported;
            break;
    }

    return last_result_;
}

float AppAxisService::clamp01(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}
#endif

} // namespace UserAPP
