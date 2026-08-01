#pragma once

#include "App/Core/App_Command.h"

namespace UserAPP
{

class AppAxisService
{
public:
    void bind(Lib_Motor::MotorAPI* motor);
    bool available() const;

    Lib_Motor::Result handleCommand(const AppCommand& cmd);
    void tickMotionStream();
    Lib_Motor::State getState() const;
    Lib_Motor::Fault getFault() const;
    void getMonitorData(Lib_Motor::MotorMonitorData& out_data) const;
    void getDebugData(Lib_Motor::MotorDebugData& out_data) const;
    Lib_Motor::Result appendSetpointSeg(const Lib_Motor::MotionSetpoint* seg, uint16_t count);
    Lib_Motor::Result triggerSegPlayback(Lib_Motor::PlaybackTrigger trig);
    Lib_Motor::Result abortSegPlayback();
#if USERAPP_ENABLE_ENERGY_COORDINATOR
    void updateEnergyBudget(const Lib_Motor::MotorEnergyBudget& budget);
    void tickEnergyBudget();
#endif

    Lib_Motor::Result lastResult() const { return last_result_; }
#if USERAPP_ENABLE_ENERGY_COORDINATOR
    float regenRatio() const { return regen_ratio_; }
#endif

private:
    Lib_Motor::Result handleMotorCommand(const AppCommand& cmd);
    Lib_Motor::Result handleMotionCommand(const AppCommand& cmd);
    Lib_Motor::Result handleSyncCommand(const AppCommand& cmd);
#if USERAPP_ENABLE_ENERGY_COORDINATOR
    Lib_Motor::Result handleEnergyCommand(const AppCommand& cmd);
    static float clamp01(float value);
#endif

    Lib_Motor::MotorAPI* motor_ = nullptr;

#if USERAPP_ENABLE_SETPOINT_STREAM
    Lib_Motor::MotionSetpoint latest_setpoint_;
    bool stream_enabled_ = false;
#endif

#if USERAPP_ENABLE_ENERGY_COORDINATOR
    Lib_Motor::MotorEnergyBudget latest_energy_budget_;
    bool energy_budget_enabled_ = false;

    float regen_ratio_ = 1.0f;
#endif
    Lib_Motor::Result last_result_ = Lib_Motor::Result::Ok;
};

} // namespace UserAPP
