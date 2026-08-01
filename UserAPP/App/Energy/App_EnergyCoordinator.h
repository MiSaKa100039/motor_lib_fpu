#pragma once

#include "App/Core/App_Config.h"

#if USERAPP_ENABLE_ENERGY_COORDINATOR
#include "App/Axis/App_AxisService.h"
#endif

namespace UserAPP
{

#if USERAPP_ENABLE_ENERGY_COORDINATOR
struct AppEnergyInput
{
    uint32_t timestamp_us = 0U;
    uint32_t valid_for_us = 0U;

    bool bms_online = false;
    bool regen_allowed = false;
    bool dump_allowed = false;
    bool brake_derated = false;
    bool source_fault = false;

    float regen_ratio = 1.0f;
    float vbus_v = 0.0f;
    float vbus_limit_v = 0.0f;
    float bms_charge_current_limit_a = 0.0f;
    float max_regen_iq_a = 0.0f;
    float max_dump_iq_a = 0.0f;
    float max_regen_dc_current_a = 0.0f;
    float max_dump_current_a = 0.0f;
    float max_regen_power_w = 0.0f;
    float max_dump_power_w = 0.0f;
    float resistor_temp_c = 0.0f;
    float resistor_temp_limit_c = 0.0f;

    Lib_Motor::BrakeEnergyPathPolicy energy_path_policy =
        Lib_Motor::BrakeEnergyPathPolicy::REGEN_FIRST;
};

class AppEnergyCoordinator
{
public:
    void bindAxis(uint8_t axis_id, AppAxisService* axis);
    void updateInput(const AppEnergyInput& input);
    Lib_Motor::Result dispatchCommand(const AppCommand& cmd);
    void tick10Hz();

private:
    AppAxisService* axes_[APP_MAX_AXES] = {};
    AppEnergyInput input_;
};

#endif

} // namespace UserAPP
