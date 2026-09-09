#include "App/Energy/App_EnergyCoordinator.h"

#if USERAPP_ENABLE_ENERGY_COORDINATOR

namespace UserAPP
{

namespace
{

float Clamp01(float value)
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

Lib_Motor::MotorEnergyBudget BuildBudgetForAxis(const AppEnergyInput& input,
                                                float axis_regen_ratio)
{
    Lib_Motor::MotorEnergyBudget budget;
    budget.timestamp_us = input.timestamp_us;
    budget.valid_for_us = input.valid_for_us;

    budget.regen_allowed = input.bms_online &&
                           input.regen_allowed &&
                           !input.source_fault;
    budget.dump_allowed = input.dump_allowed && !input.source_fault;
    budget.brake_derated = input.brake_derated;
    budget.source_fault = input.source_fault;

    budget.vbus_v = input.vbus_v;
    budget.vbus_limit_v = input.vbus_limit_v;
    budget.energy_path_policy = input.energy_path_policy;

    const float regen_ratio = Clamp01(input.regen_ratio) * Clamp01(axis_regen_ratio);
    budget.max_regen_iq_a = input.max_regen_iq_a * regen_ratio;
    budget.max_dump_iq_a = input.max_dump_iq_a;

    budget.max_regen_dc_current_a = input.max_regen_dc_current_a;
    if (budget.max_regen_dc_current_a <= 0.0f &&
        input.bms_charge_current_limit_a > 0.0f)
    {
        budget.max_regen_dc_current_a = input.bms_charge_current_limit_a;
    }
    budget.max_dump_current_a = input.max_dump_current_a;
    budget.max_regen_power_w = input.max_regen_power_w;
    budget.max_dump_power_w = input.max_dump_power_w;
    budget.resistor_temp_c = input.resistor_temp_c;
    budget.resistor_temp_limit_c = input.resistor_temp_limit_c;
    return budget;
}

} // namespace

void AppEnergyCoordinator::bindAxis(uint8_t axis_id, AppAxisService* axis)
{
    if (axis_id >= APP_MAX_AXES)
    {
        return;
    }

    axes_[axis_id] = axis;
}

void AppEnergyCoordinator::updateInput(const AppEnergyInput& input)
{
    input_ = input;
}

Lib_Motor::Result AppEnergyCoordinator::dispatchCommand(const AppCommand& cmd)
{
    if (cmd.axis_id >= APP_MAX_AXES || axes_[cmd.axis_id] == nullptr)
    {
        return Lib_Motor::Result::InvalidHandle;
    }

    return axes_[cmd.axis_id]->handleCommand(cmd);
}

void AppEnergyCoordinator::tick10Hz()
{
    for (uint8_t i = 0U; i < APP_MAX_AXES; ++i)
    {
        if (axes_[i] != nullptr)
        {
            const Lib_Motor::MotorEnergyBudget budget =
                BuildBudgetForAxis(input_, axes_[i]->regenRatio());
            axes_[i]->updateEnergyBudget(budget);
            axes_[i]->tickEnergyBudget();
        }
    }
}

} // namespace UserAPP

#endif
