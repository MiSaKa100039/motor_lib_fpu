#pragma once

#include "App/Core/App_BuildConfig.h"
#if USERAPP_ENABLE_ENERGY_COORDINATOR
#include "App/Energy/App_EnergyCoordinator.h"
#endif

namespace UserAPP
{

void App_Init();
void App_Loop();
void App_Tick1kHz();
void App_Tick10Hz();
#if USERAPP_ENABLE_ENERGY_COORDINATOR
void App_UpdateEnergyInput(const AppEnergyInput& input);
#endif

} // namespace UserAPP
