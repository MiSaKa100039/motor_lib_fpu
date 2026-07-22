#include "App/Core/App_Application.h"

#include "App/Axis/App_AxisService.h"
#include "App/Core/App_CommandInbox.h"
#if USERAPP_ENABLE_ENERGY_COORDINATOR
#include "App/Energy/App_EnergyCoordinator.h"
#endif
#if USERAPP_ENABLE_MOTION_COORDINATOR
#include "App/Motion/App_MotionCoordinator.h"
#endif
#if USERAPP_ENABLE_SYNC_COORDINATOR
#include "App/Sync/App_SyncCoordinator.h"
#endif
#include "Platform_Motor.h"

namespace UserAPP
{

namespace
{

AppAxisService g_axis0;
#if USERAPP_ENABLE_MOTION_COORDINATOR
AppMotionCoordinator g_motion;
#endif
#if USERAPP_ENABLE_ENERGY_COORDINATOR
AppEnergyCoordinator g_energy;
#endif
#if USERAPP_ENABLE_SYNC_COORDINATOR
AppSyncCoordinator g_sync;
uint32_t g_app_tick_1khz = 0U;
#endif

#if USERAPP_ENABLE_ENERGY_COORDINATOR
bool IsEnergyCommand(AppCommandType type)
{
    return type == AppCommandType::SET_REGEN_RATIO ||
           type == AppCommandType::SET_ENERGY_BUDGET;
}
#endif

} // namespace

void App_Init()
{
    g_axis0.bind(&Global_Motor_0);

#if USERAPP_ENABLE_MOTION_COORDINATOR
    g_motion.bindAxis(0U, &g_axis0);
#endif
#if USERAPP_ENABLE_ENERGY_COORDINATOR
    g_energy.bindAxis(0U, &g_axis0);
#endif
#if USERAPP_ENABLE_SYNC_COORDINATOR
    g_sync.bindMotionCoordinator(&g_motion);
#endif

    App_ClearCommands();
}

void App_Loop()
{
    AppCommand cmd;
    while (App_TryTakeCommand(cmd))
    {
#if USERAPP_ENABLE_ENERGY_COORDINATOR
        if (IsEnergyCommand(cmd.type))
        {
            (void)g_energy.dispatchCommand(cmd);
        }
        else
#endif
        {
#if USERAPP_ENABLE_MOTION_COORDINATOR
            (void)g_motion.dispatchCommand(cmd);
#else
            (void)g_axis0.handleCommand(cmd);
#endif
        }
    }
}

void App_Tick1kHz()
{
#if USERAPP_ENABLE_SYNC_COORDINATOR
    ++g_app_tick_1khz;
    g_sync.tick1kHz(g_app_tick_1khz);
#endif
#if USERAPP_ENABLE_MOTION_COORDINATOR
    g_motion.tick1kHz();
#else
    g_axis0.tickMotionStream();
#endif
}

void App_Tick10Hz()
{
#if USERAPP_ENABLE_ENERGY_COORDINATOR
    g_energy.tick10Hz();
#endif
}

#if USERAPP_ENABLE_ENERGY_COORDINATOR
void App_UpdateEnergyInput(const AppEnergyInput& input)
{
    g_energy.updateInput(input);
}
#endif

} // namespace UserAPP
