#pragma once

#include "App/Core/App_Config.h"

#if USERAPP_ENABLE_SYNC_COORDINATOR
#include "App/Core/App_Command.h"
#include "App/Motion/App_MotionCoordinator.h"
#endif

namespace UserAPP
{

#if USERAPP_ENABLE_SYNC_COORDINATOR
enum class AppSyncState : uint8_t
{
    IDLE = 0,
    PRELOADED,
    ARMED,
    RUNNING,
    ABORTED,
};

struct AppSyncFrame
{
    uint32_t sequence = 0U;
    uint32_t trigger_tick = 0U;
    bool axis_valid[APP_MAX_AXES] = {};
    Lib_Motor::MotionSetpoint setpoint[APP_MAX_AXES];
};

class AppSyncCoordinator
{
public:
    void bindMotionCoordinator(AppMotionCoordinator* motion);
    bool preloadFrame(const AppSyncFrame& frame);
    void arm(uint32_t trigger_tick);
    void triggerNow();
    void abort();
    void tick1kHz(uint32_t app_tick);

    AppSyncState state() const { return state_; }

private:
    bool hasPreloadedAxis() const;

    AppMotionCoordinator* motion_ = nullptr;
    bool axis_preloaded_[APP_MAX_AXES] = {};
    uint8_t preloaded_axis_count_ = 0U;
    uint32_t trigger_tick_ = 0U;
    AppSyncState state_ = AppSyncState::IDLE;
};

#endif

} // namespace UserAPP
