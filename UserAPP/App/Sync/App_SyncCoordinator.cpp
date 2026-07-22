#include "App/Sync/App_SyncCoordinator.h"

#if USERAPP_ENABLE_SYNC_COORDINATOR

namespace UserAPP
{

void AppSyncCoordinator::bindMotionCoordinator(AppMotionCoordinator* motion)
{
    motion_ = motion;
}

bool AppSyncCoordinator::preloadFrame(const AppSyncFrame& frame)
{
    if (motion_ == nullptr)
    {
        return false;
    }

    bool has_axis = false;
    for (uint8_t i = 0U; i < APP_MAX_AXES; ++i)
    {
        if (!frame.axis_valid[i])
        {
            continue;
        }

        has_axis = true;
        const Lib_Motor::Result result =
            motion_->appendSetpointSeg(i, &frame.setpoint[i], 1U);
        if (result != Lib_Motor::Result::Ok)
        {
            abort();
            return false;
        }

        if (!axis_preloaded_[i])
        {
            axis_preloaded_[i] = true;
            ++preloaded_axis_count_;
        }
    }

    if (!has_axis)
    {
        return false;
    }

    state_ = AppSyncState::PRELOADED;
    return true;
}

void AppSyncCoordinator::arm(uint32_t trigger_tick)
{
    if (motion_ == nullptr || !hasPreloadedAxis())
    {
        state_ = AppSyncState::IDLE;
        return;
    }

    for (uint8_t i = 0U; i < APP_MAX_AXES; ++i)
    {
        if (!axis_preloaded_[i])
        {
            continue;
        }

        const Lib_Motor::Result result =
            motion_->triggerSegPlayback(i, Lib_Motor::PlaybackTrigger::AWAIT_SYNC);
        if (result != Lib_Motor::Result::Ok)
        {
            abort();
            return;
        }
    }

    trigger_tick_ = trigger_tick;
    state_ = AppSyncState::ARMED;
}

void AppSyncCoordinator::triggerNow()
{
    if (motion_ == nullptr || !hasPreloadedAxis())
    {
        state_ = AppSyncState::IDLE;
        return;
    }

    for (uint8_t i = 0U; i < APP_MAX_AXES; ++i)
    {
        if (!axis_preloaded_[i])
        {
            continue;
        }

        const Lib_Motor::Result result =
            motion_->triggerSegPlayback(i, Lib_Motor::PlaybackTrigger::IMMEDIATE);
        if (result != Lib_Motor::Result::Ok)
        {
            abort();
            return;
        }
    }

    trigger_tick_ = 0U;
    state_ = AppSyncState::RUNNING;
}

void AppSyncCoordinator::abort()
{
    if (motion_ != nullptr)
    {
        for (uint8_t i = 0U; i < APP_MAX_AXES; ++i)
        {
            if (axis_preloaded_[i])
            {
                (void)motion_->abortSegPlayback(i);
            }
        }
    }

    for (uint8_t i = 0U; i < APP_MAX_AXES; ++i)
    {
        axis_preloaded_[i] = false;
    }
    preloaded_axis_count_ = 0U;
    trigger_tick_ = 0U;
    state_ = AppSyncState::ABORTED;
}

void AppSyncCoordinator::tick1kHz(uint32_t app_tick)
{
    if (state_ == AppSyncState::ARMED && app_tick >= trigger_tick_)
    {
        triggerNow();
    }
}

bool AppSyncCoordinator::hasPreloadedAxis() const
{
    return preloaded_axis_count_ > 0U;
}

} // namespace UserAPP

#endif
