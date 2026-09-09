#include "App/Motion/App_MotionCoordinator.h"

#if USERAPP_ENABLE_MOTION_COORDINATOR

namespace UserAPP
{

void AppMotionCoordinator::bindAxis(uint8_t axis_id, AppAxisService* axis)
{
    if (axis_id >= APP_MAX_AXES)
    {
        return;
    }

    axes_[axis_id] = axis;
}

Lib_Motor::Result AppMotionCoordinator::dispatchCommand(const AppCommand& cmd)
{
    if (cmd.axis_id >= APP_MAX_AXES || axes_[cmd.axis_id] == nullptr)
    {
        return Lib_Motor::Result::InvalidHandle;
    }

    return axes_[cmd.axis_id]->handleCommand(cmd);
}

Lib_Motor::Result AppMotionCoordinator::appendSetpointSeg(
    uint8_t axis_id, const Lib_Motor::MotionSetpoint* seg, uint16_t count)
{
    if (axis_id >= APP_MAX_AXES || axes_[axis_id] == nullptr)
    {
        return Lib_Motor::Result::InvalidHandle;
    }

    return axes_[axis_id]->appendSetpointSeg(seg, count);
}

Lib_Motor::Result AppMotionCoordinator::triggerSegPlayback(
    uint8_t axis_id, Lib_Motor::PlaybackTrigger trig)
{
    if (axis_id >= APP_MAX_AXES || axes_[axis_id] == nullptr)
    {
        return Lib_Motor::Result::InvalidHandle;
    }

    return axes_[axis_id]->triggerSegPlayback(trig);
}

Lib_Motor::Result AppMotionCoordinator::abortSegPlayback(uint8_t axis_id)
{
    if (axis_id >= APP_MAX_AXES || axes_[axis_id] == nullptr)
    {
        return Lib_Motor::Result::InvalidHandle;
    }

    return axes_[axis_id]->abortSegPlayback();
}

void AppMotionCoordinator::tick1kHz()
{
    for (uint8_t i = 0U; i < APP_MAX_AXES; ++i)
    {
        if (axes_[i] != nullptr)
        {
            axes_[i]->tickMotionStream();
        }
    }
}

} // namespace UserAPP

#endif
