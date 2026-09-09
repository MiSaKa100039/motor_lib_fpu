#pragma once

#include "App/Core/App_Config.h"

#if USERAPP_ENABLE_MOTION_COORDINATOR
#include "App/Axis/App_AxisService.h"
#endif

namespace UserAPP
{

#if USERAPP_ENABLE_MOTION_COORDINATOR
class AppMotionCoordinator
{
public:
    void bindAxis(uint8_t axis_id, AppAxisService* axis);
    Lib_Motor::Result dispatchCommand(const AppCommand& cmd);
    Lib_Motor::Result appendSetpointSeg(uint8_t axis_id,
                                        const Lib_Motor::MotionSetpoint* seg,
                                        uint16_t count);
    Lib_Motor::Result triggerSegPlayback(uint8_t axis_id,
                                         Lib_Motor::PlaybackTrigger trig);
    Lib_Motor::Result abortSegPlayback(uint8_t axis_id);
    void tick1kHz();

private:
    AppAxisService* axes_[APP_MAX_AXES] = {};
};

#endif

} // namespace UserAPP
