#include "Motor_RunPlan.h"

namespace Lib_Motor
{

RunPhase MotorRunPlan::selectInitialPhase(const MotorRunPolicy& policy,
                                          bool direct_run)
{
    if (direct_run)
    {
        return RunPhase::RUN_DIRECT;
    }

    switch (policy.steady_source)
    {
        case SteadyAngleSource::SMO:
            if (policy.startup_source == StartupSource::IF)
            {
                return RunPhase::FORCE_DRAG;
            }
            if (policy.startup_source == StartupSource::HFI)
            {
                return RunPhase::HFI_ONLY;
            }
            if (policy.startup_source == StartupSource::SENSOR)
            {
                return RunPhase::SENSOR_ONLY;
            }
            return RunPhase::NONE;

        case SteadyAngleSource::SENSOR:
            return RunPhase::SENSOR_ONLY;

        case SteadyAngleSource::HFI:
            return RunPhase::HFI_ONLY;

        default:
            return RunPhase::NONE;
    }
}

} // namespace Lib_Motor
