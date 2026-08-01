#pragma once

#include "../../Public/Motor_Config.h"
#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorRunPlan
{
public:
    static RunPhase selectInitialPhase(const MotorRunPolicy& policy,
                                       bool direct_run);
};

} // namespace Lib_Motor
