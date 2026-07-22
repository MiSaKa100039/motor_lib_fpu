#pragma once

#include "../../Public/Motor_Config.h"
#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorFeatureRegistry
{
public:
    static bool supportsStartupSource(StartupSource source);
    static bool supportsSteadySource(SteadyAngleSource source);
    static bool supportsRunPolicy(const MotorRunPolicy& policy);
    static bool supportsMode(Mode mode);
    static bool supportsStallProtection();
    static bool supportsDangerousTestApi();
    static bool supportsBrakeEnergyFsm();
    static bool supportsBatteryRegenPath();
    static bool supportsBrakeResistor();
    static bool supportsBrakeChopperControl();
    static bool supportsMechanicalBrake();

    static bool isClosedLoopSourceImplemented(SteadyAngleSource source);
    static bool isStartupHandoverImplemented(StartupSource startup,
                                             SteadyAngleSource steady);
};

} // namespace Lib_Motor
