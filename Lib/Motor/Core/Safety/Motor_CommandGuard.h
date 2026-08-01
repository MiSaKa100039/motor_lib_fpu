#pragma once

#include "../../Public/Motor_Config.h"
#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorCommandGuard
{
public:
    static bool   isDebugOrCalib(Mode mode);
    static Result validateApiModeChange(State state, Mode current_mode, Mode new_mode);
    static Result validateModeSelection(const MotorConfig& cfg, Mode mode);
    static Result validateModeSelection(const MotorConfig& cfg,
                                        const MotorRunPolicy& policy,
                                        Mode mode);
    static Result validateModeSelection(const MotorHardwareConfig& hardware,
                                        const MotorAlgorithmParam& algorithm,
                                        const MotorRunPolicy& policy,
                                        Mode mode);
};

} // namespace Lib_Motor
