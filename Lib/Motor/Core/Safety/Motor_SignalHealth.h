#pragma once

namespace Lib_Motor
{

class MotorManager;

class MotorSignalHealth
{
public:
    static void updatePositionSensors(MotorManager& m);
    static void checkPositionConsistency(MotorManager& m);
};

} // namespace Lib_Motor
