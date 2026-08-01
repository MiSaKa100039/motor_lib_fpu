#pragma once

#include "../../Public/Motor_HAL_Interface.h"

namespace Lib_Motor
{

enum class MotorSingleShuntSamplePhase : uint8_t
{
    NONE = 0,
    U    = 1,
    V    = 2,
    W    = 3,
};

struct MotorSingleShuntSampleSlot
{
    float trigger_duty = 0.5f;
    MotorSingleShuntSamplePhase phase = MotorSingleShuntSamplePhase::NONE;
    int8_t sign = 1;
};

struct MotorSingleShuntSamplePlan
{
    uint8_t valid = 0U;
    uint8_t sector = 0U;
    MotorSingleShuntSampleSlot first;
    MotorSingleShuntSampleSlot second;
};

struct MotorSingleShuntCurrents
{
    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;
};

class MotorSingleShunt
{
public:
    static bool buildSamplePlan(float& duty_u,
                                float& duty_v,
                                float& duty_w,
                                float duty_ceiling,
                                float min_sample_window_s,
                                float pwm_frequency_hz,
                                MotorSingleShuntSamplePlan& plan);

    static MotorCurrentSampleSchedule toCurrentSampleSchedule(
        const MotorSingleShuntSamplePlan& plan);

    static bool reconstruct(const MotorSingleShuntSamplePlan& plan,
                            float sample_first_a,
                            float sample_second_a,
                            MotorSingleShuntCurrents& currents);
};

} // namespace Lib_Motor
