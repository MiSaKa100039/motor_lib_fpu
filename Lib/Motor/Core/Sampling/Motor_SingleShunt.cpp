#include "Motor_SingleShunt.h"

#include <cmath>

namespace Lib_Motor
{

namespace
{

struct PhaseDuty
{
    MotorSingleShuntSamplePhase phase;
    float duty;
};

float clampDuty(float value, float ceiling)
{
    if (value < 0.0f) return 0.0f;
    if (value > ceiling) return ceiling;
    return value;
}

bool validPhase(MotorSingleShuntSamplePhase phase)
{
    return phase == MotorSingleShuntSamplePhase::U ||
           phase == MotorSingleShuntSamplePhase::V ||
           phase == MotorSingleShuntSamplePhase::W;
}

bool hasSampleWindow(float duty_high, float duty_low, float min_window_duty)
{
    constexpr float kDutyEpsilon = 1.0e-6f;
    return (duty_high - duty_low) + kDutyEpsilon >= min_window_duty;
}

void sortDescending(PhaseDuty& high, PhaseDuty& mid, PhaseDuty& low)
{
    if (mid.duty > high.duty)
    {
        PhaseDuty tmp = high;
        high = mid;
        mid = tmp;
    }
    if (low.duty > mid.duty)
    {
        PhaseDuty tmp = mid;
        mid = low;
        low = tmp;
    }
    if (mid.duty > high.duty)
    {
        PhaseDuty tmp = high;
        high = mid;
        mid = tmp;
    }
}

uint8_t sectorFromOrder(MotorSingleShuntSamplePhase high,
                        MotorSingleShuntSamplePhase mid,
                        MotorSingleShuntSamplePhase low)
{
    if (high == MotorSingleShuntSamplePhase::U &&
        mid == MotorSingleShuntSamplePhase::V &&
        low == MotorSingleShuntSamplePhase::W)
    {
        return 1U;
    }
    if (high == MotorSingleShuntSamplePhase::V &&
        mid == MotorSingleShuntSamplePhase::U &&
        low == MotorSingleShuntSamplePhase::W)
    {
        return 2U;
    }
    if (high == MotorSingleShuntSamplePhase::V &&
        mid == MotorSingleShuntSamplePhase::W &&
        low == MotorSingleShuntSamplePhase::U)
    {
        return 3U;
    }
    if (high == MotorSingleShuntSamplePhase::W &&
        mid == MotorSingleShuntSamplePhase::V &&
        low == MotorSingleShuntSamplePhase::U)
    {
        return 4U;
    }
    if (high == MotorSingleShuntSamplePhase::W &&
        mid == MotorSingleShuntSamplePhase::U &&
        low == MotorSingleShuntSamplePhase::V)
    {
        return 5U;
    }
    if (high == MotorSingleShuntSamplePhase::U &&
        mid == MotorSingleShuntSamplePhase::W &&
        low == MotorSingleShuntSamplePhase::V)
    {
        return 6U;
    }
    return 0U;
}

MotorSingleShuntSampleSlot makeSlot(float trigger_duty,
                                    MotorSingleShuntSamplePhase phase,
                                    int8_t sign)
{
    MotorSingleShuntSampleSlot slot;
    slot.trigger_duty = trigger_duty;
    slot.phase = phase;
    slot.sign = sign;
    return slot;
}

bool fillSlots(uint8_t sector,
               float trigger_first,
               float trigger_second,
               MotorSingleShuntSamplePhase low_phase,
               MotorSingleShuntSamplePhase high_phase,
               MotorSingleShuntSamplePlan& plan)
{
    plan.valid = 1U;
    plan.sector = sector;

    if (sector < 1U || sector > 6U)
    {
        plan.valid = 0U;
        return false;
    }

    plan.first = makeSlot(trigger_first, low_phase, -1);
    plan.second = makeSlot(trigger_second, high_phase, 1);
    return true;
}

void writeDuty(MotorSingleShuntSamplePhase phase,
               float value,
               float& duty_u,
               float& duty_v,
               float& duty_w)
{
    switch (phase)
    {
        case MotorSingleShuntSamplePhase::U:
            duty_u = value;
            break;
        case MotorSingleShuntSamplePhase::V:
            duty_v = value;
            break;
        case MotorSingleShuntSamplePhase::W:
            duty_w = value;
            break;
        default:
            break;
    }
}

bool assignMeasuredPhase(const MotorSingleShuntSampleSlot& slot,
                         float sample,
                         bool measured[3],
                         float phase_current[3])
{
    if (!validPhase(slot.phase) || slot.sign == 0)
    {
        return false;
    }

    const uint8_t index = static_cast<uint8_t>(slot.phase) - 1U;
    if (measured[index])
    {
        return false;
    }

    phase_current[index] = (slot.sign > 0) ? sample : -sample;
    measured[index] = true;
    return true;
}

} // namespace

bool MotorSingleShunt::buildSamplePlan(float& duty_u,
                                       float& duty_v,
                                       float& duty_w,
                                       float duty_ceiling,
                                       float min_sample_window_s,
                                       float pwm_frequency_hz,
                                       MotorSingleShuntSamplePlan& plan)
{
    plan = MotorSingleShuntSamplePlan{};

    if (!std::isfinite(duty_u) || !std::isfinite(duty_v) ||
        !std::isfinite(duty_w) || !std::isfinite(duty_ceiling) ||
        !std::isfinite(min_sample_window_s) || !std::isfinite(pwm_frequency_hz) ||
        duty_ceiling <= 0.0f || min_sample_window_s <= 0.0f ||
        pwm_frequency_hz <= 0.0f)
    {
        return false;
    }

    duty_u = clampDuty(duty_u, duty_ceiling);
    duty_v = clampDuty(duty_v, duty_ceiling);
    duty_w = clampDuty(duty_w, duty_ceiling);

    PhaseDuty high{MotorSingleShuntSamplePhase::U, duty_u};
    PhaseDuty mid{MotorSingleShuntSamplePhase::V, duty_v};
    PhaseDuty low{MotorSingleShuntSamplePhase::W, duty_w};
    sortDescending(high, mid, low);

    const float min_window_duty = min_sample_window_s * pwm_frequency_hz;
    if (min_window_duty <= 0.0f || min_window_duty >= 0.5f)
    {
        return false;
    }

    if ((high.duty - mid.duty) < min_window_duty)
    {
        high.duty = mid.duty + min_window_duty;
    }
    if ((mid.duty - low.duty) < min_window_duty)
    {
        low.duty = mid.duty - min_window_duty;
    }

    if (high.duty > duty_ceiling)
    {
        const float shift = high.duty - duty_ceiling;
        high.duty -= shift;
        mid.duty -= shift;
        low.duty -= shift;
    }
    if (low.duty < 0.0f)
    {
        const float shift = -low.duty;
        high.duty += shift;
        mid.duty += shift;
        low.duty += shift;
    }

    if (low.duty < 0.0f || high.duty > duty_ceiling ||
        !hasSampleWindow(high.duty, mid.duty, min_window_duty) ||
        !hasSampleWindow(mid.duty, low.duty, min_window_duty))
    {
        return false;
    }

    const uint8_t sector = sectorFromOrder(high.phase, mid.phase, low.phase);
    if (sector == 0U)
    {
        return false;
    }

    writeDuty(high.phase, high.duty, duty_u, duty_v, duty_w);
    writeDuty(mid.phase, mid.duty, duty_u, duty_v, duty_w);
    writeDuty(low.phase, low.duty, duty_u, duty_v, duty_w);

    const float trigger_first = 0.5f * (low.duty + mid.duty);
    const float trigger_second = 0.5f * (mid.duty + high.duty);
    return fillSlots(sector, trigger_first, trigger_second,
                     low.phase, high.phase, plan);
}

MotorCurrentSampleSchedule MotorSingleShunt::toCurrentSampleSchedule(
    const MotorSingleShuntSamplePlan& plan)
{
    MotorCurrentSampleSchedule schedule{};
    if (plan.valid == 0U ||
        !std::isfinite(plan.first.trigger_duty) ||
        !std::isfinite(plan.second.trigger_duty))
    {
        return schedule;
    }

    schedule.enabled = 1U;
    schedule.sample_count = 2U;
    schedule.trigger_duty[0] = plan.first.trigger_duty;
    schedule.trigger_duty[1] = plan.second.trigger_duty;
    return schedule;
}

bool MotorSingleShunt::reconstruct(const MotorSingleShuntSamplePlan& plan,
                                   float sample_first_a,
                                   float sample_second_a,
                                   MotorSingleShuntCurrents& currents)
{
    currents = MotorSingleShuntCurrents{};
    if (plan.valid == 0U ||
        !std::isfinite(sample_first_a) ||
        !std::isfinite(sample_second_a))
    {
        return false;
    }

    bool measured[3] = {false, false, false};
    float phase_current[3] = {0.0f, 0.0f, 0.0f};
    if (!assignMeasuredPhase(plan.first, sample_first_a, measured, phase_current) ||
        !assignMeasuredPhase(plan.second, sample_second_a, measured, phase_current))
    {
        return false;
    }

    uint8_t missing = 3U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (!measured[i])
        {
            if (missing != 3U)
            {
                return false;
            }
            missing = i;
        }
    }
    if (missing >= 3U)
    {
        return false;
    }

    phase_current[missing] = -(phase_current[0] + phase_current[1] + phase_current[2]);
    currents.ia = phase_current[0];
    currents.ib = phase_current[1];
    currents.ic = phase_current[2];
    return true;
}

} // namespace Lib_Motor
