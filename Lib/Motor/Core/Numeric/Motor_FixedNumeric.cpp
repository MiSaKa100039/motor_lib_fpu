#include "Motor_FixedNumeric.h"

#include "../../Common/Math/FocMath.h"

#include <cmath>

namespace Lib_Motor
{
namespace FixedNumeric
{
namespace
{

constexpr q15_t kQuarterWave[65] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962,
    8739, 9512, 10278, 11039, 11793, 12539, 13279, 14010, 14732, 15446,
    16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403, 22006,
    22595, 23170, 23732, 24279, 24811, 25329, 25832, 26319, 26790, 27245,
    27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852,
    31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609,
    32678, 32728, 32757, 32767};

q15_t quarterSin(std::uint32_t offset)
{
    constexpr std::uint32_t kFractionBits = 24U;
    const std::uint32_t index = offset >> kFractionBits;
    if (index >= 64U)
    {
        return kQuarterWave[64];
    }

    const std::uint32_t fractionQ16 = (offset & 0x00FFFFFFUL) >> 8U;
    const std::int32_t first = kQuarterWave[index];
    const std::int32_t delta = static_cast<std::int32_t>(kQuarterWave[index + 1U]) - first;
    const std::int32_t interpolated = first + static_cast<std::int32_t>((delta * fractionQ16) >> 16U);
    return static_cast<q15_t>(interpolated);
}

q15_t sineFromPhase(phase_u32_t phase)
{
    const std::uint32_t quadrant = phase >> 30U;
    const std::uint32_t offset = phase & 0x3FFFFFFFUL;

    switch (quadrant)
    {
        case 0U:
            return quarterSin(offset);
        case 1U:
            return quarterSin(0x40000000UL - offset);
        case 2U:
            return static_cast<q15_t>(-quarterSin(offset));
        default:
            return static_cast<q15_t>(-quarterSin(0x40000000UL - offset));
    }
}

} // namespace

q15_t fromNormalized(float value)
{
    if (!std::isfinite(value))
    {
        return 0;
    }
    if (value > 0.999969f)
    {
        value = 0.999969f;
    }
    else if (value < -1.0f)
    {
        value = -1.0f;
    }
    return saturateQ15(static_cast<std::int32_t>(value * 32768.0f));
}

float toNormalized(q15_t value)
{
    return static_cast<float>(value) * (1.0f / 32768.0f);
}

q15_t fromPhysical(float value, float base)
{
    if (!(base > 0.0f))
    {
        return 0;
    }
    return fromNormalized(value / base);
}

float toPhysical(q15_t value, float base)
{
    return toNormalized(value) * base;
}

phase_u32_t phaseFromRadians(float radians)
{
    if (!std::isfinite(radians))
    {
        return 0U;
    }
    float wrapped = std::fmod(radians, TWO_PI);
    if (wrapped < 0.0f)
    {
        wrapped += TWO_PI;
    }
    const double phase = static_cast<double>(wrapped) * (4294967296.0 / static_cast<double>(TWO_PI));
    return static_cast<phase_u32_t>(phase);
}

SinCos sinCos(phase_u32_t phase)
{
    return SinCos{sineFromPhase(phase), sineFromPhase(phase + 0x40000000UL)};
}

void FixedPiController::configure(float kp,
                                  float ki,
                                  float inputBase,
                                  float outputBase,
                                  float dt,
                                  float outputLimit,
                                  bool resetIntegrator)
{
    if (!(inputBase > 0.0f) || !(outputBase > 0.0f) || !(dt > 0.0f))
    {
        kpQ15_ = 0;
        kiPerTickQ15_ = 0;
        outputLimitQ15_ = 0;
        integratorQ30_ = 0;
        return;
    }

    kpQ15_ = fromNormalized((kp * inputBase) / outputBase);
    kiPerTickQ15_ = fromNormalized((ki * dt * inputBase) / outputBase);
    outputLimitQ15_ = absoluteQ15(fromPhysical(outputLimit, outputBase));

    const std::int32_t limitQ30 = static_cast<std::int32_t>(outputLimitQ15_) << 15U;
    if (resetIntegrator)
    {
        integratorQ30_ = 0;
    }
    else
    {
        integratorQ30_ = clamp32(integratorQ30_, -limitQ30, limitQ30);
    }
}

q15_t FixedPiController::update(q15_t error, bool holdIntegral)
{
    const std::int32_t proportionalQ30 = static_cast<std::int32_t>(kpQ15_) * error;
    const std::int32_t limitQ30 = static_cast<std::int32_t>(outputLimitQ15_) << 15U;
    if (!holdIntegral)
    {
        integratorQ30_ = clamp32(integratorQ30_ + static_cast<std::int32_t>(kiPerTickQ15_) * error,
                                 -limitQ30,
                                 limitQ30);
    }
    const std::int32_t output = (proportionalQ30 + integratorQ30_) >> 15U;
    return saturateQ15(clamp32(output, -outputLimitQ15_, outputLimitQ15_));
}

void FixedPiController::reset()
{
    integratorQ30_ = 0;
}

void FixedPiController::decayIntegral(float factor)
{
    if (!std::isfinite(factor))
    {
        return;
    }
    if (factor < 0.0f)
    {
        factor = 0.0f;
    }
    else if (factor > 1.0f)
    {
        factor = 1.0f;
    }
    integratorQ30_ = static_cast<std::int32_t>(static_cast<float>(integratorQ30_) * factor);
}

} // namespace FixedNumeric
} // namespace Lib_Motor
