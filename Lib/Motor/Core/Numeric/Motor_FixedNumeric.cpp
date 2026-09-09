#include "Motor_FixedNumeric.h"

#include "../../Control/Utils/FocMath.h"

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
    float wrapped = radians;
    if (wrapped >= TWO_PI || wrapped <= -TWO_PI)
    {
        const int turns = static_cast<int>(wrapped * (1.0f / TWO_PI));
        wrapped -= static_cast<float>(turns) * TWO_PI;
    }
    while (wrapped >= TWO_PI)
    {
        wrapped -= TWO_PI;
    }
    while (wrapped < 0.0f)
    {
        wrapped += TWO_PI;
    }
    const float phase = wrapped * (4294967296.0f / TWO_PI);
    return static_cast<phase_u32_t>(phase);
}

SinCos sinCos(phase_u32_t phase)
{
    return SinCos{sineFromPhase(phase), sineFromPhase(phase + 0x40000000UL)};
}

} // namespace FixedNumeric
} // namespace Lib_Motor
