#pragma once

#include <cstdint>

namespace Lib_Motor
{
namespace FixedNumeric
{

using q15_t = std::int16_t;
using phase_u32_t = std::uint32_t;

constexpr q15_t kQ15One = 32767;
constexpr q15_t kQ15Half = 16384;

struct SinCos
{
    q15_t sin;
    q15_t cos;
};

struct Ab
{
    q15_t alpha;
    q15_t beta;
};

struct Dq
{
    q15_t d;
    q15_t q;
};

struct DutyAbc
{
    q15_t a;
    q15_t b;
    q15_t c;
};

constexpr std::int32_t clamp32(std::int32_t value, std::int32_t minimum, std::int32_t maximum)
{
    return (value < minimum) ? minimum : ((value > maximum) ? maximum : value);
}

constexpr q15_t saturateQ15(std::int32_t value)
{
    return static_cast<q15_t>(clamp32(value, -32768, 32767));
}

constexpr q15_t saturateDutyQ15(std::int32_t value)
{
    return static_cast<q15_t>(clamp32(value, 0, 32767));
}

constexpr q15_t addQ15(q15_t lhs, q15_t rhs)
{
    return saturateQ15(static_cast<std::int32_t>(lhs) + static_cast<std::int32_t>(rhs));
}

constexpr q15_t subtractQ15(q15_t lhs, q15_t rhs)
{
    return saturateQ15(static_cast<std::int32_t>(lhs) - static_cast<std::int32_t>(rhs));
}

constexpr q15_t multiplyQ15(q15_t lhs, q15_t rhs)
{
    const std::int32_t product = static_cast<std::int32_t>(lhs) * static_cast<std::int32_t>(rhs);
    const std::int32_t magnitude = (product < 0) ? -product : product;
    const std::int32_t rounded = (magnitude + (1 << 14)) >> 15;
    return saturateQ15((product < 0) ? -rounded : rounded);
}

constexpr q15_t absoluteQ15(q15_t value)
{
    return (value == static_cast<q15_t>(-32768)) ? kQ15One
                                                 : static_cast<q15_t>((value < 0) ? -value : value);
}

q15_t fromNormalized(float value);
float toNormalized(q15_t value);
q15_t fromPhysical(float value, float base);
float toPhysical(q15_t value, float base);
phase_u32_t phaseFromRadians(float radians);
SinCos sinCos(phase_u32_t phase);

inline Dq park(const Ab& input, const SinCos& angle)
{
    return Dq{
        addQ15(multiplyQ15(input.alpha, angle.cos), multiplyQ15(input.beta, angle.sin)),
        subtractQ15(multiplyQ15(input.beta, angle.cos), multiplyQ15(input.alpha, angle.sin))};
}

inline Ab inversePark(const Dq& input, const SinCos& angle)
{
    return Ab{
        subtractQ15(multiplyQ15(input.d, angle.cos), multiplyQ15(input.q, angle.sin)),
        addQ15(multiplyQ15(input.d, angle.sin), multiplyQ15(input.q, angle.cos))};
}

inline DutyAbc svpwmVbusNormalized(const Ab& voltage)
{
    constexpr q15_t kSqrt3Over2 = 28378;
    const q15_t halfAlpha = static_cast<q15_t>(voltage.alpha >> 1);
    const q15_t betaTerm = multiplyQ15(voltage.beta, kSqrt3Over2);

    const q15_t phaseA = voltage.alpha;
    const q15_t phaseB = addQ15(static_cast<q15_t>(-halfAlpha), betaTerm);
    const q15_t phaseC = subtractQ15(static_cast<q15_t>(-halfAlpha), betaTerm);

    const q15_t maximum = (phaseA > phaseB) ? ((phaseA > phaseC) ? phaseA : phaseC)
                                            : ((phaseB > phaseC) ? phaseB : phaseC);
    const q15_t minimum = (phaseA < phaseB) ? ((phaseA < phaseC) ? phaseA : phaseC)
                                            : ((phaseB < phaseC) ? phaseB : phaseC);
    const q15_t offset = static_cast<q15_t>(-(static_cast<std::int32_t>(maximum) + minimum) / 2);

    return DutyAbc{
        saturateDutyQ15(static_cast<std::int32_t>(kQ15Half) + addQ15(phaseA, offset)),
        saturateDutyQ15(static_cast<std::int32_t>(kQ15Half) + addQ15(phaseB, offset)),
        saturateDutyQ15(static_cast<std::int32_t>(kQ15Half) + addQ15(phaseC, offset))};
}

inline DutyAbc spwmVbusNormalized(const Ab& voltage)
{
    constexpr q15_t kSqrt3Over2 = 28378;
    const q15_t halfAlpha = static_cast<q15_t>(voltage.alpha >> 1);
    const q15_t betaTerm = multiplyQ15(voltage.beta, kSqrt3Over2);

    const q15_t phaseA = voltage.alpha;
    const q15_t phaseB = addQ15(static_cast<q15_t>(-halfAlpha), betaTerm);
    const q15_t phaseC = subtractQ15(static_cast<q15_t>(-halfAlpha), betaTerm);

    return DutyAbc{
        saturateDutyQ15(static_cast<std::int32_t>(kQ15Half) + phaseA),
        saturateDutyQ15(static_cast<std::int32_t>(kQ15Half) + phaseB),
        saturateDutyQ15(static_cast<std::int32_t>(kQ15Half) + phaseC)};
}

} // namespace FixedNumeric
} // namespace Lib_Motor
