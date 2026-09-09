#pragma once

#include "../../Public/Motor_NtcLookupTable.h"

#include <cstdint>

namespace Lib_Motor
{
namespace NtcLookup
{

inline int16_t rawToDeciC(const MotorNtcLookupTable* table, uint16_t raw)
{
    if (table == nullptr || table->temp_deci_c == nullptr || table->point_count == 0U)
    {
        return 32767;
    }
    if (table->point_count == 1U)
    {
        return table->temp_deci_c[0];
    }

    const uint8_t shift = table->step_shift;
    const uint32_t step = 1UL << shift;
    const uint32_t index = static_cast<uint32_t>(raw) >> shift;

    if (index >= static_cast<uint32_t>(table->point_count - 1U))
    {
        return table->temp_deci_c[table->point_count - 1U];
    }

    const uint32_t frac = static_cast<uint32_t>(raw) & (step - 1UL);
    const int32_t y0 = table->temp_deci_c[index];
    const int32_t y1 = table->temp_deci_c[index + 1U];
    const int32_t interp = y0 + (((y1 - y0) * static_cast<int32_t>(frac)) >> shift);

    if (interp > 32767)
    {
        return 32767;
    }
    if (interp < -32768)
    {
        return -32768;
    }
    return static_cast<int16_t>(interp);
}

} // namespace NtcLookup
} // namespace Lib_Motor
