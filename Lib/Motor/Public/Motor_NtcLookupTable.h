#pragma once

#include "Motor_Config.h"

#include <cstdint>

namespace Lib_Motor
{

struct MotorNtcLookupTable
{
    NtcTopology topology;
    uint32_t r_series_ohm;
    uint32_t r25_ohm;
    uint16_t beta_k;
    uint16_t adc_resolution;
    uint8_t step_shift;          // step = 1 << step_shift, 例如 4 表示 raw 间隔 16 counts
    uint16_t point_count;
    const int16_t* temp_deci_c;  // 单位 0.1°C, constexpr 表编译后放入 Flash/rodata
};

} // namespace Lib_Motor
