/*
 * Modulation_Base.h — 调制策略抽象基类 (vtable)
 *
 * 预留扩展: 将 SVPWM/SPWM/THIPWM 封装为不同调制策略
 *   modulate() 统一接口, voltage_utilization 标记电压利用率
 */

#pragma once

namespace Lib_Motor
{

struct ModulationVTable
{
    void (*modulate)(float v_alpha, float v_beta, float vbus,
                     float* duty_u, float* duty_v, float* duty_w);
    float voltage_utilization;   // 1.0 (SPWM) / 1.1547 (SVPWM)
};

} // namespace Lib_Motor
