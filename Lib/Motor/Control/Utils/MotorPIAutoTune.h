/*
 * MotorPIAutoTune.h - 电流/速度/位置环 PI 增益自动推导工具
 */

#pragma once

namespace Lib_Motor
{

class MotorPIAutoTune
{
public:
    static void deriveCurrentLoopGains(
        float bandwidth_hz, float zeta,
        float rs, float ls,
        float& out_kp, float& out_ki)
    {
        const float omega_c = 2.0f * 3.14159265f * bandwidth_hz;
        if (bandwidth_hz <= 0.0f || zeta <= 0.0f || rs <= 0.0f || ls <= 0.0f)
        {
            out_kp = 0.0f;
            out_ki = 0.0f;
            return;
        }
        out_kp = omega_c * ls; // [V/A]
        out_ki = omega_c * rs; // [V/(A*s)]
    }

    static void deriveSpeedLoopGains(
        float bandwidth_hz, float zeta,
        float torque_constant_n_m_per_a,
        float inertia_kg_m2,
        float& out_kp, float& out_ki)
    {
        (void)bandwidth_hz;
        (void)zeta;
        (void)torque_constant_n_m_per_a;
        (void)inertia_kg_m2;
        out_kp = 0.0f;
        out_ki = 0.0f;
    }

    static void derivePositionLoopGains(
        float bandwidth_hz, float zeta,
        float torque_constant_n_m_per_a,
        float inertia_kg_m2,
        float& out_kp, float& out_ki)
    {
        (void)bandwidth_hz;
        (void)zeta;
        (void)torque_constant_n_m_per_a;
        (void)inertia_kg_m2;
        out_kp = 0.0f;
        out_ki = 0.0f;
    }
};

} // namespace Lib_Motor
