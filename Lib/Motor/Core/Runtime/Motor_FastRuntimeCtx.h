#pragma once

#include "../../Public/Motor_Features.h"

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../Numeric/Motor_FixedNumeric.h"
#endif

namespace Lib_Motor
{

/*
 * FastRuntimeCtx 只承载快环计算会高频读写的控制量。
 * 公共 RuntimeCtx 仍保留物理量镜像，供 API、Ozone 和监控接口读取。
 */
struct FastRuntimeCtx
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    using Current = FixedNumeric::q15_t;
    using Voltage = FixedNumeric::q15_t;
    using Duty = FixedNumeric::q15_t;
    using Speed = FixedNumeric::q15_t;
    using Angle = FixedNumeric::phase_u32_t;

    float current_base_a = 1.0f;
    float voltage_base_v = 1.0f;
    float speed_base_rpm = 1.0f;

    Current i_alpha = 0;
    Current i_beta = 0;
    Current i_d = 0;
    Current i_q = 0;

    Current target_id = 0;
    Current target_iq = 0;
    Speed target_speed = 0;
    Speed speed_ref_limited = 0;
    Current speed_pid_iq = 0;
    Current iq_ref_command = 0;
    Current iq_ref_limited = 0;

    Voltage v_d = 0;
    Voltage v_q = 0;
    Voltage v_alpha = 0;
    Voltage v_beta = 0;

    Angle angle_elec = 0U;
    FixedNumeric::SinCos angle_sin_cos{};

    Duty duty_a = 0;
    Duty duty_b = 0;
    Duty duty_c = 0;
#else
    using Current = float;
    using Voltage = float;
    using Duty = float;
    using Speed = float;
    using Angle = float;

    Current i_alpha = 0.0f;
    Current i_beta = 0.0f;
    Current i_d = 0.0f;
    Current i_q = 0.0f;

    Current target_id = 0.0f;
    Current target_iq = 0.0f;
    Speed target_speed = 0.0f;
    Speed speed_ref_limited = 0.0f;
    Current speed_pid_iq = 0.0f;
    Current iq_ref_command = 0.0f;
    Current iq_ref_limited = 0.0f;

    Voltage v_d = 0.0f;
    Voltage v_q = 0.0f;
    Voltage v_alpha = 0.0f;
    Voltage v_beta = 0.0f;

    Angle angle_elec = 0.0f;
    float sin_angle_elec = 0.0f;
    float cos_angle_elec = 1.0f;

    Duty duty_a = 0.0f;
    Duty duty_b = 0.0f;
    Duty duty_c = 0.0f;
#endif
};

} // namespace Lib_Motor
