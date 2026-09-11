/*
 * Motor_Runtime.h - 电机运行态上下文
 *
 * RuntimeCtx 是 Core 内部状态快照, 不作为 Public API 暴露。
 * q15/float 后端在这里完整裁切, 避免同一字段在 ISR 路径出现双语义。
 */

#pragma once

#include "../../Observer/Observer_Base.h"
#include "../../Public/Motor_Config.h"
#include "../../Public/Motor_Features.h"

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../Numeric/Motor_FixedNumeric.h"
#endif

#include "../../Control/Utils/FocMath.h"

#include <cstdint>
#include <stdint.h>

namespace Lib_Motor
{

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15

#if LIB_MOTOR_ENABLE_SENSOR || \
    LIB_MOTOR_ENABLE_HFI || \
    LIB_MOTOR_ENABLE_DEBUG_HFI_ANY || \
    LIB_MOTOR_ENABLE_TORQUE_CONTROL || \
    LIB_MOTOR_ENABLE_POSITION_CONTROL || \
    LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL || \
    LIB_MOTOR_ENABLE_STALL_PROTECTION || \
    LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM || \
    LIB_MOTOR_ENABLE_AUTO_IDENTIFY || \
    LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK || \
    LIB_MOTOR_ENABLE_FLYING_START || \
    LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD || \
    LIB_MOTOR_ENABLE_TORQUE_FEEDFORWARD || \
    LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE || \
    MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR || \
    MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
#error "fixed-q15 RuntimeCtx no longer stores float fields; enable only features with completed q15 runtime support."
#endif

using RuntimeCurrent = FixedNumeric::q15_t;
using RuntimeVoltage = FixedNumeric::q15_t;
using RuntimeSpeed = FixedNumeric::q15_t;
using RuntimeAngle = FixedNumeric::phase_u32_t;
using RuntimeDuty = FixedNumeric::q15_t;

#ifndef LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES
#define LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES 8U
#endif

static_assert(LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES > 0U,
              "LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES must be positive");
static_assert(LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES <= 255U,
              "LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES must fit uint8_t");

#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
struct RuntimeVFStartupPhaseQ15
{
    uint32_t duration_ticks;
    RuntimeSpeed final_speed_rpm;
    RuntimeDuty final_duty;
    std::int32_t speed_step_q15;
    std::uint32_t speed_remainder_q15;
    std::int8_t speed_remainder_sign;
    std::int32_t duty_step_q15;
    std::uint32_t duty_remainder_q15;
    std::int8_t duty_remainder_sign;
};
#endif

#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
#ifndef LIB_MOTOR_IF_PROFILE_MAX_PHASES
#define LIB_MOTOR_IF_PROFILE_MAX_PHASES 8U
#endif

#ifndef LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES
#define LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES LIB_MOTOR_IF_PROFILE_MAX_PHASES
#endif

static_assert(LIB_MOTOR_IF_PROFILE_MAX_PHASES > 0U,
              "LIB_MOTOR_IF_PROFILE_MAX_PHASES must be positive");
static_assert(LIB_MOTOR_IF_PROFILE_MAX_PHASES <= 255U,
              "LIB_MOTOR_IF_PROFILE_MAX_PHASES must fit uint8_t");
static_assert(LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES > 0U,
              "LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES must be positive");
static_assert(LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES <= 255U,
              "LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES must fit uint8_t");

struct RuntimeIFStartupPhaseQ15
{
    MotorIFStartupPhaseType type;
    uint32_t duration_ticks;
    uint32_t hold_ticks;
    RuntimeSpeed final_speed_rpm;
    RuntimeCurrent final_id;
    RuntimeCurrent final_iq;
    std::int32_t speed_step_q15;
    std::uint32_t speed_remainder_q15;
    std::int8_t speed_remainder_sign;
    std::int32_t id_step_q15;
    std::uint32_t id_remainder_q15;
    std::int8_t id_remainder_sign;
    std::int32_t iq_step_q15;
    std::uint32_t iq_remainder_q15;
    std::int8_t iq_remainder_sign;
};
#endif

#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY || LIB_MOTOR_ENABLE_IF_STARTUP
struct RuntimeQ15ProfileState
{
    uint8_t phase;
    bool phase_started;
    bool complete;
    uint32_t phase_ticks;
    RuntimeSpeed speed_rpm;
    RuntimeDuty duty;
    RuntimeCurrent id;
    RuntimeCurrent iq;
    uint32_t speed_remainder_accum;
    uint32_t duty_remainder_accum;
    uint32_t id_remainder_accum;
    uint32_t iq_remainder_accum;
};
#endif

struct RuntimeCtx
{
    /* [A] q15 后端换算基准与 raw/count metadata */
    std::uint32_t current_base_mA = 1000U;
    std::uint32_t voltage_base_mV = 1000U;
    std::uint32_t bus_voltage_base_mV = 1000U;
    std::uint32_t speed_base_rpm_x10 = 10U;
    std::int32_t phase_current_q15_per_count = 0;
    std::int32_t bus_current_q15_per_count = 0;
    std::int32_t vbus_q15_per_count = 0;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    std::int32_t phase_voltage_q15_per_count = 0;
#endif
    std::int32_t offset_ia_counts = 0;
    std::int32_t offset_ib_counts = 0;
    std::int32_t offset_ic_counts = 0;
    std::int32_t offset_ibus_counts = 0;
    std::int32_t phase_u_delta_counts = 0;
    std::int32_t phase_v_delta_counts = 0;
    std::int32_t phase_w_delta_counts = 0;
    std::int32_t bus_delta_counts = 0;
    std::int32_t phase_overcurrent_delta_counts = 0;
    std::int32_t bus_overcurrent_delta_counts = 0;
    std::int32_t vbus_overvoltage_counts = 0;
    std::int32_t vbus_undervoltage_counts = 0;
    std::uint16_t vbus_raw_counts = 0U;
    RuntimeCurrent phase_overcurrent_q15 = FixedNumeric::kQ15One;
    RuntimeCurrent bus_overcurrent_q15 = FixedNumeric::kQ15One;
    RuntimeCurrent current_ref_limit_q15 = FixedNumeric::kQ15One;
    RuntimeVoltage vbus_overvoltage_q15 = FixedNumeric::kQ15One;
    RuntimeVoltage vbus_undervoltage_q15 = 0;
    RuntimeVoltage vbus_default_q15 = 0;
    RuntimeDuty max_duty = FixedNumeric::kQ15One;
    RuntimeVoltage current_pid_modulation_limit_q15 = FixedNumeric::kQ15One;
    RuntimeVoltage current_pid_output_limit_q15 = FixedNumeric::kQ15One;
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    std::uint32_t speed_slew_step_q31 = 0U;
    std::uint16_t speed_slew_fraction_q16 = 0U;
    RuntimeCurrent iq_drive_slew_step_q15 = 0;
    RuntimeCurrent iq_brake_slew_step_q15 = 0;
    RuntimeSpeed reverse_zero_band_q15 = 0;
#endif
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    RuntimeSpeed smo_direction_deadband_q15 = 1;
#endif
#if LIB_MOTOR_ENABLE_SMO
    RuntimeSpeed smo_handover_min_speed_q15 = FixedNumeric::kQ15One;
    RuntimeSpeed smo_handover_max_speed_error_q15 = FixedNumeric::kQ15One;
    std::uint32_t smo_handover_max_angle_error_phase = 0U;
    RuntimeSpeed smo_auto_swap_fall_speed_q15 = 0;
#endif
    FixedNumeric::SinCos angle_sin_cos{};

    /* [B] ADC 零点偏移 */
    std::uint32_t calib_accum_ia;
    std::uint32_t calib_accum_ib;
    std::uint32_t calib_accum_ic;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    std::uint32_t calib_accum_ibus;
#endif
    uint16_t calib_counter;

#if LIB_MOTOR_ENABLE_DEBUG_ANY
    /* [B2] ADC raw samples for bring-up diagnostics only. */
    uint16_t adc_raw_phase_u;
    uint16_t adc_raw_phase_v;
    uint16_t adc_raw_phase_w;
    uint16_t adc_raw_bus_current;
    uint16_t adc_raw_vbus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    uint16_t adc_raw_phase_voltage_u;
    uint16_t adc_raw_phase_voltage_v;
    uint16_t adc_raw_phase_voltage_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
    uint16_t adc_raw_temp[MOTOR_BUILD_NTC_SLOTS];
#endif
#endif

    /* [C] 传感器与观测器反馈 */
    RuntimeCurrent i_a, i_b, i_c;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    RuntimeCurrent i_bus;
#endif
    RuntimeVoltage v_bus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    RuntimeVoltage v_phase_u, v_phase_v, v_phase_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
    std::int16_t temperature_deci_c[MOTOR_BUILD_NTC_SLOTS];
    std::int16_t ntc_over_temp_deci_c[MOTOR_BUILD_NTC_SLOTS];
#endif
    RuntimeAngle angle_elec;
    RuntimeAngle angle_elec_command;
    RuntimeAngle angle_elec_observer;
    RuntimeAngle angle_elec_sensor;
    RuntimeAngle angle_mech_sensor;
    RuntimeSpeed speed_rpm;
    RuntimeSpeed speed_rpm_observer;
    RuntimeSpeed speed_rpm_sensor;
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    ObserverEstimateQ15 smo_estimate;
#endif
#if LIB_MOTOR_ENABLE_HFI
    ObserverEstimate hfi_estimate;
    HfiInjectionCommand hfi_injection;
#endif
    int32_t sensor_raw;
    bool sensor_ready;
    SensorHealth sensor_health;

    /* [D] FOC 中间量 */
    RuntimeCurrent i_alpha, i_beta;
    RuntimeCurrent i_d, i_q;
    RuntimeVoltage v_d, v_q;
    RuntimeVoltage v_alpha, v_beta;

    /* [E] 控制目标 */
    RuntimeSpeed target_rpm;
    RuntimeCurrent target_iq;
    RuntimeCurrent target_id;
    RuntimeSpeed speed_ref_limited;
    RuntimeCurrent speed_pid_iq;
    RuntimeCurrent iq_ref_command;
    RuntimeCurrent iq_ref_limited;
    RuntimeSpeed pending_reverse_rpm = 0;   // 跨零软减速暂存反向目标 (0=无待执行)

    /* [F] 最终输出 */
    RuntimeDuty duty_a, duty_b, duty_c;
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
    /* [F2] 手动 PWM 命令缓存, 与实际输出 duty 分开 */
    RuntimeDuty debug_pwm_manual_a;
    RuntimeDuty debug_pwm_manual_b;
    RuntimeDuty debug_pwm_manual_c;
#endif

    /* 调试专用配置 */
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    RuntimeDuty vf_duty_bias;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
    uint32_t debug_ramp_ticks;
    uint32_t debug_ramp_speed_step_q15;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    uint8_t debug_vf_profile_phase_count;
    RuntimeVFStartupPhaseQ15 debug_vf_profile_phases[LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES];
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    uint8_t debug_if_profile_phase_count;
    RuntimeIFStartupPhaseQ15 debug_if_profile_phases[LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES];
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
    RuntimeQ15ProfileState debug_profile;
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    uint8_t if_profile_phase_count;
    RuntimeIFStartupPhaseQ15 if_profile_phases[LIB_MOTOR_IF_PROFILE_MAX_PHASES];
    RuntimeQ15ProfileState if_profile;
#endif

    /* [G] 计数器 */
    uint32_t fsm_timer_ticks;
    uint16_t slow_loop_counter;
    uint32_t silent_calib_timer_ticks;
    bool     silent_calib_active;
};

inline float runtimeMilliBaseToFloat(std::uint32_t milli_base)
{
    return static_cast<float>(milli_base) * 0.001f;
}

inline float runtimeSpeedBaseToFloat(std::uint32_t rpm_x10_base)
{
    return static_cast<float>(rpm_x10_base) * 0.1f;
}

inline float runtimeCurrentBaseA(const RuntimeCtx& ctx)
{
    return runtimeMilliBaseToFloat(ctx.current_base_mA);
}

inline float runtimeControlVoltageBaseV(const RuntimeCtx& ctx)
{
    return runtimeMilliBaseToFloat(ctx.voltage_base_mV);
}

inline float runtimeBusVoltageBaseV(const RuntimeCtx& ctx)
{
    return runtimeMilliBaseToFloat(ctx.bus_voltage_base_mV);
}

inline float runtimeSpeedBaseRpm(const RuntimeCtx& ctx)
{
    return runtimeSpeedBaseToFloat(ctx.speed_base_rpm_x10);
}

inline float runtimeCurrentToPhysical(const RuntimeCtx& ctx, RuntimeCurrent value)
{
    return FixedNumeric::toPhysical(value, runtimeCurrentBaseA(ctx));
}

inline RuntimeCurrent runtimeCurrentFromPhysical(const RuntimeCtx& ctx, float value)
{
    return FixedNumeric::fromPhysical(value, runtimeCurrentBaseA(ctx));
}

inline float runtimeControlVoltageToPhysical(const RuntimeCtx& ctx, RuntimeVoltage value)
{
    return FixedNumeric::toPhysical(value, runtimeControlVoltageBaseV(ctx));
}

inline RuntimeVoltage runtimeControlVoltageFromPhysical(const RuntimeCtx& ctx, float value)
{
    return FixedNumeric::fromPhysical(value, runtimeControlVoltageBaseV(ctx));
}

inline float runtimeBusVoltageToPhysical(const RuntimeCtx& ctx, RuntimeVoltage value)
{
    return FixedNumeric::toPhysical(value, runtimeBusVoltageBaseV(ctx));
}

inline RuntimeVoltage runtimeBusVoltageFromPhysical(const RuntimeCtx& ctx, float value)
{
    return FixedNumeric::fromPhysical(value, runtimeBusVoltageBaseV(ctx));
}

inline float runtimeSpeedToPhysical(const RuntimeCtx& ctx, RuntimeSpeed value)
{
    return FixedNumeric::toPhysical(value, runtimeSpeedBaseRpm(ctx));
}

inline RuntimeSpeed runtimeSpeedFromPhysical(const RuntimeCtx& ctx, float value)
{
    return FixedNumeric::fromPhysical(value, runtimeSpeedBaseRpm(ctx));
}

inline float runtimeDutyToNormalized(RuntimeDuty value)
{
    return FixedNumeric::toNormalized(value);
}

inline RuntimeDuty runtimeDutyFromNormalized(float value)
{
    return FixedNumeric::fromNormalized(value);
}

inline RuntimeDuty runtimeClampDuty(RuntimeDuty value, RuntimeDuty maximum)
{
    if (value < 0)
    {
        return 0;
    }
    return (value > maximum) ? maximum : value;
}

inline float runtimeAngleToRadians(RuntimeAngle value)
{
    return static_cast<float>(value) * (TWO_PI / 4294967296.0f);
}

inline RuntimeAngle runtimeAngleFromRadians(float value)
{
    return FixedNumeric::phaseFromRadians(value);
}

#if MOTOR_BUILD_NTC_SLOTS > 0
inline float runtimeTemperatureToC(const RuntimeCtx& ctx, uint8_t index)
{
    return static_cast<float>(ctx.temperature_deci_c[index]) * 0.1f;
}
#endif

#else

using RuntimeCurrent = float;
using RuntimeVoltage = float;
using RuntimeSpeed = float;
using RuntimeAngle = float;
using RuntimeDuty = float;

struct RuntimeCtx
{
    /* [A] 预计算换算系数 */
    float current_scale_Phase_per_count;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    float current_scale_Bus_per_count;
#endif
    float voltage_scale_V_per_count;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    float phase_voltage_scale_V_per_count;
#endif

    /* [B] ADC 零点偏移 */
    float calib_accum_ia, calib_accum_ib, calib_accum_ic;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    float calib_accum_ibus;
#endif
    uint16_t calib_counter;
    float offset_ia, offset_ib, offset_ic;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    float offset_ibus;
#endif

#if LIB_MOTOR_ENABLE_DEBUG_ANY
    /* [B2] ADC raw samples for bring-up diagnostics only. */
    uint16_t adc_raw_phase_u;
    uint16_t adc_raw_phase_v;
    uint16_t adc_raw_phase_w;
    uint16_t adc_raw_bus_current;
    uint16_t adc_raw_vbus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    uint16_t adc_raw_phase_voltage_u;
    uint16_t adc_raw_phase_voltage_v;
    uint16_t adc_raw_phase_voltage_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
    uint16_t adc_raw_temp[MOTOR_BUILD_NTC_SLOTS];
#endif
#endif

    /* [C] 传感器与观测器反馈 */
    RuntimeCurrent i_a, i_b, i_c;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    RuntimeCurrent i_bus;
#endif
    RuntimeVoltage v_bus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    RuntimeVoltage v_phase_u, v_phase_v, v_phase_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
    float temperature_c[MOTOR_BUILD_NTC_SLOTS];
#endif
    RuntimeAngle angle_elec;
    RuntimeAngle angle_elec_command;
    RuntimeAngle angle_elec_observer;
    RuntimeAngle angle_elec_sensor;
    RuntimeAngle angle_mech_sensor;
    RuntimeSpeed speed_rpm;
    RuntimeSpeed speed_rpm_observer;
    RuntimeSpeed speed_rpm_sensor;
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    ObserverEstimate smo_estimate;
#endif
#if LIB_MOTOR_ENABLE_HFI
    ObserverEstimate hfi_estimate;
    HfiInjectionCommand hfi_injection;
#endif
    int32_t sensor_raw;
    bool sensor_ready;
    SensorHealth sensor_health;

#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    RuntimeAngle angle_elec_redundant_sensor;
    RuntimeAngle angle_mech_redundant_sensor;
    int32_t redundant_sensor_raw;
    bool redundant_sensor_ready;
    SensorHealth redundant_sensor_health;
    float rotor_redundancy_error;
#endif

#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    RuntimeAngle angle_mech_output_sensor;
    int32_t output_sensor_raw;
    bool output_sensor_ready;
    SensorHealth output_sensor_health;
    float transmission_deflection_rad;
#endif

    /* [D] FOC 中间量 */
    float i_alpha_raw, i_beta_raw;
    RuntimeCurrent i_alpha, i_beta;
    RuntimeCurrent i_d, i_q;
    RuntimeVoltage v_d, v_q;
    RuntimeVoltage v_alpha, v_beta;
    float sin_angle_elec = 0.0f;
    float cos_angle_elec = 1.0f;

    /* [E] 控制目标 */
    RuntimeSpeed target_rpm;
    RuntimeCurrent target_iq;
    RuntimeCurrent target_id;
    RuntimeSpeed speed_ref_limited;
    RuntimeCurrent speed_pid_iq;
    RuntimeCurrent iq_ref_command;
    RuntimeCurrent iq_ref_limited;
    RuntimeSpeed pending_reverse_rpm = 0.0f;   // 跨零软减速暂存反向目标 (0=无待执行)

    /* === Phase 2: 位置/前馈通道 === */
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
    float target_pos_rad;
#endif
#if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
    float vel_ff_rad_s;
#endif
#if LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE
    uint32_t last_setpoint_tick;
#endif

    /* === Phase 3 占位: 力矩前馈通道 === */
#if LIB_MOTOR_ENABLE_TORQUE_FEEDFORWARD
    float torque_ff_a;
#endif

    /* === Phase 3 占位: 阻抗控制 === */
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
    float imp_pos_ref_rad;
    float imp_stiffness_n_m;
    float imp_damping_n_m_s;
    float imp_torque_bias_a;
    float imp_torque_limit_a;
#endif

    /* [F] 最终输出 */
    RuntimeDuty duty_a, duty_b, duty_c;

    /* [G] 计数器 */
    uint32_t fsm_timer_ticks;
    uint16_t slow_loop_counter;
    uint32_t silent_calib_timer_ticks;
    bool     silent_calib_active;

    /* 调试专用配置 */
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    float vf_duty_bias;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
    float test_u, test_v, test_w;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
    float debug_ramp_time_s;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    const MotorVFStartupProfile* debug_vf_profile;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    const MotorIFStartupProfile* debug_if_profile;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
    uint8_t debug_profile_phase;
    float debug_profile_phase_time_s;
    float debug_profile_start_speed_rpm;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    float debug_profile_start_duty;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    float debug_profile_start_id;
    float debug_profile_start_iq;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
    bool debug_profile_phase_started;
    bool debug_profile_complete;
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    uint8_t if_profile_phase;
    float if_profile_phase_time_s;
    float if_profile_start_speed_rpm;
    float if_profile_start_id;
    float if_profile_start_iq;
    bool if_profile_phase_started;
    bool if_profile_complete;
#endif
};

inline float runtimeCurrentToPhysical(const RuntimeCtx&, RuntimeCurrent value) { return value; }
inline RuntimeCurrent runtimeCurrentFromPhysical(const RuntimeCtx&, float value) { return value; }
inline float runtimeControlVoltageToPhysical(const RuntimeCtx&, RuntimeVoltage value) { return value; }
inline RuntimeVoltage runtimeControlVoltageFromPhysical(const RuntimeCtx&, float value) { return value; }
inline float runtimeBusVoltageToPhysical(const RuntimeCtx&, RuntimeVoltage value) { return value; }
inline RuntimeVoltage runtimeBusVoltageFromPhysical(const RuntimeCtx&, float value) { return value; }
inline float runtimeSpeedToPhysical(const RuntimeCtx&, RuntimeSpeed value) { return value; }
inline RuntimeSpeed runtimeSpeedFromPhysical(const RuntimeCtx&, float value) { return value; }
inline float runtimeDutyToNormalized(RuntimeDuty value) { return value; }
inline RuntimeDuty runtimeDutyFromNormalized(float value) { return value; }
inline RuntimeDuty runtimeClampDuty(RuntimeDuty value, RuntimeDuty maximum)
{
    if (value < 0.0f) return 0.0f;
    return (value > maximum) ? maximum : value;
}
inline float runtimeAngleToRadians(RuntimeAngle value) { return value; }
inline RuntimeAngle runtimeAngleFromRadians(float value) { return value; }

#if MOTOR_BUILD_NTC_SLOTS > 0
inline float runtimeTemperatureToC(const RuntimeCtx& ctx, uint8_t index)
{
    return ctx.temperature_c[index];
}
#endif

#endif

inline int8_t runtimeCommandDirection(const MotorConfig& cfg)
{
    return (cfg.control.command_direction < 0) ? static_cast<int8_t>(-1) : static_cast<int8_t>(1);
}

inline float runtimeUserSpeedToInternalPhysical(const MotorConfig& cfg, float rpm)
{
    return rpm * static_cast<float>(runtimeCommandDirection(cfg));
}

inline float runtimeInternalSpeedToUserPhysical(const MotorConfig& cfg, float rpm)
{
    return rpm * static_cast<float>(runtimeCommandDirection(cfg));
}

inline RuntimeSpeed runtimeSpeedFromUserPhysical(const MotorConfig& cfg,
                                                 const RuntimeCtx& ctx,
                                                 float rpm)
{
    return runtimeSpeedFromPhysical(ctx, runtimeUserSpeedToInternalPhysical(cfg, rpm));
}

inline float runtimeSpeedToUserPhysical(const MotorConfig& cfg,
                                        const RuntimeCtx& ctx,
                                        RuntimeSpeed value)
{
    return runtimeInternalSpeedToUserPhysical(cfg, runtimeSpeedToPhysical(ctx, value));
}

inline RuntimeAngle runtimeAddPiToAngle(RuntimeAngle angle)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    return static_cast<RuntimeAngle>(angle + static_cast<RuntimeAngle>(0x80000000UL));
#else
    float corrected = angle + PI;
    while (corrected >= TWO_PI)
    {
        corrected -= TWO_PI;
    }
    while (corrected < 0.0f)
    {
        corrected += TWO_PI;
    }
    return corrected;
#endif
}

inline RuntimeAngle runtimeCorrectSmoAngleForDirection(RuntimeAngle raw_angle,
                                                       int8_t internal_direction)
{
    return (internal_direction > 0) ? runtimeAddPiToAngle(raw_angle) : raw_angle;
}

} // namespace Lib_Motor
