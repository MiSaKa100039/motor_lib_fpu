#include "../Manager/Motor_Manager.h"

#include "../../Common/Math/FocMath.h"

namespace Lib_Motor
{

namespace
{

float wrapSignedAngle(float angle)
{
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
float fastCurrentToPhysical(const FastRuntimeCtx& fast, FastRuntimeCtx::Current value)
{
    return FixedNumeric::toPhysical(value, fast.current_base_a);
}

float fastVoltageToPhysical(const FastRuntimeCtx& fast, FastRuntimeCtx::Voltage value)
{
    return FixedNumeric::toPhysical(value, fast.voltage_base_v);
}

float fastSpeedToPhysical(const FastRuntimeCtx& fast, FastRuntimeCtx::Speed value)
{
    return FixedNumeric::toPhysical(value, fast.speed_base_rpm);
}

float fastDutyToPhysical(FastRuntimeCtx::Duty value)
{
    return FixedNumeric::toNormalized(value);
}
#endif

} // namespace

/* 实时状态直读 -- 从 ctx_ / config_ 快速返回, 无计算开销 */
float MotorManager::getSpeed() const      { return ctx_.speed_rpm; }
float MotorManager::getBusVoltage() const { return ctx_.v_bus; }
float MotorManager::getCurrentQ() const
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    return fastCurrentToPhysical(fast_ctx_, fast_ctx_.i_q);
#else
    return ctx_.i_q;
#endif
}
uint8_t MotorManager::getPolePairs() const { return config_.physical.pole_pairs; }

/* getMonitorData -- Tier1 监控数据: 18 个字段 (从 ctx_ 镜像到外部结构体) */
void MotorManager::getMonitorData(MotorMonitorData& data) const
{
    data.config_fault_detail = config_fault_detail_;
    data.state        = state_;
    data.fault_code   = fault_;
    data.v_bus        = ctx_.v_bus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    data.v_phase_u    = ctx_.v_phase_u;
    data.v_phase_v    = ctx_.v_phase_v;
    data.v_phase_w    = ctx_.v_phase_w;
#endif
    data.speed_rpm    = ctx_.speed_rpm;
    data.speed_rpm_observer = ctx_.speed_rpm_observer;
    data.speed_rpm_sensor = ctx_.speed_rpm_sensor;
#if MOTOR_BUILD_NTC_SLOTS > 0
    for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; ++i)
    {
        data.temperature[i] = ctx_.temperature_c[i];
    }
#endif
    data.mode         = mode_;
    data.target_mode  = target_mode_;
    data.run_phase    = run_phase_;
    data.event        = event_;
    data.energy_state = energy_state_;
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    data.stall_total_count = stall_total_count_;
#endif
    data.i_a          = ctx_.i_a;
    data.i_b          = ctx_.i_b;
    data.i_c          = ctx_.i_c;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    data.i_bus        = ctx_.i_bus;
#endif
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    data.i_d          = fastCurrentToPhysical(fast_ctx_, fast_ctx_.i_d);
    data.i_q          = fastCurrentToPhysical(fast_ctx_, fast_ctx_.i_q);
#else
    data.i_d          = ctx_.i_d;
    data.i_q          = ctx_.i_q;
#endif
    data.angle_elec   = ctx_.angle_elec;
    data.angle_elec_command  = ctx_.angle_elec_command;
    data.angle_elec_observer = ctx_.angle_elec_observer;
    data.angle_elec_sensor   = ctx_.angle_elec_sensor;
    data.angle_mech_sensor   = ctx_.angle_mech_sensor;
    data.sensor_raw          = ctx_.sensor_raw;
    data.sensor_ready        = ctx_.sensor_ready;
    data.sensor_health       = ctx_.sensor_health;
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    data.angle_elec_redundant_sensor = ctx_.angle_elec_redundant_sensor;
    data.angle_mech_redundant_sensor = ctx_.angle_mech_redundant_sensor;
    data.redundant_sensor_raw = ctx_.redundant_sensor_raw;
    data.redundant_sensor_ready = ctx_.redundant_sensor_ready;
    data.redundant_sensor_health = ctx_.redundant_sensor_health;
    data.rotor_redundancy_error = ctx_.rotor_redundancy_error;
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    data.angle_mech_output_sensor = ctx_.angle_mech_output_sensor;
    data.output_sensor_raw = ctx_.output_sensor_raw;
    data.output_sensor_ready = ctx_.output_sensor_ready;
    data.output_sensor_health = ctx_.output_sensor_health;
    data.transmission_deflection_rad = ctx_.transmission_deflection_rad;
#endif
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    data.duty_a       = fastDutyToPhysical(fast_ctx_.duty_a);
    data.duty_b       = fastDutyToPhysical(fast_ctx_.duty_b);
    data.duty_c       = fastDutyToPhysical(fast_ctx_.duty_c);
#else
    data.duty_a       = ctx_.duty_a;
    data.duty_b       = ctx_.duty_b;
    data.duty_c       = ctx_.duty_c;
#endif
    data.observer_error = ctx_.sensor_ready
        ? wrapSignedAngle(ctx_.angle_elec_observer - ctx_.angle_elec_sensor)
        : 0.0f;
    data.observer_pll_error = 0.0f;
    data.observer_signal_level = 0.0f;
    data.observer_hfi_demod_alpha = 0.0f;
    data.observer_hfi_demod_beta = 0.0f;
    data.observer_hfi_demod_d = 0.0f;
    data.observer_hfi_demod_q = 0.0f;
    data.observer_hfi_raw_demod_alpha = 0.0f;
    data.observer_hfi_raw_demod_beta = 0.0f;
    data.observer_hfi_meas_2theta = 0.0f;
    data.observer_quality = 0.0f;
    data.observer_valid_ticks = 0U;
#if LIB_MOTOR_ENABLE_HFI
    if (angle_state_ == AngleState::HFI)
    {
        data.observer_pll_error = ctx_.hfi_estimate.pll_error_rad;
        data.observer_signal_level = ctx_.hfi_estimate.signal_level;
        data.observer_hfi_demod_alpha = ctx_.hfi_estimate.hfi_demod_alpha;
        data.observer_hfi_demod_beta = ctx_.hfi_estimate.hfi_demod_beta;
        data.observer_hfi_demod_d = ctx_.hfi_estimate.hfi_demod_d;
        data.observer_hfi_demod_q = ctx_.hfi_estimate.hfi_demod_q;
        data.observer_hfi_raw_demod_alpha = ctx_.hfi_estimate.hfi_raw_demod_alpha;
        data.observer_hfi_raw_demod_beta = ctx_.hfi_estimate.hfi_raw_demod_beta;
        data.observer_hfi_meas_2theta = ctx_.hfi_estimate.hfi_meas_2theta;
        data.observer_quality = ctx_.hfi_estimate.quality;
        data.observer_valid_ticks = ctx_.hfi_estimate.valid_ticks;
    }
#endif
#if LIB_MOTOR_ENABLE_SMO
    if (angle_state_ == AngleState::SMO)
    {
        data.observer_pll_error = ctx_.smo_estimate.pll_error_rad;
        data.observer_signal_level = ctx_.smo_estimate.signal_level;
        data.observer_quality = ctx_.smo_estimate.quality;
        data.observer_valid_ticks = ctx_.smo_estimate.valid_ticks;
    }
#endif
}

/* getDebugData -- Tier2 调试数据: 11 个字段 (FOC 中间量, PID 调试值) */
void MotorManager::getDebugData(MotorDebugData& data) const
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    data.i_alpha    = fastCurrentToPhysical(fast_ctx_, fast_ctx_.i_alpha);
    data.i_beta     = fastCurrentToPhysical(fast_ctx_, fast_ctx_.i_beta);
    data.v_d        = fastVoltageToPhysical(fast_ctx_, fast_ctx_.v_d);
    data.v_q        = fastVoltageToPhysical(fast_ctx_, fast_ctx_.v_q);
    data.v_alpha    = fastVoltageToPhysical(fast_ctx_, fast_ctx_.v_alpha);
    data.v_beta     = fastVoltageToPhysical(fast_ctx_, fast_ctx_.v_beta);
    data.target_id  = fastCurrentToPhysical(fast_ctx_, fast_ctx_.target_id);
    data.target_iq  = fastCurrentToPhysical(fast_ctx_, fast_ctx_.target_iq);
    data.target_rpm = fastSpeedToPhysical(fast_ctx_, fast_ctx_.target_speed);
    data.speed_ref_limited = fastSpeedToPhysical(fast_ctx_, fast_ctx_.speed_ref_limited);
    data.speed_pid_iq = fastCurrentToPhysical(fast_ctx_, fast_ctx_.speed_pid_iq);
    data.iq_ref_command = fastCurrentToPhysical(fast_ctx_, fast_ctx_.iq_ref_command);
    data.iq_ref_limited = fastCurrentToPhysical(fast_ctx_, fast_ctx_.iq_ref_limited);
#else
    data.i_alpha    = ctx_.i_alpha;
    data.i_beta     = ctx_.i_beta;
    data.v_d        = ctx_.v_d;
    data.v_q        = ctx_.v_q;
    data.v_alpha    = ctx_.v_alpha;
    data.v_beta     = ctx_.v_beta;
    data.target_id  = ctx_.target_id;
    data.target_iq  = ctx_.target_iq;
    data.target_rpm = ctx_.target_rpm;
    data.speed_ref_limited = ctx_.speed_ref_limited;
    data.speed_pid_iq = ctx_.speed_pid_iq;
    data.iq_ref_command = ctx_.iq_ref_command;
    data.iq_ref_limited = ctx_.iq_ref_limited;
#endif
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
    data.target_pos_rad = ctx_.target_pos_rad;
#else
    data.target_pos_rad = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
    data.vel_ff_rad_s = ctx_.vel_ff_rad_s;
#else
    data.vel_ff_rad_s = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_TORQUE_FEEDFORWARD
    data.torque_ff_a = ctx_.torque_ff_a;
#else
    data.torque_ff_a = 0.0f;
#endif
    data.stream_state = stream_state_;
#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    data.playback_state = static_cast<uint8_t>(setpoint_playback_state_);
    data.playback_count = setpoint_fifo_count_;
#else
    data.playback_state = 0U;
    data.playback_count = 0U;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
    for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; ++i)
    {
        data.temperature_c[i] = ctx_.temperature_c[i];
    }
#endif
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    data.adc_raw_phase_u = ctx_.adc_raw_phase_u;
    data.adc_raw_phase_v = ctx_.adc_raw_phase_v;
    data.adc_raw_phase_w = ctx_.adc_raw_phase_w;
    data.adc_raw_bus_current = ctx_.adc_raw_bus_current;
    data.adc_raw_vbus = ctx_.adc_raw_vbus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    data.adc_raw_phase_voltage_u = ctx_.adc_raw_phase_voltage_u;
    data.adc_raw_phase_voltage_v = ctx_.adc_raw_phase_voltage_v;
    data.adc_raw_phase_voltage_w = ctx_.adc_raw_phase_voltage_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
    for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; ++i)
    {
        data.adc_raw_temp[i] = ctx_.adc_raw_temp[i];
    }
#endif
#endif
}
} // namespace Lib_Motor

