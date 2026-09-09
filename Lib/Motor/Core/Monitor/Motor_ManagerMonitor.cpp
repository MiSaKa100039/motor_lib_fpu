#include "../Manager/Motor_Manager.h"

#include "../../Control/Utils/FocMath.h"

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

} // namespace

/* 实时状态直读 -- 从 ctx_ / config_ 快速返回, 无计算开销 */
float MotorManager::getSpeed() const      { return runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_rpm); }
float MotorManager::getBusVoltage() const { return runtimeBusVoltageToPhysical(ctx_, ctx_.v_bus); }
float MotorManager::getCurrentQ() const
{
    return runtimeCurrentToPhysical(ctx_, ctx_.i_q);
}
uint8_t MotorManager::getPolePairs() const { return config_.physical.pole_pairs; }

uint8_t MotorManager::getTelemetryFloats(const MotorTelemetryChannel* channels,
                                         uint8_t channel_count,
                                         float* out_values) const
{
    if (channels == nullptr || out_values == nullptr)
    {
        return 0U;
    }

    const auto getObserverError = [this]() -> float
    {
        return ctx_.sensor_ready
            ? wrapSignedAngle(runtimeAngleToRadians(ctx_.angle_elec_observer) -
                              runtimeAngleToRadians(ctx_.angle_elec_sensor))
            : 0.0f;
    };

    const auto getObserverPllError = [this]() -> float
    {
#if LIB_MOTOR_ENABLE_HFI
        if (angle_state_ == AngleState::HFI)
        {
            return ctx_.hfi_estimate.pll_error_rad;
        }
#endif
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
        if (angle_state_ == AngleState::SMO)
        {
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
            return FixedNumeric::toNormalized(ctx_.smo_estimate.pll_error_q15) * PI;
#else
            return ctx_.smo_estimate.pll_error_rad;
#endif
        }
#endif
        return 0.0f;
    };

    const auto getObserverSignalLevel = [this]() -> float
    {
#if LIB_MOTOR_ENABLE_HFI
        if (angle_state_ == AngleState::HFI)
        {
            return ctx_.hfi_estimate.signal_level;
        }
#endif
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
        if (angle_state_ == AngleState::SMO)
        {
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
            return runtimeControlVoltageToPhysical(ctx_, ctx_.smo_estimate.signal_level_q15);
#else
            return ctx_.smo_estimate.signal_level;
#endif
        }
#endif
        return 0.0f;
    };

    const auto getObserverQuality = [this]() -> float
    {
#if LIB_MOTOR_ENABLE_HFI
        if (angle_state_ == AngleState::HFI)
        {
            return ctx_.hfi_estimate.quality;
        }
#endif
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
        if (angle_state_ == AngleState::SMO)
        {
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
            return FixedNumeric::toNormalized(ctx_.smo_estimate.quality_q15);
#else
            return ctx_.smo_estimate.quality;
#endif
        }
#endif
        return 0.0f;
    };

    const auto getObserverValidTicks = [this]() -> float
    {
#if LIB_MOTOR_ENABLE_HFI
        if (angle_state_ == AngleState::HFI)
        {
            return static_cast<float>(ctx_.hfi_estimate.valid_ticks);
        }
#endif
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
        if (angle_state_ == AngleState::SMO)
        {
            return static_cast<float>(ctx_.smo_estimate.valid_ticks);
        }
#endif
        return 0.0f;
    };

    for (uint8_t i = 0U; i < channel_count; ++i)
    {
        float value = 0.0f;
        switch (channels[i])
        {
            case MotorTelemetryChannel::PhaseCurrentA:
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_a);
                break;
            case MotorTelemetryChannel::PhaseCurrentB:
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_b);
                break;
            case MotorTelemetryChannel::PhaseCurrentC:
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_c);
                break;
            case MotorTelemetryChannel::BusCurrent:
#if MOTOR_BUILD_HAS_BUS_CURRENT
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_bus);
#endif
                break;
            case MotorTelemetryChannel::BusVoltage:
                value = runtimeBusVoltageToPhysical(ctx_, ctx_.v_bus);
                break;
            case MotorTelemetryChannel::SpeedRpm:
                value = runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_rpm);
                break;
            case MotorTelemetryChannel::SpeedRpmObserver:
                value = runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_rpm_observer);
                break;
            case MotorTelemetryChannel::SpeedRpmSensor:
                value = runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_rpm_sensor);
                break;
            case MotorTelemetryChannel::Id:
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_d);
                break;
            case MotorTelemetryChannel::Iq:
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_q);
                break;
            case MotorTelemetryChannel::IAlpha:
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_alpha);
                break;
            case MotorTelemetryChannel::IBeta:
                value = runtimeCurrentToPhysical(ctx_, ctx_.i_beta);
                break;
            case MotorTelemetryChannel::Vd:
                value = runtimeControlVoltageToPhysical(ctx_, ctx_.v_d);
                break;
            case MotorTelemetryChannel::Vq:
                value = runtimeControlVoltageToPhysical(ctx_, ctx_.v_q);
                break;
            case MotorTelemetryChannel::VAlpha:
                value = runtimeControlVoltageToPhysical(ctx_, ctx_.v_alpha);
                break;
            case MotorTelemetryChannel::VBeta:
                value = runtimeControlVoltageToPhysical(ctx_, ctx_.v_beta);
                break;
            case MotorTelemetryChannel::TargetId:
                value = runtimeCurrentToPhysical(ctx_, ctx_.target_id);
                break;
            case MotorTelemetryChannel::TargetIq:
                value = runtimeCurrentToPhysical(ctx_, ctx_.target_iq);
                break;
            case MotorTelemetryChannel::TargetRpm:
                value = runtimeSpeedToUserPhysical(config_, ctx_, ctx_.target_rpm);
                break;
            case MotorTelemetryChannel::SpeedRefLimited:
                value = runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_ref_limited);
                break;
            case MotorTelemetryChannel::SpeedPidIq:
                value = runtimeCurrentToPhysical(ctx_, ctx_.speed_pid_iq);
                break;
            case MotorTelemetryChannel::IqRefCommand:
                value = runtimeCurrentToPhysical(ctx_, ctx_.iq_ref_command);
                break;
            case MotorTelemetryChannel::IqRefLimited:
                value = runtimeCurrentToPhysical(ctx_, ctx_.iq_ref_limited);
                break;
            case MotorTelemetryChannel::DutyA:
                value = runtimeDutyToNormalized(ctx_.duty_a);
                break;
            case MotorTelemetryChannel::DutyB:
                value = runtimeDutyToNormalized(ctx_.duty_b);
                break;
            case MotorTelemetryChannel::DutyC:
                value = runtimeDutyToNormalized(ctx_.duty_c);
                break;
            case MotorTelemetryChannel::AngleElec:
                value = runtimeAngleToRadians(ctx_.angle_elec);
                break;
            case MotorTelemetryChannel::AngleCommand:
                value = runtimeAngleToRadians(ctx_.angle_elec_command);
                break;
            case MotorTelemetryChannel::AngleElecObserver:
                value = runtimeAngleToRadians(ctx_.angle_elec_observer);
                break;
            case MotorTelemetryChannel::AngleElecSensor:
                value = runtimeAngleToRadians(ctx_.angle_elec_sensor);
                break;
            case MotorTelemetryChannel::AngleMechSensor:
                value = runtimeAngleToRadians(ctx_.angle_mech_sensor);
                break;
            case MotorTelemetryChannel::ObserverError:
                value = getObserverError();
                break;
            case MotorTelemetryChannel::ObserverPllError:
                value = getObserverPllError();
                break;
            case MotorTelemetryChannel::ObserverSignalLevel:
                value = getObserverSignalLevel();
                break;
            case MotorTelemetryChannel::ObserverQuality:
                value = getObserverQuality();
                break;
            case MotorTelemetryChannel::ObserverValidTicks:
                value = getObserverValidTicks();
                break;
            case MotorTelemetryChannel::ObserverConverged:
                value = (event_.observer_converged != 0U) ? 1.0f : 0.0f;
                break;
            case MotorTelemetryChannel::Temperature0:
#if MOTOR_BUILD_NTC_SLOTS > 0
                value = runtimeTemperatureToC(ctx_, 0);
#endif
                break;
            case MotorTelemetryChannel::Temperature1:
#if MOTOR_BUILD_NTC_SLOTS > 1
                value = runtimeTemperatureToC(ctx_, 1);
#endif
                break;
            case MotorTelemetryChannel::Temperature2:
#if MOTOR_BUILD_NTC_SLOTS > 2
                value = runtimeTemperatureToC(ctx_, 2);
#endif
                break;
            case MotorTelemetryChannel::Temperature3:
#if MOTOR_BUILD_NTC_SLOTS > 3
                value = runtimeTemperatureToC(ctx_, 3);
#endif
                break;
            case MotorTelemetryChannel::AdcRawPhaseU:
#if LIB_MOTOR_ENABLE_DEBUG_ANY
                value = static_cast<float>(ctx_.adc_raw_phase_u);
#endif
                break;
            case MotorTelemetryChannel::AdcRawPhaseV:
#if LIB_MOTOR_ENABLE_DEBUG_ANY
                value = static_cast<float>(ctx_.adc_raw_phase_v);
#endif
                break;
            case MotorTelemetryChannel::AdcRawPhaseW:
#if LIB_MOTOR_ENABLE_DEBUG_ANY
                value = static_cast<float>(ctx_.adc_raw_phase_w);
#endif
                break;
            case MotorTelemetryChannel::AdcRawBusCurrent:
#if LIB_MOTOR_ENABLE_DEBUG_ANY
                value = static_cast<float>(ctx_.adc_raw_bus_current);
#endif
                break;
            case MotorTelemetryChannel::AdcRawVbus:
#if LIB_MOTOR_ENABLE_DEBUG_ANY
                value = static_cast<float>(ctx_.adc_raw_vbus);
#endif
                break;
            case MotorTelemetryChannel::AdcRawPhaseVoltageU:
#if LIB_MOTOR_ENABLE_DEBUG_ANY && MOTOR_BUILD_HAS_PHASE_VOLTAGE
                value = static_cast<float>(ctx_.adc_raw_phase_voltage_u);
#endif
                break;
            case MotorTelemetryChannel::AdcRawPhaseVoltageV:
#if LIB_MOTOR_ENABLE_DEBUG_ANY && MOTOR_BUILD_HAS_PHASE_VOLTAGE
                value = static_cast<float>(ctx_.adc_raw_phase_voltage_v);
#endif
                break;
            case MotorTelemetryChannel::AdcRawPhaseVoltageW:
#if LIB_MOTOR_ENABLE_DEBUG_ANY && MOTOR_BUILD_HAS_PHASE_VOLTAGE
                value = static_cast<float>(ctx_.adc_raw_phase_voltage_w);
#endif
                break;
            case MotorTelemetryChannel::AdcRawTemp0:
#if LIB_MOTOR_ENABLE_DEBUG_ANY && MOTOR_BUILD_NTC_SLOTS > 0
                value = static_cast<float>(ctx_.adc_raw_temp[0]);
#endif
                break;
            case MotorTelemetryChannel::AdcRawTemp1:
#if LIB_MOTOR_ENABLE_DEBUG_ANY && MOTOR_BUILD_NTC_SLOTS > 1
                value = static_cast<float>(ctx_.adc_raw_temp[1]);
#endif
                break;
            case MotorTelemetryChannel::AdcRawTemp2:
#if LIB_MOTOR_ENABLE_DEBUG_ANY && MOTOR_BUILD_NTC_SLOTS > 2
                value = static_cast<float>(ctx_.adc_raw_temp[2]);
#endif
                break;
            case MotorTelemetryChannel::AdcRawTemp3:
#if LIB_MOTOR_ENABLE_DEBUG_ANY && MOTOR_BUILD_NTC_SLOTS > 3
                value = static_cast<float>(ctx_.adc_raw_temp[3]);
#endif
                break;
            default:
                break;
        }
        out_values[i] = value;
    }

    return channel_count;
}

/* getMonitorData -- Tier1 监控数据: 18 个字段 (从 ctx_ 镜像到外部结构体) */
void MotorManager::getMonitorData(MotorMonitorData& data) const
{
    data.config_fault_detail = config_fault_detail_;
    data.runtime_fault_detail = runtime_fault_detail_;
    data.state        = state_;
    data.fault_code   = fault_;
    data.v_bus        = runtimeBusVoltageToPhysical(ctx_, ctx_.v_bus);
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    data.v_phase_u    = runtimeBusVoltageToPhysical(ctx_, ctx_.v_phase_u);
    data.v_phase_v    = runtimeBusVoltageToPhysical(ctx_, ctx_.v_phase_v);
    data.v_phase_w    = runtimeBusVoltageToPhysical(ctx_, ctx_.v_phase_w);
#endif
    data.speed_rpm    = runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_rpm);
    data.speed_rpm_observer =
        runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_rpm_observer);
    data.speed_rpm_sensor =
        runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_rpm_sensor);
#if MOTOR_BUILD_NTC_SLOTS > 0
    for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; ++i)
    {
        data.temperature[i] = runtimeTemperatureToC(ctx_, i);
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
    data.i_a          = runtimeCurrentToPhysical(ctx_, ctx_.i_a);
    data.i_b          = runtimeCurrentToPhysical(ctx_, ctx_.i_b);
    data.i_c          = runtimeCurrentToPhysical(ctx_, ctx_.i_c);
#if MOTOR_BUILD_HAS_BUS_CURRENT
    data.i_bus        = runtimeCurrentToPhysical(ctx_, ctx_.i_bus);
#endif
    data.i_d          = runtimeCurrentToPhysical(ctx_, ctx_.i_d);
    data.i_q          = runtimeCurrentToPhysical(ctx_, ctx_.i_q);
    data.angle_elec   = runtimeAngleToRadians(ctx_.angle_elec);
    data.angle_elec_command  = runtimeAngleToRadians(ctx_.angle_elec_command);
    data.angle_elec_observer = runtimeAngleToRadians(ctx_.angle_elec_observer);
    data.angle_elec_sensor   = runtimeAngleToRadians(ctx_.angle_elec_sensor);
    data.angle_mech_sensor   = runtimeAngleToRadians(ctx_.angle_mech_sensor);
    data.sensor_raw          = ctx_.sensor_raw;
    data.sensor_ready        = ctx_.sensor_ready;
    data.sensor_health       = ctx_.sensor_health;
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    data.angle_elec_redundant_sensor =
        runtimeAngleToRadians(ctx_.angle_elec_redundant_sensor);
    data.angle_mech_redundant_sensor =
        runtimeAngleToRadians(ctx_.angle_mech_redundant_sensor);
    data.redundant_sensor_raw = ctx_.redundant_sensor_raw;
    data.redundant_sensor_ready = ctx_.redundant_sensor_ready;
    data.redundant_sensor_health = ctx_.redundant_sensor_health;
    data.rotor_redundancy_error = ctx_.rotor_redundancy_error;
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    data.angle_mech_output_sensor =
        runtimeAngleToRadians(ctx_.angle_mech_output_sensor);
    data.output_sensor_raw = ctx_.output_sensor_raw;
    data.output_sensor_ready = ctx_.output_sensor_ready;
    data.output_sensor_health = ctx_.output_sensor_health;
    data.transmission_deflection_rad = ctx_.transmission_deflection_rad;
#endif
    data.duty_a       = runtimeDutyToNormalized(ctx_.duty_a);
    data.duty_b       = runtimeDutyToNormalized(ctx_.duty_b);
    data.duty_c       = runtimeDutyToNormalized(ctx_.duty_c);
    data.observer_error = ctx_.sensor_ready
        ? wrapSignedAngle(runtimeAngleToRadians(ctx_.angle_elec_observer) -
                          runtimeAngleToRadians(ctx_.angle_elec_sensor))
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
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    if (angle_state_ == AngleState::SMO)
    {
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        data.observer_pll_error =
            FixedNumeric::toNormalized(ctx_.smo_estimate.pll_error_q15) * PI;
        data.observer_signal_level =
            runtimeControlVoltageToPhysical(ctx_, ctx_.smo_estimate.signal_level_q15);
        data.observer_quality =
            FixedNumeric::toNormalized(ctx_.smo_estimate.quality_q15);
        data.observer_valid_ticks = ctx_.smo_estimate.valid_ticks;
#else
        data.observer_pll_error = ctx_.smo_estimate.pll_error_rad;
        data.observer_signal_level = ctx_.smo_estimate.signal_level;
        data.observer_quality = ctx_.smo_estimate.quality;
        data.observer_valid_ticks = ctx_.smo_estimate.valid_ticks;
#endif
    }
#endif
}

/* getDebugData -- Tier2 调试数据: 11 个字段 (FOC 中间量, PID 调试值) */
void MotorManager::getDebugData(MotorDebugData& data) const
{
    data.i_alpha    = runtimeCurrentToPhysical(ctx_, ctx_.i_alpha);
    data.i_beta     = runtimeCurrentToPhysical(ctx_, ctx_.i_beta);
    data.v_d        = runtimeControlVoltageToPhysical(ctx_, ctx_.v_d);
    data.v_q        = runtimeControlVoltageToPhysical(ctx_, ctx_.v_q);
    data.v_alpha    = runtimeControlVoltageToPhysical(ctx_, ctx_.v_alpha);
    data.v_beta     = runtimeControlVoltageToPhysical(ctx_, ctx_.v_beta);
    data.target_id  = runtimeCurrentToPhysical(ctx_, ctx_.target_id);
    data.target_iq  = runtimeCurrentToPhysical(ctx_, ctx_.target_iq);
    data.target_rpm = runtimeSpeedToUserPhysical(config_, ctx_, ctx_.target_rpm);
    data.speed_ref_limited =
        runtimeSpeedToUserPhysical(config_, ctx_, ctx_.speed_ref_limited);
    data.speed_pid_iq = runtimeCurrentToPhysical(ctx_, ctx_.speed_pid_iq);
    data.iq_ref_command = runtimeCurrentToPhysical(ctx_, ctx_.iq_ref_command);
    data.iq_ref_limited = runtimeCurrentToPhysical(ctx_, ctx_.iq_ref_limited);
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
        data.temperature_c[i] = runtimeTemperatureToC(ctx_, i);
    }
#endif
#if LIB_MOTOR_ENABLE_DEBUG_ANY
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
