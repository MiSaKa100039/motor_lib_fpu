#include "Motor_AngleFSM.h"

#include "../../Feature/Motor_FeatureRegistry.h"
#include "../../Manager/Motor_Manager.h"
#include "../../../Common/Math/FocMath.h"
#include "../../../Safety/Basic/Motor_ConfigCheck.h"

#include <cmath>

namespace Lib_Motor
{

void Motor_AngleFSM::step(MotorManager& m)
{
    if (!MotorFeatureRegistry::isClosedLoopSourceImplemented(
            m.active_policy_.steady_source))
    {
        faultUnavailable(m, AngleState::UNAVAILABLE, Fault::OBSERVER_UNAVAILABLE);
        return;
    }

    switch (m.active_policy_.steady_source)
    {
        case SteadyAngleSource::SENSOR:
            useSensorSource(m);
            break;

        case SteadyAngleSource::SMO:
            useSmoSource(m);
            break;

        case SteadyAngleSource::HFI:
            useHfiSource(m);
            break;

        default:
            faultUnavailable(m, AngleState::UNAVAILABLE, Fault::OBSERVER_UNAVAILABLE);
            break;
    }
}

bool Motor_AngleFSM::validateClosedLoopEntry(MotorManager& m)
{
    if (!m.checkConfigValid(m.config_))
    {
        m.setFault(Fault::PARAM_ERROR);
        m.angle_state_ = AngleState::UNAVAILABLE;
        return false;
    }

    const Fault feedback_fault =
        MotorConfigCheck::validateFeedback(m.config_, m.active_policy_);
    if (feedback_fault != Fault::NONE)
    {
        m.setFault(feedback_fault);
        m.angle_state_ = AngleState::UNAVAILABLE;
        return false;
    }

    if (!MotorFeatureRegistry::isClosedLoopSourceImplemented(
            m.active_policy_.steady_source))
    {
        m.setFault(Fault::OBSERVER_UNAVAILABLE);
        m.angle_state_ = AngleState::UNAVAILABLE;
        return false;
    }

    if (m.active_policy_.steady_source == SteadyAngleSource::SENSOR &&
        !m.ctx_.sensor_ready)
    {
        m.setFault(Fault::SENSOR_LOSS);
        m.angle_state_ = AngleState::UNAVAILABLE;
        return false;
    }

    switch (m.active_policy_.steady_source)
    {
        case SteadyAngleSource::SMO:
            m.angle_state_ = AngleState::SMO;
            return true;

        case SteadyAngleSource::SENSOR:
            m.angle_state_ = AngleState::SENSOR;
            return true;

        case SteadyAngleSource::HFI:
            m.angle_state_ = AngleState::HFI;
            return true;

        default:
            m.angle_state_ = AngleState::UNAVAILABLE;
            m.setFault(Fault::OBSERVER_UNAVAILABLE);
            return false;
    }
}

void Motor_AngleFSM::clear(MotorManager& m)
{
    m.angle_state_ = AngleState::NONE;
    m.sensor_invalid_ticks_ = 0U;
}

void Motor_AngleFSM::useSensorSource(MotorManager& m)
{
    if (m.config_.position.rotor_sensor == nullptr)
    {
        faultUnavailable(m, AngleState::UNAVAILABLE, Fault::SENSOR_CONFIG_ERROR);
        return;
    }

    if (!m.ctx_.sensor_ready)
    {
        m.angle_state_ = AngleState::UNAVAILABLE;
        if (++m.sensor_invalid_ticks_ >=
            m.config_.position.health.invalid_confirm_ticks)
        {
            const Fault fault =
                (m.ctx_.sensor_health == SensorHealth::NOT_READY ||
                 m.ctx_.sensor_health == SensorHealth::NOT_CONFIGURED)
                ? Fault::SENSOR_LOSS
                : Fault::SENSOR_SIGNAL_ERROR;
            m.stopForFault(fault);
        }
        return;
    }

    m.angle_state_ = AngleState::SENSOR;
    m.sensor_invalid_ticks_ = 0U;
    m.ctx_.angle_elec = m.ctx_.angle_elec_sensor;
    m.ctx_.speed_rpm = m.ctx_.speed_rpm_sensor;
}

void Motor_AngleFSM::useSmoSource(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_SMO
    float angle_rad = m.ctx_.angle_elec_observer;
    float speed_rad_s = 0.0f;
    m.smo_.update(m.ctx_.v_alpha, m.ctx_.v_beta,
                  m.ctx_.i_alpha, m.ctx_.i_beta,
                  m.dt_, &angle_rad, &speed_rad_s);
    m.ctx_.angle_elec_observer = angle_rad;
    m.ctx_.smo_estimate = m.smo_.estimate();
    m.ctx_.speed_rpm_observer =
        speed_rad_s * 60.0f /
        (TWO_PI * m.config_.physical.pole_pairs);
    m.ctx_.angle_elec = m.ctx_.angle_elec_observer;
    m.ctx_.speed_rpm = m.ctx_.speed_rpm_observer;
m.event_.observer_converged = m.ctx_.smo_estimate.valid ? 1U : 0U;
    m.event_.speed_valid = m.ctx_.smo_estimate.valid ? 1U : 0U;
    m.angle_state_ = AngleState::SMO;

    /* === [P2] SMO 滑窗 Ke 同步到 ke_v_per_rad_s_identified ===
     * 加严条件:
     *   1. SMO valid (已收敛进入稳态源)
     *   2. valid_ticks >= 2 × convergence_ticks (避免刚收敛 1 tick 抢写)
     *   3. speed_abs > hfi_to_smo_rpm + hysteresis_rpm (高速稳态才写, 防低速段信号弱误估)
     * 仅 SMO 应用满足以上条件才持续微更新 identified;
     * 非 SMO 应用或低速段保留 measured 不变, 工业做法由 VCU 温度补偿。
     */
    if (m.ctx_.smo_estimate.valid &&
        static_cast<uint32_t>(m.ctx_.smo_estimate.valid_ticks) >=
            2UL * static_cast<uint32_t>(m.config_.observer.convergence_ticks))
    {
        const float speed_abs_rpm = fabsf(m.ctx_.speed_rpm);
        const float threshold =
            m.config_.observer.hfi_to_smo_rpm + m.config_.observer.switch_hysteresis_rpm;
        if (speed_abs_rpm > threshold)
        {
            const float ke_est = m.smo_.getEstimatedKe();
            if (ke_est > 0.0f)
            {
                m.config_.physical.ke_v_per_rad_s_identified = ke_est;
            }
        }
    }
#else
    faultUnavailable(m, AngleState::UNAVAILABLE, Fault::OBSERVER_UNAVAILABLE);
#endif
}

void Motor_AngleFSM::useHfiSource(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_HFI
    const float ct = cosf(m.ctx_.hfi_estimate.angle_rad);
    const float st = sinf(m.ctx_.hfi_estimate.angle_rad);
    const float i_d_hfi = m.ctx_.i_alpha_raw * ct + m.ctx_.i_beta_raw * st;
    const float i_q_hfi = -m.ctx_.i_alpha_raw * st + m.ctx_.i_beta_raw * ct;

    m.hfi_.setEnabled(true);
    m.hfi_.setInjectionVoltage(
        m.ctx_.v_bus * m.config_.observer.hfi_injection_voltage_ratio);
    m.hfi_.setPllGains(m.config_.observer.hfi_pll_kp,
                       m.config_.observer.hfi_pll_ki);
    m.hfi_.setPllLimits(m.config_.observer.hfi_pll_integral_limit_rad_s,
                        m.config_.observer.hfi_pll_speed_limit_rad_s);
    m.hfi_.setLockTicks(m.config_.observer.hfi_lock_ticks);
    m.hfi_.setMinSignalLevel(m.config_.observer.hfi_min_signal_level_a);
    m.hfi_.setSquareCurrentScale(m.config_.observer.hfi_square_current_scale);
    m.hfi_.setSquareLpfCutoffRatio(
        m.config_.observer.hfi_square_lpf_cutoff_ratio);
    m.hfi_.setSineLpfCutoffRatio(
        m.config_.observer.hfi_sine_lpf_cutoff_ratio);
    m.hfi_.update(m.ctx_.i_alpha_raw, m.ctx_.i_beta_raw, i_d_hfi, i_q_hfi,
                  m.dt_, &m.ctx_.hfi_estimate);
    m.ctx_.hfi_injection = m.hfi_.getInjectionCommand();
    m.ctx_.angle_elec_observer = m.ctx_.hfi_estimate.angle_rad;
    m.ctx_.speed_rpm_observer =
        m.ctx_.hfi_estimate.speed_rad_s * 60.0f /
        (TWO_PI * m.config_.physical.pole_pairs);
    m.event_.observer_converged = m.ctx_.hfi_estimate.valid ? 1U : 0U;
    m.event_.speed_valid = m.ctx_.hfi_estimate.valid ? 1U : 0U;
    m.ctx_.angle_elec = m.ctx_.angle_elec_observer;
    m.ctx_.speed_rpm = m.ctx_.speed_rpm_observer;
    m.angle_state_ = AngleState::HFI;
#else
    faultUnavailable(m, AngleState::UNAVAILABLE, Fault::OBSERVER_UNAVAILABLE);
#endif
}

void Motor_AngleFSM::faultUnavailable(MotorManager& m,
                                      AngleState state,
                                      Fault fault)
{
    m.angle_state_ = state;
    m.stopForFault(fault);
}

} // namespace Lib_Motor
