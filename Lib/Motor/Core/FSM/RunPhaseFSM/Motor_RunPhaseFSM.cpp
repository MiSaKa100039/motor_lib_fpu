#include "Motor_RunPhaseFSM.h"

#include "../../Manager/Motor_Manager.h"
#include "../../RunPlan/Motor_RunPlan.h"
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../../Numeric/Motor_FixedNumeric.h"
#endif

#include <cmath>

#if LIB_MOTOR_ENABLE_SMO && !LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;

float wrapSignedAngleLocal(float angle)
{
    while (angle > kPi) angle -= kTwoPi;
    while (angle < -kPi) angle += kTwoPi;
    return angle;
}
} // namespace
#endif

namespace Lib_Motor
{

RunPhase Motor_RunPhaseFSM::selectInitialPhase(MotorManager& m, bool direct_run)
{
    const RunPhase initial = MotorRunPlan::selectInitialPhase(m.active_policy_, direct_run);
#if LIB_MOTOR_ENABLE_FLYING_START
    if (!direct_run &&
        initial == RunPhase::FORCE_DRAG &&
        m.config_.observer.flying_start_mode == FlyingStartMode::PHASE_VOLTAGE &&
        m.config_.sensor.has_phase_voltage &&
        fabsf(runtimeSpeedToPhysical(m.ctx_, m.ctx_.speed_rpm)) >
            m.config_.observer.flying_start_speed_threshold_rpm)
    {
        m.flying_start_routine_.reset();
        return RunPhase::FLYING_START;
    }
#endif
    return initial;
}

void Motor_RunPhaseFSM::step(MotorManager& m)
{
    switch (m.runPhase())
    {
        case RunPhase::RUN_DIRECT:
        case RunPhase::SENSOR_ONLY:
            break;

        case RunPhase::FUSION:
            handleFusion(m);
            break;

        case RunPhase::ALIGNMENT:
            handleAlignment(m);
            break;

#if LIB_MOTOR_ENABLE_FLYING_START
        case RunPhase::FLYING_START:
            m.flying_start_routine_.step(m);
            break;
#endif

        case RunPhase::FORCE_DRAG:
            handleForceDrag(m);
            break;

        case RunPhase::SMO_ONLY:
            handleSmo(m);
            break;

        case RunPhase::HFI_ONLY:
            handleHfi(m);
            break;

        default:
            break;
    }
}

void Motor_RunPhaseFSM::handleAlignment(MotorManager& m)
{
    /* IF 对齐已经并入 FORCE_DRAG profile；该阶段仅保留给辨识流程。 */
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    if (m.targetMode() == Mode::CALIB_RL_IDENTIFY)
    {
        return;
    }
#endif
    m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
}

void Motor_RunPhaseFSM::handleForceDrag(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_IF_STARTUP
    const bool profile_complete = m.ifStartupProfileComplete();
#if LIB_MOTOR_ENABLE_SMO
    const bool allow_transition =
        !m.if_profile_hold_target_ || profile_complete;
    if (tryBeginSmoHandover(m, allow_transition))
    {
        return;
    }
#endif

    if (profile_complete)
    {
#if LIB_MOTOR_ENABLE_SMO
        /* profile-hold 在最终速度等待已开始的连续确认；条件一旦失效，
         * tryBeginSmoHandover 会清零计数，本拍按原启动失败策略处理。
         */
        if (m.if_profile_hold_target_ && m.smo_handover_confirm_ticks_ > 0U)
        {
            return;
        }
#endif
        m.handleIFStartupFailure();
    }
#else
    m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
#endif
}

bool Motor_RunPhaseFSM::tryBeginSmoHandover(MotorManager& m,
                                            bool allow_transition)
{
#if LIB_MOTOR_ENABLE_SMO
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const RuntimeSpeed source_speed = m.ctx_.speed_rpm;
    const RuntimeSpeed observer_speed = m.ctx_.speed_rpm_observer;
    const RuntimeAngle observer_angle = m.correctSmoAngleForControl(
        m.ctx_.smo_estimate.angle_phase, observer_speed);
    const int32_t angle_error_signed =
        static_cast<int32_t>(observer_angle - m.ctx_.angle_elec);
    const uint32_t angle_error_abs = (angle_error_signed < 0)
        ? (0U - static_cast<uint32_t>(angle_error_signed))
        : static_cast<uint32_t>(angle_error_signed);
    const int32_t speed_error =
        static_cast<int32_t>(observer_speed) - static_cast<int32_t>(source_speed);
    const uint32_t speed_error_abs = (speed_error < 0)
        ? static_cast<uint32_t>(-speed_error)
        : static_cast<uint32_t>(speed_error);
    const bool same_direction =
        (source_speed > 0 && observer_speed > 0) ||
        (source_speed < 0 && observer_speed < 0);
    m.smo_handover_angle_error_phase_ = angle_error_abs;
    const bool ready =
        m.event_.observer_converged != 0U &&
        FixedNumeric::absoluteQ15(observer_speed) >=
            m.ctx_.smo_handover_min_speed_q15 &&
        same_direction &&
        speed_error_abs <= static_cast<uint32_t>(
            m.ctx_.smo_handover_max_speed_error_q15) &&
        angle_error_abs <= m.ctx_.smo_handover_max_angle_error_phase;
#else
    const float source_speed = runtimeSpeedToPhysical(m.ctx_, m.ctx_.speed_rpm);
    const float observer_speed =
        runtimeSpeedToPhysical(m.ctx_, m.ctx_.speed_rpm_observer);
    const RuntimeAngle observer_angle = m.correctSmoAngleForControl(
        m.ctx_.smo_estimate.angle_rad, m.ctx_.speed_rpm_observer);
    const float angle_error =
        wrapSignedAngleLocal(observer_angle - m.ctx_.angle_elec);
    const bool same_direction =
        (source_speed > 0.0f && observer_speed > 0.0f) ||
        (source_speed < 0.0f && observer_speed < 0.0f);
    m.smo_handover_angle_error_rad_ = fabsf(angle_error);
    const bool ready =
        m.event_.observer_converged != 0U &&
        fabsf(observer_speed) >= m.config_.observer.smo_handover.min_speed_rpm &&
        same_direction &&
        fabsf(observer_speed - source_speed) <=
            m.config_.observer.smo_handover.max_speed_error_rpm &&
        m.smo_handover_angle_error_rad_ <=
            m.config_.observer.smo_handover.max_angle_error_rad;
#endif

    if (!ready)
    {
        m.smo_handover_confirm_ticks_ = 0U;
        return false;
    }
    if (m.smo_handover_confirm_ticks_ < 0xFFFFU)
    {
        ++m.smo_handover_confirm_ticks_;
    }
    if (m.smo_handover_confirm_ticks_ <
        m.config_.observer.smo_handover.confirm_ticks)
    {
        return false;
    }
    if (!allow_transition)
    {
        return false;
    }

#if LIB_MOTOR_ENABLE_IF_STARTUP
    m.clearStartupRestartState();
#endif
    m.smo_handover_confirm_ticks_ = 0U;
    m.smo_fusion_ticks_ = 0U;
    m.smo_fusion_start_speed_ = m.ctx_.speed_rpm;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const RuntimeAngle corrected_angle = m.correctSmoAngleForControl(
        m.ctx_.smo_estimate.angle_phase, m.ctx_.speed_rpm_observer);
    const int32_t angle_offset_phase =
        static_cast<int32_t>(m.ctx_.angle_elec - corrected_angle);
    const uint32_t angle_offset_magnitude = (angle_offset_phase < 0)
        ? (0U - static_cast<uint32_t>(angle_offset_phase))
        : static_cast<uint32_t>(angle_offset_phase);
    const int32_t angle_offset_q15 =
        static_cast<int32_t>(angle_offset_magnitude >> 16U);
    m.smo_fusion_angle_offset_q15_ =
        (angle_offset_phase < 0) ? -angle_offset_q15 : angle_offset_q15;
    m.smo_fusion_progress_q15_ = 0U;
    m.smo_fusion_remainder_accum_ = 0U;
    m.ctx_.speed_ref_limited = m.ctx_.speed_rpm;
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    m.ctx_.speed_slew_fraction_q16 = 0U;
#endif
    m.controller_.preloadFixedSpeedOutputQ15(0, m.ctx_.iq_ref_limited);
#else
    const RuntimeAngle corrected_angle = m.correctSmoAngleForControl(
        m.ctx_.smo_estimate.angle_rad, m.ctx_.speed_rpm_observer);
    m.smo_fusion_angle_offset_rad_ =
        wrapSignedAngleLocal(m.ctx_.angle_elec - corrected_angle);
    m.ctx_.speed_ref_limited = m.ctx_.speed_rpm;
    m.controller_.pid_speed.preloadOutput(0.0f, m.ctx_.iq_ref_limited);
#endif
    m.setRunPhase(RunPhase::FUSION);
    m.ctx_.fsm_timer_ticks = 0U;
    return true;
#else
    (void)m;
    return false;
#endif
}

void Motor_RunPhaseFSM::handleFusion(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_SMO
    if (!m.event_.observer_converged)
    {
        m.stopForFault(Fault::OBSERVER_LOSS);
        return;
    }
    if (m.smo_fusion_ticks_ < m.smo_fusion_total_ticks_)
    {
        ++m.smo_fusion_ticks_;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        uint32_t increment = m.smo_fusion_step_q15_;
        if (m.smo_fusion_remainder_q15_ > 0U)
        {
            const uint32_t remainder_space =
                m.smo_fusion_total_ticks_ - m.smo_fusion_remainder_accum_;
            if (m.smo_fusion_remainder_q15_ >= remainder_space)
            {
                m.smo_fusion_remainder_accum_ =
                    m.smo_fusion_remainder_q15_ - remainder_space;
                ++increment;
            }
            else
            {
                m.smo_fusion_remainder_accum_ += m.smo_fusion_remainder_q15_;
            }
        }
        const uint32_t progress =
            static_cast<uint32_t>(m.smo_fusion_progress_q15_) + increment;
        m.smo_fusion_progress_q15_ = static_cast<uint16_t>(
            (progress < 32768U) ? progress : 32768U);
#endif
    }
    if (m.smo_fusion_ticks_ >= m.smo_fusion_total_ticks_)
    {
        m.setRunPhase(RunPhase::SMO_ONLY);
        m.ctx_.fsm_timer_ticks = 0U;
    }
#else
    m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
#endif
}

void Motor_RunPhaseFSM::handleSmo(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_SMO
    if (m.event_.observer_converged)
    {
        return;
    }

    /* SMO 内部释放门槛已经过滤瞬时无效；锁存状态真正释放后再决定降级或停机。
     * enable_auto_swap=true 时，仅在低于运行期回切门槛后尝试切回 HFI/IF。
     * enable_auto_swap=false 时直接报告 OBSERVER_LOSS，避免静默使用失效角度。
     */
    const auto& obs = m.config_.observer;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const RuntimeSpeed speed_abs = FixedNumeric::absoluteQ15(m.ctx_.speed_rpm);
#else
    const float speed_abs = fabsf(runtimeSpeedToPhysical(m.ctx_, m.ctx_.speed_rpm));
#endif
    if (obs.enable_auto_swap)
    {
        // 减速切回: 速度低于 smo_to_hfi_rpm 时主动降级
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        if (speed_abs < m.ctx_.smo_auto_swap_fall_speed_q15)
#else
        const float fall_threshold = obs.smo_to_hfi_rpm - obs.switch_hysteresis_rpm;
        if (speed_abs < fall_threshold)
#endif
        {
#if LIB_MOTOR_ENABLE_HFI
            // HFI 预热已完成则切到 HFI_ONLY; 否则回 IF 强拖
            m.setRunPhase(RunPhase::HFI_ONLY);
            m.ctx_.fsm_timer_ticks = 0U;
            return;
#elif LIB_MOTOR_ENABLE_IF_STARTUP
            m.resetIFStartupProfileState();
            m.resetIFStartupObserverState();
            m.if_angle_gen_.reset();
            m.setRunPhase(RunPhase::FORCE_DRAG);
            m.ctx_.fsm_timer_ticks = 0U;
            return;
#endif
        }

    }

    m.stopForFault(Fault::OBSERVER_LOSS);
#else
    m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
#endif
}

void Motor_RunPhaseFSM::handleHfi(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_HFI
    /* HFI 启动与 IF 启动共用 SMO 有效性和交接门槛；
     * HFI 失效且无法进入 SMO 时，在宽限窗口结束后停机。
     */
    const auto& obs = m.config_.observer;

    if (m.active_policy_.steady_source == SteadyAngleSource::SMO &&
        tryBeginSmoHandover(m, true))
    {
        return;
    }

    // HFI 失效判定: observer_converged=false 持续超过 grace 窗口
    if (!m.event_.observer_converged)
    {
        const uint32_t grace_ticks =
            static_cast<uint32_t>(obs.switch_grace_time_s *
                                  m.config_.control.control_freq_hz);
        if (grace_ticks > 0U && m.ctx_.fsm_timer_ticks < grace_ticks)
        {
            return; // grace 内仍可恢复
        }
        m.stopForFault(Fault::OBSERVER_LOSS);
    }
#else
    (void)m;
    m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
#endif
}

} // namespace Lib_Motor
