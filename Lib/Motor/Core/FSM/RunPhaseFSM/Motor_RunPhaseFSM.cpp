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
        case RunPhase::FUSION:
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
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#if LIB_MOTOR_ENABLE_SMO
    const FixedNumeric::q15_t speed_abs =
        FixedNumeric::absoluteQ15(m.ctx_.smo_estimate.speed_rpm_q15);
    const RuntimeAngle smo_handover_angle =
        m.correctSmoAngleForControl(m.ctx_.smo_estimate.angle_phase,
                                    m.ctx_.smo_estimate.speed_rpm_q15);
    const int32_t phase_error_signed =
        static_cast<int32_t>(smo_handover_angle - m.ctx_.angle_elec);
    const uint32_t phase_error_abs = (phase_error_signed < 0)
        ? static_cast<uint32_t>(-static_cast<int64_t>(phase_error_signed))
        : static_cast<uint32_t>(phase_error_signed);
    const uint32_t max_phase_error =
        FixedNumeric::phaseFromRadians(m.config_.observer.max_handover_error_rad);

    if (speed_abs > m.ctx_.if_switch_up_speed_q15 &&
        m.event_.observer_converged &&
        phase_error_abs <= max_phase_error)
    {
        m.clearStartupRestartState();
        m.setRunPhase(RunPhase::SMO_ONLY);
        m.ctx_.fsm_timer_ticks = 0U;
        return;
    }
#endif

    if (m.ifStartupProfileComplete())
    {
        m.handleIFStartupFailure();
    }
#else
    const auto& obs_cfg = m.config_.observer;
    const float speed_abs =
        fabsf(runtimeSpeedToPhysical(m.ctx_, m.ctx_.speed_rpm_observer));
    // [P4] 启动期 IF→SMO 单向加速切换用 hfi_to_smo_startup_rpm (旧 switch_speed_rpm)
    const float switch_up = obs_cfg.hfi_to_smo_startup_rpm + obs_cfg.hfi_to_smo_startup_hysteresis_rpm;

#if LIB_MOTOR_ENABLE_SMO
    const RuntimeAngle smo_handover_angle =
        m.correctSmoAngleForControl(m.ctx_.smo_estimate.angle_rad,
                                    m.ctx_.speed_rpm_observer);
    const float handover_error =
        fabsf(wrapSignedAngleLocal(smo_handover_angle - m.ctx_.angle_elec));
    if (speed_abs > switch_up &&
        m.event_.observer_converged &&
        handover_error <= obs_cfg.max_handover_error_rad)
    {
        m.clearStartupRestartState();
        m.setRunPhase(RunPhase::SMO_ONLY);
        m.ctx_.fsm_timer_ticks = 0U;
        return;
    }
#else
    if (speed_abs > switch_up)
    {
        m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
        return;
    }
#endif

    if (m.ifStartupProfileComplete())
    {
        m.handleIFStartupFailure();
    }
#endif
#else
    m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
#endif
}

void Motor_RunPhaseFSM::handleSmo(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_SMO
    if (m.event_.observer_converged)
    {
        // 加速到 hfi_to_smo_rpm 之上且 SMO 稳时关闭 HFI 预热 (本轮仅语义提示)
        return;
    }

    /* 改: SMO 失效时不再直接锁故障, 优先尝试切回 IF/HFI;
     * - observer_swap.enable_auto_swap=true: 减速到 smo_to_hfi_rpm 以下即主动切回;
     * - 切回失败超时 (switch_grace_time_s) 才发 OBSERVER_LOSS。
     * - enable_auto_swap=false: 维持旧的"立即 OBSERVER_LOSS"行为, 向后兼容。
     */
    const auto& obs = m.config_.observer;
    const float speed_abs = fabsf(runtimeSpeedToPhysical(m.ctx_, m.ctx_.speed_rpm));
    const uint32_t grace_ticks =
        static_cast<uint32_t>(obs.switch_grace_time_s *
                              m.config_.control.control_freq_hz);
    if (grace_ticks > 0U && m.ctx_.fsm_timer_ticks < grace_ticks)
    {
        return;
    }

    if (obs.enable_auto_swap)
    {
        // 减速切回: 速度低于 smo_to_hfi_rpm 时主动降级
        const float fall_threshold = obs.smo_to_hfi_rpm - obs.switch_hysteresis_rpm;
        if (speed_abs < fall_threshold)
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

        // SMO 突然失效但速度仍在工作区: 进入 grace 窗口
        const uint32_t grace_ticks =
            static_cast<uint32_t>(obs.switch_grace_time_s *
                                  m.config_.control.control_freq_hz);
        if (grace_ticks > 0U && m.ctx_.fsm_timer_ticks < grace_ticks)
        {
            return; // grace 期内等待恢复
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
    /* 加速段: speed > hfi_to_smo_rpm 时主动切到 SMO (不等到 HFI 信号弱);
     * 减速段: HFI 已稳定时不切回 SMO; HFI 失效且无 SMO 兜底则 Fault。
     */
    const auto& obs = m.config_.observer;
    const float speed_abs = fabsf(runtimeSpeedToPhysical(m.ctx_, m.ctx_.speed_rpm));

    if (obs.enable_auto_swap && speed_abs > obs.hfi_to_smo_rpm + obs.switch_hysteresis_rpm)
    {
#if LIB_MOTOR_ENABLE_SMO
        m.smo_.seedAngle(m.ctx_.angle_elec);
        m.setRunPhase(RunPhase::SMO_ONLY);
        m.ctx_.fsm_timer_ticks = 0U;
        return;
#endif
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
