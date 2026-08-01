#include "Motor_RunPhaseFSM.h"

#include "../../Manager/Motor_Manager.h"
#include "../../RunPlan/Motor_RunPlan.h"

#include <cmath>

#if LIB_MOTOR_ENABLE_SMO
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
    return MotorRunPlan::selectInitialPhase(m.active_policy_, direct_run);
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
    /* [P2] start() 时自动检测顺逆风: 若 cfg.observer.flying_start_mode==PHASE_VOLTAGE
     * 且启动初始 |ω| 已超过顺逆风启动阈值 → 切到 FLYING_START 走相电压 PLL 重构, 而非 ALIGNMENT 电流拉转子。
     * 否则 (cfg未打开 / ω 低于阈值 / 无相电压采样) → 走标准 ALIGNMENT 流程。
     */
#if LIB_MOTOR_ENABLE_FLYING_START
    if (m.config_.observer.flying_start_mode == FlyingStartMode::PHASE_VOLTAGE &&
        m.config_.sensor.has_phase_voltage &&
        m.ctx_.fsm_timer_ticks == 1U)   // 仅第一 tick 检测
    {
        const float speed_abs = fabsf(m.getSpeed());
        // [P3.B] 顺逆风启动阈值 cfg.observer.flying_start_speed_threshold_rpm, 默认 100 RPM
        if (speed_abs > m.config_.observer.flying_start_speed_threshold_rpm)
        {
            m.flying_start_routine_.reset();
            m.flying_start_routine_.step(m);    // 立即进入 WAIT_BEMF 一次
            m.setRunPhase(RunPhase::FLYING_START);
            return;
        }
    }
#endif

#if LIB_MOTOR_ENABLE_IF_STARTUP
    const uint32_t align_ticks =
        static_cast<uint32_t>(m.config_.observer.align_time_s *
                              m.config_.control.control_freq_hz);

    if (m.ctx_.fsm_timer_ticks <= align_ticks)
    {
        return;
    }

    m.if_angle_gen_.reset();
    m.setRunPhase(RunPhase::FORCE_DRAG);
    m.ctx_.fsm_timer_ticks = 0U;
#else
    m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
#endif
}

void Motor_RunPhaseFSM::handleForceDrag(MotorManager& m)
{
#if LIB_MOTOR_ENABLE_IF_STARTUP
    const auto& obs_cfg = m.config_.observer;
    const float speed_abs = fabsf(m.getSpeed());
    // [P4] 启动期 IF→SMO 单向加速切换用 hfi_to_smo_startup_rpm (旧 switch_speed_rpm)
    const float switch_up = obs_cfg.hfi_to_smo_startup_rpm + obs_cfg.hfi_to_smo_startup_hysteresis_rpm;

#if LIB_MOTOR_ENABLE_SMO
    const float handover_error =
        fabsf(wrapSignedAngleLocal(m.ctx_.smo_estimate.angle_rad -
                                   m.ctx_.angle_elec));
    if (speed_abs > switch_up &&
        m.event_.observer_converged &&
        handover_error <= obs_cfg.max_handover_error_rad)
    {
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

    const uint32_t timeout_ticks =
        static_cast<uint32_t>(obs_cfg.force_drag_timeout_s *
                              m.config_.control.control_freq_hz);
    if (timeout_ticks > 0U && m.ctx_.fsm_timer_ticks > timeout_ticks)
    {
        m.stopForFault(Fault::OBSERVER_LOSS);
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
        // 加速到 hfi_to_smo_rpm 之上且 SMO 稳时关闭 HFI 预热 (本轮仅语义提示)
        return;
    }

    /* 改: SMO 失效时不再直接锁故障, 优先尝试切回 IF/HFI;
     * - observer_swap.enable_auto_swap=true: 减速到 smo_to_hfi_rpm 以下即主动切回;
     * - 切回失败超时 (switch_grace_time_s) 才发 OBSERVER_LOSS。
     * - enable_auto_swap=false: 维持旧的"立即 OBSERVER_LOSS"行为, 向后兼容。
     */
    const auto& obs = m.config_.observer;
    const float speed_abs = fabsf(m.getSpeed());
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
    const float speed_abs = fabsf(m.getSpeed());

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
