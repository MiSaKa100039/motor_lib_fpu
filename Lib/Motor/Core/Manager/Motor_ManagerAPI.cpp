#include "Motor_Manager.h"

#include <cmath>

namespace Lib_Motor
{

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && \
    (LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL || LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_IF_ANY)
namespace
{

uint32_t profileSecondsToTicks(float seconds, float dt)
{
    uint32_t duration_ticks = 1U;
    if (seconds > 0.0f && dt > 0.0f)
    {
        const float ticks = seconds / dt;
        duration_ticks =
            (ticks >= 4294967040.0f)
                ? 0xFFFFFFFFUL
                : static_cast<uint32_t>(ticks + 0.5f);
        if (duration_ticks == 0U)
        {
            duration_ticks = 1U;
        }
    }
    return duration_ticks;
}

#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
uint32_t calculateQ15SpeedRampStep(RuntimeSpeed target_rpm, uint32_t ramp_ticks)
{
    return AngleGenerator::calculateSpeedStepQ15(target_rpm, ramp_ticks);
}
#endif

void configureQ15DdaStep(FixedNumeric::q15_t start,
                         FixedNumeric::q15_t final,
                         uint32_t duration_ticks,
                         std::int32_t& step_q15,
                         std::uint32_t& remainder_q15,
                         std::int8_t& remainder_sign)
{
    if (duration_ticks == 0U)
    {
        duration_ticks = 1U;
    }

    const std::int32_t delta =
        static_cast<std::int32_t>(final) - static_cast<std::int32_t>(start);
    const std::uint32_t abs_delta =
        (delta < 0)
            ? static_cast<std::uint32_t>(-delta)
            : static_cast<std::uint32_t>(delta);

    remainder_sign = (delta > 0) ? 1 : ((delta < 0) ? -1 : 0);
    if (abs_delta == 0U)
    {
        step_q15 = 0;
        remainder_q15 = 0U;
        remainder_sign = 0;
        return;
    }

    if (duration_ticks > abs_delta)
    {
        step_q15 = 0;
        remainder_q15 = abs_delta;
        return;
    }

    const std::int32_t divisor = static_cast<std::int32_t>(duration_ticks);
    step_q15 = delta / divisor;
    remainder_q15 = abs_delta % duration_ticks;
}

#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
bool buildDebugVFProfileSnapshot(const MotorVFStartupProfile* profile,
                                 const MotorConfig& cfg,
                                 RuntimeCtx& ctx,
                                 float dt)
{
    if (profile == nullptr || profile->phases == nullptr)
    {
        ctx.debug_vf_profile_phase_count = 0U;
        return false;
    }

    const uint8_t phase_count = profile->phase_count;
    if (phase_count == 0U || phase_count > LIB_MOTOR_DEBUG_VF_PROFILE_MAX_PHASES)
    {
        ctx.debug_vf_profile_phase_count = 0U;
        return false;
    }

    RuntimeSpeed start_speed = 0;
    RuntimeDuty start_duty = 0;
    for (uint8_t i = 0U; i < phase_count; ++i)
    {
        const volatile MotorVFStartupPhase& phase = profile->phases[i];
        RuntimeVFStartupPhaseQ15& snapshot = ctx.debug_vf_profile_phases[i];
        snapshot.duration_ticks = profileSecondsToTicks(phase.duration_s, dt);
        snapshot.final_speed_rpm =
            runtimeSpeedFromUserPhysical(cfg, ctx, phase.final_speed_rpm);
        snapshot.final_duty = runtimeClampDuty(
            runtimeDutyFromNormalized(phase.final_duty),
            FixedNumeric::kQ15One);

        configureQ15DdaStep(start_speed,
                            snapshot.final_speed_rpm,
                            snapshot.duration_ticks,
                            snapshot.speed_step_q15,
                            snapshot.speed_remainder_q15,
                            snapshot.speed_remainder_sign);
        configureQ15DdaStep(start_duty,
                            snapshot.final_duty,
                            snapshot.duration_ticks,
                            snapshot.duty_step_q15,
                            snapshot.duty_remainder_q15,
                            snapshot.duty_remainder_sign);
        start_speed = snapshot.final_speed_rpm;
        start_duty = snapshot.final_duty;
    }

    ctx.debug_vf_profile_phase_count = phase_count;
    return true;
}
#endif

#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
bool buildIFProfileSnapshot(const MotorIFStartupProfile* profile,
                            const MotorConfig& cfg,
                            RuntimeCtx& ctx,
                            RuntimeIFStartupPhaseQ15* snapshots,
                            uint8_t max_phase_count,
                            uint8_t& out_phase_count,
                            float dt)
{
    out_phase_count = 0U;
    if (profile == nullptr || profile->phases == nullptr)
    {
        return false;
    }

    const uint8_t phase_count = profile->phase_count;
    if (phase_count == 0U || phase_count > max_phase_count)
    {
        return false;
    }

    RuntimeSpeed start_speed = 0;
    RuntimeCurrent start_id = 0;
    RuntimeCurrent start_iq = 0;
    for (uint8_t i = 0U; i < phase_count; ++i)
    {
        const volatile MotorIFStartupPhase& phase = profile->phases[i];
        RuntimeIFStartupPhaseQ15& snapshot = snapshots[i];
        snapshot.type = phase.type;
        snapshot.duration_ticks = profileSecondsToTicks(phase.duration_s, dt);
        snapshot.hold_ticks = (phase.type == MotorIFStartupPhaseType::ALIGNMENT)
            ? profileSecondsToTicks(phase.hold_time_s, dt)
            : 0U;
        snapshot.final_speed_rpm =
            runtimeSpeedFromUserPhysical(cfg, ctx, phase.final_speed_rpm);
        snapshot.final_id = runtimeCurrentFromPhysical(ctx, phase.final_id_a);
        snapshot.final_iq = runtimeCurrentFromPhysical(ctx, phase.final_iq_a);

        configureQ15DdaStep(start_speed,
                            snapshot.final_speed_rpm,
                            snapshot.duration_ticks,
                            snapshot.speed_step_q15,
                            snapshot.speed_remainder_q15,
                            snapshot.speed_remainder_sign);
        configureQ15DdaStep(start_id,
                            snapshot.final_id,
                            snapshot.duration_ticks,
                            snapshot.id_step_q15,
                            snapshot.id_remainder_q15,
                            snapshot.id_remainder_sign);
        configureQ15DdaStep(start_iq,
                            snapshot.final_iq,
                            snapshot.duration_ticks,
                            snapshot.iq_step_q15,
                            snapshot.iq_remainder_q15,
                            snapshot.iq_remainder_sign);
        start_speed = snapshot.final_speed_rpm;
        start_id = snapshot.final_id;
        start_iq = snapshot.final_iq;
    }

    out_phase_count = phase_count;
    return true;
}
#endif

} // namespace
#endif

/* setTargetSpeed -- 校验并写入用户机械转速目标, 不切换控制模式。
 *
 * 当前 IF -> SMO 无低速角度源:
 *   - STOP 下 [0, acquire_min_speed_rpm) 表示本次按 IF profile 最终速度启动并保持。
 *   - RUN 下同一范围无可靠 SMO 角度, 拒绝且保留原目标。
 *   - API 负方向在 HFI/顺逆风重启链补齐前保持禁用；实际物理方向由 command_direction 映射。
 */
Result MotorManager::setTargetSpeed(float rpm)
{
    if (!std::isfinite(rpm) || rpm < 0.0f ||
        !(config_.limit.max_speed_rpm > 0.0f) ||
        rpm > config_.limit.max_speed_rpm)
    {
        return Result::InvalidParam;
    }

#if LIB_MOTOR_ENABLE_IF_STARTUP && LIB_MOTOR_ENABLE_SMO
    const bool uses_if_smo_velocity =
        target_mode_ == Mode::VELOCITY_CONTROL &&
        active_policy_.startup_source == StartupSource::IF &&
        active_policy_.steady_source == SteadyAngleSource::SMO;
    if (uses_if_smo_velocity &&
        rpm < config_.observer.smo_validity.acquire_min_speed_rpm)
    {
        if (state_ != State::STOP)
        {
            return Result::InvalidParam;
        }

        if_profile_hold_target_ = true;
        ctx_.target_rpm = 0;
        ctx_.pending_reverse_rpm = 0;
        syncRuntimeTargets();
        return Result::Ok;
    }

    if (uses_if_smo_velocity)
    {
        if_profile_hold_target_ = false;
    }
#endif

    ctx_.target_rpm = runtimeSpeedFromUserPhysical(config_, ctx_, rpm);
    ctx_.pending_reverse_rpm = 0;
    syncRuntimeTargets();
    return Result::Ok;
}

/* setTargetTorque -- 直接覆盖 Q 轴电流目标 (A), 仅写目标, 不切模式 */
void MotorManager::setTargetTorque(float current_a)
{
    ctx_.target_iq = runtimeCurrentFromPhysical(ctx_, current_a);
    syncRuntimeTargets();
}

/* setTargetId -- 直接覆盖 D 轴电流目标 (A), 仅写目标, 不切模式 */
void MotorManager::setTargetId(float current_a)
{
    ctx_.target_id = runtimeCurrentFromPhysical(ctx_, current_a);
    syncRuntimeTargets();
}

/*
 * applySetpoint -- setpoint 目标字段落地
 *
 * 每控制周期由上层协调器调用一次, 上拍覆盖下拍, 无任务、无完成判据。
 * 行为:
 *   - 按 sp.mode 切换 target_mode_ (仅在合法且不同时切)
 *   - 按 mode 将 pos_ref/vel_ff/torque_ff 落地到 ctx_
 *   - 切换 stream_state_ -> ACTIVE
 *
 * 注: 本版仅写目标字段, 不消费 vel_ff/torque_ff/stiffness/damping;
 *     IMPEDANCE 控制环、位置 FF 在 Phase 2 实现 (RuntimeCtx 扩展)。
 */
Result MotorManager::applySetpoint(const MotionSetpoint& sp)
{
    /* TODO HostTest: writeSetpoint 各 mode 字段落地 (TORQUE→target_iq / VELOCITY→target_rpm /
     *                POSITION→target_pos_rad + vel_ff_rad_s / IMPEDANCE→imp_*) 未编单测,
     *                当前依赖 VOFA 真硬件验证 (新增字段需在 MonitorData 暴露才能在 VOFA 观察)。 */
    switch (sp.mode)
    {
        case Mode::TORQUE_CONTROL:
            ctx_.target_iq = runtimeCurrentFromPhysical(ctx_, sp.torque_ff);
            break;
        case Mode::VELOCITY_CONTROL:
        {
            const Result result = setTargetSpeed(sp.vel_ff);
            if (result != Result::Ok)
            {
                return result;
            }
            break;
        }
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
        case Mode::POSITION_CONTROL:
            ctx_.target_pos_rad = sp.pos_ref;
        #if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
            ctx_.vel_ff_rad_s   = sp.vel_ff;
        #endif
            break;
#endif
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
        case Mode::IMPEDANCE_CONTROL:
            /* Phase 2: 字段写入 ctx_ 即生效, 但控制环不消费
             * (mode==IMPEDANCE_CONTROL 当前因 supportsMode=false 在 selectMode 阶段被拒,
             *  本分支仅为字段一致性占位, 真实回放在 Phase 3 启用 BuildCfg IMPEDANCE 后流通) */
            ctx_.imp_pos_ref_rad    = sp.pos_ref;
            ctx_.imp_stiffness_n_m  = sp.stiffness;
            ctx_.imp_damping_n_m_s  = sp.damping;
            ctx_.imp_torque_bias_a  = sp.torque_bias;
            ctx_.imp_torque_limit_a = sp.torque_limit;
        #if LIB_MOTOR_ENABLE_TORQUE_FEEDFORWARD
            ctx_.torque_ff_a        = sp.torque_ff;
        #endif
            break;
#endif
        default:
            break;
    }
    syncRuntimeTargets();
    return Result::Ok;
}

/* writeSetpoint: 外部流式入口, 记录流状态后复用内部目标落地路径。 */
Result MotorManager::writeSetpoint(const MotionSetpoint& sp)
{
    const Result result = applySetpoint(sp);
    if (result != Result::Ok)
    {
        return result;
    }

#if LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE
    ctx_.last_setpoint_tick = fsmTimerTicks();
#endif

    if (sp.mode != Mode::NONE)
    {
        stream_state_ = StreamState::ACTIVE;
    }
    return Result::Ok;
}

/*
 * === 本地 FIFO/trigger 同步回放 ===
 * 只处理已解析 setpoint 的本地预装、触发和逐拍落地。
 * 协议帧、ACK、水位、SYNC 广播和轨迹规划仍在 lib 外层。
 */
Result MotorManager::appendSetpointSeg(const MotionSetpoint* seg, uint16_t count)
{
    return appendSetpointPlaybackSegment(seg, count);
}

Result MotorManager::triggerSegPlayback(PlaybackTrigger trig)
{
    return triggerSetpointPlayback(trig);
}

Result MotorManager::abortSegPlayback()
{
    return abortSetpointPlayback();
}

void MotorManager::resetSetpointPlayback()
{
#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    setpoint_playback_state_ = SetpointPlaybackState::IDLE;
    setpoint_fifo_head_ = 0U;
    setpoint_fifo_tail_ = 0U;
    setpoint_fifo_count_ = 0U;
#endif
}

void MotorManager::serviceSetpointPlayback()
{
#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    if (state_ != State::RUN)
    {
        return;
    }

    /*
     * RUNNING 状态每 tick 最多弹出一拍, 并在控制环前调用 applySetpoint()。
     */
    if (setpoint_playback_state_ != SetpointPlaybackState::RUNNING)
    {
        return;
    }

    if (setpoint_fifo_count_ == 0U)
    {
        setpoint_playback_state_ = SetpointPlaybackState::IDLE;
        return;
    }

    const MotionSetpoint sp = setpoint_fifo_[setpoint_fifo_head_];
    setpoint_fifo_head_ = static_cast<uint16_t>(
        (setpoint_fifo_head_ + 1U) % LIB_MOTOR_SETPOINT_FIFO_CAPACITY);
    --setpoint_fifo_count_;

    (void)applySetpoint(sp);
#if LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE
    ctx_.last_setpoint_tick = fsmTimerTicks();
#endif
    if (sp.mode != Mode::NONE)
    {
        stream_state_ = StreamState::ACTIVE;
    }

    if (setpoint_fifo_count_ == 0U)
    {
        setpoint_playback_state_ = SetpointPlaybackState::IDLE;
    }
#endif
}

Result MotorManager::appendSetpointPlaybackSegment(const MotionSetpoint* seg, uint16_t count)
{
#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    /* 运行中暂不允许追加, 避免控制 tick 与上层预装同时修改 FIFO。 */
    if (seg == nullptr || count == 0U)
    {
        return Result::InvalidParam;
    }

    if (setpoint_playback_state_ == SetpointPlaybackState::RUNNING ||
        setpoint_playback_state_ == SetpointPlaybackState::ARMED)
    {
        return Result::Busy;
    }

    const uint16_t free_count = static_cast<uint16_t>(
        LIB_MOTOR_SETPOINT_FIFO_CAPACITY - setpoint_fifo_count_);
    if (count > free_count)
    {
        return Result::Busy;
    }

    for (uint16_t i = 0U; i < count; ++i)
    {
        setpoint_fifo_[setpoint_fifo_tail_] = seg[i];
        setpoint_fifo_tail_ = static_cast<uint16_t>(
            (setpoint_fifo_tail_ + 1U) % LIB_MOTOR_SETPOINT_FIFO_CAPACITY);
    }
    setpoint_fifo_count_ = static_cast<uint16_t>(setpoint_fifo_count_ + count);
    setpoint_playback_state_ = SetpointPlaybackState::PRELOADED;
    return Result::Ok;
#else
    (void)seg;
    (void)count;
    return Result::NotSupported;
#endif
}

Result MotorManager::triggerSetpointPlayback(PlaybackTrigger trig)
{
    (void)trig;
#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    /* AWAIT_SYNC 只进入 ARMED; 外部 SYNC 到达后再次 IMMEDIATE 启播。 */
    if (setpoint_fifo_count_ == 0U)
    {
        return Result::InvalidState;
    }

    switch (trig)
    {
        case PlaybackTrigger::IMMEDIATE:
            setpoint_playback_state_ = SetpointPlaybackState::RUNNING;
            return Result::Ok;

        case PlaybackTrigger::AWAIT_SYNC:
            setpoint_playback_state_ = SetpointPlaybackState::ARMED;
            return Result::Ok;

        default:
            return Result::InvalidParam;
    }
#else
    return Result::NotSupported;
#endif
}

Result MotorManager::abortSetpointPlayback()
{
#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    /* abort 只停止本地消费; 总线 ACK/错误恢复由上层处理。 */
    setpoint_fifo_head_ = 0U;
    setpoint_fifo_tail_ = 0U;
    setpoint_fifo_count_ = 0U;
    setpoint_playback_state_ = SetpointPlaybackState::ABORTED;
    return Result::Ok;
#else
    return Result::NotSupported;
#endif
}

Result MotorManager::setRunPolicy(const MotorRunPolicy& policy)
{
    if (state_ != State::STOP)
    {
        return Result::InvalidState;
    }

    const Result validation = validateRunPolicy(policy);
    if (validation != Result::Ok)
    {
        return validation;
    }

    active_policy_ = policy;
    pending_policy_ = policy;
#if LIB_MOTOR_ENABLE_IF_STARTUP
    clearStartupRestartState();
#endif
    return Result::Ok;
}

Result MotorManager::getRunPolicy(MotorRunPolicy& policy) const
{
    policy = active_policy_;
    return Result::Ok;
}

#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
void MotorManager::setEnergyBudget(const MotorEnergyBudget& budget)
{
    energy_budget_ = budget;
    if (energy_budget_.max_regen_dc_current_a <= 0.0f &&
        budget.regen_dc_current_limit_a > 0.0f)
    {
        energy_budget_.max_regen_dc_current_a = budget.regen_dc_current_limit_a;
    }
    if (energy_budget_.max_dump_current_a <= 0.0f &&
        budget.dump_current_limit_a > 0.0f)
    {
        energy_budget_.max_dump_current_a = budget.dump_current_limit_a;
    }
    if (budget.max_brake_power_w > 0.0f)
    {
        if (energy_budget_.max_regen_power_w <= 0.0f)
        {
            energy_budget_.max_regen_power_w = budget.max_brake_power_w;
        }
        if (energy_budget_.max_dump_power_w <= 0.0f)
        {
            energy_budget_.max_dump_power_w = budget.max_brake_power_w;
        }
    }

    // 将上层 valid_for_us 转为控制环 tick 计数 (内部时钟节拍, 与上层 loop 漂移无关)
    // valid_ticks = us * 1e-6 * control_freq_hz = us * 1e-6 / dt
    uint32_t valid_ticks = 0U;
    if (budget.valid_for_us > 0U)
    {
        const float tf = static_cast<float>(budget.valid_for_us) * 1e-6f / dt_;
        if (tf >= 1.0f)
        {
            valid_ticks = static_cast<uint32_t>(tf);
        }
        else
        {
            valid_ticks = 1U;
        }
    }

    if (valid_ticks == 0U)
    {
        // 上层明确表达"即时失效"或未给时长, 直接当作已过期
        budget_active_ = false;
        budget_loss_blocked_ =
            (config_.brake.budget_loss_behavior == BudgetLossBehavior::BLOCK_ON_LOSS);
        if (budget_loss_blocked_)
        {
            event_.budget_expired = 1U;
        }
        budget_expire_ticks_ = 0U;
        return;
    }

    budget_active_ = true;
    budget_loss_blocked_ = false;
    budget_expire_ticks_ = fsmTimerTicks() + valid_ticks;
}
#endif

bool MotorManager::setPIDGains(PidGroup group, float kp, float ki, float kd)
{
    PIDParam param;

    switch (group)
    {
        case PidGroup::CurrentD:
            param = config_.control.current_d;
            break;
        case PidGroup::CurrentQ:
            param = config_.control.current_q;
            break;
        default:
            return false;
    }

    param.kp = kp;
    param.ki = ki;
    param.kd = kd;

    const bool use_critical =
        (config_.hal && config_.hal->enter_critical && config_.hal->exit_critical);
    if (use_critical) config_.hal->enter_critical();
    const bool updated = controller_.setCurrentPidParam(group, param);
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    if (updated)
    {
        configureFixedCurrentPidRuntime(false);
    }
#endif
    if (use_critical) config_.hal->exit_critical();

    return updated;
}

/* setVFDutyBias -- 设置 V/F 控制电压偏置 (占空比偏置, [0, 1]) */
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
void MotorManager::setVFDutyBias(float bias)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.vf_duty_bias = runtimeClampDuty(
        runtimeDutyFromNormalized(bias),
        FixedNumeric::kQ15One);
#else
    ctx_.vf_duty_bias = bias;
#endif
}
#endif

#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
/* setRawPWM -- 调试用: 直接设置三相 PWM 占空比, 绕过 FOC */
void MotorManager::setRawPWM(float u, float v, float w)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.debug_pwm_manual_a = FixedNumeric::fromNormalized(u);
    ctx_.debug_pwm_manual_b = FixedNumeric::fromNormalized(v);
    ctx_.debug_pwm_manual_c = FixedNumeric::fromNormalized(w);
#else
    ctx_.test_u = u;
    ctx_.test_v = v;
    ctx_.test_w = w;
#endif
}
#endif

#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
/* setDebugRampTime -- 设置 IF/VF 强拖速度斜坡时间 (s), <=0 使用默认值 */
void MotorManager::setDebugRampTime(float seconds)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    if (seconds > 0.0f && dt_ > 0.0f)
    {
        ctx_.debug_ramp_ticks = profileSecondsToTicks(seconds, dt_);
    }
    else
    {
        ctx_.debug_ramp_ticks = 0U;
    }
    ctx_.debug_ramp_speed_step_q15 = 0U;
#else
    ctx_.debug_ramp_time_s = (seconds > 0.0f) ? seconds : 0.0f;
#endif
}

void MotorManager::clearDebugStartupProfiles()
{
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.debug_vf_profile_phase_count = 0U;
#else
    ctx_.debug_vf_profile = nullptr;
#endif
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.debug_if_profile_phase_count = 0U;
#else
    ctx_.debug_if_profile = nullptr;
#endif
#endif
    resetDebugProfileState();
}
#endif

#if LIB_MOTOR_ENABLE_IF_STARTUP
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
bool MotorManager::prepareIFStartupProfileSnapshot(const MotorIFStartupProfile* profile)
{
    return buildIFProfileSnapshot(profile,
                                  config_,
                                  ctx_,
                                  ctx_.if_profile_phases,
                                  LIB_MOTOR_IF_PROFILE_MAX_PHASES,
                                  ctx_.if_profile_phase_count,
                                  dt_);
}

void MotorManager::cacheIFStartupRestartInterval()
{
    startup_restart_interval_ticks_ = profileSecondsToTicks(
        active_policy_.startup_restart_interval_s, dt_);
}
#endif

void MotorManager::prepareIFStartupRuntimeForStart()
{
    resetIFStartupProfileState();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    cacheIFStartupRestartInterval();
#endif

    if (active_policy_.startup_source != StartupSource::IF)
    {
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        ctx_.if_profile_phase_count = 0U;
#endif
        return;
    }

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    prepareIFStartupProfileSnapshot(config_.observer.if_startup_profile);
    if (if_profile_hold_target_ && ctx_.if_profile_phase_count > 0U)
    {
        ctx_.target_rpm =
            ctx_.if_profile_phases[ctx_.if_profile_phase_count - 1U].final_speed_rpm;
        ctx_.pending_reverse_rpm = 0;
    }
#else
    const MotorIFStartupProfile* profile = config_.observer.if_startup_profile;
    if (if_profile_hold_target_ && profile != nullptr &&
        profile->phases != nullptr && profile->phase_count > 0U)
    {
        const volatile MotorIFStartupPhase& final_phase =
            profile->phases[profile->phase_count - 1U];
        ctx_.target_rpm = runtimeSpeedFromUserPhysical(
            config_, ctx_, final_phase.final_speed_rpm);
        ctx_.pending_reverse_rpm = 0.0f;
    }
#endif
    syncRuntimeTargets();
}
#endif

#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY && LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
void MotorManager::prepareDebugOpenLoopAngleRampForStart()
{
    ctx_.debug_ramp_speed_step_q15 = 0U;

    const bool debug_open_loop =
        false
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
        || target_mode_ == Mode::DEBUG_VF_DRAG
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
        || target_mode_ == Mode::DEBUG_IF_DRAG
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
        || target_mode_ == Mode::DEBUG_IF_SMO_OBSERVER
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
        || target_mode_ == Mode::DEBUG_IF_HFI_OBSERVER
#endif
        ;
    if (!debug_open_loop)
    {
        return;
    }

#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    if (target_mode_ == Mode::DEBUG_VF_DRAG &&
        ctx_.debug_vf_profile_phase_count > 0U)
    {
        return;
    }
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    if ((false
#if LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
         || target_mode_ == Mode::DEBUG_IF_DRAG
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
         || target_mode_ == Mode::DEBUG_IF_SMO_OBSERVER
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
         || target_mode_ == Mode::DEBUG_IF_HFI_OBSERVER
#endif
        ) &&
        ctx_.debug_if_profile_phase_count > 0U)
    {
        return;
    }
#endif

    ctx_.debug_ramp_speed_step_q15 =
        calculateQ15SpeedRampStep(ctx_.target_rpm, ctx_.debug_ramp_ticks);
}
#endif

#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
bool MotorManager::setDebugVFStartupProfile(const MotorVFStartupProfile* profile)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    if (!buildDebugVFProfileSnapshot(profile, config_, ctx_, dt_))
    {
        resetDebugProfileState();
        return false;
    }
#else
    ctx_.debug_vf_profile = profile;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.debug_if_profile_phase_count = 0U;
#else
    ctx_.debug_if_profile = nullptr;
#endif
#endif
    resetDebugProfileState();
    return true;
}
#endif

#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
bool MotorManager::setDebugIFStartupProfile(const MotorIFStartupProfile* profile)
{
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.debug_vf_profile_phase_count = 0U;
#else
    ctx_.debug_vf_profile = nullptr;
#endif
#endif
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    if (!buildIFProfileSnapshot(profile,
                                config_,
                                ctx_,
                                ctx_.debug_if_profile_phases,
                                LIB_MOTOR_DEBUG_IF_PROFILE_MAX_PHASES,
                                ctx_.debug_if_profile_phase_count,
                                dt_))
    {
        resetDebugProfileState();
        return false;
    }
#else
    ctx_.debug_if_profile = profile;
#endif
    resetDebugProfileState();
    controller_.reset();

    if (state_ != State::RUN)
    {
        if_angle_gen_.reset();
        ctx_.angle_elec = runtimeAngleFromRadians(0.0f);
        ctx_.angle_elec_command = runtimeAngleFromRadians(0.0f);
        ctx_.speed_rpm = runtimeSpeedFromPhysical(ctx_, 0.0f);
    }

#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
    if (target_mode_ == Mode::DEBUG_IF_SMO_OBSERVER)
    {
        smo_.reset();
        resetSmoAngleDirectionLatch();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        ctx_.smo_estimate = ObserverEstimateQ15();
#else
        ctx_.smo_estimate = ObserverEstimate();
#endif
        ctx_.angle_elec_observer = ctx_.angle_elec_command;
        ctx_.speed_rpm_observer = runtimeSpeedFromPhysical(ctx_, 0.0f);
        event_.observer_converged = 0U;
        event_.speed_valid = 0U;
    }
#endif

#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
    if (target_mode_ == Mode::DEBUG_IF_HFI_OBSERVER)
    {
        hfi_.reset();
        ctx_.hfi_estimate = ObserverEstimate();
        ctx_.hfi_injection = HfiInjectionCommand();
        ctx_.angle_elec_observer = ctx_.angle_elec_command;
        ctx_.speed_rpm_observer = runtimeSpeedFromPhysical(ctx_, 0.0f);
        event_.observer_converged = 0U;
        event_.speed_valid = 0U;
    }
#endif

    return true;
}
#endif
} // namespace Lib_Motor
