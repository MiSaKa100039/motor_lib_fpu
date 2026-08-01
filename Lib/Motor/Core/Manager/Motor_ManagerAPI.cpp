#include "Motor_Manager.h"

namespace Lib_Motor
{

/* setTargetSpeed -- 直接覆盖速度目标值 (RPM), 仅写目标, 不切模式
 *
 * 跨零软减速 (本轮新增):
 *   若新 rpm 与当前 target_rpm 反号 且 当前实测速度 |ω| > reverse_zero_band_rpm,
 *   表示用户在大速度时直接给反向指令; 为避免"目标跨零直跳"造成的电流冲击
 *   与 SMO 跨零失效, 先写 0 让速度环减速到接近 0, 下一拍再写实际 rpm。
 *   通过 ctx_.pending_reverse_rpm 暂存待执行的目标。
 */
void MotorManager::setTargetSpeed(float rpm)
{
    const float current_target = ctx_.target_rpm;
    const bool sign_flip = (current_target * rpm < 0.0f);  // 反号
    const bool fast_running = fabsf(ctx_.speed_rpm) > config_.motion.reverse_zero_band_rpm;

    if (sign_flip && fast_running)
    {
        // 暂存反向目标, 当前 target 写 0 (速度环会把速度带到 0)
        ctx_.pending_reverse_rpm = rpm;
        ctx_.target_rpm = 0.0f;
        syncFastRuntimeTargets();
        return;
    }

    ctx_.target_rpm = rpm;
    syncFastRuntimeTargets();
}

/* setTargetTorque -- 直接覆盖 Q 轴电流目标 (A), 仅写目标, 不切模式 */
void MotorManager::setTargetTorque(float current_a)
{
    ctx_.target_iq = current_a;
    syncFastRuntimeTargets();
}

/* setTargetId -- 直接覆盖 D 轴电流目标 (A), 仅写目标, 不切模式 */
void MotorManager::setTargetId(float current_a)
{
    ctx_.target_id = current_a;
    syncFastRuntimeTargets();
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
void MotorManager::applySetpoint(const MotionSetpoint& sp)
{
    /* TODO HostTest: writeSetpoint 各 mode 字段落地 (TORQUE→target_iq / VELOCITY→target_rpm /
     *                POSITION→target_pos_rad + vel_ff_rad_s / IMPEDANCE→imp_*) 未编单测,
     *                当前依赖 VOFA 真硬件验证 (新增字段需在 MonitorData 暴露才能在 VOFA 观察)。 */
    switch (sp.mode)
    {
        case Mode::TORQUE_CONTROL:
            ctx_.target_iq = sp.torque_ff;
            break;
        case Mode::VELOCITY_CONTROL:
            ctx_.target_rpm = sp.vel_ff;
            break;
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
    syncFastRuntimeTargets();
}

/* writeSetpoint: 外部流式入口, 记录流状态后复用内部目标落地路径。 */
void MotorManager::writeSetpoint(const MotionSetpoint& sp)
{
    applySetpoint(sp);

#if LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE
    ctx_.last_setpoint_tick = fsmTimerTicks();
#endif

    if (sp.mode != Mode::NONE)
    {
        stream_state_ = StreamState::ACTIVE;
    }
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

    applySetpoint(sp);
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
    PidController* pid = nullptr;

    switch (group)
    {
        case PidGroup::CurrentD:
            param = config_.control.current_d;
            pid = &controller_.pid_d;
            break;
        case PidGroup::CurrentQ:
            param = config_.control.current_q;
            pid = &controller_.pid_q;
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
    pid->init(param);
    if (use_critical) config_.hal->exit_critical();

    return true;
}

/* setVFDutyBias -- 设置 V/F 控制电压偏置 (占空比偏置, [0, 1]) */
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
void MotorManager::setVFDutyBias(float bias)
{
    ctx_.vf_duty_bias = bias;
}

/* setRawPWM -- 调试用: 直接设置三相 PWM 占空比, 绕过 FOC */
void MotorManager::setRawPWM(float u, float v, float w)
{
    ctx_.test_u = u;
    ctx_.test_v = v;
    ctx_.test_w = w;
}

/* setDebugRampTime -- 设置 IF/VF 强拖速度斜坡时间 (s), <=0 使用默认值 */
void MotorManager::setDebugRampTime(float seconds)
{
    ctx_.debug_ramp_time_s = (seconds > 0.0f) ? seconds : 0.0f;
}

void MotorManager::clearDebugStartupProfiles()
{
    ctx_.debug_vf_profile = nullptr;
#if LIB_MOTOR_ENABLE_IF_STARTUP
    ctx_.debug_if_profile = nullptr;
#endif
    resetDebugProfileState();
}

void MotorManager::setDebugVFStartupProfile(const MotorVFStartupProfile* profile)
{
    ctx_.debug_vf_profile = profile;
#if LIB_MOTOR_ENABLE_IF_STARTUP
    ctx_.debug_if_profile = nullptr;
#endif
    resetDebugProfileState();
}

#if LIB_MOTOR_ENABLE_IF_STARTUP
void MotorManager::setDebugIFStartupProfile(const MotorIFStartupProfile* profile)
{
    ctx_.debug_if_profile = profile;
    ctx_.debug_vf_profile = nullptr;
    resetDebugProfileState();
    controller_.reset();

    if (state_ != State::RUN)
    {
        if_angle_gen_.reset();
        ctx_.angle_elec = 0.0f;
        ctx_.angle_elec_command = 0.0f;
        ctx_.speed_rpm = 0.0f;
    }

#if LIB_MOTOR_ENABLE_SMO
    if (target_mode_ == Mode::DEBUG_IF_SMO_OBSERVER)
    {
        smo_.reset();
        ctx_.smo_estimate = ObserverEstimate();
        ctx_.angle_elec_observer = ctx_.angle_elec_command;
        ctx_.speed_rpm_observer = 0.0f;
        event_.observer_converged = 0U;
        event_.speed_valid = 0U;
    }
#endif

#if LIB_MOTOR_ENABLE_HFI
    if (target_mode_ == Mode::DEBUG_IF_HFI_OBSERVER)
    {
        hfi_.reset();
        ctx_.hfi_estimate = ObserverEstimate();
        ctx_.hfi_injection = HfiInjectionCommand();
        ctx_.angle_elec_observer = ctx_.angle_elec_command;
        ctx_.speed_rpm_observer = 0.0f;
        event_.observer_converged = 0U;
        event_.speed_valid = 0U;
    }
#endif
}
#endif
#endif
} // namespace Lib_Motor

