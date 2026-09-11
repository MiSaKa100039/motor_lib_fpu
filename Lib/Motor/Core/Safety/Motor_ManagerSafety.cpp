#include "../Manager/Motor_Manager.h"

#include "../FSM/Motor_FSM.h"
#include "../Feature/Motor_FeatureRegistry.h"
#include "../Limit/Motor_TargetLimiter.h"
#include "Motor_CommandGuard.h"
#include "Motor_ConfigCheck.h"
#include "Motor_SignalHealth.h"

#include <cmath>

namespace Lib_Motor
{

#if LIB_MOTOR_ENABLE_STALL_PROTECTION || \
    (LIB_MOTOR_ENABLE_IF_STARTUP && !LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15)
static uint32_t secondsToTicks(float seconds, float dt)
{
    if (seconds <= 0.0f || dt <= 0.0f) return 0U;
    const float ticks = seconds / dt;
    if (ticks < 1.0f) return 1U;
    return static_cast<uint32_t>(ticks);
}
#endif

static MotorRuntimeFaultDetail hardwareFaultDetailFromFlags(uint32_t flags)
{
    if ((flags & MOTOR_HAL_HW_FAULT_TIM1_BREAK) != 0UL)
    {
        return MotorRuntimeFaultDetail::HARDWARE_BUS_OVERCURRENT_COMPARATOR;
    }
    if ((flags & MOTOR_HAL_HW_FAULT_TIM1_BREAK2) != 0UL)
    {
        return MotorRuntimeFaultDetail::HARDWARE_BUS_OVERCURRENT_COMPARATOR;
    }
    if ((flags & MOTOR_HAL_HW_FAULT_COMPARATOR) != 0UL)
    {
        return MotorRuntimeFaultDetail::HARDWARE_BUS_OVERCURRENT_COMPARATOR;
    }
    return MotorRuntimeFaultDetail::HARDWARE_FAULT_UNKNOWN;
}

void MotorManager::runSafetyCheck()
{
    if (state_ == State::UNINITIALIZED ||
        state_ == State::INIT ||
        state_ == State::ADC_CAL ||
        state_ == State::ERROR ||
        state_ == State::ISR_DIAGNOSTIC)
    {
        return;
    }

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    pollHardwareFaultLatch();

    FixedNumeric::q15_t max_phase_current =
        FixedNumeric::absoluteQ15(ctx_.i_a);
    const FixedNumeric::q15_t abs_ib = FixedNumeric::absoluteQ15(ctx_.i_b);
    const FixedNumeric::q15_t abs_ic = FixedNumeric::absoluteQ15(ctx_.i_c);
    if (abs_ib > max_phase_current) max_phase_current = abs_ib;
    if (abs_ic > max_phase_current) max_phase_current = abs_ic;

    if (ctx_.phase_overcurrent_q15 > 0 &&
        max_phase_current > ctx_.phase_overcurrent_q15)
    {
        setFault(Fault::OVERCURRENT,
                 MotorRuntimeFaultDetail::SOFTWARE_PHASE_OVERCURRENT);
    }

#if MOTOR_BUILD_HAS_BUS_CURRENT
    if (config_.sensor.has_bus_current &&
        ctx_.bus_overcurrent_q15 > 0 &&
        FixedNumeric::absoluteQ15(ctx_.i_bus) > ctx_.bus_overcurrent_q15)
    {
        setFault(Fault::OVERCURRENT,
                 MotorRuntimeFaultDetail::SOFTWARE_BUS_OVERCURRENT);
    }
#endif

    if (ctx_.v_bus > ctx_.vbus_overvoltage_q15)
    {
        setFault(Fault::OVERVOLT);
    }

    if (ctx_.vbus_undervoltage_q15 > 0 &&
        ctx_.v_bus < ctx_.vbus_undervoltage_q15)
    {
        setFault(Fault::UNDERVOLT);
    }

#if MOTOR_BUILD_NTC_SLOTS > 0
    for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; ++i)
    {
        if (!config_.sensor.ntc[i].enabled) continue;
        if (ctx_.temperature_deci_c[i] > ctx_.ntc_over_temp_deci_c[i])
        {
            setFault(Fault::OVERTEMP);
        }
    }
#endif

    MotorSignalHealth::checkPositionConsistency(*this);
#else
#if MOTOR_BUILD_NTC_SLOTS > 0 && !LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const float* temperatures = ctx_.temperature_c;
#else
    static const float temperatures[4] = {0.0f, 0.0f, 0.0f, 0.0f};
#endif

    pollHardwareFaultLatch();

    runtime_monitor_.check(config_.limit, config_.sensor,
                           runtimeCurrentToPhysical(ctx_, ctx_.i_a),
                           runtimeCurrentToPhysical(ctx_, ctx_.i_b),
                           runtimeCurrentToPhysical(ctx_, ctx_.i_c),
#if MOTOR_BUILD_HAS_BUS_CURRENT
                           runtimeCurrentToPhysical(ctx_, ctx_.i_bus),
#else
                           0.0f,
#endif
                           runtimeBusVoltageToPhysical(ctx_, ctx_.v_bus), temperatures,
                           state_, &fault_,
                           &runtime_fault_detail_);

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && MOTOR_BUILD_NTC_SLOTS > 0
    for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; ++i)
    {
        if (!config_.sensor.ntc[i].enabled) continue;
        if (ctx_.temperature_deci_c[i] > ctx_.ntc_over_temp_deci_c[i])
        {
            fault_ = static_cast<Fault>(
                static_cast<uint16_t>(fault_) |
                static_cast<uint16_t>(Fault::OVERTEMP));
        }
    }
#endif

    MotorSignalHealth::checkPositionConsistency(*this);
#endif
}

void MotorManager::clearStallState()
{
    event_.motor_stalled = 0U;
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    stall_candidate_ticks_ = 0U;
    stall_release_ticks_ = 0U;
    stall_restart_wait_ticks_ = 0U;
    stall_restart_attempts_ = 0U;
    stall_restart_pending_ = false;
#endif
}

Result MotorManager::validateModeSelection(Mode mode) const
{
    return MotorCommandGuard::validateModeSelection(config_, active_policy_, mode);
}

Result MotorManager::validateRunPolicy(const MotorRunPolicy& policy) const
{
    if (!checkConfigValid(config_))
    {
        return Result::InvalidParam;
    }

    if (!MotorFeatureRegistry::supportsRunPolicy(policy))
    {
        return Result::NotSupported;
    }

    const Fault feedback_fault = MotorConfigCheck::validateFeedback(config_, policy);
    if (feedback_fault != Fault::NONE)
    {
        return Result::NotSupported;
    }

    return Result::Ok;
}

void MotorManager::prepareModeTransition(Mode previous_mode, Mode next_mode)
{
    if (previous_mode == next_mode) return;

#if LIB_MOTOR_ENABLE_IF_STARTUP
    clearStartupRestartState();
#endif

    const bool touches_debug_or_calib =
        MotorCommandGuard::isDebugOrCalib(previous_mode) ||
        MotorCommandGuard::isDebugOrCalib(next_mode);

    if (touches_debug_or_calib)
    {
        controller_.reset();
    }

#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    if (next_mode == Mode::CALIB_RL_IDENTIFY)
    {
        rl_identify_routine_.reset();
        event_.rl_identify_done = 0U;
        if (config_fault_detail_ == MotorConfigFaultDetail::RL_IDENTIFY_FAILED)
        {
            config_fault_detail_ = MotorConfigFaultDetail::NONE;
        }
    }
#endif

    if (previous_mode == Mode::VELOCITY_CONTROL || next_mode == Mode::VELOCITY_CONTROL)
    {
        controller_.resetSpeedPid();
    }

    ctx_.iq_ref_limited = (state_ == State::RUN) ? ctx_.i_q : 0;
    ctx_.iq_ref_command = 0;
    ctx_.speed_pid_iq = 0;

    if (next_mode == Mode::VELOCITY_CONTROL)
    {
        ctx_.speed_ref_limited = ctx_.speed_rpm;
    }
    else
    {
        ctx_.speed_ref_limited = 0;
    }
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    ctx_.speed_slew_fraction_q16 = 0U;
#endif

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    event_.motor_stalled = 0U;
    stall_candidate_ticks_ = 0U;
    stall_release_ticks_ = 0U;
#endif
}

void MotorManager::transitionToStop(StopMode mode, bool clear_user_targets, bool reset_rl_identify)
{
    const bool use_critical =
        (config_.hal && config_.hal->enter_critical && config_.hal->exit_critical);
    if (use_critical) config_.hal->enter_critical();

    stop_mode_ = mode;
    run_requested_ = false;
    mode_ = Mode::NONE;
    run_phase_ = RunPhase::NONE;
    angle_state_ = AngleState::NONE;
    stop_state_ = StopState::IDLE;
    energy_state_ = EnergyState::IDLE;
    stream_state_ = StreamState::NONE;
    state_ = State::STOPPING;

    if (clear_user_targets)
    {
        target_mode_ = config_.control.startup_target_mode;
        clearControlTargets();
        clearStallState();
#if LIB_MOTOR_ENABLE_IF_STARTUP
        clearStartupRestartState();
#endif
    }
    else
    {
        clearDerivedTargets();
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        stall_candidate_ticks_ = 0U;
        stall_release_ticks_ = 0U;
#endif
    }

    ctx_.duty_a = 0;
    ctx_.duty_b = 0;
    ctx_.duty_c = 0;
    ctx_.v_d = 0;
    ctx_.v_q = 0;
    ctx_.v_alpha = 0;
    ctx_.v_beta = 0;
    controller_.reset();
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    if (reset_rl_identify)
    {
        rl_identify_routine_.reset();
    }
#else
    (void)reset_rl_identify;
#endif
#if LIB_MOTOR_ENABLE_HFI
    hfi_.setEnabled(false);
    ctx_.hfi_injection = HfiInjectionCommand();
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    resetIFStartupProfileState();
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    if_angle_gen_.reset();
#endif

    if (use_critical) config_.hal->exit_critical();
}

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
void MotorManager::updateStallProtection(float speed_error, float iq_ref, float iq_limit)
{
    const auto& stall = config_.motion.stall;
    if (!stall.enable || state_ != State::RUN || mode_ != Mode::VELOCITY_CONTROL)
    {
        event_.motor_stalled = 0U;
        stall_candidate_ticks_ = 0U;
        stall_release_ticks_ = 0U;
        return;
    }

    const bool candidate = MotorTargetLimiter::isStallCandidate(
        config_, runtimeSpeedToPhysical(ctx_, ctx_.speed_rpm), speed_error, iq_ref, iq_limit);
    const uint32_t confirm_ticks = secondsToTicks(stall.confirm_time_s, dt_);

    if (candidate)
    {
        stall_release_ticks_ = 0U;
        if (event_.motor_stalled == 0U)
        {
            if (++stall_candidate_ticks_ >= confirm_ticks)
            {
                event_.motor_stalled = 1U;
                stall_candidate_ticks_ = confirm_ticks;
                if (stall.count_total_events)
                {
                    ++stall_total_count_;
                }

                if (stall.auto_restart)
                {
                    if (stall_restart_attempts_ < stall.restart_count)
                    {
                        ++stall_restart_attempts_;
                        stall_restart_pending_ = true;
                        stall_restart_wait_ticks_ = 0U;
                        transitionToStop(stall.restart_stop_mode, false);
                    }
                    else
                    {
                        if (stall.require_manual_clear_after_retry_exhausted)
                        {
                            setFault(Fault::STALL_RETRY_EXHAUSTED);
                        }
                        stopForFault(Fault::STALL);
                    }
                }
                else if (stall.latch_fault)
                {
                    stopForFault(Fault::STALL);
                }
            }
        }
        return;
    }

    stall_candidate_ticks_ = 0U;
    if (event_.motor_stalled != 0U)
    {
        const bool recovered =
            fabsf(runtimeSpeedToPhysical(ctx_, ctx_.speed_rpm)) >= stall.release_speed_rpm ||
            fabsf(speed_error) <= stall.speed_error_rpm;
        if (recovered)
        {
            const uint32_t release_ticks = secondsToTicks(stall.release_confirm_time_s, dt_);
            if (++stall_release_ticks_ >= release_ticks)
            {
                event_.motor_stalled = 0U;
                stall_release_ticks_ = 0U;
                stall_restart_attempts_ = 0U;
                // 启动渐进解冻: 200 ms @ control_freq (约 4000 tick @ 20kHz)
                stall_thaw_remaining_ticks_ =
                    static_cast<uint32_t>(0.2f * config_.control.control_freq_hz);
            }
        }
        else
        {
            stall_release_ticks_ = 0U;
        }
    }

    /* 渐进解冻: 速度环积分 decay 从 0.98 → 1.0 线性, 防止释放瞬间积分暴冲 */
    if (stall_thaw_remaining_ticks_ > 0U)
    {
        --stall_thaw_remaining_ticks_;
        // 线性插值: 起始 0.98, 终止 1.0
        // 当前接口仅有 decayIntegral(factor), 用一个略小于 1 的 factor
        // 总解冻期 N ticks: 每 tick 衰减 1 + (1-0.98)/N ≈ 1.0, 保留对小残积分的微抑制
        const float thaw_factor =
            0.98f + 0.02f * (1.0f - static_cast<float>(stall_thaw_remaining_ticks_) /
                                       static_cast<float>(0.2f * config_.control.control_freq_hz + 1.0f));
        controller_.decaySpeedIntegral(thaw_factor);
    }
}

void MotorManager::processPendingStallRestart()
{
    if (!stall_restart_pending_ || state_ != State::STOP)
    {
        return;
    }

    const uint32_t wait_ticks = secondsToTicks(config_.motion.stall.restart_interval_s, dt_);
    if (++stall_restart_wait_ticks_ < wait_ticks)
    {
        return;
    }

    if (config_.motion.stall.clear_fault_before_restart)
    {
        Motor_FSM::clearFaultBitsNow(*this);
    }

    stall_restart_pending_ = false;
    stall_restart_wait_ticks_ = 0U;
    event_.motor_stalled = 0U;
    stall_candidate_ticks_ = 0U;
    stall_release_ticks_ = 0U;
    run_requested_ = (target_mode_ != Mode::NONE);
}
#else
void MotorManager::updateStallProtection(float, float, float) {}
void MotorManager::processPendingStallRestart() {}
#endif

void MotorManager::setFault(Fault fault)
{
    setFault(fault, MotorRuntimeFaultDetail::NONE);
}

#if LIB_MOTOR_ENABLE_IF_STARTUP
void MotorManager::clearStartupRestartState()
{
    startup_restart_wait_ticks_ = 0U;
    startup_restart_attempts_ = 0U;
    startup_restart_pending_ = false;
}

bool MotorManager::ifStartupProfileComplete() const
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    return ctx_.if_profile.complete;
#else
    return ctx_.if_profile_complete;
#endif
}

void MotorManager::handleIFStartupFailure()
{
    const bool can_retry =
        active_policy_.startup_auto_restart &&
        startup_restart_attempts_ < active_policy_.startup_max_retry_count;
    if (!can_retry)
    {
        stopForFault(Fault::OBSERVER_LOSS,
                     MotorRuntimeFaultDetail::IF_SMO_STARTUP_HANDOVER_FAILED);
        return;
    }

    ++startup_restart_attempts_;
    startup_restart_pending_ = true;
    startup_restart_wait_ticks_ = 0U;
    transitionToStop(StopMode::COAST, false);
}

void MotorManager::processPendingStartupRestart()
{
    if (!startup_restart_pending_ || state_ != State::STOP)
    {
        return;
    }

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const uint32_t wait_ticks = startup_restart_interval_ticks_;
#else
    const uint32_t wait_ticks =
        secondsToTicks(active_policy_.startup_restart_interval_s, dt_);
#endif
    if (wait_ticks > 0U && ++startup_restart_wait_ticks_ < wait_ticks)
    {
        return;
    }

    startup_restart_pending_ = false;
    startup_restart_wait_ticks_ = 0U;
    resetIFStartupProfileState();
    resetIFStartupObserverState();
    run_requested_ = (target_mode_ != Mode::NONE);
}
#endif

void MotorManager::setFault(Fault fault, MotorRuntimeFaultDetail detail)
{
    fault_ = (Fault)((uint16_t)fault_ | (uint16_t)fault);
    if (detail != MotorRuntimeFaultDetail::NONE &&
        runtime_fault_detail_ == MotorRuntimeFaultDetail::NONE)
    {
        runtime_fault_detail_ = detail;
    }
}

void MotorManager::setConfigFaultDetail(MotorConfigFaultDetail detail)
{
    config_fault_detail_ = detail;
}

void MotorManager::stopForFault(Fault fault)
{
    Motor_FSM::latchFault(*this, fault);
}

void MotorManager::stopForFault(Fault fault, MotorRuntimeFaultDetail detail)
{
    setFault(fault, detail);
    Motor_FSM::latchFault(*this, fault);
}

void MotorManager::pollHardwareFaultLatch()
{
    if (config_.hal == nullptr ||
        config_.hal->read_hardware_fault_flags == nullptr)
    {
        return;
    }

    const uint32_t flags = config_.hal->read_hardware_fault_flags();
    if (flags == MOTOR_HAL_HW_FAULT_NONE)
    {
        return;
    }

    hardware_fault_flags_ |= flags;
    setFault(Fault::OVERCURRENT, hardwareFaultDetailFromFlags(flags));
}

void MotorManager::clearRuntimeFaultDetail()
{
    if (config_.hal != nullptr &&
        config_.hal->clear_hardware_fault_flags != nullptr)
    {
        const uint32_t clear_flags =
            (hardware_fault_flags_ != MOTOR_HAL_HW_FAULT_NONE)
                ? hardware_fault_flags_
                : MOTOR_HAL_HW_FAULT_ALL;
        config_.hal->clear_hardware_fault_flags(clear_flags);
    }
    runtime_fault_detail_ = MotorRuntimeFaultDetail::NONE;
    hardware_fault_flags_ = MOTOR_HAL_HW_FAULT_NONE;
}

bool MotorManager::validateControlFeedbackSource()
{
    return Motor_FSM::validateClosedLoopEntry(*this);
}

bool MotorManager::checkConfigValid(const MotorConfig& cfg) const
{
    return MotorConfigCheck::validateBase(cfg) == Fault::NONE;
}

} // namespace Lib_Motor
