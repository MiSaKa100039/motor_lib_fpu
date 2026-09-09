#include "../Manager/Motor_Manager.h"

#include "../FSM/Motor_FSM.h"

#include "../../Control/Utils/FocMath.h"

#include "../../Control/Modulation_SVPWM.h"

#include <cmath>

namespace Lib_Motor
{

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && \
    (LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL || LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_IF_ANY)
namespace
{

void resetQ15ProfileAccumulators(RuntimeQ15ProfileState& state)
{
    state.speed_remainder_accum = 0U;
    state.duty_remainder_accum = 0U;
    state.id_remainder_accum = 0U;
    state.iq_remainder_accum = 0U;
}

FixedNumeric::q15_t advanceQ15ProfileValue(FixedNumeric::q15_t current,
                                           std::int32_t step_q15,
                                           std::uint32_t remainder_q15,
                                           std::int8_t remainder_sign,
                                           uint32_t duration_ticks,
                                           uint32_t& remainder_accum)
{
    if (duration_ticks == 0U)
    {
        duration_ticks = 1U;
    }

    std::int32_t next = static_cast<std::int32_t>(current) + step_q15;
    if (remainder_q15 != 0U && remainder_sign != 0)
    {
        if (remainder_accum >= duration_ticks)
        {
            remainder_accum = 0U;
        }

        const uint32_t remaining = duration_ticks - remainder_accum;
        if (remainder_q15 >= remaining)
        {
            remainder_accum = remainder_q15 - remaining;
            next += static_cast<std::int32_t>(remainder_sign);
        }
        else
        {
            remainder_accum += remainder_q15;
        }
    }

    return FixedNumeric::saturateQ15(next);
}

uint32_t advanceQ15ProfileTicks(RuntimeQ15ProfileState& state, uint32_t duration_ticks)
{
    if (duration_ticks == 0U)
    {
        duration_ticks = 1U;
    }
    if (state.phase_ticks < duration_ticks)
    {
        state.phase_ticks++;
    }
    return state.phase_ticks;
}

bool advanceIFProfileQ15(const RuntimeIFStartupPhaseQ15* phases,
                         uint8_t phase_count,
                         RuntimeQ15ProfileState& state,
                         RuntimeSpeed& speed_rpm,
                         RuntimeCurrent& id_ref,
                         RuntimeCurrent& iq_ref)
{
    if (phases == nullptr || phase_count == 0U)
    {
        return false;
    }

    if (state.phase >= phase_count)
    {
        state.phase = static_cast<uint8_t>(phase_count - 1U);
    }

    if (!state.phase_started)
    {
        state.phase_started = true;
        state.phase_ticks = 0U;
        state.complete = false;
        resetQ15ProfileAccumulators(state);
    }

    const RuntimeIFStartupPhaseQ15& phase = phases[state.phase];
    const uint32_t ramp_ticks = (phase.duration_ticks == 0U) ? 1U : phase.duration_ticks;
    const uint32_t hold_ticks =
        (phase.type == MotorIFStartupPhaseType::ALIGNMENT) ? phase.hold_ticks : 0U;
    const uint32_t total_ticks =
        (0xFFFFFFFFUL - ramp_ticks < hold_ticks) ? 0xFFFFFFFFUL : ramp_ticks + hold_ticks;
    const uint32_t phase_ticks = advanceQ15ProfileTicks(state, total_ticks);
    const bool ramp_complete = phase_ticks >= ramp_ticks;
    const bool phase_complete = phase_ticks >= total_ticks;

    if (ramp_complete)
    {
        state.speed_rpm = phase.final_speed_rpm;
        state.id = phase.final_id;
        state.iq = phase.final_iq;
    }
    else
    {
        state.speed_rpm = advanceQ15ProfileValue(state.speed_rpm,
                                                 phase.speed_step_q15,
                                                 phase.speed_remainder_q15,
                                                 phase.speed_remainder_sign,
                                                 ramp_ticks,
                                                 state.speed_remainder_accum);
        state.id = advanceQ15ProfileValue(state.id,
                                          phase.id_step_q15,
                                          phase.id_remainder_q15,
                                          phase.id_remainder_sign,
                                          ramp_ticks,
                                          state.id_remainder_accum);
        state.iq = advanceQ15ProfileValue(state.iq,
                                          phase.iq_step_q15,
                                          phase.iq_remainder_q15,
                                          phase.iq_remainder_sign,
                                          ramp_ticks,
                                          state.iq_remainder_accum);
    }

    speed_rpm = state.speed_rpm;
    id_ref = state.id;
    iq_ref = state.iq;

    if (phase_complete)
    {
        if ((state.phase + 1U) < phase_count)
        {
            state.phase++;
            state.phase_ticks = 0U;
            resetQ15ProfileAccumulators(state);
        }
        else
        {
            state.complete = true;
        }
    }

    return true;
}

} // namespace
#endif

#if !LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && \
    (LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_IF_ANY)
namespace
{

bool advanceIFProfileFloat(const MotorIFStartupProfile* profile,
                           float dt,
                           uint8_t& phase_index,
                           float& phase_time_s,
                           float& start_speed_rpm,
                           float& start_id,
                           float& start_iq,
                           bool& phase_started,
                           bool& complete,
                           float& speed_rpm,
                           float& id_ref,
                           float& iq_ref)
{
    if (profile == nullptr || profile->phases == nullptr || profile->phase_count == 0U)
    {
        return false;
    }

    if (phase_index >= profile->phase_count)
    {
        phase_index = static_cast<uint8_t>(profile->phase_count - 1U);
    }
    if (!phase_started)
    {
        phase_started = true;
        phase_time_s = 0.0f;
        complete = false;
    }

    const volatile MotorIFStartupPhase& phase = profile->phases[phase_index];
    const float ramp_time_s = (phase.duration_s < dt) ? dt : phase.duration_s;
    const float hold_time_s =
        (phase.type == MotorIFStartupPhaseType::ALIGNMENT) ? phase.hold_time_s : 0.0f;
    const float total_time_s = ramp_time_s + hold_time_s;
    phase_time_s += dt;

    float ratio = phase_time_s / ramp_time_s;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < 0.0f) ratio = 0.0f;

    speed_rpm = start_speed_rpm + (phase.final_speed_rpm - start_speed_rpm) * ratio;
    id_ref = start_id + (phase.final_id_a - start_id) * ratio;
    iq_ref = start_iq + (phase.final_iq_a - start_iq) * ratio;

    if (phase_time_s >= total_time_s)
    {
        speed_rpm = phase.final_speed_rpm;
        id_ref = phase.final_id_a;
        iq_ref = phase.final_iq_a;

        if ((phase_index + 1U) < profile->phase_count)
        {
            phase_index++;
            phase_time_s = 0.0f;
            start_speed_rpm = speed_rpm;
            start_id = id_ref;
            start_iq = iq_ref;
        }
        else
        {
            phase_time_s = total_time_s;
            complete = true;
        }
    }

    return true;
}

} // namespace
#endif

#if LIB_MOTOR_ENABLE_IF_STARTUP
void MotorManager::resetIFStartupProfileState()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.if_profile = RuntimeQ15ProfileState{};
#else
    ctx_.if_profile_phase = 0U;
    ctx_.if_profile_phase_time_s = 0.0f;
    ctx_.if_profile_start_speed_rpm = 0.0f;
    ctx_.if_profile_start_id = 0.0f;
    ctx_.if_profile_start_iq = 0.0f;
    ctx_.if_profile_phase_started = false;
    ctx_.if_profile_complete = false;
#endif
}

void MotorManager::resetIFStartupObserverState()
{
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    smo_.reset();
    resetSmoAngleDirectionLatch();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.smo_estimate = ObserverEstimateQ15{};
#else
    ctx_.smo_estimate = ObserverEstimate{};
#endif
    ctx_.angle_elec_observer = runtimeAngleFromRadians(0.0f);
    ctx_.speed_rpm_observer = runtimeSpeedFromPhysical(ctx_, 0.0f);
#endif
    event_.observer_converged = 0U;
    event_.speed_valid = 0U;
}

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
bool MotorManager::getIFStartupProfileCommand(RuntimeSpeed& speed_rpm,
                                              RuntimeCurrent& id_ref,
                                              RuntimeCurrent& iq_ref)
{
    return advanceIFProfileQ15(ctx_.if_profile_phases,
                               ctx_.if_profile_phase_count,
                               ctx_.if_profile,
                               speed_rpm,
                               id_ref,
                               iq_ref);
}
#else
bool MotorManager::getIFStartupProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref)
{
    return advanceIFProfileFloat(config_.observer.if_startup_profile,
                                 dt_,
                                 ctx_.if_profile_phase,
                                 ctx_.if_profile_phase_time_s,
                                 ctx_.if_profile_start_speed_rpm,
                                 ctx_.if_profile_start_id,
                                 ctx_.if_profile_start_iq,
                                 ctx_.if_profile_phase_started,
                                 ctx_.if_profile_complete,
                                 speed_rpm,
                                 id_ref,
                                 iq_ref);
}
#endif

#if MOTOR_LIB_HOST_TEST && LIB_MOTOR_ENABLE_IF_STARTUP
bool MotorManager::stepIFStartupProfileForTest(float& speed_rpm,
                                               float& id_ref,
                                               float& iq_ref)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    RuntimeSpeed speed = 0;
    RuntimeCurrent id = 0;
    RuntimeCurrent iq = 0;
    const bool active = getIFStartupProfileCommand(speed, id, iq);
    speed_rpm = runtimeSpeedToUserPhysical(config_, ctx_, speed);
    id_ref = runtimeCurrentToPhysical(ctx_, id);
    iq_ref = runtimeCurrentToPhysical(ctx_, iq);
    return active;
#else
    return getIFStartupProfileCommand(speed_rpm, id_ref, iq_ref);
#endif
}
#endif
#endif

#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
void MotorManager::resetDebugProfileState()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.debug_profile = RuntimeQ15ProfileState{};
    ctx_.debug_ramp_speed_step_q15 = 0U;
#else
    ctx_.debug_profile_phase = 0U;
    ctx_.debug_profile_phase_time_s = 0.0f;
    ctx_.debug_profile_start_speed_rpm = 0.0f;
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    ctx_.debug_profile_start_duty = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    ctx_.debug_profile_start_id = 0.0f;
    ctx_.debug_profile_start_iq = 0.0f;
#endif
    ctx_.debug_profile_phase_started = false;
    ctx_.debug_profile_complete = false;
#endif
    if_angle_gen_.reset();
}
#endif

#if MOTOR_LIB_HOST_TEST && LIB_MOTOR_ENABLE_DEBUG_IF_ANY
bool MotorManager::stepDebugIFProfileForTest(float& speed_rpm,
                                             float& id_ref,
                                             float& iq_ref)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    RuntimeSpeed speed = 0;
    RuntimeCurrent id = 0;
    RuntimeCurrent iq = 0;
    const bool active = getDebugIFProfileCommand(speed, id, iq);
    speed_rpm = runtimeSpeedToUserPhysical(config_, ctx_, speed);
    id_ref = runtimeCurrentToPhysical(ctx_, id);
    iq_ref = runtimeCurrentToPhysical(ctx_, iq);
    return active;
#else
    return getDebugIFProfileCommand(speed_rpm, id_ref, iq_ref);
#endif
}
#endif

#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
bool MotorManager::getDebugVFProfileCommand(RuntimeSpeed& speed_rpm, RuntimeDuty& duty)
{
    const uint8_t phase_count = ctx_.debug_vf_profile_phase_count;
    if (phase_count == 0U)
    {
        return false;
    }

    RuntimeQ15ProfileState& state = ctx_.debug_profile;
    if (state.phase >= phase_count)
    {
        state.phase = static_cast<uint8_t>(phase_count - 1U);
    }

    if (!state.phase_started)
    {
        state.phase_started = true;
        state.phase_ticks = 0U;
        resetQ15ProfileAccumulators(state);
    }

    const RuntimeVFStartupPhaseQ15& phase =
        ctx_.debug_vf_profile_phases[state.phase];
    const uint32_t duration_ticks =
        (phase.duration_ticks == 0U) ? 1U : phase.duration_ticks;
    const uint32_t phase_ticks = advanceQ15ProfileTicks(state, duration_ticks);
    const bool final_tick = (phase_ticks >= duration_ticks);

    if (final_tick)
    {
        state.speed_rpm = phase.final_speed_rpm;
        state.duty = runtimeClampDuty(phase.final_duty, FixedNumeric::kQ15One);
    }
    else
    {
        state.speed_rpm = advanceQ15ProfileValue(state.speed_rpm,
                                                 phase.speed_step_q15,
                                                 phase.speed_remainder_q15,
                                                 phase.speed_remainder_sign,
                                                 duration_ticks,
                                                 state.speed_remainder_accum);
        state.duty = runtimeClampDuty(
            advanceQ15ProfileValue(state.duty,
                                   phase.duty_step_q15,
                                   phase.duty_remainder_q15,
                                   phase.duty_remainder_sign,
                                   duration_ticks,
                                   state.duty_remainder_accum),
            FixedNumeric::kQ15One);
    }

    speed_rpm = state.speed_rpm;
    duty = state.duty;

    if (final_tick && (state.phase + 1U) < phase_count)
    {
        state.phase++;
        state.phase_ticks = 0U;
        resetQ15ProfileAccumulators(state);
    }

    return true;
}
#else
bool MotorManager::getDebugVFProfileCommand(float& speed_rpm, float& duty)
{
    const MotorVFStartupProfile* profile = ctx_.debug_vf_profile;
    if (profile == nullptr || profile->phases == nullptr || profile->phase_count == 0U)
    {
        return false;
    }

    if (ctx_.debug_profile_phase >= profile->phase_count)
    {
        ctx_.debug_profile_phase = (uint8_t)(profile->phase_count - 1U);
    }

    const volatile MotorVFStartupPhase& phase = profile->phases[ctx_.debug_profile_phase];
    float duration_s = phase.duration_s;
    float final_speed_rpm = phase.final_speed_rpm;
    float final_duty = phase.final_duty;
    if (!ctx_.debug_profile_phase_started)
    {
        ctx_.debug_profile_phase_started = true;
        ctx_.debug_profile_phase_time_s = 0.0f;
    }

    if (duration_s < dt_) duration_s = dt_;

    float phase_time_s = ctx_.debug_profile_phase_time_s + dt_;
    float ratio = phase_time_s / duration_s;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < 0.0f) ratio = 0.0f;

    speed_rpm = ctx_.debug_profile_start_speed_rpm +
                (final_speed_rpm - ctx_.debug_profile_start_speed_rpm) * ratio;
    duty = ctx_.debug_profile_start_duty +
           (final_duty - ctx_.debug_profile_start_duty) * ratio;

    ctx_.debug_profile_phase_time_s = phase_time_s;
    if (phase_time_s >= duration_s)
    {
        if ((ctx_.debug_profile_phase + 1U) < profile->phase_count)
        {
            ctx_.debug_profile_phase++;
            ctx_.debug_profile_phase_time_s = 0.0f;
            ctx_.debug_profile_start_speed_rpm = final_speed_rpm;
            ctx_.debug_profile_start_duty = final_duty;
            ctx_.debug_profile_phase_started = true;
        }
        else
        {
            ctx_.debug_profile_phase_time_s = duration_s;
            ctx_.debug_profile_start_speed_rpm = final_speed_rpm;
            ctx_.debug_profile_start_duty = final_duty;
        }
    }

    return true;
}
#endif
#endif

#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
bool MotorManager::getDebugIFProfileCommand(RuntimeSpeed& speed_rpm,
                                            RuntimeCurrent& id_ref,
                                            RuntimeCurrent& iq_ref)
{
    return advanceIFProfileQ15(ctx_.debug_if_profile_phases,
                               ctx_.debug_if_profile_phase_count,
                               ctx_.debug_profile,
                               speed_rpm,
                               id_ref,
                               iq_ref);
}
#else
bool MotorManager::getDebugIFProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref)
{
    return advanceIFProfileFloat(ctx_.debug_if_profile,
                                 dt_,
                                 ctx_.debug_profile_phase,
                                 ctx_.debug_profile_phase_time_s,
                                 ctx_.debug_profile_start_speed_rpm,
                                 ctx_.debug_profile_start_id,
                                 ctx_.debug_profile_start_iq,
                                 ctx_.debug_profile_phase_started,
                                 ctx_.debug_profile_complete,
                                 speed_rpm,
                                 id_ref,
                                 iq_ref);
}
#endif
#endif


/*
 * runCurrentLock -- 电流闭环锁定 (预对齐/RL辨识用)
 *
 * 将角度锁定为 0 度, 用 d/q 电流闭环控制。
 *
 * 原理 (theta = 0):
 *   Park 简化:   i_d = i_alpha, i_q = i_beta
 *   PID 控制:    输出 v_d, v_q
 *   InvPark 简化: v_alpha = v_d, v_beta = v_q
 *
 * 用途:
 *   - 预对齐: 注入 d 轴电流将转子拉到已知位置
 *   - RL 辨识: 注入阶跃电压 -> 测量 V/I -> 计算 Rs/Ls
 *   - 编码器零点搜索
 *
 * 注意: IF 对齐由启动 profile 的 ALIGNMENT 首段执行，不经过本函数。
 */
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY || LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
void MotorManager::runCurrentLock()
{
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    /* [P2] mode=CALIB_RL_IDENTIFY 时交给 RL 辨识状态机开环注入 Vstep, 不走常规电流 PID 锁定 */
    if (mode_ == Mode::CALIB_RL_IDENTIFY)
    {
        ctx_.angle_elec_command = 0.0f;
        ctx_.angle_elec = ctx_.angle_elec_command;
        ctx_.i_d = ctx_.i_alpha;
        ctx_.i_q = ctx_.i_beta;

        rl_identify_routine_.step(*this);
        // 调制输出仍走 Lock 的简化 InvPark+SVPWM 路径 (角度=0):
        ctx_.v_alpha = ctx_.v_d;
        ctx_.v_beta  = ctx_.v_q;
        float duty[3];
        svpwm_modulate(ctx_.v_alpha, ctx_.v_beta, ctx_.v_bus, duty);
        ctx_.duty_a = duty[0];
        ctx_.duty_b = duty[1];
        ctx_.duty_c = duty[2];
        return;
    }
#endif

    /* 电流锁定: 角度固定为 0 度, 仅 d 轴注入电流 (预对齐) */
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.angle_elec_command = 0;
    ctx_.angle_elec = ctx_.angle_elec_command;
    ctx_.angle_sin_cos = FixedNumeric::sinCos(ctx_.angle_elec);

    RuntimeCurrent id_ref = ctx_.target_id;
    RuntimeCurrent iq_ref = ctx_.target_iq;

    id_ref = clampCurrentTargetQ15(id_ref);
    iq_ref = clampCurrentTargetQ15(iq_ref);

#if LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
    const bool debug_current_lock_idle =
        mode_ == Mode::DEBUG_CURRENT_LOCK &&
        run_phase_ == RunPhase::RUN_DIRECT &&
        id_ref == 0 &&
        iq_ref == 0;
#else
    const bool debug_current_lock_idle = false;
#endif

    ctx_.target_id = id_ref;
    ctx_.target_iq = iq_ref;
    ctx_.i_d = ctx_.i_alpha;
    ctx_.i_q = ctx_.i_beta;

    if (debug_current_lock_idle)
    {
        controller_.resetCurrentPid();
        ctx_.v_d = 0;
        ctx_.v_q = 0;
    }
    else
    {
        refreshFixedCurrentPidOutputLimit();
        ctx_.v_d = controller_.updateFixedCurrentD(
            FixedNumeric::subtractQ15(ctx_.target_id, ctx_.i_d));
        ctx_.v_q = controller_.updateFixedCurrentQ(
            FixedNumeric::subtractQ15(ctx_.target_iq, ctx_.i_q));
    }

    energy_state_ = (ctx_.target_id == 0 && ctx_.target_iq == 0)
        ? EnergyState::IDLE
        : EnergyState::DRIVE;

    ctx_.v_alpha = ctx_.v_d;
    ctx_.v_beta = ctx_.v_q;

    const FixedNumeric::Ab voltage_ab{ctx_.v_alpha, ctx_.v_beta};
    const FixedNumeric::DutyAbc duty =
        (config_.control.modulation == ModulationMethod::SPWM)
            ? FixedNumeric::spwmVbusNormalized(voltage_ab)
            : FixedNumeric::svpwmVbusNormalized(voltage_ab);
    setPwmDutyQ15(duty);
#else
    ctx_.angle_elec_command = runtimeAngleFromRadians(0.0f);
    ctx_.angle_elec = ctx_.angle_elec_command;

    float id_ref = runtimeCurrentToPhysical(ctx_, ctx_.target_id);
    float iq_ref = runtimeCurrentToPhysical(ctx_, ctx_.target_iq);

    iq_ref = Motor_FSM::limitIqReferenceByEnergy(
        *this, iq_ref, runtimeSpeedToPhysical(ctx_, ctx_.speed_rpm));

#if LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
    const bool debug_current_lock_idle =
        mode_ == Mode::DEBUG_CURRENT_LOCK &&
        run_phase_ == RunPhase::RUN_DIRECT &&
        fabsf(id_ref) < 1.0e-6f &&
        fabsf(iq_ref) < 1.0e-6f;
#else
    const bool debug_current_lock_idle = false;
#endif

    /* Park 变换简化 (theta=0 度):
     *   i_d =  i_alpha * cos(0) + i_beta * sin(0) = i_alpha
     *   i_q = -i_alpha * sin(0) + i_beta * cos(0) = i_beta
     */
    ctx_.i_d = ctx_.i_alpha * 1.0f + ctx_.i_beta * 0.0f;
    ctx_.i_q = -ctx_.i_alpha * 0.0f + ctx_.i_beta * 1.0f;

    if (debug_current_lock_idle)
    {
        controller_.resetCurrentPid();
        ctx_.v_d = 0.0f;
        ctx_.v_q = 0.0f;
    }
    else
    {
        updateCurrentPidVoltageLimit(0.0f);

        /* d/q 电流 PID */
        ctx_.v_d = controller_.pid_d.update(id_ref - ctx_.i_d, dt_);
        ctx_.v_q = controller_.pid_q.update(iq_ref - ctx_.i_q, dt_);
    }

    /* InvPark 变换简化 (theta=0 度):
     *   v_alpha = v_d * 1 - v_q * 0 = v_d
     *   v_beta  = v_d * 0 + v_q * 1 = v_q
     */
    ctx_.v_alpha = ctx_.v_d;
    ctx_.v_beta  = ctx_.v_q;

    /* 调制 -> 三相占空比 */
    switch (config_.control.modulation)
    {
        case ModulationMethod::SPWM:
        {
            float duty[3];
            spwm_modulate(ctx_.v_alpha, ctx_.v_beta, ctx_.v_bus, duty);
            ctx_.duty_a = duty[0];
            ctx_.duty_b = duty[1];
            ctx_.duty_c = duty[2];
        }
        break;
        case ModulationMethod::SVPWM:
        default:
        {
            float duty[3];
            svpwm_modulate(ctx_.v_alpha, ctx_.v_beta, ctx_.v_bus, duty);
            ctx_.duty_a = duty[0];
            ctx_.duty_b = duty[1];
            ctx_.duty_c = duty[2];
        }
        break;
    }
#endif
}
#endif

#if LIB_MOTOR_ENABLE_SMO_OBSERVER
void MotorManager::runSMOShadowObserver()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    smo_.update(ctx_.v_alpha, ctx_.v_beta,
                ctx_.i_alpha, ctx_.i_beta,
                &ctx_.smo_estimate);

    ctx_.speed_rpm_observer = ctx_.smo_estimate.speed_rpm_q15;
    ctx_.angle_elec_observer =
        correctSmoAngleForControl(ctx_.smo_estimate.angle_phase, ctx_.speed_rpm);
#else
    smo_.update(ctx_.v_alpha, ctx_.v_beta,
                ctx_.i_alpha, ctx_.i_beta,
                dt_, &ctx_.smo_estimate);

    ctx_.speed_rpm_observer =
        ctx_.smo_estimate.speed_rad_s * 60.0f /
        (TWO_PI * config_.physical.pole_pairs);
    ctx_.angle_elec_observer =
        correctSmoAngleForControl(ctx_.smo_estimate.angle_rad, ctx_.speed_rpm);
#endif
    event_.observer_converged = ctx_.smo_estimate.valid ? 1U : 0U;
    event_.speed_valid = ctx_.smo_estimate.valid ? 1U : 0U;
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
    if (mode_ == Mode::DEBUG_IF_SMO_OBSERVER)
    {
        angle_state_ = AngleState::SMO;
    }
#endif
}
#endif

/*
 * runIFControl -- I/F 对齐与强拖电流闭环控制
 *
 * 电流模式开环调度: 强拖角度 (开环) + 电流闭环 (PID)。
 *
 *   数据流:
 *     AngleGenerator -> 生成开环角度 (速度积分)
 *     Clarke -> Park -> PID(d/q) -> InvPark -> 调制 -> PWM
 *
 * 与 V/F 的区别:
 *   V/F: 电压开环 (直接输出电压, 无电流反馈)
 *   I/F: 电流闭环 (PID 跟踪 dq 电流, 角度开环)
 *
 * 用途: profile 首段完成零角度对齐，后续 RAMP 拖动至 SMO 收敛后切速度闭环
 */
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
void MotorManager::runIFControl()
{
    /* ================================================================
     * [1] 开环角度生成 (同 VF 使用 if_angle_gen_)
     *
     * 注意: 启动进入 FORCE_DRAG 前会 reset；ALIGNMENT profile 阶段速度固定为零，
     *       因此角度保持在零电角度，随后 RAMP 从同一状态连续起步。
     * ================================================================ */
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    RuntimeSpeed target_rpm = ctx_.target_rpm;
    RuntimeCurrent profile_id = ctx_.target_id;
    RuntimeCurrent profile_iq = ctx_.target_iq;
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    const bool debug_profile_active =
        (
#if LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
          mode_ == Mode::DEBUG_IF_DRAG ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
          mode_ == Mode::DEBUG_IF_SMO_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
          mode_ == Mode::DEBUG_IF_HFI_OBSERVER ||
#endif
          false) &&
        run_phase_ == RunPhase::RUN_DIRECT;
    bool use_profile = debug_profile_active &&
        getDebugIFProfileCommand(target_rpm, profile_id, profile_iq);
#else
    constexpr bool debug_profile_active = false;
    bool use_profile = false;
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    // 正式 FORCE_DRAG 必须使用配置的分段启动 profile。
    if (!use_profile && run_phase_ == RunPhase::FORCE_DRAG)
    {
        use_profile = getIFStartupProfileCommand(target_rpm, profile_id, profile_iq);
    }
#endif
    (void)use_profile;

    /* 正式 IF 不覆盖用户闭环目标；Debug 仍显示 profile 的即时指令。 */
    if (debug_profile_active && use_profile)
    {
        ctx_.target_rpm = target_rpm;
        ctx_.target_id = profile_id;
        ctx_.target_iq = profile_iq;
    }

    if_angle_gen_.setTargetSpeedQ15(target_rpm, 0U, 0U);
    if_angle_gen_.updateQ15();
    const FixedNumeric::phase_u32_t phase = if_angle_gen_.getAnglePhase();
    const RuntimeSpeed speed_rpm_q15 = if_angle_gen_.getSpeedQ15();

    ctx_.angle_elec_command = phase;
    ctx_.angle_elec = ctx_.angle_elec_command;
    ctx_.speed_rpm = speed_rpm_q15;
    ctx_.angle_sin_cos = FixedNumeric::sinCos(phase);

    const RuntimeCurrent id_ref = clampCurrentTargetQ15(profile_id);
    const RuntimeCurrent iq_ref = clampCurrentTargetQ15(profile_iq);
    ctx_.iq_ref_command = iq_ref;
    ctx_.iq_ref_limited = iq_ref;
    energy_state_ = (id_ref == 0 && iq_ref == 0)
        ? EnergyState::IDLE
        : EnergyState::DRIVE;
#else
    float target_rpm = runtimeSpeedToPhysical(ctx_, ctx_.target_rpm);
    float profile_id = runtimeCurrentToPhysical(ctx_, ctx_.target_id);
    float profile_iq = runtimeCurrentToPhysical(ctx_, ctx_.target_iq);
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    const bool debug_profile_active =
        (
#if LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
          mode_ == Mode::DEBUG_IF_DRAG ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
          mode_ == Mode::DEBUG_IF_SMO_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
          mode_ == Mode::DEBUG_IF_HFI_OBSERVER ||
#endif
          false) &&
        run_phase_ == RunPhase::RUN_DIRECT;
    bool use_profile = debug_profile_active &&
        getDebugIFProfileCommand(target_rpm, profile_id, profile_iq);
#else
    constexpr bool debug_profile_active = false;
    bool use_profile = false;
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    // 正式 FORCE_DRAG 必须使用配置的分段启动 profile。
    if (!use_profile && run_phase_ == RunPhase::FORCE_DRAG)
    {
        use_profile = getIFStartupProfileCommand(target_rpm, profile_id, profile_iq);
    }
#endif
    if (use_profile)
    {
        target_rpm = runtimeUserSpeedToInternalPhysical(config_, target_rpm);
    }

    /* 正式 IF 不覆盖用户闭环目标；Debug 仍显示 profile 的即时指令。 */
    if (debug_profile_active && use_profile)
    {
        ctx_.target_rpm = runtimeSpeedFromPhysical(ctx_, target_rpm);
        ctx_.target_id = runtimeCurrentFromPhysical(ctx_, profile_id);
        ctx_.target_iq = runtimeCurrentFromPhysical(ctx_, profile_iq);
    }

    if_angle_gen_.setTargetSpeed(target_rpm, 0.0f);

    if_angle_gen_.update(dt_);
    float angle = if_angle_gen_.getAngle();
    float speed_rpm = if_angle_gen_.getSpeedRPM();

    ctx_.angle_elec_command = runtimeAngleFromRadians(angle);
    ctx_.angle_elec = ctx_.angle_elec_command;
    ctx_.speed_rpm  = runtimeSpeedFromPhysical(ctx_, speed_rpm);

    float id_ref = profile_id;
    float iq_ref = profile_iq;
    iq_ref = Motor_FSM::limitIqReferenceByEnergy(
        *this, iq_ref, runtimeSpeedToPhysical(ctx_, ctx_.speed_rpm));

    const float omega_elec = speed_rpm * config_.physical.pole_pairs * (2.0f * 3.14159265f / 60.0f);
#endif

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const FixedNumeric::Ab current_ab{ctx_.i_alpha, ctx_.i_beta};
    const FixedNumeric::Dq current_dq = FixedNumeric::park(current_ab, ctx_.angle_sin_cos);
    ctx_.i_d = current_dq.d;
    ctx_.i_q = current_dq.q;

    refreshFixedCurrentPidOutputLimit();

    const FixedNumeric::Dq voltage_dq{
        controller_.updateFixedCurrentD(
            FixedNumeric::subtractQ15(id_ref, current_dq.d)),
        controller_.updateFixedCurrentQ(
            FixedNumeric::subtractQ15(iq_ref, current_dq.q))};
    ctx_.v_d = voltage_dq.d;
    ctx_.v_q = voltage_dq.q;

    FixedNumeric::Ab voltage_ab = FixedNumeric::inversePark(voltage_dq, ctx_.angle_sin_cos);
    ctx_.v_alpha = voltage_ab.alpha;
    ctx_.v_beta = voltage_ab.beta;

#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    const bool run_smo_shadow =
#if LIB_MOTOR_ENABLE_IF_STARTUP
        (run_phase_ == RunPhase::FORCE_DRAG &&
         active_policy_.steady_source == SteadyAngleSource::SMO) ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
        mode_ == Mode::DEBUG_IF_SMO_OBSERVER ||
#endif
        false;
    if (run_smo_shadow)
    {
        runSMOShadowObserver();
    }
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
    if (mode_ == Mode::DEBUG_IF_HFI_OBSERVER)
    {
        runHFIShadowObserver(true);
    }
#endif

    const FixedNumeric::DutyAbc duty =
        (config_.control.modulation == ModulationMethod::SPWM)
            ? FixedNumeric::spwmVbusNormalized(voltage_ab)
            : FixedNumeric::svpwmVbusNormalized(voltage_ab);
    setPwmDutyQ15(duty);
#else
    float ct = cosf(angle);
    float st = sinf(angle);
    ctx_.i_d =  ctx_.i_alpha * ct + ctx_.i_beta * st;
    ctx_.i_q = -ctx_.i_alpha * st + ctx_.i_beta * ct;
    updateCurrentPidVoltageLimit(omega_elec);

    ctx_.v_d = controller_.pid_d.update(id_ref - ctx_.i_d, dt_);
    ctx_.v_q = controller_.pid_q.update(iq_ref - ctx_.i_q, dt_);
    ctx_.v_alpha = ctx_.v_d * ct - ctx_.v_q * st;
    ctx_.v_beta  = ctx_.v_d * st + ctx_.v_q * ct;

#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    const bool run_smo_shadow =
#if LIB_MOTOR_ENABLE_IF_STARTUP
        (run_phase_ == RunPhase::FORCE_DRAG &&
         active_policy_.steady_source == SteadyAngleSource::SMO) ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
        mode_ == Mode::DEBUG_IF_SMO_OBSERVER ||
#endif
        false;
    if (run_smo_shadow)
    {
        runSMOShadowObserver();
    }
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
    if (mode_ == Mode::DEBUG_IF_HFI_OBSERVER)
    {
        runHFIShadowObserver(true);
    }
#endif

    float duty[3];
    switch (config_.control.modulation)
    {
        case ModulationMethod::SPWM:
            spwm_modulate(ctx_.v_alpha, ctx_.v_beta, ctx_.v_bus, duty);
            break;
        case ModulationMethod::SVPWM:
        default:
            svpwm_modulate(ctx_.v_alpha, ctx_.v_beta, ctx_.v_bus, duty);
            break;
    }
    ctx_.duty_a = duty[0];
    ctx_.duty_b = duty[1];
    ctx_.duty_c = duty[2];
#endif
}
#endif

} // namespace Lib_Motor
