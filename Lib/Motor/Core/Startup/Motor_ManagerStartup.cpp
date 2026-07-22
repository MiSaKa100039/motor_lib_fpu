#include "../Manager/Motor_Manager.h"

#include "../FSM/Motor_FSM.h"

#include "../../Common/Math/FocMath.h"

#include "../../Control/Modulation_SVPWM.h"

#include <cmath>

namespace Lib_Motor
{

/* setDragCurrent -- 设置 IF 强拖电流 (A), >0 覆盖 config 默认值 */
#if LIB_MOTOR_ENABLE_IF_STARTUP
void MotorManager::setDragCurrent(float A)
{
    drag_current_override_ = A;
}

/* getEffectiveDragCurrent -- 返回有效的拖拽电流值 */
float MotorManager::getEffectiveDragCurrent() const
{
    if (drag_current_override_ > 0.0f) return drag_current_override_;
    return config_.observer.drag_current_a;
}

void MotorManager::resetIFStartupProfileState()
{
    ctx_.if_profile_phase = 0U;
    ctx_.if_profile_phase_time_s = 0.0f;
    ctx_.if_profile_start_speed_rpm = 0.0f;
    ctx_.if_profile_start_id = 0.0f;
    ctx_.if_profile_start_iq = 0.0f;
    ctx_.if_profile_phase_started = false;
}

bool MotorManager::getIFStartupProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref)
{
    const MotorIFStartupProfile* profile = config_.observer.if_startup_profile;
    if (profile == nullptr || profile->phases == nullptr || profile->phase_count == 0U)
    {
        return false;
    }

    if (ctx_.if_profile_phase >= profile->phase_count)
    {
        ctx_.if_profile_phase = (uint8_t)(profile->phase_count - 1U);
    }

    const volatile MotorIFStartupPhase& phase = profile->phases[ctx_.if_profile_phase];
    float duration_s = phase.duration_s;
    const float final_speed_rpm = phase.final_speed_rpm;
    const float final_id_a = phase.final_id_a;
    const float final_iq_a = phase.final_iq_a;

    if (!ctx_.if_profile_phase_started)
    {
        ctx_.if_profile_phase_started = true;
        ctx_.if_profile_phase_time_s = 0.0f;
    }

    if (duration_s < dt_) duration_s = dt_;

    const float phase_time_s = ctx_.if_profile_phase_time_s + dt_;
    float ratio = phase_time_s / duration_s;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < 0.0f) ratio = 0.0f;

    speed_rpm = ctx_.if_profile_start_speed_rpm +
                (final_speed_rpm - ctx_.if_profile_start_speed_rpm) * ratio;
    id_ref = ctx_.if_profile_start_id +
             (final_id_a - ctx_.if_profile_start_id) * ratio;
    iq_ref = ctx_.if_profile_start_iq +
             (final_iq_a - ctx_.if_profile_start_iq) * ratio;

    ctx_.if_profile_phase_time_s = phase_time_s;
    if (phase_time_s >= duration_s)
    {
        if ((ctx_.if_profile_phase + 1U) < profile->phase_count)
        {
            ctx_.if_profile_phase++;
            ctx_.if_profile_phase_time_s = 0.0f;
            ctx_.if_profile_start_speed_rpm = final_speed_rpm;
            ctx_.if_profile_start_id = final_id_a;
            ctx_.if_profile_start_iq = final_iq_a;
            ctx_.if_profile_phase_started = true;
        }
        else
        {
            ctx_.if_profile_phase_time_s = duration_s;
            ctx_.if_profile_start_speed_rpm = final_speed_rpm;
            ctx_.if_profile_start_id = final_id_a;
            ctx_.if_profile_start_iq = final_iq_a;
        }
    }

    return true;
}
#endif

#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
void MotorManager::resetDebugProfileState()
{
    ctx_.debug_profile_phase = 0U;
    ctx_.debug_profile_phase_time_s = 0.0f;
    ctx_.debug_profile_start_speed_rpm = 0.0f;
    ctx_.debug_profile_start_duty = 0.0f;
#if LIB_MOTOR_ENABLE_IF_STARTUP
    ctx_.debug_profile_start_id = 0.0f;
    ctx_.debug_profile_start_iq = 0.0f;
#endif
    ctx_.debug_profile_phase_started = false;
    if_angle_gen_.reset();
}

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

#if LIB_MOTOR_ENABLE_IF_STARTUP
bool MotorManager::getDebugIFProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref)
{
    const MotorIFStartupProfile* profile = ctx_.debug_if_profile;
    if (profile == nullptr || profile->phases == nullptr || profile->phase_count == 0U)
    {
        return false;
    }

    if (ctx_.debug_profile_phase >= profile->phase_count)
    {
        ctx_.debug_profile_phase = (uint8_t)(profile->phase_count - 1U);
    }

    const volatile MotorIFStartupPhase& phase = profile->phases[ctx_.debug_profile_phase];
    float duration_s = phase.duration_s;
    float final_speed_rpm = phase.final_speed_rpm;
    float final_id_a = phase.final_id_a;
    float final_iq_a = phase.final_iq_a;
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
    id_ref = ctx_.debug_profile_start_id +
             (final_id_a - ctx_.debug_profile_start_id) * ratio;
    iq_ref = ctx_.debug_profile_start_iq +
             (final_iq_a - ctx_.debug_profile_start_iq) * ratio;

    ctx_.debug_profile_phase_time_s = phase_time_s;
    if (phase_time_s >= duration_s)
    {
        if ((ctx_.debug_profile_phase + 1U) < profile->phase_count)
        {
            ctx_.debug_profile_phase++;
            ctx_.debug_profile_phase_time_s = 0.0f;
            ctx_.debug_profile_start_speed_rpm = final_speed_rpm;
            ctx_.debug_profile_start_id = final_id_a;
            ctx_.debug_profile_start_iq = final_iq_a;
            ctx_.debug_profile_phase_started = true;
        }
        else
        {
            ctx_.debug_profile_phase_time_s = duration_s;
            ctx_.debug_profile_start_speed_rpm = final_speed_rpm;
            ctx_.debug_profile_start_id = final_id_a;
            ctx_.debug_profile_start_iq = final_iq_a;
        }
    }

    return true;
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
 * 注意: ALIGNMENT 模式下 align_current_a 由配置指定
 */
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
    ctx_.angle_elec_command = 0.0f;
    ctx_.angle_elec = ctx_.angle_elec_command;

    float id_ref = ctx_.target_id;
    float iq_ref = ctx_.target_iq;

    // 对齐模式且 Id/Iq 为 0 时: 使用配置的预对齐电流注入 d 轴
    // (调试直驱电流锁定/校准模式的 Id/Iq 则由用户指定)
    if (run_phase_ == RunPhase::ALIGNMENT &&
        fabsf(id_ref) < 1e-6f && fabsf(iq_ref) < 1e-6f) {
        id_ref = config_.observer.align_current_a;
        iq_ref = 0.0f;
    }
    iq_ref = Motor_FSM::limitIqReferenceByEnergy(*this, iq_ref, ctx_.speed_rpm);

    /* Park 变换简化 (theta=0 度):
     *   i_d =  i_alpha * cos(0) + i_beta * sin(0) = i_alpha
     *   i_q = -i_alpha * sin(0) + i_beta * cos(0) = i_beta
     */
    ctx_.i_d = ctx_.i_alpha * 1.0f + ctx_.i_beta * 0.0f;
    ctx_.i_q = -ctx_.i_alpha * 0.0f + ctx_.i_beta * 1.0f;

#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    const bool debug_current_lock_idle =
        mode_ == Mode::DEBUG_CURRENT_LOCK &&
        run_phase_ == RunPhase::RUN_DIRECT &&
        fabsf(id_ref) < 1.0e-6f &&
        fabsf(iq_ref) < 1.0e-6f;
#else
    const bool debug_current_lock_idle = false;
#endif

    if (debug_current_lock_idle)
    {
        controller_.pid_d.reset();
        controller_.pid_q.reset();
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
}

/*
 * runIFControl -- I/F 强拖电流闭环控制
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
 * 用途: FORCE_DRAG 启动阶段, I/F 拖动至 SMO 收敛后切速度闭环
 */
#if LIB_MOTOR_ENABLE_IF_STARTUP
void MotorManager::runIFControl()
{
    /* ================================================================
     * [1] 开环角度生成 (同 VF 使用 if_angle_gen_)
     *
     * 注意: if_angle_gen_ 在 init() 初始化后不再 reset,
     *       ALIGNMENT 完成后由 FSM 切换 RunPhase 时调用 if_angle_gen_.reset()
     *       (具体见 Motor_FSM::handleStop -> pwm_enable 逻辑)
     * ================================================================ */
    float target_rpm = ctx_.target_rpm;
    float profile_id = ctx_.target_id;
    float profile_iq = ctx_.target_iq;
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    bool use_profile =
        ((mode_ == Mode::DEBUG_IF_DRAG ||
#if LIB_MOTOR_ENABLE_SMO
          mode_ == Mode::DEBUG_IF_SMO_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_HFI
          mode_ == Mode::DEBUG_IF_HFI_OBSERVER ||
#endif
          false) &&
         run_phase_ == RunPhase::RUN_DIRECT) &&
        getDebugIFProfileCommand(target_rpm, profile_id, profile_iq);
#else
    bool use_profile = false;
#endif
    // Normal FORCE_DRAG uses the staged startup profile when one is configured.
    if (!use_profile && run_phase_ == RunPhase::FORCE_DRAG)
    {
        use_profile = getIFStartupProfileCommand(target_rpm, profile_id, profile_iq);
    }

    ctx_.target_rpm = target_rpm;
    ctx_.target_id = profile_id;
    ctx_.target_iq = profile_iq;

    /*
     * 斜坡语义拆分:
     *   - 正常 FORCE_DRAG 只使用 drag_accel_rpm_s
     *   - 调试 RUN_DIRECT 才允许使用 debug_ramp_time_s / profile
     */
    float ramp_s = 0.0f;

    if (!use_profile &&
        run_phase_ == RunPhase::FORCE_DRAG &&
        config_.observer.drag_accel_rpm_s > 0.0f)
    {
        ramp_s = fabsf(target_rpm) / config_.observer.drag_accel_rpm_s;
    }

#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    if (run_phase_ == RunPhase::RUN_DIRECT)
    {
        ramp_s = use_profile ? 0.0f : ctx_.debug_ramp_time_s;
    }
#endif
    if_angle_gen_.setTargetSpeed(target_rpm, ramp_s);

    if_angle_gen_.update(dt_);
    float angle = if_angle_gen_.getAngle();
    float speed_rpm = if_angle_gen_.getSpeedRPM();

    ctx_.angle_elec_command = angle;
    ctx_.angle_elec = ctx_.angle_elec_command;
    ctx_.speed_rpm  = speed_rpm;

    float id_ref = ctx_.target_id;
    // 从 target_iq 获取 Iq 参考值, 无效时使用拖拽电流
    float iq_ref = ctx_.target_iq;
    if (!use_profile && iq_ref <= 0.0f) {
        iq_ref = getEffectiveDragCurrent();
    }
    iq_ref = Motor_FSM::limitIqReferenceByEnergy(*this, iq_ref, ctx_.speed_rpm);

    /* ================================================================
     * [2] 电流读取 + Park 变换: alpha/beta -> dq (旋转坐标系)
     *
     * Clarke 变换在 updateSensorMeasurements[6] 已完成
     * i_alpha = ctx_.i_a, i_beta = (ia + 2*ib)/sqrt(3)
     * ================================================================ */
    float ct = cosf(angle);
    float st = sinf(angle);
    // ctx_.i_alpha / ctx_.i_beta 在 updateSensorMeasurements 中已更新

    /* ================================================================
     * [3] Park 变换: alpha/beta -> dq (旋转坐标系)
     * ================================================================ */
    ctx_.i_d =  ctx_.i_alpha * ct + ctx_.i_beta * st;
    ctx_.i_q = -ctx_.i_alpha * st + ctx_.i_beta * ct;
    const float omega_elec = speed_rpm * config_.physical.pole_pairs * (2.0f * 3.14159265f / 60.0f);
    updateCurrentPidVoltageLimit(omega_elec);

    /* ================================================================
     * [4] d/q 电流 PID
     *    与 FOC 相同, 电流闭环 PID 控制, Id/Iq 分别独立
     * ================================================================ */
    ctx_.v_d = controller_.pid_d.update(id_ref - ctx_.i_d, dt_);
    ctx_.v_q = controller_.pid_q.update(iq_ref - ctx_.i_q, dt_);

    /* ================================================================
     * [5] InvPark 变换 + 调制 -> 三相占空比
     * ================================================================ */
    ctx_.v_alpha = ctx_.v_d * ct - ctx_.v_q * st;
    ctx_.v_beta  = ctx_.v_d * st + ctx_.v_q * ct;

#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
#if LIB_MOTOR_ENABLE_SMO
    if (mode_ == Mode::DEBUG_IF_SMO_OBSERVER)
    {
        runSMOShadowObserver();
    }
#endif
#if LIB_MOTOR_ENABLE_HFI
    if (mode_ == Mode::DEBUG_IF_HFI_OBSERVER)
    {
        runHFIShadowObserver(true);
    }
#endif
#endif

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
}
#endif

} // namespace Lib_Motor
