#include "../Manager/Motor_Manager.h"

#include "../FSM/Motor_FSM.h"

#include "../Limit/Motor_TargetLimiter.h"

#include "../../Common/Math/FocMath.h"

#include "../../Control/Modulation_SVPWM.h"
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../Numeric/Motor_FixedNumeric.h"
#endif

#include <cmath>

namespace Lib_Motor
{

void MotorManager::updateCurrentPidVoltageLimit(float omega_elec_rad_s)
{
    (void)omega_elec_rad_s;

    const float k_mod = MotorTargetLimiter::modulationKmod(config_.control.modulation);
    const float vbus = (ctx_.v_bus > 0.0f) ? ctx_.v_bus : 0.0f;
    float v_available = k_mod * vbus;

    if (!std::isfinite(v_available) || v_available < 0.0f)
    {
        v_available = 0.0f;
    }

    const float pid_limit = (v_available > 1.0e-6f) ? v_available : 1.0e-6f;
    controller_.pid_d.setOutputLimit(pid_limit);
    controller_.pid_q.setOutputLimit(pid_limit);
}

void MotorManager::runObserverLoop()
{
    Motor_FSM::runObserverStep(*this);
}

void MotorManager::syncFastRuntimeTargets()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const float current_base = MotorControl::currentBase(config_);
    const float speed_base = MotorControl::speedBase(config_);
    fast_ctx_.current_base_a = current_base;
    fast_ctx_.speed_base_rpm = speed_base;
    fast_ctx_.target_id = FixedNumeric::fromPhysical(ctx_.target_id, current_base);
    fast_ctx_.target_iq = FixedNumeric::fromPhysical(ctx_.target_iq, current_base);
    fast_ctx_.target_speed = FixedNumeric::fromPhysical(ctx_.target_rpm, speed_base);
    fast_ctx_.speed_ref_limited = FixedNumeric::fromPhysical(ctx_.speed_ref_limited, speed_base);
    fast_ctx_.speed_pid_iq = FixedNumeric::fromPhysical(ctx_.speed_pid_iq, current_base);
    fast_ctx_.iq_ref_command = FixedNumeric::fromPhysical(ctx_.iq_ref_command, current_base);
    fast_ctx_.iq_ref_limited = FixedNumeric::fromPhysical(ctx_.iq_ref_limited, current_base);
#else
    fast_ctx_.target_id = ctx_.target_id;
    fast_ctx_.target_iq = ctx_.target_iq;
    fast_ctx_.target_speed = ctx_.target_rpm;
    fast_ctx_.speed_ref_limited = ctx_.speed_ref_limited;
    fast_ctx_.speed_pid_iq = ctx_.speed_pid_iq;
    fast_ctx_.iq_ref_command = ctx_.iq_ref_command;
    fast_ctx_.iq_ref_limited = ctx_.iq_ref_limited;
#endif
}

void MotorManager::syncFastRuntimeInputs(float id_ref, float iq_ref)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const float current_base = MotorControl::currentBase(config_);
    const float voltage_base = (ctx_.v_bus > 0.1f) ? ctx_.v_bus : 0.1f;
    fast_ctx_.current_base_a = current_base;
    fast_ctx_.voltage_base_v = voltage_base;
    fast_ctx_.speed_base_rpm = MotorControl::speedBase(config_);

    fast_ctx_.i_alpha = FixedNumeric::fromPhysical(ctx_.i_alpha, current_base);
    fast_ctx_.i_beta = FixedNumeric::fromPhysical(ctx_.i_beta, current_base);
    fast_ctx_.target_id = FixedNumeric::fromPhysical(id_ref, current_base);
    fast_ctx_.target_iq = FixedNumeric::fromPhysical(iq_ref, current_base);
    fast_ctx_.angle_elec = FixedNumeric::phaseFromRadians(ctx_.angle_elec);
    fast_ctx_.angle_sin_cos = FixedNumeric::sinCos(fast_ctx_.angle_elec);
#else
    fast_ctx_.i_alpha = ctx_.i_alpha;
    fast_ctx_.i_beta = ctx_.i_beta;
    fast_ctx_.target_id = id_ref;
    fast_ctx_.target_iq = iq_ref;
    fast_ctx_.angle_elec = ctx_.angle_elec;
    foc_sin_cos_f32(fast_ctx_.angle_elec,
                    fast_ctx_.sin_angle_elec,
                    fast_ctx_.cos_angle_elec);
#endif
}

void MotorManager::publishFastRuntimeOutputs()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.i_d = FixedNumeric::toPhysical(fast_ctx_.i_d, fast_ctx_.current_base_a);
    ctx_.i_q = FixedNumeric::toPhysical(fast_ctx_.i_q, fast_ctx_.current_base_a);
    ctx_.v_d = FixedNumeric::toPhysical(fast_ctx_.v_d, fast_ctx_.voltage_base_v);
    ctx_.v_q = FixedNumeric::toPhysical(fast_ctx_.v_q, fast_ctx_.voltage_base_v);
    ctx_.v_alpha = FixedNumeric::toPhysical(fast_ctx_.v_alpha, fast_ctx_.voltage_base_v);
    ctx_.v_beta = FixedNumeric::toPhysical(fast_ctx_.v_beta, fast_ctx_.voltage_base_v);
    ctx_.duty_a = FixedNumeric::toNormalized(fast_ctx_.duty_a);
    ctx_.duty_b = FixedNumeric::toNormalized(fast_ctx_.duty_b);
    ctx_.duty_c = FixedNumeric::toNormalized(fast_ctx_.duty_c);
#else
    ctx_.i_d = fast_ctx_.i_d;
    ctx_.i_q = fast_ctx_.i_q;
    ctx_.v_d = fast_ctx_.v_d;
    ctx_.v_q = fast_ctx_.v_q;
    ctx_.v_alpha = fast_ctx_.v_alpha;
    ctx_.v_beta = fast_ctx_.v_beta;
    ctx_.duty_a = fast_ctx_.duty_a;
    ctx_.duty_b = fast_ctx_.duty_b;
    ctx_.duty_c = fast_ctx_.duty_c;
#endif
}

void MotorManager::clearControlTargets()
{
    /*
     * 清除所有控制目标值: 清零 target_rpm/target_iq/target_id, 类似 stop 时的复位
     * 通常在 clearFaultAndStop 或 start 后重新进入时调用
     */
    ctx_.target_rpm = 0.0f;
    ctx_.target_iq = 0.0f;
    ctx_.target_id = 0.0f;
    ctx_.pending_reverse_rpm = 0.0f;
    clearDerivedTargets();
    syncFastRuntimeTargets();
}

void MotorManager::clearDerivedTargets()
{
    /*
     * 清除衍生控制目标 (速度环 PID 中间量, Iq 斜坡限制器状态)
     * 通常在模式切换或停止时调用, 清零 intermediate 值
     */
    ctx_.speed_ref_limited = 0.0f;
    ctx_.speed_pid_iq = 0.0f;
ctx_.iq_ref_command = 0.0f;
    ctx_.iq_ref_limited = 0.0f;
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
    ctx_.target_pos_rad = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
    ctx_.vel_ff_rad_s = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_TORQUE_FEEDFORWARD
    ctx_.torque_ff_a = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
    ctx_.imp_pos_ref_rad    = 0.0f;
    ctx_.imp_stiffness_n_m  = 0.0f;
    ctx_.imp_damping_n_m_s  = 0.0f;
    ctx_.imp_torque_bias_a  = 0.0f;
    ctx_.imp_torque_limit_a = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_HFI
    ctx_.hfi_injection = HfiInjectionCommand();
#endif
    syncFastRuntimeTargets();
}


/*
 * computeIqReference -- 计算 Iq 电流参考值 (三层级联)
 *
 * 三层级联:
 *   1. 模式计算:
 *      VELOCITY_CONTROL -> 速度 PID 输出 Iq (用户设置目标转速)
 *      TORQUE_CONTROL   -> 直接使用 ctx_.target_iq (由 API 设置)
 *      其他              -> iq_ref = 0 (安全关闭)
 *
 *   2. 限幅:
 *      MotorTargetLimiter::limitIqReference()
 *        (电流硬限 + 速度弱磁 + 功率限制)
 *
 *   3. 斜坡:
 *      MotorTargetLimiter::applyIqSlewRate()
 *        (每 tick 斜率限制, 防止电流突变)
 *
 *   4. 堵转保护 (速度环模式下)
 *
 * 输出: ctx_.iq_ref_command / ctx_.iq_ref_limited 用于调试监控
 */
float MotorManager::computeIqReference()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    const float speed_pid_output_limit = MotorTargetLimiter::configuredIqLimit(config_);
#endif
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    const float iq_limit = MotorTargetLimiter::configuredIqLimit(config_);  // 最大 Iq 限幅 (从 motion/motor/hard_limit 读取)
#endif

    if (mode_ == Mode::VELOCITY_CONTROL)
    {
        // [A] 速度参考值限幅: 对目标转速施加加速度斜坡限制

        // [A.0] 跨零软减速: 若 pending_reverse_rpm 待执行 & 实测速度已落到 zero_band 内, 提交目标
        if (ctx_.pending_reverse_rpm != 0.0f &&
            fabsf(ctx_.speed_rpm) <= config_.motion.reverse_zero_band_rpm)
        {
            ctx_.target_rpm = ctx_.pending_reverse_rpm;
            ctx_.pending_reverse_rpm = 0.0f;
        }

        ctx_.speed_ref_limited = MotorTargetLimiter::limitSpeedReference(
            config_, dt_, ctx_.speed_ref_limited, ctx_.target_rpm);
        const float speed_error = ctx_.speed_ref_limited - ctx_.speed_rpm;

        // [B] 堵转保持: 堵转状态主动衰减积分项, 防止积分饱和导致恢复时电流冲击
        const bool stall_hold = (event_.motor_stalled != 0U);
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        if (stall_hold)
        {
            controller_.pid_speed.decayIntegral(config_.motion.stall.speed_integral_decay);
        }
#endif
        // [C] 速度 PID 输出 Iq 参考值
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        controller_.syncFixedSpeedPid(config_, dt_, speed_pid_output_limit);
        ctx_.speed_pid_iq = controller_.updateFixedSpeed(config_, speed_error, stall_hold);
#else
        ctx_.speed_pid_iq = controller_.pid_speed.update(speed_error, dt_, stall_hold);
#endif
        ctx_.iq_ref_command = ctx_.speed_pid_iq;
    }
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
    else if (mode_ == Mode::POSITION_CONTROL)
    {
        // [P1] 位置环: pos_err PID 输出 speed_ref, 叠加速度前馈 vel_ff
        /* TODO HostTest: 位置环方向校正 (pos_ref > θ 时 iq_ref 应为正)、vel_ff 叠加方向、
         *                位置阶跃收敛方向均未编单测, 当前依赖 VOFA 真硬件验证。
         *                调试 PID 前先确认 pos_err 与 iq_ref 极性一致, 方向反了电机会跑飞。 */
        const float pos_err = ctx_.target_pos_rad - ctx_.angle_mech_sensor;
        float speed_ref = controller_.pid_position.update(pos_err, dt_);
#if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
        if (config_.control.feedforward.enable_vel_ff)
        {
            speed_ref += config_.control.feedforward.vel_ff_gain * ctx_.vel_ff_rad_s;
        }
#endif
        // [P2] 速度参考限幅 (沿用速度环限幅器, 防 Iq 突变)
        ctx_.speed_ref_limited = MotorTargetLimiter::limitSpeedReference(
            config_, dt_, ctx_.speed_ref_limited, speed_ref);
        const float speed_error = ctx_.speed_ref_limited - ctx_.speed_rpm;

        // [P3] 堵转保持 (与速度环同语义)
        const bool stall_hold = (event_.motor_stalled != 0U);
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        if (stall_hold)
        {
            controller_.pid_speed.decayIntegral(config_.motion.stall.speed_integral_decay);
        }
#endif
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        controller_.syncFixedSpeedPid(config_, dt_, speed_pid_output_limit);
        ctx_.speed_pid_iq = controller_.updateFixedSpeed(config_, speed_error, stall_hold);
#else
        ctx_.speed_pid_iq = controller_.pid_speed.update(speed_error, dt_, stall_hold);
#endif
        ctx_.iq_ref_command = ctx_.speed_pid_iq;

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        // 位置环下也清堵转候选计数, 防止位置静止时被误判为堵转
        event_.motor_stalled = 0U;
        stall_candidate_ticks_ = 0U;
        stall_release_ticks_ = 0U;
#endif
    }
#endif
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
    else if (mode_ == Mode::IMPEDANCE_CONTROL)
    {
        /* Phase 3 接通点: 阻抗律
         *   iq_ref = K*(pos_ref-θ) - D*(-ω) + torque_bias + torque_ff
         *   iq_ref = clamp(iq_ref, ±torque_limit)
         * Phase 2 仅占位: BuildCfg 启用 IMPEDANCE 时本段被编译,
         * 启动入口被 supportsMode 拒, 实际不会执行到此处。 */
        ctx_.speed_ref_limited = 0.0f;
        ctx_.speed_pid_iq = 0.0f;
        ctx_.iq_ref_command = 0.0f;
    }
#endif
    else
    {
        // 扭矩/其他模式: Iq 参考值直接使用用户设置的 target_iq
        ctx_.speed_ref_limited = 0.0f;
        ctx_.speed_pid_iq = 0.0f;
        ctx_.iq_ref_command = ctx_.target_iq;
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        // 非速度环模式下清除堵转状态 (堵转保护仅在速度环下生效)
        if (mode_ != Mode::VELOCITY_CONTROL)
        {
            event_.motor_stalled = 0U;
            stall_candidate_ticks_ = 0U;
            stall_release_ticks_ = 0U;
        }
#endif
    }

    // [D] Iq 限幅: 电流硬限 + 速度弱磁 + 功率限制
    float iq_ref = MotorTargetLimiter::limitIqReference(
        config_, mode_, ctx_.speed_rpm, ctx_.iq_ref_command);
    iq_ref = Motor_FSM::limitIqReferenceByEnergy(*this, iq_ref, ctx_.speed_rpm);
    // [E] Iq 斜率限制: 每 tick 限制变化幅度, 防止电流突变
    iq_ref = MotorTargetLimiter::applyIqSlewRate(
        config_, dt_, ctx_.iq_ref_limited, iq_ref);
    ctx_.iq_ref_limited = iq_ref;
    syncFastRuntimeTargets();

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    /* 位置环下也跑速度环, 故 stall 检测也生效; IMPEDANCE 不跑速度环不参与 */
    if (mode_ == Mode::VELOCITY_CONTROL ||
        mode_ == Mode::POSITION_CONTROL)
    {
        updateStallProtection(ctx_.speed_ref_limited - ctx_.speed_rpm,
                              ctx_.iq_ref_limited,
                              iq_limit);
    }
#endif

    return ctx_.iq_ref_limited;
}

void MotorManager::runControlLoop()
{
    float id_ref = ctx_.target_id;
    float iq_ref = computeIqReference();
    syncFastRuntimeInputs(id_ref, iq_ref);

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    {
    const float current_base = fast_ctx_.current_base_a;
    const float vbus = fast_ctx_.voltage_base_v;
    const FixedNumeric::Ab current_ab{
        fast_ctx_.i_alpha,
        fast_ctx_.i_beta};
    const FixedNumeric::Dq current_dq = FixedNumeric::park(current_ab, fast_ctx_.angle_sin_cos);
    fast_ctx_.i_d = current_dq.d;
    fast_ctx_.i_q = current_dq.q;
    ctx_.i_d = FixedNumeric::toPhysical(fast_ctx_.i_d, current_base);
    ctx_.i_q = FixedNumeric::toPhysical(fast_ctx_.i_q, current_base);

    const float omega_elec = ctx_.speed_rpm * config_.physical.pole_pairs * (2.0f * 3.14159265f / 60.0f);
    updateCurrentPidVoltageLimit(omega_elec);
    controller_.syncFixedCurrentPid(config_, dt_, vbus);

    FixedNumeric::Dq voltage_dq{
        controller_.updateFixedCurrentD(
            FixedNumeric::subtractQ15(fast_ctx_.target_id, current_dq.d)),
        controller_.updateFixedCurrentQ(
            FixedNumeric::subtractQ15(fast_ctx_.target_iq, current_dq.q))};
    fast_ctx_.v_d = voltage_dq.d;
    fast_ctx_.v_q = voltage_dq.q;
    ctx_.v_d = FixedNumeric::toPhysical(fast_ctx_.v_d, vbus);
    ctx_.v_q = FixedNumeric::toPhysical(fast_ctx_.v_q, vbus);
#if LIB_MOTOR_ENABLE_HFI
    if (ctx_.hfi_injection.enabled)
    {
        ctx_.v_d += ctx_.hfi_injection.v_d;
        ctx_.v_q += ctx_.hfi_injection.v_q;
    }
#endif

    const float v_q_pre_clamp = ctx_.v_q;
    MotorTargetLimiter::limitVoltageVector(config_, ctx_.v_bus, omega_elec,
                                            ctx_.v_d, ctx_.v_q);

#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL || LIB_MOTOR_ENABLE_POSITION_CONTROL
    {
        const float vq_mag_pre = fabsf(v_q_pre_clamp);
        const float vq_mag_post = fabsf(ctx_.v_q);
        const bool voltage_saturated =
            (vq_mag_pre > 1e-3f) &&
            (vq_mag_post < vq_mag_pre * 0.95f);
        if (voltage_saturated)
        {
            controller_.decayFixedSpeedIntegral(0.95f);
        }
    }
#endif

    voltage_dq = FixedNumeric::Dq{
        FixedNumeric::fromPhysical(ctx_.v_d, vbus),
        FixedNumeric::fromPhysical(ctx_.v_q, vbus)};
    fast_ctx_.v_d = voltage_dq.d;
    fast_ctx_.v_q = voltage_dq.q;
    const FixedNumeric::Ab voltage_ab = FixedNumeric::inversePark(voltage_dq, fast_ctx_.angle_sin_cos);
    fast_ctx_.v_alpha = voltage_ab.alpha;
    fast_ctx_.v_beta = voltage_ab.beta;

    const FixedNumeric::DutyAbc duty = FixedNumeric::svpwmVbusNormalized(voltage_ab);
    fast_ctx_.duty_a = duty.a;
    fast_ctx_.duty_b = duty.b;
    fast_ctx_.duty_c = duty.c;
    publishFastRuntimeOutputs();
    }
#else

    /* ================================================================
     * [1] 预算 sin/cos: Park 与 InvPark 共用同一电角度 angle_elec,
     *     只调用一次联合接口, 消除原先 6 次三角调用 → 2 次。
     *     (中间 [3] 段 PID/HFI 注入只改 v_d/v_q, 不改角度, 故可复用)
     * ================================================================ */
    const float sin_ae = fast_ctx_.sin_angle_elec;
    const float cos_ae = fast_ctx_.cos_angle_elec;

    /* ================================================================
     * [2] Park 变换: alpha/beta (两相) -> dq (旋转)
     *
     *   i_d =  i_alpha * cos(theta) + i_beta * sin(theta)
     *   i_q = -i_alpha * sin(theta) + i_beta * cos(theta)
     *
     * theta = ctx_.angle_elec (电角度, 来自观测器/SMO/传感器)
     * i_alpha/beta = ctx_.i_alpha/beta (来自 updateSensorMeasurements Clarke 变换)
     * ================================================================ */
    park_transform_sc(fast_ctx_.i_alpha, fast_ctx_.i_beta, sin_ae, cos_ae,
                      fast_ctx_.i_d, fast_ctx_.i_q);
    const float omega_elec = ctx_.speed_rpm * config_.physical.pole_pairs * (2.0f * 3.14159265f / 60.0f);
    updateCurrentPidVoltageLimit(omega_elec);

    /* ================================================================
     * [3] d/q 电流 PID 控制
     *
     *   v_d = PID_d(id_ref - i_d)  -- D 轴, id_ref = 0 (Id=0 控制)
     *   v_q = PID_q(iq_ref - i_q)  -- Q 轴, iq_ref 来自速度环/扭矩命令
     *
     * PID 控制器内部已含积分限幅, output_limit 防止饱和
     * ================================================================ */
    fast_ctx_.v_d = controller_.pid_d.update(fast_ctx_.target_id - fast_ctx_.i_d, dt_);
    fast_ctx_.v_q = controller_.pid_q.update(fast_ctx_.target_iq - fast_ctx_.i_q, dt_);
#if LIB_MOTOR_ENABLE_HFI
    if (ctx_.hfi_injection.enabled)
    {
        fast_ctx_.v_d += ctx_.hfi_injection.v_d;
        fast_ctx_.v_q += ctx_.hfi_injection.v_q;
    }
#endif

    /* [3.5] 联合电压矢量圆限幅: √(vd²+vq²) ≤ K_mod × Vbus
     * 替代旧的固定 ±10V 方限制, 解决宽 VBUS 失配问题;
     * 饱和前留一份 v_q_pre_clamp 用于饱和回算;
     * Ke 不在此处扣减; 当前未接入 ωKe 电压前馈, 仅做实时 Vbus 圆限幅。
     */
    const float v_q_pre_clamp = fast_ctx_.v_q;
    MotorTargetLimiter::limitVoltageVector(config_, ctx_.v_bus, omega_elec,
                                            fast_ctx_.v_d, fast_ctx_.v_q);

    /* [3.6] 电流环饱和回算给速度环 (anti-windup 一致性):
     * 当 vq 被 limitVoltageVector 显著缩放 (电压饱和) 或电流环 PID 自身 isSaturated() 时,
     * 把速度环积分衰减一次, 防止速度环继续推 Iq 而物理上无法实现 → 漂移。
     */
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL || LIB_MOTOR_ENABLE_POSITION_CONTROL
    {
        const float vq_mag_pre = fabsf(v_q_pre_clamp);
        const float vq_mag_post = fabsf(fast_ctx_.v_q);
        const bool voltage_saturated =
            (vq_mag_pre > 1e-3f) &&
            (vq_mag_post < vq_mag_pre * 0.95f);
        const bool pid_saturated = controller_.pid_q.isSaturated();
        if (voltage_saturated || pid_saturated)
        {
            // 电压饱和: 衰减速度环积分 5% 每 tick, 不一次性清零;
            // 用经验值 0.95 (本轮回算系数固定, P2 阶段可暴露为 cfg)
            controller_.pid_speed.decayIntegral(0.95f);
        }
    }
#endif

    /* ================================================================
     * [4] InvPark 变换: dq (旋转) -> alpha/beta (两相)
     *
     *   v_alpha = v_d * cos(theta) - v_q * sin(theta)
     *   v_beta  = v_d * sin(theta) + v_q * cos(theta)
     *
     * 复用 [1] 预算的 sin_ae/cos_ae, 不再重复调用三角函数
     * ================================================================ */
    inv_park_transform_sc(fast_ctx_.v_d, fast_ctx_.v_q, sin_ae, cos_ae,
                          fast_ctx_.v_alpha, fast_ctx_.v_beta);

    /* ================================================================
     * [5] 调制: alpha/beta 电压 -> 三相占空比
     *
     * SPWM: 正弦调制, 电压利用率 1.0
     *   公式: duty = 0.5 + V_phase / Vbus
     *
     * SVPWM: 空间矢量调制, 电压利用率 2/sqrt(3) ~ 1.1547
     *   将 V_alpha/beta 映射为 8 种开关状态 (000~111), 计算占空比
     *   比 SPWM 提高 15.47% 电压利用率
     * ================================================================ */
    switch (config_.control.modulation)
    {
        case ModulationMethod::SPWM:
        {
            float duty[3];
            spwm_modulate(fast_ctx_.v_alpha, fast_ctx_.v_beta, ctx_.v_bus, duty);
            fast_ctx_.duty_a = duty[0];
            fast_ctx_.duty_b = duty[1];
            fast_ctx_.duty_c = duty[2];
        }
        break;

        case ModulationMethod::SVPWM:
        default:
        {
            float duty[3];
            svpwm_modulate(fast_ctx_.v_alpha, fast_ctx_.v_beta, ctx_.v_bus, duty);
            fast_ctx_.duty_a = duty[0];
            fast_ctx_.duty_b = duty[1];
            fast_ctx_.duty_c = duty[2];
        }
        break;
    }
    publishFastRuntimeOutputs();
#endif
}

} // namespace Lib_Motor
