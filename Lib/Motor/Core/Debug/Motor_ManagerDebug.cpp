#include "../Manager/Motor_Manager.h"

#include "../../Common/Math/FocMath.h"

#include "../../Control/Modulation_SVPWM.h"

#include <cmath>

namespace Lib_Motor
{


/*
 * runPWMManual -- 手动 PWM 控制 (调试用)
 *
 * 将 debugPWMManual(u,v,w) 设置的原始值直接作为占空比输出,
 * 绕过所有 FOC 计算。
 *
 * 安全: PWM 输出期间 ADC 和保护仍在运行
 * 警告: 无电流闭环, 需谨慎设置电压以防过流
 */
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
void MotorManager::runPWMManual()
{
    ctx_.duty_a = ctx_.test_u;
    ctx_.duty_b = ctx_.test_v;
    ctx_.duty_c = ctx_.test_w;
}

/*
 * runVFControl -- V/F 开环电压拖动
 *
 * 电压频率开环控制: 给定 V/F 特性, 生成旋转电压矢量。
 *
 * 原理:
 *   AngleGenerator 生成开环角度 (速度积分)
 *   duty 由配置的 V/F 偏置值决定, 以 0.5 为中心输出 PWM
 *
 * 注意: 无电流闭环, 适用于轻载/空载调试场景。
 *       变频/反电势变化可能导致电流不可控。
 *       建议观测器就绪后切换至速度闭环。
 *
 * 输入:
 *   target_rpm      -- 目标转速 (来自 setTargetSpeed)
 *   vf_duty_bias    -- 电压偏置占空比 (来自 setVFDutyBias)
 *   debug_ramp_time_s -- 软斜坡时间 (来自 debugVFControl)
 */
void MotorManager::runVFControl()
{
    /* ================================================================
     * [1] 开环角度生成 (速度斜坡 + 积分)
     *
     * if_angle_gen_ 与 MotorManager 共享
     *   - 每 tick 状态更新 (current_rpm / angle 保持)
     *   - init() 初始化参数 (control_freq_hz), 零时重置角度
     *   - setTargetSpeed -> 设置 ramp_rate
     *   - update(dt) -> 速度斜坡 + 积分
     * ================================================================ */
    float target_rpm = ctx_.target_rpm;
    float duty = ctx_.vf_duty_bias;
    bool use_profile =
        (mode_ == Mode::DEBUG_VF_DRAG && run_phase_ == RunPhase::RUN_DIRECT) &&
        getDebugVFProfileCommand(target_rpm, duty);

    ctx_.target_rpm = target_rpm;
    ctx_.vf_duty_bias = duty;

    float ramp_s = use_profile ? 0.0f : ctx_.debug_ramp_time_s;
    if_angle_gen_.setTargetSpeed(target_rpm, ramp_s);

    if_angle_gen_.update(dt_);
    float angle = if_angle_gen_.getAngle();
    float speed_rpm = if_angle_gen_.getSpeedRPM();

    /* ================================================================
     * [2] 电压矢量生成
     *     vf_duty_bias 由配置指定占空比偏置, clamp 至 [0, 1] 防止溢出
     * ================================================================ */
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    float ct = cosf(angle);
    float st = sinf(angle);
    float v_bus = ctx_.v_bus;
    if (v_bus < 0.1f) v_bus = 0.1f;

    ctx_.angle_elec_command = angle;
    ctx_.angle_elec = ctx_.angle_elec_command;
    ctx_.speed_rpm  = speed_rpm;
    ctx_.v_alpha = duty * v_bus * ct;
    ctx_.v_beta  = duty * v_bus * st;

    /* ================================================================
     * [3] 调制: duty 由三相正弦计算 -> SVPWM 中心化
     *     duty=0.03 时实际输出偏置 0.5 后接近 50% 占空比
     * ================================================================ */
    constexpr float SQRT3_2 = 0.8660254037844386f;

    float u = duty * ct;
    float v = duty * (-0.5f * ct + SQRT3_2 * st);
    float w = duty * (-0.5f * ct - SQRT3_2 * st);

    switch (config_.control.modulation)
    {
        case ModulationMethod::SVPWM:
        {
            float u_max = u; if (v > u_max) u_max = v; if (w > u_max) u_max = w;
            float u_min = u; if (v < u_min) u_min = v; if (w < u_min) u_min = w;
            float offset = -(u_max + u_min) * 0.5f;
            u += offset;
            v += offset;
            w += offset;
        }
        break;
        case ModulationMethod::SPWM:
        default:
            break;
    }

    ctx_.duty_a = 0.5f + u;
    ctx_.duty_b = 0.5f + v;
    ctx_.duty_c = 0.5f + w;
}

#if LIB_MOTOR_ENABLE_HFI
void MotorManager::runHFIObserverTest()
{
    const float theta_in = ctx_.hfi_estimate.angle_rad;
    const float ct_in = cosf(theta_in);
    const float st_in = sinf(theta_in);
    const float i_d_hfi = ctx_.i_alpha_raw * ct_in + ctx_.i_beta_raw * st_in;
    const float i_q_hfi = -ctx_.i_alpha_raw * st_in + ctx_.i_beta_raw * ct_in;

    hfi_.setEnabled(true);
    hfi_.setInjectionVoltage(ctx_.v_bus * config_.observer.hfi_injection_voltage_ratio);
    hfi_.setPllGains(config_.observer.hfi_pll_kp,
                     config_.observer.hfi_pll_ki);
    hfi_.setPllLimits(config_.observer.hfi_pll_integral_limit_rad_s,
                      config_.observer.hfi_pll_speed_limit_rad_s);
    hfi_.setLockTicks(config_.observer.hfi_lock_ticks);
    hfi_.setMinSignalLevel(config_.observer.hfi_min_signal_level_a);
    hfi_.setSquareCurrentScale(config_.observer.hfi_square_current_scale);
    hfi_.setSquareLpfCutoffRatio(
        config_.observer.hfi_square_lpf_cutoff_ratio);
    hfi_.setSineLpfCutoffRatio(
        config_.observer.hfi_sine_lpf_cutoff_ratio);
    hfi_.update(ctx_.i_alpha_raw, ctx_.i_beta_raw, i_d_hfi, i_q_hfi,
                dt_, &ctx_.hfi_estimate);
    ctx_.hfi_injection = hfi_.getInjectionCommand();

    ctx_.angle_elec_observer = ctx_.hfi_estimate.angle_rad;
    ctx_.speed_rpm_observer =
        ctx_.hfi_estimate.speed_rad_s * 60.0f /
        (TWO_PI * config_.physical.pole_pairs);
    ctx_.angle_elec = ctx_.angle_elec_observer;
    ctx_.speed_rpm = ctx_.speed_rpm_observer;
    event_.observer_converged = ctx_.hfi_estimate.valid ? 1U : 0U;
    event_.speed_valid = ctx_.hfi_estimate.valid ? 1U : 0U;
    angle_state_ = AngleState::HFI;

    const float ct = cosf(ctx_.angle_elec);
    const float st = sinf(ctx_.angle_elec);
    ctx_.i_d = ctx_.i_alpha * ct + ctx_.i_beta * st;
    ctx_.i_q = -ctx_.i_alpha * st + ctx_.i_beta * ct;

    ctx_.v_d = ctx_.hfi_injection.v_d;
    ctx_.v_q = ctx_.hfi_injection.v_q;
    ctx_.v_alpha = ctx_.v_d * ct - ctx_.v_q * st;
    ctx_.v_beta = ctx_.v_d * st + ctx_.v_q * ct;

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
}

void MotorManager::runHFIShadowObserver(bool inject_voltage)
{
    const float theta_in = ctx_.hfi_estimate.angle_rad;
    const float ct_in = cosf(theta_in);
    const float st_in = sinf(theta_in);
    const float i_d_hfi = ctx_.i_alpha_raw * ct_in + ctx_.i_beta_raw * st_in;
    const float i_q_hfi = -ctx_.i_alpha_raw * st_in + ctx_.i_beta_raw * ct_in;

    hfi_.setEnabled(true);
    hfi_.setInjectionVoltage(ctx_.v_bus * config_.observer.hfi_injection_voltage_ratio);
    hfi_.setPllGains(config_.observer.hfi_pll_kp,
                     config_.observer.hfi_pll_ki);
    hfi_.setPllLimits(config_.observer.hfi_pll_integral_limit_rad_s,
                      config_.observer.hfi_pll_speed_limit_rad_s);
    hfi_.setLockTicks(config_.observer.hfi_lock_ticks);
    hfi_.setMinSignalLevel(config_.observer.hfi_min_signal_level_a);
    hfi_.setSquareCurrentScale(config_.observer.hfi_square_current_scale);
    hfi_.setSquareLpfCutoffRatio(
        config_.observer.hfi_square_lpf_cutoff_ratio);
    hfi_.setSineLpfCutoffRatio(
        config_.observer.hfi_sine_lpf_cutoff_ratio);
    hfi_.update(ctx_.i_alpha_raw, ctx_.i_beta_raw, i_d_hfi, i_q_hfi,
                dt_, &ctx_.hfi_estimate);
    ctx_.hfi_injection = hfi_.getInjectionCommand();

    ctx_.angle_elec_observer = ctx_.hfi_estimate.angle_rad;
    ctx_.speed_rpm_observer =
        ctx_.hfi_estimate.speed_rad_s * 60.0f /
        (TWO_PI * config_.physical.pole_pairs);
    event_.observer_converged = ctx_.hfi_estimate.valid ? 1U : 0U;
    event_.speed_valid = ctx_.hfi_estimate.valid ? 1U : 0U;
    angle_state_ = AngleState::HFI;

    if (inject_voltage)
    {
        const float theta = ctx_.hfi_estimate.angle_rad;
        const float ct = cosf(theta);
        const float st = sinf(theta);
        ctx_.v_alpha += ctx_.hfi_injection.v_d * ct -
                        ctx_.hfi_injection.v_q * st;
        ctx_.v_beta += ctx_.hfi_injection.v_d * st +
                       ctx_.hfi_injection.v_q * ct;
    }
}
#endif
#endif

#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API && LIB_MOTOR_ENABLE_SMO
void MotorManager::runSMOShadowObserver()
{
    smo_.update(ctx_.v_alpha, ctx_.v_beta,
                ctx_.i_alpha, ctx_.i_beta,
                dt_, &ctx_.smo_estimate);

    ctx_.angle_elec_observer = ctx_.smo_estimate.angle_rad;
    ctx_.speed_rpm_observer =
        ctx_.smo_estimate.speed_rad_s * 60.0f /
        (TWO_PI * config_.physical.pole_pairs);
    event_.observer_converged = ctx_.smo_estimate.valid ? 1U : 0U;
    event_.speed_valid = ctx_.smo_estimate.valid ? 1U : 0U;
    angle_state_ = AngleState::SMO;
}
#endif

} // namespace Lib_Motor

