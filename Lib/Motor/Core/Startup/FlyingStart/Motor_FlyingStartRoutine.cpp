/*
 * Motor_FlyingStartRoutine.cpp — 顺逆风相电压 PLL 启动内部流程 (P2)
 *
 * 数据来源: cfg.sensor.has_phase_voltage=true 时 MotorManager::updateSensorMeasurements
 *           已经把相电压采样到 ctx_.v_phase_u/v/w (V) 单位。
 * Clarke 变换: v_alpha = (v_u - v_w), v_beta = (v_u - 2v_v + v_w) / sqrt(3)
 * 角度观测:   θ_bemf = atan2(-v_alpha_lpf, v_beta_lpf) (与 SMO 反电动势提取一致)
 * PLL 跟踪:    err = wrap(θ_bemf - θ_est) → PI → ω_est → ∫ → θ_est
 *
 * 实施时 PWM 处于 coast (config_.hal->pwm_coast 调用), 不强行注入电流,
 * 让外力自然转动转子产生反电势。
 * PLL locked 后 angle_est/speed_est 直接 seedAngle 给 SMO, setRunPhase(SMO_ONLY)。
 */

#include "../../../Public/Motor_Features.h"

#if LIB_MOTOR_ENABLE_FLYING_START

#include "Motor_FlyingStartRoutine.h"

#include "../Manager/Motor_Manager.h"
#include "../FSM/Motor_FSM.h"

#include <cmath>

namespace Lib_Motor
{

namespace
{
    constexpr float  SQRT3_INV         = 0.5773502691f;
    constexpr uint32_t TIMEOUT_TICKS  = 2000U;    // 100 ms @20kHz 超时
    constexpr uint16_t LOCK_TICKS     = 500U;     // 25 ms 锁相确认
    constexpr float  LOCK_ERROR_RAD   = 0.1f;     // PLL 误差需稳定低于此值才认为锁相

    float wrapAnglePi(float a)
    {
        while (a >  3.14159265358979f) a -= 6.28318530717958f;
        while (a < -3.14159265358979f) a += 6.28318530717958f;
        return a;
    }
}

void MotorFlyingStartRoutine::step(MotorManager& m)
{
    auto& cfg  = m.config_;
    auto& ctx  = m.ctx_;
    auto& hal  = *(cfg.hal);

    switch (phase_)
    {
    case Phase::IDLE:
    {
        // 关闭功率管让相电压自由感应反电势
        if (hal.pwm_coast != nullptr) hal.pwm_coast();
        phase_         = Phase::WAIT_BEMF;
        tick_counter_ = 0U;
        locked_ticks_  = 0U;
        angle_est_     = 0.0f;
        speed_est_     = 0.0f;
        pll_integral_ = 0.0f;
        break;
    }

    case Phase::WAIT_BEMF:
    {
        ++tick_counter_;
        // PWM 关断, 相电压 = 反电势投影, Clarke 变换 (按 SVPWM 同段公式简化)
        const float v_u = ctx.v_phase_u;
        const float v_v = ctx.v_phase_v;
        const float v_w = ctx.v_phase_w;
        const float v_alpha = v_u - v_w;
        const float v_beta  = (v_u - 2.0f * v_v + v_w) * SQRT3_INV;

        const float mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
        if (mag > 1e-3f)
        {
            const float angle_obs = atan2f(-v_alpha, v_beta);
            const float err = wrapAnglePi(angle_obs - angle_est_);
            if (fabsf(err) < LOCK_ERROR_RAD) ++locked_ticks_; else locked_ticks_ = 0U;
            last_angle_obs_ = angle_obs;
            // PI 跟踪
            const float dts = 1.0f / cfg.control.control_freq_hz;
            pll_integral_ += err * dts * pll_ki_;
            speed_est_     = err * pll_kp_ + pll_integral_;
            angle_est_    += speed_est_ * dts;
            // 归一化 [0, 2π)
            while (angle_est_ >  6.28318530717958f) angle_est_ -= 6.28318530717958f;
            while (angle_est_ < 0.0f)              angle_est_ += 6.28318530717958f;
        }

        if (tick_counter_ > TIMEOUT_TICKS)
        {
            // PLL 不收敛时从 IF profile 的对齐首段重新开始。
            phase_ = Phase::DONE;
            m.resetIFStartupProfileState();
            m.resetIFStartupObserverState();
            m.if_angle_gen_.reset();
            m.setRunPhase(RunPhase::FORCE_DRAG);
            m.ctx_.fsm_timer_ticks = 0U;
            break;
        }

        if (locked_ticks_ >= LOCK_TICKS)
        {
            phase_ = Phase::SEED_SMO;
        }
        break;
    }

    case Phase::SEED_SMO:
    {
        // SMO 直接灌入 PLL 估计的 angle + speed
#if LIB_MOTOR_ENABLE_SMO
        m.smo_.seedAngle(angle_est_);
        // speed_est_ 注入通过 Sin/Cos 让 SMO 进入预同步(用 seedAngle 内部置零, 实测需手动)
        // (实际 SMO PLL 与顺逆风启动 PLL 独立; 这里 angle_est_ 注入 SMO 即可)
        ctx.angle_elec_observer = angle_est_;
        ctx.speed_rpm_observer = speed_est_ * 60.0f /
            (6.28318530717958f * cfg.physical.pole_pairs);
        ctx.angle_elec = angle_est_;
        ctx.speed_rpm  = ctx.speed_rpm_observer;
        m.event_.observer_converged = 1U;
        m.event_.speed_valid        = 1U;
        m.event_.flying_start_done  = 1U;
        m.setRunPhase(RunPhase::SMO_ONLY);
        m.ctx_.fsm_timer_ticks = 0U;
#else
        // 没 SMO 时报 fault
        m.stopForFault(Fault::OBSERVER_UNAVAILABLE);
#endif
        phase_ = Phase::DONE;
        break;
    }

    case Phase::DONE:
        // 已完成
        break;
    }
}

} // namespace Lib_Motor

#endif /* LIB_MOTOR_ENABLE_FLYING_START */
