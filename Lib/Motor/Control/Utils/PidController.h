/*
 * PidController.h — PID 控制器 (抗积分饱和)
 *
 * 积分解耦: P 输出不计入积分限幅 (避免积分饱和)
 *   output = P + I + D
 *   当 |output| > limit → integral = limit - (P + D)
 *
 * 使用:
 *   PidController pid;
 *   pid.init(cfg.control.current_q);  // 从 MotorConfig 加载参数
 *   float v = pid.update(error, dt);  // 每控制周期调用
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../../Public/Motor_Config.h"

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../../Core/Numeric/Motor_FixedNumeric.h"
#endif

namespace Lib_Motor
{

class PidController
{
public:
    PidController() = default;

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    void init(const PIDParam& param)
    {
        param_ = param;
        reset();
    }

    void reset()
    {
        integrator_q30_ = 0;
        output_q15_ = 0;
        saturated_ = false;
    }

    void setOutputLimit(float limit)
    {
        param_.output_limit = limit;
    }

    void setOutputLimitQ15(FixedNumeric::q15_t limit, bool reset_integrator = false)
    {
        output_limit_q15_ = FixedNumeric::absoluteQ15(limit);
        const std::int32_t limit_q30 =
            static_cast<std::int32_t>(output_limit_q15_) << 15U;
        if (reset_integrator)
        {
            integrator_q30_ = 0;
        }
        else
        {
            integrator_q30_ =
                FixedNumeric::clamp32(integrator_q30_, -limit_q30, limit_q30);
        }
        saturated_ = false;
    }

    void decayIntegral(float factor)
    {
        if (!std::isfinite(factor))
        {
            return;
        }
        factor = std::max(0.0f, std::min(1.0f, factor));
        integrator_q30_ = static_cast<std::int32_t>(
            static_cast<float>(integrator_q30_) * factor);
    }

    bool isSaturated() const { return saturated_; }

    void configureFixedGains(float input_base,
                             float output_base,
                             float dt,
                             bool reset_integrator = false)
    {
        if (!(input_base > 0.0f) || !(output_base > 0.0f) || !(dt > 0.0f))
        {
            kp_q15_ = 0;
            ki_per_tick_q15_ = 0;
            output_limit_q15_ = 0;
            integrator_q30_ = 0;
            output_q15_ = 0;
            saturated_ = false;
            return;
        }

        kp_q15_ = FixedNumeric::fromNormalized((param_.kp * input_base) / output_base);
        ki_per_tick_q15_ =
            FixedNumeric::fromNormalized((param_.ki * dt * input_base) / output_base);

        const std::int32_t limit_q30 = static_cast<std::int32_t>(output_limit_q15_) << 15U;
        if (reset_integrator)
        {
            integrator_q30_ = 0;
        }
        else
        {
            integrator_q30_ =
                FixedNumeric::clamp32(integrator_q30_, -limit_q30, limit_q30);
        }
    }

    void configureFixed(float input_base,
                        float output_base,
                        float dt,
                        float output_limit,
                        bool reset_integrator = false)
    {
        configureFixedGains(input_base, output_base, dt, reset_integrator);
        setOutputLimitQ15(
            FixedNumeric::fromPhysical(output_limit, output_base),
            reset_integrator);
    }

    FixedNumeric::q15_t update(FixedNumeric::q15_t error)
    {
        return update(error, false);
    }

    FixedNumeric::q15_t update(FixedNumeric::q15_t error, bool hold_integral)
    {
        const std::int32_t proportional_q30 =
            static_cast<std::int32_t>(kp_q15_) * error;
        const std::int32_t limit_q30 = static_cast<std::int32_t>(output_limit_q15_) << 15U;
        if (!hold_integral)
        {
            integrator_q30_ = FixedNumeric::clamp32(
                integrator_q30_ + static_cast<std::int32_t>(ki_per_tick_q15_) * error,
                -limit_q30,
                limit_q30);
        }

        const std::int32_t unclamped =
            (proportional_q30 + integrator_q30_) >> 15U;
        const std::int32_t clamped =
            FixedNumeric::clamp32(unclamped, -output_limit_q15_, output_limit_q15_);
        saturated_ = (clamped != unclamped);
        output_q15_ = FixedNumeric::saturateQ15(clamped);
        return output_q15_;
    }

    FixedNumeric::q15_t getOutputQ15() const { return output_q15_; }
    std::int32_t getIntegralQ30() const { return integrator_q30_; }
    float kp() const { return param_.kp; }
    float ki() const { return param_.ki; }
    float kd() const { return param_.kd; }
    float outputLimit() const { return param_.output_limit; }

private:
    PIDParam param_{};
    FixedNumeric::q15_t kp_q15_ = 0;
    FixedNumeric::q15_t ki_per_tick_q15_ = 0;
    FixedNumeric::q15_t output_limit_q15_ = 0;
    std::int32_t integrator_q30_ = 0;
    FixedNumeric::q15_t output_q15_ = 0;
    bool saturated_ = false;
#else
    void init(const struct PIDParam& param)
    {
        kp_ = param.kp; ki_ = param.ki; kd_ = param.kd;
        limit_ = param.output_limit;
        reset();
    }

    void reset() { integral_ = prev_error_ = output_ = 0.0f; saturated_ = false; }

    void setOutputLimit(float limit) { limit_ = limit; }

    void decayIntegral(float factor)
    {
        factor = std::max(0.0f, std::min(1.0f, factor));
        integral_ *= factor;
    }

    /* 当前周期是否被内部 limit 钳位 (饱和标志) */
    bool isSaturated() const { return saturated_; }

    /*
     * 一次 PID 更新
     * @param error 参考值 - 反馈值
     * @param dt    控制周期 (秒)
     * @return 控制输出
     */
    float update(float error, float dt)
    {
        return update(error, dt, false);
    }

    float update(float error, float dt, bool hold_integral)
    {
        float p = kp_ * error;

        // 积分项 (带限幅)
        if (!hold_integral)
        {
            integral_ += ki_ * error * dt;
        }
        if (limit_ > 0.0f)
            integral_ = std::max(-limit_, std::min(limit_, integral_));

        // 微分项
        float d = 0.0f;
        if (dt > 1e-9f) d = kd_ * (error - prev_error_) / dt;

        output_ = p + integral_ + d;
        saturated_ = false;

        // 抗积分饱和: P+D 部分不计入积分限幅
        if (limit_ > 0.0f)
        {
            float p_plus_d = p + d;
            if (output_ > limit_)
            {
                integral_ = limit_ - p_plus_d;
                output_ = limit_;
                saturated_ = true;
            }
            else if (output_ < -limit_)
            {
                integral_ = -limit_ - p_plus_d;
                output_ = -limit_;
                saturated_ = true;
            }
        }

        prev_error_ = error;
        return output_;
    }

    float getOutput()   const { return output_; }
    float getIntegral() const { return integral_; }
    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }
    float outputLimit() const { return limit_; }

private:
    float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
    float limit_     = 0.0f;
    float integral_  = 0.0f;
    float prev_error_= 0.0f;
    float output_    = 0.0f;
    bool  saturated_ = false;
#endif
};

} // namespace Lib_Motor
