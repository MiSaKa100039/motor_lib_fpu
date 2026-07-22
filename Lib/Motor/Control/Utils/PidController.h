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

#include <cmath>
#include <algorithm>

namespace Lib_Motor
{

class PidController
{
public:
    PidController() = default;

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

private:
    float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
    float limit_     = 0.0f;
    float integral_  = 0.0f;
    float prev_error_= 0.0f;
    float output_    = 0.0f;
    bool  saturated_ = false;
};

} // namespace Lib_Motor
