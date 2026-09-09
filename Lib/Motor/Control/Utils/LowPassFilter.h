/*
 * LowPassFilter.h — 一阶 IIR 低通滤波器
 *
 * 差分方程: y[n] = y[n-1] + α × (x[n] - y[n-1])
 * α = Ts / (Tf + Ts),  Tf=滤波时间常数, Ts=采样周期
 *
 * 用于: 电流采样噪声抑制 (闭环反馈需使用轻滤波或直通)
 *       母线电流/电压滤波 (可使用较长时间常数)
 *
 * 注意: 首次调用 update() 时自动初始化输出为输入值
 */

#pragma once

#include "../../Public/Motor_Features.h"
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../../Core/Numeric/Motor_FixedNumeric.h"
#endif

#include <cmath>
#include <cstdint>

namespace Lib_Motor
{

class LowPassFilter
{
public:
    /*
     * tf_s 是滤波时间常数, 不是截止频率:
     *   fc ~= 1 / (2*pi*Tf)
     * tf_s == 0 表示无滤波; tf_s < 0 为非法配置, 由配置校验拦截.
     */
    explicit LowPassFilter(float tf_s = 0.005f)
        : tf_(tf_s), sample_period_s_(1.0f / 20000.0f)
        , alpha_(1.0f), prev_output_(0.0f), initialized_(false)
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        , alpha_q15_(FixedNumeric::kQ15One), prev_output_q15_(0), initialized_q15_(false)
#endif
    {
        recomputeAlpha();
    }

    /* 允许在初始化时重设; 设置为 0 可关闭滤波而保留统一调用路径. */
    void setTf(float tf_s)
    {
        tf_ = tf_s;
        recomputeAlpha();
    }

    /* 滤波系数依赖真实采样周期, 电流环中通常应与 PWM/ISR 周期一致. */
    void setSamplePeriod(float dt_s)
    {
        if (dt_s > 0.0f)
        {
            sample_period_s_ = dt_s;
            recomputeAlpha();
        }
    }

    float update(float input)
    {
        /*
         * 构造或 reset() 后第一次更新直接输出输入值,
         * 避免初始状态人为设为 0 而形成一段虚假的滤波过渡.
         */
        if (!initialized_)
        {
            prev_output_ = input;
            initialized_ = true;
            return input;
        }

        /*
         * 一阶低通的后向欧拉离散形式:
         *   y[n] = y[n-1] + alpha * (x[n] - y[n-1])
         *   alpha = Ts / (Tf + Ts)
         *
         * Tf 为 0 时 alpha=1, 输出等于输入.
         * Tf 为负值属于非法配置, 正常应在 MotorConfigCheck 中被拒绝.
         * 若本滤波器位于反馈链路, Tf 过大会增加闭环相位滞后;
         * 应按环路带宽和噪声共同选值, 不能只追求波形平滑.
         */
        prev_output_ = prev_output_ + alpha_ * (input - prev_output_);
        return prev_output_;
    }

    /*
     * 清除历史状态. reset() 后第一次 update() 仍以新输入直接初始化;
     * 参数 value 只在第一次 update() 之前通过 getValue() 可见.
     */
    void reset(float value = 0.0f) { prev_output_ = value; initialized_ = false; }

    float getValue() const { return prev_output_; }

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    FixedNumeric::q15_t updateQ15(FixedNumeric::q15_t input)
    {
        if (!initialized_q15_)
        {
            prev_output_q15_ = input;
            initialized_q15_ = true;
            return input;
        }

        const std::int32_t error =
            static_cast<std::int32_t>(input) - static_cast<std::int32_t>(prev_output_q15_);
        const std::int32_t step =
            (error * static_cast<std::int32_t>(alpha_q15_) + (1 << 14)) >> 15;
        prev_output_q15_ =
            FixedNumeric::saturateQ15(static_cast<std::int32_t>(prev_output_q15_) + step);
        return prev_output_q15_;
    }

    void resetQ15(FixedNumeric::q15_t value = 0)
    {
        prev_output_q15_ = value;
        initialized_q15_ = false;
    }

    FixedNumeric::q15_t getValueQ15() const { return prev_output_q15_; }
#endif

private:
    void recomputeAlpha()
    {
        float alpha = 1.0f;
        if (tf_ > 0.0f && sample_period_s_ > 0.0f)
        {
            alpha = sample_period_s_ / (tf_ + sample_period_s_);
        }
        if (!std::isfinite(alpha))
        {
            alpha = 1.0f;
        }
        if (alpha > 1.0f) alpha = 1.0f;
        if (alpha < 0.0f) alpha = 0.0f;
        alpha_ = alpha;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        alpha_q15_ = FixedNumeric::fromNormalized(alpha);
#endif
    }

    float tf_;              // 时间常数 (s)
    float sample_period_s_; // 采样周期 (s)
    float alpha_;           // Cached IIR coefficient; update() has no division.
    float prev_output_;     // y[n-1]
    bool  initialized_;     // 首次调用标志
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    FixedNumeric::q15_t alpha_q15_;       // q15 滤波系数, ISR 更新无浮点乘法
    FixedNumeric::q15_t prev_output_q15_; // q15 y[n-1]
    bool initialized_q15_;                // q15 首次调用标志
#endif
};

} // namespace Lib_Motor
