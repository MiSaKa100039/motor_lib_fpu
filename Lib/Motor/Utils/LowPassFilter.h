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

#include <cmath>

namespace Lib_Motor
{

class LowPassFilter
{
public:
    /*
     * tf_s 是滤波时间常数, 不是截止频率:
     *   fc ~= 1 / (2*pi*Tf)
     * tf_s <= 0 时 update() 等效为直通, 可用于关闭软件滤波.
     */
    explicit LowPassFilter(float tf_s = 0.005f)
        : tf_(tf_s), sample_period_s_(1.0f / 20000.0f)
        , prev_output_(0.0f), initialized_(false)
    {}

    /* 允许在初始化时重设; 设置为 0 可关闭滤波而保留统一调用路径. */
    void setTf(float tf_s) { tf_ = tf_s; }

    /* 滤波系数依赖真实采样周期, 电流环中通常应与 PWM/ISR 周期一致. */
    void setSamplePeriod(float dt_s)
    {
        if (dt_s > 0.0f) sample_period_s_ = dt_s;
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
         * Tf 为 0 或负值时 alpha=1, 输出等于输入.
         * 若本滤波器位于反馈链路, Tf 过大会增加闭环相位滞后;
         * 应按环路带宽和噪声共同选值, 不能只追求波形平滑.
         */
        float alpha = (tf_ > 0.0f)
            ? sample_period_s_ / (tf_ + sample_period_s_)
            : 1.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        if (alpha < 0.0f) alpha = 0.0f;

        prev_output_ = prev_output_ + alpha * (input - prev_output_);
        return prev_output_;
    }

    /*
     * 清除历史状态. reset() 后第一次 update() 仍以新输入直接初始化;
     * 参数 value 只在第一次 update() 之前通过 getValue() 可见.
     */
    void reset(float value = 0.0f) { prev_output_ = value; initialized_ = false; }

    float getValue() const { return prev_output_; }

private:
    float tf_;              // 时间常数 (s)
    float sample_period_s_; // 采样周期 (s)
    float prev_output_;     // y[n-1]
    bool  initialized_;     // 首次调用标志
};

} // namespace Lib_Motor
