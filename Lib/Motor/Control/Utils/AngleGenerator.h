/*
 * AngleGenerator.h — 开环角度发生器 (速度斜坡)
 *
 * 用途: V/F 和 I/F 调试模式下生成开环电角度
 *       也用于无感启动的 FORCE_DRAG 阶段 (I/F 强拖)
 *
 * 原理 (速度斜坡 + 角积分):
 *
 *   ┌──────────────────────────────────────────────┐
 *   │  setTargetSpeed(rpm, ramp_time)              │
 *   │    → 如果首次调用: ramp_rate = |rpm| / time  │
 *   │      (后续调用不改变 ramp_rate, 继续按原速率) │
 *   │                                              │
 *   │  update(dt)                                  │
 *   │    [1] 速度斜坡                              │
 *   │        if ramp_rate > 0:                     │
 *   │          current → ramp → target             │
 *   │          step = ramp_rate * dt               │
 *   │          当误差 < step 时直接跳到 target      │
 *   │                                              │
 *   │    [2] 角度积分                              │
 *   │        speed_rad_s = current_rpm × π/30      │
 *   │        angle_elec += speed_rad_s × pp × dt   │
 *   │        angle ∈ [0, 2π)                       │
 *   └──────────────────────────────────────────────┘
 *
 * ramp_rate = 0 的特殊情况:
 *   - 首次调用但 ramp_time ≤ 1ms → 零加速时间, 直接跳到 target
 *   - reset() 后再次 setTargetSpeed 时, 非首次调用 → 瞬间加速完成
 *
 * 典型用法:
 *   AngleGenerator gen;
 *   gen.init(20000);                           // 控制频率 20kHz
 *   gen.setTargetSpeed(1000, 2.0f);            // 2 秒加速到 1000 RPM
 *   while (running) { gen.update(dt); angle = gen.getAngle(); }
 */

#pragma once

#include "../../Public/Motor_Features.h"
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../../Core/Numeric/Motor_FixedNumeric.h"
#endif

#include <stdint.h>
#include <cmath>
#include <algorithm>

namespace Lib_Motor
{

class AngleGenerator
{
public:
    AngleGenerator() = default;

    /* 记录控制频率/极对数 (用于日志/验证), 重置所有内部状态 */
    void init(float freq_hz, uint8_t pole_pairs = 1, float speed_base_rpm = 1.0f)
    {
        freq_hz_ = freq_hz;
        pole_pairs_ = (pole_pairs == 0U) ? 1U : pole_pairs;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        configureFixedScale(speed_base_rpm);
#else
        (void)speed_base_rpm;
#endif
        reset();
    }

    /* 清零: current_rpm=0, angle=0, 标记为首次调用 (下次 setTargetSpeed 重新计算 ramp) */
    void reset()
    {
        current_rpm_ = 0.0f;
        angle_ = 0.0f;
        first_call_ = true;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        target_speed_q15_ = 0;
        current_speed_q15_ = 0;
        speed_step_q15_per_tick_ = 0U;
        angle_phase_ = 0U;
        first_call_q15_ = true;
#endif
    }

    /*
     * 设置目标转速和加速时间
     *
     * 首次调用 (first_call_ == true):
     *   - 从当前转速 0 开始加速
     *   - ramp_rate = |target_rpm| / ramp_time_s
     *   - 若 ramp_time_s ≤ 0.001s → ramp_rate = 0 (瞬间完成)
     *
     * 后续调用 (first_call_ == false):
     *   - 仅更新 target_rpm_, 不改变 ramp_rate_ (保持原加速速率)
     *   - 当前速度继续按旧 ramp_rate 逼近新 target
     */
    void setTargetSpeed(float rpm, float ramp_time_s)
    {
        target_rpm_ = rpm;
        if (first_call_)
        {
            current_rpm_ = 0.0f;
            ramp_rate_ = 0.0f;
            if (ramp_time_s > 0.001f)
                ramp_rate_ = fabsf(target_rpm_) / ramp_time_s;
            first_call_ = false;
        }
    }

    /*
     * 每控制周期调用一次, dt = 1/control_freq_hz (秒)
     *
     * [Step 1] 速度斜坡逼近:
     *   - 有 ramp_rate: current_rpm 以固定速率向 target_rpm 斜升
     *   - 无 ramp_rate (瞬间): current_rpm 直接等于 target_rpm
     *
     * [Step 2] 角度积分:
     *   mechanical RPM → mechanical rad/s → 乘极对数 → 累加到电角度
     *   angle 始终归一化到 [0, 2π)
     */
    void update(float dt)
    {
        // [1] 速度斜升至目标
        if (ramp_rate_ > 0.0f)
        {
            float error = target_rpm_ - current_rpm_;
            float step  = ramp_rate_ * dt;          // 本帧速度增量
            if (fabsf(error) < step)
                current_rpm_ = target_rpm_;          // 误差 < 增量: 直接跳到目标, 防过冲
            else
                current_rpm_ += (error > 0 ? step : -step);  // 正常步进
        }
        else
        {
            current_rpm_ = target_rpm_;              // 无斜坡: 瞬间到目标
        }

        // [2] 角度积分: angle_elec = ∫ ω_mech * pole_pairs · dt
        float speed_rad_s = current_rpm_ * PI / 30.0f;  // mechanical RPM → mechanical rad/s
        angle_ += speed_rad_s * (float)pole_pairs_ * dt;
        while (angle_ > TWO_PI) angle_ -= TWO_PI;       // 归一化 [0, 2π)
        while (angle_ < 0.0f)   angle_ += TWO_PI;
    }

    float getAngle() const    { return angle_; }       // 当前开环电角度 (rad)
    float getSpeedRPM() const { return current_rpm_; } // 当前斜坡转速 (RPM)

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    /*
     * Q15 速度斜坡入口:
     *   rpm_q15: 机械转速 / speed_base_rpm 的 Q15 表示
     *   ramp_ticks: 从 0 斜升到目标所需控制周期数; 0/1 表示瞬时到达
     */
    void setTargetSpeedQ15(FixedNumeric::q15_t rpm_q15, uint32_t ramp_ticks)
    {
        setTargetSpeedQ15(rpm_q15, ramp_ticks, calculateSpeedStepQ15(rpm_q15, ramp_ticks));
    }

    void setTargetSpeedQ15(FixedNumeric::q15_t rpm_q15,
                           uint32_t ramp_ticks,
                           uint32_t speed_step_q15_per_tick)
    {
        target_speed_q15_ = rpm_q15;
        if (first_call_q15_)
        {
            current_speed_q15_ = 0;
            speed_step_q15_per_tick_ =
                (ramp_ticks > 1U) ? clampSpeedStepQ15(speed_step_q15_per_tick) : 0U;
            first_call_q15_ = false;
        }
    }

    /* Q15 后端每 tick 调用; 只做整数 ramp 与相位累加。 */
    void updateQ15()
    {
        if (speed_step_q15_per_tick_ > 0U)
        {
            const int32_t error =
                static_cast<int32_t>(target_speed_q15_) -
                static_cast<int32_t>(current_speed_q15_);
            const uint32_t error_abs =
                (error < 0) ? static_cast<uint32_t>(-error)
                            : static_cast<uint32_t>(error);
            if (error_abs <= speed_step_q15_per_tick_)
            {
                current_speed_q15_ = target_speed_q15_;
            }
            else
            {
                const int32_t next =
                    static_cast<int32_t>(current_speed_q15_) +
                    ((error > 0) ? static_cast<int32_t>(speed_step_q15_per_tick_)
                                 : -static_cast<int32_t>(speed_step_q15_per_tick_));
                current_speed_q15_ = FixedNumeric::saturateQ15(next);
            }
        }
        else
        {
            current_speed_q15_ = target_speed_q15_;
        }

        angle_phase_ = static_cast<FixedNumeric::phase_u32_t>(
            angle_phase_ + phaseStepFromSpeedQ15(current_speed_q15_));
    }

    FixedNumeric::phase_u32_t getAnglePhase() const { return angle_phase_; }
    FixedNumeric::q15_t getSpeedQ15() const { return current_speed_q15_; }

    static uint32_t calculateSpeedStepQ15(FixedNumeric::q15_t rpm_q15,
                                          uint32_t ramp_ticks)
    {
        if (ramp_ticks <= 1U)
        {
            return 0U;
        }

        const uint32_t target_abs =
            (rpm_q15 < 0)
                ? static_cast<uint32_t>(-static_cast<int32_t>(rpm_q15))
                : static_cast<uint32_t>(rpm_q15);
        uint32_t step = (target_abs + ramp_ticks - 1U) / ramp_ticks;
        if (step == 0U && target_abs > 0U)
        {
            step = 1U;
        }
        return clampSpeedStepQ15(step);
    }
#endif

private:
    float freq_hz_    = 10000.0f;   // 控制频率 (Hz), 仅记录用
    float target_rpm_ = 0.0f;       // 目标转速 (RPM)
    float current_rpm_= 0.0f;       // 当前斜坡转速 (RPM), 每帧向 target 逼近
    float ramp_rate_  = 0.0f;       // 加速度 (RPM/s), 首次 setTargetSpeed 时计算
    float angle_      = 0.0f;       // 当前电角度 (rad), 积分累积
    uint8_t pole_pairs_ = 1U;        // 机械角速度 → 电角速度倍数
    bool  first_call_ = true;       // true=尚未调用 setTargetSpeed, 下次会重新计算 ramp

    static constexpr float PI     = 3.14159265358979323846f;
    static constexpr float TWO_PI = 6.28318530717958647692f;

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    void configureFixedScale(float speed_base_rpm)
    {
        phase_step_q16_per_speed_q15_ = 0U;
        if (!(freq_hz_ > 0.0f) || !(speed_base_rpm > 0.0f))
        {
            return;
        }

        /*
         * phase_step = speed_q15 * speed_base_rpm * pole_pairs / 60 / freq_hz * 2^32 / 2^15
         * 保存为 Q16 系数，tick 内只需一次 32x32->64 乘法和右移。
         */
        const float scaled =
            speed_base_rpm *
            static_cast<float>(pole_pairs_) *
            8589934592.0f /
            (60.0f * freq_hz_);
        if (scaled >= 4294967040.0f)
        {
            phase_step_q16_per_speed_q15_ = 0xFFFFFFFFUL;
        }
        else if (scaled > 0.0f)
        {
            phase_step_q16_per_speed_q15_ =
                static_cast<uint32_t>(scaled + 0.5f);
        }
    }

    FixedNumeric::phase_u32_t phaseStepFromSpeedQ15(FixedNumeric::q15_t speed_q15) const
    {
        const int32_t speed = static_cast<int32_t>(speed_q15);
        const uint32_t speed_abs =
            (speed < 0) ? static_cast<uint32_t>(-speed)
                        : static_cast<uint32_t>(speed);
        const uint32_t step = static_cast<uint32_t>(
            (static_cast<uint64_t>(speed_abs) *
             static_cast<uint64_t>(phase_step_q16_per_speed_q15_)) >> 16U);
        return (speed < 0) ? static_cast<FixedNumeric::phase_u32_t>(0UL - step)
                           : step;
    }

    static uint32_t clampSpeedStepQ15(uint32_t step)
    {
        return (step > 32767U) ? 32767U : step;
    }

    FixedNumeric::q15_t target_speed_q15_ = 0;
    FixedNumeric::q15_t current_speed_q15_ = 0;
    uint32_t speed_step_q15_per_tick_ = 0U;
    FixedNumeric::phase_u32_t angle_phase_ = 0U;
    uint32_t phase_step_q16_per_speed_q15_ = 0U;
    bool first_call_q15_ = true;
#endif
};

} // namespace Lib_Motor
