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
    void init(float freq_hz, uint8_t pole_pairs = 1)
    {
        freq_hz_ = freq_hz;
        pole_pairs_ = (pole_pairs == 0U) ? 1U : pole_pairs;
        reset();
    }

    /* 清零: current_rpm=0, angle=0, 标记为首次调用 (下次 setTargetSpeed 重新计算 ramp) */
    void reset() { current_rpm_ = 0.0f; angle_ = 0.0f; first_call_ = true; }

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
};

} // namespace Lib_Motor
