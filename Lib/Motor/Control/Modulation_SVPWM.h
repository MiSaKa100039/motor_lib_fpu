/*
 * Modulation_SVPWM.h — 空间矢量 PWM 与 正弦 PWM 调制
 *
 * 两种实现:
 *   svpwm_modulate(): 七段式 SVPWM
 *     — 通过 min/max 共同模注入, 电压利用率 1.1547×SPWM
 *     — 波形: 注入三次谐波的马鞍波形
 *
 *   spwm_modulate(): 正弦 PWM
 *     — 无注入, 电压利用率 1.0
 *     — 波形: 纯正弦波, 无三次谐波畸变
 *
 * 输入: v_alpha, v_beta (静止坐标系电压, -vbus/2 ~ +vbus/2)
 * 输出: duty[0~2] (占空比, 0.0 ~ 1.0, 对应 αβ→UVW 三相)
 *
 * 两者均为 header-only inline 函数, 零调用开销
 */

#pragma once

#include <cmath>

namespace Lib_Motor
{

/*
 * SVPWM — 空间矢量调制
 *
 * 原理:
 *   1. αβ → UVW 三相电压 (Clarke 反变换)
 *   2. min-max 共同模注入: offset = -(max+min)/2
 *   3. 归一化到 [0,1] 占空比
 *
 * 注: 注入后的波形等效于在马鞍波顶端加了一直流偏移
 *     有效电压范围比纯正弦大 15.47%
 */
inline void svpwm_modulate(float v_alpha, float v_beta, float vbus,
                           float duty[3])
{
    if (vbus < 0.1f) vbus = 0.1f;         // 防除零

    constexpr float SQRT3_2 = 0.8660254037844386f;  // √3/2

    // αβ → 三相电压
    float u = v_alpha;
    float v = -0.5f * v_alpha + SQRT3_2 * v_beta;
    float w = -0.5f * v_alpha - SQRT3_2 * v_beta;

    // min-max 共同模注入
    float v_max = u; if (v > v_max) v_max = v; if (w > v_max) v_max = w;
    float v_min = u; if (v < v_min) v_min = v; if (w < v_min) v_min = w;
    float v_offset = -(v_max + v_min) * 0.5f;   // 共同模偏移

    u += v_offset; v += v_offset; w += v_offset;

    // 归一化: 将 [-vbus/2, vbus/2] 映射到 [0, 1]
    float scale = 1.0f / vbus;
    duty[0] = 0.5f + u * scale;
    duty[1] = 0.5f + v * scale;
    duty[2] = 0.5f + w * scale;
}

/*
 * SPWM — 正弦调制 (无共同模注入)
 *
 * 区别: 不做 min-max 注入, 输出为纯正弦波
 * 电压利用率较低: 最大正弦峰值 = vbus/2 (对应占空比 1.0)
 */
inline void spwm_modulate(float v_alpha, float v_beta, float vbus,
                          float duty[3])
{
    if (vbus < 0.1f) vbus = 0.1f;

    constexpr float SQRT3_2 = 0.8660254037844386f;

    float u = v_alpha;
    float v = -0.5f * v_alpha + SQRT3_2 * v_beta;
    float w = -0.5f * v_alpha - SQRT3_2 * v_beta;

    float scale = 1.0f / vbus;
    duty[0] = 0.5f + u * scale;
    duty[1] = 0.5f + v * scale;
    duty[2] = 0.5f + w * scale;
}

} // namespace Lib_Motor
