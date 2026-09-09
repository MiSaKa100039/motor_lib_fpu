/*
 * FocMath_Libm.h — libm 软件浮点后端 (默认)
 *
 * 适用: 所有 MCU (含 FPU 走 VFP 硬件指令, 无 FPU 走软浮点)。
 * 行为与原内联 sinf/cosf/atan2f/sqrtf 完全等价, 作为 CORDIC 不可用时的回退。
 *
 * 接口契约:
 *   foc_sin_cos_f32: 输入 theta_rad (rad), 输出 sin/cos (归一化 [-1,1])
 *   foc_atan2_f32:   输入 y, x, 输出 [-π, π]
 *   foc_sqrt_f32:    输入 x≥0, 输出 √x
 */

#pragma once

#include <cmath>

/* libm 回退后端: 适用于所有带 FPU 或软浮点的 MCU。
 * 行为与原内联 sinf/cosf/atan2f/sqrtf 完全等价。
 *
 * 本文件被 FocMath.h 在 namespace Lib_Motor 内部 include,
 * 故此处不再包裹 namespace, 避免双重命名空间。 */

/* sin + cos 联合接口: libm 后端内部仍为两次独立调用,
 * 但接口形式统一, 便于调用方复用结果, 避免重复计算。 */
inline void foc_sin_cos_f32(float theta_rad, float& sin_v, float& cos_v)
{
    sin_v = sinf(theta_rad);
    cos_v = cosf(theta_rad);
}

inline float foc_atan2_f32(float y, float x)
{
    return atan2f(y, x);
}

inline float foc_sqrt_f32(float x)
{
    return sqrtf(x);
}
