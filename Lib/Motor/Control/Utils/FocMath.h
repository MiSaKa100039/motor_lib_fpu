/*
 * FocMath.h — FOC 数学库 (统一接口 + 后端分发)
 *
 * 提供三层接口:
 *   [1] 超越函数统一接口 (调用方只依赖这三个, 不感知后端):
 *        foc_sin_cos_f32(theta, &sin, &cos)  -- sin+cos 联合接口
 *        foc_atan2_f32(y, x)
 *        foc_sqrt_f32(x)
 *
 *   [2] 坐标变换 (内部使用统一接口, 自动消除冗余三角调用):
 *        clarke_transform      ABC → αβ
 *        park_transform         αβ → dq
 *        inv_park_transform     dq  → αβ
 *
 *   [3] 后端分发 (编译时根据宏选择):
 *        MOTOR_BUILD_ENABLE_CORDIC_ACCEL → FocMath_Cordic.h (硬件加速)
 *        其他                            → FocMath_Libm.h    (libm 回退)
 *
 * 设计要点:
 *   - park_transform / inv_park_transform 内部均只调用一次 foc_sin_cos_f32,
 *     从源头消除 "Park 和 InvPark 各自调用 sinf/cosf" 的冗余。
 *   - 调用方 (runControlLoop 等) 若 Park/InvPark 共用同一电角度,
 *     应在函数入口预算一次 sin/cos 后直接代入, 避免两次 park_*_transform 调用。
 */

#pragma once

#include <cmath>

namespace Lib_Motor
{

constexpr float PI       = 3.14159265358979323846f;
constexpr float TWO_PI   = 6.28318530717958647692f;
constexpr float SQRT3    = 1.73205080756887729353f;
constexpr float SQRT3_D2 = 0.86602540378443864676f;

/* ================================================================
 * [1] 后端分发 (必须在变换函数之前 include, 使 foc_sin_cos_f32 等定义可见)
 *
 * CORDIC 后端: sin/cos 走硬件, atan2/sqrt 暂保留 libm
 * Libm  后端: 全部走 libm, 适用于无 CORDIC 的 MCU
 * ================================================================ */
#ifdef MOTOR_BUILD_ENABLE_CORDIC_ACCEL
  #include "FocMath_Cordic.h"
#else
  #include "FocMath_Libm.h"
#endif

/* ================================================================
 * [2] 坐标变换 (内部使用统一接口, 自动消除冗余三角调用)
 * ================================================================ */

/*
 * Clarke 变换: Ia,Ib → i_alpha, i_beta
 * (双电阻采样, Ic = -(Ia+Ib) 由 KCL 计算)
 */
inline void clarke_transform(float ia, float ib, float& i_alpha, float& i_beta)
{
    i_alpha = ia;
    i_beta  = (ia + 2.0f * ib) / SQRT3;
}

/*
 * Park 变换: i_alpha, i_beta → i_d, i_q
 * θ 为电角度 (rad)
 * 内部只调用一次 foc_sin_cos_f32, 从源头消除冗余 sin/cos。
 */
inline void park_transform(float i_alpha, float i_beta, float theta,
                           float& i_d, float& i_q)
{
    float sin_v, cos_v;
    foc_sin_cos_f32(theta, sin_v, cos_v);
    i_d =  i_alpha * cos_v + i_beta * sin_v;
    i_q = -i_alpha * sin_v + i_beta * cos_v;
}

/*
 * Park 反变换: v_d, v_q → v_alpha, v_beta
 * 与 Park 正变换共用同一 θ; 调用方若已算过 sin/cos, 可直接用下面带 sin/cos 入参的重载。
 */
inline void inv_park_transform(float v_d, float v_q, float theta,
                                float& v_alpha, float& v_beta)
{
    float sin_v, cos_v;
    foc_sin_cos_f32(theta, sin_v, cos_v);
    v_alpha = v_d * cos_v - v_q * sin_v;
    v_beta  = v_d * sin_v + v_q * cos_v;
}

/* Park 变换 (带预算 sin/cos 入参): 调用方已在入口算过 sin/cos 时使用, 避免重复计算 */
inline void park_transform_sc(float i_alpha, float i_beta,
                              float sin_v, float cos_v,
                              float& i_d, float& i_q)
{
    i_d =  i_alpha * cos_v + i_beta * sin_v;
    i_q = -i_alpha * sin_v + i_beta * cos_v;
}

/* Park 反变换 (带预算 sin/cos 入参): 同上 */
inline void inv_park_transform_sc(float v_d, float v_q,
                                   float sin_v, float cos_v,
                                   float& v_alpha, float& v_beta)
{
    v_alpha = v_d * cos_v - v_q * sin_v;
    v_beta  = v_d * sin_v + v_q * cos_v;
}

} // namespace Lib_Motor
