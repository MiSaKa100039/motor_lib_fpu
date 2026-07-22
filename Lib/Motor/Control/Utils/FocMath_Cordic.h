/*
 * FocMath_Cordic.h — CORDIC 硬件协处理器后端 (STM32G4/H5/U5/WL55 等)
 *
 * sin/cos 走 CORDIC 硬件 (~6 周期/次 + q31 转换), atan2/sqrt 暂保留 libm。
 *
 * CORDIC 外设接口 (见 stm32g431xx.h / RM0440 第 17 章):
 *   CSR   — 控制/状态 (FUNC[3:0], PRECISION[7:4], SCALE[10:8], RRDY[31])
 *   WDATA — 输入参数 (q1.31, 即 int32_t 范围 [-1, 1))
 *   RDATA — 输出结果 (q1.31)
 *
 * FUNC 码:
 *   0x00 COSINE   1 arg → 1 res
 *   0x01 SINE     1 arg → 1 res
 *   0x02 PHASE    2 arg → 1 res  (atan2, 暂未启用)
 *   0x03 MODULUS  2 arg → 1 res  (sqrt(x²+y²), 暂未启用)
 *   0x09 SQRT     1 arg → 1 res  (暂未启用)
 *
 * 角度约定: 输入 [-π, π] → q1.31 (rad/π × 2^31); 输出 [-1, 1) → float (q/2^31)
 *
 * 初始化: foc_math_cordic_init() 开时钟 (CubeMX 的 MX_CORDIC_Init 也会开, 此处为冗余保障)
 *
 * 注意: ISR 内为轮询模式 (while !RRDY), 不使用中断, NVIC 无需配置
 */

#pragma once

#include <cmath>
#include <cstdint>

/* HAL 层提供 __HAL_RCC_CORDIC_CLK_ENABLE 宏 */
#include "stm32g4xx_hal.h"

/* 本文件被 FocMath.h 在 namespace Lib_Motor 内部 include,
 * 故此处不再包裹 namespace, 避免双重命名空间。 */

/* ================================================================
 * CORDIC 初始化: 开启 AHB1 时钟, 之后常开
 * 调用点: Platform_Motor_Init() 开头 (在 MX_CORDIC_Init 之后, 冗余但安全)
 * ================================================================ */
inline void foc_math_cordic_init()
{
    __HAL_RCC_CORDIC_CLK_ENABLE();
}

/* 等待 RRDY 置位 (轮询, 约 6 周期/次迭代, PRECISION=0 时约 6 次迭代) */
inline void foc_cordic_wait_ready()
{
    while ((CORDIC->CSR & CORDIC_CSR_RRDY_Msk) == 0U)
    {
        /* 等待结果就绪 */
    }
}

/* float rad [-π, π] → q1.31
 * rad/π 映射到 [-1, 1), 再乘 2^31
 * 超出 [-π, π] 时先归一化 (调用方角度通常为 [0, 2π)) */
inline int32_t foc_rad_to_q31(float rad)
{
    /* 归一化到 [-π, π] */
    while (rad >  PI)  rad -= TWO_PI;
    while (rad < -PI)  rad += TWO_PI;
    /* rad/π ∈ [-1, 1], 乘 2^31 得 q1.31; 用 double 中间量避免溢出 */
    return static_cast<int32_t>(static_cast<double>(rad) *
                                (2147483648.0 / 3.14159265358979323846));
}

/* q1.31 → float [-1, 1) */
inline float foc_q31_to_float(int32_t q)
{
    return static_cast<float>(q) * (1.0f / 2147483648.0f);
}

/* ================================================================
 * sin + cos 联合接口 (CORDIC 后端)
 *
 * CORDIC 的 COSINE 和 SINE 是两个独立 FUNC, 各需一次写 WDATA + 读 RDATA。
 * 两次硬件调用合计约 12~16 周期 (含轮询), 仍远低于 libm 的 ~200 周期。
 * ================================================================ */
inline void foc_sin_cos_f32(float theta_rad, float& sin_v, float& cos_v)
{
    const int32_t q = foc_rad_to_q31(theta_rad);

    /* COSINE: FUNC=0x00, PRECISION=0 (最快, 24 位精度, 约 6 周期) */
    CORDIC->CSR = (0u << CORDIC_CSR_PRECISION_Pos)
                | (0x00u << CORDIC_CSR_FUNC_Pos);
    CORDIC->WDATA = static_cast<uint32_t>(q);
    foc_cordic_wait_ready();
    cos_v = foc_q31_to_float(static_cast<int32_t>(CORDIC->RDATA));

    /* SINE: FUNC=0x01 */
    CORDIC->CSR = (0u << CORDIC_CSR_PRECISION_Pos)
                | (0x01u << CORDIC_CSR_FUNC_Pos);
    CORDIC->WDATA = static_cast<uint32_t>(q);
    foc_cordic_wait_ready();
    sin_v = foc_q31_to_float(static_cast<int32_t>(CORDIC->RDATA));
}

/* ================================================================
 * atan2 / sqrt: 暂保留 libm
 * (CORDIC 输入要求 q1.31 [-1,1), 需归一化, 阶段 3 按需扩展)
 * ================================================================ */
inline float foc_atan2_f32(float y, float x)
{
    return atan2f(y, x);
}

inline float foc_sqrt_f32(float x)
{
    return sqrtf(x);
}
