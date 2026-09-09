/*
 * ABZ_Driver.h — ABZ 编码器算法层
 *
 * 纯算法，零硬件依赖。通过参数接收原始计数值和硬件状态，
 * 输出角度/速度/健康诊断。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Motor_Definitions.h"
#include "Sensor_Config.h"

#ifdef BSP_ABZ_ENABLE

namespace Lib_Sensor
{

// 速度估算器: 基于位置差分+过零翻转处理
struct ABZ_Velocity
{
    int32_t  last_count;     // 上一次采样计数值
    float    velocity;       // 当前速度(rad/s)
    uint32_t last_tick;      // 上一次采样时间戳(ms)

    // 更新速度: delta=当前计数-上次计数, 自动处理计数器溢出翻转(半圈检测)
    float Update(int32_t current_count, uint32_t cpr, uint32_t now_tick, float kTwoPi)
    {
        const uint32_t elapsed_ms = now_tick - last_tick;
        if (elapsed_ms == 0U || cpr == 0U)
        {
            last_count = current_count;
            last_tick  = now_tick;
            return velocity;
        }

        int32_t delta = current_count - last_count;
        const int32_t half_cpr = static_cast<int32_t>(cpr / 2U);       // 半圈阈值
        const int32_t signed_cpr = static_cast<int32_t>(cpr);

        // 若位移超过半圈，认为是计数器溢出翻转而非真实位移
        if (delta > half_cpr)  delta -= signed_cpr;
        if (delta < -half_cpr) delta += signed_cpr;

        velocity = static_cast<float>(delta) * kTwoPi * 1000.0f /
                   (static_cast<float>(cpr) * static_cast<float>(elapsed_ms));

        last_count = current_count;
        last_tick  = now_tick;
        return velocity;
    }
};

// ABZ编码器算法上下文
struct ABZ_Driver_Ctx
{
    uint32_t    cpr;          // 每圈计数(Counts Per Revolution)
    ABZ_Velocity vel;         // 速度估算器
    float       last_angle;   // 上一次计算的角度
};

void     ABZ_Driver_Init(ABZ_Driver_Ctx* drv, uint32_t cpr);
float    ABZ_Driver_ReadAngle(ABZ_Driver_Ctx* drv, int32_t raw_count, uint32_t now_tick, float kTwoPi);
float    ABZ_Driver_ReadVelocity(ABZ_Driver_Ctx* drv);
bool     ABZ_Driver_IsReady(ABZ_Driver_Ctx* drv, bool hw_running, bool period_match);
Lib_Motor::SensorHealth ABZ_Driver_GetHealth(ABZ_Driver_Ctx* drv,
                          bool init_ok, bool hw_running, bool period_match);
void     ABZ_Driver_Calibrate(ABZ_Driver_Ctx* drv, int32_t raw_count, uint32_t now_tick);

} // namespace Lib_Sensor

#endif /* BSP_ABZ_ENABLE */
