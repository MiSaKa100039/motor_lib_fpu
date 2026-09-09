/*
 * AS5600_Driver.h — AS5600 磁编码器算法层
 *
 * 纯算法，零硬件依赖。通过参数接收原始角度值，
 * 输出角度/速度/健康诊断。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Motor_Definitions.h"
#include "Sensor_Config.h"

#ifdef BSP_AS5600_ENABLE

namespace Lib_Sensor
{

constexpr uint32_t AS5600_CPR = 4096U;  // AS5600 12位ADC分辨率 = 4096 counts/圈

// 速度估算器: 与ABZ完全相同的逻辑，仅CPR固定为4096
struct AS5600_Velocity
{
    int32_t  last_count;     // 上一次采样原始值
    float    velocity;       // 当前速度(rad/s)
    uint32_t last_tick;      // 上一次采样时间戳(ms)

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
        const int32_t half_cpr = static_cast<int32_t>(cpr / 2U);
        const int32_t signed_cpr = static_cast<int32_t>(cpr);

        if (delta > half_cpr)  delta -= signed_cpr;   // 正向溢出翻转
        if (delta < -half_cpr) delta += signed_cpr;   // 反向溢出翻转

        velocity = static_cast<float>(delta) * kTwoPi * 1000.0f /
                   (static_cast<float>(cpr) * static_cast<float>(elapsed_ms));

        last_count = current_count;
        last_tick  = now_tick;
        return velocity;
    }
};

// AS5600编码器算法上下文
struct AS5600_Driver_Ctx
{
    AS5600_Velocity vel;     // 速度估算器
    float    last_angle;     // 上一次计算的角度
};

void     AS5600_Driver_Init(AS5600_Driver_Ctx* drv);
float    AS5600_Driver_ReadAngle(AS5600_Driver_Ctx* drv, int32_t raw_value, uint32_t now_tick, float kTwoPi);
float    AS5600_Driver_ReadVelocity(AS5600_Driver_Ctx* drv);
bool     AS5600_Driver_IsReady(AS5600_Driver_Ctx* drv, int32_t raw_value);
Lib_Motor::SensorHealth AS5600_Driver_GetHealth(AS5600_Driver_Ctx* drv, bool i2c_ok);
void     AS5600_Driver_Calibrate(AS5600_Driver_Ctx* drv, int32_t raw_value, uint32_t now_tick);

} // namespace Lib_Sensor

#endif /* BSP_AS5600_ENABLE */
