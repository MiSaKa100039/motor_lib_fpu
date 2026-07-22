/*
 * AS5600_Driver.cpp — AS5600 磁编码器算法实现
 */

#include "AS5600_Driver.h"

#ifdef BSP_AS5600_ENABLE

namespace Lib_Sensor
{

void AS5600_Driver_Init(AS5600_Driver_Ctx* drv)
{
    if (drv == nullptr) return;
    *drv = {};
}

float AS5600_Driver_ReadAngle(AS5600_Driver_Ctx* drv, int32_t raw_value, uint32_t now_tick, float kTwoPi)
{
    if (drv == nullptr) return 0.0f;

    /*
     * TODO(sensor-frame-filter): validate magnetic/status bits and bus errors
     * before accepting a frame; reject out-of-range raw values and apply a
     * max-angle-step check before updating last_angle/velocity.
     */
    // AS5600输出12位原始值(0~4095)，直接按比例映射到 [0, 2π)
    const float angle = static_cast<float>(raw_value) * kTwoPi / static_cast<float>(AS5600_CPR);

    drv->vel.Update(raw_value, AS5600_CPR, now_tick, kTwoPi); // 同时更新速度
    drv->last_angle = angle;
    return angle;
}

float AS5600_Driver_ReadVelocity(AS5600_Driver_Ctx* drv)
{
    if (drv == nullptr) return 0.0f;
    return drv->vel.velocity;
}

bool AS5600_Driver_IsReady(AS5600_Driver_Ctx* drv, int32_t raw_value)
{
    (void)drv;
    (void)raw_value;
    return false;
}

Lib_Motor::SensorHealth AS5600_Driver_GetHealth(AS5600_Driver_Ctx* drv, bool i2c_ok)
{
    // I2C通信失败则NOT_READY，否则OK
    if (drv == nullptr) return Lib_Motor::SensorHealth::NOT_CONFIGURED;
    if (!i2c_ok)         return Lib_Motor::SensorHealth::NOT_READY;
    return Lib_Motor::SensorHealth::OK;
}

void AS5600_Driver_Calibrate(AS5600_Driver_Ctx* drv, int32_t raw_value, uint32_t now_tick)
{
    if (drv == nullptr) return;
    // 校准: 记录当前值作为参考，清除之前的速度/角度
    drv->vel.last_count = raw_value;
    drv->vel.last_tick  = now_tick;
    drv->vel.velocity   = 0.0f;
    drv->last_angle     = 0.0f;
}

} // namespace Lib_Sensor

#endif /* BSP_AS5600_ENABLE */
