/*
 * BSP_AS5600.cpp — AS5600 I2C 磁编码器 BSP 驱动
 *
 * 使用前需在 CubeMX 中:
 *   1. 配置一个 I2C 外设 (如 I2C1), Speed: Fast Mode 400kHz
 *   2. stm32g4xx_hal_conf.h 中取消注释 #define HAL_I2C_MODULE_ENABLED
 *   3. 取消下面 I2C 调用代码的注释
 *
 * 硬件连接:
 *   AS5600 VCC → 3.3V, GND → GND
 *   AS5600 SDA → I2C1_SDA, AS5600 SCL → I2C1_SCL
 *   AS5600 DIR → GND (顺时针为正)
 */

#include "BSP_AS5600.h"
// #include "main.h"           // I2C 配置后取消注释

#ifdef BSP_AS5600_ENABLE

// extern I2C_HandleTypeDef hi2c1;  // I2C 配置后取消注释

void BSP_AS5600_Init(void* ctx)
{
    AS5600_Ctx_t* dev = (AS5600_Ctx_t*)ctx;
    if (dev == nullptr) return;
    *dev = {0};
    // dev->i2c = &hi2c1;       // I2C 配置后绑定
}

bool BSP_AS5600_IsReady(void* ctx)
{
    AS5600_Ctx_t* dev = (AS5600_Ctx_t*)ctx;
    if (dev == nullptr || dev->i2c == nullptr) return false;

    // uint8_t status = 0;
    // HAL_I2C_Mem_Read(dev->i2c, AS5600_I2C_ADDR<<1,
    //     AS5600_REG_STATUS, 1, &status, 1, 100);
    // return (status & 0x20) != 0;

    (void)dev;
    return false;
}

Lib_Motor::SensorHealth BSP_AS5600_GetHealth(void* ctx)
{
    AS5600_Ctx_t* dev = (AS5600_Ctx_t*)ctx;
    if (dev == nullptr || dev->i2c == nullptr)
    {
        return Lib_Motor::SensorHealth::NOT_CONFIGURED;
    }

    /*
     * Reserve the hardware checks here once I2C access is enabled:
     * timeout -> BUS_TIMEOUT; missing/weak/strong magnet -> MAGNET_ERROR.
     */
    return Lib_Motor::SensorHealth::NOT_READY;
}

int32_t BSP_AS5600_GetRaw(void* ctx)
{
    AS5600_Ctx_t* dev = (AS5600_Ctx_t*)ctx;
    if (dev == nullptr || dev->i2c == nullptr) return 0;

    // uint8_t buf[2];
    // HAL_I2C_Mem_Read(dev->i2c, AS5600_I2C_ADDR<<1,
    //     AS5600_REG_ANGLE_H, 1, buf, 2, 100);
    // uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    // dev->last_raw = raw;
    // return (int32_t)raw;

    (void)dev;
    return 0;
}

float BSP_AS5600_ReadAngle(void* ctx)
{
    int32_t raw = BSP_AS5600_GetRaw(ctx);
    float angle = (float)raw * 6.283185307f / (float)AS5600_CPR;

    AS5600_Ctx_t* dev = (AS5600_Ctx_t*)ctx;
    if (dev != nullptr)
    {
        float diff = angle - dev->last_angle;
        while (diff > 3.14159265f)  diff -= 6.283185307f;
        while (diff < -3.14159265f) diff += 6.283185307f;
        dev->velocity = diff / 0.001f;
        dev->last_angle = angle;
    }
    return angle;
}

float BSP_AS5600_ReadVelocity(void* ctx)
{
    AS5600_Ctx_t* dev = (AS5600_Ctx_t*)ctx;
    if (dev == nullptr) return 0.0f;
    return dev->velocity;
}

void BSP_AS5600_Calibrate(void* ctx)
{
    AS5600_Ctx_t* dev = (AS5600_Ctx_t*)ctx;
    if (dev == nullptr) return;
    // 零位校准逻辑在此实现
    (void)dev;
}

#endif /* BSP_AS5600_ENABLE */
