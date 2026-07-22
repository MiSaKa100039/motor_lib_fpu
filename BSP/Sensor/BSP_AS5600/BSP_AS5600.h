/*
 * BSP_AS5600.h — AS5600 I2C 磁编码器 BSP 驱动
 *
 * AS5600 是 12-bit 非接触式磁编码器, I2C 接口
 * 0x0C (高8位) + 0x0D (低4位) = 12-bit 角度值
 *
 * 需要在 CubeMX 中配置 I2C 外设并启用
 */

#pragma once

#include "Motor_Sensor_Interface.h"
#include <stdint.h>
#include <stdbool.h>
#include "Sensor_Config.h"

#ifdef BSP_AS5600_ENABLE

/* AS5600 I2C 地址 (7-bit) */
#define AS5600_I2C_ADDR    0x36
#define AS5600_REG_ANGLE_H 0x0C
#define AS5600_REG_ANGLE_L 0x0D
#define AS5600_REG_STATUS  0x0B
#define AS5600_CPR          4096

typedef struct {
    void* i2c;               // I2C_HandleTypeDef* 指针
    uint16_t last_raw;       // 上次原始值
    float    last_angle;     // 上次角度
    float    velocity;       // 估算速度
} AS5600_Ctx_t;

#ifdef __cplusplus
extern "C" {
#endif

void     BSP_AS5600_Init(void* ctx);
bool     BSP_AS5600_IsReady(void* ctx);
Lib_Motor::SensorHealth BSP_AS5600_GetHealth(void* ctx);
float    BSP_AS5600_ReadAngle(void* ctx);
float    BSP_AS5600_ReadVelocity(void* ctx);
int32_t  BSP_AS5600_GetRaw(void* ctx);
void     BSP_AS5600_Calibrate(void* ctx);

#ifdef __cplusplus
}
#endif

#endif /* BSP_AS5600_ENABLE */
