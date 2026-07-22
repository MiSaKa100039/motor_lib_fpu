/*
 * BSP_Sensor_Example.h 模板
 *
 * 设备自身检查在 BSP 实现，并通过 SensorHealth 报告给 Lib。
 */

#pragma once

#include "Motor_Sensor_Interface.h"

typedef struct
{
    void* bus_or_timer;
    int32_t last_raw;
    float velocity;
} SensorExample_Ctx_t;

#ifdef __cplusplus
extern "C" {
#endif

void BSP_SensorExample_Init(void* ctx);
bool BSP_SensorExample_IsReady(void* ctx);
Lib_Motor::SensorHealth BSP_SensorExample_GetHealth(void* ctx);
float BSP_SensorExample_ReadAngle(void* ctx);
float BSP_SensorExample_ReadVelocity(void* ctx);
int32_t BSP_SensorExample_GetRaw(void* ctx);
void BSP_SensorExample_Calibrate(void* ctx);

#ifdef __cplusplus
}
#endif

