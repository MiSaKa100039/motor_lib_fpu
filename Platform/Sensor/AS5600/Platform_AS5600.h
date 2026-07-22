/*
 * Platform_AS5600.h — AS5600 传感器平台层
 *
 * 绑定 BSP 层函数到 SensorInterface_t, 供 Observer_Sensor 调用
 */

#pragma once

#include "Motor_Sensor_Interface.h"

#ifdef __cplusplus
extern "C" {
#endif

Lib_Motor::SensorInterface_t* Platform_Get_AS5600(void);

#ifdef __cplusplus
}
#endif
