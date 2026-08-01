/*
 * Platform_ABZ.h — ABZ 正交编码器平台层
 *
 * 绑定 BSP_ABZ 函数到 SensorInterface_t, 供 Lib 调用
 */

#pragma once

#include "Motor_Sensor_Interface.h"

#ifdef BSP_ABZ_ENABLE

#ifdef __cplusplus
extern "C" {
#endif

Lib_Motor::SensorInterface_t* Platform_Get_ABZ(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ABZ_ENABLE */
