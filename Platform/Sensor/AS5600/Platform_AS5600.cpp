/*
 * Platform_AS5600.cpp — AS5600 传感器平台层
 *
 * 绑定 BSP 函数 → SensorInterface_t, 对 Lib 暴露统一接口
 */

#include "Platform_AS5600.h"
#include "BSP_AS5600.h"

#ifdef BSP_AS5600_ENABLE

/* ============================================================
 * 传感器实例 + 上下文
 * ============================================================ */
static AS5600_Ctx_t g_as5600;

static Lib_Motor::SensorInterface_t g_sensor_as5600 =
{
    .read_angle    = BSP_AS5600_ReadAngle,
    .is_ready      = BSP_AS5600_IsReady,
    .read_velocity = BSP_AS5600_ReadVelocity,
    .get_raw       = BSP_AS5600_GetRaw,
    .get_health    = BSP_AS5600_GetHealth,
    .init          = BSP_AS5600_Init,
    .calibrate     = BSP_AS5600_Calibrate,
    .ctx           = &g_as5600,
    .spec          = { .direction = -1, .cpr = 4096, .offset_elec = 0.0f },
};

/* ============================================================
 * 对外接口: 返回传感器实例指针
 * ============================================================ */
Lib_Motor::SensorInterface_t* Platform_Get_AS5600(void)
{
    return &g_sensor_as5600;
}

#endif /* BSP_AS5600_ENABLE */
