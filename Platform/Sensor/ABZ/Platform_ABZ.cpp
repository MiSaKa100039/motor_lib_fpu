/*
 * Platform_ABZ.cpp — ABZ 正交编码器平台层
 *
 * 绑定 BSP_ABZ 函数 → SensorInterface_t
 * 在 Platform_Motor.cpp 中通过 cfg.position.rotor_sensor = Platform_Get_ABZ() 注入
 */

#include "Platform_ABZ.h"
#include "BSP_ABZ.h"

#ifdef BSP_ABZ_ENABLE

/* ============================================================
 * 传感器上下文 (ABZ_Ctx_t 在 BSP_ABZ.h 中定义)
 * ============================================================ */
static ABZ_Ctx_t g_abz;

static Lib_Motor::SensorInterface_t g_sensor_abz =
{
    .read_angle    = BSP_ABZ_ReadAngle,
    .is_ready      = BSP_ABZ_IsReady,
    .read_velocity = BSP_ABZ_ReadVelocity,
    .get_raw       = BSP_ABZ_GetRaw,
    .get_health    = BSP_ABZ_GetHealth,
    .init          = BSP_ABZ_Init,
    .calibrate     = BSP_ABZ_Calibrate,
    .ctx           = &g_abz,
    .spec          = { .direction = 1, .cpr = BSP_ABZ_CPR, .offset_elec = 0.0f },
};

Lib_Motor::SensorInterface_t* Platform_Get_ABZ(void)
{
    return &g_sensor_abz;
}

#endif /* BSP_ABZ_ENABLE */
