/*
 * Platform_HALL3.cpp - 将 BSP_HALL3 + Lib_Sensor HALL3 绑定到 Motor SensorInterface_t
 */

#include "Platform_HALL3.h"
#include "HALL3/HALL3_Driver.h"

#ifdef BSP_HALL3_ENABLE

namespace
{
// 规范化扇区号: 有效性范围0~5，超出返回无效扇区常量
uint8_t normalizeSector(uint8_t sector)
{
    return sector < 6U ? sector : Lib_Sensor::HALL3_INVALID_SECTOR;
}
} // namespace

extern Lib_Motor::SensorInterface_t g_sensor_hall3;

namespace
{
// 运行时实例: 包含BSP硬件和Driver算法上下文
struct Platform_HALL3_Runtime
{
    BSP_HALL3_HW_t hw;
    Lib_Sensor::HALL3_Driver_Ctx drv;
};

static Platform_HALL3_Runtime g_hall3;         // 全局运行时实例
static Platform_HALL3_Config_t g_hall3_config; // 全局配置缓存
static bool g_hall3_config_loaded = false;     // 配置是否已加载

// 更新调试变量，来自Driver层计算结果
void publishDebug(const Lib_Sensor::HALL3_Driver_Ctx* drv)
{
    if (drv == nullptr) return;

    BSP_HALL3_Debug_Sector = drv->sector;
    BSP_HALL3_Debug_TransitionCount = drv->transition_count;
    BSP_HALL3_Debug_EdgeCount = drv->edge_count;
    BSP_HALL3_Debug_InvalidCount =
        drv->illegal_state_count + drv->invalid_transition_count;
}

// 将平台配置转换为Driver层配置格式
void buildDriverConfig(const Platform_HALL3_Config_t& src,
                       Lib_Sensor::HALL3_Driver_Config* dst)
{
    if (dst == nullptr) return;

    dst->pole_pairs = src.pole_pairs;
    dst->active_low_mask = static_cast<uint8_t>(src.active_low_mask & 0x07U);
    dst->use_interpolation = src.use_interpolation != 0U ? 1U : 0U;
    dst->stale_timeout_us = src.stale_timeout_us;
    dst->min_edge_interval_us = src.min_edge_interval_us;
    dst->velocity_filter_alpha = src.velocity_filter_alpha;
    for (uint8_t i = 0U; i < Lib_Sensor::HALL3_STATE_COUNT; ++i)
    {
        dst->state_to_sector[i] = normalizeSector(src.state_to_sector[i]);
    }
}

// 根据当前极对数更新传感器接口规格(CPR和电气偏移)
void updateSensorSpec()
{
    ::g_sensor_hall3.spec.direction = g_hall3_config.direction;
    ::g_sensor_hall3.spec.cpr = 6U * static_cast<uint32_t>(g_hall3_config.pole_pairs);
    ::g_sensor_hall3.spec.offset_elec = g_hall3_config.offset_elec_rad;
}

// 确保配置已加载: 首次调用时加载默认值
void ensureConfigLoaded()
{
    if (!g_hall3_config_loaded)
    {
        Platform_HALL3_LoadDefaultConfig(&g_hall3_config);
        g_hall3_config_loaded = true;
        updateSensorSpec();
    }
}

// 从硬件读取最新状态并送入Driver算法更新
void updateFromHardware(Platform_HALL3_Runtime* rt)
{
    if (rt == nullptr) return;

    const uint8_t raw = BSP_HALL3_ReadRawState(&rt->hw);
    Lib_Sensor::HALL3_Driver_Update(
        &rt->drv, raw, BSP_HALL3_NowTicks(), BSP_HALL3_TicksPerSecond());
    publishDebug(&rt->drv);
}

// EXTI边沿回调: 由BSP层在中断中触发，通知Driver处理
void hallEdgeCallback(uint8_t raw_state,
                      uint32_t now_tick,
                      uint32_t ticks_per_second,
                      void* user)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(user);
    if (rt == nullptr) return;

    Lib_Sensor::HALL3_Driver_Update(&rt->drv, raw_state, now_tick, ticks_per_second);
    publishDebug(&rt->drv);
}

// Wrapper: 初始化硬件 → 注册回调 → 初始化算法 → 首次读数
void Platform_HALL3_Init(void* ctx)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(ctx);
    if (rt == nullptr) return;

    ensureConfigLoaded();

    BSP_HALL3_HW_Init(&rt->hw);
    BSP_HALL3_RegisterEdgeCallback(&rt->hw, hallEdgeCallback, rt);

    Lib_Sensor::HALL3_Driver_Config drv_cfg;
    buildDriverConfig(g_hall3_config, &drv_cfg);
    Lib_Sensor::HALL3_Driver_Init(&rt->drv, &drv_cfg);

    updateFromHardware(rt);
    rt->drv.last_angle = Lib_Sensor::HALL3_Driver_ReadAngle(
        &rt->drv, BSP_HALL3_NowTicks(), BSP_HALL3_TicksPerSecond());
}

// Wrapper: 检查硬件+算法就绪状态
bool Platform_HALL3_IsReady(void* ctx)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(ctx);
    if (rt == nullptr || !BSP_HALL3_IsHardwareReady(&rt->hw)) return false;

    updateFromHardware(rt);
    return Lib_Sensor::HALL3_Driver_IsReady(&rt->drv);
}

// Wrapper: 读取机械角度(硬件→算法→插值计算)
float Platform_HALL3_ReadAngle(void* ctx)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(ctx);
    if (rt == nullptr) return 0.0f;

    updateFromHardware(rt);
    return Lib_Sensor::HALL3_Driver_ReadAngle(
        &rt->drv, BSP_HALL3_NowTicks(), BSP_HALL3_TicksPerSecond());
}

// Wrapper: 读取估算速度
float Platform_HALL3_ReadVelocity(void* ctx)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(ctx);
    if (rt == nullptr) return 0.0f;

    updateFromHardware(rt);
    return Lib_Sensor::HALL3_Driver_ReadVelocity(&rt->drv);
}

// Wrapper: 获取原始跳变计数
int32_t Platform_HALL3_GetRaw(void* ctx)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(ctx);
    if (rt == nullptr) return 0;

    updateFromHardware(rt);
    return Lib_Sensor::HALL3_Driver_GetRaw(&rt->drv);
}

// Wrapper: 健康诊断(硬件就绪+算法状态)
Lib_Motor::SensorHealth Platform_HALL3_GetHealth(void* ctx)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(ctx);
    if (rt == nullptr) return Lib_Motor::SensorHealth::NOT_CONFIGURED;
    if (!BSP_HALL3_IsHardwareReady(&rt->hw)) return Lib_Motor::SensorHealth::NOT_READY;

    updateFromHardware(rt);
    return Lib_Sensor::HALL3_Driver_GetHealth(&rt->drv);
}

// Wrapper: 校准(以当前扇区为零点)
void Platform_HALL3_Calibrate(void* ctx)
{
    Platform_HALL3_Runtime* rt = static_cast<Platform_HALL3_Runtime*>(ctx);
    if (rt == nullptr) return;

    updateFromHardware(rt);
    Lib_Sensor::HALL3_Driver_Calibrate(&rt->drv, BSP_HALL3_NowTicks());
    publishDebug(&rt->drv);
}

} // namespace

// 传感器接口实例: 绑定所有Wrapper函数指针到统一接口
Lib_Motor::SensorInterface_t g_sensor_hall3 =
{
    .read_angle    = Platform_HALL3_ReadAngle,
    .is_ready      = Platform_HALL3_IsReady,
    .read_velocity = Platform_HALL3_ReadVelocity,
    .get_raw       = Platform_HALL3_GetRaw,
    .get_health    = Platform_HALL3_GetHealth,
    .init          = Platform_HALL3_Init,
    .calibrate     = Platform_HALL3_Calibrate,
    .ctx           = &g_hall3,
    .spec          = {
        .direction = 1,
        .cpr = 6U * static_cast<uint32_t>(PLATFORM_HALL3_POLE_PAIRS_DEFAULT),
        .offset_elec = PLATFORM_HALL3_OFFSET_ELEC_RAD,
    },
};

Lib_Motor::SensorInterface_t* Platform_Get_HALL3(void)
{
    ensureConfigLoaded();
    return &g_sensor_hall3;
}

bool Platform_HALL3_IsEnabled(void)
{
    return true;
}

// 从宏加载默认配置值
void Platform_HALL3_LoadDefaultConfig(Platform_HALL3_Config_t* cfg)
{
    if (cfg == nullptr) return;

    *cfg = {};
    cfg->direction = PLATFORM_HALL3_DIRECTION;
    cfg->pole_pairs = PLATFORM_HALL3_POLE_PAIRS_DEFAULT;
    cfg->offset_elec_rad = PLATFORM_HALL3_OFFSET_ELEC_RAD;
    cfg->active_low_mask = static_cast<uint8_t>(PLATFORM_HALL3_ACTIVE_LOW_MASK & 0x07U);
    cfg->use_interpolation = PLATFORM_HALL3_USE_INTERPOLATION != 0 ? 1U : 0U;
    cfg->stale_timeout_us = PLATFORM_HALL3_STALE_TIMEOUT_US;
    cfg->min_edge_interval_us = PLATFORM_HALL3_MIN_EDGE_INTERVAL_US;
    cfg->velocity_filter_alpha = PLATFORM_HALL3_VELOCITY_FILTER_ALPHA;
    cfg->state_to_sector[0] = normalizeSector(PLATFORM_HALL3_SECTOR_000);
    cfg->state_to_sector[1] = normalizeSector(PLATFORM_HALL3_SECTOR_001);
    cfg->state_to_sector[2] = normalizeSector(PLATFORM_HALL3_SECTOR_010);
    cfg->state_to_sector[3] = normalizeSector(PLATFORM_HALL3_SECTOR_011);
    cfg->state_to_sector[4] = normalizeSector(PLATFORM_HALL3_SECTOR_100);
    cfg->state_to_sector[5] = normalizeSector(PLATFORM_HALL3_SECTOR_101);
    cfg->state_to_sector[6] = normalizeSector(PLATFORM_HALL3_SECTOR_110);
    cfg->state_to_sector[7] = normalizeSector(PLATFORM_HALL3_SECTOR_111);
}

// 应用配置: 规范化参数后同步到Driver层和接口规格
void Platform_HALL3_ApplyConfig(const Platform_HALL3_Config_t* cfg)
{
    if (cfg == nullptr) return;

    g_hall3_config = *cfg;
    g_hall3_config.active_low_mask &= 0x07U;
    g_hall3_config.use_interpolation = g_hall3_config.use_interpolation != 0U ? 1U : 0U;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        g_hall3_config.state_to_sector[i] = normalizeSector(g_hall3_config.state_to_sector[i]);
    }

    g_hall3_config_loaded = true;
    updateSensorSpec();

    Lib_Sensor::HALL3_Driver_Config drv_cfg;
    buildDriverConfig(g_hall3_config, &drv_cfg);
    Lib_Sensor::HALL3_Driver_SetConfig(&g_hall3.drv, &drv_cfg);
}

void Platform_HALL3_GetConfig(Platform_HALL3_Config_t* cfg)
{
    if (cfg == nullptr) return;
    ensureConfigLoaded();
    *cfg = g_hall3_config;
}

// 运行时修改极对数: 影响CPR(6*极对数)并同步到Driver
void Platform_HALL3_SetDirection(int8_t direction)
{
    ensureConfigLoaded();
    if (direction != 1 && direction != -1) return;
    Platform_HALL3_Config_t cfg = g_hall3_config;
    cfg.direction = direction;
    Platform_HALL3_ApplyConfig(&cfg);
}

void Platform_HALL3_SetPolePairs(uint8_t pole_pairs)
{
    ensureConfigLoaded();
    if (pole_pairs == 0U) return;
    Platform_HALL3_Config_t cfg = g_hall3_config;
    cfg.pole_pairs = pole_pairs;
    Platform_HALL3_ApplyConfig(&cfg);
}

void Platform_HALL3_SetElectricalOffset(float offset_elec_rad)
{
    ensureConfigLoaded();
    Platform_HALL3_Config_t cfg = g_hall3_config;
    cfg.offset_elec_rad = offset_elec_rad;
    Platform_HALL3_ApplyConfig(&cfg);
}

void Platform_HALL3_SetActiveLowMask(uint8_t active_low_mask)
{
    ensureConfigLoaded();
    Platform_HALL3_Config_t cfg = g_hall3_config;
    cfg.active_low_mask = static_cast<uint8_t>(active_low_mask & 0x07U);
    Platform_HALL3_ApplyConfig(&cfg);
}

void Platform_HALL3_SetStateToSector(const uint8_t state_to_sector[8])
{
    if (state_to_sector == nullptr) return;
    ensureConfigLoaded();
    Platform_HALL3_Config_t cfg = g_hall3_config;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        cfg.state_to_sector[i] = normalizeSector(state_to_sector[i]);
    }
    Platform_HALL3_ApplyConfig(&cfg);
}

#endif /* BSP_HALL3_ENABLE */
