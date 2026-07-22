/*
 * HALL3_Driver.cpp - 三路数字霍尔传感器算法实现
 */

#include "HALL3_Driver.h"

#ifdef BSP_HALL3_ENABLE

namespace Lib_Sensor
{

namespace
{
// 数学常量
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kSectorAngle = kPi / 3.0f;   // 每个扇区60度(电角度) = π/3

// 规范化扇区号: 0~5有效，超出范围返回无效扇区
uint8_t normalizeSector(uint8_t sector)
{
    return sector < 6U ? sector : HALL3_INVALID_SECTOR;
}

// 数值钳位到 [0, 1]
float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

// 角度归一化到 [0, 2π)
float wrapTwoPi(float angle)
{
    while (angle >= kTwoPi) angle -= kTwoPi;
    while (angle < 0.0f) angle += kTwoPi;
    return angle;
}

// 正余数运算: 保证结果始终在 [0, modulo) 范围
int32_t positiveModulo(int32_t value, int32_t modulo)
{
    int32_t result = value % modulo;
    if (result < 0) result += modulo;
    return result;
}

// 节拍数转微秒
uint32_t ticksToUs(uint32_t ticks, uint32_t ticks_per_second)
{
    if (ticks_per_second == 0U) return 0U;
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(ticks) * 1000000ULL) /
        static_cast<uint64_t>(ticks_per_second));
}

// 节拍数转秒
float ticksToSeconds(uint32_t ticks, uint32_t ticks_per_second)
{
    if (ticks_per_second == 0U) return 0.0f;
    return static_cast<float>(ticks) / static_cast<float>(ticks_per_second);
}

// 判断相邻扇区步进: +1=正向, -1=反向, 0=非法跳变
int8_t sectorStep(uint8_t previous, uint8_t current)
{
    int8_t delta = static_cast<int8_t>(current) - static_cast<int8_t>(previous);
    if (delta == 1 || delta == -5) return 1;
    if (delta == -1 || delta == 5) return -1;
    return 0;
}

// 计算扇区间最短距离(±3范围内): 用于非法跳变时的方向推测
int8_t shortestSectorDelta(uint8_t previous, uint8_t current)
{
    int8_t delta = static_cast<int8_t>(current) - static_cast<int8_t>(previous);
    if (delta > 3) delta -= 6;
    if (delta < -3) delta += 6;
    return delta;
}

// 应用低有效掩码得到归一化状态值
uint8_t normalizedState(const HALL3_Driver_Ctx* drv, uint8_t raw_state)
{
    return static_cast<uint8_t>((raw_state ^ drv->config.active_low_mask) & 0x07U);
}

// 查表将状态值转换为扇区号
uint8_t stateToSector(const HALL3_Driver_Ctx* drv, uint8_t state)
{
    return normalizeSector(drv->config.state_to_sector[state & 0x07U]);
}

// 检查数据是否过期: 长时间无新边沿则速度归零
void updateStaleVelocity(HALL3_Driver_Ctx* drv, uint32_t now_tick, uint32_t ticks_per_second)
{
    if (drv->valid == 0U ||
        drv->config.stale_timeout_us == 0U ||
        drv->last_edge_tick == 0U)
    {
        return;
    }

    const uint32_t elapsed_us = ticksToUs(now_tick - drv->last_edge_tick, ticks_per_second);
    if (elapsed_us > drv->config.stale_timeout_us)
    {
        drv->velocity = 0.0f;
        drv->direction = 0;
        drv->last_interval_ticks = 0U;
    }
}
} // namespace

// 设置标准120度霍尔序列映射: 001→0, 010→4, 011→5, 100→2, 101→1, 110→3
void HALL3_Driver_SetDefault120Map(uint8_t state_to_sector[HALL3_STATE_COUNT])
{
    if (state_to_sector == nullptr) return;

    state_to_sector[0] = HALL3_INVALID_SECTOR; // 000 - 非法
    state_to_sector[1] = 0U;                   // 001 -> 扇区0
    state_to_sector[2] = 4U;                   // 010 -> 扇区4
    state_to_sector[3] = 5U;                   // 011 -> 扇区5
    state_to_sector[4] = 2U;                   // 100 -> 扇区2
    state_to_sector[5] = 1U;                   // 101 -> 扇区1
    state_to_sector[6] = 3U;                   // 110 -> 扇区3
    state_to_sector[7] = HALL3_INVALID_SECTOR; // 111 - 非法
}

// 生成默认配置
void HALL3_Driver_DefaultConfig(HALL3_Driver_Config* cfg, uint8_t pole_pairs)
{
    if (cfg == nullptr) return;

    *cfg = {};
    cfg->pole_pairs = pole_pairs;
    cfg->active_low_mask = 0U;
    cfg->use_interpolation = 1U;
    cfg->stale_timeout_us = 100000U;     // 100ms超时
    cfg->min_edge_interval_us = 20U;     // 20us最小间隔
    cfg->velocity_filter_alpha = 0.35f;
    HALL3_Driver_SetDefault120Map(cfg->state_to_sector);
}

// 初始化Driver上下文
void HALL3_Driver_Init(HALL3_Driver_Ctx* drv, const HALL3_Driver_Config* cfg)
{
    if (drv == nullptr) return;

    HALL3_Driver_Config local_cfg;
    if (cfg != nullptr)
    {
        local_cfg = *cfg;
    }
    else
    {
        HALL3_Driver_DefaultConfig(&local_cfg, 1U); // 默认1对极
    }

    *drv = {};
    drv->sector = HALL3_INVALID_SECTOR;
    drv->health = Lib_Motor::SensorHealth::NOT_READY;
    HALL3_Driver_SetConfig(drv, &local_cfg);
}

// 设置(更新)配置: 规范化各参数
void HALL3_Driver_SetConfig(HALL3_Driver_Ctx* drv, const HALL3_Driver_Config* cfg)
{
    if (drv == nullptr || cfg == nullptr) return;

    drv->config = *cfg;
    drv->config.active_low_mask &= 0x07U;       // 仅保留低3位
    drv->config.use_interpolation = drv->config.use_interpolation != 0U ? 1U : 0U;
    drv->config.velocity_filter_alpha = clamp01(drv->config.velocity_filter_alpha);

    for (uint8_t i = 0U; i < HALL3_STATE_COUNT; ++i)
    {
        drv->config.state_to_sector[i] = normalizeSector(drv->config.state_to_sector[i]);
    }
}

// 核心状态机: 接收原始状态和时间戳，更新扇区/方向/速度/健康
void HALL3_Driver_Update(HALL3_Driver_Ctx* drv,
                         uint8_t raw_state,
                         uint32_t now_tick,
                         uint32_t ticks_per_second)
{
    if (drv == nullptr) return;

    drv->raw_state = static_cast<uint8_t>(raw_state & 0x07U);

    // 极对数为0时标记未配置
    if (drv->config.pole_pairs == 0U)
    {
        drv->valid = 0U;
        drv->sector = HALL3_INVALID_SECTOR;
        drv->velocity = 0.0f;
        drv->health = Lib_Motor::SensorHealth::NOT_CONFIGURED;
        return;
    }

    const uint8_t state = normalizedState(drv, drv->raw_state);
    const uint8_t sector = stateToSector(drv, state);
    if (sector == HALL3_INVALID_SECTOR)
    {
        // 非法状态(000或111): 标记无效，记录错误
        drv->state = state;
        drv->sector = HALL3_INVALID_SECTOR;
        drv->valid = 0U;
        drv->direction = 0;
        drv->velocity = 0.0f;
        drv->health = Lib_Motor::SensorHealth::ILLEGAL_STATE;
        drv->illegal_state_count++;
        return;
    }

    if (drv->valid == 0U)
    {
        // 首次识别到有效状态: 初始化上下文
        drv->state = state;
        drv->sector = sector;
        drv->valid = 1U;
        drv->direction = 0;
        drv->transition_count = sector;
        drv->last_edge_tick = now_tick;
        drv->last_interval_ticks = 0U;
        drv->velocity = 0.0f;
        drv->health = Lib_Motor::SensorHealth::OK;
        return;
    }

    if (state == drv->state)
    {
        // 状态未变: 检查信号是否过期
        updateStaleVelocity(drv, now_tick, ticks_per_second);
        return;
    }

    // 距离上次边沿的间隔过短，滤除高频噪声
    const uint32_t elapsed_ticks = now_tick - drv->last_edge_tick;
    if (drv->config.min_edge_interval_us != 0U &&
        ticksToUs(elapsed_ticks, ticks_per_second) < drv->config.min_edge_interval_us)
    {
        return;
    }

    int8_t step = sectorStep(drv->sector, sector);
    if (step == 0)
    {
        /*
         * TODO(sensor-frame-filter): treat a single impossible Hall jump as a
         * bad edge and keep the last-good sector; only fault after consecutive
         * bad jumps or a configurable timeout.
         */
        // 非法跳变: 用最短路径推测方向，记录错误
        step = shortestSectorDelta(drv->sector, sector);
        drv->invalid_transition_count++;
        drv->health = Lib_Motor::SensorHealth::ILLEGAL_STATE;
    }
    else
    {
        drv->health = Lib_Motor::SensorHealth::OK;
    }

    drv->transition_count += step;
    drv->state = state;
    drv->sector = sector;
    drv->direction = step > 0 ? 1 : (step < 0 ? -1 : 0);
    drv->edge_count++;

    if (step == 1 || step == -1)
    {
        // 正常跳变: 更新速度(低通滤波)
        const float dt_s = ticksToSeconds(elapsed_ticks, ticks_per_second);
        if (dt_s > 0.0f)
        {
            const float instant_velocity =
                static_cast<float>(step) * kSectorAngle /
                (static_cast<float>(drv->config.pole_pairs) * dt_s);
            const float alpha = drv->config.velocity_filter_alpha;
            drv->velocity += alpha * (instant_velocity - drv->velocity);
            drv->last_interval_ticks = elapsed_ticks;
        }
    }
    else
    {
        // 跳过了扇区(>1步): 清空速度估算
        drv->velocity = 0.0f;
        drv->last_interval_ticks = 0U;
    }

    drv->last_edge_tick = now_tick;
}

// 计算当前机械角度(含插值)
float HALL3_Driver_ReadAngle(HALL3_Driver_Ctx* drv,
                             uint32_t now_tick,
                             uint32_t ticks_per_second)
{
    if (drv == nullptr || drv->valid == 0U || drv->config.pole_pairs == 0U)
    {
        return 0.0f;
    }

    // 每机械圈扇区数 = 6(电圈扇区) × 极对数
    const int32_t sectors_per_mech_rev =
        static_cast<int32_t>(6U * drv->config.pole_pairs);
    const int32_t count_mod = positiveModulo(drv->transition_count, sectors_per_mech_rev);
    float sector_position = static_cast<float>(count_mod) + 0.5f; // 扇区中点

    // 角度插值: 在两个霍尔边沿之间按时间比例估算当前位置
    if (drv->config.use_interpolation != 0U &&
        drv->direction != 0 &&
        drv->last_interval_ticks != 0U)
    {
        const uint32_t elapsed_ticks = now_tick - drv->last_edge_tick;
        const float fraction = static_cast<float>(elapsed_ticks) /
                               static_cast<float>(drv->last_interval_ticks);
        sector_position += static_cast<float>(drv->direction) * (clamp01(fraction) - 0.5f);
    }

    // 转换为机械角度 [0, 2π)
    drv->last_angle =
        wrapTwoPi(sector_position * kSectorAngle / static_cast<float>(drv->config.pole_pairs));
    (void)ticks_per_second;
    return drv->last_angle;
}

float HALL3_Driver_ReadVelocity(const HALL3_Driver_Ctx* drv)
{
    if (drv == nullptr) return 0.0f;
    return drv->velocity;
}

int32_t HALL3_Driver_GetRaw(const HALL3_Driver_Ctx* drv)
{
    if (drv == nullptr) return 0;
    return drv->transition_count;
}

// 就绪条件: 上下文有效 + 已识别 + 健康状态OK
bool HALL3_Driver_IsReady(const HALL3_Driver_Ctx* drv)
{
    return drv != nullptr &&
           drv->valid != 0U &&
           drv->health == Lib_Motor::SensorHealth::OK;
}

Lib_Motor::SensorHealth HALL3_Driver_GetHealth(const HALL3_Driver_Ctx* drv)
{
    if (drv == nullptr) return Lib_Motor::SensorHealth::NOT_CONFIGURED;
    return drv->health;
}

// 校准: 以当前扇区为零点重置累计位置
void HALL3_Driver_Calibrate(HALL3_Driver_Ctx* drv, uint32_t now_tick)
{
    if (drv == nullptr || drv->valid == 0U) return;

    drv->transition_count = drv->sector;
    drv->velocity = 0.0f;
    drv->direction = 0;
    drv->last_interval_ticks = 0U;
    drv->last_edge_tick = now_tick;
    drv->health = Lib_Motor::SensorHealth::OK;
}

} // namespace Lib_Sensor

#endif /* BSP_HALL3_ENABLE */
