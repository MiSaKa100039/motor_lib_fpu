/*
 * ABZ_Driver.cpp — ABZ 编码器算法实现
 */

#include "ABZ_Driver.h"

#ifdef BSP_ABZ_ENABLE

namespace Lib_Sensor
{

void ABZ_Driver_Init(ABZ_Driver_Ctx* drv, uint32_t cpr)
{
    if (drv == nullptr) return;
    *drv = {};
    drv->cpr = cpr;                  // 设置编码器CPR
}

float ABZ_Driver_ReadAngle(ABZ_Driver_Ctx* drv, int32_t raw_count, uint32_t now_tick, float kTwoPi)
{
    if (drv == nullptr || drv->cpr == 0U) return 0.0f;

    /*
     * TODO(sensor-frame-filter): add encoder-specific plausibility checks here:
     * max count delta per sample, optional Z-index consistency, and last-good
     * hold for one bad sample before reporting SensorHealth::ANGLE_JUMP.
     */
    // 计数取模: 映射到 [0, CPR) 范围
    int32_t cnt_mod = raw_count % static_cast<int32_t>(drv->cpr);
    if (cnt_mod < 0) cnt_mod += static_cast<int32_t>(drv->cpr);

    const float angle = static_cast<float>(cnt_mod) * kTwoPi / static_cast<float>(drv->cpr);

    drv->vel.Update(raw_count, drv->cpr, now_tick, kTwoPi); // 同时更新速度
    drv->last_angle = angle;
    return angle;
}

float ABZ_Driver_ReadVelocity(ABZ_Driver_Ctx* drv)
{
    if (drv == nullptr) return 0.0f;
    return drv->vel.velocity;
}

bool ABZ_Driver_IsReady(ABZ_Driver_Ctx* drv, bool hw_running, bool period_match)
{
    // 四个条件同时满足: 上下文有效 + CPR非零 + TIM运行 + ARR匹配
    return drv != nullptr && drv->cpr != 0U && hw_running && period_match;
}

Lib_Motor::SensorHealth ABZ_Driver_GetHealth(ABZ_Driver_Ctx* drv,
    bool init_ok, bool hw_running, bool period_match)
{
    // 健康状态优先级(由重到轻): 未配置 > 非法状态 > TIM未运行 > 未就绪 > 正常
    if (drv == nullptr)        return Lib_Motor::SensorHealth::NOT_CONFIGURED;
    if (!init_ok)              return Lib_Motor::SensorHealth::ILLEGAL_STATE;
    if (!hw_running)           return Lib_Motor::SensorHealth::TIMER_NOT_RUNNING;
    if (drv->cpr == 0U)        return Lib_Motor::SensorHealth::NOT_READY;
    if (!period_match)         return Lib_Motor::SensorHealth::ILLEGAL_STATE;
    return Lib_Motor::SensorHealth::OK;
}

void ABZ_Driver_Calibrate(ABZ_Driver_Ctx* drv, int32_t raw_count, uint32_t now_tick)
{
    if (drv == nullptr) return;
    // 校准: 记录当前位置为零点，清除之前的速度/角度值
    drv->vel.last_count = raw_count;
    drv->vel.last_tick  = now_tick;
    drv->vel.velocity   = 0.0f;
    drv->last_angle     = 0.0f;
}

} // namespace Lib_Sensor

#endif /* BSP_ABZ_ENABLE */
