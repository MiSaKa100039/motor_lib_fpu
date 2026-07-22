/*
 * BSP_ABZ.cpp — ABZ 正交编码器 BSP 实现
 *
 * STM32G474 + TIM3 编码器模式
 * PC6 = TIM3_CH1 (A),  PC7 = TIM3_CH2 (B),  PC8 = TIM3_CH3 (Z)
 *
 * 使用前:
 *   1. 按 README.md 在 CubeMX 中配置 TIM3 编码器模式
 *   2. 未知编码器先以 BSP_ABZ_CPR=0 测量原始 CNT
 *   3. 闭环运行前填写 BSP_ABZ_CPR，并将 TIM3 ARR 配置为 CPR - 1
 *
 * 工作原理:
 *   TIM3 在编码器模式下 CNT 随 A/B 相位自动增减
 *   每转 CNT 变化量 = 4 × PPR (编码器线数)
 *   读取 CNT → 转换为机械角度
 */

#include "BSP_ABZ.h"
#include "main.h"

#ifdef BSP_ABZ_ENABLE

extern "C" {
extern TIM_HandleTypeDef htim3;
}

void BSP_ABZ_Init(void* ctx)
{
    ABZ_Ctx_t* dev = (ABZ_Ctx_t*)ctx;
    if (dev == nullptr) return;
    *dev = {0};

    dev->tim = &htim3;
    dev->cpr = BSP_ABZ_CPR;
    HAL_TIM_Encoder_Start(static_cast<TIM_HandleTypeDef*>(dev->tim), TIM_CHANNEL_ALL);
}

bool BSP_ABZ_IsReady(void* ctx)
{
    ABZ_Ctx_t* dev = (ABZ_Ctx_t*)ctx;
    return dev != nullptr && dev->tim != nullptr && dev->cpr != 0U &&
           BSP_ABZ_COUNTER_READ_ENABLED != 0;
}

Lib_Motor::SensorHealth BSP_ABZ_GetHealth(void* ctx)
{
    ABZ_Ctx_t* dev = (ABZ_Ctx_t*)ctx;
    if (dev == nullptr || dev->tim == nullptr || BSP_ABZ_COUNTER_READ_ENABLED == 0)
    {
        return Lib_Motor::SensorHealth::TIMER_NOT_RUNNING;
    }
    if (dev->cpr == 0U)
    {
        return Lib_Motor::SensorHealth::NOT_READY;
    }
    return Lib_Motor::SensorHealth::OK;
}

/* ============================================================
 * 读取当前计数值 (TIM3 CNT 寄存器)
 * ============================================================ */
int32_t BSP_ABZ_GetRaw(void* ctx)
{
    ABZ_Ctx_t* dev = (ABZ_Ctx_t*)ctx;
    if (dev == nullptr || dev->tim == nullptr) return 0;

    TIM_HandleTypeDef* htim = static_cast<TIM_HandleTypeDef*>(dev->tim);
    return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim));
}

/* ============================================================
 * 读取机械角度 (rad), [0, 2π)
 *
 * 通过 CNT 寄存器值计算角度:
 *   angle = (cnt % CPR) / CPR × 2π
 *
 * 速度通过角度差分估算:
 *   velocity = Δangle / Δt
 * ============================================================ */
float BSP_ABZ_ReadAngle(void* ctx)
{
    ABZ_Ctx_t* dev = (ABZ_Ctx_t*)ctx;
    if (dev == nullptr || dev->tim == nullptr || dev->cpr == 0U) return 0.0f;

    int32_t cnt = BSP_ABZ_GetRaw(ctx);
    uint32_t cpr = dev->cpr;

    int32_t cnt_mod = cnt % (int32_t)cpr;
    if (cnt_mod < 0) cnt_mod += cpr;

    float angle = (float)cnt_mod * 6.283185307f / (float)cpr;

    // 速度估算: 角度差分
    float diff = angle - dev->last_angle;
    while (diff > 3.14159265f)  diff -= 6.283185307f;
    while (diff < -3.14159265f) diff += 6.283185307f;
    dev->velocity = diff / 0.001f;
    dev->last_angle = angle;

    return angle;
}

float BSP_ABZ_ReadVelocity(void* ctx)
{
    ABZ_Ctx_t* dev = (ABZ_Ctx_t*)ctx;
    if (dev == nullptr) return 0.0f;
    return dev->velocity;
}

/* ============================================================
 * Z 脉冲校准: 当检测到 Z 索引脉冲时复位角度零点
 * ============================================================ */
void BSP_ABZ_Calibrate(void* ctx)
{
    ABZ_Ctx_t* dev = (ABZ_Ctx_t*)ctx;
    if (dev == nullptr || dev->tim == nullptr) return;

    // 可选: 硬复位 TIM3 CNT = 0
    // __HAL_TIM_SET_COUNTER(dev->tim, 0);

    dev->last_angle = 0.0f;
}

#endif /* BSP_ABZ_ENABLE */
