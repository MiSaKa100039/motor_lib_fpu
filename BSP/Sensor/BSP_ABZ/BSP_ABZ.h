/*
 * BSP_ABZ.h — ABZ 正交编码器 BSP 驱动
 *
 * 使用 STM32 TIM3 编码器模式 (PC6=A, PC7=B, PC8=Z 可选)
 * 读取旋转角度和速度, 供 Observer_Sensor 调用
 */

#pragma once

#include "Motor_Sensor_Interface.h"

#ifndef BSP_ABZ_COUNTER_READ_ENABLED
#define BSP_ABZ_COUNTER_READ_ENABLED 1
#endif

/*
 * x4 decoding counts per mechanical revolution.
 * Keep at 0 while measuring an unknown encoder with TIM3 ARR=65535.
 * Before SENSOR closed-loop operation, set this value and TIM3 ARR=CPR-1.
 */
#ifndef BSP_ABZ_CPR
#define BSP_ABZ_CPR 0U
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ABZ 编码器上下文 */
typedef struct {
    void*    tim;             // TIM_HandleTypeDef* 指针
    uint32_t cpr;             // 硬件 Counts Per Revolution (= 4 × PPR)
    int32_t  last_count;      // 上次计数值
    float    last_angle;      // 上次角度
    float    velocity;        // 估算速度
    uint32_t vel_update_tick; // 速度更新时间戳
} ABZ_Ctx_t;

void     BSP_ABZ_Init(void* ctx);
bool     BSP_ABZ_IsReady(void* ctx);
Lib_Motor::SensorHealth BSP_ABZ_GetHealth(void* ctx);
float    BSP_ABZ_ReadAngle(void* ctx);
float    BSP_ABZ_ReadVelocity(void* ctx);
int32_t  BSP_ABZ_GetRaw(void* ctx);
void     BSP_ABZ_Calibrate(void* ctx);

#ifdef __cplusplus
}
#endif
