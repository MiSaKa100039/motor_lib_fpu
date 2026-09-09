/*
 * BSP_HALL3.h - 三路数字霍尔传感器硬件层
 *
 * 本层仅负责读取GPIO引脚并提供时间戳。霍尔状态解码、序列校验、
 * 角度和速度估算均在 Lib/Sensor/Drivers/HALL3 中实现。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "Sensor_Config.h"

// GPIO引脚配置由 CubeMX 在 Core/Src/main.c:MX_GPIO_Init() 中统一管理
// 引脚宏定义来自 Core/Inc/main.h (User Label: HALL_H1/HALL_H2/HALL_H3)

#ifdef BSP_HALL3_ENABLE

#ifndef BSP_HALL3_EXTI_ENABLED
#define BSP_HALL3_EXTI_ENABLED 0       // EXTI中断处理代码编译开关(CubeMX中需同步配置NVIC)
#endif

#ifdef __cplusplus
extern "C" {
#endif

// 边沿变化回调函数类型: 传入原始状态、时间戳、时基和用户指针
typedef void (*BSP_HALL3_EdgeCallback_t)(uint8_t raw_state,
                                         uint32_t now_tick,
                                         uint32_t ticks_per_second,
                                         void* user);

// BSP硬件上下文
typedef struct {
    uint8_t raw_state;                  // 当前原始GPIO状态(3bit)
    uint8_t initialized;               // 初始化完成标志
    BSP_HALL3_EdgeCallback_t edge_cb;  // 边沿回调(EXTI模式使用)
    void* edge_user;                   // 回调用户参数
} BSP_HALL3_HW_t;

void     BSP_HALL3_HW_Init(BSP_HALL3_HW_t* hw);               // 初始化GPIO与时基
bool     BSP_HALL3_IsHardwareReady(BSP_HALL3_HW_t* hw);       // 硬件是否就绪
uint8_t  BSP_HALL3_ReadRawState(BSP_HALL3_HW_t* hw);          // 读取原始3bit霍尔状态
void     BSP_HALL3_RegisterEdgeCallback(BSP_HALL3_HW_t* hw,   // 注册边沿回调
                                        BSP_HALL3_EdgeCallback_t cb,
                                        void* user);
uint32_t BSP_HALL3_NowTicks(void);      // 获取当前时间戳
uint32_t BSP_HALL3_TicksPerSecond(void); // 获取每秒节拍数

// 调试观测变量
extern volatile uint32_t BSP_HALL3_Debug_State;
extern volatile uint32_t BSP_HALL3_Debug_Sector;
extern volatile int32_t  BSP_HALL3_Debug_TransitionCount;
extern volatile uint32_t BSP_HALL3_Debug_EdgeCount;
extern volatile uint32_t BSP_HALL3_Debug_InvalidCount;

#ifdef __cplusplus
}
#endif

#endif /* BSP_HALL3_ENABLE */
