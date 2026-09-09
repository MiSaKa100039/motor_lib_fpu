/*
 * BSP_Motor.h — 电机驱动硬件抽象层声明
 *
 * 本文件定义 BSP 层的两个公开函数:
 *   1. BSP_Get_HAL_Impl()       — 返回 MotorHAL_t 接口指针 (供 Platform 绑定)
 *   2. BSP_Motor_Hardware_Start() — 启动 PWM 定时器、ADC、DMA 和中断
 *
 * Lib/Motor 通过 MotorHAL_t 调用硬件回调, 不依赖任何 MCU 硬件头文件
 */

#pragma once

#include <stdint.h>

#define BSP_MOTOR_HW_FAULT_NONE        (0UL)
#define BSP_MOTOR_HW_FAULT_TIM1_BREAK  (1UL << 0)
#define BSP_MOTOR_HW_FAULT_TIM1_BREAK2 (1UL << 1)
#define BSP_MOTOR_HW_FAULT_COMPARATOR  (1UL << 2)
#define BSP_MOTOR_HW_FAULT_GATE_DRIVER (1UL << 3)
#define BSP_MOTOR_HW_FAULT_UNKNOWN     (1UL << 31)
#define BSP_MOTOR_HW_FAULT_ALL         (0xFFFFFFFFUL)

#ifdef __cplusplus
#include "Motor_HAL_Interface.h"
#include "Platform_MotorFeatures.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 获取已实现的硬件接口结构体
 * Platform 层在 Platform_Motor_Init() 中调用
 * 返回的指针绑定到 MotorConfig.hal
 */
#ifdef __cplusplus
const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void);
#endif

/*
 * 传入当前控制频率, 供 BSP 侧周期预算统计使用
 * 预算值 = SystemCoreClock / control_freq_hz
 */
void BSP_Motor_SetControlFrequencyHz(float control_freq_hz);

/*
 * 启动底层硬件:
 *   - ADC 校准 (hadc1, hadc2)
 *   - ADC2 DMA 循环缓冲 (Vbus, Temp)
 *   - ADC1/ADC2 注入组中断 (相电流)
 *   - TIM1 CH4 PWM 输出 (ADC 触发源)
 *   - TIM1 主计数器使能
 *
 * 在 MotorAPI::init() 之后调用
 */
void BSP_Motor_Hardware_Start(void);
uint8_t BSP_Motor_ResyncSamplingPath(void);
void BSP_Motor_LatchHardwareFaultFromISR(uint32_t flags);
uint32_t BSP_Motor_ReadHardwareFaultLatch(void);
void BSP_Motor_ClearHardwareFaultLatch(uint32_t flags);

/*
 * 取走一次遥测发送请求:
 *   - ADC ISR 内部分频到时只置位 pending
 *   - 用户层/后台循环调用本函数, 若返回 true 则自行决定发送什么内容
 */
void BSP_Motor_RequestVofaTelemetryFromFastIrq(void);
#ifdef __cplusplus
bool BSP_Motor_TakeTelemetryPending(void);
#endif
extern volatile uint32_t BSP_Motor_TelemetryProducedCount;
extern volatile uint32_t BSP_Motor_TelemetryConsumedCount;
extern volatile uint32_t BSP_Motor_TelemetryDroppedCount;
extern volatile uint32_t BSP_Motor_HardwareFaultLatch;

#ifdef MOTOR_BUILD_ENABLE_TICK_PROFILING
extern volatile uint32_t BSP_Motor_InjectedAdc1CallbackRateHz; // 最近 1 秒窗口内处理到的 ADC 注入回调频率
extern volatile uint32_t BSP_Motor_InjectedAdc1CallbackCount; // ADC 注入完成回调累计次数, 用于确认真实控制 tick 频率
extern volatile uint32_t BSP_Motor_TickBudgetCycles;       // 每 tick 预算周期数 (SystemCoreClock / control_freq), 0 表示未计算
extern volatile uint32_t BSP_Motor_TickCoreCyclesLast;     // 最近一次 FOC 核心计算耗时 (cycles)
extern volatile uint32_t BSP_Motor_TickCoreCyclesMax;      // FOC 核心计算历史峰值 (cycles)
extern volatile uint32_t BSP_Motor_TickIsrCyclesLast;      // 最近一次 ISR 总耗时 (cycles, 含 FOC + 遥测分频等)
extern volatile uint32_t BSP_Motor_TickIsrCyclesMax;       // ISR 总耗时历史峰值 (cycles)
extern volatile uint32_t BSP_Motor_TickOverBudgetCount;    // ISR 超预算次数累计 (isr_cycles > budget 时 +1)
#endif

#ifdef __cplusplus
}
#endif
