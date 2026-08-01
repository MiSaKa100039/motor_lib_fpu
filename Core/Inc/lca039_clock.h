#ifndef LCA039_CLOCK_H
#define LCA039_CLOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SystemClock_Config() 的返回值用于记录时钟 bring-up 卡在哪个阶段。 */
typedef enum
{
    LCA039_CLOCK_STATUS_OK = 0,
    LCA039_CLOCK_STATUS_NOT_CONFIGURED,
    LCA039_CLOCK_STATUS_RCH_TIMEOUT,
    LCA039_CLOCK_STATUS_OSCH_TIMEOUT,
    LCA039_CLOCK_STATUS_PLL_TIMEOUT,
    LCA039_CLOCK_STATUS_SWITCH_TIMEOUT,
    LCA039_CLOCK_STATUS_VERIFY_FAILED
} LCA039_ClockStatus;

/* CMSIS 约定的当前内核时钟，SysTick 和依赖 SystemCoreClock 的外设都会读取它。 */
extern uint32_t SystemCoreClock;

/* 按 CMake 生成的 lca039_clock_config.h 配置 RCH/OSCH/PLL、APB 分频和 Flash wait state。 */
LCA039_ClockStatus SystemClock_Config(void);

/* 从芯片当前寄存器反推 SystemCoreClock，用于时钟切换后复核实际频率。 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif