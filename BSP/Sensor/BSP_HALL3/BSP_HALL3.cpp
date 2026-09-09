/*
 * BSP_HALL3.cpp - 三路数字霍尔传感器硬件实现
 */

#include "BSP_HALL3.h"

#ifdef BSP_HALL3_ENABLE

#include "main.h"

// ===================== HALL3 板级接线 =====================
// 引脚定义来自 CubeMX 生成代码 (Core/Inc/main.h)
// User Label 约定: HALL_H1 / HALL_H2 / HALL_H3
// H1/H2/H3 分别对应 BSP_HALL3_ReadRawState() 中的 bit0/bit1/bit2
#ifndef BSP_HALL3_H1_GPIO_PORT
#define BSP_HALL3_H1_GPIO_PORT GPIOC
#endif
#ifndef BSP_HALL3_H1_GPIO_PIN
#define BSP_HALL3_H1_GPIO_PIN  GPIO_PIN_6
#endif
#ifndef BSP_HALL3_H2_GPIO_PORT
#define BSP_HALL3_H2_GPIO_PORT GPIOC
#endif
#ifndef BSP_HALL3_H2_GPIO_PIN
#define BSP_HALL3_H2_GPIO_PIN  GPIO_PIN_7
#endif
#ifndef BSP_HALL3_H3_GPIO_PORT
#define BSP_HALL3_H3_GPIO_PORT GPIOC
#endif
#ifndef BSP_HALL3_H3_GPIO_PIN
#define BSP_HALL3_H3_GPIO_PIN  GPIO_PIN_8
#endif

extern "C" {
volatile uint32_t BSP_HALL3_Debug_State = 0U;              // 调试: 霍尔原始状态
volatile uint32_t BSP_HALL3_Debug_Sector = 0U;             // 调试: 当前扇区
volatile int32_t  BSP_HALL3_Debug_TransitionCount = 0;     // 调试: 跳变累计
volatile uint32_t BSP_HALL3_Debug_EdgeCount = 0U;          // 调试: 边沿计数
volatile uint32_t BSP_HALL3_Debug_InvalidCount = 0U;       // 调试: 非法状态计数
}

namespace
{
uint32_t g_ticks_per_second = 1000U;     // 时基: 每秒节拍数
bool g_use_cycle_counter = false;        // 时基: 是否使用DWT周期计数器
BSP_HALL3_HW_t* g_active_hw = nullptr;   // 当前活跃的硬件实例(EXTI回调用)

// 初始化时基: 优先使用DWT(高精度CPU周期计数器), 失败则回退到HAL_GetTick(1ms)
void initTimebase()
{
#if defined(DWT) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
    if (SystemCoreClock >= 1000000U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        g_ticks_per_second = SystemCoreClock;
        g_use_cycle_counter = true;
        return;
    }
#endif
    g_ticks_per_second = 1000U;
    g_use_cycle_counter = false;
}

// 读取三路GPIO电平，组合为3bit状态: bit0=H1, bit1=H2, bit2=H3
uint8_t readPins()
{
    uint8_t state = 0U;
    if (HAL_GPIO_ReadPin(BSP_HALL3_H1_GPIO_PORT, BSP_HALL3_H1_GPIO_PIN) == GPIO_PIN_SET)
    {
        state |= 0x01U;
    }
    if (HAL_GPIO_ReadPin(BSP_HALL3_H2_GPIO_PORT, BSP_HALL3_H2_GPIO_PIN) == GPIO_PIN_SET)
    {
        state |= 0x02U;
    }
    if (HAL_GPIO_ReadPin(BSP_HALL3_H3_GPIO_PORT, BSP_HALL3_H3_GPIO_PIN) == GPIO_PIN_SET)
    {
        state |= 0x04U;
    }
    return state;
}

#if BSP_HALL3_EXTI_ENABLED != 0
// EXTI边沿通知: 读取当前状态并调用注册的回调
void notifyEdge(BSP_HALL3_HW_t* hw)
{
    if (hw == nullptr || hw->edge_cb == nullptr) return;

    const uint8_t raw = BSP_HALL3_ReadRawState(hw);
    hw->edge_cb(raw, BSP_HALL3_NowTicks(), BSP_HALL3_TicksPerSecond(), hw->edge_user);
}
#endif
} // namespace

void BSP_HALL3_HW_Init(BSP_HALL3_HW_t* hw)
{
    if (hw == nullptr) return;

    *hw = {};
    initTimebase();           // 初始化时基(DWT或SysTick)

    hw->raw_state = readPins();  // 首次读取初始状态
    hw->initialized = 1U;
    g_active_hw = hw;

    BSP_HALL3_Debug_State = hw->raw_state;
}

bool BSP_HALL3_IsHardwareReady(BSP_HALL3_HW_t* hw)
{
    return hw != nullptr && hw->initialized != 0U;
}

uint8_t BSP_HALL3_ReadRawState(BSP_HALL3_HW_t* hw)
{
    if (hw == nullptr || hw->initialized == 0U) return 0U;

    hw->raw_state = readPins();          // 更新最新GPIO状态
    BSP_HALL3_Debug_State = hw->raw_state;
    return hw->raw_state;
}

void BSP_HALL3_RegisterEdgeCallback(BSP_HALL3_HW_t* hw,
                                    BSP_HALL3_EdgeCallback_t cb,
                                    void* user)
{
    if (hw == nullptr) return;
    hw->edge_cb = cb;
    hw->edge_user = user;
}

uint32_t BSP_HALL3_NowTicks(void)
{
    return g_use_cycle_counter ? DWT->CYCCNT : HAL_GetTick(); // DWT或SysTick
}

uint32_t BSP_HALL3_TicksPerSecond(void)
{
    return g_ticks_per_second;
}

// ========== EXTI外部中断处理(仅在 BSP_HALL3_EXTI_ENABLED=1 时编译) ==========
#if BSP_HALL3_EXTI_ENABLED != 0
// 通用EXTI回调: 仅处理霍尔引脚的中断
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != BSP_HALL3_H1_GPIO_PIN &&
        GPIO_Pin != BSP_HALL3_H2_GPIO_PIN &&
        GPIO_Pin != BSP_HALL3_H3_GPIO_PIN)
    {
        return;
    }
    notifyEdge(g_active_hw);
}

// EXTI9_5中断向量: 分发到HAL进行引脚级回调
extern "C" void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BSP_HALL3_H1_GPIO_PIN);
    HAL_GPIO_EXTI_IRQHandler(BSP_HALL3_H2_GPIO_PIN);
    HAL_GPIO_EXTI_IRQHandler(BSP_HALL3_H3_GPIO_PIN);
}
#endif /* BSP_HALL3_EXTI_ENABLED */

#endif /* BSP_HALL3_ENABLE */
