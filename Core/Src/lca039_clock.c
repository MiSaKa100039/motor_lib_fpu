/*
 * 文件说明：负责系统时钟切换、Flash 等待周期、电源高速配置以及时钟频率回读校验。
 * 外设侧统一依赖这里最终建立的 SystemCoreClock / PCLK 结果。
 */

#include "lca039_clock.h"

#include "lca039_clock_config.h"
#include "lcm32f039.h"
#include "lcm32f039_rcc.h"

/*
 * 修改时钟频率时通常不要直接改这个文件。
 * 入口参数来自 CMake：LCA039_CLOCK_SOURCE、LCA039_SYSCLK_HZ、LCA039_OSCH_HZ、
 * LCA039_APB0_DIV、LCA039_APB1_DIV 和 LCA039_VDD_MV。CMake 会生成 PLL、Flash、
 * APB 与 ADC 同步分频配置；外设代码再使用生成后的 PCLK0/PCLK1 宏重新计算周期。
 */
#define LCA039_CHIPCTRL_KEY (0x87214365UL)

#define LCA039_CLK_SOURCE_OSCH (1UL << 20)
#define LCA039_CLK_SOURCE_PLL (2UL << 20)
#define LCA039_CLK_RCH_ENABLE (1UL << 19)
#define LCA039_CLK_OSCH_ENABLE (1UL << 17)
#define LCA039_CLK_PLL_ENABLE (1UL << 16)

#define LCA039_CLK_APB0_DIV(value) (((uint32_t)(value) - 1UL) << 8)
#define LCA039_CLK_APB1_DIV(value) (((uint32_t)(value) - 1UL) << 12)

#define LCA039_FLASH_LATENCY_MASK (0x7UL)
#define LCA039_LVR_ENABLE (1UL)
#define LCA039_LVR_LEVEL_2V5 (3UL << 1)
#define LCA039_LVR_LEVEL_4V0 (7UL << 1)
#define LCA039_LVR_LEVEL_MASK (0x7UL << 1)
#define LCA039_LDO_DRIVER_200UA (3UL << 2)
#define LCA039_LDO_DRIVER_MASK (3UL << 2)

uint32_t SystemCoreClock = LCA039_RCH_HZ;

static void LCA039_ChipCtrlWrite(volatile uint32_t *reg, uint32_t value)
{
    /* CHIPCTRL 写保护寄存器需要先写 KEY；关中断保证 KEY 和目标写入连续完成。 */
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DSB();
    CHIPCTRL->KEY = LCA039_CHIPCTRL_KEY;
    *reg = value;
    __DSB();
    __ISB();
    __set_PRIMASK(primask);
}

static uint32_t LCA039_ApbDividerBits(void)
{
    /* APB0/APB1 分频由 CMake 配置生成；改分频后外设应使用新的 PCLK 宏自动换算。 */
    return LCA039_CLK_APB0_DIV(LCA039_APB0_DIV) |
           LCA039_CLK_APB1_DIV(LCA039_APB1_DIV);
}

static uint32_t LCA039_RchClockConfig(void)
{
    return LCA039_CLK_RCH_ENABLE | LCA039_ApbDividerBits();
}

static int LCA039_WaitForStatus(uint32_t mask)
{
    /* 所有振荡器、PLL 和系统时钟切换都走超时等待，避免启动阶段永久卡死。 */
    uint32_t timeout = LCA039_CLOCK_TIMEOUT_LOOPS;

    while (timeout > 0UL)
    {
        if ((CHIPCTRL->STS & mask) == mask)
        {
            return 1;
        }
        --timeout;
    }

    return 0;
}

static void LCA039_SetFlashLatency(uint32_t wait_states)
{
    /* Flash wait state 必须在升频前设置；具体等待周期由 CMake 根据 SYSCLK 生成。 */
    uint32_t value = FLASH->ACR;

    value &= ~LCA039_FLASH_LATENCY_MASK;
    value |= wait_states & LCA039_FLASH_LATENCY_MASK;
    FLASH->ACR = value;
    __DSB();
    __ISB();
}

static void LCA039_ConfigureHighSpeedPower(void)
{
#if LCA039_HIGH_SPEED_POWER_REQUIRED
    uint32_t value;
#if LCA039_SYSCLK_HZ > 72000000UL
    const uint32_t lvr_level = LCA039_LVR_LEVEL_4V0;
#else
    const uint32_t lvr_level = LCA039_LVR_LEVEL_2V5;
#endif

    value = CHIPCTRL->LDOCR;
    value &= ~LCA039_LDO_DRIVER_MASK;
    value |= LCA039_LDO_DRIVER_200UA;
    LCA039_ChipCtrlWrite(&CHIPCTRL->LDOCR, value);

    value = CHIPCTRL->PWR_CFG;
    value &= ~LCA039_LVR_LEVEL_MASK;
    value |= LCA039_LVR_ENABLE | lvr_level;
    LCA039_ChipCtrlWrite(&CHIPCTRL->PWR_CFG, value);

    /* 升到 64 MHz 及以上时先增强 LDO/LVR，按厂商时序等待电源监控稳定后再切高速。 */
    delay10us(1000UL);
#endif
}

static void LCA039_FallbackToRch(void)
{
    /* 任一时钟阶段失败时回退内部 RCH 16 MHz，让系统保留可诊断的运行环境。 */
    LCA039_ChipCtrlWrite(&CHIPCTRL->CLK_CFG, LCA039_RchClockConfig());
    LCA039_SetFlashLatency(0UL);
    SystemCoreClock = LCA039_RCH_HZ;
}

void SystemCoreClockUpdate(void)
{
    /* 根据当前芯片寄存器反推内核频率，避免只相信编译期目标频率。 */
    uint32_t source_frequency;
    uint32_t system_frequency;
    uint32_t source = CHIPCTRL->CLK_CFG_b.CLK_SRC_SEL;

    if (source == 0UL)
    {
        source_frequency = LCA039_RCH_HZ;
    }
    else if (source == 1UL)
    {
        source_frequency = LCA039_OSCH_HZ;
    }
    else if (source == 2UL)
    {
        const uint32_t pll_input = (CHIPCTRL->PLL_CFG_b.PLL_SRC != 0UL)
                                       ? LCA039_OSCH_HZ
                                       : LCA039_RCH_HZ;
        const uint32_t numerator = pll_input * (CHIPCTRL->PLL_CFG_b.PLL_DN + 1UL);
        const uint32_t denominator = (CHIPCTRL->PLL_CFG_b.PLL_DM + 1UL) *
                                     (CHIPCTRL->PLL_CFG_b.PLL_OD + 1UL) * 2UL;

        source_frequency = numerator / denominator;
    }
    else
    {
        source_frequency = 24000UL;
    }

    switch ((CHIPCTRL->CLK_CFG_b.SYS_CLK_SEL >> 6) & 0x3UL)
    {
    case 0UL:
        system_frequency = source_frequency;
        break;
    case 1UL:
        system_frequency = source_frequency /
                           (2UL * ((CHIPCTRL->CLK_CFG_b.SYS_CLK_SEL & 0x3FUL) + 1UL));
        break;
    case 2UL:
        system_frequency = source_frequency / 3UL;
        break;
    default:
        system_frequency = source_frequency / 5UL;
        break;
    }

    SystemCoreClock = system_frequency;
}

LCA039_ClockStatus SystemClock_Config(void)
{
    /* CubeMX 风格的系统时钟入口：先回到 RCH，再按生成配置启用 OSCH/PLL 并复核频率。 */
    uint32_t clock_config = LCA039_RchClockConfig();

    LCA039_ChipCtrlWrite(&CHIPCTRL->CLK_CFG, clock_config);
    if (LCA039_WaitForStatus(CHIPCTRL_STS_RCH_STB_Msk) == 0)
    {
        LCA039_FallbackToRch();
        return LCA039_CLOCK_STATUS_RCH_TIMEOUT;
    }

    LCA039_SetFlashLatency(LCA039_FLASH_WAIT_STATES);
    LCA039_ConfigureHighSpeedPower();

#if LCA039_CLOCK_SOURCE == LCA039_CLOCK_SOURCE_OSCH
    OSCH_GPIO_INIT();
    clock_config |= LCA039_CLK_OSCH_ENABLE;
    LCA039_ChipCtrlWrite(&CHIPCTRL->CLK_CFG, clock_config);
    if (LCA039_WaitForStatus(CHIPCTRL_STS_OSCH_STB_Msk) == 0)
    {
        LCA039_FallbackToRch();
        return LCA039_CLOCK_STATUS_OSCH_TIMEOUT;
    }
#endif

#if LCA039_USE_PLL
    LCA039_ChipCtrlWrite(&CHIPCTRL->PLL_CFG, LCA039_PLL_CFG_VALUE);
    clock_config |= LCA039_CLK_PLL_ENABLE;
    LCA039_ChipCtrlWrite(&CHIPCTRL->CLK_CFG, clock_config);
    if (LCA039_WaitForStatus(CHIPCTRL_STS_PLL_LOCK_Msk) == 0)
    {
        LCA039_FallbackToRch();
        return LCA039_CLOCK_STATUS_PLL_TIMEOUT;
    }

    clock_config |= LCA039_CLK_SOURCE_PLL;
#elif LCA039_CLOCK_SOURCE == LCA039_CLOCK_SOURCE_OSCH
    clock_config |= LCA039_CLK_SOURCE_OSCH;
#endif

    LCA039_ChipCtrlWrite(&CHIPCTRL->CLK_CFG, clock_config);

#if LCA039_USE_PLL || (LCA039_CLOCK_SOURCE == LCA039_CLOCK_SOURCE_OSCH)
    if (LCA039_WaitForStatus(CHIPCTRL_STS_SYS_CLK_LOCK_Msk) == 0)
    {
        LCA039_FallbackToRch();
        return LCA039_CLOCK_STATUS_SWITCH_TIMEOUT;
    }
#endif

    SystemCoreClockUpdate();
    if (SystemCoreClock != LCA039_SYSCLK_HZ)
    {
        LCA039_FallbackToRch();
        return LCA039_CLOCK_STATUS_VERIFY_FAILED;
    }

    return LCA039_CLOCK_STATUS_OK;
}
