#ifndef LCA039_PWM_OUTPUTS_H
#define LCA039_PWM_OUTPUTS_H

#include <stdbool.h>
#include "lcm32f039_tim.h"

/* 只操作三相功率通道，CH4、计数器、死区和 Break 配置由初始化路径持有。 */
static inline void LCA039_PWM_Disable(TIM_TypeDef* timer)
{
    TIM_CCxCmd(timer, TIM_Channel_1, TIM_CCx_Disable);
    TIM_CCxNCmd(timer, TIM_Channel_1, TIM_CCxN_Disable);
    TIM_CCxCmd(timer, TIM_Channel_2, TIM_CCx_Disable);
    TIM_CCxNCmd(timer, TIM_Channel_2, TIM_CCxN_Disable);
    TIM_CCxCmd(timer, TIM_Channel_3, TIM_CCx_Disable);
    TIM_CCxNCmd(timer, TIM_Channel_3, TIM_CCxN_Disable);
}

static inline void LCA039_PWM_Enable(TIM_TypeDef* timer)
{
    LCA039_PWM_Disable(timer);
    if ((timer->SR & (TIM_FLAG_Break | TIM_FLAG_Break2)) != 0U)
    {
        return;
    }

    /* 关闭预装载后写安全比较值，避免恢复 PWM1 时短暂使用制动前的活动 CCR。 */
    TIM_OC1PreloadConfig(timer, TIM_OCPreload_Disable);
    TIM_OC2PreloadConfig(timer, TIM_OCPreload_Disable);
    TIM_OC3PreloadConfig(timer, TIM_OCPreload_Disable);
    TIM_SetCompare1(timer, 0U);
    TIM_SetCompare2(timer, 0U);
    TIM_SetCompare3(timer, 0U);
    TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_SelectOCxM(timer, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_SelectOCxM(timer, TIM_Channel_2, TIM_OCMode_PWM1);
    TIM_SelectOCxM(timer, TIM_Channel_3, TIM_OCMode_PWM1);

    /* 软件锁存由 BSP 检查；此处还需尊重尚未被中断处理的 Break 标志。 */
    if ((timer->SR & (TIM_FLAG_Break | TIM_FLAG_Break2)) != 0U)
    {
        return;
    }
    TIM_CtrlPWMOutputs(timer, ENABLE);
    TIM_CCxCmd(timer, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(timer, TIM_Channel_1, TIM_CCxN_Enable);
    TIM_CCxCmd(timer, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(timer, TIM_Channel_2, TIM_CCxN_Enable);
    TIM_CCxCmd(timer, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(timer, TIM_Channel_3, TIM_CCxN_Enable);

    if ((timer->SR & (TIM_FLAG_Break | TIM_FLAG_Break2)) != 0U)
    {
        TIM_CtrlPWMOutputs(timer, DISABLE);
        LCA039_PWM_Disable(timer);
    }
}

static inline void LCA039_PWM_BrakeLow(TIM_TypeDef* timer, bool allowed)
{
    LCA039_PWM_Disable(timer);
    if (!allowed || (timer->BDTR & TIM_BDTR_MOE) == 0U ||
        (timer->SR & (TIM_FLAG_Break | TIM_FLAG_Break2)) != 0U)
    {
        return;
    }

    /* 高边关闭且 OCxREF 强制无效，只使能高电平有效的互补低边输出。 */
    /* 修正：仅使能 OCxN 时输出不取反，必须强制 OCxREF 有效才能拉高 LIN。 */
    TIM_ForcedOC1Config(timer, TIM_ForcedAction_Active);
    TIM_ForcedOC2Config(timer, TIM_ForcedAction_Active);
    TIM_ForcedOC3Config(timer, TIM_ForcedAction_Active);
    TIM_SetCompare1(timer, 0U);
    TIM_SetCompare2(timer, 0U);
    TIM_SetCompare3(timer, 0U);
    TIM_CCxNCmd(timer, TIM_Channel_1, TIM_CCxN_Enable);
    TIM_CCxNCmd(timer, TIM_Channel_2, TIM_CCxN_Enable);
    TIM_CCxNCmd(timer, TIM_Channel_3, TIM_CCxN_Enable);

    /* 切换中发生的硬件 Break 仍有最高优先级，绝不在制动入口重新置位 MOE。 */
    if ((timer->BDTR & TIM_BDTR_MOE) == 0U ||
        (timer->SR & (TIM_FLAG_Break | TIM_FLAG_Break2)) != 0U)
    {
        LCA039_PWM_Disable(timer);
    }
}

#endif
