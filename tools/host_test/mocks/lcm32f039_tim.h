#pragma once

#include <cstdint>

constexpr uint32_t TIM_Channel_1 = 0, TIM_Channel_2 = 4, TIM_Channel_3 = 8;
constexpr uint32_t TIM_CCx_Disable = 0, TIM_CCx_Enable = 1;
constexpr uint32_t TIM_CCxN_Disable = 0, TIM_CCxN_Enable = 4;
constexpr uint32_t TIM_OCPreload_Disable = 0, TIM_OCPreload_Enable = 8;
constexpr uint32_t TIM_OCMode_PWM1 = 0x60, TIM_ForcedAction_InActive = 0x40;
constexpr uint32_t TIM_BDTR_MOE = 0x8000;
constexpr uint32_t TIM_FLAG_Break = 0x80, TIM_FLAG_Break2 = 0x100;
constexpr uint32_t ENABLE = 1, DISABLE = 0;

struct TIM_TypeDef
{
    uint32_t CR1 = 0, DIER = 0, SR = 0, CCMR1 = 0, CCMR2 = 0;
    uint32_t CCER = 0, BDTR = 0, CNT = 0, ARR = 0;
    uint32_t CCR1 = 0, CCR2 = 0, CCR3 = 0, CCR4 = 0;
    uint32_t active_compare[3]{};
    unsigned moe_writes = 0, operations = 0, inject_break_at = 0;
    bool mode_changed_while_enabled = false;
    bool unsafe_enable = false;
};

namespace MockTim
{
inline uint32_t mode(const TIM_TypeDef* t, unsigned i)
{
    const uint32_t ccmr = i < 2 ? t->CCMR1 : t->CCMR2;
    return (ccmr >> ((i % 2) * 8)) & 0x70;
}
inline void step(TIM_TypeDef* t)
{
    if (++t->operations == t->inject_break_at)
    {
        t->SR |= TIM_FLAG_Break;
        t->BDTR &= ~TIM_BDTR_MOE;
    }
}
inline void setMode(TIM_TypeDef* t, unsigned i, uint32_t m)
{
    if ((t->CCER & 0x555) != 0) t->mode_changed_while_enabled = true;
    uint32_t& reg = i < 2 ? t->CCMR1 : t->CCMR2;
    const unsigned shift = (i % 2) * 8;
    reg = (reg & ~(0x70U << shift)) | (m << shift);
    step(t);
}
inline void preload(TIM_TypeDef* t, unsigned i, uint32_t v)
{
    uint32_t& reg = i < 2 ? t->CCMR1 : t->CCMR2;
    const unsigned shift = (i % 2) * 8;
    reg = (reg & ~(8U << shift)) | (v << shift);
    step(t);
}
inline void compare(TIM_TypeDef* t, unsigned i, uint32_t v)
{
    uint32_t& reg = i == 0 ? t->CCR1 : (i == 1 ? t->CCR2 : t->CCR3);
    reg = v;
    const uint32_t ccmr = i < 2 ? t->CCMR1 : t->CCMR2;
    if ((ccmr & (8U << ((i % 2) * 8))) == 0) t->active_compare[i] = v;
    step(t);
}
inline void checkEnable(TIM_TypeDef* t, bool main)
{
    // 任何功率使能都应发生在三相模式已全部恢复或全部强制无效之后。
    const bool all_pwm = mode(t, 0) == 0x60 && mode(t, 1) == 0x60 && mode(t, 2) == 0x60;
    const bool all_brake = mode(t, 0) == 0x40 && mode(t, 1) == 0x40 && mode(t, 2) == 0x40;
    if (!(all_pwm || (!main && all_brake))) t->unsafe_enable = true;
    if (all_pwm && (t->active_compare[0] || t->active_compare[1] || t->active_compare[2]))
        t->unsafe_enable = true;
}
}

inline void TIM_CCxCmd(TIM_TypeDef* t, uint32_t ch, uint32_t value)
{
    if (value) MockTim::checkEnable(t, true);
    t->CCER = (t->CCER & ~(1U << ch)) | (value << ch);
    MockTim::step(t);
}
inline void TIM_CCxNCmd(TIM_TypeDef* t, uint32_t ch, uint32_t value)
{
    if (value) MockTim::checkEnable(t, false);
    t->CCER = (t->CCER & ~(4U << ch)) | (value << ch);
    MockTim::step(t);
}
inline void TIM_SelectOCxM(TIM_TypeDef* t, uint32_t ch, uint32_t mode)
{
    t->CCER &= ~(1U << ch);
    MockTim::setMode(t, ch / 4, mode);
}
inline void TIM_ForcedOC1Config(TIM_TypeDef* t, uint32_t m) { MockTim::setMode(t, 0, m); }
inline void TIM_ForcedOC2Config(TIM_TypeDef* t, uint32_t m) { MockTim::setMode(t, 1, m); }
inline void TIM_ForcedOC3Config(TIM_TypeDef* t, uint32_t m) { MockTim::setMode(t, 2, m); }
inline void TIM_OC1PreloadConfig(TIM_TypeDef* t, uint32_t v) { MockTim::preload(t, 0, v); }
inline void TIM_OC2PreloadConfig(TIM_TypeDef* t, uint32_t v) { MockTim::preload(t, 1, v); }
inline void TIM_OC3PreloadConfig(TIM_TypeDef* t, uint32_t v) { MockTim::preload(t, 2, v); }
inline void TIM_SetCompare1(TIM_TypeDef* t, uint32_t v) { MockTim::compare(t, 0, v); }
inline void TIM_SetCompare2(TIM_TypeDef* t, uint32_t v) { MockTim::compare(t, 1, v); }
inline void TIM_SetCompare3(TIM_TypeDef* t, uint32_t v) { MockTim::compare(t, 2, v); }
inline void TIM_CtrlPWMOutputs(TIM_TypeDef* t, uint32_t value)
{
    ++t->moe_writes;
    t->BDTR = (t->BDTR & ~TIM_BDTR_MOE) | (value ? TIM_BDTR_MOE : 0);
    MockTim::step(t);
}
