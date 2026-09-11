#include "MotorTestRunner.h"
#include "lca039_pwm_outputs.h"
#include <initializer_list>

namespace
{
TIM_TypeDef runningTimer()
{
    TIM_TypeDef t;
    t.CR1 = 0xA1;
    t.DIER = 0x180;
    t.CNT = 321;
    t.ARR = 3999;
    t.CCMR1 = 0x6868;
    t.CCMR2 = 0x6068;
    t.CCER = 0x1555;
    t.BDTR = TIM_BDTR_MOE | 0x1428;
    t.CCR1 = t.CCR2 = t.CCR3 = 1800;
    t.CCR4 = 3900;
    for (auto& c : t.active_compare) c = 1800;
    return t;
}
void expectTimingPreserved(const TIM_TypeDef& t)
{
    MOTOR_ASSERT_EQ(t.CR1, 0xA1U);
    MOTOR_ASSERT_EQ(t.DIER, 0x180U);
    MOTOR_ASSERT_EQ(t.CNT, 321U);
    MOTOR_ASSERT_EQ(t.ARR, 3999U);
    MOTOR_ASSERT_EQ(t.CCMR2 & 0xFF00, 0x6000U);
    MOTOR_ASSERT_EQ(t.CCER & ~0x555U, 0x1000U);
    MOTOR_ASSERT_EQ(t.CCR4, 3900U);
    MOTOR_ASSERT_EQ(t.BDTR & ~TIM_BDTR_MOE, 0x1428U);
    MOTOR_ASSERT_TRUE(!t.mode_changed_while_enabled);
    MOTOR_ASSERT_TRUE(!t.unsafe_enable);
}
void expectPowerOutputsOff(const TIM_TypeDef& t)
{
    for (unsigned i = 0; i < 3; ++i)
    {
        MOTOR_ASSERT_TRUE(!MockTim::mainOutputActive(&t, i));
        MOTOR_ASSERT_TRUE(!MockTim::complementaryOutputActive(&t, i));
    }
}
void expectLowSideBrakeActive(const TIM_TypeDef& t)
{
    for (unsigned i = 0; i < 3; ++i)
    {
        MOTOR_ASSERT_TRUE(!MockTim::mainOutputActive(&t, i));
        MOTOR_ASSERT_TRUE(MockTim::complementaryOutputActive(&t, i));
    }
}
}

MOTOR_TEST(pwm_brake_only_enables_lows_and_preserves_adc_timing)
{
    auto t = runningTimer();
    LCA039_PWM_BrakeLow(&t, true);
    MOTOR_ASSERT_EQ(t.CCER & 0x555, 0x444U);
    for (unsigned i = 0; i < 3; ++i)
    {
        MOTOR_ASSERT_EQ(MockTim::mode(&t, i), TIM_ForcedAction_Active);
    }
    expectLowSideBrakeActive(t);
    MOTOR_ASSERT_EQ(t.BDTR & TIM_BDTR_MOE, TIM_BDTR_MOE);
    MOTOR_ASSERT_EQ(t.moe_writes, 0U);
    LCA039_PWM_BrakeLow(&t, true);
    MOTOR_ASSERT_EQ(t.CCER & 0x555, 0x444U);
    expectLowSideBrakeActive(t);
    MOTOR_ASSERT_EQ(t.moe_writes, 0U);
    expectTimingPreserved(t);
}

MOTOR_TEST(pwm_coast_and_low_side_brake_have_distinct_effective_outputs)
{
    auto t = runningTimer();
    LCA039_PWM_Disable(&t);
    MOTOR_ASSERT_EQ(t.CCER & 0x555, 0U);
    expectPowerOutputsOff(t);

    LCA039_PWM_BrakeLow(&t, true);
    MOTOR_ASSERT_EQ(t.CCER & 0x555, 0x444U);
    expectLowSideBrakeActive(t);
    expectTimingPreserved(t);
}

MOTOR_TEST(pwm_brake_denied_or_break_latched_stays_off)
{
    for (unsigned scenario = 0; scenario < 4; ++scenario)
    {
        auto t = runningTimer();
        if (scenario == 1) t.BDTR &= ~TIM_BDTR_MOE;
        if (scenario == 2) t.SR |= TIM_FLAG_Break;
        if (scenario == 3) t.SR |= TIM_FLAG_Break2;
        const auto bdtr = t.BDTR;
        const auto sr = t.SR;
        LCA039_PWM_BrakeLow(&t, scenario != 0);
        MOTOR_ASSERT_EQ(t.CCER & 0x555, 0U);
        MOTOR_ASSERT_EQ(t.BDTR, bdtr);
        MOTOR_ASSERT_EQ(t.SR, sr);
        MOTOR_ASSERT_EQ(t.moe_writes, 0U);
        expectPowerOutputsOff(t);
        expectTimingPreserved(t);
    }
}

MOTOR_TEST(pwm_break_during_brake_switching_cannot_reenable_moe)
{
    // 在每一个寄存器写操作后注入硬件 Break。
    for (unsigned position = 1; position <= 15; ++position)
    {
        auto t = runningTimer();
        t.inject_break_at = position;
        LCA039_PWM_BrakeLow(&t, true);
        MOTOR_ASSERT_EQ(t.BDTR & TIM_BDTR_MOE, 0U);
        MOTOR_ASSERT_EQ(t.CCER & 0x555, 0U);
        MOTOR_ASSERT_EQ(t.moe_writes, 0U);
        MOTOR_ASSERT_TRUE((t.SR & TIM_FLAG_Break) != 0);
        expectPowerOutputsOff(t);
        expectTimingPreserved(t);
    }
}

MOTOR_TEST(pwm_release_and_restart_restore_safe_active_compares)
{
    auto t = runningTimer();
    LCA039_PWM_BrakeLow(&t, true);
    LCA039_PWM_Disable(&t);
    MOTOR_ASSERT_EQ(t.CCER & 0x555, 0U);
    expectPowerOutputsOff(t);
    LCA039_PWM_Enable(&t);
    MOTOR_ASSERT_EQ(t.CCER & 0x555, 0x555U);
    for (unsigned i = 0; i < 3; ++i)
    {
        MOTOR_ASSERT_EQ(MockTim::mode(&t, i), 0x60U);
        MOTOR_ASSERT_EQ(t.active_compare[i], 0U);
    }
    MOTOR_ASSERT_EQ(t.CCMR1 & 0x0808, 0x0808U);
    MOTOR_ASSERT_EQ(t.CCMR2 & 8, 8U);
    expectTimingPreserved(t);
}

MOTOR_TEST(pwm_restart_refuses_pending_break)
{
    for (auto flag : {TIM_FLAG_Break, TIM_FLAG_Break2})
    {
        auto t = runningTimer();
        t.BDTR &= ~TIM_BDTR_MOE;
        t.SR |= flag;
        LCA039_PWM_Enable(&t);
        MOTOR_ASSERT_EQ(t.CCER & 0x555, 0U);
        MOTOR_ASSERT_EQ(t.BDTR & TIM_BDTR_MOE, 0U);
        MOTOR_ASSERT_EQ(t.moe_writes, 0U);
        MOTOR_ASSERT_EQ(t.SR, flag);
    }
}

MOTOR_TEST(pwm_break_during_restart_keeps_fault_priority)
{
    for (unsigned position = 1; position <= 25; ++position)
    {
        auto t = runningTimer();
        LCA039_PWM_BrakeLow(&t, true);
        t.operations = 0;
        t.inject_break_at = position;
        LCA039_PWM_Enable(&t);
        MOTOR_ASSERT_EQ(t.CCER & 0x555, 0U);
        MOTOR_ASSERT_EQ(t.BDTR & TIM_BDTR_MOE, 0U);
        MOTOR_ASSERT_TRUE((t.SR & TIM_FLAG_Break) != 0);
        expectTimingPreserved(t);
    }
}
