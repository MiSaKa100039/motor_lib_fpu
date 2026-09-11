#include "MotorTestRunner.h"
#include "Motor_Runtime.h"
#include "Observer_SMO.h"
#include "PidController.h"
#include <limits>
#include <initializer_list>

namespace Lib_Motor
{
// 测试专用友元只访问数值步骤，不向固件公开调试 API。
struct SmoQ15TestAccess
{
    static FixedNumeric::q15_t integrate(SlidingModeObserverQ15& smo, FixedNumeric::q15_t error)
    { return smo.updatePllIntegral(error); }
    static std::int64_t integral(const SlidingModeObserverQ15& smo)
    { return smo.pll_integral_speed_q35_; }
    static FixedNumeric::q15_t sliding(const SlidingModeObserverQ15& smo, FixedNumeric::q15_t error)
    { return smo.slidingControl(error); }
    static FixedNumeric::q15_t scale(FixedNumeric::q15_t value, std::int32_t coefficient)
    { return SlidingModeObserverQ15::scaleSignedByQ12(value, coefficient); }
    static std::int32_t coefficient(const SlidingModeObserverQ15& smo)
    { return smo.sliding_gain_q12_; }
    static std::int32_t encode(float value)
    { return SlidingModeObserverQ15::positiveToQ12(value); }
};
}

using namespace Lib_Motor;

namespace
{
SlidingModeObserverQ15 makeSmo(float gain = 8.0f)
{
    SlidingModeObserverQ15 smo;
    smo.init(0.0275f, 0.000038f, gain, 80.0f, 1200.0f,
             100.0f, 24.0f, 5000.0f, 1.0f / 12000.0f, 4U, 300.0f);
    return smo;
}
}

MOTOR_TEST(smo_small_errors_accumulate_and_are_sign_symmetric)
{
    for (float radians : {0.05f, 0.1f, 0.35f})
    {
        auto positive = makeSmo();
        auto negative = makeSmo();
        const auto error = FixedNumeric::fromNormalized(radians / 3.14159265358979323846f);
        FixedNumeric::q15_t output = 0;
        for (int tick = 0; tick < 1000; ++tick)
        {
            output = SmoQ15TestAccess::integrate(positive, error);
            MOTOR_ASSERT_EQ(SmoQ15TestAccess::integrate(negative, -error), -output);
        }
        const double expected = 1200.0 * (1.0 / 12000.0) *
            (static_cast<double>(error) / 32768.0 * 3.14159265358979323846) *
            1000.0 * 60.0 / (2.0 * 3.14159265358979323846 * 4.0);
        MOTOR_ASSERT_TRUE(output > 0);
        // 系数量化误差小于半个 Q20 LSB，另允许一个输出 Q15 LSB。
        const double tolerance = 1000.0 * error * 0.5 / 1048576.0 * 5000.0 / 32768.0
                               + 5000.0 / 32768.0;
        MOTOR_ASSERT_NEAR(FixedNumeric::toPhysical(output, 5000.0f), expected, tolerance);
    }
}

MOTOR_TEST(smo_integral_reversal_cancels_fractional_remainder)
{
    auto smo = makeSmo();
    for (int i = 0; i < 701; ++i) SmoQ15TestAccess::integrate(smo, 1043);
    for (int i = 0; i < 701; ++i) SmoQ15TestAccess::integrate(smo, -1043);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::integral(smo), 0);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::integrate(smo, 0), 0);
}

MOTOR_TEST(smo_integral_saturates_and_can_recover)
{
    auto positive = makeSmo();
    auto negative = makeSmo();
    for (int i = 0; i < 20000; ++i)
    {
        SmoQ15TestAccess::integrate(positive, 32767);
        SmoQ15TestAccess::integrate(negative, -32767);
    }
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::integrate(positive, 0), 32767);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::integrate(negative, 0), -32767);
    MOTOR_ASSERT_TRUE(SmoQ15TestAccess::integrate(positive, -32767) < 32767);
    MOTOR_ASSERT_TRUE(SmoQ15TestAccess::integrate(negative, 32767) > -32767);
}

MOTOR_TEST(smo_reset_and_both_angle_seeds_clear_integral)
{
    auto smo = makeSmo();
    SmoQ15TestAccess::integrate(smo, 1);
    smo.reset();
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::integral(smo), 0);
    SmoQ15TestAccess::integrate(smo, 1);
    smo.seedAngle(1.0f);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::integral(smo), 0);
    SmoQ15TestAccess::integrate(smo, -1);
    smo.seedAngle(static_cast<FixedNumeric::phase_u32_t>(1234U));
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::integral(smo), 0);
    MOTOR_ASSERT_EQ(smo.estimate().valid_ticks, 0);
}

MOTOR_TEST(smo_gains_6_8_12_have_distinct_physical_feedback)
{
    std::int32_t previous = 0;
    for (float gain : {6.0f, 8.0f, 12.0f})
    {
        auto smo = makeSmo(gain);
        const auto error = FixedNumeric::fromPhysical(0.1f, 100.0f);
        const auto coefficient = SmoQ15TestAccess::coefficient(smo);
        MOTOR_ASSERT_TRUE(coefficient > previous);
        previous = coefficient;
        const float expected = gain * FixedNumeric::toPhysical(error, 100.0f);
        MOTOR_ASSERT_NEAR(FixedNumeric::toPhysical(SmoQ15TestAccess::sliding(smo, error), 24.0f),
                          expected, 2.0f * 24.0f / 32768.0f);
        MOTOR_ASSERT_NEAR(FixedNumeric::toPhysical(SmoQ15TestAccess::sliding(smo, -error), 24.0f),
                          -expected, 2.0f * 24.0f / 32768.0f);
        MOTOR_ASSERT_NEAR(FixedNumeric::toPhysical(SmoQ15TestAccess::sliding(smo, 32767), 24.0f),
                          gain, 24.0f / 32768.0f);
    }
}

MOTOR_TEST(smo_q12_wide_products_saturate_before_narrowing)
{
    const auto maximum = (std::numeric_limits<std::int32_t>::max)();
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::scale(32767, maximum), 32767);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::scale(-32768, maximum), -32768);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::scale(0, maximum), 0);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::encode(1000000.0f), maximum);
    MOTOR_ASSERT_EQ(SmoQ15TestAccess::encode(-1.0f), 0);
}

MOTOR_TEST(smo_q15_angle_branch_uses_internal_direction)
{
    const FixedNumeric::phase_u32_t raw = 1234U;
    MOTOR_ASSERT_EQ(runtimeCorrectSmoAngleForDirection(raw, 1),
                    static_cast<FixedNumeric::phase_u32_t>(raw + 0x80000000UL));
    MOTOR_ASSERT_EQ(runtimeCorrectSmoAngleForDirection(raw, -1), raw);
}

MOTOR_TEST(q15_unsigned_blend_scales_signed_values_without_int64)
{
    MOTOR_ASSERT_EQ(FixedNumeric::scaleSignedByUnsignedQ15(65535, 0U), 0);
    MOTOR_ASSERT_EQ(FixedNumeric::scaleSignedByUnsignedQ15(65535, 16384U), 32768);
    MOTOR_ASSERT_EQ(FixedNumeric::scaleSignedByUnsignedQ15(65535, 32768U), 65535);
    MOTOR_ASSERT_EQ(FixedNumeric::scaleSignedByUnsignedQ15(-65535, 16384U), -32768);
    MOTOR_ASSERT_EQ(FixedNumeric::scaleSignedByUnsignedQ15(-32768, 32768U), -32768);
}

MOTOR_TEST(smo_validity_uses_release_hysteresis_before_unlocking)
{
    auto smo = makeSmo();
    smo.setValidityCriteria(0.0f,
                            3.14159265358979323846f,
                            2U,
                            0.0f,
                            1000.0f,
                            3.14159265358979323846f,
                            2U,
                            0.0f);

    ObserverEstimateQ15 estimate{};
    smo.update(0, 0, 0, 0, &estimate); // 初始化电流模型
    smo.update(0, 0, 0, 0, &estimate);
    MOTOR_ASSERT_TRUE(!estimate.valid);
    smo.update(0, 0, 0, 0, &estimate);
    MOTOR_ASSERT_TRUE(estimate.valid);
    MOTOR_ASSERT_EQ(estimate.invalid_ticks, 0U);

    smo.update(0, 0, 0, 0, &estimate);
    MOTOR_ASSERT_TRUE(estimate.valid);
    MOTOR_ASSERT_EQ(estimate.invalid_ticks, 1U);
    smo.update(0, 0, 0, 0, &estimate);
    MOTOR_ASSERT_TRUE(!estimate.valid);
    MOTOR_ASSERT_EQ(estimate.invalid_ticks, 2U);
}

MOTOR_TEST(q15_speed_pid_preserves_gain_above_one_normalized)
{
    PIDParam param{};
    param.kp = 0.12f;
    param.ki = 0.0f;
    param.output_limit = 100.0f;
    PidController pid;
    pid.init(param);
    pid.configureFixed(5000.0f, 100.0f, 1.0f / 12000.0f, 100.0f, true);

    const auto output = pid.update(FixedNumeric::fromNormalized(0.10f), true);
    MOTOR_ASSERT_NEAR(FixedNumeric::toPhysical(output, 100.0f), 60.0f, 0.02f);

    const auto preload = FixedNumeric::fromPhysical(1.2f, 100.0f);
    pid.preloadOutputQ15(0, preload);
    MOTOR_ASSERT_EQ(pid.update(0, true), preload);
}
