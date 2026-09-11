#include "MotorTestRunner.h"

#include "Motor_ConfigCheck.h"
#include "Motor_Manager.h"
#include "Core/FSM/RunPhaseFSM/Motor_RunPhaseFSM.h"

#include <limits>

using namespace Lib_Motor;

namespace
{

void noopDuty(float, float, float) {}
void noopDutyQ15(int16_t, int16_t, int16_t) {}
void noopVoid() {}
void readZeroCurrents(uint16_t* u, uint16_t* v, uint16_t* w, uint16_t* bus)
{
    *u = 0U;
    *v = 0U;
    *w = 0U;
    *bus = 0U;
}

MotorHAL_t g_if_hal = [] {
    MotorHAL_t hal{};
    hal.set_duty = noopDuty;
    hal.set_duty_q15 = noopDutyQ15;
    hal.pwm_enable = noopVoid;
    hal.pwm_disable = noopVoid;
    hal.pwm_coast = noopVoid;
    hal.read_currents_raw = readZeroCurrents;
    hal.phase_current_valid_mask =
        MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_V_VALID;
    return hal;
}();

volatile MotorIFStartupPhase g_if_phases[] = {
    MotorIFStartupPhase::Alignment(0.002f, 0.003f, 1.0f),
    MotorIFStartupPhase::Ramp(0.002f, 100.0f, 0.5f, 2.0f),
};

MotorIFStartupProfile g_if_profile = {
    g_if_phases,
    2U,
};

volatile MotorIFStartupPhase g_if_multiphase[] = {
    MotorIFStartupPhase::Alignment(0.001f, 0.001f, 1.0f),
    MotorIFStartupPhase::Ramp(0.001f, 100.0f, 0.5f, 2.0f),
    MotorIFStartupPhase::Ramp(0.001f, 200.0f, 0.5f, 2.0f),
    MotorIFStartupPhase::Ramp(0.001f, 400.0f, 0.5f, 2.0f),
    MotorIFStartupPhase::Ramp(0.001f, 600.0f, 0.5f, 2.0f),
};

volatile MotorIFStartupPhase g_if_alignment_only_phases[] = {
    MotorIFStartupPhase::Alignment(0.002f, 0.003f, 1.0f),
};

MotorIFStartupProfile g_if_alignment_only_profile = {
    g_if_alignment_only_phases,
    1U,
};

MotorConfig makeIFConfig()
{
    MotorConfig cfg{};
    cfg.hal = &g_if_hal;
    cfg.physical.pole_pairs = 4U;
    cfg.physical.rs_ohm_measured = 0.15f;
    cfg.physical.ls_h_measured = 0.001f;
    cfg.control.control_freq_hz = 1000.0f;
    cfg.control.auto_derive_current_pid = false;
    cfg.control.modulation = ModulationMethod::SVPWM;
    cfg.control.startup_target_mode = Mode::VELOCITY_CONTROL;
    cfg.sensor.current_sense_mode = CurrentSenseMode::DUAL_SENSOR;
    cfg.sensor.current_sensor_type = CurrentSensorType::SHUNT_RESISTOR;
    cfg.sensor.adc_v_ref = 1.0f;
    cfg.sensor.adc_resolution = 1000.0f;
    cfg.sensor.phase_shunt_resistor = 1.0f;
    cfg.sensor.phase_amp_gain = 1.0f;
    cfg.sensor.bus_shunt_resistor = 1.0f;
    cfg.sensor.bus_amp_gain = 1.0f;
    cfg.sensor.has_bus_voltage = false;
    cfg.limit.max_phase_current_a = 10.0f;
    cfg.limit.max_bus_current_a = 10.0f;
    cfg.limit.max_speed_rpm = 1000.0f;
    cfg.limit.over_voltage_v = 30.0f;
    cfg.limit.under_voltage_v = 0.0f;
    cfg.observer.smo_gain = 1.0f;
    cfg.observer.smo_pll_kp = 1.0f;
    cfg.observer.smo_pll_ki = 1.0f;
    cfg.observer.smo_bemf_lpf_cutoff_hz = 1000.0f;
    cfg.observer.smo_validity.acquire_min_speed_rpm = 50.0f;
    cfg.observer.smo_validity.acquire_min_signal_level = 0.0f;
    cfg.observer.smo_validity.acquire_max_pll_error_rad = 0.35f;
    cfg.observer.smo_validity.acquire_ticks = 2U;
    cfg.observer.smo_validity.release_min_speed_rpm = 40.0f;
    cfg.observer.smo_validity.release_min_signal_level = 0.0f;
    cfg.observer.smo_validity.release_max_pll_error_rad = 0.60f;
    cfg.observer.smo_validity.release_ticks = 2U;
    cfg.observer.smo_handover.min_speed_rpm = 60.0f;
    cfg.observer.smo_handover.max_angle_error_rad = 0.35f;
    cfg.observer.smo_handover.max_speed_error_rpm = 20.0f;
    cfg.observer.smo_handover.confirm_ticks = 1U;
    cfg.observer.smo_handover.blend_time_s = 0.002f;
    cfg.observer.allow_smo_closed_loop = true;
    cfg.observer.if_startup_profile = &g_if_profile;
    cfg.default_run_policy.startup_source = StartupSource::IF;
    cfg.default_run_policy.steady_source = SteadyAngleSource::SMO;
    return cfg;
}

void expectProfilePoint(float speed,
                        float id,
                        float iq,
                        float expected_speed,
                        float expected_id,
                        float expected_iq)
{
    MOTOR_ASSERT_NEAR(speed, expected_speed, 0.05f);
    MOTOR_ASSERT_NEAR(id, expected_id, 0.01f);
    MOTOR_ASSERT_NEAR(iq, expected_iq, 0.01f);
}

#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
MotorConfig makeSpeedControlConfig(float speed_slew_rate_rpm_s,
                                   float global_iq_limit_a = 15.0f,
                                   float speed_pid_limit_a = 10.0f)
{
    MotorConfig cfg = makeIFConfig();
    cfg.control.control_freq_hz = 12000.0f;
    cfg.control.command_direction = 1;
    cfg.control.speed.kp = 1.0f;
    cfg.control.speed.ki = 0.0f;
    cfg.control.speed.kd = 0.0f;
    cfg.control.speed.output_limit = speed_pid_limit_a;
    cfg.motion.max_iq_ref_a = global_iq_limit_a;
    cfg.motion.speed_slew_rate_rpm_s = speed_slew_rate_rpm_s;
    cfg.motion.iq_slew_rate_a_per_s = 0.0f;
    cfg.motion.iq_slew_rate_brake_a_per_s = 0.0f;
    cfg.limit.max_phase_current_a = 50.0f;
    cfg.limit.max_speed_rpm = 5000.0f;
    return cfg;
}

float runSpeedPidLimitCase(float global_iq_limit_a, float speed_pid_limit_a)
{
    MotorConfig cfg = makeSpeedControlConfig(
        0.0f, global_iq_limit_a, speed_pid_limit_a);
    MotorManager manager(cfg);
    manager.init();
    manager.setActiveModeForTest(Mode::VELOCITY_CONTROL);
    RuntimeCtx& ctx = manager.ctxMutForTest();
    ctx.speed_rpm = runtimeSpeedFromPhysical(ctx, 0.0f);
    ctx.speed_ref_limited = runtimeSpeedFromPhysical(ctx, 0.0f);
    ctx.target_rpm = runtimeSpeedFromPhysical(ctx, 1000.0f);
    return runtimeCurrentToPhysical(
        ctx, manager.computeIqReferenceForTest());
}
#endif

} // namespace

#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
MOTOR_TEST(speed_pid_uses_local_and_global_iq_limits_consistently)
{
    MOTOR_ASSERT_NEAR(runSpeedPidLimitCase(15.0f, 10.0f), 10.0f, 0.02f);
    MOTOR_ASSERT_NEAR(runSpeedPidLimitCase(15.0f, 20.0f), 15.0f, 0.02f);
    MOTOR_ASSERT_NEAR(runSpeedPidLimitCase(15.0f, 0.0f), 15.0f, 0.02f);

    MotorConfig cfg = makeSpeedControlConfig(0.0f, 15.0f, 20.0f);
    MotorManager manager(cfg);
    manager.init();
    MOTOR_ASSERT_NEAR(manager.speedPidOutputLimitForTest(), 15.0f, 0.001f);
}
#endif

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && LIB_MOTOR_ENABLE_VELOCITY_CONTROL
MOTOR_TEST(q15_speed_slew_accumulates_sub_lsb_steps)
{
    MotorConfig cfg = makeSpeedControlConfig(200.0f);
    MotorManager manager(cfg);
    manager.init();
    manager.setActiveModeForTest(Mode::VELOCITY_CONTROL);
    RuntimeCtx& ctx = manager.ctxMutForTest();
    ctx.speed_ref_limited = runtimeSpeedFromPhysical(ctx, 400.0f);
    ctx.target_rpm = runtimeSpeedFromPhysical(ctx, 1000.0f);
    const RuntimeSpeed initial_ref = ctx.speed_ref_limited;

    manager.computeIqReferenceForTest();
    MOTOR_ASSERT_EQ(ctx.speed_ref_limited, initial_ref);
    for (uint32_t tick = 1U; tick < 12000U; ++tick)
    {
        manager.computeIqReferenceForTest();
    }
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(ctx, ctx.speed_ref_limited),
                      600.0f,
                      0.20f);

    for (uint32_t tick = 12000U; tick < 36000U; ++tick)
    {
        manager.computeIqReferenceForTest();
    }
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(ctx, ctx.speed_ref_limited),
                      1000.0f,
                      0.20f);
}

MOTOR_TEST(q15_speed_slew_handles_deceleration_target_change_and_direction)
{
    MotorConfig cfg = makeSpeedControlConfig(200.0f);
    MotorManager manager(cfg);
    manager.init();
    manager.setActiveModeForTest(Mode::VELOCITY_CONTROL);
    RuntimeCtx& ctx = manager.ctxMutForTest();
    ctx.speed_ref_limited = runtimeSpeedFromPhysical(ctx, 1000.0f);
    ctx.target_rpm = runtimeSpeedFromPhysical(ctx, 400.0f);

    for (uint32_t tick = 0U; tick < 12000U; ++tick)
    {
        manager.computeIqReferenceForTest();
    }
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(ctx, ctx.speed_ref_limited),
                      800.0f,
                      0.20f);

    ctx.target_rpm = runtimeSpeedFromPhysical(ctx, 900.0f);
    const RuntimeSpeed before_target_change = ctx.speed_ref_limited;
    manager.computeIqReferenceForTest();
    MOTOR_ASSERT_TRUE(ctx.speed_ref_limited >= before_target_change);
    MOTOR_ASSERT_TRUE(ctx.speed_ref_limited != ctx.target_rpm);

    cfg.control.command_direction = -1;
    MotorManager reverse_manager(cfg);
    reverse_manager.init();
    reverse_manager.setActiveModeForTest(Mode::VELOCITY_CONTROL);
    RuntimeCtx& reverse_ctx = reverse_manager.ctxMutForTest();
    reverse_ctx.speed_ref_limited =
        runtimeSpeedFromUserPhysical(cfg, reverse_ctx, 400.0f);
    MOTOR_ASSERT_EQ(reverse_manager.setTargetSpeed(1000.0f), Result::Ok);
    for (uint32_t tick = 0U; tick < 12000U; ++tick)
    {
        reverse_manager.computeIqReferenceForTest();
    }
    MOTOR_ASSERT_NEAR(
        runtimeSpeedToUserPhysical(cfg, reverse_ctx, reverse_ctx.speed_ref_limited),
        600.0f,
        0.20f);
}

MOTOR_TEST(q15_speed_slew_disable_and_resets_clear_fraction)
{
    MotorConfig cfg = makeSpeedControlConfig(0.0f);
    MotorManager immediate_manager(cfg);
    immediate_manager.init();
    immediate_manager.setActiveModeForTest(Mode::VELOCITY_CONTROL);
    RuntimeCtx& immediate_ctx = immediate_manager.ctxMutForTest();
    immediate_ctx.speed_ref_limited = runtimeSpeedFromPhysical(immediate_ctx, 400.0f);
    immediate_ctx.target_rpm = runtimeSpeedFromPhysical(immediate_ctx, 1000.0f);
    immediate_manager.computeIqReferenceForTest();
    MOTOR_ASSERT_EQ(immediate_ctx.speed_ref_limited, immediate_ctx.target_rpm);

    cfg.motion.speed_slew_rate_rpm_s = 200.0f;
    MotorManager manager(cfg);
    manager.init();
    manager.setActiveModeForTest(Mode::VELOCITY_CONTROL);
    RuntimeCtx& ctx = manager.ctxMutForTest();
    ctx.speed_ref_limited = runtimeSpeedFromPhysical(ctx, 400.0f);
    ctx.target_rpm = runtimeSpeedFromPhysical(ctx, 1000.0f);
    for (uint32_t tick = 0U; tick < 5U; ++tick)
    {
        manager.computeIqReferenceForTest();
    }
    MOTOR_ASSERT_TRUE(ctx.speed_slew_fraction_q16 != 0U);

    manager.clearDerivedTargetsForTest();
    MOTOR_ASSERT_EQ(ctx.speed_slew_fraction_q16, 0U);
    ctx.speed_ref_limited = runtimeSpeedFromPhysical(ctx, 400.0f);
    ctx.target_rpm = runtimeSpeedFromPhysical(ctx, 1000.0f);
    manager.computeIqReferenceForTest();
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(ctx, ctx.speed_ref_limited),
                      400.0f,
                      0.20f);

    ctx.speed_slew_fraction_q16 = 123U;
    manager.requestStart();
    MOTOR_ASSERT_EQ(ctx.speed_slew_fraction_q16, 0U);

    ctx.speed_slew_fraction_q16 = 456U;
    manager.setActiveModeForTest(Mode::VELOCITY_CONTROL);
    manager.setMode(Mode::NONE);
    MOTOR_ASSERT_EQ(ctx.speed_slew_fraction_q16, 0U);
}
#endif

MOTOR_TEST(if_profile_validation_rejects_invalid_layout_and_values)
{
    MOTOR_ASSERT_TRUE(MotorConfigCheck::validIFStartupProfile(&g_if_profile));
    MOTOR_ASSERT_TRUE(
        MotorConfigCheck::validDebugIFStartupProfile(&g_if_alignment_only_profile));
    MOTOR_ASSERT_TRUE(
        !MotorConfigCheck::validIFStartupProfile(&g_if_alignment_only_profile));

    volatile MotorIFStartupPhase bad_first[] = {
        MotorIFStartupPhase::Ramp(0.1f, 0.0f, 1.0f, 0.0f),
        MotorIFStartupPhase::Ramp(0.1f, 100.0f, 0.0f, 1.0f),
    };
    MotorIFStartupProfile bad_first_profile{bad_first, 2U};
    MOTOR_ASSERT_TRUE(!MotorConfigCheck::validIFStartupProfile(&bad_first_profile));

    volatile MotorIFStartupPhase no_hold[] = {
        MotorIFStartupPhase::Alignment(0.0f, 0.0f, 1.0f),
        MotorIFStartupPhase::Ramp(0.1f, 100.0f, 0.0f, 1.0f),
    };
    MotorIFStartupProfile no_hold_profile{no_hold, 2U};
    MOTOR_ASSERT_TRUE(!MotorConfigCheck::validIFStartupProfile(&no_hold_profile));

    volatile MotorIFStartupPhase second_alignment[] = {
        MotorIFStartupPhase::Alignment(0.0f, 0.1f, 1.0f),
        MotorIFStartupPhase::Alignment(0.0f, 0.1f, 1.0f),
    };
    MotorIFStartupProfile second_alignment_profile{second_alignment, 2U};
    MOTOR_ASSERT_TRUE(!MotorConfigCheck::validIFStartupProfile(&second_alignment_profile));

    volatile MotorIFStartupPhase zero_ramp[] = {
        MotorIFStartupPhase::Alignment(0.0f, 0.1f, 1.0f),
        MotorIFStartupPhase::Ramp(0.0f, 100.0f, 0.0f, 1.0f),
    };
    MotorIFStartupProfile zero_ramp_profile{zero_ramp, 2U};
    MOTOR_ASSERT_TRUE(!MotorConfigCheck::validIFStartupProfile(&zero_ramp_profile));

    volatile MotorIFStartupPhase non_finite[] = {
        MotorIFStartupPhase::Alignment(0.0f, 0.1f, 1.0f),
        MotorIFStartupPhase::Ramp(0.1f,
                                  std::numeric_limits<float>::quiet_NaN(),
                                  0.0f,
                                  1.0f),
    };
    MotorIFStartupProfile non_finite_profile{non_finite, 2U};
    MOTOR_ASSERT_TRUE(!MotorConfigCheck::validIFStartupProfile(&non_finite_profile));

    MotorIFStartupProfile too_many_profile{
        g_if_phases,
        static_cast<uint8_t>(LIB_MOTOR_IF_PROFILE_MAX_PHASES + 1U),
    };
    MOTOR_ASSERT_TRUE(!MotorConfigCheck::validIFStartupProfile(&too_many_profile));
}

#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
MOTOR_TEST(debug_if_alignment_only_ramps_and_holds_final_current)
{
    MotorConfig cfg = makeIFConfig();
    MotorManager manager(cfg);
    manager.init();
    MOTOR_ASSERT_TRUE(manager.setDebugIFStartupProfile(&g_if_alignment_only_profile));

    float speed = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;

    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 0.0f, 0.5f, 0.0f);
    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 0.0f, 1.0f, 0.0f);

    for (unsigned i = 0U; i < 3U; ++i)
    {
        MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
        expectProfilePoint(speed, id, iq, 0.0f, 1.0f, 0.0f);
    }
    MOTOR_ASSERT_TRUE(manager.debugIFProfileCompleteForTest());

    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 0.0f, 1.0f, 0.0f);
    MOTOR_ASSERT_TRUE(manager.debugIFProfileCompleteForTest());
}

MOTOR_TEST(debug_if_profile_ramps_holds_and_stays_at_final_command)
{
    MotorConfig cfg = makeIFConfig();
    MotorManager manager(cfg);
    manager.init();
    MOTOR_ASSERT_TRUE(manager.setDebugIFStartupProfile(&g_if_profile));

    float speed = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;

    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 0.0f, 0.5f, 0.0f);
    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 0.0f, 1.0f, 0.0f);

    for (unsigned i = 0U; i < 3U; ++i)
    {
        MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
        expectProfilePoint(speed, id, iq, 0.0f, 1.0f, 0.0f);
    }

    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 50.0f, 0.75f, 1.0f);
    MOTOR_ASSERT_TRUE(!manager.debugIFProfileCompleteForTest());

    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 100.0f, 0.5f, 2.0f);
    MOTOR_ASSERT_TRUE(manager.debugIFProfileCompleteForTest());

    MOTOR_ASSERT_TRUE(manager.stepDebugIFProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 100.0f, 0.5f, 2.0f);
    MOTOR_ASSERT_TRUE(manager.debugIFProfileCompleteForTest());
}

MOTOR_TEST(debug_if_smo_profile_completion_keeps_direct_run_without_fault)
{
    MotorConfig cfg = makeIFConfig();
    cfg.control.startup_target_mode = Mode::DEBUG_IF_SMO_OBSERVER;
    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);
    manager.setMode(Mode::DEBUG_IF_SMO_OBSERVER);
    MOTOR_ASSERT_TRUE(manager.setDebugIFStartupProfile(&g_if_profile));
    manager.requestStart();

    for (unsigned i = 0U; i < 12U; ++i)
    {
        manager.tick();
    }

    MOTOR_ASSERT_EQ(manager.state(), State::RUN);
    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::RUN_DIRECT);
    MOTOR_ASSERT_EQ(manager.fault(), Fault::NONE);
    MOTOR_ASSERT_TRUE(manager.debugIFProfileCompleteForTest());
    MOTOR_ASSERT_NEAR(runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
                      100.0f,
                      0.05f);
}

MOTOR_TEST(debug_if_smo_alignment_only_keeps_direct_run_without_fault)
{
    MotorConfig cfg = makeIFConfig();
    cfg.control.startup_target_mode = Mode::DEBUG_IF_SMO_OBSERVER;
    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);
    manager.setMode(Mode::DEBUG_IF_SMO_OBSERVER);
    MOTOR_ASSERT_TRUE(manager.setDebugIFStartupProfile(&g_if_alignment_only_profile));
    manager.requestStart();

    for (unsigned i = 0U; i < 8U; ++i)
    {
        manager.tick();
    }

    MOTOR_ASSERT_EQ(manager.state(), State::RUN);
    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::RUN_DIRECT);
    MOTOR_ASSERT_EQ(manager.fault(), Fault::NONE);
    MOTOR_ASSERT_TRUE(manager.debugIFProfileCompleteForTest());
    MOTOR_ASSERT_NEAR(runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
                      0.0f,
                      0.01f);
    MOTOR_ASSERT_NEAR(runtimeCurrentToPhysical(manager.ctx(), manager.ctx().target_id),
                      1.0f,
                      0.01f);
    MOTOR_ASSERT_NEAR(runtimeCurrentToPhysical(manager.ctx(), manager.ctx().target_iq),
                      0.0f,
                      0.01f);
}
#endif

#if LIB_MOTOR_ENABLE_IF_STARTUP
MOTOR_TEST(formal_if_profile_matches_debug_phase_semantics)
{
    MotorConfig cfg = makeIFConfig();
    MotorManager manager(cfg);
    manager.init();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    manager.requestStart();
#endif

    float speed = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    MOTOR_ASSERT_TRUE(manager.stepIFStartupProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 0.0f, 0.5f, 0.0f);

    for (unsigned i = 0U; i < 4U; ++i)
    {
        MOTOR_ASSERT_TRUE(manager.stepIFStartupProfileForTest(speed, id, iq));
    }
    expectProfilePoint(speed, id, iq, 0.0f, 1.0f, 0.0f);

    MOTOR_ASSERT_TRUE(manager.stepIFStartupProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 50.0f, 0.75f, 1.0f);
    MOTOR_ASSERT_TRUE(manager.stepIFStartupProfileForTest(speed, id, iq));
    expectProfilePoint(speed, id, iq, 100.0f, 0.5f, 2.0f);
}

MOTOR_TEST(formal_if_handover_enters_smo_when_all_conditions_are_met)
{
    MotorConfig cfg = makeIFConfig();
    cfg.observer.smo_handover.max_angle_error_rad = 3.2f;
    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::RUN);
    manager.setRunPhase(RunPhase::FORCE_DRAG);
    const RuntimeSpeed handover_speed =
        runtimeSpeedFromPhysical(manager.ctx(), 100.0f);
    manager.ctxMutForTest().speed_rpm = handover_speed;
    manager.ctxMutForTest().speed_rpm_observer = handover_speed;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    manager.ctxMutForTest().smo_estimate.speed_rpm_q15 = handover_speed;
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    manager.ctxMutForTest().speed_slew_fraction_q16 = 321U;
#endif
#endif
    manager.eventMutForTest().observer_converged = 1U;

    Motor_RunPhaseFSM::step(manager);

    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::FUSION);
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    MOTOR_ASSERT_EQ(manager.ctx().speed_slew_fraction_q16, 0U);
#endif
    Motor_RunPhaseFSM::step(manager);
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    MOTOR_ASSERT_EQ(manager.smoFusionProgressQ15ForTest(), 16384U);
#endif
    Motor_RunPhaseFSM::step(manager);
    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::SMO_ONLY);
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    MOTOR_ASSERT_EQ(manager.smoFusionProgressQ15ForTest(), 32768U);
#endif
    MOTOR_ASSERT_EQ(manager.fault(), Fault::NONE);
    MOTOR_ASSERT_TRUE(!manager.startupRestartPendingForTest());
}

MOTOR_TEST(if_profile_hold_uses_last_enabled_phase_for_2_4_5_phases)
{
    struct ProfileCase
    {
        uint8_t phase_count;
        float expected_rpm;
    };
    const ProfileCase cases[] = {
        {2U, 100.0f},
        {4U, 400.0f},
        {5U, 600.0f},
    };

    for (const ProfileCase& profile_case : cases)
    {
        MotorConfig cfg = makeIFConfig();
        MotorIFStartupProfile profile{
            g_if_multiphase,
            profile_case.phase_count,
        };
        cfg.observer.if_startup_profile = &profile;

        MotorManager manager(cfg);
        manager.init();
        manager.setState(State::STOP);
        MOTOR_ASSERT_EQ(manager.setTargetSpeed(0.0f), Result::Ok);
        MOTOR_ASSERT_TRUE(manager.ifProfileHoldTargetForTest());
        manager.requestStart();

        MOTOR_ASSERT_NEAR(
            runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
            profile_case.expected_rpm,
            0.1f);
    }
}

MOTOR_TEST(if_profile_hold_defers_handover_until_profile_completion)
{
    MotorConfig cfg = makeIFConfig();
    cfg.observer.smo_handover.max_angle_error_rad = 3.2f;
    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);
    MOTOR_ASSERT_EQ(manager.setTargetSpeed(0.0f), Result::Ok);
    manager.requestStart();
    manager.setState(State::RUN);
    manager.setRunPhase(RunPhase::FORCE_DRAG);

    const RuntimeSpeed handover_speed =
        runtimeSpeedFromPhysical(manager.ctx(), 100.0f);
    manager.ctxMutForTest().speed_rpm = handover_speed;
    manager.ctxMutForTest().speed_rpm_observer = handover_speed;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    manager.ctxMutForTest().smo_estimate.speed_rpm_q15 = handover_speed;
#endif
    manager.eventMutForTest().observer_converged = 1U;

    Motor_RunPhaseFSM::step(manager);
    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::FORCE_DRAG);

    float speed = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    while (!manager.ifStartupProfileCompleteForTest())
    {
        MOTOR_ASSERT_TRUE(manager.stepIFStartupProfileForTest(speed, id, iq));
    }

    manager.ctxMutForTest().speed_rpm = handover_speed;
    manager.ctxMutForTest().speed_rpm_observer = handover_speed;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    manager.ctxMutForTest().smo_estimate.speed_rpm_q15 = handover_speed;
#endif
    manager.eventMutForTest().observer_converged = 1U;
    Motor_RunPhaseFSM::step(manager);
    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::FUSION);
    Motor_RunPhaseFSM::step(manager);
    Motor_RunPhaseFSM::step(manager);
    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::SMO_ONLY);
    MOTOR_ASSERT_NEAR(
        runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
        100.0f,
        0.1f);
}

MOTOR_TEST(if_smo_runtime_speed_updates_validate_and_preserve_target)
{
    MotorConfig cfg = makeIFConfig();
    cfg.observer.smo_validity.acquire_min_speed_rpm = 300.0f;
    cfg.observer.smo_validity.release_min_speed_rpm = 250.0f;
    cfg.observer.smo_handover.min_speed_rpm = 350.0f;
    MotorIFStartupProfile profile{g_if_multiphase, 5U};
    cfg.observer.if_startup_profile = &profile;
    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);

    MOTOR_ASSERT_EQ(manager.setTargetSpeed(0.0f), Result::Ok);
    MOTOR_ASSERT_TRUE(manager.ifProfileHoldTargetForTest());
    MOTOR_ASSERT_EQ(manager.setTargetSpeed(500.0f), Result::Ok);
    MOTOR_ASSERT_TRUE(!manager.ifProfileHoldTargetForTest());

    manager.setState(State::RUN);
    manager.setRunPhase(RunPhase::SMO_ONLY);
    MOTOR_ASSERT_EQ(manager.setTargetSpeed(600.0f), Result::Ok);
    const float accepted_rpm = runtimeSpeedToUserPhysical(
        cfg, manager.ctx(), manager.ctx().target_rpm);
    MOTOR_ASSERT_NEAR(accepted_rpm, 600.0f, 0.1f);

    const float invalid_targets[] = {
        200.0f,
        -100.0f,
        cfg.limit.max_speed_rpm + 1.0f,
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
    };
    for (float invalid_target : invalid_targets)
    {
        MOTOR_ASSERT_EQ(manager.setTargetSpeed(invalid_target), Result::InvalidParam);
        MOTOR_ASSERT_NEAR(
            runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
            accepted_rpm,
            0.1f);
    }

    MotionSetpoint sp{};
    sp.mode = Mode::VELOCITY_CONTROL;
    sp.vel_ff = 200.0f;
    MOTOR_ASSERT_EQ(manager.writeSetpoint(sp), Result::InvalidParam);
    MOTOR_ASSERT_NEAR(
        runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
        accepted_rpm,
        0.1f);
}

MOTOR_TEST(if_smo_feedback_validation_checks_final_enabled_ramp_speed)
{
    MotorConfig cfg = makeIFConfig();
    cfg.observer.rotor_stationary_guarantee =
        RotorStationaryGuarantee::EXPLICIT_BYPASS;
    cfg.observer.rotor_explicit_bypass_confirmed = true;
    MotorIFStartupProfile profile{g_if_multiphase, 2U};
    cfg.observer.if_startup_profile = &profile;
    cfg.observer.smo_handover.min_speed_rpm = 350.0f;

    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateFeedback(cfg, &detail), Fault::PARAM_ERROR);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::FEEDBACK_IF_PARAM_INVALID);

    profile.phase_count = 4U;
    detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateFeedback(cfg, &detail), Fault::NONE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::NONE);

    cfg.limit.max_speed_rpm = 350.0f;
    detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateFeedback(cfg, &detail), Fault::PARAM_ERROR);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::FEEDBACK_IF_PARAM_INVALID);
}

MOTOR_TEST(formal_if_profile_does_not_overwrite_user_target)
{
    MotorConfig cfg = makeIFConfig();
    cfg.observer.smo_validity.acquire_min_speed_rpm = 0.0f;
    cfg.observer.smo_validity.acquire_ticks = 1U;
    cfg.observer.smo_validity.release_min_speed_rpm = 0.0f;
    cfg.observer.smo_handover.min_speed_rpm = 0.0f;
    cfg.observer.smo_handover.max_angle_error_rad = 3.2f;
    cfg.control.current_d.kp = 1.0f;
    MotorManager manager(cfg);
    manager.init();
    manager.setTargetSpeed(321.0f);
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    manager.requestStart();
#endif
    manager.setState(State::RUN);
    manager.setRunPhase(RunPhase::FORCE_DRAG);
    manager.ctxMutForTest().v_bus =
        runtimeBusVoltageFromPhysical(manager.ctx(), 15.0f);

    manager.runIFControl();
    manager.runIFControl();
    manager.runIFControl();

    MOTOR_ASSERT_NEAR(runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
                      321.0f,
                      0.05f);
}

MOTOR_TEST(if_startup_failure_retries_then_latches_observer_loss)
{
    MotorConfig cfg = makeIFConfig();
    cfg.default_run_policy.startup_auto_restart = true;
    cfg.default_run_policy.startup_max_retry_count = 1U;
    cfg.default_run_policy.startup_restart_interval_s = 0.002f;

    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::RUN);
    manager.setTargetSpeed(321.0f);
    manager.eventMutForTest().observer_converged = 1U;
    manager.ctxMutForTest().speed_rpm_observer =
        runtimeSpeedFromPhysical(manager.ctx(), 100.0f);
    manager.triggerIFStartupFailureForTest();

    MOTOR_ASSERT_EQ(manager.state(), State::STOPPING);
    MOTOR_ASSERT_TRUE(manager.startupRestartPendingForTest());
    MOTOR_ASSERT_EQ(manager.startupRestartAttemptsForTest(), 1U);
    MOTOR_ASSERT_NEAR(runtimeSpeedToUserPhysical(cfg, manager.ctx(), manager.ctx().target_rpm),
                      321.0f,
                      0.05f);

    manager.setState(State::STOP);
    manager.processPendingStartupRestartForTest();
    MOTOR_ASSERT_TRUE(manager.startupRestartPendingForTest());
    manager.processPendingStartupRestartForTest();
    MOTOR_ASSERT_TRUE(!manager.startupRestartPendingForTest());
    MOTOR_ASSERT_TRUE(manager.runRequestedForTest());
    MOTOR_ASSERT_EQ(manager.event().observer_converged, 0U);
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(manager.ctx(), manager.ctx().speed_rpm_observer),
                      0.0f,
                      0.01f);

    manager.setState(State::RUN);
    manager.triggerIFStartupFailureForTest();
    MOTOR_ASSERT_TRUE((static_cast<uint16_t>(manager.fault()) &
                       static_cast<uint16_t>(Fault::OBSERVER_LOSS)) != 0U);
    MOTOR_ASSERT_EQ(manager.runtimeFaultDetail(),
                    MotorRuntimeFaultDetail::IF_SMO_STARTUP_HANDOVER_FAILED);
    MOTOR_ASSERT_EQ(manager.state(), State::ERROR);
}

MOTOR_TEST(manual_stop_cancels_pending_if_startup_retry)
{
    MotorConfig cfg = makeIFConfig();
    cfg.default_run_policy.startup_auto_restart = true;
    cfg.default_run_policy.startup_max_retry_count = 2U;
    cfg.default_run_policy.startup_restart_interval_s = 1.0f;

    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::RUN);
    manager.triggerIFStartupFailureForTest();
    MOTOR_ASSERT_TRUE(manager.startupRestartPendingForTest());

    manager.requestStop(StopMode::COAST);
    MOTOR_ASSERT_TRUE(!manager.startupRestartPendingForTest());
    MOTOR_ASSERT_EQ(manager.startupRestartAttemptsForTest(), 0U);
}
#endif
