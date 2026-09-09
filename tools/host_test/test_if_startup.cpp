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
    cfg.observer.smo_min_signal_level = 0.0f;
    cfg.observer.hfi_to_smo_startup_rpm = 50.0f;
    cfg.observer.hfi_to_smo_startup_hysteresis_rpm = 10.0f;
    cfg.observer.max_handover_error_rad = 0.35f;
    cfg.observer.convergence_ticks = 2U;
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

} // namespace

MOTOR_TEST(if_profile_validation_rejects_invalid_layout_and_values)
{
    MOTOR_ASSERT_TRUE(MotorConfigCheck::validIFStartupProfile(&g_if_profile));

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
    cfg.observer.max_handover_error_rad = 3.2f;
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
#endif
    manager.eventMutForTest().observer_converged = 1U;

    Motor_RunPhaseFSM::step(manager);

    MOTOR_ASSERT_EQ(manager.runPhase(), RunPhase::SMO_ONLY);
    MOTOR_ASSERT_EQ(manager.fault(), Fault::NONE);
    MOTOR_ASSERT_TRUE(!manager.startupRestartPendingForTest());
}

MOTOR_TEST(formal_if_profile_does_not_overwrite_user_target)
{
    MotorConfig cfg = makeIFConfig();
    cfg.observer.hfi_to_smo_startup_rpm = 0.0f;
    cfg.observer.hfi_to_smo_startup_hysteresis_rpm = 0.0f;
    cfg.observer.max_handover_error_rad = 3.2f;
    cfg.observer.convergence_ticks = 1U;
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
    MOTOR_ASSERT_EQ(manager.event().observer_converged, 1U);
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
