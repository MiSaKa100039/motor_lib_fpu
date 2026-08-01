#include "MotorTestRunner.h"

#include "Motor_API.h"
#include "Motor_CommandGuard.h"
#include "Motor_ConfigCheck.h"
#include "Motor_SingleShunt.h"

#include <cstring>

using namespace Lib_Motor;

namespace
{

void noop()
{
}

void setDuty(float, float, float)
{
}

void readCurrents(uint16_t* raw_u, uint16_t* raw_v, uint16_t* raw_w, uint16_t* raw_bus)
{
    if (raw_u != nullptr) *raw_u = 2048U;
    if (raw_v != nullptr) *raw_v = 2048U;
    if (raw_w != nullptr) *raw_w = 2048U;
    if (raw_bus != nullptr) *raw_bus = 0U;
}

uint16_t readVbus()
{
    return 2048U;
}

bool readSingleShuntPair(uint16_t* raw_first, uint16_t* raw_second)
{
    if (raw_first != nullptr) *raw_first = 2048U;
    if (raw_second != nullptr) *raw_second = 2048U;
    return true;
}

void applyCurrentSampleSchedule(const MotorCurrentSampleSchedule*)
{
}

float readSensorAngle(void*)
{
    return 0.0f;
}

bool sensorReady(void*)
{
    return true;
}

MotorHAL_t makeSingleShuntHal()
{
    MotorHAL_t hal{};
    hal.set_duty = setDuty;
    hal.pwm_enable = noop;
    hal.pwm_disable = noop;
    hal.pwm_coast = noop;
    hal.pwm_brake_lowside = noop;
    hal.mechanical_brake_engage = noop;
    hal.mechanical_brake_release = noop;
    hal.read_currents_raw = readCurrents;
    hal.phase_current_valid_mask = 0U;
    hal.read_vbus_raw = readVbus;
    hal.enter_critical = noop;
    hal.exit_critical = noop;
    hal.read_single_shunt_pair_raw = readSingleShuntPair;
    hal.apply_current_sample_schedule = applyCurrentSampleSchedule;
    return hal;
}

SensorInterface_t makeSensor()
{
    SensorInterface_t sensor{};
    sensor.read_angle = readSensorAngle;
    sensor.is_ready = sensorReady;
    sensor.spec.direction = 1;
    sensor.spec.cpr = 4096U;
    return sensor;
}

MotorConfig makeSingleShuntConfig(const MotorHAL_t& hal)
{
    MotorConfig cfg{};
    cfg.hal = &hal;
    cfg.sensor.current_sense_mode = CurrentSenseMode::SINGLE_SHUNT;
    cfg.sensor.current_sensor_type = CurrentSensorType::SHUNT_RESISTOR;
    cfg.sensor.single_shunt_min_sample_window_s = 1.5e-6f;
    cfg.control.modulation = ModulationMethod::SVPWM;
    cfg.control.control_freq_hz = 10000.0f;
    cfg.observer.allow_smo_closed_loop = true;
    cfg.observer.rotor_stationary_guarantee = RotorStationaryGuarantee::LOW_SIDE_BRAKE;
    cfg.default_run_policy.startup_source = StartupSource::IF;
    cfg.default_run_policy.steady_source = SteadyAngleSource::SMO;
    return cfg;
}

MotorHardwareConfig makeHardware(const MotorConfig& cfg)
{
    MotorHardwareConfig hardware{};
    hardware.hal = cfg.hal;
    hardware.physical = cfg.physical;
    hardware.limit = cfg.limit;
    hardware.sensor = cfg.sensor;
    hardware.position = cfg.position;
    return hardware;
}

MotorAlgorithmParam makeAlgorithm(const MotorConfig& cfg)
{
    MotorAlgorithmParam algorithm{};
    algorithm.observer = cfg.observer;
    algorithm.control = cfg.control;
    algorithm.motion = cfg.motion;
    algorithm.brake = cfg.brake;
    algorithm.identify = cfg.identify;
    return algorithm;
}

float phaseCurrent(MotorSingleShuntSamplePhase phase)
{
    switch (phase)
    {
        case MotorSingleShuntSamplePhase::U: return 2.4f;
        case MotorSingleShuntSamplePhase::V: return -0.7f;
        case MotorSingleShuntSamplePhase::W: return -1.7f;
        default: return 0.0f;
    }
}

float sampleForSlot(const MotorSingleShuntSampleSlot& slot)
{
    const float current = phaseCurrent(slot.phase);
    return slot.sign > 0 ? current : -current;
}

void expectBaseDetail(const MotorConfig& cfg, MotorConfigFaultDetail expected)
{
    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    const Fault fault = MotorConfigCheck::validateBase(cfg, &detail);
    MOTOR_ASSERT_TRUE(fault != Fault::NONE);
    MOTOR_ASSERT_EQ(detail, expected);
}

void expectInitDiagnosticError(const MotorConfig& cfg,
                               Result expected_result,
                               Fault expected_fault,
                               MotorConfigFaultDetail expected_detail)
{
    MotorAPI api;
    MOTOR_ASSERT_EQ(api.init(cfg), expected_result);
    MOTOR_ASSERT_EQ(api.getState(), State::ERROR);
    MOTOR_ASSERT_EQ(api.getFault(), expected_fault);
    MOTOR_ASSERT_EQ(api.getConfigFaultDetail(), expected_detail);

    Motor_Global_Process_Handler(0);

    MotorMonitorData monitor{};
    api.getMonitorData(monitor);
    MOTOR_ASSERT_EQ(monitor.state, State::ERROR);
    MOTOR_ASSERT_EQ(monitor.fault_code, expected_fault);
    MOTOR_ASSERT_EQ(monitor.config_fault_detail, expected_detail);
    MOTOR_ASSERT_EQ(api.deInit(), Result::Ok);
}

} // namespace

MOTOR_TEST(host_test_runner_starts)
{
    MOTOR_ASSERT_TRUE(true);
}

MOTOR_TEST(single_shunt_build_config_accepts_mode_1)
{
    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateBuildConfig(&detail), Fault::NONE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::NONE);
}

MOTOR_TEST(single_shunt_base_config_accepts_valid_hal_and_sensor)
{
    const MotorHAL_t hal = makeSingleShuntHal();
    const MotorConfig cfg = makeSingleShuntConfig(hal);

    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateBase(cfg, &detail), Fault::NONE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::NONE);
}

MOTOR_TEST(single_shunt_base_config_rejects_illegal_combinations)
{
    MotorHAL_t hal = makeSingleShuntHal();
    MotorConfig cfg = makeSingleShuntConfig(hal);

    cfg.sensor.current_sensor_type = CurrentSensorType::HALL_SENSOR;
    expectBaseDetail(cfg, MotorConfigFaultDetail::SINGLE_SHUNT_REQUIRES_SHUNT_SENSOR);

    cfg = makeSingleShuntConfig(hal);
    cfg.control.modulation = ModulationMethod::SPWM;
    expectBaseDetail(cfg, MotorConfigFaultDetail::SINGLE_SHUNT_REQUIRES_SVPWM);

    cfg = makeSingleShuntConfig(hal);
    cfg.sensor.single_shunt_min_sample_window_s = 100.0e-6f;
    expectBaseDetail(cfg, MotorConfigFaultDetail::SINGLE_SHUNT_WINDOW_INVALID);

    cfg = makeSingleShuntConfig(hal);
    hal.read_single_shunt_pair_raw = nullptr;
    cfg.hal = &hal;
    expectBaseDetail(cfg, MotorConfigFaultDetail::SINGLE_SHUNT_HAL_INVALID);

    hal = makeSingleShuntHal();
    hal.apply_current_sample_schedule = nullptr;
    cfg = makeSingleShuntConfig(hal);
    expectBaseDetail(cfg, MotorConfigFaultDetail::SINGLE_SHUNT_HAL_INVALID);

    hal = makeSingleShuntHal();
    hal.phase_current_valid_mask = MOTOR_PHASE_CURRENT_U_VALID;
    cfg = makeSingleShuntConfig(hal);
    expectBaseDetail(cfg, MotorConfigFaultDetail::PHASE_CURRENT_SINGLE_SHUNT_MASK_INVALID);
}

MOTOR_TEST(single_shunt_build_config_rejects_motorcfg_mismatch)
{
    MotorHAL_t hal = makeSingleShuntHal();
    hal.phase_current_valid_mask =
        MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_V_VALID;

    MotorConfig cfg = makeSingleShuntConfig(hal);
    cfg.sensor.current_sense_mode = CurrentSenseMode::DUAL_SENSOR;

    expectBaseDetail(cfg, MotorConfigFaultDetail::CURRENT_SENSE_MODE_BUILD_MISMATCH);
}

MOTOR_TEST(motor_api_init_base_error_enters_tick_diagnostic_error)
{
    MotorHAL_t hal = makeSingleShuntHal();
    hal.phase_current_valid_mask =
        MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_V_VALID;

    MotorConfig cfg = makeSingleShuntConfig(hal);
    cfg.sensor.current_sense_mode = CurrentSenseMode::DUAL_SENSOR;

    expectInitDiagnosticError(cfg,
                              Result::InvalidParam,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::CURRENT_SENSE_MODE_BUILD_MISMATCH);
}

MOTOR_TEST(motor_api_init_startup_error_enters_tick_diagnostic_error)
{
    const MotorHAL_t hal = makeSingleShuntHal();
    MotorConfig cfg = makeSingleShuntConfig(hal);
    cfg.control.startup_target_mode = Mode::DEBUG_HFI_OBSERVER;

    expectInitDiagnosticError(cfg,
                              Result::NotSupported,
                              Fault::OBSERVER_UNAVAILABLE,
                              MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED);
}

MOTOR_TEST(motor_api_init_feedback_error_enters_tick_diagnostic_error)
{
    const MotorHAL_t hal = makeSingleShuntHal();
    MotorConfig cfg = makeSingleShuntConfig(hal);
    cfg.control.startup_target_mode = Mode::VELOCITY_CONTROL;
    cfg.observer.allow_smo_closed_loop = false;

    expectInitDiagnosticError(cfg,
                              Result::NotSupported,
                              Fault::OBSERVER_UNAVAILABLE,
                              MotorConfigFaultDetail::FEEDBACK_SMO_CLOSED_LOOP_DISABLED);
}

MOTOR_TEST(single_shunt_reconstructs_all_svpwm_sectors)
{
    struct SectorDuty
    {
        float u;
        float v;
        float w;
        uint8_t sector;
    };

    const SectorDuty cases[] = {
        {0.70f, 0.50f, 0.30f, 1U},
        {0.50f, 0.70f, 0.30f, 2U},
        {0.30f, 0.70f, 0.50f, 3U},
        {0.30f, 0.50f, 0.70f, 4U},
        {0.50f, 0.30f, 0.70f, 5U},
        {0.70f, 0.30f, 0.50f, 6U},
    };

    for (const SectorDuty& item : cases)
    {
        float u = item.u;
        float v = item.v;
        float w = item.w;
        MotorSingleShuntSamplePlan plan{};
        MOTOR_ASSERT_TRUE(MotorSingleShunt::buildSamplePlan(u, v, w, 0.95f,
                                                             1.0e-6f, 10000.0f,
                                                             plan));
        MOTOR_ASSERT_EQ(plan.valid, 1U);
        MOTOR_ASSERT_EQ(plan.sector, item.sector);
        MOTOR_ASSERT_EQ(plan.first.sign, -1);
        MOTOR_ASSERT_EQ(plan.second.sign, 1);

        MotorSingleShuntCurrents currents{};
        MOTOR_ASSERT_TRUE(MotorSingleShunt::reconstruct(plan,
                                                        sampleForSlot(plan.first),
                                                        sampleForSlot(plan.second),
                                                        currents));
        MOTOR_ASSERT_NEAR(currents.ia, 2.4f, 1.0e-5f);
        MOTOR_ASSERT_NEAR(currents.ib, -0.7f, 1.0e-5f);
        MOTOR_ASSERT_NEAR(currents.ic, -1.7f, 1.0e-5f);
        MOTOR_ASSERT_NEAR(currents.ia + currents.ib + currents.ic, 0.0f, 1.0e-5f);
    }
}

MOTOR_TEST(single_shunt_plan_separates_tight_active_windows)
{
    float u = 0.501f;
    float v = 0.500f;
    float w = 0.499f;
    MotorSingleShuntSamplePlan plan{};

    MOTOR_ASSERT_TRUE(MotorSingleShunt::buildSamplePlan(u, v, w, 0.95f,
                                                        1.0e-6f, 10000.0f,
                                                        plan));
    MOTOR_ASSERT_EQ(plan.valid, 1U);
    MOTOR_ASSERT_TRUE(plan.second.trigger_duty > plan.first.trigger_duty);
    MOTOR_ASSERT_TRUE((u - v) >= 0.0099f);
    MOTOR_ASSERT_TRUE((v - w) >= 0.0099f);
}

MOTOR_TEST(single_shunt_exports_hardware_schedule_without_phase_semantics)
{
    float u = 0.70f;
    float v = 0.50f;
    float w = 0.30f;
    MotorSingleShuntSamplePlan plan{};

    MOTOR_ASSERT_TRUE(MotorSingleShunt::buildSamplePlan(u, v, w, 0.95f,
                                                        1.0e-6f, 10000.0f,
                                                        plan));

    const MotorCurrentSampleSchedule schedule =
        MotorSingleShunt::toCurrentSampleSchedule(plan);
    MOTOR_ASSERT_EQ(schedule.enabled, 1U);
    MOTOR_ASSERT_EQ(schedule.sample_count, 2U);
    MOTOR_ASSERT_NEAR(schedule.trigger_duty[0], plan.first.trigger_duty, 1.0e-6f);
    MOTOR_ASSERT_NEAR(schedule.trigger_duty[1], plan.second.trigger_duty, 1.0e-6f);
}

MOTOR_TEST(single_shunt_feedback_allows_sensor_and_if_smo)
{
    const MotorHAL_t hal = makeSingleShuntHal();
    MotorConfig cfg = makeSingleShuntConfig(hal);
    SensorInterface_t sensor = makeSensor();

    cfg.position.rotor_sensor = &sensor;
    cfg.position.rotor_feedback.feedback_class = AngleFeedbackClass::HIGH_RES_ENCODER;
    cfg.default_run_policy.startup_source = StartupSource::SENSOR;
    cfg.default_run_policy.steady_source = SteadyAngleSource::SENSOR;

    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateFeedback(makeHardware(cfg),
                                                       makeAlgorithm(cfg),
                                                       cfg.default_run_policy,
                                                       &detail),
                    Fault::NONE);

    cfg = makeSingleShuntConfig(hal);
    detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateFeedback(makeHardware(cfg),
                                                       makeAlgorithm(cfg),
                                                       cfg.default_run_policy,
                                                       &detail),
                    Fault::NONE);
}

MOTOR_TEST(single_shunt_feedback_rejects_hfi_sources)
{
    const MotorHAL_t hal = makeSingleShuntHal();
    MotorConfig cfg = makeSingleShuntConfig(hal);

    MotorRunPolicy policy{};
    policy.startup_source = StartupSource::HFI;
    policy.steady_source = SteadyAngleSource::SMO;

    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateFeedback(makeHardware(cfg),
                                                       makeAlgorithm(cfg),
                                                       policy,
                                                       &detail),
                    Fault::OBSERVER_UNAVAILABLE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED);

    policy.startup_source = StartupSource::IF;
    policy.steady_source = SteadyAngleSource::HFI;
    detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateFeedback(makeHardware(cfg),
                                                       makeAlgorithm(cfg),
                                                       policy,
                                                       &detail),
                    Fault::OBSERVER_UNAVAILABLE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED);
}

MOTOR_TEST(single_shunt_rejects_hfi_debug_entries)
{
    const MotorHAL_t hal = makeSingleShuntHal();
    MotorConfig cfg = makeSingleShuntConfig(hal);

    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    cfg.control.startup_target_mode = Mode::DEBUG_HFI_OBSERVER;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateStartupTargetMode(cfg, &detail),
                    Fault::OBSERVER_UNAVAILABLE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED);

    cfg.control.startup_target_mode = Mode::DEBUG_IF_HFI_OBSERVER;
    detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateStartupTargetMode(cfg, &detail),
                    Fault::OBSERVER_UNAVAILABLE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED);

    MOTOR_ASSERT_EQ(MotorCommandGuard::validateModeSelection(cfg, Mode::DEBUG_HFI_OBSERVER),
                    Result::NotSupported);
    MOTOR_ASSERT_EQ(MotorCommandGuard::validateModeSelection(cfg, Mode::DEBUG_IF_HFI_OBSERVER),
                    Result::NotSupported);
}

MOTOR_TEST(single_shunt_fault_details_have_strings)
{
    MOTOR_ASSERT_TRUE(std::strcmp(MotorConfigFaultDetailToString(
                                      MotorConfigFaultDetail::SINGLE_SHUNT_HAL_INVALID),
                                  "Single-shunt HAL callbacks invalid") == 0);
    MOTOR_ASSERT_TRUE(std::strcmp(MotorConfigFaultDetailToString(
                                      MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED),
                                  "Single-shunt sampling does not support HFI") == 0);
}

int main()
{
    return MotorHostTest::runAll();
}
