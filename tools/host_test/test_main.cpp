#include "MotorTestRunner.h"

#include "Motor_ConfigCheck.h"
#include "Motor_Manager.h"
#include "Motor_RuntimeMonitor.h"

using namespace Lib_Motor;

namespace
{

uint16_t g_raw_u = 0U;
uint16_t g_raw_v = 0U;
uint16_t g_raw_w = 0U;
uint16_t g_raw_bus = 0U;
uint32_t g_hw_flags = MOTOR_HAL_HW_FAULT_NONE;

void noop()
{
}

void setDuty(float, float, float)
{
}

void readCurrents(uint16_t* raw_u, uint16_t* raw_v, uint16_t* raw_w, uint16_t* raw_bus)
{
    if (raw_u != nullptr) *raw_u = g_raw_u;
    if (raw_v != nullptr) *raw_v = g_raw_v;
    if (raw_w != nullptr) *raw_w = g_raw_w;
    if (raw_bus != nullptr) *raw_bus = g_raw_bus;
}

uint32_t readHardwareFaultFlags()
{
    return g_hw_flags;
}

void clearHardwareFaultFlags(uint32_t flags)
{
    if (flags == MOTOR_HAL_HW_FAULT_NONE || flags == MOTOR_HAL_HW_FAULT_ALL)
    {
        g_hw_flags = MOTOR_HAL_HW_FAULT_NONE;
        return;
    }
    g_hw_flags &= ~flags;
}

MotorHAL_t makeDualHal(uint8_t phase_mask =
    static_cast<uint8_t>(MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_V_VALID))
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
    hal.phase_current_valid_mask = phase_mask;
    hal.enter_critical = noop;
    hal.exit_critical = noop;
    hal.read_hardware_fault_flags = readHardwareFaultFlags;
    hal.clear_hardware_fault_flags = clearHardwareFaultFlags;
    return hal;
}

MotorConfig makeDualConfig(const MotorHAL_t& hal)
{
    MotorConfig cfg{};
    cfg.hal = &hal;
    cfg.physical.pole_pairs = 7U;
    cfg.control.control_freq_hz = 10000.0f;
    cfg.control.modulation = ModulationMethod::SVPWM;
    cfg.sensor.current_sense_mode = CurrentSenseMode::DUAL_SENSOR;
    cfg.sensor.current_sensor_type = CurrentSensorType::SHUNT_RESISTOR;
    cfg.sensor.adc_v_ref = 1.0f;
    cfg.sensor.adc_resolution = 1.0f;
    cfg.sensor.phase_shunt_resistor = 1.0f;
    cfg.sensor.phase_amp_gain = 1.0f;
    cfg.sensor.bus_shunt_resistor = 1.0f;
    cfg.sensor.bus_amp_gain = 1.0f;
    cfg.sensor.phase_current_lpf_tf_s = 0.0f;
    cfg.sensor.bus_current_lpf_tf_s = 0.0f;
    cfg.sensor.bus_voltage_lpf_tf_s = 0.0f;
    cfg.sensor.phase_voltage_lpf_tf_s = 0.0f;
    cfg.sensor.has_bus_voltage = false;
    cfg.sensor.has_bus_current = true;
    cfg.sensor.has_phase_voltage = false;
    cfg.limit.max_phase_current_a = 100.0f;
    cfg.limit.max_bus_current_a = 100.0f;
    cfg.limit.over_voltage_v = 30.0f;
    cfg.limit.under_voltage_v = 0.0f;
    cfg.observer.allow_smo_closed_loop = true;
    cfg.observer.rotor_stationary_guarantee = RotorStationaryGuarantee::LOW_SIDE_BRAKE;
    cfg.default_run_policy.startup_source = StartupSource::IF;
    cfg.default_run_policy.steady_source = SteadyAngleSource::SMO;
    return cfg;
}

bool hasFault(Fault faults, Fault expected)
{
    return (static_cast<uint16_t>(faults) &
            static_cast<uint16_t>(expected)) != 0U;
}

void expectBaseDetail(const MotorConfig& cfg, MotorConfigFaultDetail expected)
{
    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    const Fault fault = MotorConfigCheck::validateBase(cfg, &detail);
    MOTOR_ASSERT_TRUE(fault != Fault::NONE);
    MOTOR_ASSERT_EQ(detail, expected);
}

} // namespace

MOTOR_TEST(host_test_runner_starts)
{
    MOTOR_ASSERT_TRUE(true);
}

MOTOR_TEST(dual_sensor_build_config_accepts_mode_2)
{
    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateBuildConfig(&detail), Fault::NONE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::NONE);
}

MOTOR_TEST(dual_sensor_base_accepts_uv_mask_and_bus_current)
{
    const MotorHAL_t hal = makeDualHal();
    const MotorConfig cfg = makeDualConfig(hal);

    MotorConfigFaultDetail detail = MotorConfigFaultDetail::NONE;
    MOTOR_ASSERT_EQ(MotorConfigCheck::validateBase(cfg, &detail), Fault::NONE);
    MOTOR_ASSERT_EQ(detail, MotorConfigFaultDetail::NONE);
}

MOTOR_TEST(dual_sensor_base_rejects_invalid_masks)
{
    MotorHAL_t hal = makeDualHal(MOTOR_PHASE_CURRENT_U_VALID);
    MotorConfig cfg = makeDualConfig(hal);
    expectBaseDetail(cfg, MotorConfigFaultDetail::PHASE_CURRENT_DUAL_MASK_INVALID);

    hal = makeDualHal(MOTOR_PHASE_CURRENT_ALL_VALID);
    cfg = makeDualConfig(hal);
    expectBaseDetail(cfg, MotorConfigFaultDetail::PHASE_CURRENT_DUAL_MASK_INVALID);

    hal = makeDualHal(static_cast<uint8_t>(MOTOR_PHASE_CURRENT_U_VALID | 0x80U));
    cfg = makeDualConfig(hal);
    expectBaseDetail(cfg, MotorConfigFaultDetail::PHASE_CURRENT_MASK_INVALID);
}

MOTOR_TEST(command_direction_maps_api_speed_and_public_telemetry)
{
    const MotorHAL_t hal = makeDualHal();
    MotorConfig cfg = makeDualConfig(hal);
    cfg.control.command_direction = -1;

    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);

    MOTOR_ASSERT_EQ(manager.setTargetSpeed(600.0f), Result::Ok);
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(manager.ctx(), manager.ctx().target_rpm),
                      -600.0f,
                      1.0e-5f);

    MotorTelemetryChannel channels[] = {
        MotorTelemetryChannel::TargetRpm,
        MotorTelemetryChannel::SpeedRpm,
    };
    float values[2] = {};
    RuntimeCtx& ctx = manager.ctxMutForTest();
    ctx.speed_rpm = runtimeSpeedFromPhysical(ctx, -123.0f);
    MOTOR_ASSERT_EQ(manager.getTelemetryFloats(channels, 2U, values), 2U);
    MOTOR_ASSERT_NEAR(values[0], 600.0f, 1.0e-5f);
    MOTOR_ASSERT_NEAR(values[1], 123.0f, 1.0e-5f);
    MOTOR_ASSERT_NEAR(manager.getSpeed(), 123.0f, 1.0e-5f);

    MotorDebugData debug{};
    manager.getDebugData(debug);
    MOTOR_ASSERT_NEAR(debug.target_rpm, 600.0f, 1.0e-5f);

    // 当前 API 负方向尚未开放，拒绝后必须保留原目标。
    MOTOR_ASSERT_EQ(manager.setTargetSpeed(-400.0f), Result::InvalidParam);
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(manager.ctx(), manager.ctx().target_rpm),
                      -600.0f,
                      1.0e-5f);
    MOTOR_ASSERT_EQ(manager.getTelemetryFloats(channels, 1U, values), 1U);
    MOTOR_ASSERT_NEAR(values[0], 600.0f, 1.0e-5f);
}

MOTOR_TEST(command_direction_maps_velocity_setpoint)
{
    const MotorHAL_t hal = makeDualHal();
    MotorConfig cfg = makeDualConfig(hal);
    cfg.control.command_direction = -1;

    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);

    MotionSetpoint sp{};
    sp.mode = Mode::VELOCITY_CONTROL;
    sp.vel_ff = 600.0f;
    MOTOR_ASSERT_EQ(manager.writeSetpoint(sp), Result::Ok);
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(manager.ctx(), manager.ctx().target_rpm),
                      -600.0f,
                      1.0e-5f);

    sp.vel_ff = -250.0f;
    MOTOR_ASSERT_EQ(manager.writeSetpoint(sp), Result::InvalidParam);
    MOTOR_ASSERT_NEAR(runtimeSpeedToPhysical(manager.ctx(), manager.ctx().target_rpm),
                      -600.0f,
                      1.0e-5f);
}

MOTOR_TEST(command_direction_rejects_invalid_values)
{
    const MotorHAL_t hal = makeDualHal();
    MotorConfig cfg = makeDualConfig(hal);
    cfg.control.command_direction = 0;
    expectBaseDetail(cfg, MotorConfigFaultDetail::CONTROL_DIRECTION_INVALID);
}

MOTOR_TEST(smo_angle_branch_uses_internal_direction)
{
    MOTOR_ASSERT_NEAR(runtimeCorrectSmoAngleForDirection(0.25f, 1),
                      0.25f + PI,
                      1.0e-6f);
    MOTOR_ASSERT_NEAR(runtimeCorrectSmoAngleForDirection(5.75f, 1),
                      5.75f + PI - TWO_PI,
                      1.0e-6f);
    MOTOR_ASSERT_NEAR(runtimeCorrectSmoAngleForDirection(0.25f, -1),
                      0.25f,
                      1.0e-6f);
}

MOTOR_TEST(dual_sensor_kcl_reconstructs_missing_w_from_uv)
{
    g_raw_u = 2U;
    g_raw_v = 3U;
    g_raw_w = 99U;
    g_raw_bus = 4U;
    g_hw_flags = MOTOR_HAL_HW_FAULT_NONE;

    const MotorHAL_t hal = makeDualHal();
    MotorConfig cfg = makeDualConfig(hal);
    cfg.limit.max_phase_current_a = 1000.0f;
    cfg.limit.max_bus_current_a = 1000.0f;

    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);
    manager.tick();

    MotorMonitorData monitor{};
    manager.getMonitorData(monitor);
    MOTOR_ASSERT_NEAR(monitor.i_a, 2.0f, 1.0e-5f);
    MOTOR_ASSERT_NEAR(monitor.i_b, 3.0f, 1.0e-5f);
    MOTOR_ASSERT_NEAR(monitor.i_c, -5.0f, 1.0e-5f);
    MOTOR_ASSERT_NEAR(monitor.i_a + monitor.i_b + monitor.i_c, 0.0f, 1.0e-5f);
}

MOTOR_TEST(phase_overcurrent_reports_phase_detail)
{
    MotorLimitParam limits{};
    limits.max_phase_current_a = 10.0f;
    limits.max_bus_current_a = 10.0f;
    limits.over_voltage_v = 30.0f;
    limits.under_voltage_v = 0.0f;

    MotorSensorParam sensor{};
    sensor.has_bus_current = true;
    const float temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    Fault fault = Fault::NONE;
    MotorRuntimeFaultDetail detail = MotorRuntimeFaultDetail::NONE;

    MotorRuntimeMonitor monitor;
    monitor.check(limits, sensor, 11.0f, 0.0f, 0.0f, 0.0f,
                  12.0f, temp, State::RUN, &fault, &detail);

    MOTOR_ASSERT_TRUE(hasFault(fault, Fault::OVERCURRENT));
    MOTOR_ASSERT_EQ(detail, MotorRuntimeFaultDetail::SOFTWARE_PHASE_OVERCURRENT);
}

MOTOR_TEST(bus_current_disabled_ignores_bus_overcurrent)
{
    MotorLimitParam limits{};
    limits.max_phase_current_a = 10.0f;
    limits.max_bus_current_a = 1.0f;
    limits.over_voltage_v = 30.0f;
    limits.under_voltage_v = 0.0f;

    MotorSensorParam sensor{};
    sensor.has_bus_current = false;
    const float temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    Fault fault = Fault::NONE;
    MotorRuntimeFaultDetail detail = MotorRuntimeFaultDetail::NONE;

    MotorRuntimeMonitor monitor;
    monitor.check(limits, sensor, 0.0f, 0.0f, 0.0f, 50.0f,
                  12.0f, temp, State::RUN, &fault, &detail);

    MOTOR_ASSERT_EQ(fault, Fault::NONE);
    MOTOR_ASSERT_EQ(detail, MotorRuntimeFaultDetail::NONE);
}

MOTOR_TEST(bus_current_enabled_reports_bus_detail)
{
    MotorLimitParam limits{};
    limits.max_phase_current_a = 10.0f;
    limits.max_bus_current_a = 1.0f;
    limits.over_voltage_v = 30.0f;
    limits.under_voltage_v = 0.0f;

    MotorSensorParam sensor{};
    sensor.has_bus_current = true;
    const float temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    Fault fault = Fault::NONE;
    MotorRuntimeFaultDetail detail = MotorRuntimeFaultDetail::NONE;

    MotorRuntimeMonitor monitor;
    monitor.check(limits, sensor, 0.0f, 0.0f, 0.0f, 2.0f,
                  12.0f, temp, State::RUN, &fault, &detail);

    MOTOR_ASSERT_TRUE(hasFault(fault, Fault::OVERCURRENT));
    MOTOR_ASSERT_EQ(detail, MotorRuntimeFaultDetail::SOFTWARE_BUS_OVERCURRENT);
}

MOTOR_TEST(hardware_bus_comparator_fault_latches_overcurrent_detail)
{
    g_raw_u = 0U;
    g_raw_v = 0U;
    g_raw_w = 0U;
    g_raw_bus = 0U;
    g_hw_flags = MOTOR_HAL_HW_FAULT_NONE;

    const MotorHAL_t hal = makeDualHal();
    MotorConfig cfg = makeDualConfig(hal);

    MotorManager manager(cfg);
    manager.init();
    manager.setState(State::STOP);
    g_hw_flags = MOTOR_HAL_HW_FAULT_TIM1_BREAK;
    manager.tick();

    MOTOR_ASSERT_TRUE(hasFault(manager.fault(), Fault::OVERCURRENT));
    MOTOR_ASSERT_EQ(manager.runtimeFaultDetail(),
                    MotorRuntimeFaultDetail::HARDWARE_BUS_OVERCURRENT_COMPARATOR);
}

int main()
{
    return MotorHostTest::runAll();
}
