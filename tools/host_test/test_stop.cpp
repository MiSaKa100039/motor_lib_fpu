#include "MotorTestRunner.h"
#include "Motor_Manager.h"

using namespace Lib_Motor;

namespace
{
enum class Output { Off, Pwm, Brake };
Output output = Output::Off;
unsigned brake_calls = 0, enable_calls = 0, clear_calls = 0, sample_calls = 0;
uint16_t raw_current = 0;
uint32_t hardware_fault = MOTOR_HAL_HW_FAULT_NONE;

void noop() {}
void duty(float, float, float) {}
void dutyQ15(std::int16_t, std::int16_t, std::int16_t) {}
void enable() { output = Output::Pwm; ++enable_calls; }
void disable() { output = Output::Off; }
void brake() { output = Output::Brake; ++brake_calls; }
void samples(uint16_t* a, uint16_t* b, uint16_t* c, uint16_t* bus)
{
    ++sample_calls;
    *a = *b = *c = *bus = raw_current;
}
uint32_t faults() { return hardware_fault; }
void clearFaults(uint32_t) { ++clear_calls; hardware_fault = 0; }

MotorHAL_t makeHal()
{
    MotorHAL_t hal{};
    hal.set_duty = duty;
    hal.set_duty_q15 = dutyQ15;
    hal.pwm_enable = enable;
    hal.pwm_disable = disable;
    hal.pwm_coast = disable;
    hal.pwm_brake_lowside = brake;
    hal.read_currents_raw = samples;
    hal.phase_current_valid_mask = MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_V_VALID;
    hal.enter_critical = noop;
    hal.exit_critical = noop;
    hal.read_hardware_fault_flags = faults;
    hal.clear_hardware_fault_flags = clearFaults;
    return hal;
}

MotorConfig makeConfig(const MotorHAL_t& hal)
{
    MotorConfig cfg{};
    cfg.hal = &hal;
    cfg.physical.pole_pairs = 4;
    cfg.control.control_freq_hz = 12000.0f;
    cfg.control.startup_target_mode = Mode::DEBUG_IF_SMO_OBSERVER;
    cfg.sensor.current_sense_mode = CurrentSenseMode::DUAL_SENSOR;
    cfg.sensor.adc_v_ref = 1.0f;
    cfg.sensor.adc_resolution = 1000.0f;
    cfg.sensor.phase_shunt_resistor = 1.0f;
    cfg.sensor.phase_amp_gain = 1.0f;
    cfg.sensor.bus_shunt_resistor = 1.0f;
    cfg.sensor.bus_amp_gain = 1.0f;
    cfg.sensor.has_bus_voltage = false;
    cfg.sensor.has_bus_current = true;
    cfg.sensor.has_phase_voltage = false;
    cfg.sensor.adc_calibration_samples = 8;
    cfg.sensor.auto_calib_interval_s = 0.001f;
    cfg.limit.max_phase_current_a = 100.0f;
    cfg.limit.max_bus_current_a = 100.0f;
    cfg.limit.over_voltage_v = 24.0f;
    cfg.limit.under_voltage_v = 0.0f;
    cfg.brake.allow_low_side_brake = true;
    cfg.observer.allow_smo_closed_loop = false;
    return cfg;
}

struct Fixture
{
    MotorHAL_t hal = makeHal();
    MotorConfig cfg = makeConfig(hal);
    MotorManager manager{cfg};
    Fixture()
    {
        raw_current = 0;
        hardware_fault = 0;
        output = Output::Off;
        manager.init();
        manager.setState(State::STOP);
        brake_calls = enable_calls = clear_calls = sample_calls = 0;
        MOTOR_ASSERT_EQ(manager.fault(), Fault::NONE);
    }
    void ticks(unsigned count)
    {
        while (count-- != 0) manager.tick();
        MOTOR_ASSERT_EQ(manager.fault(), Fault::NONE);
    }
    void stop(StopMode mode)
    {
        manager.requestStop(mode);
        ticks(2);
        MOTOR_ASSERT_EQ(manager.state(), State::STOP);
    }
    void expectOffsets(int value)
    {
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        MOTOR_ASSERT_EQ(manager.ctx().offset_ia_counts, value);
        MOTOR_ASSERT_EQ(manager.ctx().offset_ib_counts, value);
        MOTOR_ASSERT_EQ(manager.ctx().offset_ic_counts, value);
        MOTOR_ASSERT_EQ(manager.ctx().offset_ibus_counts, value);
#else
        MOTOR_ASSERT_EQ(manager.ctx().offset_ia, value);
        MOTOR_ASSERT_EQ(manager.ctx().offset_ib, value);
        MOTOR_ASSERT_EQ(manager.ctx().offset_ic, value);
        MOTOR_ASSERT_EQ(manager.ctx().offset_ibus, value);
#endif
    }
};
}

MOTOR_TEST(stop_can_release_reapply_and_repeat_brake)
{
    Fixture f;
    f.stop(StopMode::ELECTRICAL_BRAKE);
    MOTOR_ASSERT_EQ(output, Output::Brake);
    f.ticks(100);
    MOTOR_ASSERT_EQ(output, Output::Brake);
    MOTOR_ASSERT_EQ(brake_calls, 1U);
    MOTOR_ASSERT_TRUE(sample_calls >= 100U);
    f.stop(StopMode::ELECTRICAL_BRAKE);
    MOTOR_ASSERT_EQ(brake_calls, 2U);
    f.stop(StopMode::COAST);
    MOTOR_ASSERT_EQ(output, Output::Off);
    f.stop(StopMode::COAST);
    MOTOR_ASSERT_EQ(output, Output::Off);
    f.stop(StopMode::ELECTRICAL_BRAKE);
    MOTOR_ASSERT_EQ(output, Output::Brake);
    MOTOR_ASSERT_EQ(brake_calls, 3U);
}

MOTOR_TEST(stop_brake_restart_enters_original_run_path)
{
    Fixture f;
    f.stop(StopMode::ELECTRICAL_BRAKE);
    f.manager.setMode(Mode::DEBUG_IF_SMO_OBSERVER);
    f.manager.requestStart();
    f.ticks(1);
    MOTOR_ASSERT_EQ(f.manager.state(), State::RUN);
    MOTOR_ASSERT_EQ(output, Output::Pwm);
    MOTOR_ASSERT_EQ(enable_calls, 1U);
    f.stop(StopMode::ELECTRICAL_BRAKE);
    MOTOR_ASSERT_EQ(output, Output::Brake);
}

MOTOR_TEST(stop_fault_has_priority_over_brake_and_restart)
{
    Fixture f;
    f.stop(StopMode::ELECTRICAL_BRAKE);
    hardware_fault = MOTOR_HAL_HW_FAULT_TIM1_BREAK;
    f.manager.requestStop(StopMode::ELECTRICAL_BRAKE);
    f.manager.tick();
    MOTOR_ASSERT_EQ(f.manager.state(), State::ERROR);
    MOTOR_ASSERT_EQ(output, Output::Off);
    MOTOR_ASSERT_EQ(brake_calls, 1U);
    f.manager.setMode(Mode::DEBUG_IF_SMO_OBSERVER);
    f.manager.requestStart();
    f.manager.tick();
    MOTOR_ASSERT_EQ(f.manager.state(), State::ERROR);
    MOTOR_ASSERT_EQ(output, Output::Off);
    MOTOR_ASSERT_EQ(enable_calls, 0U);
    MOTOR_ASSERT_EQ(clear_calls, 0U);
    MOTOR_ASSERT_EQ(hardware_fault, MOTOR_HAL_HW_FAULT_TIM1_BREAK);
}

MOTOR_TEST(stop_missing_brake_callback_falls_back_to_coast)
{
    Fixture f;
    f.hal.pwm_brake_lowside = nullptr;
    f.stop(StopMode::ELECTRICAL_BRAKE);
    MOTOR_ASSERT_EQ(output, Output::Off);
    MOTOR_ASSERT_EQ(brake_calls, 0U);
}

MOTOR_TEST(stop_brake_discards_partial_calibration_and_restarts_wait)
{
    Fixture f;
    raw_current = 10;
    // 先形成已确认的零偏，再中断下一轮校准。
    f.ticks(20);
    f.expectOffsets(10);
    raw_current = 20;
    f.ticks(15);
    MOTOR_ASSERT_TRUE(f.manager.ctx().silent_calib_active);
    MOTOR_ASSERT_TRUE(f.manager.ctx().calib_counter > 0U);
    MOTOR_ASSERT_TRUE(f.manager.ctx().calib_counter < 8U);
    f.stop(StopMode::ELECTRICAL_BRAKE);
    raw_current = 500;
    f.ticks(100);
    f.expectOffsets(10);
    MOTOR_ASSERT_TRUE(!f.manager.ctx().silent_calib_active);
    MOTOR_ASSERT_EQ(f.manager.ctx().silent_calib_timer_ticks, 0U);
    MOTOR_ASSERT_EQ(f.manager.ctx().calib_counter, 0U);
    MOTOR_ASSERT_EQ(f.manager.ctx().calib_accum_ia, 0U);
    MOTOR_ASSERT_EQ(f.manager.ctx().calib_accum_ib, 0U);
    MOTOR_ASSERT_EQ(f.manager.ctx().calib_accum_ic, 0U);
    MOTOR_ASSERT_EQ(f.manager.ctx().calib_accum_ibus, 0U);
    raw_current = 30;
    f.stop(StopMode::COAST);
    MOTOR_ASSERT_EQ(f.manager.ctx().silent_calib_timer_ticks, 1U);
    f.ticks(5);
    MOTOR_ASSERT_TRUE(!f.manager.ctx().silent_calib_active);
    f.expectOffsets(10);
    f.ticks(14);
    f.expectOffsets(30);
}
