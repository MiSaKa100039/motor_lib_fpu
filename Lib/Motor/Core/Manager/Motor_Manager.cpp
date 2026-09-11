#include "Motor_Manager.h"

#include "../FSM/Motor_FSM.h"
#include "../Limit/Motor_TargetLimiter.h"
#include "../../Control/Utils/FocMath.h"

#include <cmath>

namespace Lib_Motor
{

/* 临时 PA5 分段探针，测完后移除。 */
constexpr uint32_t kGpioaBsrrAddress = 0x4800001CUL;
constexpr uint32_t kPa5ScopeProbePin = (1UL << 5);

static inline void setPa5ScopeProbe()
{
    *reinterpret_cast<volatile uint32_t*>(kGpioaBsrrAddress) =
        kPa5ScopeProbePin;
}

static inline void resetPa5ScopeProbe()
{
    *reinterpret_cast<volatile uint32_t*>(kGpioaBsrrAddress) =
        (kPa5ScopeProbePin << 16U);
}

static inline float clampFloatFast(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && MOTOR_BUILD_NTC_SLOTS > 0
static int16_t temperatureCToDeci(float value)
{
    if (!std::isfinite(value))
    {
        return 32767;
    }

    const float scaled = value * 10.0f;
    int32_t rounded = (scaled >= 0.0f)
        ? static_cast<int32_t>(scaled + 0.5f)
        : static_cast<int32_t>(scaled - 0.5f);

    if (rounded > 32767)
    {
        return 32767;
    }
    if (rounded < -32768)
    {
        return -32768;
    }
    return static_cast<int16_t>(rounded);
}
#endif

MotorManager::MotorManager(const MotorConfig& config)
    : config_(config)
    , active_policy_(config.default_run_policy)
    , pending_policy_(config.default_run_policy)
    , ctx_()
    , controller_()
{
    // 单次控制周期时间 (秒): dt_ = 1 / control_freq_hz
    // 例: control_freq_hz = 10000 Hz → dt_ = 0.0001 s = 100 μs
    dt_ = 1.0f / config_.control.control_freq_hz;

    // 将用户配置的秒级静默校准间隔转换为 tick 数
    //   ticks = seconds / dt_ = seconds * control_freq_hz
    // 例: auto_calib_interval_s = 30.0 s, 10000 Hz → 300000 ticks
    // 若用户设为 0 → 禁用静默校准 (auto_calib_interval_ticks_ 保持 0,
    // tick() 中的 else if (auto_calib_interval_ticks_ > 0) 不命中)
    auto_calib_interval_ticks_ = config_.sensor.auto_calib_interval_s > 0.0f
        ? (uint32_t)(config_.sensor.auto_calib_interval_s / dt_)
        : 0;
}

/*
 * init() -- 上电初始化, 填充运行时上下文并触发自动标定
 *
 * 调用序列:
 *   [A] 复位 FSM 状态机变量 (state/mode/fault/event)
 *   [B] 清空 RuntimeCtx (传感器/变换/控制中间量)
 *   [C] 计算物理量换算系数 (ADC count -> 物理单位)
 *   [D] 配置低通滤波器参数
 *   [E] 初始化电流 PID 控制器
 *
 * 调用链:
 *   - 平台初始化层将调用转发到此处 (MotorAPI::init -> MotorManager::init)
 *   - reset() 等效于 re-init()
 */
void MotorManager::init()
{
    /* ================================================================
     * [A] FSM 状态变量复位
     * ================================================================ */
    state_       = State::INIT;
    mode_        = Mode::NONE;
    target_mode_ = config_.control.startup_target_mode;
    run_requested_ = false;
    run_phase_   = RunPhase::NONE;
    fault_       = Fault::NONE;
    config_fault_detail_ = MotorConfigFaultDetail::NONE;
    runtime_fault_detail_ = MotorRuntimeFaultDetail::NONE;
    hardware_fault_flags_ = MOTOR_HAL_HW_FAULT_NONE;
    safety_state_ = SafetyState::CLEAR;
    command_state_ = CommandState::IDLE;
    stream_state_ = StreamState::NONE;
    resetSetpointPlayback();
    angle_state_ = AngleState::NONE;
    stop_state_ = StopState::IDLE;
    energy_state_ = EnergyState::IDLE;
    event_       = MotorEvent();
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    resetSmoAngleDirectionLatch();
#if LIB_MOTOR_ENABLE_SMO
    smo_handover_confirm_ticks_ = 0U;
    smo_fusion_ticks_ = 0U;
    smo_fusion_total_ticks_ =
        (config_.observer.smo_handover.blend_time_s > 0.0f && dt_ > 0.0f)
            ? static_cast<uint32_t>(
                  config_.observer.smo_handover.blend_time_s / dt_ + 0.5f)
            : 1U;
    if (smo_fusion_total_ticks_ == 0U) smo_fusion_total_ticks_ = 1U;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    smo_fusion_angle_offset_q15_ = 0;
    smo_fusion_progress_q15_ = 0U;
    smo_fusion_step_q15_ = static_cast<uint16_t>(
        32768UL / smo_fusion_total_ticks_);
    smo_fusion_remainder_q15_ = 32768UL % smo_fusion_total_ticks_;
    smo_fusion_remainder_accum_ = 0U;
    smo_handover_angle_error_phase_ = 0U;
#else
    smo_fusion_angle_offset_rad_ = 0.0f;
    smo_handover_angle_error_rad_ = 0.0f;
#endif
    smo_fusion_start_speed_ = 0;
#endif
#endif
    clearRuntimeFaultDetail();
    active_policy_ = config_.default_run_policy;
    pending_policy_ = config_.default_run_policy;
#if LIB_MOTOR_ENABLE_IF_STARTUP && LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    cacheIFStartupRestartInterval();
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    if_profile_hold_target_ = false;
#endif
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    rl_identify_routine_.reset();
#endif
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
    energy_budget_ = MotorEnergyBudget{};
    budget_active_ = false;
    budget_loss_blocked_ = false;
    budget_expire_ticks_ = 0U;
#endif

    /* ================================================================
     * [B] Runtime 上下文清零 (传感器读数/变换中间量/控制输出/故障计数等)
    * ================================================================ */
    ctx_ = RuntimeCtx{};
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && MOTOR_BUILD_NTC_SLOTS > 0
    for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; ++i)
    {
        ctx_.ntc_over_temp_deci_c[i] = temperatureCToDeci(config_.sensor.ntc[i].over_temp_c);
    }
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.debug_ramp_ticks =
        (dt_ > 0.0f) ? static_cast<uint32_t>((2.0f / dt_) + 0.5f) : 0U;
#else
    ctx_.debug_ramp_time_s = 2.0f; // 默认斜坡时间
#endif
#endif
    ctx_.sensor_health = SensorHealth::NOT_CONFIGURED;
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    ctx_.redundant_sensor_health = SensorHealth::NOT_CONFIGURED;
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    ctx_.output_sensor_health = SensorHealth::NOT_CONFIGURED;
#endif
    sensor_invalid_ticks_ = 0U;
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    redundant_error_ticks_ = 0U;
#endif
    clearStallState();
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    stall_total_count_ = 0U;
#endif

    /* ================================================================
     * [C] 计算 ADC count -> 物理单位换算系数
     *
     * 电流传感器换算:
     *   HALL_SENSOR: Hall ADC 零点校准后按灵敏度换算 -> 电流 (A)
     *     current_A = (raw_adc - offset_adc) * 1000 * V_ref /
     *                 (adc_res * sensitivity_mV_per_A)
     *     其中 scale = 1000 * V_ref / (adc_res * sensitivity)
     *
     *   SHUNT_RESISTOR: 分流电阻采样 -> 电流 (A)
     *     current_A = V_shunt / (R_shunt * amp_gain)
     *     V_shunt = raw_adc * V_ref / adc_res
     *     其中 scale = V_ref / (adc_res * R_shunt * amp_gain)
     *
     * 电压传感器换算:
     *   V_bus = V_ADC * (R_up + R_down) / R_down
     *   V_ADC = raw_adc * V_ref / adc_res
     *   其中 scale = V_ref / adc_res * divider_ratio
     * ================================================================ */
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    configureFixedRuntimeScales();
#else
    float adc_res = config_.sensor.adc_resolution;
    float v_ref   = config_.sensor.adc_v_ref;
    if (config_.sensor.current_sensor_type == CurrentSensorType::HALL_SENSOR)
    {
        // Hall 传感器: ADC -> mV -> A
        ctx_.current_scale_Phase_per_count = 1000.0f * v_ref / (adc_res * config_.sensor.hall_sensitivity_mV_per_A);
    }
    else
    {
        // 分流电阻: ADC -> V -> A
        ctx_.current_scale_Phase_per_count = v_ref / (adc_res * config_.sensor.phase_shunt_resistor * config_.sensor.phase_amp_gain);
    }
#if MOTOR_BUILD_HAS_BUS_CURRENT
    ctx_.current_scale_Bus_per_count   = v_ref / (adc_res * config_.sensor.bus_shunt_resistor * config_.sensor.bus_amp_gain);
#endif

    float vbus_divider_ratio = (config_.sensor.vbus_r_up + config_.sensor.vbus_r_down) / config_.sensor.vbus_r_down;
    ctx_.voltage_scale_V_per_count = v_ref / adc_res * vbus_divider_ratio;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    float vphase_divider_ratio = (config_.sensor.vphase_r_up + config_.sensor.vphase_r_down) / config_.sensor.vphase_r_down;
    ctx_.phase_voltage_scale_V_per_count = v_ref / adc_res * vphase_divider_ratio;
#endif
#endif

    /* ================================================================
     * [D] 低通滤波器初始化
     *
     * 相电流/Id/Iq 控制需要平滑的反馈信号, 因此对原始 ADC 转换值进行 IIR 低通滤波。
     * 滤波时间常数由配置决定。硬件 Platform 层可能已做 R-C 滤波或软件 IIR,
     * 但驱动层仍保留独立的一阶 LPF 以应对不同硬件。
     *
     * 系数 α = Ts / (Tf + Ts),  Ts = 1/control_freq_hz = dt_
     * ================================================================ */
    lpf_ia_.setTf(config_.sensor.phase_current_lpf_tf_s);
    lpf_ib_.setTf(config_.sensor.phase_current_lpf_tf_s);
    lpf_ic_.setTf(config_.sensor.phase_current_lpf_tf_s);
#if MOTOR_BUILD_HAS_BUS_CURRENT
    lpf_ibus_.setTf(config_.sensor.bus_current_lpf_tf_s);
#endif
    lpf_vbus_.setTf(config_.sensor.bus_voltage_lpf_tf_s);
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    lpf_vphase_u_.setTf(config_.sensor.phase_voltage_lpf_tf_s);
    lpf_vphase_v_.setTf(config_.sensor.phase_voltage_lpf_tf_s);
    lpf_vphase_w_.setTf(config_.sensor.phase_voltage_lpf_tf_s);
#endif
    lpf_ia_.setSamplePeriod(dt_);
    lpf_ib_.setSamplePeriod(dt_);
    lpf_ic_.setSamplePeriod(dt_);
#if MOTOR_BUILD_HAS_BUS_CURRENT
    lpf_ibus_.setSamplePeriod(dt_);
#endif
    lpf_vbus_.setSamplePeriod(dt_);
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    lpf_vphase_u_.setSamplePeriod(dt_);
    lpf_vphase_v_.setSamplePeriod(dt_);
    lpf_vphase_w_.setSamplePeriod(dt_);
#endif

    /* ================================================================
     * [E] PID 控制器参数初始化 (从配置读取 KP/KI/KD/OutputLimit/RampRate)
     * ================================================================ */
    controller_.init(config_);
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    const float speed_pid_output_limit =
        MotorTargetLimiter::configuredSpeedPidOutputLimit(config_);
    controller_.setSpeedOutputLimit(speed_pid_output_limit);
#endif
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    configureFixedCurrentPidRuntime(true);
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    controller_.syncFixedSpeedPid(
        config_, dt_, speed_pid_output_limit);
#endif
#endif

    /* ================================================================
     * [F] 观测器初始化 (SMO/传感器观测器/冗余传感器/输出传感器)
     * ================================================================ */
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    /* [P2] SMO 用 effective R/L (来源由 physical.electrical_param_source 决定) */
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    smo_.init(config_.physical.rs_effective(), config_.physical.ls_effective(),
              config_.observer.smo_gain,
              config_.observer.smo_pll_kp, config_.observer.smo_pll_ki,
              runtimeCurrentBaseA(ctx_), runtimeControlVoltageBaseV(ctx_),
              runtimeSpeedBaseRpm(ctx_), dt_, config_.physical.pole_pairs,
              config_.observer.smo_bemf_lpf_cutoff_hz);
#else
    smo_.init(config_.physical.rs_effective(), config_.physical.ls_effective(),
              config_.observer.smo_gain,
              config_.observer.smo_pll_kp, config_.observer.smo_pll_ki,
              config_.observer.smo_bemf_lpf_cutoff_hz);
#endif
    smo_.setValidityCriteria(
        config_.observer.smo_validity.acquire_min_speed_rpm *
            TWO_PI * config_.physical.pole_pairs / 60.0f,
        config_.observer.smo_validity.acquire_max_pll_error_rad,
        config_.observer.smo_validity.acquire_ticks,
        config_.observer.smo_validity.acquire_min_signal_level,
        config_.observer.smo_validity.release_min_speed_rpm *
            TWO_PI * config_.physical.pole_pairs / 60.0f,
        config_.observer.smo_validity.release_max_pll_error_rad,
        config_.observer.smo_validity.release_ticks,
        config_.observer.smo_validity.release_min_signal_level);
#endif
#if LIB_MOTOR_ENABLE_HFI
    hfi_.init(config_.observer.hfi_injection_mode,
              config_.observer.hfi_sine_injection_freq_hz,
              0.0f,
              config_.observer.hfi_pll_kp,
              config_.observer.hfi_pll_ki,
              config_.observer.hfi_lock_ticks,
              config_.observer.hfi_min_signal_level_a,
              config_.observer.hfi_voltage_ramp_time_s,
              config_.observer.hfi_square_current_scale,
              config_.observer.hfi_sine_lpf_cutoff_ratio,
              config_.observer.hfi_square_lpf_cutoff_ratio,
              config_.observer.hfi_pll_integral_limit_rad_s,
              config_.observer.hfi_pll_speed_limit_rad_s);
#endif
#if LIB_MOTOR_ENABLE_SENSOR
    rotor_sensor_obs_.init(config_.position.rotor_sensor, config_.physical.pole_pairs);
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    redundant_sensor_obs_.init(config_.position.redundant_rotor_sensor, config_.physical.pole_pairs);
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    output_sensor_obs_.init(config_.position.output_sensor, config_.physical.pole_pairs);
#endif
    if (config_.position.rotor_sensor != nullptr && config_.position.rotor_sensor->init != nullptr)
    {
        config_.position.rotor_sensor->init(config_.position.rotor_sensor->ctx);
    }
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    if (config_.position.enable_redundant_rotor_sensor &&
        config_.position.redundant_rotor_sensor != nullptr &&
        config_.position.redundant_rotor_sensor->init != nullptr)
    {
        config_.position.redundant_rotor_sensor->init(config_.position.redundant_rotor_sensor->ctx);
    }
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    if (config_.position.enable_output_sensor &&
        config_.position.output_sensor != nullptr &&
        config_.position.output_sensor->init != nullptr)
    {
        config_.position.output_sensor->init(config_.position.output_sensor->ctx);
    }
#endif
#endif
    /* ================================================================
     * [G] IF 强拖角度发生器初始化
     * ================================================================ */
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    if_angle_gen_.init(config_.control.control_freq_hz,
                       config_.physical.pole_pairs,
                       runtimeSpeedBaseRpm(ctx_));
#else
    if_angle_gen_.init(config_.control.control_freq_hz, config_.physical.pole_pairs);
#endif
#endif
    /* ★ 安全: 初始化完成后强制关闭 PWM 输出 */
    if (config_.hal && config_.hal->pwm_disable) config_.hal->pwm_disable();
    writePwmZeroOutputs();
}

/* reset() -- 与 init() 类似, 上电重新初始化上下文和自动标定 */
void MotorManager::reset()
{
    init();
}

/* setState -- 直接设置 FSM 状态 (由 Motor_FSM 调用, 不经过 API 安全校验) */
void MotorManager::setState(State next_state)
{
    state_ = next_state;
}

/* setMode -- 设置控制模式, FSM 在 RUN 状态时将 mode_ = target_mode_ */
void MotorManager::enterIsrDiagnostic(Fault fault)
{
    mode_ = Mode::NONE;
    target_mode_ = Mode::NONE;
    run_requested_ = false;
    run_phase_ = RunPhase::NONE;
    state_ = State::ISR_DIAGNOSTIC;
    fault_ = fault;
    runtime_fault_detail_ = MotorRuntimeFaultDetail::NONE;
    hardware_fault_flags_ = MOTOR_HAL_HW_FAULT_NONE;
    safety_state_ = (fault == Fault::NONE) ? SafetyState::CLEAR : SafetyState::FAULT_LATCHED;
    command_state_ = (fault == Fault::NONE) ? CommandState::IDLE : CommandState::FAULTED;
    stream_state_ = StreamState::NONE;
    angle_state_ = AngleState::NONE;
    stop_state_ = StopState::IDLE;
    energy_state_ = EnergyState::IDLE;
    ctx_.duty_a = 0;
    ctx_.duty_b = 0;
    ctx_.duty_c = 0;
    if (config_.hal != nullptr && config_.hal->pwm_disable != nullptr)
    {
        config_.hal->pwm_disable();
    }
    writePwmZeroOutputs();
}

void MotorManager::setMode(Mode mode)
{
    Motor_FSM::selectMode(*this, mode);
}

void MotorManager::writePwmOutputs()
{
    if (config_.hal == nullptr)
    {
        return;
    }

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    if (config_.hal->set_duty_q15 != nullptr)
    {
        config_.hal->set_duty_q15(ctx_.duty_a,
                                  ctx_.duty_b,
                                  ctx_.duty_c);
    }
    return;
#else
    if (config_.hal->set_duty != nullptr)
    {
        config_.hal->set_duty(ctx_.duty_a, ctx_.duty_b, ctx_.duty_c);
    }
#endif
}

void MotorManager::writePwmZeroOutputs()
{
    if (config_.hal == nullptr)
    {
        return;
    }

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    if (config_.hal->set_duty_q15 != nullptr)
    {
        config_.hal->set_duty_q15(0, 0, 0);
    }
#else
    if (config_.hal->set_duty != nullptr)
    {
        config_.hal->set_duty(0.0f, 0.0f, 0.0f);
    }
#endif
}

void MotorManager::serviceSlowMonitor()
{
    if (state_ == State::UNINITIALIZED ||
        state_ == State::ISR_DIAGNOSTIC)
    {
        return;
    }

    /* 兼容入口: 安全刷新已由 tick() 每 ADC tick 执行, 外部不再需要依赖本函数。 */
    updateSensorMeasurements();

#if LIB_MOTOR_ENABLE_SENSOR
    if (state_ != State::RUN)
    {
        updatePositionSensorMeasurement();
    }
#endif

    if (state_ != State::INIT && state_ != State::ADC_CAL)
    {
        runSafetyCheck();
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        processPendingStallRestart();
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
        processPendingStartupRestart();
#endif
        if (fault_ != Fault::NONE || state_ == State::ERROR)
        {
            Motor_FSM::step(*this);
        }
    }
}

void MotorManager::requestStart()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15 && LIB_MOTOR_ENABLE_VELOCITY_CONTROL
    ctx_.speed_slew_fraction_q16 = 0U;
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    clearStartupRestartState();
    if (active_policy_.startup_source == StartupSource::IF)
    {
        if (target_mode_ == Mode::VELOCITY_CONTROL &&
            active_policy_.steady_source == SteadyAngleSource::SMO &&
            !if_profile_hold_target_)
        {
            const float target_rpm = runtimeSpeedToUserPhysical(
                config_, ctx_, ctx_.target_rpm);
            if (target_rpm >= 0.0f &&
                target_rpm < config_.observer.smo_validity.acquire_min_speed_rpm)
            {
                if_profile_hold_target_ = true;
            }
        }
        resetIFStartupProfileState();
        resetIFStartupObserverState();
        prepareIFStartupRuntimeForStart();
    }
#endif
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    resetSmoAngleDirectionLatch();
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY && LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    prepareDebugOpenLoopAngleRampForStart();
#endif
    Motor_FSM::requestStart(*this);
}

/*
 * requestStop -- 请求电机停止
 * @param mode 停止模式: COAST / ELECTRICAL_BRAKE / MECHANICAL_BRAKE
 *
 * 说明:
 *   这里只登记停机意图，真正的 STOPPING -> STOP 收敛
 *   由顶层 Motor_FSM 在生命周期管线中完成。
 */
void MotorManager::requestStop(StopMode mode)
{
#if LIB_MOTOR_ENABLE_IF_STARTUP
    clearStartupRestartState();
#endif
    Motor_FSM::requestStop(*this, mode);
}

void MotorManager::requestEmergencyStop()
{
#if LIB_MOTOR_ENABLE_IF_STARTUP
    clearStartupRestartState();
#endif
    Motor_FSM::requestEmergencyStop(*this);
}

/* clearFaultBits -- 通过顶层 Motor_FSM 清除可恢复故障位与相关子状态 */
void MotorManager::clearFaultBits()
{
    Motor_FSM::clearFaultBitsNow(*this);
}

/*
 * clearFaultAndStop -- 清除故障并立即停机
 *
 * 语义:
 *   - 清除可恢复故障锁存
 *   - 清除角度源附加计数器
 *   - 推入 STOPPING
 *   - 立即通过顶层 Motor_FSM 完成一次同步状态推进，收敛到 STOP
 *
 * 说明:
 *   这里不再由 Manager 直接调用 StopFSM::step()/CommandFSM::clearCommand()，
 *   而是统一委托给 Motor_FSM::clearFaultAndStopNow() 封装顶层事务。
 */
void MotorManager::clearFaultAndStop()
{
    Motor_FSM::clearFaultAndStopNow(*this);
}

/*
 * tick() -- 主循环调度入口 (由 ADC ISR -> Motor_Global_Process_Handler 触发)
 *
 * 运行频率: control_freq_hz (通常 20kHz, 即 dt_ = 50us)
 *
 * 调用序列 (严格顺序):
 *
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │ tick() 入口                                                      │
 *   ├──────────────────────────────────────────────────────────────────┤
 *   │ fsm_timer_ticks++                                                │
 *   │                                                                  │
 *   │ [1] ISR_DIAGNOSTIC 早退                                           │
 *   │     普通 ERROR 状态仍继续完整 tick, 只由 state gate 禁止 PWM 输出    │
 *   │                                                                  │
 *   │ [2] updateSensorMeasurements()                                   │
 *   │     读取 BSP raw 快照 -> q15 快环输入 + float 物理量/低通监控       │
 *   │                                                                  │
 *   │ [3] runSafetyCheck()                                             │
 *   │     硬件 latch + LPF 后电流/电压/温度检查 -> 合并到 fault_ 寄存器    │
 *   │                                                                  │
 *   │ [4] Motor_FSM::step(*this)                                       │
 *   │     故障快速关断 (fault!=NONE -> PWM关闭 -> ERROR)                │
 *   │     状态机转换: INIT->ADC_CAL->STOP / STOP->RUN / RUN->ERROR      │
 *   │                                                                  │
 *   │ [5] 控制策略分支 (由 state_ / mode_ 决定)                         │
 *   │     ADC_CAL  -> runAdcCalibration()                               │
 *   │     RUN + DEBUG_PWM   -> runPWMManual()                          │
 *   │     RUN + CURRENT_LOCK -> runCurrentLock()                        │
 *   │     RUN + IF_DRAG     -> runIFControl()                          │
 *   │     RUN + VF_DRAG     -> runVFControl()                          │
 *   │     RUN + CALIB_RL    -> runCurrentLock() (拖拽辨识)               │
 *   │     RUN + 其它        -> runControlLoop() (FOC 闭环)              │
 *   │                                                                  │
 *   │ [6] Duty 限幅: clamp(duty, 0, max_duty_cycle)                    │
 *   │                                                                  │
 *   │ [7] PWM 输出: 仅在 RUN 状态调用 set_duty()/set_duty_q15()         │
 *   │     非RUN: MOE 安全关断 FSM 控制 (pwm_coast/pwm_disable)         │
 *   └──────────────────────────────────────────────────────────────────┘
 */

void MotorManager::tick()
{
    ctx_.fsm_timer_ticks++;  // 每个 tick 递增, 由 FSM 用于超时判断 (如 ALIGNMENT 持续时间)

    /* [1] ISR 诊断态只保留中断节拍, 不运行采样、保护和控制链。 */
    if (state_ == State::ISR_DIAGNOSTIC)
    {
        return;
    }

    updateSensorMeasurements(); //10us

#if LIB_MOTOR_ENABLE_SENSOR
    updatePositionSensorMeasurement();
#endif

    /* [2] 安全检查 (合并到 fault_ 寄存器) */
    runSafetyCheck();   //3us

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    /* 堵转自动重启: 检查是否处于 STOP 并需要重新触发 run_requested_ */
    processPendingStallRestart();
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    processPendingStartupRestart();
#endif

    /* [3] 状态机编排 -- 故障锁存 + 状态转换 */
    Motor_FSM::step(*this); //1.7us

    /* 预加载回放必须在控制环读取目标前服务, 当前阶段为空框架。 */
    serviceSetpointPlayback();  //150ns

    /* ================================================================
     * [4] 控制策略: ADC 校准
     *     每个 tick 读取一次 ADC, calib_counter 递增
     *     计数器由 FSM::handleAdcCal 管理和停止
     * ================================================================ */
    if (state_ == State::ADC_CAL)
    {
        runAdcCalibration();
    }

    /* ================================================================
     * [静默校准] 静止 STOP 子状态
     *
     * 按照 auto_calib_interval_s 周期性地进行 ADC 偏移校准,
     * 以补偿温度漂移导致的 ADC 零点偏移。
     *
     * 状态机 (三阶段):
     *   [idle] silent_calib_timer_ticks 递增
     *   [校准中] 清零 calib_counter/accum -> 每个 tick 累加
     *   [采样]   每个 tick 调用 runAdcCalibration() 累积 ADC 值
     *   [完成]   calib_counter >= adc_calibration_samples -> 存储 offset_*
     *            恢复 idle 状态
     *
     * 注意: 静默校准仅在 STOP 状态执行, 进入 STOP 时复位
     * ================================================================ */
    // 短路制动时绕组仍可能有电流，不能把该电流重新标定为零点。
    if (state_ == State::STOP && stop_state_ != StopState::ELECTRICAL_BRAKE)
    {
        if (ctx_.silent_calib_active)
        {
            runAdcCalibration();
            if (ctx_.calib_counter >= config_.sensor.adc_calibration_samples)
            {
                ctx_.silent_calib_active = false;
            }
        }
        else if (auto_calib_interval_ticks_ > 0)
        {
            ctx_.silent_calib_timer_ticks++;
            if (ctx_.silent_calib_timer_ticks >= auto_calib_interval_ticks_)
            {
                ctx_.calib_counter = 0;
                ctx_.calib_accum_ia = 0;
                ctx_.calib_accum_ib = 0;
                ctx_.calib_accum_ic = 0;
#if MOTOR_BUILD_HAS_BUS_CURRENT
                ctx_.calib_accum_ibus = 0;
#endif
                ctx_.silent_calib_active = true;
                ctx_.silent_calib_timer_ticks = 0;
            }
        }
    }

    else
    {
        if (ctx_.silent_calib_active)
        {
            ctx_.calib_counter = 0U;
            ctx_.calib_accum_ia = 0;
            ctx_.calib_accum_ib = 0;
            ctx_.calib_accum_ic = 0;
#if MOTOR_BUILD_HAS_BUS_CURRENT
            ctx_.calib_accum_ibus = 0;
#endif
        }
        ctx_.silent_calib_timer_ticks = 0;
        ctx_.silent_calib_active = false;
    }

    /* ================================================================
     * [4续] 模式分支: 根据 mode_ 执行对应控制策略
     *        mode_ 由 RUN 时自动同步 self->target_mode_
     * ================================================================ */
    if (state_ == State::RUN)
    {
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
        const bool is_rl_identify = (target_mode_ == Mode::CALIB_RL_IDENTIFY);
#else
        constexpr bool is_rl_identify = false;
#endif

        /* API 写入 target_mode_ 前已完成完整校验，快环只应用已验证的模式。 */
        mode_ = target_mode_;  // 强制执行: 由顶层命令驱动模式切换

#if LIB_MOTOR_ENABLE_DEBUG_ANY
        bool is_debug_direct =
            (run_phase_ == RunPhase::RUN_DIRECT) &&
            (
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
             mode_ == Mode::DEBUG_PWM_MANUAL ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
             mode_ == Mode::DEBUG_CURRENT_LOCK ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_HFI_OBSERVER
             mode_ == Mode::DEBUG_HFI_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
             mode_ == Mode::DEBUG_IF_DRAG ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
             mode_ == Mode::DEBUG_IF_SMO_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
             mode_ == Mode::DEBUG_IF_HFI_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
             mode_ == Mode::DEBUG_VF_DRAG);
#else
             false);
#endif
#else
        bool is_debug_direct = false;
#endif

        /* ================================================================
         * 观测器环路: 传感器数据融合 + 速度计算
         *   调试直驱模式跳过观测器, 直接使用 setTargetSpeed 等设置值
         *   观测器更新 angle_elec/speed_rpm 给后续环节
         * ================================================================ */
        if (!is_debug_direct && !is_rl_identify)
        {
            runObserverLoop();
        }

        /* ================================================================
         * 控制策略选择: 由 RunPhase 决定
         *
         *   RUN_DIRECT             -> 调试直驱 (mode_ 决定具体控制子程序)
         *   ALIGNMENT              -> runCurrentLock()  (初始对齐)
         *   FORCE_DRAG             -> runIFControl()    (IF 强拖)
         *   SMO_ONLY / SENSOR / 其它 -> runControlLoop()  (FOC 闭环)
         * ================================================================ */
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
        if (is_rl_identify)
        {
            runCurrentLock();
        }
        else if (run_phase_ == RunPhase::RUN_DIRECT)
#else
        if (run_phase_ == RunPhase::RUN_DIRECT)
#endif
        {
            switch (mode_)
            {
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
                case Mode::DEBUG_PWM_MANUAL:    runPWMManual();   break;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
                case Mode::DEBUG_CURRENT_LOCK:  runCurrentLock(); break;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_HFI_OBSERVER
                case Mode::DEBUG_HFI_OBSERVER:  runHFIObserverTest(); break;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
            case Mode::DEBUG_IF_DRAG:            runIFControl(); break;  //20us
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
                case Mode::DEBUG_IF_SMO_OBSERVER: runIFControl(); break;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
                case Mode::DEBUG_IF_HFI_OBSERVER: runIFControl(); break;
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
                case Mode::DEBUG_VF_DRAG:       runVFControl();   break;
#endif
                default:                      runControlLoop();   break;
            }
        }
        else
        {
            switch (run_phase_)
            {
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY || LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
                case RunPhase::ALIGNMENT:   runCurrentLock();   break;  // 初始对齐
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
                case RunPhase::FORCE_DRAG:  runIFControl();     break;  // IF 强拖
#endif
                default:                    runControlLoop();   break;  // FOC 闭环
            }
        }
    }
    /* ================================================================
     * [5] Duty 限幅: 三相占空比 clamp 至 [0, max_duty_cycle]
     *
     * 安全: 硬件 PWM 死区时间需要占空比不超过上限, 另外留有余量防止过调制。
     * 例如 max_duty_cycle = 0.95 (5% 余量)
     * ================================================================ */
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.duty_a = runtimeClampDuty(ctx_.duty_a, ctx_.max_duty);
    ctx_.duty_b = runtimeClampDuty(ctx_.duty_b, ctx_.max_duty);
    ctx_.duty_c = runtimeClampDuty(ctx_.duty_c, ctx_.max_duty);
#else
    float scale = config_.limit.max_duty_cycle;
    ctx_.duty_a = clampFloatFast(ctx_.duty_a, 0.0f, scale);
    ctx_.duty_b = clampFloatFast(ctx_.duty_b, 0.0f, scale);
    ctx_.duty_c = clampFloatFast(ctx_.duty_c, 0.0f, scale);
#endif

    /* ================================================================
     * [6] PWM 输出: 仅在 RUN 状态输出占空比
     *
     * 非RUN 状态 PWM 由 FSM 控制:
     *   STOP/COAST:     MOE=0 (Hi-Z)   -> FSM::handleRun -> pwm_coast()
     *   STOP/BRAKE_LOW: MOE=1, CCR=0   -> FSM::handleRun -> pwm_brake_lowside()
     *   ERROR:          MOE=0          -> FSM::step()    -> pwm_disable()
     * ================================================================ */
    if (state_ == State::RUN)
    {
        // setPa5ScopeProbe();
        writePwmOutputs();  //3.28us
        // resetPa5ScopeProbe();
    }

    /* [P2] Ke 辨识运行期预留:
     * 当前 SMO getEstimatedKe() 保持 0，不会写入有效 identified。
     * 后续 Identify 层接入有效 Ke 估计后，再复用此完成条件。
     */
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    if (ke_identify_ticks_remaining_ > 0U)
    {
        --ke_identify_ticks_remaining_;

        const bool valid = (ctx_.smo_estimate.valid &&
                            static_cast<uint32_t>(ctx_.smo_estimate.valid_ticks) >=
                                2UL * static_cast<uint32_t>(
                                    config_.observer.smo_validity.acquire_ticks));
        const float speed_abs = fabsf(runtimeSpeedToPhysical(ctx_, ctx_.speed_rpm));
        const float threshold = config_.observer.hfi_to_smo_rpm + config_.observer.switch_hysteresis_rpm;
        if (valid && speed_abs > threshold)
        {
            const float ke = smo_.getEstimatedKe();
            if (ke > 0.0f)
            {
                config_.physical.ke_v_per_rad_s_identified = ke;
                event_.ke_identify_done = 1U;
                ke_identify_ticks_remaining_ = 0U;   // 一旦写入即完成
            }
        }
        else if (ke_identify_ticks_remaining_ == 0U)
        {
            // 超时仍未辨识成功, event 不置位, 上层轮询会发现 false 并自行停 API
        }
    }
#endif
}

#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
Result MotorManager::startKEstimationInternal()
{
    if (state_ != State::RUN) return Result::InvalidState;
    if (run_phase_ != RunPhase::SMO_ONLY) return Result::InvalidState;
    if (ke_identify_ticks_remaining_ > 0U) return Result::InvalidState;   // 已在进行中
    // 4000 tick = 200 ms @20kHz 采样窗口
    ke_identify_ticks_remaining_ = 4000U;
    event_.ke_identify_done = 0U;
    return Result::Ok;
}
#endif

#if LIB_MOTOR_ENABLE_FLYING_START
Result MotorManager::triggerFlyingStartIfSpinningInternal()
{
    if (state_ != State::STOP) return Result::InvalidState;
    if (config_.observer.flying_start_mode != FlyingStartMode::PHASE_VOLTAGE)
        return Result::InvalidParam;
    /* E 组完整实现顺逆风启动 FSM; 此处仅触发 */
    requestStart();   // 实际 start() 中 handleAlignment 前会自动检测 ω
    return Result::Ok;
}
#endif

} // namespace Lib_Motor
