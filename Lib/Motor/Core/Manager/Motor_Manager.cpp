#include "Motor_Manager.h"

#include "../FSM/Motor_FSM.h"
#include "../../Common/Math/FocMath.h"

#include <cmath>

namespace Lib_Motor
{

static inline float clampFloatFast(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void MotorManager::prepareSingleShuntSampling()
{
    if (config_.sensor.current_sense_mode != CurrentSenseMode::SINGLE_SHUNT)
    {
        return;
    }

    MotorSingleShuntSamplePlan plan;
    if (!MotorSingleShunt::buildSamplePlan(ctx_.duty_a,
                                           ctx_.duty_b,
                                           ctx_.duty_c,
                                           config_.limit.max_duty_cycle,
                                           config_.sensor.single_shunt_min_sample_window_s,
                                           config_.control.control_freq_hz,
                                           plan))
    {
        ctx_.single_shunt_sample_plan = MotorSingleShuntSamplePlan{};
        if (config_.hal != nullptr &&
            config_.hal->apply_current_sample_schedule != nullptr)
        {
            ctx_.current_sample_schedule =
                MotorSingleShunt::toCurrentSampleSchedule(ctx_.single_shunt_sample_plan);
            config_.hal->apply_current_sample_schedule(&ctx_.current_sample_schedule);
        }
        ctx_.duty_a = 0.0f;
        ctx_.duty_b = 0.0f;
        ctx_.duty_c = 0.0f;
        setFault(Fault::PARAM_ERROR);
        return;
    }

    ctx_.single_shunt_sample_plan = plan;
    if (config_.hal != nullptr &&
        config_.hal->apply_current_sample_schedule != nullptr)
    {
        ctx_.current_sample_schedule =
            MotorSingleShunt::toCurrentSampleSchedule(ctx_.single_shunt_sample_plan);
        config_.hal->apply_current_sample_schedule(&ctx_.current_sample_schedule);
    }
}

void MotorManager::disableSingleShuntSamplingSchedule()
{
    if (config_.sensor.current_sense_mode != CurrentSenseMode::SINGLE_SHUNT)
    {
        return;
    }

    if (ctx_.single_shunt_sample_plan.valid == 0U &&
        ctx_.current_sample_schedule.enabled == 0U)
    {
        return;
    }

    ctx_.single_shunt_sample_plan = MotorSingleShuntSamplePlan{};
    ctx_.current_sample_schedule = MotorCurrentSampleSchedule{};
    if (config_.hal != nullptr &&
        config_.hal->apply_current_sample_schedule != nullptr)
    {
        config_.hal->apply_current_sample_schedule(&ctx_.current_sample_schedule);
    }
}

MotorManager::MotorManager(const MotorConfig& config)
    : config_(config)
    , active_policy_(config.default_run_policy)
    , pending_policy_(config.default_run_policy)
    , ctx_()
    , fast_ctx_()
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
    safety_state_ = SafetyState::CLEAR;
    command_state_ = CommandState::IDLE;
    stream_state_ = StreamState::NONE;
    resetSetpointPlayback();
    angle_state_ = AngleState::NONE;
    stop_state_ = StopState::IDLE;
    energy_state_ = EnergyState::IDLE;
    event_       = MotorEvent();
    active_policy_ = config_.default_run_policy;
    pending_policy_ = config_.default_run_policy;
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
    fast_ctx_ = FastRuntimeCtx{};
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    ctx_.debug_ramp_time_s = 2.0f; // 默认斜坡时间
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

    /* ================================================================
     * [F] 观测器初始化 (SMO/传感器观测器/冗余传感器/输出传感器)
     * ================================================================ */
#if LIB_MOTOR_ENABLE_SMO
    /* [P2] SMO 用 effective R/L (来源由 physical.electrical_param_source 决定) */
    smo_.init(config_.physical.rs_effective(), config_.physical.ls_effective(),
              config_.observer.smo_gain,
              config_.observer.smo_pll_kp, config_.observer.smo_pll_ki);
    smo_.setValidityCriteria(
        // [P4] 启动期 IF→SMO 单向加速切换阈值 (旧 switch_speed_rpm)
        config_.observer.hfi_to_smo_startup_rpm * TWO_PI * config_.physical.pole_pairs / 60.0f,
        config_.observer.max_handover_error_rad,
        config_.observer.convergence_ticks,
        config_.observer.smo_min_signal_level);
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
    /* ================================================================
     * [G] IF 强拖角度发生器初始化
     * ================================================================ */
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    if_angle_gen_.init(config_.control.control_freq_hz, config_.physical.pole_pairs);
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    drag_current_override_ = 0.0f;
#endif

    /* ★ 安全: 初始化完成后强制关闭 PWM 输出 */
    if (config_.hal && config_.hal->pwm_disable) config_.hal->pwm_disable();
    if (config_.hal && config_.hal->set_duty) config_.hal->set_duty(0.0f, 0.0f, 0.0f);
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
    safety_state_ = (fault == Fault::NONE) ? SafetyState::CLEAR : SafetyState::FAULT_LATCHED;
    command_state_ = (fault == Fault::NONE) ? CommandState::IDLE : CommandState::FAULTED;
    stream_state_ = StreamState::NONE;
    angle_state_ = AngleState::NONE;
    stop_state_ = StopState::IDLE;
    energy_state_ = EnergyState::IDLE;
    ctx_.duty_a = 0.0f;
    ctx_.duty_b = 0.0f;
    ctx_.duty_c = 0.0f;
    ctx_.single_shunt_sample_plan = MotorSingleShuntSamplePlan{};
    ctx_.current_sample_schedule = MotorCurrentSampleSchedule{};
    if (config_.hal != nullptr && config_.hal->pwm_disable != nullptr)
    {
        config_.hal->pwm_disable();
    }
    if (config_.hal != nullptr && config_.hal->set_duty != nullptr)
    {
        config_.hal->set_duty(0.0f, 0.0f, 0.0f);
    }
}

void MotorManager::setMode(Mode mode)
{
    Motor_FSM::selectMode(*this, mode);
}

void MotorManager::requestStart()
{
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
    Motor_FSM::requestStop(*this, mode);
}

void MotorManager::requestEmergencyStop()
{
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
 *   │ [1] updateSensorMeasurements()                                   │
 *   │     ADC 采样 -> 物理量(电流/电压/温度) + 低通滤波                  │
 *   │                                                                  │
 *   │ [2] runSafetyCheck()                                             │
 *   │     电流/电压/温度/传感器检查 -> 合并到 fault_ 寄存器              │
 *   │                                                                  │
 *   │ [3] Motor_FSM::step(*this)                                       │
 *   │     故障快速关断 (fault!=NONE -> PWM关闭 -> ERROR)                │
 *   │     状态机转换: INIT->ADC_CAL->STOP / STOP->RUN / RUN->ERROR      │
 *   │                                                                  │
 *   │ [4] 控制策略分支 (由 state_ / mode_ 决定)                         │
 *   │     ADC_CAL  -> runAdcCalibration()                               │
 *   │     RUN + DEBUG_PWM   -> runPWMManual()                          │
 *   │     RUN + CURRENT_LOCK -> runCurrentLock()                        │
 *   │     RUN + IF_DRAG     -> runIFControl()                          │
 *   │     RUN + VF_DRAG     -> runVFControl()                          │
 *   │     RUN + CALIB_RL    -> runCurrentLock() (拖拽辨识)               │
 *   │     RUN + 其它        -> runControlLoop() (FOC 闭环)              │
 *   │                                                                  │
 *   │ [5] Duty 限幅: clamp(duty, 0, max_duty_cycle)                    │
 *   │                                                                  │
 *   │ [6] PWM 输出: 仅在 RUN 状态调用 set_duty()                        │
 *   │     非RUN: MOE 安全关断 FSM 控制 (pwm_coast/pwm_disable)         │
 *   └──────────────────────────────────────────────────────────────────┘
 */
void MotorManager::tick()
{
    ctx_.fsm_timer_ticks++;  // 每个 tick 递增, 由 FSM 用于超时判断 (如 ALIGNMENT 持续时间)

    /* [1] 相电流采样 + 物理量换算 + 低通滤波 */
    if (state_ == State::ISR_DIAGNOSTIC)
    {
        return;
    }

    updateSensorMeasurements();
    updatePositionSensorMeasurement();

    /*
     * Keep telemetry fresh after a latched fault. ERROR must not run safety,
     * control, or PWM output, but ADC-derived monitor values should continue
     * to update for bring-up and fault diagnosis.
     */
    if (state_ == State::ERROR)
    {
        Motor_FSM::step(*this);
        return;
    }

    /* [2] 安全检查 (合并到 fault_ 寄存器) */
    runSafetyCheck();

    /* 堵转自动重启: 检查是否处于 STOP 并需要重新触发 run_requested_ */
    processPendingStallRestart();

    /* [3] 状态机编排 -- 故障锁存 + 状态转换 */
    Motor_FSM::step(*this);

    /* 预加载回放必须在控制环读取目标前服务, 当前阶段为空框架。 */
    serviceSetpointPlayback();

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
    if (state_ == State::STOP)
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
                ctx_.calib_accum_ia = 0.0f;
                ctx_.calib_accum_ib = 0.0f;
                ctx_.calib_accum_ic = 0.0f;
                ctx_.calib_accum_single_shunt_first = 0.0f;
                ctx_.calib_accum_single_shunt_second = 0.0f;
#if MOTOR_BUILD_HAS_BUS_CURRENT
                ctx_.calib_accum_ibus = 0.0f;
#endif
                ctx_.silent_calib_active = true;
                ctx_.silent_calib_timer_ticks = 0;
            }
        }
    }
    else
    {
        ctx_.silent_calib_timer_ticks = 0;
        ctx_.silent_calib_active = false;
    }

    /* ================================================================
     * [4续] 模式分支: 根据 mode_ 执行对应控制策略
     *        mode_ 由 RUN 时自动同步 self->target_mode_
     * ================================================================ */
    if (state_ == State::RUN)
    {
        const bool is_rl_identify = (target_mode_ == Mode::CALIB_RL_IDENTIFY);

        if (mode_ != target_mode_ && validateModeSelection(target_mode_) != Result::Ok)
        {
            stopForFault(Fault::PARAM_ERROR);
            return;
        }
        mode_ = target_mode_;  // 强制执行: 由顶层命令驱动模式切换

#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
        bool is_debug_direct =
            (run_phase_ == RunPhase::RUN_DIRECT) &&
            (mode_ == Mode::DEBUG_PWM_MANUAL ||
             mode_ == Mode::DEBUG_CURRENT_LOCK ||
#if LIB_MOTOR_ENABLE_HFI
             mode_ == Mode::DEBUG_HFI_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
             mode_ == Mode::DEBUG_IF_DRAG ||
#if LIB_MOTOR_ENABLE_SMO
             mode_ == Mode::DEBUG_IF_SMO_OBSERVER ||
#endif
#if LIB_MOTOR_ENABLE_HFI
             mode_ == Mode::DEBUG_IF_HFI_OBSERVER ||
#endif
#endif
             mode_ == Mode::DEBUG_VF_DRAG);
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
        if (is_rl_identify)
        {
            runCurrentLock();
        }
        else if (run_phase_ == RunPhase::RUN_DIRECT)
        {
            switch (mode_)
            {
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
                case Mode::DEBUG_PWM_MANUAL:    runPWMManual();   break;
                case Mode::DEBUG_CURRENT_LOCK:  runCurrentLock(); break;
#if LIB_MOTOR_ENABLE_HFI
                case Mode::DEBUG_HFI_OBSERVER:  runHFIObserverTest(); break;
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
                case Mode::DEBUG_IF_DRAG:       runIFControl();   break;
#if LIB_MOTOR_ENABLE_SMO
                case Mode::DEBUG_IF_SMO_OBSERVER: runIFControl(); break;
#endif
#if LIB_MOTOR_ENABLE_HFI
                case Mode::DEBUG_IF_HFI_OBSERVER: runIFControl(); break;
#endif
#endif
                case Mode::DEBUG_VF_DRAG:       runVFControl();   break;
#endif
                default:                        runControlLoop(); break;
            }
        }
        else
        {
            switch (run_phase_)
            {
#if LIB_MOTOR_ENABLE_IF_STARTUP
                case RunPhase::ALIGNMENT:   runCurrentLock();   break;  // 初始对齐
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
    float scale = config_.limit.max_duty_cycle;
    ctx_.duty_a = clampFloatFast(ctx_.duty_a, 0.0f, scale);
    ctx_.duty_b = clampFloatFast(ctx_.duty_b, 0.0f, scale);
    ctx_.duty_c = clampFloatFast(ctx_.duty_c, 0.0f, scale);

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
        prepareSingleShuntSampling();
        config_.hal->set_duty(ctx_.duty_a, ctx_.duty_b, ctx_.duty_c);
    }
    else
    {
        disableSingleShuntSamplingSchedule();
    }

    /* [P2] Ke 辨识运行期检查: 若 ke_identify_ticks_remaining_>0, 持续递减,
     * 每过 tick 检查 SMO 收敛 + 高速稳态, 满足后写 identified + 置 event。
     * 不满足条件时只递减 ticks, 不强制写入 (避免误估)。
     */
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    if (ke_identify_ticks_remaining_ > 0U)
    {
        --ke_identify_ticks_remaining_;

        const bool valid = (ctx_.smo_estimate.valid &&
                            static_cast<uint32_t>(ctx_.smo_estimate.valid_ticks) >=
                                2UL * static_cast<uint32_t>(config_.observer.convergence_ticks));
        const float speed_abs = fabsf(ctx_.speed_rpm);
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
