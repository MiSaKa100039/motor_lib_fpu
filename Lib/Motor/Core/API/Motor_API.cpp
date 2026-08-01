/*
 * Motor_API.cpp — 电机对外 API 层实现
 *
 * 职责: 为上层应用提供电机控制的统一接口, 封装参数校验、权限检查和
 *       对 MotorManager 的委托调用。
 *
 * API 分层设计:
 *   [上层应用] → MotorAPI (本文件) → MotorManager → Motor_FSM/Control
 *
 * 安全设计原则:
 *   - 所有 public API 先做空指针和参数校验, 再委托 MotorManager
 *   - 模式切换 (selectMode/debugXxx) 经过 checkSafeModeChange 安全把关
 *   - 调试 API 受 debug_locked_ 和 ENABLE_DANGEROUS_TEST_API 双重保护
 */

#include "Motor_API.h"

#include "../Feature/Motor_FeatureRegistry.h"
#include "../Manager/Motor_Manager.h"
#include "../Debug/Motor_OzoneTuning.h"
#include "../../Safety/Basic/Motor_CommandGuard.h"
#include "../../Safety/Basic/Motor_ConfigCheck.h"
#include "../FSM/Motor_FSM.h"
#include "BuildCfg/Platform_MotorBuildConfig.h"

#include <cmath>
#include <new>

namespace Lib_Motor
{

/* File-local storage and init helpers. These symbols must not be exported
 * because MotorAPI owns the static manager pool and init-time diagnostics.
 */
namespace
{

alignas(MotorManager) static unsigned char g_motor_storage_pool[MOTOR_MAX_INSTANCES][sizeof(MotorManager)];
static bool g_motor_slot_claimed[MOTOR_MAX_INSTANCES] = { false };
static uint32_t g_isr_diagnostic_tick_countdown[MOTOR_MAX_INSTANCES] = { 0U };

#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
constexpr float kIsrDiagnosticTickHz = 1000.0f;
#endif

void diagnosticNoop()
{
}

void diagnosticSetDuty(float, float, float)
{
}

void diagnosticReadCurrents(uint16_t* raw_u,
                            uint16_t* raw_v,
                            uint16_t* raw_w,
                            uint16_t* raw_bus)
{
    if (raw_u != nullptr) *raw_u = 0U;
    if (raw_v != nullptr) *raw_v = 0U;
    if (raw_w != nullptr) *raw_w = 0U;
    if (raw_bus != nullptr) *raw_bus = 0U;
}

void diagnosticReadPhaseVoltages(uint16_t* raw_u,
                                 uint16_t* raw_v,
                                 uint16_t* raw_w)
{
    if (raw_u != nullptr) *raw_u = 0U;
    if (raw_v != nullptr) *raw_v = 0U;
    if (raw_w != nullptr) *raw_w = 0U;
}

uint16_t diagnosticReadVbus()
{
    return 0U;
}

uint16_t diagnosticReadTemp(uint8_t)
{
    return 0U;
}

bool diagnosticReadSingleShuntPair(uint16_t* raw_first, uint16_t* raw_second)
{
    if (raw_first != nullptr) *raw_first = 0U;
    if (raw_second != nullptr) *raw_second = 0U;
    return false;
}

void diagnosticApplyCurrentSampleSchedule(const MotorCurrentSampleSchedule*)
{
}

CurrentSenseMode buildCurrentSenseMode()
{
#if MOTOR_BUILD_CURRENT_SENSE_MODE == 1
    return CurrentSenseMode::SINGLE_SHUNT;
#elif MOTOR_BUILD_CURRENT_SENSE_MODE == 3
    return CurrentSenseMode::TRIPLE_SENSOR;
#else
    return CurrentSenseMode::DUAL_SENSOR;
#endif
}

uint8_t diagnosticPhaseCurrentMask()
{
#if MOTOR_BUILD_CURRENT_SENSE_MODE == 1
    return 0U;
#elif MOTOR_BUILD_CURRENT_SENSE_MODE == 3
    return MOTOR_PHASE_CURRENT_ALL_VALID;
#else
    return static_cast<uint8_t>(MOTOR_PHASE_CURRENT_U_VALID |
                                MOTOR_PHASE_CURRENT_V_VALID);
#endif
}

const MotorHAL_t* diagnosticHal()
{
    static const MotorHAL_t hal{
        diagnosticSetDuty,
        diagnosticNoop,
        diagnosticNoop,
        diagnosticNoop,
        diagnosticNoop,
        nullptr,
        nullptr,
        diagnosticNoop,
        diagnosticNoop,
        diagnosticReadCurrents,
        diagnosticPhaseCurrentMask(),
        diagnosticReadPhaseVoltages,
        diagnosticReadVbus,
        diagnosticReadTemp,
        diagnosticNoop,
        diagnosticNoop,
        nullptr,
        nullptr,
        nullptr,
        0U,
        diagnosticReadSingleShuntPair,
        diagnosticApplyCurrentSampleSchedule};
    return &hal;
}

MotorConfig makeTickSafeDiagnosticConfig(const MotorConfig& cfg)
{
    MotorConfig safe{};
    safe.hal = diagnosticHal();
    safe.physical.pole_pairs = 7U;
    safe.control.control_freq_hz = 10000.0f;
    safe.sensor.adc_calibration_samples = 1000U;
    safe.sensor.adc_v_ref = 3.3f;
    safe.sensor.adc_resolution = 4096.0f;
    safe.sensor.phase_shunt_resistor = 0.001f;
    safe.sensor.phase_amp_gain = 50.0f;
    safe.sensor.bus_shunt_resistor = 0.001f;
    safe.sensor.bus_amp_gain = 20.0f;
    safe.sensor.vbus_r_down = 6200.0f;
    safe.sensor.vphase_r_down = 6200.0f;
    safe.limit.max_duty_cycle = 0.95f;
    safe.limit.over_voltage_v = 30.0f;
    safe.control.startup_target_mode = cfg.control.startup_target_mode;
    safe.default_run_policy = cfg.default_run_policy;

    /*
     * Base config errors may include null HAL or invalid denominators. Keep only
     * fields that are useful to inspect and force the rest to a tick-safe shape.
     */
    if (cfg.physical.pole_pairs != 0U)
    {
        safe.physical.pole_pairs = cfg.physical.pole_pairs;
    }
    if (std::isfinite(cfg.control.control_freq_hz) &&
        cfg.control.control_freq_hz > 0.0f)
    {
        safe.control.control_freq_hz = cfg.control.control_freq_hz;
    }
    if (cfg.sensor.adc_calibration_samples != 0U)
    {
        safe.sensor.adc_calibration_samples = cfg.sensor.adc_calibration_samples;
    }

    safe.sensor.current_sense_mode = buildCurrentSenseMode();
    safe.sensor.current_sensor_type = CurrentSensorType::SHUNT_RESISTOR;
    safe.sensor.has_bus_voltage = false;
    safe.sensor.has_bus_current = false;
    safe.sensor.has_phase_voltage = false;
    safe.control.modulation = ModulationMethod::SVPWM;
    return safe;
}

/* Return the placement-new address for a claimed MotorManager slot. */
static MotorManager* managerStorageAt(int slot)
{
    return reinterpret_cast<MotorManager*>(&g_motor_storage_pool[slot][0]);
}

/* Map init-time config/feature faults to the public Result surface. */
Result faultToInitResult(Fault fault)
{
    switch (fault)
    {
        case Fault::NONE:
            return Result::Ok;

        case Fault::OBSERVER_UNAVAILABLE:
            return Result::NotSupported;

        default:
            return Result::InvalidParam;
    }
}

/* Normal closed-loop startup modes require feedback validation during init. */
bool startupModeRequiresFeedback(Mode mode)
{
    switch (mode)
    {
        case Mode::TORQUE_CONTROL:
        case Mode::VELOCITY_CONTROL:
        case Mode::POSITION_CONTROL:
            return true;

        default:
            return false;
    }
}

/* Debug/calib runs are exclusive; mode/target-changing APIs must wait for stop. */
bool isDebugOrCalibApiLocked(const MotorManager* mgr)
{
    if (mgr == nullptr)
    {
        return false;
    }

    return MotorCommandGuard::isDebugOrCalib(mgr->mode()) ||
           MotorCommandGuard::isDebugOrCalib(mgr->targetMode());
}

/* Allow the selected bench-test API to finish configuring a preloaded target
 * while the manager is still idle. This keeps active debug/calib runs exclusive.
 */
bool isSameIdleDebugOrCalibRequest(const MotorManager* mgr, Mode requested_mode)
{
    if (mgr == nullptr || !MotorCommandGuard::isDebugOrCalib(requested_mode))
    {
        return false;
    }

    return mgr->state() == State::STOP &&
           mgr->mode() == Mode::NONE &&
           mgr->targetMode() == requested_mode;
}

bool isDebugOrCalibApiLockedForRequest(const MotorManager* mgr, Mode requested_mode)
{
    if (!isDebugOrCalibApiLocked(mgr))
    {
        return false;
    }

    return !isSameIdleDebugOrCalibRequest(mgr, requested_mode);
}

#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
uint32_t isrDiagnosticTickDivider(const MotorManager& mgr)
{
    const float control_freq_hz = mgr.config().control.control_freq_hz;
    if (!std::isfinite(control_freq_hz) ||
        control_freq_hz <= kIsrDiagnosticTickHz)
    {
        return 1U;
    }

    const float divider = control_freq_hz / kIsrDiagnosticTickHz;
    if (divider >= 65535.0f)
    {
        return 65535U;
    }

    const uint32_t rounded_divider = static_cast<uint32_t>(divider + 0.5f);
    return (rounded_divider == 0U) ? 1U : rounded_divider;
}

bool acceptIsrDiagnosticPulse(int motor_id, const MotorManager& mgr)
{
    /*
     * ISR 负载诊断只验证快中断是否饿住 main loop，首个 tick 立即放行，
     * 之后按 control_freq_hz/kIsrDiagnosticTickHz 分频。
     */
    if (g_isr_diagnostic_tick_countdown[motor_id] == 0U)
    {
        const uint32_t divider = isrDiagnosticTickDivider(mgr);
        g_isr_diagnostic_tick_countdown[motor_id] =
            (divider > 1U) ? (divider - 1U) : 0U;
        return true;
    }

    --g_isr_diagnostic_tick_countdown[motor_id];
    return false;
}
#endif

} // namespace

// 全局电机实例池, 最多 MOTOR_MAX_INSTANCES 个实例
MotorManager* g_motor_pool[MOTOR_MAX_INSTANCES] = { nullptr };

/*
 * MotorAPI 构造函数 — 预占一个静态实例槽位
 *
 * 设计:
 *   - g_motor_slot_claimed[] 负责句柄槽位归属
 *   - g_motor_pool[] 只表示该槽位上是否已构造出 MotorManager 实例
 * 这样可以避免“两个 MotorAPI 都在 init 之前看到 nullptr, 最终抢到同一个 id”。
 */
MotorAPI::MotorAPI()
{
    for (int i = 0; i < MOTOR_MAX_INSTANCES; i++)
    {
        if (!g_motor_slot_claimed[i])
        {
            g_motor_slot_claimed[i] = true;
            id_ = i;
            break;
        }
    }
}

MotorAPI::~MotorAPI()
{
    if (id_ < 0) return;

    if (g_motor_pool[id_] != nullptr)
    {
        deInit();
    }

    g_motor_slot_claimed[id_] = false;
    id_ = -1;
}

/*
 * init — 电机初始化
 *
 * 调用链:
 *   Platform_Motor_Init() → MotorAPI::init(cfg) → placement new MotorManager(cfg) → Manager::init()
 *
 * 此函数完成:
 *   1. 校验配置参数合法性 (validateBase)
 *   2. 创建 MotorManager 实例 (构造函数内拷贝 config, 计算派生值如 dt_、校准tick数)
 *   3. 调用 MotorManager::init() 推进 FSM 到 INIT 状态
 *
 * 注意: init() 后电机处于 INIT 态, 第一个 tick() 会自动推进到 ADC_CAL → STOP
 */
Result MotorAPI::init(const MotorConfig& cfg)
{
    if (id_ < 0) return Result::InvalidHandle;
    if (g_motor_pool[id_] != nullptr) return Result::InvalidState;

    MotorConfigFaultDetail init_detail = MotorConfigFaultDetail::NONE;
    Fault init_fault = MotorConfigCheck::validateBase(cfg, &init_detail);
    const bool base_faulted = (init_fault != Fault::NONE);

    if (init_fault == Fault::NONE)
    {
        init_fault = MotorConfigCheck::validateStartupTargetMode(cfg, &init_detail);
        if (init_fault == Fault::NONE &&
            startupModeRequiresFeedback(cfg.control.startup_target_mode))
        {
            init_fault = MotorConfigCheck::validateFeedback(cfg, &init_detail);
        }
    }

    const MotorConfig manager_config =
        base_faulted ? makeTickSafeDiagnosticConfig(cfg) : cfg;
    MotorManager* const mgr = new (managerStorageAt(id_)) MotorManager(manager_config);
    g_motor_pool[id_] = mgr;
#if LIB_MOTOR_ENABLE_OZONE_TUNING
    Motor_OzoneTuning_LoadFromConfig(static_cast<uint8_t>(id_), manager_config);
#endif
    mgr->init();
    mgr->setConfigFaultDetail(init_detail);

#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
    mgr->enterIsrDiagnostic(init_fault);
    last_fault_ = init_fault;
    last_config_fault_detail_ = init_detail;
    return faultToInitResult(init_fault);
#endif

    if (init_fault != Fault::NONE)
    {
        Motor_FSM::latchFault(*mgr, init_fault);
        last_fault_ = init_fault;
        last_config_fault_detail_ = init_detail;
        return faultToInitResult(init_fault);
    }

    last_fault_ = Fault::NONE;
    last_config_fault_detail_ = MotorConfigFaultDetail::NONE;
    return Result::Ok;
}

/*
 * deInit — 反初始化, 释放资源
 */
Result MotorAPI::deInit()
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    g_motor_pool[id_]->~MotorManager();
    g_motor_pool[id_] = nullptr;
    g_isr_diagnostic_tick_countdown[id_] = 0U;
    last_fault_ = Fault::NONE;
    last_config_fault_detail_ = MotorConfigFaultDetail::NONE;
    return Result::Ok;
}

/*
 * start — 启动电机
 *
 * 前置条件:
 *   - 电机状态必须为 STOP (空闲) 或 RUN (运行中)
 *   - 启动模式由 targetMode() 或 startupTargetMode() 决定
 *
 * 支持的启动模式:
 *   - TORQUE_CONTROL:  转矩控制 (电流环)
 *   - VELOCITY_CONTROL: 速度控制 (速度环 + 电流环)
 *
 * 启动前会验证所选模式的反馈源是否合法 (validateModeSelection)
 */
Result MotorAPI::start()
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    MotorManager* mgr = g_motor_pool[id_];
    State state = mgr->state();
    if (state != State::STOP && state != State::RUN)
    {
        return Result::InvalidState;
    }

    /* Debug/Calib 独占运行时拒绝 start, 防止目标模式被外部启停冲撞 */
    if (isDebugOrCalibApiLocked(mgr))
    {
        return Result::InvalidState;
    }

    Mode startup_mode = mgr->targetMode();
    if (startup_mode == Mode::NONE)
    {
        startup_mode = mgr->startupTargetMode();
    }

    switch (startup_mode)
    {
        case Mode::TORQUE_CONTROL:
        case Mode::VELOCITY_CONTROL:
        case Mode::POSITION_CONTROL:
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
        case Mode::IMPEDANCE_CONTROL:
#endif
        {
            Result capability = mgr->validateModeSelection(startup_mode);
            if (capability != Result::Ok) return capability;

            mgr->setMode(startup_mode);
            mgr->requestStart();
            return Result::Ok;
        }

        case Mode::NONE:
            return Result::InvalidState;

        default:
            return Result::NotSupported;
    }
}

/*
 * stop — 停止电机
 *
 * @param mode  制动模式 (COAST: 惯性滑行 / ELECTRICAL_BRAKE: 下管短路制动 / MECHANICAL_BRAKE: 机械抱闸)
 *
 * 入参合法性校验:
 *   - COAST            永远允许
 *   - ELECTRICAL_BRAKE MotorCfg.brake.allow_low_side_brake 且 HAL->pwm_brake_lowside 非空
 *   - MECHANICAL_BRAKE MotorCfg.brake.allow_mechanical_brake 且 HAL->mechanical_brake_engage 非空
 *   不满足上述条件返回 NotSupported, 拒绝静默降级到 COAST
 */
Result MotorAPI::stop(StopMode mode)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    const MotorManager& m = *g_motor_pool[id_];
    const MotorConfig& cfg = m.config();

    switch (mode)
    {
        case StopMode::COAST:
            break;                                                 // 永远允许

        case StopMode::ELECTRICAL_BRAKE:
            if (!cfg.brake.allow_low_side_brake ||
                cfg.hal == nullptr ||
                cfg.hal->pwm_brake_lowside == nullptr)
            {
                return Result::NotSupported;
            }
            break;

        case StopMode::MECHANICAL_BRAKE:
            if (!cfg.brake.allow_mechanical_brake ||
                cfg.hal == nullptr ||
                cfg.hal->mechanical_brake_engage == nullptr)
            {
                return Result::NotSupported;
            }
            break;

        default:
            return Result::InvalidParam;
    }

    g_motor_pool[id_]->requestStop(mode);
    return Result::Ok;
}

/*
 * emergencyStop — 紧急停止
 *
 * 与 stop() 的区别:
 *   - emergencyStop 直接 latchFault(EMERGENCY_STOP), 立即关断 PWM
 *   - stop() 是正常制动流程 (STOPPING 状态), 可控减速
 */
Result MotorAPI::emergencyStop()
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
    if (g_motor_pool[id_]->state() == State::ISR_DIAGNOSTIC) return Result::InvalidState;
#endif
    g_motor_pool[id_]->requestEmergencyStop();
    return Result::Ok;
}

/*
 * clearFault — 清除故障锁存
 *
 * 调用后:
 *   - fault_ 清零, SafetyState 恢复 CLEAR
 *   - 所有子 FSM 状态复位
 *   - 电机回到 STOP 态, 可重新启动
 */
Result MotorAPI::clearFault()
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
    if (g_motor_pool[id_]->state() == State::ISR_DIAGNOSTIC) return Result::InvalidState;
#endif
    g_motor_pool[id_]->clearFaultAndStop();
    return Result::Ok;
}

/*
 * reset — 完全复位, 等效于 deInit + init
 */
Result MotorAPI::reset()
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    g_motor_pool[id_]->reset();
#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
    g_motor_pool[id_]->enterIsrDiagnostic(last_fault_);
#endif
    return Result::Ok;
}

/*
 * setTargetTorque — 直接覆盖 Iq 目标 (A), 是 writeSetpoint 的语法糖
 */
Result MotorAPI::setTargetTorque(float current_a)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;

    MotionSetpoint sp;
    sp.mode      = Mode::TORQUE_CONTROL;
    sp.torque_ff = current_a;
    g_motor_pool[id_]->writeSetpoint(sp);
    return Result::Ok;
}

/*
 * setTargetSpeed — 直接覆盖速度目标 (RPM), 是 writeSetpoint 的语法糖
 */
Result MotorAPI::setTargetSpeed(float rpm)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;

    MotionSetpoint sp;
    sp.mode   = Mode::VELOCITY_CONTROL;
    sp.vel_ff = rpm;
    g_motor_pool[id_]->writeSetpoint(sp);
    return Result::Ok;
}

/*
 * setTargetPosition — 设置目标位置 (弧度)
 *
 * 委托到 writeSetpoint, 仅在 POSITION_CONTROL 模式下有意义
 */
Result MotorAPI::setTargetPosition(float angle_rad)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;

    MotionSetpoint sp;
    sp.mode    = Mode::POSITION_CONTROL;
    sp.pos_ref = angle_rad;
    g_motor_pool[id_]->writeSetpoint(sp);
    return Result::Ok;
}

/*
 * writeSetpoint — 单拍运动设定值流式入口 (核心 API)
 *
 * 上层协调器每控制周期调用一次, 上拍覆盖下拍, 无任务、无完成判据。
 * 多轴 XY 同步由上层协调器在相同的时间基下并行调用多个实例的 writeSetpoint 实现。
 */
Result MotorAPI::writeSetpoint(const MotionSetpoint& sp)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    g_motor_pool[id_]->writeSetpoint(sp);
    return Result::Ok;
}

/*
 * === Phase 3 占位 API: SetpointFifo 回放 (本 Phase 一律返回 NotSupported) ===
 * 多 MCU 分布式同步用, 详见 Templates/Coordinator/README.md。
 * Phase 3 接通通信层 + lib 内 SetpointFifo 后启用。
 */
Result MotorAPI::appendSetpointSeg(const MotionSetpoint* seg, uint16_t count)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    if (seg == nullptr || count == 0) return Result::InvalidParam;
    return g_motor_pool[id_]->appendSetpointSeg(seg, count);
}

Result MotorAPI::triggerSegPlayback(PlaybackTrigger trig)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    return g_motor_pool[id_]->triggerSegPlayback(trig);
}

Result MotorAPI::abortSegPlayback()
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    return g_motor_pool[id_]->abortSegPlayback();
}

/*
 * selectMode — 切换控制模式
 *
 * 安全检查:
 *   1. mode != NONE (不能空模式)
 *   2. mode 不能是 Debug/Calib 模式 (这些只能通过专用 API)
 *   3. checkSafeModeChange: 验证从当前状态切换到新模式的合法性
 *   4. validateModeSelection: 验证新模式的反馈源/配置完整性
 */

Result MotorAPI::selectMode(Mode mode)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (mode == Mode::NONE) return Result::InvalidParam;
    if (isDebugOrCalib(mode)) return Result::InvalidParam;

    MotorManager* mgr = g_motor_pool[id_];
    if (isDebugOrCalibApiLocked(mgr))
    {
        return Result::InvalidState;
    }

    Result r = checkSafeModeChange(mode);
    if (r != Result::Ok) return r;

    r = g_motor_pool[id_]->validateModeSelection(mode);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(mode);
    return Result::Ok;
}


/*
 * setControlMethod — 设置控制算法 (FOC/DTC等)
 *
 * 当前版本仅 FOC, 预留接口
 */
Result MotorAPI::setControlMethod(ControlMethod method)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    (void)method;
    return Result::NotSupported;
}

/*
 * setDragCurrent — 设置 I/F 强拖电流幅值
 */
#if LIB_MOTOR_ENABLE_IF_STARTUP
Result MotorAPI::setDragCurrent(float A)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    g_motor_pool[id_]->setDragCurrent(A);
    return Result::Ok;
}
#endif

/*
 * setModulation — 设置调制方式 (SVPWM/SPWM)
 *
 * 当前版本固定 SVPWM, 预留接口
 */
Result MotorAPI::setModulation(ModulationMethod mod)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    (void)mod;
    return Result::NotSupported;
}

/*
 * startRLIdentification — 启动电阻/电感辨识 (校准模式)
 */

Result MotorAPI::startRLIdentification()
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLockedForRequest(g_motor_pool[id_], Mode::CALIB_RL_IDENTIFY))
    {
        return Result::InvalidState;
    }

    /* [P4] BuildCfg 未编 AUTO_IDENTIFY 时直接返 NotSupported */
    if (!Platform_MotorBuildConfig::supportsAutoIdentify())
    {
        return Result::NotSupported;
    }

    Result r = checkSafeModeChange(Mode::CALIB_RL_IDENTIFY);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::CALIB_RL_IDENTIFY);
    g_motor_pool[id_]->requestStart();
    return Result::Ok;
}


/*
 * getMonitorData — 获取监控数据 (电流/电压/速度/温度/故障码等)
 */
void MotorAPI::getMonitorData(MotorMonitorData& out_data) const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr)
    {
        out_data = {};
        return;
    }
    g_motor_pool[id_]->getMonitorData(out_data);
}

/*
 * getDebugData — 获取调试数据 (观测器中间量/PID内部状态等)
 */
void MotorAPI::getDebugData(MotorDebugData& out_data) const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr)
    {
        out_data = {};
        return;
    }
    g_motor_pool[id_]->getDebugData(out_data);
}

/*
 * getState — 获取当前顶层状态 (INIT/ADC_CAL/STOP/RUN/STOPPING/ERROR)
 */
State MotorAPI::getState() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return State::UNINITIALIZED;
    return g_motor_pool[id_]->state();
}

Fault MotorAPI::getFault() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return last_fault_;
    return g_motor_pool[id_]->fault();
}

MotorConfigFaultDetail MotorAPI::getConfigFaultDetail() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return last_config_fault_detail_;
    return g_motor_pool[id_]->configFaultDetail();
}

/*
 * getSpeed — 获取当前转速 (RPM)
 */
float MotorAPI::getSpeed() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return 0.0f;
    return g_motor_pool[id_]->getSpeed();
}

/* [本轮新增] 配置错误码转可读字符串 */
const char* MotorAPI::getConfigFaultDetailString() const
{
    return MotorConfigFaultDetailToString(getConfigFaultDetail());
}

/* [本轮新增] 堵转重试计数 */
uint8_t MotorAPI::getStallRetryCounter() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return 0;
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    return g_motor_pool[id_]->stallRestartAttempts();
#else
    return 0;
#endif
}

/* [P2] SMO 收敛后辨识得到的 Ke (写 cfg.physical.ke_v_per_rad_s_identified) */
float MotorAPI::getKEstimated() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return 0.0f;
    return g_motor_pool[id_]->config().physical.ke_v_per_rad_s_identified;
}

/* [P2] RL 辨识完成轮询 */
bool MotorAPI::isRLIdentifyDone() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return false;
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    return g_motor_pool[id_]->event().rl_identify_done != 0U;
#else
    return false;
#endif
}

/* [P2] Ke 辨识完成轮询 (由 startKEstimation 触发) */
bool MotorAPI::isKEstimationDone() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return false;
    return g_motor_pool[id_]->event().ke_identify_done != 0U;
}

/* [P2] 顺逆风启动完成轮询 */
bool MotorAPI::isFlyingStartDone() const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return false;
#if LIB_MOTOR_ENABLE_FLYING_START
    return g_motor_pool[id_]->event().flying_start_done != 0U;
#else
    return false;
#endif
}

/* [P2] startKEstimation 实现 (占位: 真正落地在 D 组 MotorKEstimationFSM 中) */
Result MotorAPI::startKEstimation()
{
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    return g_motor_pool[id_]->startKEstimationInternal();
#else
    (void)id_;
    return Result::NotSupported;
#endif
}

/* [P2] triggerFlyingStartIfSpinning 占位, E 组实现 */
Result MotorAPI::triggerFlyingStartIfSpinning()
{
#if LIB_MOTOR_ENABLE_FLYING_START
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    return g_motor_pool[id_]->triggerFlyingStartIfSpinningInternal();
#else
    (void)id_;
    return Result::NotSupported;
#endif
}

/* [P2] 一次性获取 identified 电机参数快照 (RL 辨识 + Ke 辨识完成后填) */
MotorAPI::MotorLearnedParamsSnapshot MotorAPI::getLearnedMotorParams() const
{
    MotorLearnedParamsSnapshot snap{};
    if (id_ < 0 || g_motor_pool[id_] == nullptr)
    {
        snap.rs_ohm          = 0.0f;
        snap.ls_h            = 0.0f;
        snap.ke_v_per_rad_s  = 0.0f;
        snap.valid           = false;
        return snap;
    }
    const auto& p = g_motor_pool[id_]->config().physical;
    snap.rs_ohm         = p.rs_ohm_identified;
    snap.ls_h           = p.ls_h_identified;
    snap.ke_v_per_rad_s = p.ke_v_per_rad_s_identified;
    snap.valid          = (snap.rs_ohm > 0.0f) && (snap.ls_h > 0.0f);
    return snap;
}

/*
 * lockDebug / unlockDebug — 调试锁
 *
 * 用途: 防止调试 API 被意外调用 (如生产环境)
 *       必须 unlock → 调 debug → lock 的使用流程
 */
void MotorAPI::lockDebug()
{
    debug_locked_ = true;
}

void MotorAPI::unlockDebug()
{
    debug_locked_ = false;
}

/*
 * setPIDGains — 在线调整 PID 参数
 *
 * 参数校验:
 *   - kp/ki/kd 必须为有限值且非负
 */
Result MotorAPI::setPIDGains(PidGroup group, float kp, float ki, float kd)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return Result::InvalidParam;
    }

    return g_motor_pool[id_]->setPIDGains(group, kp, ki, kd)
        ? Result::Ok
        : Result::InvalidParam;
}

Result MotorAPI::setRunPolicy(const MotorRunPolicy& policy)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    return g_motor_pool[id_]->setRunPolicy(policy);
}

Result MotorAPI::getRunPolicy(MotorRunPolicy& policy) const
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    return g_motor_pool[id_]->getRunPolicy(policy);
}

Result MotorAPI::setEnergyBudget(const MotorEnergyBudget& budget)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLocked(g_motor_pool[id_])) return Result::InvalidState;
    if (!MotorFeatureRegistry::supportsBrakeEnergyFsm())
    {
        (void)budget;
        return Result::NotSupported;
    }

#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
    g_motor_pool[id_]->setEnergyBudget(budget);
    return Result::Ok;
#else
    (void)budget;
    return Result::NotSupported;
#endif
}

// ================================================================
//  危险调试 API (需 ENABLE_DANGEROUS_TEST_API 宏 + debug_locked=false)
//  这些 API 绕过正常的安全检查, 仅用于调试和测试
// ================================================================
#ifdef ENABLE_DANGEROUS_TEST_API

/*
 * debugPWMManual — 手动 PWM 开环输出
 *
 * 直接设置三相占空比, 绕过 FOC 控制环
 * 安全: 需要 debug_locked_ == false 且通过 checkSafeModeChange
 */
Result MotorAPI::debugPWMManual(float u, float v, float w)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    Result r = checkSafeModeChange(Mode::DEBUG_PWM_MANUAL);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_PWM_MANUAL);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->setRawPWM(u, v, w);
    return Result::Ok;
}

/*
 * debugCurrentLock — 电流锁存调试
 *
 * 锁定 d/q 轴电流到固定值, 用于调试电流环
 */
Result MotorAPI::debugCurrentLock(float i_d, float i_q)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    Result r = checkSafeModeChange(Mode::DEBUG_CURRENT_LOCK);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_CURRENT_LOCK);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->setTargetId(i_d);
    g_motor_pool[id_]->setTargetTorque(i_q);
    return Result::Ok;
}

/*
 * debugIFControl — IF 强拖调试 (简化参数)
 *
 * 以 IF (流频比) 方式开环拖动电机
 */
#if LIB_MOTOR_ENABLE_IF_STARTUP
Result MotorAPI::debugIFControl(float target_rpm, float id_amp, float iq_amp, float ramp_time_s)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    Result r = checkSafeModeChange(Mode::DEBUG_IF_DRAG);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_IF_DRAG);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->clearDebugStartupProfiles();
    g_motor_pool[id_]->setDebugRampTime(ramp_time_s);
    g_motor_pool[id_]->setTargetSpeed(target_rpm);
    g_motor_pool[id_]->setTargetId(id_amp);
    g_motor_pool[id_]->setTargetTorque(iq_amp);
    return Result::Ok;
}

/*
 * debugIFControl — IF 强拖调试 (分段启动曲线)
 *
 * 使用 MotorIFStartupProfile 定义多段启动曲线
 */
Result MotorAPI::debugIFControl(const MotorIFStartupProfile& profile)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (profile.phases == nullptr || profile.phase_count == 0U) return Result::InvalidParam;

    Result r = checkSafeModeChange(Mode::DEBUG_IF_DRAG);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_IF_DRAG);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->setDebugIFStartupProfile(&profile);
    return Result::Ok;
}

#if LIB_MOTOR_ENABLE_SMO
Result MotorAPI::debugIFSMOObserver(const MotorIFStartupProfile& profile)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (profile.phases == nullptr || profile.phase_count == 0U) return Result::InvalidParam;

    Result r = checkSafeModeChange(Mode::DEBUG_IF_SMO_OBSERVER);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_IF_SMO_OBSERVER);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->setDebugIFStartupProfile(&profile);
    return Result::Ok;
}
#endif

#if LIB_MOTOR_ENABLE_HFI
Result MotorAPI::debugIFHFIObserver(const MotorIFStartupProfile& profile)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (profile.phases == nullptr || profile.phase_count == 0U) return Result::InvalidParam;

    Result r = checkSafeModeChange(Mode::DEBUG_IF_HFI_OBSERVER);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_IF_HFI_OBSERVER);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->setDebugIFStartupProfile(&profile);
    return Result::Ok;
}
#endif
#endif

/*
 * debugVFControl — VF 调试 (简化参数)
 *
 * 以 VF (压频比) 方式开环拖动电机
 */
#if LIB_MOTOR_ENABLE_HFI
Result MotorAPI::debugHFIObserver()
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    Result r = checkSafeModeChange(Mode::DEBUG_HFI_OBSERVER);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_HFI_OBSERVER);
    g_motor_pool[id_]->requestStart();
    return Result::Ok;
}
#endif

Result MotorAPI::debugVFControl(float duty, float speed_rpm, float ramp_time_s)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;

    Result r = checkSafeModeChange(Mode::DEBUG_VF_DRAG);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_VF_DRAG);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->clearDebugStartupProfiles();
    g_motor_pool[id_]->setDebugRampTime(ramp_time_s);
    g_motor_pool[id_]->setVFDutyBias(duty);
    g_motor_pool[id_]->setTargetSpeed(speed_rpm);
    return Result::Ok;
}

/*
 * debugVFControl — VF 调试 (分段启动曲线)
 */
Result MotorAPI::debugVFControl(const MotorVFStartupProfile& profile)
{
    if (debug_locked_) return Result::InvalidState;
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (profile.phases == nullptr || profile.phase_count == 0U) return Result::InvalidParam;

    Result r = checkSafeModeChange(Mode::DEBUG_VF_DRAG);
    if (r != Result::Ok) return r;

    g_motor_pool[id_]->setMode(Mode::DEBUG_VF_DRAG);
    g_motor_pool[id_]->requestStart();
    g_motor_pool[id_]->setDebugVFStartupProfile(&profile);
    return Result::Ok;
}

#endif

/*
 * isDebugOrCalib — 判断是否为调试或校准模式
 */
bool MotorAPI::isDebugOrCalib(Mode m)
{
    return MotorCommandGuard::isDebugOrCalib(m);
}

/*
 * checkSafeModeChange — 校验从当前状态切换到新模式的合法性
 */
Result MotorAPI::checkSafeModeChange(Mode new_mode)
{
    if (id_ < 0 || g_motor_pool[id_] == nullptr) return Result::InvalidHandle;
    if (isDebugOrCalibApiLockedForRequest(g_motor_pool[id_], new_mode))
    {
        return Result::InvalidState;
    }

    return MotorCommandGuard::validateApiModeChange(
        g_motor_pool[id_]->state(),
        g_motor_pool[id_]->mode(),
        new_mode);
}

} // namespace Lib_Motor

/*
 * Motor_Global_Process_Handler — ADC 注入组转换完成的 ISR 入口
 *
 * 每个控制周期由 BSP 层 ADC ISR 调用:
 *   STM32 ADC 注入组转换完成中断
 *     → 用户 BSP 读取 ADC 寄存器缓存
 *     → 调用本函数, 驱动一次完整的控制周期
 *
 * 安全性: 若 g_motor_pool[motor_id] 为 nullptr (电机尚未 MotorAPI::init),
 *         直接跳过, 不做任何操作。
 *
 * 调用者示例 (用户 BSP 层):
 *   void ADC3_IRQHandler(void) {
 *       if (ADC_GetITStatus(ADC3, ADC_IT_JEOC)) {
 *           ADC_ClearITPendingBit(ADC3, ADC_IT_JEOC);
 *           BSP_ReadADCResults(...);         // 将 ADC 值缓存到 BSP
 *           Motor_Global_Process_Handler(0); // 驱动电机控制周期
 *       }
 *   }
 *
 * @param motor_id  电机实例编号 [0, MOTOR_MAX_INSTANCES)
 *                  对应 MotorAPI(motor_id, cfg) 创建时绑定的 id
 */
extern "C" bool Motor_Global_Process_Handler_TickAccepted(int motor_id)
{
    if (motor_id >= 0 && motor_id < MOTOR_MAX_INSTANCES)
    {
        Lib_Motor::MotorManager* mgr = Lib_Motor::g_motor_pool[motor_id];
        if (mgr != nullptr)
        {
#if LIB_MOTOR_ENABLE_ISR_LOAD_DIAGNOSTIC
            if (mgr->state() == Lib_Motor::State::ISR_DIAGNOSTIC)
            {
                if (!Lib_Motor::acceptIsrDiagnosticPulse(motor_id, *mgr))
                {
                    return false;
                }

                mgr->tick();
                return true;
            }
#endif

            mgr->tick();
            return true;
        }
    }

    return false;
}

extern "C" void Motor_Global_Process_Handler(int motor_id)
{
    (void)Motor_Global_Process_Handler_TickAccepted(motor_id);
    return;

    if (motor_id >= 0 && motor_id < MOTOR_MAX_INSTANCES)
    {
        Lib_Motor::MotorManager* mgr = Lib_Motor::g_motor_pool[motor_id];
        if (mgr != nullptr)                    // 电机未创建时跳过, 不执行任何操作
        {
            mgr->tick();                       // 执行一次完整控制周期 (传感器→FSM→控制算法→PWM)
        }
    }
}
