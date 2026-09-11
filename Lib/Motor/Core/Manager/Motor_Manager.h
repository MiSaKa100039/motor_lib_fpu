/*
 * Motor_Manager.h - 电机核心管理器声明
 *
 * MotorManager 是电机库内部的控制核心
 *   tick()
 *     -> updateSensorMeasurements()
 *     -> updatePositionSensorMeasurement()
 *     -> runSafetyCheck()
 *     -> Motor_FSM::step()
 *     -> runObserverLoop() / runControlLoop() / runXXX()
 *     -> set_duty()
 *
 * 用户不直接操作 MotorManager，而是通过 Public 层的 MotorAPI 间接访问。
 */

#pragma once

#include "../../Public/Motor_Definitions.h"
#include "../../Public/Motor_Config.h"
#include "../../Control/Motor_Control.h"
#include "../../Control/Utils/LowPassFilter.h"
#include "../../Observer/Observer_Base.h"
#include "../Safety/Motor_RuntimeMonitor.h"
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
#include "../../Observer/Observer_SMO.h"
#endif
#if LIB_MOTOR_ENABLE_HFI
#include "../../Observer/Observer_HFI.h"
#endif
#if LIB_MOTOR_ENABLE_SENSOR
#include "../../Observer/Observer_Sensor.h"
#endif
#include "../Runtime/Motor_Runtime.h"
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
#include "../../Control/Utils/AngleGenerator.h"
#endif
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
#include "../Identify/Motor_RLIdentifyRoutine.h"
#endif
#if LIB_MOTOR_ENABLE_FLYING_START
#include "../Startup/FlyingStart/Motor_FlyingStartRoutine.h"
#endif

namespace Lib_Motor
{

class Motor_RunPhaseFSM;
class Motor_LifecycleFSM;
class Motor_SafetyFSM;
class Motor_CommandFSM;
class Motor_StreamFSM;
class Motor_AngleFSM;
class Motor_StopFSM;
class Motor_EnergyFSM;
class MotorSignalHealth;

class MotorManager
{
    friend class Motor_FSM;
    friend class Motor_LifecycleFSM;
    friend class Motor_RunPhaseFSM;
    friend class Motor_SafetyFSM;
    friend class Motor_CommandFSM;
    friend class Motor_StreamFSM;
    friend class Motor_AngleFSM;
    friend class Motor_StopFSM;
    friend class Motor_EnergyFSM;
    friend class MotorSignalHealth;

    /* 内部分阶段流程: 由 Manager 每 tick 调用, 不属于 Core/FSM 架构层状态机。 */
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    friend class MotorRLIdentifyRoutine;
#endif
#if LIB_MOTOR_ENABLE_FLYING_START
    friend class MotorFlyingStartRoutine;
#endif

#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    enum class SetpointPlaybackState : uint8_t
    {
        IDLE = 0,
        PRELOADED,
        ARMED,
        RUNNING,
        ABORTED,
    };
#endif

public:
    explicit MotorManager(const MotorConfig& config);

    /* ======================== 生命周期 ======================== */
    void init();
    void reset();
    void setState(State s);

    /* ======================== MotorAPI 调用入口 ======================== */
    void setMode(Mode mode);
    void requestStart();
    void clearFaultBits();
    void clearFaultAndStop();
    void setConfigFaultDetail(MotorConfigFaultDetail detail);
    void enterIsrDiagnostic(Fault fault);
    void requestStop(StopMode mode = StopMode::COAST);
    void requestEmergencyStop();
    Result setTargetSpeed(float rpm);
    void setTargetTorque(float current_a);
    void setTargetId(float current_a);
    Result writeSetpoint(const MotionSetpoint& sp);

    /* === 本地 FIFO/trigger 同步回放 ===
     * 实现见 Motor_ManagerAPI.cpp: BuildCfg 关闭时返回 NotSupported, 开启后使用固定容量 FIFO。
     */
    Result appendSetpointSeg(const MotionSetpoint* seg, uint16_t count);
    Result triggerSegPlayback(PlaybackTrigger trig);
    Result abortSegPlayback();
    bool setPIDGains(PidGroup group, float kp, float ki, float kd);
    Result validateModeSelection(Mode mode) const;
    Result setRunPolicy(const MotorRunPolicy& policy);
    Result getRunPolicy(MotorRunPolicy& policy) const;
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
    void setEnergyBudget(const MotorEnergyBudget& budget);
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    void setVFDutyBias(float bias);
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
    void setRawPWM(float u, float v, float w);
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
    void setDebugRampTime(float seconds);
    void clearDebugStartupProfiles();
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    bool setDebugVFStartupProfile(const MotorVFStartupProfile* profile);
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    bool setDebugIFStartupProfile(const MotorIFStartupProfile* profile);
#endif

    /* ======================== 查询接口 ======================== */
    float   getSpeed() const;
    float   getBusVoltage() const;
    float   getCurrentQ() const;
    uint8_t getPolePairs() const;

    void getMonitorData(MotorMonitorData& data) const;
    void getDebugData(MotorDebugData& data) const;
    uint8_t getTelemetryFloats(const MotorTelemetryChannel* channels,
                               uint8_t channel_count,
                               float* out_values) const;

    State        state() const      { return state_; }
    Mode         mode() const       { return mode_; }
    Mode         targetMode() const { return target_mode_; }
    Mode         startupTargetMode() const { return config_.control.startup_target_mode; }
    const MotorConfig& config() const { return config_; }
    MotorConfig& configMut() { return config_; }

    /* 堵转自动重试计数 (供 API 调用诊断) */
    uint8_t stallRestartAttempts() const
    {
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        return stall_restart_attempts_;
#else
        return 0;
#endif
    }
    Fault        fault() const      { return fault_; }

    // FSM 时钟节拍 (每控制 tick 自增), 仅供 lib 内部时戳用 (budget 租约等)
    // 不依赖 wall clock, 与上层 loop 漂移无关
    uint32_t     fsmTimerTicks() const { return ctx_.fsm_timer_ticks; }
    MotorConfigFaultDetail configFaultDetail() const { return config_fault_detail_; }
    MotorRuntimeFaultDetail runtimeFaultDetail() const { return runtime_fault_detail_; }
    StopMode     stopMode() const   { return stop_mode_; }
    StreamState  streamState() const { return stream_state_; }
    const MotorEvent& event() const { return event_; }
    RunPhase     runPhase() const   { return run_phase_; }
    void         setRunPhase(RunPhase phase) { run_phase_ = phase; }
    const RuntimeCtx& ctx() const   { return ctx_; }
#if MOTOR_LIB_HOST_TEST
    RuntimeCtx& ctxMutForTest() { return ctx_; }
    MotorEvent& eventMutForTest() { return event_; }
    void setActiveModeForTest(Mode mode)
    {
        mode_ = mode;
        target_mode_ = mode;
    }
    RuntimeCurrent computeIqReferenceForTest() { return computeIqReference(); }
    void clearDerivedTargetsForTest() { clearDerivedTargets(); }
    float speedPidOutputLimitForTest() const
    {
        return controller_.pid_speed.outputLimit();
    }
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    bool stepDebugIFProfileForTest(float& speed_rpm, float& id_ref, float& iq_ref);
    bool debugIFProfileCompleteForTest() const
    {
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        return ctx_.debug_profile.complete;
#else
        return ctx_.debug_profile_complete;
#endif
    }
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    bool stepIFStartupProfileForTest(float& speed_rpm, float& id_ref, float& iq_ref);
    void triggerIFStartupFailureForTest() { handleIFStartupFailure(); }
    void processPendingStartupRestartForTest() { processPendingStartupRestart(); }
    bool startupRestartPendingForTest() const { return startup_restart_pending_; }
    uint8_t startupRestartAttemptsForTest() const { return startup_restart_attempts_; }
    bool runRequestedForTest() const { return run_requested_; }
    bool ifProfileHoldTargetForTest() const { return if_profile_hold_target_; }
    bool ifStartupProfileCompleteForTest() const { return ifStartupProfileComplete(); }
#endif
#if LIB_MOTOR_ENABLE_SMO && LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    uint16_t smoFusionProgressQ15ForTest() const { return smo_fusion_progress_q15_; }
#endif
#endif
    const MotorRunPolicy& defaultPolicy() const { return config_.default_run_policy; }

    /* [P2] 辨识 / 顺逆风启动内部入口 (API 委托) */
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    Result startKEstimationInternal();   // D 组实现
#endif
#if LIB_MOTOR_ENABLE_FLYING_START
    Result triggerFlyingStartIfSpinningInternal();   // E 组实现
#endif
    const MotorRunPolicy& activePolicy() const { return active_policy_; }
    const MotorRunPolicy& pendingPolicy() const { return pending_policy_; }

    /* ======================== 控制周期入口 ======================== */
    void tick();
    void serviceSlowMonitor();

    /* ======================== 运行辅助 ======================== */
    void runAdcCalibration();
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
    void runPWMManual();
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    void runVFControl();
#endif
#if LIB_MOTOR_ENABLE_DEBUG_HFI_OBSERVER
    void runHFIObserverTest();
#endif
#if LIB_MOTOR_ENABLE_DEBUG_HFI_ANY
    void runHFIShadowObserver(bool inject_voltage);
#endif
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    void runSMOShadowObserver();
#endif
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY || LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
    void runCurrentLock();
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    void runIFControl();
#endif

private:
    /* ======================== 状态机上下文 ======================== */
    State      state_ = State::INIT;
    Mode       mode_ = Mode::NONE;
    Mode       target_mode_ = Mode::NONE;
    bool       run_requested_ = false;
    RunPhase   run_phase_ = RunPhase::NONE;
    Fault      fault_ = Fault::NONE;
    MotorConfigFaultDetail config_fault_detail_ = MotorConfigFaultDetail::NONE;
    MotorRuntimeFaultDetail runtime_fault_detail_ = MotorRuntimeFaultDetail::NONE;
    uint32_t hardware_fault_flags_ = MOTOR_HAL_HW_FAULT_NONE;
    SafetyState safety_state_ = SafetyState::CLEAR;
    CommandState command_state_ = CommandState::IDLE;
    StreamState    stream_state_ = StreamState::NONE;
#if LIB_MOTOR_ENABLE_SETPOINT_FIFO_PLAYBACK
    SetpointPlaybackState setpoint_playback_state_ = SetpointPlaybackState::IDLE;
    MotionSetpoint setpoint_fifo_[LIB_MOTOR_SETPOINT_FIFO_CAPACITY];
    uint16_t setpoint_fifo_head_ = 0U;
    uint16_t setpoint_fifo_tail_ = 0U;
    uint16_t setpoint_fifo_count_ = 0U;
#endif
    AngleState   angle_state_ = AngleState::NONE;
    StopState    stop_state_ = StopState::IDLE;
    EnergyState energy_state_ = EnergyState::IDLE;
    MotorEvent event_;
    StopMode   stop_mode_ = StopMode::COAST;

    /* ======================== 公共滤波与工具 ======================== */
    LowPassFilter lpf_ia_;
    LowPassFilter lpf_ib_;
    LowPassFilter lpf_ic_;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    LowPassFilter lpf_ibus_;
#endif
    LowPassFilter lpf_vbus_;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    LowPassFilter lpf_vphase_u_;
    LowPassFilter lpf_vphase_v_;
    LowPassFilter lpf_vphase_w_;
#endif

    float dt_ = 0.0f;
    uint32_t auto_calib_interval_ticks_ = 0;

mutable MotorConfig config_;   // mutable: 运行期辨识流程可写 physical.*_identified 缓存
    MotorRunPolicy active_policy_;
    MotorRunPolicy pending_policy_;
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
    MotorEnergyBudget energy_budget_;
    // 上层 setEnergyBudget 是否已注入过且未超时; false=未注入或已超时, 走 cfg 静态路径
    bool        budget_active_ = false;
    bool        budget_loss_blocked_ = false;
    // 租约过期时戳 (fsm_timer_ticks_); 当 fsm_timer_ticks_ > budget_expire_ticks_ 时回退
    uint32_t    budget_expire_ticks_ = 0U;
    // 标记 budget 已在本周期内被检测过期 (供 event.budget_expired latched)
    // 注: 真正 latched event 由 Motor_EnergyFSM 检测时执行
#endif

    RuntimeCtx ctx_;

    MotorControl controller_;
    MotorRuntimeMonitor runtime_monitor_;
#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    MotorRLIdentifyRoutine rl_identify_routine_;
    // Ke 辨识内部状态 (>0 表示剩余 tick 数, 0 表示空闲)
    uint32_t ke_identify_ticks_remaining_ = 0U;
#endif
#if LIB_MOTOR_ENABLE_FLYING_START
    MotorFlyingStartRoutine flying_start_routine_;
#endif
    uint16_t sensor_invalid_ticks_ = 0;
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    uint16_t redundant_error_ticks_ = 0;
#endif
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    uint32_t stall_candidate_ticks_ = 0;
    uint32_t stall_release_ticks_ = 0;
    uint32_t stall_restart_wait_ticks_ = 0;
    uint32_t stall_total_count_ = 0;
    uint8_t  stall_restart_attempts_ = 0;
    bool     stall_restart_pending_ = false;
    uint32_t stall_thaw_remaining_ticks_ = 0;  // 堵转释放后渐进解冻剩余 tick 数
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
    uint32_t startup_restart_wait_ticks_ = 0U;
    uint32_t startup_restart_interval_ticks_ = 0U;
    uint8_t  startup_restart_attempts_ = 0U;
    bool     startup_restart_pending_ = false;
    bool     if_profile_hold_target_ = false;
#endif

    /* ======================== 主控制链 ======================== */
    void updateSensorMeasurements();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    void configureFixedRuntimeScales();
    void refreshFixedRuntimeOffsetCounts();
    void configureFixedCurrentPidRuntime(bool reset_integrator);
    void refreshFixedCurrentPidOutputLimit();
    RuntimeCurrent clampCurrentTargetQ15(RuntimeCurrent value) const;
    void setPwmDutyQ15(const FixedNumeric::DutyAbc& duty);
#endif
#if LIB_MOTOR_ENABLE_SENSOR
    void updatePositionSensorMeasurement();
#endif
    void runSafetyCheck();
    void runObserverLoop();
    void runControlLoop();
    void updateCurrentPidVoltageLimit(float omega_elec_rad_s);

    RuntimeCurrent computeIqReference();
    void  syncRuntimeInputs(float id_ref, float iq_ref);
    void  syncRuntimeTargets();
    void  publishRuntimeOutputs();
    Result applySetpoint(const MotionSetpoint& sp);
    void  resetSetpointPlayback();
    void  serviceSetpointPlayback();
    Result appendSetpointPlaybackSegment(const MotionSetpoint* seg, uint16_t count);
    Result triggerSetpointPlayback(PlaybackTrigger trig);
    Result abortSetpointPlayback();
    Result validateRunPolicy(const MotorRunPolicy& policy) const;
    void  updateStallProtection(float speed_error, float iq_ref, float iq_limit);
    void  processPendingStallRestart();
#if LIB_MOTOR_ENABLE_IF_STARTUP
    void  clearStartupRestartState();
    void  resetIFStartupObserverState();
    void  processPendingStartupRestart();
    void  handleIFStartupFailure();
    bool  ifStartupProfileComplete() const;
#endif
    void  clearControlTargets();
    void  clearDerivedTargets();
    void  clearStallState();
    void  transitionToStop(StopMode mode, bool clear_user_targets, bool reset_rl_identify = true);
    void  prepareModeTransition(Mode previous_mode, Mode next_mode);
    void  writePwmOutputs();
    void  writePwmZeroOutputs();

#if MOTOR_BUILD_NTC_SLOTS > 0 && !LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    float calculateNTC(uint16_t raw_adc, uint8_t ntc_index);
#endif
    bool  checkConfigValid(const MotorConfig& cfg) const;
    bool  validateControlFeedbackSource();
    void  setFault(Fault fault);
    void  setFault(Fault fault, MotorRuntimeFaultDetail detail);
    void  stopForFault(Fault fault);
    void  stopForFault(Fault fault, MotorRuntimeFaultDetail detail);
    void  pollHardwareFaultLatch();
    void  clearRuntimeFaultDetail();
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    void  resetSmoAngleDirectionLatch();
    int8_t updateSmoAngleDirectionLatch(RuntimeSpeed direction_hint);
    RuntimeAngle correctSmoAngleForControl(RuntimeAngle raw_angle,
                                           RuntimeSpeed direction_hint);
#endif

    /* ======================== 观测器 / 传感器 ======================== */
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    SlidingModeObserver smo_;
    int8_t smo_angle_direction_ = 0;
#if LIB_MOTOR_ENABLE_SMO
    uint16_t smo_handover_confirm_ticks_ = 0U;
    uint32_t smo_fusion_ticks_ = 0U;
    uint32_t smo_fusion_total_ticks_ = 1U;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    int32_t smo_fusion_angle_offset_q15_ = 0;
    uint16_t smo_fusion_progress_q15_ = 0U;
    uint16_t smo_fusion_step_q15_ = 0U;
    uint32_t smo_fusion_remainder_q15_ = 0U;
    uint32_t smo_fusion_remainder_accum_ = 0U;
#else
    float smo_fusion_angle_offset_rad_ = 0.0f;
#endif
    RuntimeSpeed smo_fusion_start_speed_ = 0;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    uint32_t smo_handover_angle_error_phase_ = 0U;
#else
    float smo_handover_angle_error_rad_ = 0.0f;
#endif
#endif
#endif
#if LIB_MOTOR_ENABLE_HFI
    HighFreqInjectionObserver hfi_;
#endif
#if LIB_MOTOR_ENABLE_SENSOR
    SensorObserver rotor_sensor_obs_;
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    SensorObserver redundant_sensor_obs_;
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    SensorObserver output_sensor_obs_;
#endif
#endif

    /* ======================== 启动辅助 ======================== */
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL || LIB_MOTOR_ENABLE_DEBUG_IF_ANY
    AngleGenerator if_angle_gen_;
#endif

    /* ======================== 调试辅助 ======================== */
#if LIB_MOTOR_ENABLE_IF_STARTUP
    void resetIFStartupProfileState();
    void prepareIFStartupRuntimeForStart();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    bool prepareIFStartupProfileSnapshot(const MotorIFStartupProfile* profile);
    void cacheIFStartupRestartInterval();
    bool getIFStartupProfileCommand(RuntimeSpeed& speed_rpm,
                                    RuntimeCurrent& id_ref,
                                    RuntimeCurrent& iq_ref);
#else
    bool getIFStartupProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref);
#endif
#endif
#if LIB_MOTOR_ENABLE_DEBUG_PROFILE_ANY
    void resetDebugProfileState();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    void prepareDebugOpenLoopAngleRampForStart();
#endif
#endif
#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    bool getDebugVFProfileCommand(RuntimeSpeed& speed_rpm, RuntimeDuty& duty);
#else
    bool getDebugVFProfileCommand(float& speed_rpm, float& duty);
#endif
#endif
#if LIB_MOTOR_ENABLE_DEBUG_IF_ANY
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    bool getDebugIFProfileCommand(RuntimeSpeed& speed_rpm,
                                  RuntimeCurrent& id_ref,
                                  RuntimeCurrent& iq_ref);
#else
    bool getDebugIFProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref);
#endif
#endif
};

} // namespace Lib_Motor
