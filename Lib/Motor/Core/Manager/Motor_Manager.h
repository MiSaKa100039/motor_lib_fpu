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
#include "../../Common/Filter/LowPassFilter.h"
#include "../../Safety/Basic/Motor_RuntimeMonitor.h"
#include "../../Observer/Observer_Base.h"
#if LIB_MOTOR_ENABLE_SMO
#include "../../Observer/Observer_SMO.h"
#endif
#if LIB_MOTOR_ENABLE_HFI
#include "../../Observer/Observer_HFI.h"
#endif
#include "../../Observer/Observer_Sensor.h"
#include "../Sampling/Motor_SingleShunt.h"
#include "../Runtime/Motor_FastRuntimeCtx.h"
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
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

    struct RuntimeCtx;
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
    void setTargetSpeed(float rpm);
    void setTargetTorque(float current_a);
    void setTargetId(float current_a);
    void writeSetpoint(const MotionSetpoint& sp);

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
#if LIB_MOTOR_ENABLE_IF_STARTUP
    void setDragCurrent(float A);
#endif
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    void setVFDutyBias(float bias);
    void setRawPWM(float u, float v, float w);
    void setDebugRampTime(float seconds);
    void clearDebugStartupProfiles();
    void setDebugVFStartupProfile(const MotorVFStartupProfile* profile);
#if LIB_MOTOR_ENABLE_IF_STARTUP
    void setDebugIFStartupProfile(const MotorIFStartupProfile* profile);
#endif
#endif

    /* ======================== 查询接口 ======================== */
    float   getSpeed() const;
    float   getBusVoltage() const;
    float   getCurrentQ() const;
    uint8_t getPolePairs() const;

    void getMonitorData(MotorMonitorData& data) const;
    void getDebugData(MotorDebugData& data) const;

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
    StopMode     stopMode() const   { return stop_mode_; }
    StreamState  streamState() const { return stream_state_; }
    const MotorEvent& event() const { return event_; }
    RunPhase     runPhase() const   { return run_phase_; }
    void         setRunPhase(RunPhase phase) { run_phase_ = phase; }
    const RuntimeCtx& ctx() const   { return ctx_; }
    const FastRuntimeCtx& fastCtx() const { return fast_ctx_; }
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

    /* ======================== 运行辅助 ======================== */
    void runAdcCalibration();
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    void runPWMManual();
    void runVFControl();
#if LIB_MOTOR_ENABLE_HFI
    void runHFIObserverTest();
    void runHFIShadowObserver(bool inject_voltage);
#endif
#if LIB_MOTOR_ENABLE_SMO
    void runSMOShadowObserver();
#endif
#endif
    void runCurrentLock();
#if LIB_MOTOR_ENABLE_IF_STARTUP
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
#if LIB_MOTOR_ENABLE_IF_STARTUP
    float drag_current_override_ = 0.0f;
#endif

mutable MotorConfig config_;   // mutable: SMO 收敛后写 physical.ke_v_per_rad_s_identified 等 identified 字段 (cache)
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

    struct RuntimeCtx
    {
        /* [A] 预计算换算系数 */
        float current_scale_Phase_per_count;
#if MOTOR_BUILD_HAS_BUS_CURRENT
        float current_scale_Bus_per_count;
#endif
        float voltage_scale_V_per_count;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
        float phase_voltage_scale_V_per_count;
#endif

        /* [B] ADC 零点偏移 */
        float calib_accum_ia, calib_accum_ib, calib_accum_ic;
        float calib_accum_single_shunt_first, calib_accum_single_shunt_second;
#if MOTOR_BUILD_HAS_BUS_CURRENT
        float calib_accum_ibus;
#endif
        uint16_t calib_counter;
        float offset_ia, offset_ib, offset_ic;
        float offset_single_shunt_first, offset_single_shunt_second;
#if MOTOR_BUILD_HAS_BUS_CURRENT
        float offset_ibus;
#endif

#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
        /* [B2] ADC raw samples for bring-up diagnostics only. */
        uint16_t adc_raw_phase_u;
        uint16_t adc_raw_phase_v;
        uint16_t adc_raw_phase_w;
        uint16_t adc_raw_single_shunt_first;
        uint16_t adc_raw_single_shunt_second;
        uint16_t adc_raw_bus_current;
        uint16_t adc_raw_vbus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
        uint16_t adc_raw_phase_voltage_u;
        uint16_t adc_raw_phase_voltage_v;
        uint16_t adc_raw_phase_voltage_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
        uint16_t adc_raw_temp[MOTOR_BUILD_NTC_SLOTS];
#endif
#endif

        /* [C] 传感器与观测器反馈 */
        float i_a, i_b, i_c;
#if MOTOR_BUILD_HAS_BUS_CURRENT
        float i_bus;
#endif
        float v_bus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
        float v_phase_u, v_phase_v, v_phase_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
        float temperature_c[MOTOR_BUILD_NTC_SLOTS];
#endif
        float angle_elec;
        float angle_elec_command;
        float angle_elec_observer;
        float angle_elec_sensor;
        float angle_mech_sensor;
        float speed_rpm;
        float speed_rpm_observer;
        float speed_rpm_sensor;
#if LIB_MOTOR_ENABLE_SMO
        ObserverEstimate smo_estimate;
#endif
#if LIB_MOTOR_ENABLE_HFI
        ObserverEstimate hfi_estimate;
        HfiInjectionCommand hfi_injection;
#endif
        int32_t sensor_raw;
        bool sensor_ready;
        SensorHealth sensor_health;

#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
        float angle_elec_redundant_sensor;
        float angle_mech_redundant_sensor;
        int32_t redundant_sensor_raw;
        bool redundant_sensor_ready;
        SensorHealth redundant_sensor_health;
        float rotor_redundancy_error;
#endif

#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
        float angle_mech_output_sensor;
        int32_t output_sensor_raw;
        bool output_sensor_ready;
        SensorHealth output_sensor_health;
        float transmission_deflection_rad;
#endif

        /* [D] FOC 中间量 */
        float i_alpha_raw, i_beta_raw;
        float i_alpha, i_beta;
        MotorSingleShuntSamplePlan single_shunt_sample_plan;
        MotorCurrentSampleSchedule current_sample_schedule;
        float i_d, i_q;
        float v_d, v_q;
        float v_alpha, v_beta;

        /* [E] 控制目标 */
        float target_rpm;
        float target_iq;
        float target_id;
        float speed_ref_limited;
        float speed_pid_iq;
        float iq_ref_command;
        float iq_ref_limited;
        float pending_reverse_rpm = 0.0f;   // 跨零软减速暂存反向目标 (本轮新增, 0=无待执行)

        /* === Phase 2: 位置/前馈通道 ===
         * target_pos_rad: 位置目标 (rad), 由 writeSetpoint(mode=POSITION_CONTROL, pos_ref=...) 落地
         * vel_ff_rad_s:   速度前馈 (rad/s), 由 MotionSetpoint.vel_ff 落地
         * last_setpoint_tick: 距上次 writeSetpoint 的 tick 数, 由 StreamFSM::step 消费判 HELD 超时
         *
         * BuildCfg 闸控:
         *   LIB_MOTOR_ENABLE_POSITION_CONTROL: 适用 target_pos_rad
         *   LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD: 适用 vel_ff_rad_s
         *   LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE: 适用 last_setpoint_tick
         */
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
        float target_pos_rad;
#endif
#if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
        float vel_ff_rad_s;
#endif
#if LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE
        uint32_t last_setpoint_tick;
#endif

        /* === Phase 3 占位: 力矩前馈通道 === */
#if LIB_MOTOR_ENABLE_TORQUE_FEEDFORWARD
        float torque_ff_a;
#endif

        /* === Phase 3 占位: 阻抗控制 ===
         * IMPEDANCE 字段在本 Phase 仅写入 ctx_, 控制环不消费;
         * 切到 IMPEDANCE_CONTROL 的电机启动会被 MotorFeatureRegistry::supportsMode 拒绝
         * (Phase 2 该函数仍对 IMPEDANCE 返回 false)。Phase 3 接通 runImpedanceControl()。
         */
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
        float imp_pos_ref_rad;
        float imp_stiffness_n_m;
        float imp_damping_n_m_s;
        float imp_torque_bias_a;
        float imp_torque_limit_a;
#endif

        /* [F] 最终输出 */
        float duty_a, duty_b, duty_c;

        /* [G] 计数器 */
        uint32_t fsm_timer_ticks;
        uint16_t slow_loop_counter;
        uint32_t silent_calib_timer_ticks;
        bool     silent_calib_active;

        /* 调试专用配置 */
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
        float vf_duty_bias;
        float test_u, test_v, test_w;
        float debug_ramp_time_s;
        const MotorVFStartupProfile* debug_vf_profile;
#if LIB_MOTOR_ENABLE_IF_STARTUP
        const MotorIFStartupProfile* debug_if_profile;
#endif
        uint8_t debug_profile_phase;
        float debug_profile_phase_time_s;
        float debug_profile_start_speed_rpm;
        float debug_profile_start_duty;
#if LIB_MOTOR_ENABLE_IF_STARTUP
        float debug_profile_start_id;
        float debug_profile_start_iq;
#endif
        bool debug_profile_phase_started;
#endif
#if LIB_MOTOR_ENABLE_IF_STARTUP
        uint8_t if_profile_phase;
        float if_profile_phase_time_s;
        float if_profile_start_speed_rpm;
        float if_profile_start_id;
        float if_profile_start_iq;
        bool if_profile_phase_started;
#endif
    } ctx_;

    FastRuntimeCtx fast_ctx_;

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

    /* ======================== 主控制链 ======================== */
    void updateSensorMeasurements();
    void updatePositionSensorMeasurement();
    void runSafetyCheck();
    void runObserverLoop();
    void runControlLoop();
    void updateCurrentPidVoltageLimit(float omega_elec_rad_s);

    float computeIqReference();
    void  syncFastRuntimeInputs(float id_ref, float iq_ref);
    void  syncFastRuntimeTargets();
    void  publishFastRuntimeOutputs();
    void  applySetpoint(const MotionSetpoint& sp);
    void  resetSetpointPlayback();
    void  serviceSetpointPlayback();
    Result appendSetpointPlaybackSegment(const MotionSetpoint* seg, uint16_t count);
    Result triggerSetpointPlayback(PlaybackTrigger trig);
    Result abortSetpointPlayback();
    Result validateRunPolicy(const MotorRunPolicy& policy) const;
    void  updateStallProtection(float speed_error, float iq_ref, float iq_limit);
    void  processPendingStallRestart();
    void  clearControlTargets();
    void  clearDerivedTargets();
    void  clearStallState();
    void  prepareSingleShuntSampling();
    void  disableSingleShuntSamplingSchedule();
    void  transitionToStop(StopMode mode, bool clear_user_targets, bool reset_rl_identify = true);
    void  prepareModeTransition(Mode previous_mode, Mode next_mode);

#if MOTOR_BUILD_NTC_SLOTS > 0
    float calculateNTC(uint16_t raw_adc, uint8_t ntc_index);
#endif
    bool  checkConfigValid(const MotorConfig& cfg) const;
    bool  validateControlFeedbackSource();
    void  setFault(Fault fault);
    void  stopForFault(Fault fault);

    /* ======================== 观测器 / 传感器 ======================== */
#if LIB_MOTOR_ENABLE_SMO
    SlidingModeObserver smo_;
#endif
#if LIB_MOTOR_ENABLE_HFI
    HighFreqInjectionObserver hfi_;
#endif
    SensorObserver rotor_sensor_obs_;
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    SensorObserver redundant_sensor_obs_;
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    SensorObserver output_sensor_obs_;
#endif

    /* ======================== 启动辅助 ======================== */
#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    AngleGenerator if_angle_gen_;
#endif

    /* ======================== 调试辅助 ======================== */
#if LIB_MOTOR_ENABLE_IF_STARTUP
    float getEffectiveDragCurrent() const;
    void resetIFStartupProfileState();
    bool getIFStartupProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref);
#endif
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    void resetDebugProfileState();
    bool getDebugVFProfileCommand(float& speed_rpm, float& duty);
#if LIB_MOTOR_ENABLE_IF_STARTUP
    bool getDebugIFProfileCommand(float& speed_rpm, float& id_ref, float& iq_ref);
#endif
#endif
};

} // namespace Lib_Motor
