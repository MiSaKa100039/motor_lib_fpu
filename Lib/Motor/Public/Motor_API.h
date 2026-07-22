/*
 * Motor_API.h
 *
 * 用户侧电机控制接口:
 *   1. init(cfg) 绑定平台配置
 *   2. selectMode(mode) 选择目标模式 (TORQUE/VELOCITY/POSITION/IMPEDANCE)
 *   3. setTargetXxx(...) 直接覆盖目标值 (语法糖, 等价于单字段 writeSetpoint)
 *   4. writeSetpoint(...) 流式设置每控制拍设定值 (核心 API, Phase 2)
 *   5. appendSetpointSeg/triggerSegPlayback/abortSegPlayback -- 本地 FIFO/trigger 同步回放
 *   6. start()/stop() 控制运行与停止
 *
 * 说明:
 *   - 单轴 lib 不持"任务/队列"概念; writeSetpoint 每拍覆盖, 无完成判据
 *   - 多轴同步、轨迹插补由 lib 之上的 Coordinator 层完成 (参见 Templates/Coordinator/)
 *   - BuildCfg 决定哪些 Mode 编进固件; MotorCfg 决定本电机用哪个 Mode
 *   - setTargetXxx/writeSetpoint 仅在非 Debug/Calib 独占态下覆盖目标, 内部委托到 writeSetpoint
 *   - selectMode 只切目标模式, 不自动启动
 *   - start() 启动当前已选模式; 若当前目标模式为 NONE, 则回退到平台默认启动模式
 *   - debug/calib 模式必须走专用 API, 不允许通过 selectMode 进入
 *
 * 调用顺序约束 (Phase 2 重要):
 *   - writeSetpoint / setTargetXxx 仅写 ctx_ 控制目标, 不切换 mode_;
 *   - 上层必须先 selectMode(POSITION_CONTROL) → start() → 再 writeSetpoint;
 *   - 否则控制环按旧 mode_ 消费 ctx_, 位置环根本不会执行。
 *
 * 测试基础设施状态 (Phase 2 待补):
 *   - 当前 motor_lib 尚未引入 host 单元测试基础设施, 也未引入 on-target test 框架;
 *   - FSM 状态跳转、位置环方向、StreamState::HELD 路径、CommandGuard 入口门控等
 *     均依赖写代码时的逻辑推演 + VOFA 真硬件验证;
 *   - TODO HostTest 见 tools/host_test/README.md (待 Phase 2.6 引入 LLVM MinGW 后建);
 *   - 极端工况(瞬时过压、堵转-恢复、传感器断线) 需在真硬件做故障注入测试。
 */

#pragma once

#include "Motor_Config.h"
#include "Motor_Definitions.h"
#include "Motor_Types.h"

namespace Lib_Motor
{

class MotorManager;

/* 全局实例池，便于 ISR 按 motor_id 分发 tick() */
extern MotorManager* g_motor_pool[MOTOR_MAX_INSTANCES];

class MotorAPI
{
public:
    MotorAPI();
    ~MotorAPI();

    MotorAPI(const MotorAPI&) = delete;
    MotorAPI& operator=(const MotorAPI&) = delete;
    MotorAPI(MotorAPI&&) = delete;
    MotorAPI& operator=(MotorAPI&&) = delete;

    /* ==================== 生命周期 ==================== */
    Result init(const MotorConfig& cfg);
    Result deInit();

    /* ==================== 运行控制 ==================== */
    Result start();
    Result stop(StopMode mode = StopMode::COAST);
    Result emergencyStop();
    Result clearFault();
    Result reset();

    /* ==================== 目标值设置 ==================== */
    Result setTargetTorque(float current_a);          // 等价于 setpoint.mode=TORQUE, torque_ff=current
    Result setTargetSpeed(float rpm);                 // 等价于 setpoint.mode=VELOCITY, vel_ff=rpm
    Result setTargetPosition(float angle_rad);        // 等价于 setpoint.mode=POSITION, pos_ref=angle

    /* ==================== 流式设定值 (核心 API) ==================== */
    /* 每控制周期由上层协调器写入; 上拍覆盖, 无任务, 无完成判据 */
    Result writeSetpoint(const MotionSetpoint& sp);

    /* ==================== 本地 FIFO/trigger 同步回放 ====================
     * 用于多 MCU 分布式架构: 主机协调器把"未来 N 拍"的 setpoint 段预加载到本地 lib FIFO,
     * 收到 SYNC 信号后从 MCU 与本地 lib 同时开始按 tick 消费。
     *
     * BuildCfg 未开启时返回 Result::NotSupported; 开启后使用固定容量本地 FIFO。
     * G-code、协议帧、ACK、水位和主从同步策略仍由 lib 外层负责。
     *
     * 调用序参见 Templates/Coordinator/README.md。
     */
    Result appendSetpointSeg(const MotionSetpoint* seg, uint16_t count);
    Result triggerSegPlayback(PlaybackTrigger trig);
    Result abortSegPlayback();

    /* ==================== 模式选择 ==================== */

    Result selectMode(Mode mode);

    /* ==================== 运行时参数 ==================== */
#if LIB_MOTOR_ENABLE_IF_STARTUP
    Result setDragCurrent(float A);
#endif
    Result setControlMethod(ControlMethod method);
    Result setModulation(ModulationMethod mod);
    Result setPIDGains(PidGroup group, float kp, float ki, float kd);
    Result setRunPolicy(const MotorRunPolicy& policy);
    Result getRunPolicy(MotorRunPolicy& policy) const;
    Result setEnergyBudget(const MotorEnergyBudget& budget);

    /* ==================== 校准 ==================== */

    Result startRLIdentification();

    /* ==================== 监视 ==================== */
    void getMonitorData(MotorMonitorData& out_data) const;
    void getDebugData(MotorDebugData& out_data) const;

    State getState() const;
    Fault getFault() const;
    MotorConfigFaultDetail getConfigFaultDetail() const;
    float getSpeed() const;

    /* === [本轮新增] 诊断 getter === */

    /* 把 last_config_fault_detail 翻译成可读字符串;
     * BuildCfg 关闭 LIB_MOTOR_ENABLE_HUMAN_READABLE_FAULT_DETAIL 时返回空串,
     * 调用方可自行用裸 detail 值查表。 */
    const char* getConfigFaultDetailString() const;

    /* 堵转自动重试计数器 (已经历第几次重试) */
    uint8_t getStallRetryCounter() const;

    /* SMO 收敛后辨识的 Ke (V/(rad/s)), 未辨识过 0; 也可由 physical.ke_v_per_rad_s_identified 直接读 */
    float getKEstimated() const;

    /* [P2] RL 辨识是否完成 (event_.rl_identify_done == 1) */
    bool isRLIdentifyDone() const;
    /* [P2] Ke 辨识 (startKEstimation 触发的 API 流程) 是否完成 */
    bool isKEstimationDone() const;
    /* [P2] 顺逆风启动是否完成 (event_.flying_start_done == 1) */
    bool isFlyingStartDone() const;
    /* [P2] 触发 Ke 辨识: 让电机在 RUN+SMO_ONLY 下采样 200ms 滑窗写 ke_v_per_rad_s_identified */
    Result startKEstimation();
    /* [P2] 触发顺逆风启动: 若 |ω|>threshold 则尝试 PHASE_VOLTAGE 相电压 PLL 重构, 失败 fallback 到 ALIGNMENT */
    Result triggerFlyingStartIfSpinning();

    /* 一次性拿 identified 电机参数 (rs/ls/ke), AutoTune 后填充 */
    struct MotorLearnedParamsSnapshot
    {
        float rs_ohm;
        float ls_h;
        float ke_v_per_rad_s;
        bool  valid;
    };
    MotorLearnedParamsSnapshot getLearnedMotorParams() const;

    /* ==================== 安全锁 ==================== */
    void lockDebug();
    void unlockDebug();

#ifdef ENABLE_DANGEROUS_TEST_API
    /* ==================== 调试专用 ==================== */
    Result debugPWMManual(float u, float v, float w);
    Result debugCurrentLock(float i_d, float i_q);
#if LIB_MOTOR_ENABLE_IF_STARTUP
    Result debugIFControl(float target_rpm, float id_amp, float iq_amp, float ramp_time_s);
    Result debugIFControl(const MotorIFStartupProfile& profile);
#if LIB_MOTOR_ENABLE_SMO
    Result debugIFSMOObserver(const MotorIFStartupProfile& profile);
#endif
#if LIB_MOTOR_ENABLE_HFI
    Result debugIFHFIObserver(const MotorIFStartupProfile& profile);
#endif
#endif
#if LIB_MOTOR_ENABLE_HFI
    Result debugHFIObserver();
#endif
    Result debugVFControl(float duty, float speed_rpm, float ramp_time_s);
    Result debugVFControl(const MotorVFStartupProfile& profile);
#endif

private:
    static bool isDebugOrCalib(Mode m);
    Result checkSafeModeChange(Mode new_mode);

    int id_ = -1;
    bool debug_locked_ = false;
    Fault last_fault_ = Fault::NONE;
    MotorConfigFaultDetail last_config_fault_detail_ = MotorConfigFaultDetail::NONE;
};

} // namespace Lib_Motor

extern "C" void Motor_Global_Process_Handler(int motor_id);
