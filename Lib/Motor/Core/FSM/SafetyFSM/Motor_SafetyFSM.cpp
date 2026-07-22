/*
 * Motor_SafetyFSM.cpp — 安全/故障 FSM 实现
 *
 * 在 Motor_FSM::step() 管线中的位置: 第 ① 步 (最先执行)
 * 调用链: MotorManager::tick() → Motor_FSM::step() → Motor_SafetyFSM::step()
 *
 * 职责边界:
 *   - 监控 m.fault() 故障掩码 (已由 runSafetyCheck 填充)
 *   - 有故障→关断, 无故障→保持 CLEAR
 *   - 不负责故障检测 (检测在 MotorManager::runSafetyCheck 中完成)
 *   - 不负责故障清除后自动恢复 (需用户手动 clearFault)
 *
 * SafetyState 转换:
 *   ┌─────────┐  fault() != NONE  ┌────────────────┐
 *   │  CLEAR  │ ────────────────→ │ FAULT_LATCHED   │  (普通故障: 过流/过压/过温/传感器丢失...)
 *   └─────────┘                   │ or              │
 *       ↑                         │ ESTOP_LATCHED   │  (紧急停止: Fault::EMERGENCY_STOP)
 *       │       clearLatchedFault └────────┬───────┘
 *       └─────────────────────────────────┘
 */

#include "Motor_SafetyFSM.h"

#include "../../Manager/Motor_Manager.h"

namespace
{

bool containsFault(Lib_Motor::Fault fault, Lib_Motor::Fault flag)
{
    return (static_cast<uint16_t>(fault) & static_cast<uint16_t>(flag)) != 0U;
}

bool isConfigurationFault(Lib_Motor::Fault fault)
{
    return containsFault(fault, Lib_Motor::Fault::PARAM_ERROR) ||
           containsFault(fault, Lib_Motor::Fault::SENSOR_CONFIG_ERROR) ||
           containsFault(fault, Lib_Motor::Fault::OBSERVER_UNAVAILABLE);
}

}

namespace Lib_Motor
{

/*
 * step — 每个 tick 的故障门控入口
 *
 * 这是 FSM 调度管线中最先执行的模块。设计理由:
 *   - 在任何业务逻辑执行前先拦截故障, 避免对故障态电机执行控制算法
 *   - 如果本 tick 检测到故障, LifecycleFSM 下一个 tick 会发现 state_=ERROR,
 *     从而进入 handleError (不做任何操作), 直到用户手动清除
 *
 * SafetyState 与 fault_ 的关系:
 *   - fault_ 是"是否检测到过故障"的位掩码 (由 runSafetyCheck 设置)
 *   - SafetyState 是"当前安全状态机处于什么阶段"的枚举
 *   - 即使 fault_ 中的故障条件已消失 (如过流已结束),
 *     只要 fault_ 掩码未清零, SafetyState 就保持 LATCHED
 */
void Motor_SafetyFSM::step(MotorManager& m)
{
    if (m.fault() == Fault::NONE)
    {
        m.safety_state_ = SafetyState::CLEAR;   // 无故障 → 保持清除态
        return;
    }

    enterFaultLatched(m);                        // 有故障位 → 进入锁存流程
}

/*
 * latchFault — 由外部调用者 (子 FSM / MotorAPI) 主动触发故障锁存
 *
 * 与 step() 的区别:
 *   - step() 是被动检测: 检查 fault_ 掩码是否已有故障位
 *   - latchFault() 是主动触发: 调用者先设故障位, 再立即执行关断
 *
 * 调用链:
 *   Motor_CommandFSM::requestEmergencyStop()
 *     → Motor_SafetyFSM::latchFault(m, EMERGENCY_STOP)
 *       → m.setFault(EMERGENCY_STOP)  // OR 合并到 fault_
 *       → enterFaultLatched(m)        // 立即关断
 */
void Motor_SafetyFSM::latchFault(MotorManager& m, Fault fault)
{
    m.setFault(fault);                           // OR 合并到 fault_ 掩码 (不覆盖已有故障)
    enterFaultLatched(m);                        // 立即执行关断
}

/*
 * clearLatchedFault — 清除故障锁存并恢复所有子 FSM 状态
 *
 * 调用时机:
 *   - 用户调用 MotorAPI::clearFault() (主动恢复)
 *   - 堵转自动重试前的 clear_fault_before_restart (自动恢复)
 *
 * 重置各子 FSM 状态的原因:
 *   fault_ = NONE          → 核心: 清除故障码, SafetyFSM 恢复 CLEAR
 *   command_state = IDLE   → 命令队列恢复, 下次可接受新命令
 *   task_state = NONE      → 清除故障期间被 FAULTED_LATCHED 的任务
 *   angle_state = NONE     → 角度源需重新选择 (可能传感器已恢复)
 *   stop_state = IDLE      → 制动状态归零, 避免残留状态影响下次停止
 *   energy_state = IDLE    → 能量状态归零
 */
void Motor_SafetyFSM::clearLatchedFault(MotorManager& m)
{
    m.fault_ = Fault::NONE;                      // 清除故障掩码
    m.safety_state_ = SafetyState::CLEAR;        // 安全状态恢复清除
    m.command_state_ = CommandState::IDLE;       // 命令状态恢复
    m.stream_state_ = StreamState::NONE;         // 流状态恢复
    m.angle_state_ = AngleState::NONE;           // 角度源状态恢复
    m.stop_state_ = StopState::IDLE;             // 停机状态恢复
    m.energy_state_ = EnergyState::IDLE;         // 能量状态恢复
}

/*
 * ignoresFaultTransition — 故障关断白名单
 *
 * 某些状态下故障位可能被置位, 但不应该立即关断:
 *
 *   State::INIT:
 *     - 硬件 (ADC/PWM/GPIO) 尚未初始化完毕
 *     - 此时读取的 ADC 值为随机值 → 可能触发过流/过压假故障
 *     - 如果此时关断 PWM, 会导致 MotorManager::init() 中的 ADC 在校准完成前就失败
 *
 *   State::ADC_CAL:
 *     - 正在进行 ADC 零点偏移校准
 *     - 此时 PWM 未输出, 电流应为零, 但由于校准中 offset 未稳定 → 可能假过流
 *     - 校准完成进入 STOP 后, 如果故障仍在, 下一个 tick 会立即被 step() 捕获
 *
 *   State::ERROR:
 *     - 已经在故障态, PWM 已关断
 *     - 再次调用 enterFaultLatched 只会重复清零状态 ← 无意义但无害
 *     - 白名单化可避免冗余代码路径
 */
bool Motor_SafetyFSM::ignoresFaultTransition(State state, Fault fault)
{
    // INIT: 硬件尚未初始化完成 / ADC_CAL: 自校准中 / ERROR: 已在故障态
    if (state == State::ERROR)
    {
        return true;
    }

    if (state == State::INIT || state == State::ADC_CAL)
    {
        return !isConfigurationFault(fault);
    }

    return false;
}

/*
 * enterFaultLatched — 故障锁存的核心执行逻辑
 *
 * 这是 SafetyFSM 的最底层, 完成从"检测到故障"到"电机完全停止"的全部操作。
 *
 * ╔════════════════════════════════════════════════════════════╗
 * ║  进入条件: fault_ 掩码中至少有一个故障位被置位            ║
 * ║                                                            ║
 * ║  [1] 判断故障类型: ESTOP 位 → ESTOP_LATCHED               ║
 * ║                    其他     → FAULT_LATCHED                ║
 * ║                                                            ║
 * ║  [2] 白名单检查: 若当前处于 INIT/ADC_CAL/ERROR → 直接返回 ║
 * ║                                                            ║
 * ║  [3] 清零所有运行状态变量 (见下方详细注释)                 ║
 * ║                                                            ║
 * ║  [4] 清零控制派生目标 (speed_ref/iir_ref_* = 0)            ║
 * ║                                                            ║
 * ║  [5] 取消待定自动重试 (堵转恢复被中断)                     ║
 * ║                                                            ║
 * ║  [6] 软件端清零三相占空比                                  ║
 * ║                                                            ║
 * ║  [7] 硬件端关闭 PWM 输出 (MOE=0, Hi-Z)                     ║
 * ║                                                            ║
 * ║  [8] 推送 State::ERROR → LifecycleFSM 下次进入 handleError ║
 * ╚════════════════════════════════════════════════════════════╝
 */
void Motor_SafetyFSM::enterFaultLatched(MotorManager& m)
{
    // ── [1] 区分紧急停止和普通故障 ──
    // ESTOP (紧急停止) 与普通 FAULT 的区别:
    //   - ESTOP: 用户主动触发, 电机应立即以最安全方式停止
    //   - FAULT: 系统检测到异常, 电机被动进入保护
    // 两者最终效果相同 (关断 PWM), 但 SafetyState 区分以便上位机区分原因
    const bool estop = ((static_cast<uint16_t>(m.fault()) &
                         static_cast<uint16_t>(Fault::EMERGENCY_STOP)) != 0U);
    m.safety_state_ = estop ? SafetyState::ESTOP_LATCHED      // 紧急停止 → ESTOP_LATCHED
                            : SafetyState::FAULT_LATCHED;     // 普通故障 → FAULT_LATCHED

    // ── [2] 白名单检查 ──
    if (ignoresFaultTransition(m.state(), m.fault()))
    {
        return;                              // INIT/ADC_CAL/ERROR 状态下不执行关断
    }

    // ── [3] 清零全部运行状态 ──
    // 必须清零全部子 FSM 状态: 如果不清零,
    // 下次 clearFault 后这些残留值可能导致电机意外自动重启或模式错乱
    m.run_requested_ = false;                // 不再请求运行
    m.mode_ = Mode::NONE;                    // 模式归零
    m.run_phase_ = RunPhase::NONE;           // 运行子阶段归零
    m.command_state_ = CommandState::FAULTED; // 命令状态 → 已故障 (外部 API 可见)
    if (m.stream_state_ == StreamState::ACTIVE)
    {
        m.stream_state_ = StreamState::SUSPENDED_BY_FAULT;  // 活跃流 → 故障冻结
    }
    m.angle_state_ = AngleState::NONE;       // 角度源状态归零
    m.stop_state_ = StopState::IDLE;         // 停机状态归零
    m.energy_state_ = EnergyState::IDLE;     // 能量状态归零

    // ── [4] 清零控制目标 ──
    // 清零派生目标而非用户目标:
    //   保留 ctx_.target_rpm / target_iq / target_id — 这些是用户设置的值,
    //   下次用户手动 clearFault 后可以继续使用原目标重启
    //   清零 speed_ref_limited / iq_ref_command / iq_ref_limited — 这些是
    //   运行时根据 target 计算的衍生值, 必须清零防止下次瞬间跳变
    m.clearDerivedTargets();

    // ── [5] 取消待定自动重试 ──
    // 如果电机正在堵转自动重试的等待期间发生故障,
    // 取消重试流程, 避免故障清除后意外自动重启
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    m.stall_restart_pending_ = false;
    m.stall_restart_wait_ticks_ = 0U;
#endif

    // ── [6] 软件端清零 PWM 占空比 ──
    // 必须在硬件关断前先清零软件端, 防止 MotorManager 在同一个 tick
    // 的后续步骤 (如 runControlLoop) 中写入新的 duty 值到硬件
    m.ctx_.duty_a = 0.0f;
    m.ctx_.duty_b = 0.0f;
    m.ctx_.duty_c = 0.0f;

    // ── [7] 硬件端关断 PWM 输出 ──
    // 调用 BSP 层 pwm_disable() → MOE=0, 三相输出 Hi-Z
    // 此时即使软件端 write PWM CCR, 硬件也不会输出 (MOE 已关)
    if (m.config_.hal != nullptr && m.config_.hal->pwm_disable != nullptr)
    {
        m.config_.hal->pwm_disable();        // MOE=0, 三相 Hi-Z
    }

    // ── [8] 推送 ERROR 状态 ──
    // 只有推送 ERROR 后, 下一个 tick 的 LifecycleFSM::step() 才会进入 handleError,
    // CommandFSM::step() 才会返回 FAULTED, TaskFSM::step() 才会锁存未完成的任务
    m.setState(State::ERROR);
}

} // namespace Lib_Motor
