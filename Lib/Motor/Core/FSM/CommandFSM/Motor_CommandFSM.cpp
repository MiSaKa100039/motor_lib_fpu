/*
 * Motor_CommandFSM.cpp — 命令队列 FSM 实现
 */

#include "Motor_CommandFSM.h"

#include "../../Manager/Motor_Manager.h"
#include "../SafetyFSM/Motor_SafetyFSM.h"

namespace Lib_Motor
{

/*
 * step — 将当前电机状态映射为 CommandState
 *
 * 映射规则:
 *   ERROR 或 fault非零 → FAULTED (故障优先)
 *   STOPPING           → STOP_REQUESTED (正在制动)
 *   RUN                → ACTIVE (正在运行)
 *   STOP               → run_requested_? START_REQUESTED : IDLE
 *   其他               → IDLE (兜底)
 */
void Motor_CommandFSM::step(MotorManager& m)
{
    if (m.state() == State::ERROR || m.fault() != Fault::NONE)
    {
        m.command_state_ = CommandState::FAULTED;
        return;
    }

    if (m.state() == State::STOPPING)
    {
        m.command_state_ = CommandState::STOP_REQUESTED;
        return;
    }

    if (m.state() == State::RUN)
    {
        m.command_state_ = CommandState::ACTIVE;
        return;
    }

    if (m.state() == State::STOP)
    {
        m.command_state_ = m.run_requested_ ? CommandState::START_REQUESTED : CommandState::IDLE;
        return;
    }

    m.command_state_ = CommandState::IDLE;              // 兜底
}

/*
 * selectMode — 切换目标模式
 * 若与当前 target_mode_ 相同则跳过;
 * 切换前调用 prepareModeTransition 做前置处理 (如 PID 复位)
 */
void Motor_CommandFSM::selectMode(MotorManager& m, Mode mode)
{
    if (mode == m.target_mode_)
    {
        return;                                         // 未变化, 跳过
    }

    const Mode previous_mode = (m.mode_ != Mode::NONE) ? m.mode_ : m.target_mode_;
    m.prepareModeTransition(previous_mode, mode);       // 预处理 (调试模式进入时复位 PID)
    m.target_mode_ = mode;
}

/*
 * requestStart — 请求启动
 * 仅当 target_mode_ 不是 NONE 时才允许启动
 */
void Motor_CommandFSM::requestStart(MotorManager& m)
{
    if (m.target_mode_ == Mode::NONE)
    {
        return;                                         // 未选择模式, 拒绝启动
    }

    m.run_requested_ = true;
    m.command_state_ = CommandState::START_REQUESTED;
}

/*
 * requestStop — 请求停止
 * 记录制动方式到 stop_mode_, 供 LifecycleFSM::handleRun → transitionToStop 使用
 */
void Motor_CommandFSM::requestStop(MotorManager& m, StopMode mode)
{
    m.stop_mode_ = mode;
    m.run_requested_ = false;
    m.command_state_ = CommandState::STOP_REQUESTED;
}

/*
 * requestEmergencyStop — 紧急停止
 *
 * 与 requestStop 的区别:
 *   - 清除用户控制目标 (target_rpm/target_iq/target_id = 0)
 *   - 锁存 EMERGENCY_STOP 故障位 (不可恢复, 需手动清除)
 *   - SafetyFSM 检测到 EMERGENCY_STOP 位后会标记 ESTOP_LATCHED
 */
void Motor_CommandFSM::requestEmergencyStop(MotorManager& m)
{
    m.clearControlTargets();                             // 清零控制目标
    m.run_requested_ = false;
    m.command_state_ = CommandState::FAULTED;
    Motor_SafetyFSM::latchFault(m, Fault::EMERGENCY_STOP);
}

void Motor_CommandFSM::clearCommand(MotorManager& m)
{
    m.command_state_ = CommandState::IDLE;
}

} // namespace Lib_Motor
