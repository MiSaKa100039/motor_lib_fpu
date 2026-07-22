/*
 * Motor_StreamFSM.cpp — setpoint 流临停锁存管理 FSM 实现
 */

#include "Motor_StreamFSM.h"

#include "../../Manager/Motor_Manager.h"

namespace Lib_Motor
{

/*
 * step — 响应系统状态变化, 流被故障/停机冻结
 */
void Motor_StreamFSM::step(MotorManager& m)
{
    /* TODO HostTest: ACTIVE→HELD 超时转换、SUSPENDED_BY_FAULT/SUSPENDED_BY_USER 切换
     *                未编单测, 当前依赖 VOFA 真硬件验证 (BuildCfg 默认 HOLD 关闭时本段不编) */
    if (m.stream_state_ == StreamState::NONE)
    {
        return;
    }

#if LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE
    /* [H1] 上层断流超时判据: 距上次 writeSetpoint 超过 hold_timeout_ticks */
    if (m.stream_state_ == StreamState::ACTIVE &&
        m.config_.motion.hold_timeout_ticks > 0U &&
        (m.fsmTimerTicks() - m.ctx_.last_setpoint_tick) > m.config_.motion.hold_timeout_ticks)
    {
        /* 冻结最后一拍的 pos_ref 为位置目标 (hold_when_idle=true 时)。
         * Phase 2 仅完成状态切换; Phase 3 在此把 ctx_.target_pos_rad 再补一次冻结也不迟。
         * 当前行为: 写入新设置由上层在恢复 writeSetpoint 时覆盖, lib 不需主动写 ctx_。
         */
        m.stream_state_ = StreamState::HELD;
        return;
    }
#endif

    if (m.state() == State::ERROR || m.fault() != Fault::NONE)
    {
        if (m.stream_state_ == StreamState::ACTIVE)
        {
            m.stream_state_ = StreamState::SUSPENDED_BY_FAULT;
        }
        return;
    }

    if (m.command_state_ == CommandState::STOP_REQUESTED &&
        m.stream_state_ == StreamState::ACTIVE)
    {
        m.stream_state_ = StreamState::SUSPENDED_BY_USER;
    }
}

void Motor_StreamFSM::clear(MotorManager& m)
{
    m.stream_state_ = StreamState::NONE;
}

} // namespace Lib_Motor