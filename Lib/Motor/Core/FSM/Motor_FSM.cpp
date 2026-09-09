/*
 * Motor_FSM.cpp — 电机顶层 FSM 门面实现
 *
 * 这里一方面负责周期 step() 调度，另一方面负责把 Manager 侧的
 * 子 FSM 访问统一收口到顶层门面中。
 */

#include "Motor_FSM.h"

#include "../Manager/Motor_Manager.h"
#include "AngleFSM/Motor_AngleFSM.h"
#include "CommandFSM/Motor_CommandFSM.h"
#include "LifecycleFSM/Motor_LifecycleFSM.h"
#include "SafetyFSM/Motor_SafetyFSM.h"
#include "StopFSM/Motor_StopFSM.h"
#include "StreamFSM/Motor_StreamFSM.h"
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
#include "EnergyFSM/Motor_EnergyFSM.h"
#endif

namespace Lib_Motor
{

void Motor_FSM::step(MotorManager& m)
{
    Motor_SafetyFSM::step(m);       // ① 安全门控：故障拦截 + 关断
    Motor_LifecycleFSM::step(m);    // ② 生命周期：顶层状态推进
    Motor_CommandFSM::step(m);      // ③ 命令状态：生命周期状态 -> CommandState
    Motor_StreamFSM::step(m);       // ④ 流临停：故障/停机后的 setpoint 流冻结
}

void Motor_FSM::selectMode(MotorManager& m, Mode mode)
{
    Motor_CommandFSM::selectMode(m, mode);
}

void Motor_FSM::requestStart(MotorManager& m)
{
    Motor_CommandFSM::requestStart(m);
}

void Motor_FSM::requestStop(MotorManager& m, StopMode mode)
{
    Motor_CommandFSM::requestStop(m, mode);
}

void Motor_FSM::requestEmergencyStop(MotorManager& m)
{
    Motor_CommandFSM::requestEmergencyStop(m);
}

void Motor_FSM::clearFaultBitsNow(MotorManager& m)
{
    /*
     * clearLatchedFault() 已负责清除 fault_ 以及基础子状态。
     * 这里继续显式调用各 clear()，是为了保留子 FSM 自身的附加清理逻辑。
     */
    Motor_SafetyFSM::clearLatchedFault(m);
    Motor_AngleFSM::clear(m);
    Motor_StopFSM::clear(m);
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
    Motor_EnergyFSM::clear(m);
#else
    m.energy_state_ = EnergyState::IDLE;
#endif
    Motor_StreamFSM::clear(m);
}

void Motor_FSM::clearFaultAndStopNow(MotorManager& m)
{
    clearFaultBitsNow(m);
    m.transitionToStop(StopMode::COAST, true);
    step(m); // 同步推进一次顶层 FSM 管线，完成 STOPPING -> STOP
}

void Motor_FSM::latchFault(MotorManager& m, Fault fault)
{
    Motor_SafetyFSM::latchFault(m, fault);
}

void Motor_FSM::runObserverStep(MotorManager& m)
{
    Motor_AngleFSM::step(m);
}

float Motor_FSM::limitIqReferenceByEnergy(MotorManager& m,
                                          float requested_iq,
                                          float speed_rpm)
{
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
    return Motor_EnergyFSM::limitIqReference(m, requested_iq, speed_rpm);
#else
    if (requested_iq == 0.0f)
    {
        m.energy_state_ = EnergyState::IDLE;
        return 0.0f;
    }
    if ((requested_iq * speed_rpm) < 0.0f)
    {
        m.energy_state_ = EnergyState::BLOCKED;
        return 0.0f;
    }
    m.energy_state_ = EnergyState::DRIVE;
    return requested_iq;
#endif
}

bool Motor_FSM::validateClosedLoopEntry(MotorManager& m)
{
    return Motor_AngleFSM::validateClosedLoopEntry(m);
}

} // namespace Lib_Motor
