/*
 * Motor_StreamFSM.h — setpoint 流临停锁存管理 FSM 声明
 *
 * 职责:
 *   StreamFSM 描述"writeSetpoint 流"在异常下的临停状态, 不再持有任务概念。
 *   它在 FSM 调度管线中执行, 确保在 Safety/Lifecycle/Command 全部更新后,
 *   再根据系统状态变化将 ACTIVE 流冻结为 SUSPENDED_BY_FAULT / SUSPENDED_BY_USER。
 *
 * 状态 (StreamState):
 *   NONE               — 上层从未写入过 setpoint
 *   ACTIVE             — 上层正在每拍 writeSetpoint 跟随中
 *   SUSPENDED_BY_FAULT — 故障打断, 上一拍 reference 已冻结
 *   SUSPENDED_BY_USER  — stop()/emergencyStop 主动冻结流
 *   HELD               — 上层断流超时, 自动退到位置保持 (留待 Phase 2)
 *   FAULTED_LATCHED    — 旧任务语义残留转义的终态保留值, 与 SUSPENDED_BY_FAULT 等价
 */

#pragma once

namespace Lib_Motor
{

class MotorManager;

class Motor_StreamFSM
{
public:
    /*
     * step — 响应系统状态变化更新流临停锁存
     *
     * 规则:
     *   1. 无流 (NONE) → 不处理
     *   2. 系统故障 (ERROR 或 fault_ 非零):
     *      ACTIVE → SUSPENDED_BY_FAULT (流被故障冻结, 待 clearFault 恢复)
     *   3. 停机请求 (command_state == STOP_REQUESTED):
     *      ACTIVE → SUSPENDED_BY_USER (上层主动停止, 不释放 reference 锁存)
     */
    static void step(MotorManager& m);

    static void clear(MotorManager& m);
};

} // namespace Lib_Motor