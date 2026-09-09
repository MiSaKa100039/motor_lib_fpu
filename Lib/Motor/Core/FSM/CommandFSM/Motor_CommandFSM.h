/*
 * Motor_CommandFSM.h — 命令队列 FSM 声明
 *
 * 职责:
 *   CommandFSM 管理外部命令与电机状态的映射。
 *   它在 Motor_FSM 调度管线中第三位执行 (Safety → Lifecycle → Command → Task),
 *   将当前电机状态翻译为 CommandState, 供外部 API 查询和决策。
 *
 * 命令状态 (CommandState):
 *   IDLE              — 空闲, 等待命令
 *   START_REQUESTED   — 已请求启动 (run_requested_=true, 等待 LifecycleFSM 处理)
 *   ACTIVE            — 正在运行 (对应 RUN 状态)
 *   STOP_REQUESTED    — 已请求停止 / 正在制动 (对应 STOPPING 状态)
 *   FAULTED           — 已故障 (对应 ERROR 状态)
 *
 * 命令接口 (由 MotorAPI 调用):
 *   selectMode     — 切换目标模式 (自动 prepareModeTransition)
 *   requestStart   — 请求启动 (设置 run_requested_=true)
 *   requestStop    — 请求停止 (记录 stop_mode_)
 *   requestEmergencyStop — 紧急停止 (清目标 + 锁 EMERGENCY_STOP 故障)
 */

#pragma once

#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorManager;

class Motor_CommandFSM
{
public:
    /*
     * step — 将当前电机状态映射为 CommandState
     *
     * 映射规则:
     *   ERROR / fault非零 → FAULTED
     *   STOPPING           → STOP_REQUESTED
     *   RUN                → ACTIVE
     *   STOP               → IDLE (无请求) 或 START_REQUESTED (有请求)
     */
    static void step(MotorManager& m);

    /*
     * selectMode — 选择目标模式
     * 自动调用 prepareModeTransition 做模式切换前的 PID 复位等预处理
     */
    static void selectMode(MotorManager& m, Mode mode);

    /*
     * requestStart — 请求启动
     * 设置 run_requested_=true, 由 LifecycleFSM 在下一个 tick 处理
     */
    static void requestStart(MotorManager& m);

    /*
     * requestStop — 请求停止 (指定制动方式)
     * 记录 stop_mode_ 供 transitionToStop 使用
     */
    static void requestStop(MotorManager& m, StopMode mode);

    /*
     * requestEmergencyStop — 紧急停止
     * 清除用户控制目标, 退出运行, 锁存 EMERGENCY_STOP 故障
     */
    static void requestEmergencyStop(MotorManager& m);

    static void clearCommand(MotorManager& m);
};

} // namespace Lib_Motor
