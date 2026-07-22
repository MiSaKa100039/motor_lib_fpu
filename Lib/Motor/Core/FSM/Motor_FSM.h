/*
 * Motor_FSM.h — 电机顶层 FSM 门面
 *
 * Motor_FSM 负责两类职责：
 *
 *   1. 周期调度
 *      在每个控制 tick 内，按固定顺序推进顶层状态机管线。
 *
 *   2. 顶层门面
 *      对 MotorManager 暴露统一入口，由它转发到具体子 FSM，
 *      避免 MotorManager 直接依赖大量子 FSM 头文件与实现细节。
 *
 * 约束:
 *   - MotorManager 对子 FSM 的调用，应优先通过 Motor_FSM 收口
 *   - 子 FSM 之间的内部协作，仍可在 FSM 层内部直接发生
 */

#pragma once

#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorManager;

class Motor_FSM
{
public:
    /* 每个控制 tick 推进一次顶层 FSM 调度管线 */
    static void step(MotorManager& m);

    /* 命令入口门面 */
    static void selectMode(MotorManager& m, Mode mode);
    static void requestStart(MotorManager& m);
    static void requestStop(MotorManager& m, StopMode mode);
    static void requestEmergencyStop(MotorManager& m);

    /* 故障/恢复门面 */
    static void clearFaultBitsNow(MotorManager& m);
    static void clearFaultAndStopNow(MotorManager& m);
    static void latchFault(MotorManager& m, Fault fault);

    /* 运行辅助门面 */
    static void runObserverStep(MotorManager& m);
    static float limitIqReferenceByEnergy(MotorManager& m,
                                          float requested_iq,
                                          float speed_rpm);
    static bool validateClosedLoopEntry(MotorManager& m);
};

} // namespace Lib_Motor
