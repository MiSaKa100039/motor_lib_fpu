/*
 * Motor_SafetyFSM.h — 安全/故障 FSM 声明
 *
 * 职责:
 *   SafetyFSM 是 FSM 调度管线中第一个执行的门控模块。
 *   它监控 fault_ 故障掩码, 一旦检测到任何故障位被置位,
 *   立即执行硬件关断 + 状态清零, 并将电机推入 ERROR 状态。
 *
 * 状态:
 *   CLEAR          — 无故障, 正常运行
 *   FAULT_LATCHED  — 普通故障已锁存, PWM 已关闭
 *   ESTOP_LATCHED  — 紧急停止已锁存 (Fault::EMERGENCY_STOP 位被置位)
 *
 * 白名单:
 *   INIT / ADC_CAL / ERROR 状态下忽略故障转换:
 *   - INIT: 硬件尚未初始化完毕, 假故障不应停机
 *   - ADC_CAL: 自校准中可能产生超范围读数
 *   - ERROR: 已经在故障态, 避免重复关断
 */

#pragma once

#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorManager;

class Motor_SafetyFSM
{
public:
    /*
     * step — 每个 tick 检查故障掩码, 有故障则进入锁存
     *
     * 调用频率: 每 50μs (20kHz), 由 Motor_FSM::step() 第 ① 步调用
     *
     * 行为:
     *   fault() == NONE → safety_state_ = CLEAR (无故障则保持清除)
     *   fault() != NONE → enterFaultLatched(m) (有故障立即关断)
     *
     * 设计要点: step() 只负责"反应"已有故障; 不负责故障检测本身
     */
    static void step(MotorManager& m);

    /*
     * latchFault — 主动锁存故障 (由子 FSM 或外部 API 调用)
     * @param fault  故障位, 通过 OR 合并到 fault_ 掩码
     */
    static void latchFault(MotorManager& m, Fault fault);

    /*
     * clearLatchedFault — 清除故障锁存, 恢复所有子状态
     * 将 safety_state_ 恢复为 CLEAR,
     * 同时重置所有子 FSM 状态 (command/task/angle/stop/energy)
     */
    static void clearLatchedFault(MotorManager& m);

private:
    /*
     * ignoresFaultTransition — 白名单检查
     * 某些生命周期阶段允许故障位存在而不触发关断
     */
    static bool ignoresFaultTransition(State state, Fault fault);

    /*
     * enterFaultLatched — 进入故障锁存态的核心逻辑
     *
     * 执行顺序:
     *   1. 区分 ESTOP 和普通 FAULT (通过 Fault::EMERGENCY_STOP 位)
     *   2. 白名单检查 (INIT/ADC_CAL/ERROR 跳过)
     *   3. 清零全部运行状态 (run_requested/mode/phase/sub-states)
     *   4. 清零控制目标 (speed_ref_limited/iq_ref_*)
     *   5. 取消待定自动重试
     *   6. 关断 PWM 硬件输出
     *   7. 推送 m.setState(ERROR)
     */
    static void enterFaultLatched(MotorManager& m);
};

} // namespace Lib_Motor
