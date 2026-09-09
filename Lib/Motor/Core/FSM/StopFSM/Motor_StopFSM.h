/*
 * Motor_StopFSM.h — 停机策略 FSM 声明
 *
 * 职责:
 *   在 STOPPING 状态下执行制动策略, 将抽象的 StopMode 指令
 *   解析为具体的硬件动作 (COAST / ELECTRICAL_BRAKE / MECHANICAL_BRAKE)。
 *
 * 停机状态 (StopState):
 *   IDLE                    — 空闲, 未停机
 *   COAST                   — 三相全关滑行 (Hi-Z)
 *   ELECTRICAL_BRAKE        — 电气制动 (下桥臂短路, 能量在绕组内耗散)
 *   MECHANICAL_BRAKE_WAIT   — 等待机械制动生效
 *   MECHANICAL_BRAKE_ENGAGED — 机械制动已接合
 *
 * 能力检查:
 *   canApplyElectricalBrake: brake.allow_low_side_brake && HAL 支持
 *   canApplyMechanicalBrake: 当前固定返回 false (机械制动未实现)
 *
 * 回退策略:
 *   MECHANICAL_BRAKE → 机械不可用 → 尝试电气制动 → 电气不可用 → COAST
 */

#pragma once

#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorManager;

class Motor_StopFSM
{
public:
    /*
     * step — 解析停机模式并执行硬件制动动作
     * 完成后推送 State::STOP
     */
    static void step(MotorManager& m);

    static void clear(MotorManager& m);

private:
    /*
     * resolveState — 将 StopMode 指令解析为可行的 StopState
     * 支持回退: MECHANICAL → ELECTRICAL → COAST
     */
    static StopState resolveState(const MotorManager& m, StopMode requested_mode);

    /*
     * applyResolvedState — 根据 StopState 调用硬件制动函数
     */
    static void applyResolvedState(MotorManager& m, StopState state);

    static bool canApplyElectricalBrake(const MotorManager& m);
    static bool canApplyMechanicalBrake(const MotorManager& m);
};

} // namespace Lib_Motor
