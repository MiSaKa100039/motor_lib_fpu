/*
 * Motor_RunPhaseFSM.h — RUN 状态内部子阶段 FSM 声明
 *
 * 职责:
 *   管理电机运行时的内部阶段序列, 包括无感启动全过程
 *   (对齐 → IF 强拖 → SMO 切换 → 闭环运行)。
 *
 * 子阶段 (RunPhase):
 *   RUN_DIRECT     — 调试直通模式 (PWM_MANUAL/CURRENT_LOCK/IF_DRAG/VF_DRAG)
 *   SENSOR_ONLY    — 有感闭环 (直接使用编码器/霍尔)
 *   ALIGNMENT      — 辨识等非 IF 流程保留阶段
 *   FORCE_DRAG     — IF profile 内完成对齐和开环强拖
 *   SMO_ONLY       — SMO 无感闭环
 *   HFI_ONLY       — HFI 高频注入 (当前占位, 配置检查会拒绝)
 *   FUSION         — 启动角度源到 SMO 的平滑交接
 *
 * 启动流程:
 *   IF + SMO 启动:    FORCE_DRAG(ALIGNMENT + RAMP) → SMO_ONLY
 *   HFI + SMO 启动:   计划为 HFI_ONLY → SMO_ONLY, 当前 HFI 未实现
 *   有感启动:          SENSOR_ONLY (直接)
 */

#pragma once

#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorManager;

class Motor_RunPhaseFSM
{
public:
    /*
     * selectInitialPhase — 根据配置选择初始运行阶段
     *
     * @param m           MotorManager 引用
     * @param direct_run  true=调试直通 (RUN_DIRECT), false=正常启动流程
     * @return            初始 RunPhase 的整数值
     */
    static RunPhase selectInitialPhase(MotorManager& m, bool direct_run);

    static void step(MotorManager& m);

private:
    static void handleAlignment(MotorManager& m);
    static void handleForceDrag(MotorManager& m);
    static bool tryBeginSmoHandover(MotorManager& m, bool allow_transition);
    static void handleFusion(MotorManager& m);
    static void handleSmo(MotorManager& m);
    static void handleHfi(MotorManager& m);
};

} // namespace Lib_Motor
