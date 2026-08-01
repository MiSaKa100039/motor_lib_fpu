/*
 * Motor_AngleFSM.h — 角度源选择 FSM 声明
 *
 * 职责:
 *   在 RUN 状态下根据配置选择角度反馈源 (SENSOR / SMO / HFI)。
 *   每 tick 由 RunPhaseFSM 中间接调用 (通过 runObserverLoop 或 RunPhase::RUN_DIRECT),
 *   负责传感器就绪检查、失效确认和故障上报。
 *
 * 角度源状态 (AngleState):
 *   NONE          — 未选择
 *   SENSOR        — 使用外部位置传感器 (HALL3/ABZ/AS5600)
 *   SMO           — 使用滑模观测器 (无感中高速)
 *   HFI           — 高频注入 (未实现, 选作源时触发 OBSERVER_UNAVAILABLE)
 *   UNAVAILABLE   — 无可用源
 *
 * 失效处理:
 *   传感器连续无效 ≥ invalid_confirm_ticks → stopForFault (SENSOR_LOSS / SENSOR_SIGNAL_ERROR)
 *   SMO/HFI 未编译或不可用 → stopForFault (OBSERVER_UNAVAILABLE)
 */

#pragma once

#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorManager;

class Motor_AngleFSM
{
public:
    static void step(MotorManager& m);
    static bool validateClosedLoopEntry(MotorManager& m);
    static void clear(MotorManager& m);

private:
    static void useSensorSource(MotorManager& m);
    static void useSmoSource(MotorManager& m);
    static void useHfiSource(MotorManager& m);
    static void faultUnavailable(MotorManager& m, AngleState state, Fault fault);
};

} // namespace Lib_Motor
