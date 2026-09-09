/*
 * Motor_StopFSM.cpp — 停机策略 FSM 实现
 */

#include "Motor_StopFSM.h"

#include "../../Manager/Motor_Manager.h"

namespace Lib_Motor
{

/*
 * step — 停机执行: 解析 → 应用 → 推送到 STOP
 */
void Motor_StopFSM::step(MotorManager& m)
{
    const StopState state = resolveState(m, m.stop_mode_);  // 将指令解析为可执行状态
    applyResolvedState(m, state);                            // 调用硬件动作
    m.setState(State::STOP);                                 // 制动完成, 进入 STOP (Idle)
}

void Motor_StopFSM::clear(MotorManager& m)
{
    m.stop_state_ = StopState::IDLE;
}

/*
 * resolveState — 将 StopMode 指令解析为可行的 StopState
 *
 * 回退策略:
 *   MECHANICAL_BRAKE:
 *     → 机械制动可用? → MECHANICAL_BRAKE_WAIT
 *     → 否则电气制动可用? → ELECTRICAL_BRAKE
 *     → 都不可用 → COAST
 *
 *   ELECTRICAL_BRAKE:
 *     → 电气制动可用? → ELECTRICAL_BRAKE
 *     → 不可用 → COAST
 *
 *   COAST: → COAST (无条件)
 */
StopState Motor_StopFSM::resolveState(const MotorManager& m, StopMode requested_mode)
{
    switch (requested_mode)
    {
        case StopMode::MECHANICAL_BRAKE:
            if (canApplyMechanicalBrake(m))
            {
                return StopState::MECHANICAL_BRAKE_WAIT;    // 请求机械制动 → 等待生效
            }
            if (canApplyElectricalBrake(m))
            {
                return StopState::ELECTRICAL_BRAKE;         // 回退到电气制动
            }
            return StopState::COAST;                        // 最终回退到滑行

        case StopMode::ELECTRICAL_BRAKE:
            if (canApplyElectricalBrake(m))
            {
                return StopState::ELECTRICAL_BRAKE;         // 电气制动
            }
            return StopState::COAST;                        // 回退到滑行

        case StopMode::COAST:
        default:
            return StopState::COAST;                        // 滑行 (无条件)
    }
}

/*
 * applyResolvedState — 根据 StopState 调用硬件制动函数
 *
 * ELECTRICAL_BRAKE: pwm_brake_lowside() (三相下桥臂短路)
 *   若 HAL 不支持 → fallthrough 到 COAST
 *
 * 其他 (COAST/MECHANICAL): pwm_coast() 或 pwm_disable()
 *   + set_duty(0,0,0) 确保软件端也无残留输出
 */
void Motor_StopFSM::applyResolvedState(MotorManager& m, StopState state)
{
    m.stop_state_ = state;

    if (m.config_.hal == nullptr)
    {
        return;                                              // 无硬件抽象层 → 无法操作硬件
    }

switch (state)
    {
        case StopState::ELECTRICAL_BRAKE:
            if (m.config_.brake.allow_low_side_brake &&       // 配置允许电气制动
                m.config_.hal->pwm_brake_lowside != nullptr)   // HAL 支持
            {
                m.config_.hal->pwm_brake_lowside();            // 执行电气制动
                return;
            }
            [[fallthrough]];                                   // 不支持 → 回退到滑行

        case StopState::MECHANICAL_BRAKE_WAIT:
#if LIB_MOTOR_ENABLE_MECHANICAL_BRAKE
            if (m.config_.brake.allow_mechanical_brake &&
                m.config_.hal->mechanical_brake_engage != nullptr)
            {
                if (m.config_.hal->pwm_coast != nullptr)
                {
                    m.config_.hal->pwm_coast();
                }
                else if (m.config_.hal->pwm_disable != nullptr)
                {
                    m.config_.hal->pwm_disable();
                }
                m.writePwmZeroOutputs();
                m.config_.hal->mechanical_brake_engage();
                m.stop_state_ = StopState::MECHANICAL_BRAKE_ENGAGED;
                return;
            }
#endif
            [[fallthrough]];

        case StopState::MECHANICAL_BRAKE_ENGAGED:
        case StopState::COAST:
        case StopState::IDLE:
        default:
            if (m.config_.hal->pwm_coast != nullptr)
            {
                m.config_.hal->pwm_coast();                    // Hi-Z 滑行
            }
            else if (m.config_.hal->pwm_disable != nullptr)
            {
                m.config_.hal->pwm_disable();                  // 关断 PWM
            }
            m.writePwmZeroOutputs();                           // 软件端清零占空比
            return;
    }
}

/*
 * canApplyElectricalBrake — 电气制动能力检查
 * 条件: 配置允许 + HAL 支持 pwm_brake_lowside
 */
bool Motor_StopFSM::canApplyElectricalBrake(const MotorManager& m)
{
    return m.config_.brake.allow_low_side_brake &&
           m.config_.hal != nullptr &&
           m.config_.hal->pwm_brake_lowside != nullptr;
}

/*
 * canApplyMechanicalBrake — 机械制动能力检查
 * 条件: 编译能力开启 + 配置允许 + HAL 支持 mechanical_brake_engage
 */
bool Motor_StopFSM::canApplyMechanicalBrake(const MotorManager& m)
{
#if LIB_MOTOR_ENABLE_MECHANICAL_BRAKE
    return m.config_.brake.allow_mechanical_brake &&
           m.config_.hal != nullptr &&
           m.config_.hal->mechanical_brake_engage != nullptr;
#else
    (void)m;
    return false;
#endif
}

} // namespace Lib_Motor
