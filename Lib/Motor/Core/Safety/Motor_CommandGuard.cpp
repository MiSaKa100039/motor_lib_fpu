/*
 * Motor_CommandGuard.cpp -- 安全检测: 命令防护
 *
 * 文件层级: Safety/CommandGuard
 *
 * 职责: 验证 API 命令的合法性, 防止非法模式切换
 *   1. validateApiModeChange: 校验模式切换请求的合法性 (状态/模式组合)
 *   2. validateModeSelection: 校验所选模式是否与当前配置兼容
 */

#include "Motor_CommandGuard.h"

#include "../Feature/Motor_FeatureRegistry.h"
#include "Motor_ConfigCheck.h"

namespace Lib_Motor
{

/*
 * isDebugOrCalib -- 判断模式是否为调试/校准类
 * 调试/校准模式允许在 STOP 状态直接进入, 无需经过正常启动流程
 */
bool MotorCommandGuard::isDebugOrCalib(Mode mode)
{

    if (mode == Mode::DEBUG_PWM_MANUAL ||
        mode == Mode::DEBUG_CURRENT_LOCK ||
        mode == Mode::DEBUG_HFI_OBSERVER ||
        mode == Mode::DEBUG_VF_DRAG)
    {
        return true;
    }

    if (mode == Mode::DEBUG_IF_DRAG)
    {
        return true;
    }

    if (mode == Mode::DEBUG_IF_SMO_OBSERVER ||
        mode == Mode::DEBUG_IF_HFI_OBSERVER)
    {
        return true;
    }

    return mode == Mode::CALIB_RL_IDENTIFY;
}

/*
 * validateApiModeChange -- API 模式切换合法性校验
 *
 * 规则:
 *   - 模式切换仅在 STOP 或 RUN 状态允许
 *   - 从调试模式切换到正常模式: 禁止 (需先 STOP 再换)
 *   - 到调试模式: 允许 (用于 STOP 状态进入调试)
 */
Result MotorCommandGuard::validateApiModeChange(State state, Mode current_mode, Mode new_mode)
{
    if (state != State::STOP && state != State::RUN)
    {
        return Result::InvalidState;
    }

    const bool from_debug = isDebugOrCalib(current_mode);
    const bool to_debug = isDebugOrCalib(new_mode);

    if (!from_debug && !to_debug)
    {
        return Result::Ok;
    }

    if (from_debug && !to_debug)
    {
        return Result::InvalidState;
    }

    if (to_debug)
    {
        return Result::Ok;
    }

    return Result::Error;
}

/*
 * validateModeSelection -- 模式选择与配置兼容性检查
 *
 * 检查流程:
 *   1. 基础配置有效性 (validateBase)
 *   2. 闭环模式: 反馈配置有效性 (validateFeedback)
 *   3. 传感器闭环: 反馈能力检查 (角度类/扭矩支持/速度支持)
 *   4. 速度闭环: 速度环编译检查
 *   5. 位置控制: 暂不支持
 *   6. 调试/校准: 无条件放行
 */
Result MotorCommandGuard::validateModeSelection(const MotorConfig& cfg, Mode mode)
{
    return validateModeSelection(cfg, cfg.default_run_policy, mode);
}

Result MotorCommandGuard::validateModeSelection(const MotorConfig& cfg,
                                                const MotorRunPolicy& policy,
                                                Mode mode)
{
    return validateModeSelection(MakeMotorHardwareConfig(cfg),
                                 MakeMotorAlgorithmParam(cfg),
                                 policy,
                                 mode);
}

Result MotorCommandGuard::validateModeSelection(const MotorHardwareConfig& hardware,
                                                const MotorAlgorithmParam& algorithm,
                                                const MotorRunPolicy& policy,
                                                Mode mode)
{
    MotorConfig base_cfg;
    base_cfg.hal = hardware.hal;
    base_cfg.physical = hardware.physical;
    base_cfg.limit = hardware.limit;
    base_cfg.sensor = hardware.sensor;
    base_cfg.position = hardware.position;
    base_cfg.control = algorithm.control;
    base_cfg.brake = algorithm.brake;
    base_cfg.identify = algorithm.identify;

    if (MotorConfigCheck::validateBase(base_cfg) != Fault::NONE)
    {
        return Result::InvalidParam;
    }

    if (!MotorFeatureRegistry::supportsMode(mode))
    {
        return Result::NotSupported;
    }

    switch (mode)
    {
        case Mode::NONE:
            return Result::Ok;

case Mode::TORQUE_CONTROL:
        case Mode::VELOCITY_CONTROL:
        case Mode::POSITION_CONTROL:
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
        case Mode::IMPEDANCE_CONTROL:
#endif
        {
            /* TODO HostTest: POSITION/IMPEDANCE 入口门控 (HIGH_RES_ENCODER 通过/HALL 拒绝)
             *                未编单测, 当前依赖 VOFA 真硬件验证不同传感器反馈类下的 accept/reject。 */
            if (MotorConfigCheck::validateFeedback(hardware, algorithm, policy) != Fault::NONE)
            {
                return Result::NotSupported;
            }

            if (policy.steady_source == SteadyAngleSource::SENSOR)
            {
                const auto& capability = hardware.position.rotor_feedback;
                const AngleFeedbackClass feedback_class = capability.feedback_class;
                if (!FeedbackClassSupportsTorqueControl(feedback_class))
                {
                    return Result::NotSupported;
                }
                if (mode == Mode::VELOCITY_CONTROL &&
                    !FeedbackClassSupportsSpeedControl(feedback_class))
                {
                    return Result::NotSupported;
                }
            }
            /* 位置环与阻抗环要求高分辨率有感角度, 无感/HALL 扇区反馈不开放精细位置闭环。 */
            bool requires_high_res_feedback = false;
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
            requires_high_res_feedback = requires_high_res_feedback ||
                                         mode == Mode::POSITION_CONTROL;
#endif
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
            requires_high_res_feedback = requires_high_res_feedback ||
                                         mode == Mode::IMPEDANCE_CONTROL;
#endif
            if (requires_high_res_feedback)
            {
                if (policy.steady_source != SteadyAngleSource::SENSOR)
                {
                    return Result::NotSupported;
                }
                if (!FeedbackClassSupportsPositionControl(hardware.position.rotor_feedback.feedback_class))
                {
                    return Result::NotSupported;
                }
            }
            return Result::Ok;
        }

        case Mode::DEBUG_PWM_MANUAL:
        case Mode::DEBUG_CURRENT_LOCK:
        case Mode::DEBUG_HFI_OBSERVER:
        case Mode::DEBUG_VF_DRAG:
            return Result::Ok;

        case Mode::DEBUG_IF_DRAG:
        case Mode::DEBUG_IF_SMO_OBSERVER:
        case Mode::DEBUG_IF_HFI_OBSERVER:
            return Result::Ok;

        case Mode::CALIB_RL_IDENTIFY:
            return Result::Ok;

        default:
            return Result::NotSupported;
    }
}

} // namespace Lib_Motor
