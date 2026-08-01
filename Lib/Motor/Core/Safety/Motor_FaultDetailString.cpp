/*
 * Motor_FaultDetailString.cpp -- 配置错误码 → 可读字符串映射
 *
 * 文件层级: Safety/Diagnostic
 *
 * 编译开关: LIB_MOTOR_ENABLE_HUMAN_READABLE_FAULT_DETAIL
 *   =1 (默认): 编入本文件全部字符串表 (~1.3KB ROM);
 *   =0 量产关闭: 函数保留 (ABI 稳) 返回空串, 应用层查裸 detail 值自行解码。
 *
 * 设计原则:
 *   1. 全表 switch 单点维护, 与 Motor_Definitions.h 的 MotorConfigFaultDetail 一一对应;
 *   2. 每个枚举返回简洁中文说明 (≤40 字符), 上层打印一眼定位;
 *   3. 未知码返回 "UNKNOWN 0xXXXX", 不静默返空, 防止新增枚举时漏改本表。
 */

#include "../../Public/Motor_Definitions.h"
#include "../../Public/Motor_Features.h"

namespace Lib_Motor
{

const char* MotorConfigFaultDetailToString(MotorConfigFaultDetail detail)
{
#if LIB_MOTOR_ENABLE_HUMAN_READABLE_FAULT_DETAIL
    switch (detail)
    {
        case MotorConfigFaultDetail::NONE:
            return "无配置错误";

        /* 0x01xx BuildCfg 宏非法 */
        case MotorConfigFaultDetail::BUILD_CURRENT_SENSE_MODE_INVALID:
            return "BuildCfg: 电流采样模式必须为 2 或 3";
        case MotorConfigFaultDetail::BUILD_NTC_SLOTS_INVALID:
            return "BuildCfg: NTC 通道数越界 (0~4)";
        case MotorConfigFaultDetail::BUILD_REDUNDANT_REQUIRES_SENSOR:
            return "BuildCfg: 冗余传感器需先开 SENSOR";
        case MotorConfigFaultDetail::BUILD_OUTPUT_REQUIRES_SENSOR:
            return "BuildCfg: 输出侧传感器需先开 SENSOR";
        case MotorConfigFaultDetail::BUILD_REGEN_REQUIRES_ENERGY_FSM:
            return "BuildCfg: 电池回充需开制动能量 FSM";
        case MotorConfigFaultDetail::BUILD_RESISTOR_REQUIRES_ENERGY_FSM:
            return "BuildCfg: 制动电阻需开制动能量 FSM";

        /* 0x02xx 基础硬件 */
        case MotorConfigFaultDetail::BUILD_TICK_PROFILING_UNSUPPORTED:
            return "BuildCfg: Cortex-M0 does not support internal tick profiling; use GPIO and oscilloscope";
        case MotorConfigFaultDetail::HAL_NULL:
            return "HAL 接口指针为空";
        case MotorConfigFaultDetail::POLE_PAIRS_INVALID:
            return "极对数为 0, 非法";
        case MotorConfigFaultDetail::CONTROL_FREQ_INVALID:
            return "控制频率 ≤0 或非有限值";
        case MotorConfigFaultDetail::CURRENT_SENSE_MODE_INVALID:
            return "电流采样模式非法";
        case MotorConfigFaultDetail::CURRENT_SENSE_MODE_BUILD_MISMATCH:
            return "电流采样模式与 BuildCfg 不一致";
        case MotorConfigFaultDetail::BUS_CURRENT_NOT_BUILT:
            return "母线电流采样未编入固件";
        case MotorConfigFaultDetail::PHASE_VOLTAGE_NOT_BUILT:
            return "相电压采样未编入 BuildCfg";
        case MotorConfigFaultDetail::CURRENT_SENSOR_CONFIG_INVALID:
            return "Current sensor config invalid";
        case MotorConfigFaultDetail::PHASE_CURRENT_MASK_INVALID:
            return "相电流有效掩码非法";
        case MotorConfigFaultDetail::PHASE_CURRENT_DUAL_MASK_INVALID:
            return "双电阻模式相电流掩码非法";
        case MotorConfigFaultDetail::PHASE_CURRENT_TRIPLE_MASK_INVALID:
            return "三电阻模式相电流掩码非法";
        case MotorConfigFaultDetail::PHASE_CURRENT_SINGLE_SHUNT_MASK_INVALID:
            return "Single-shunt phase current mask invalid";
        case MotorConfigFaultDetail::SINGLE_SHUNT_HAL_INVALID:
            return "Single-shunt HAL callbacks invalid";
        case MotorConfigFaultDetail::SINGLE_SHUNT_WINDOW_INVALID:
            return "Single-shunt sample window invalid";
        case MotorConfigFaultDetail::SINGLE_SHUNT_REQUIRES_SHUNT_SENSOR:
            return "Single-shunt requires shunt-resistor current sensing";
        case MotorConfigFaultDetail::SINGLE_SHUNT_REQUIRES_SVPWM:
            return "Single-shunt requires SVPWM modulation";
        case MotorConfigFaultDetail::SENSOR_LPF_INVALID:
            return "LPF 时间常数非有限";
        case MotorConfigFaultDetail::SENSOR_LPF_NEGATIVE:
            return "LPF 时间常数为负";
        case MotorConfigFaultDetail::VBUS_CONFIG_INVALID:
            return "母线电压分压/参数非法";
        case MotorConfigFaultDetail::PHASE_VOLTAGE_CONFIG_INVALID:
            return "相电压采样配置非法";
        case MotorConfigFaultDetail::BRAKE_NO_SINK_PATH:
            return "反向 Iq 已开启但无任何能量承接路径 (无 regen/resistor/low_side)";
        case MotorConfigFaultDetail::BRAKE_ENERGY_FSM_NOT_BUILT:
            return "制动能量 FSM 未编译";
        case MotorConfigFaultDetail::BATTERY_REGEN_NOT_BUILT:
            return "电池回充路径未编译";
        case MotorConfigFaultDetail::BRAKE_RESISTOR_NOT_BUILT:
            return "制动电阻路径未编译";
        case MotorConfigFaultDetail::MECHANICAL_BRAKE_NOT_BUILT:
            return "机械制动未编译";
        case MotorConfigFaultDetail::MECHANICAL_BRAKE_HAL_INVALID:
            return "机械制动 HAL 回调非法";
        case MotorConfigFaultDetail::BRAKE_CHOPPER_NOT_BUILT:
            return "制动电阻斩波控制未编译";
        case MotorConfigFaultDetail::BRAKE_CHOPPER_HAL_INVALID:
            return "制动电阻斩波 HAL 回调非法";
        case MotorConfigFaultDetail::BRAKE_CHOPPER_REQUIRES_RESISTOR:
            return "制动电阻斩波控制需要 has_brake_resistor=true";
        case MotorConfigFaultDetail::PHYSICAL_PARAM_INVALID:
            return "电机物理参数来源或固化/辨识值非法";

        /* 0x03xx 启动目标模式 */
        case MotorConfigFaultDetail::STARTUP_DEBUG_API_DISABLED:
            return "调试 API 未启用";
        case MotorConfigFaultDetail::STARTUP_IF_DEBUG_DISABLED:
            return "IF 调试未编译";
        case MotorConfigFaultDetail::STARTUP_SPEED_LOOP_DISABLED:
            return "速度环未编译";
        case MotorConfigFaultDetail::STARTUP_SENSOR_TORQUE_UNSUPPORTED:
            return "SENSOR 与模式组合不支持";
        case MotorConfigFaultDetail::STARTUP_SENSOR_SPEED_UNSUPPORTED:
            return "SENSOR 速度模式构建未支持";
        case MotorConfigFaultDetail::STARTUP_POSITION_UNSUPPORTED:
            return "位置模式未编译";
        case MotorConfigFaultDetail::STARTUP_MODE_INVALID:
            return "启动目标模式非法";

        /* 0x04xx 反馈路径 */
        case MotorConfigFaultDetail::FEEDBACK_REDUNDANT_NOT_BUILT:
            return "冗余传感器未编译";
        case MotorConfigFaultDetail::FEEDBACK_OUTPUT_NOT_BUILT:
            return "输出传感器未编译";
        case MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_NOT_BUILT:
            return "稳态角度源未编译";
        case MotorConfigFaultDetail::FEEDBACK_STARTUP_SOURCE_NOT_BUILT:
            return "启动角度源未编译";
        case MotorConfigFaultDetail::FEEDBACK_HEALTH_PARAM_INVALID:
            return "位置健康参数非法";
        case MotorConfigFaultDetail::FEEDBACK_REDUNDANT_SENSOR_INVALID:
            return "冗余传感器配置非法";
        case MotorConfigFaultDetail::FEEDBACK_OUTPUT_SENSOR_INVALID:
            return "输出传感器配置非法";
        case MotorConfigFaultDetail::FEEDBACK_REDUNDANT_CHECK_INVALID:
            return "冗余校验配置非法";
        case MotorConfigFaultDetail::FEEDBACK_ROTOR_SENSOR_INVALID:
            return "主转子传感器配置非法";
        case MotorConfigFaultDetail::FEEDBACK_SENSOR_STARTUP_MISMATCH:
            return "启动源与传感器配置不一致";
        case MotorConfigFaultDetail::FEEDBACK_ROTOR_CAPABILITY_INVALID:
            return "转子反馈能力非法";
        case MotorConfigFaultDetail::FEEDBACK_SENSOR_INVALID_TICKS:
            return "传感器 tick 数非法";
        case MotorConfigFaultDetail::FEEDBACK_SENSOR_FAULT_ACTION_UNSUPPORTED:
            return "传感器故障动作不支持";
        case MotorConfigFaultDetail::FEEDBACK_SMO_PARAM_INVALID:
            return "SMO 参数非法";
        case MotorConfigFaultDetail::FEEDBACK_IF_PARAM_INVALID:
            return "IF 启动参数非法 (align_time/drag_current/timeout)";
        case MotorConfigFaultDetail::FEEDBACK_HFI_UNAVAILABLE:
            return "HFI 观测器未实现";
        case MotorConfigFaultDetail::FEEDBACK_SENSOR_TO_SMO_UNAVAILABLE:
            return "SENSOR→SMO 接管链未实现";
        case MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_UNAVAILABLE:
            return "稳态角度源未实现";
        case MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_INVALID:
            return "稳态角度源非法";
        case MotorConfigFaultDetail::FEEDBACK_HFI_PARAM_INVALID:
            return "HFI 参数非法";
        case MotorConfigFaultDetail::FEEDBACK_SMO_CLOSED_LOOP_DISABLED:
            return "SMO 闭环需显式打开 allow_smo_closed_loop";
        case MotorConfigFaultDetail::FEEDBACK_FLYING_START_NOT_BUILT:
            return "顺逆风启动未在 BuildCfg 编译";
        case MotorConfigFaultDetail::FEEDBACK_FLYING_START_NEEDS_PHASE_VOLTAGE:
            return "顺逆风启动需相电压采样能力";
        case MotorConfigFaultDetail::FEEDBACK_FLYING_START_UNAVAILABLE:
            return "顺逆风启动执行链未实现";
        case MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED:
            return "Single-shunt sampling does not support HFI";
        case MotorConfigFaultDetail::FEEDBACK_FLYING_START_SENSOR_STARTUP_UNSUPPORTED:
            return "有感启动不应开启相电压顺逆风启动";

        /* 0x04xx 段 — IF 静止保证机制 (本轮新增) */
        case MotorConfigFaultDetail::IF_STATIONARY_GUARANTEE_REQUIRED:
            return "IF 启动未配置静止保证机制 (三选一: 相电压顺逆风启动/低边短路/明确绕过)";
        case MotorConfigFaultDetail::IF_STATIONARY_NEEDS_PHASE_VOLTAGE_BUILD:
            return "选相电压顺逆风启动需 BuildCfg 开 PHASE_VOLTAGE 且 cfg.has_phase_voltage=true";
        case MotorConfigFaultDetail::IF_STATIONARY_NEEDS_LOW_SIDE_BRAKE:
            return "选低边短路需 cfg.brake.allow_low_side_brake=true 且 HAL 有 pwm_brake_lowside";
        case MotorConfigFaultDetail::IF_STATIONARY_EXPLICIT_BYPASS_UNCONFIRMED:
            return "选明确绕过需 if_explicit_bypass_confirmed=true";

        /* 0x05xx 电流环带宽 / 自动推导 (本轮新增) */
        case MotorConfigFaultDetail::CURRENT_LOOP_BW_TOO_HIGH:
            return "电流环带宽 ≥ 控制频率/10, 离散化失效";
        case MotorConfigFaultDetail::CURRENT_LOOP_BW_REQUIRES_L_KNOWN:
            return "自动推导电流环 PI 暂缺 R/L, init 使用手填 fallback";
        case MotorConfigFaultDetail::SPEED_LOOP_AUTOTUNE_NOT_IMPLEMENTED:
            return "速度环自动推导尚未实现 (P5 阶段开放)";
        case MotorConfigFaultDetail::RL_IDENTIFY_NOT_BUILT:
            return "BuildCfg 未编 AUTO_IDENTIFY: CALIB_RL_IDENTIFY 不可用";
        case MotorConfigFaultDetail::RL_IDENTIFY_FAILED:
            return "RL 辨识失败 (di/dt 太小或异常)";
        case MotorConfigFaultDetail::KE_IDENTIFY_FAILED:
            return "Ke 辨识失败 (SMO 未收敛或速度过低)";
        case MotorConfigFaultDetail::POSITION_LOOP_AUTOTUNE_NOT_IMPLEMENTED:
            return "位置环自动推导尚未实现 (P5 阶段开放)";
        case MotorConfigFaultDetail::RL_IDENTIFY_PARAM_INVALID:
            return "RL 辨识约束参数非法";

        /* 0x06xx 调制 / 控制拓扑 (本轮新增) */
        case MotorConfigFaultDetail::MODULATION_SIXSTEP_REQUIRES_TRAP:
            return "SIXSTEP 调制需 BuildCfg 开 TRAP_CONTROL";
        case MotorConfigFaultDetail::CONTROL_TOPOLOGY_NONE_ENABLED:
            return "FOC / TRAP 至少一个 BuildCfg 开关未启用";

        default:
            return "UNKNOWN 0x????";
    }
#else
    (void)detail;
    return "";
#endif
}

} // namespace Lib_Motor
