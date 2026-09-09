#pragma once

#include "Motor_Config.h"

namespace Platform_MotorConfig
{

inline void ApplyBrakeAndCommandConfig(Lib_Motor::MotorConfig& cfg)
{
    /* ===================== [20] 制动策略 ===================== */
    /*
     * 当前仅声明策略开关, 实际硬件动作由 BSP 层执行。
     * BrakeManager/EnergyManager 后续将消费这些策略值。
     *
     * allow_foc_negative_iq:   是否允许 FOC 反向 Iq 再生制动
     *                          电机回馈能量经 MOS 体二极管整流到母线
     *                          → 母线电压上升, 过高则触发过压保护
     * allow_low_side_brake:    是否允许三相下桥臂短路制动
     *                          能量在电机绕组内耗散, 不抬升母线
     * allow_coast:             是否允许自由滑行 (三相全关)
     * has_brake_resistor:      是否有制动电阻 (能耗制动)
     * has_battery_regen_path:  是否有电池回充通路 (能量回收)
     * control_brake_resistor_chopper: 是否由本 MCU 控制制动电阻斩波 MOSFET
     *
     * max_brake_iq_a:          全局制动 Iq 上限, 0 表示跟随 motion.max_iq_ref_a
     * max_regen_iq_a:          电池回充路径 Iq 上限, 0 表示跟随 max_brake_iq_a
     * max_dump_iq_a:           电阻泄放路径 Iq 上限, 0 表示跟随 max_brake_iq_a
     * normal_decel_rpm_s:      正常减速斜率 (RPM/s)
     * fast_decel_rpm_s:        快速/急停减速斜率 (RPM/s)
     */
    cfg.brake.allow_foc_negative_iq = false; // 负 Iq 主动制动尚未完成能量路径验证, 默认关闭
    cfg.brake.allow_low_side_brake = true;    // 允许三相下桥臂短路制动 (能量在绕组内耗散, 不抬升母线)
    cfg.brake.allow_mechanical_brake = false; // 机械制动执行机构未接入前默认关闭
    cfg.brake.allow_coast = true;             // 允许自由滑行 (三相全关 Hi-Z)

    cfg.brake.has_brake_resistor = false;     // 是否配置了制动电阻 (能耗制动), true 且有电阻才能用再生制动
    cfg.brake.has_battery_regen_path = false; // 是否有电池回充通路 (能量回收), 无则禁用 regen 以免母线过压
    cfg.brake.control_brake_resistor_chopper = false; // false=电阻支路外部自控或未接入, lib 只消费 budget
    cfg.brake.max_brake_iq_a = 0.0f;          // 制动电流上限 (A), 0=不限制, 设非零则限制制动时的最大反向 Iq
    cfg.brake.max_regen_iq_a = 0.0f;          // 回充路径静态 Iq 上限 (A), 0=跟随 max_brake_iq_a
    cfg.brake.max_dump_iq_a = 0.0f;           // 泄放路径静态 Iq 上限 (A), 0=跟随 max_brake_iq_a
    cfg.brake.max_regen_power_w = 0.0f;       // 静态回收功率边界 (W), 0=未声明
    cfg.brake.max_dump_power_w = 0.0f;        // 静态泄放功率边界 (W), 0=未声明
    cfg.brake.energy_path_policy = Lib_Motor::BrakeEnergyPathPolicy::REGEN_FIRST;
    cfg.brake.normal_decel_rpm_s = 0.0f;      // 正常减速斜率 (RPM/s), 0=不限制, 用于 STOP 命令的平滑停机
    cfg.brake.fast_decel_rpm_s = 0.0f;        // 急停减速斜率 (RPM/s), 0=不限制, 用于紧急停机
    cfg.brake.budget_loss_behavior = Lib_Motor::BudgetLossBehavior::BRAKE_ON_LOSS;
}

} // namespace Platform_MotorConfig
