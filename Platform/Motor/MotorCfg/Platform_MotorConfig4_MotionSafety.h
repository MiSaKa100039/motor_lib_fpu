#pragma once

#include "Motor_Config.h"

namespace Platform_MotorConfig
{

inline void ApplyMotionSafetyConfig(Lib_Motor::MotorConfig& cfg)
{
    /* ===================== [19] 运动软限幅 ===================== */
    /*
     * 这些是正常控制路径中的软限制, 不等同于硬故障阈值。
     * max_iq_ref_a:              速度环/转矩模式允许输出的最大 Iq 指令 (A)
     * iq_slew_rate_a_per_s:      Iq 变化斜率限制 (A/s), 防止负载突变时电流台阶过冲
     * speed_slew_rate_rpm_s:     速度目标斜率限制 (RPM/s), 用于柔和启动/停机
     * torque_speed_guard_band_rpm: 转矩模式接近限速时提前削减同向 Iq 的保护带 (RPM)
     */
    cfg.motion.max_iq_ref_a = 100.0f;
    cfg.motion.iq_slew_rate_a_per_s = 400.0f;
    cfg.motion.speed_slew_rate_rpm_s = 1000.0f;
    cfg.motion.torque_speed_guard_band_rpm = 100.0f;

    /* 双向非对称 Iq 斜率 + 超速故障系数 + 跨零门限 */
    cfg.motion.iq_slew_rate_brake_a_per_s = 100.0f;   // 制动方向斜率, 卸载救援快
    cfg.motion.overspeed_factor            = 1.10f;   // |ω| > max_speed_rpm × 1.10 → Fault::OVERSPEED
    cfg.motion.reverse_zero_band_rpm      = 50.0f;    // 反转跨零软减速门限, |ω|>50 RPM 反向指令先减速到 0

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    /* ===================== [20] 堵转保护 ===================== */
    /*
     * 堵转检测条件 (需同时满足):
     *   1. |speed| < speed_abs_rpm (低速判定)
     *   2. |speed_error| > speed_error_rpm (期望转但实际不转)
     *   3. |Iq| > iq_limit × iq_limit_ratio (已输出极限力矩)
     *
     * enable:                    堵转检测总开关, false=关闭全部堵转保护
     * speed_abs_rpm:             低速阈值 (RPM), |speed| < 此值视为"几乎静止"
     * speed_error_rpm:           速度误差阈值 (RPM), 期望转但实际不转 → 候选
     * iq_limit_ratio:            Iq 饱和度阈值, |Iq| > iq_limit × 此比例 → 已输出极限力矩
     * confirm_time_s:            连续满足三条件多久才确认堵转 (s), 防瞬间误判
     *
     * speed_integral_decay:      堵转期间速度环积分衰减系数 (<1 防积分饱和, 1=不衰减)
     * latch_fault:               确认堵转后是否锁存故障, true=永久锁需手动清除, false=仅事件标志
     *
     * release_speed_rpm:         自动释放堵转的转速阈值 (RPM), 转速恢复到此值以上开始释放
     * release_confirm_time_s:    自动释放确认时间 (s), 需此时间内持续满足恢复条件
     * count_total_events:        是否累计堵转总数 (stall_total_count_, 供上位机查询)
     */
    cfg.motion.stall.enable = true;
    cfg.motion.stall.speed_abs_rpm = 30.0f;
    cfg.motion.stall.speed_error_rpm = 150.0f;
    cfg.motion.stall.iq_limit_ratio = 0.90f;
    cfg.motion.stall.confirm_time_s = 0.5f;
    cfg.motion.stall.speed_integral_decay = 0.98f;
    cfg.motion.stall.latch_fault = true;
    cfg.motion.stall.release_speed_rpm = 120.0f;
    cfg.motion.stall.release_confirm_time_s = 0.10f;
    cfg.motion.stall.count_total_events = true;

    /*
     * 自动重试参数:
     * auto_restart:                      堵转后是否自动重试, true=风机/泵类自恢复, false=手动清故障
     * restart_count:                     最大自动重试次数, 0=不限制, >0 用完则锁故障
     * restart_interval_s:                两次自动重试之间的等待间隔 (s)
     * restart_stop_mode:                 重试前制动方式
     *   COAST            → 自由滑行 (三相全关)
     *   ELECTRICAL_BRAKE → 能耗制动
     *   MECHANICAL_BRAKE → 机械抱闸
     * clear_fault_before_restart:        重试前是否自动清除旧故障码
     * require_manual_clear_after_retry_exhausted: 重试次数用完后是否要求手动清故障
     */
    cfg.motion.stall.auto_restart = false;
    cfg.motion.stall.restart_count = 0;
    cfg.motion.stall.restart_interval_s = 5.0f;
    cfg.motion.stall.restart_stop_mode = Lib_Motor::StopMode::COAST;
    cfg.motion.stall.clear_fault_before_restart = true;
    cfg.motion.stall.require_manual_clear_after_retry_exhausted = true;
#endif
}

} // namespace Platform_MotorConfig
