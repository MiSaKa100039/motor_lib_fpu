#pragma once

#include "Motor_Config.h"

namespace Platform_MotorConfig
{

inline void ApplyControlConfig(Lib_Motor::MotorConfig& cfg)
{
    /* ===================== [14] 控制方法与默认启动模式 ===================== */
    /*
     * default_control_method:
     *   描述当前工程主控制策略, 便于监控和后续扩展。
     *
     * startup_target_mode:
     *   用户调用 start() 后默认进入的模式。
     *   正常应用在这里配置, 调试宏只在 Platform_Motor.cpp 中临时覆盖它。
     *
     * command_direction:
     *   API 正方向到内部电机方向的映射, 只允许 +1/-1。
     *   -1 表示用户给 +rpm 时, 内部按反向旋转。
     */
    cfg.control.default_control_method = Lib_Motor::ControlMethod::FOC_SPEED;
    cfg.control.modulation = Lib_Motor::ModulationMethod::SVPWM;
    cfg.control.startup_target_mode = Lib_Motor::Mode::VELOCITY_CONTROL;
    cfg.control.control_freq_hz = 12000.0f;
    cfg.control.command_direction = 1;

    /* ===================== [15] 电流环 PI 参数 ===================== */
    /*
     * 电流环 PI 输出是 d/q 轴电压指令(V), 不是电流阈值。
     * 真正的电流目标上限由 motion.max_iq_ref_a、速度环输出上限和保护阈值共同限制。
     * 下面的 kp/ki 是手填 fallback; auto_derive_current_pid=true 时会被 R/L 推导值覆盖。
     */
    cfg.control.current_d.kp = 0.3f;
    cfg.control.current_d.ki = 10.0f;
    cfg.control.current_d.output_limit = 10.0f; // 初始化兜底, 运行期由 Vbus 动态覆盖

    cfg.control.current_q.kp = 0.2f;
    cfg.control.current_q.ki = 10.0f;
    cfg.control.current_q.output_limit = 10.0f; // 初始化兜底, 运行期由 Vbus 动态覆盖

    /* 电流环 PI 自动推导
     * auto_derive_current_pid=true 且 R/L 已知时, init() 用以下两字段推导 kp/ki 覆盖上面的手填 kp/ki:
     *   kp = L*(2*pi*f_bw)                  [V/A]
     *   ki = R*(2*pi*f_bw)                  [V/(A.s)]
     *   current_d/current_q.output_limit 仅作初始化兜底, 运行时按 Vbus 自动计算可用电压。
     * R/L 未知时先使用上面的手填 fallback
     * ConfigCheck 保证 f_bw < control_freq_hz/10 (避免离散化失效)。
     */
    cfg.control.auto_derive_current_pid    = false;
    cfg.control.current_loop_bandwidth_hz  = 500.0f;
    cfg.control.current_loop_damping_ratio = 0.707f; // 保留字段: 0.707 为二阶常用阻尼比; 当前一阶公式不参与 kp/ki 计算, 仅要求 >0

    /* ===================== [16] 速度环 PI 参数 ===================== */
    /*
     * 速度环输出 = Iq 指令(A), 然后再进入 q 轴电流环。
     *
     * output_limit 是速度 PI 的局部 Iq 上限, 用于输出钳位和抗积分饱和。
     * 实际生效值为 min(output_limit, motion.max_iq_ref_a 的三级回退值);
     * output_limit<=0 时回退到全局 Iq 软上限。该值不是相电流故障阈值。
     * 当前配置跟随 motion.max_iq_ref_a, 后续可直接填固定值形成更低的速度环上限。
     */
    cfg.control.speed.kp = 0.12f;
    cfg.control.speed.ki = 0.02f;
    cfg.control.speed.output_limit =(cfg.motion.max_iq_ref_a > 0.0f) ? cfg.motion.max_iq_ref_a : 50.0f;

    /* [P5 预留] 速度环自动推导 - 本轮 ConfigCheck 仍拒绝 auto_derive_speed_pid=true
     * 推导需 cfg.physical.load_inertia > 0 与 torque_constant > 0, 本工程填 0。
     * 字段就位以便 P5 直接放开 (ABI 不破坏)。
     */
    cfg.control.auto_derive_speed_pid     = false;
    cfg.control.speed_loop_bandwidth_hz   = 100.0f;
    cfg.control.speed_loop_damping_ratio = 0.707f;

    /* ===================== [17] 位置环 PI 参数 (占位) ===================== */
    /*
     * 位置环暂未接入执行链。
     * 预期链路:
     *   position_error(rad) -> 位置环 -> speed_ref(RPM) -> 速度环 -> Iq
     *
     * Hall3 这类扇区级反馈默认不开放精细位置环。
     * 后续只有在 BuildCfg 开位置环且反馈类别为 HIGH_RES_ENCODER 时才应启用。
     */
    cfg.control.position.kp = 0.0f;
    cfg.control.position.ki = 0.0f;
    cfg.control.position.kd = 0.0f;
    cfg.control.position.output_limit = 0.0f;
    cfg.control.position.ramp_rate = 0.0f;

    /* [P4/P5 预留] 位置环自动推导 - 本轮 ConfigCheck 拒 auto_derive_position_pid=true
     * 推导需 cfg.physical.load_inertia > 0 与 torque_constant > 0, 本工程填 0。
     * 字段就位以便 P5 直接放开 (ABI 不破坏)。
     */
    cfg.control.auto_derive_position_pid    = false;
    cfg.control.position_loop_bandwidth_hz  = 100.0f;
    cfg.control.position_loop_damping_ratio = 0.707f;

    /* ===================== [18] 弱磁参数 (占位) ===================== */
    /*
     * 当前仅保留配置入口, 尚未接入执行链。
     *
     * 未来预期:
     *   基速以下: Id 维持默认值 (通常 0A)
     *   接近电压极限且继续提速时: 逐步注入负 Id, 换取更高反电势工作区
     *
     * 注意:
     *   弱磁生效后, 电压限幅必须改成基于实时 Vbus 的电压矢量限幅,
     *   同时还要与总电流圆限制、Iq 裕量和超速保护联动。
     */
    cfg.control.field_weakening.enable = false;
    cfg.control.field_weakening.enter_speed_rpm = 0.0f;
    cfg.control.field_weakening.exit_speed_rpm = 0.0f;
    cfg.control.field_weakening.target_voltage_ratio = 0.95f;
    cfg.control.field_weakening.max_negative_id_a = 0.0f;
    cfg.control.field_weakening.current_reserve_ratio = 0.10f;
    cfg.control.field_weakening.kp = 0.0f;
    cfg.control.field_weakening.ki = 0.0f;
}

} // namespace Platform_MotorConfig
