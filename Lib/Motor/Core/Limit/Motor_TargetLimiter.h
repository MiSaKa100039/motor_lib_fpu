/*
 * Motor_TargetLimiter.h - 目标限幅/斜率/堵转候选的纯计算辅助模块
 *
 * 职责边界:
 *   MotorManager 持有状态、PID 和故障事件。
 *   本模块只根据配置和当前反馈计算“应该给多少目标值”。
 */

#pragma once

#include "../../Public/Motor_Config.h"

namespace Lib_Motor
{

class MotorTargetLimiter
{
public:
    /* 获取当前生效的 Iq 软上限(A) */
    static float configuredIqLimit(const MotorConfig& cfg);

    /* 速度目标限幅: 绝对值钳位 + 斜率限制 */
    static float limitSpeedReference(const MotorConfig& cfg,
                                     float dt,
                                     float current_limited_rpm,
                                     float requested_rpm);

    /* Iq 目标限幅: 绝对值钳位 + 接近限速时的保护削减 */
    static float limitIqReference(const MotorConfig& cfg,
                                  float speed_rpm,
                                  float iq_ref);

/* Iq 斜率限制: 双向非对称 (驱动 vs 制动分别两个 slew_rate) */
    static float applyIqSlewRate(const MotorConfig& cfg,
                                 float dt,
                                 float current_limited_iq,
                                 float requested_iq);

    /* 调制系数 K_mod (电压利用率, 矢量长度相对 Vbus):
     *   SPWM    = 0.5
     *   SVPWM   = 1/√3 ≈ 0.5774
     *   THIPWM  = 1/√3
     *   SIXSTEP = 1.0 (方波线电压 ≈ Vbus, 但相电压非线性, 仅作占位)
     */
    static float modulationKmod(ModulationMethod m);

    /* 联合电压矢量限幅: √(vd² + vq²) ≤ Vs_max = K_mod × Vbus
     * 若超出 Vs_max 则按比例缩放 vd/vq (圆限制, 替代旧的方限制);
     * 返回 (vd_clamped, vq_clamped) 经过引用修改;
     * 当前不扣减 Ke, 因为反电势前馈尚未接入最终电压输出。
     */
    static void limitVoltageVector(const MotorConfig& cfg,
                                   float vbus,
                                   float omega_elec_rad_s,
                                   float& vd,
                                   float& vq);

    /* 超速故障判定: |ω| > max_speed_rpm × overspeed_factor */
    static bool isOverspeed(const MotorConfig& cfg, float speed_rpm);

    /* 负载突变飞车预兆监控:
     * 若 |dω/dt| 超过 spike_threshold 且与 Iq 方向相反 (卸载飞车征兆),
     * 主动施加反向 Iq N ticks 抢先制动 (返回 override Iq, 0 = 不干预)。
     *
     * state: 飞车监控内部状态 (调用方持有, 0 初始化);
     * speed_rpm_prev: 上一拍速度 (由调用方维护);
     * 返回值: override 的 Iq, 0 = 不干预。
     */
    static float loadStepGuard(const MotorConfig& cfg,
                               float dt,
                               float speed_rpm_now,
                               float speed_rpm_prev,
                               float iq_ref_current,
                               int32_t& state_inout);

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    /*
     * 堵转候选判定:
     *   1. 实际速度很低
     *   2. 速度误差很大 (含 speed_error_hysteresis_rpm 退出滞回)
     *   3. Iq 已经接近当前允许上限
     *
     * 这里只判断"候选", 持续确认由 MotorManager 完成。
     */
    static bool isStallCandidate(const MotorConfig& cfg,
                                 float speed_rpm,
                                 float speed_error_rpm,
                                 float iq_ref,
                                 float iq_limit);
#endif
};

} // namespace Lib_Motor
