#include "Motor_TargetLimiter.h"

#include <cmath>

namespace Lib_Motor
{

namespace
{
// 浮点值绝对值钳位: 将 value 限制在 [min_value, max_value] 区间
float clampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// 浮点值斜率限制: current 每步最多变化 max_delta, 向 target 梯形逼近
// max_delta ≤ 0 表示不限制 (直通)
float slewFloat(float current, float target, float max_delta)
{
    if (max_delta <= 0.0f) return target;

    const float delta = target - current;
    if (delta > max_delta) return current + max_delta;   // 正向加速, 按 max_delta 逐步逼近
    if (delta < -max_delta) return current - max_delta;   // 反向减速, 按 max_delta 逐步逼近
    return target;                                        // 差值在允许范围内, 直接到目标
}

} // namespace

/*
 * configuredIqLimit — 获取当前生效的 Iq 软上限 (三级回退)
 *
 * 优先级:
 *   1. cfg.motion.max_iq_ref_a      (用户配置的全局正常运行 Iq 软上限)
 *   2. cfg.physical.rated_current   (电机额定连续电流)
 *   3. cfg.limit.max_phase_current_a (相线软件过流阈值, 最后兜底)
 */
float MotorTargetLimiter::configuredIqLimit(const MotorConfig& cfg)
{
    if (cfg.motion.max_iq_ref_a > 0.0f)
    {
        return cfg.motion.max_iq_ref_a;
    }
    if (cfg.physical.rated_current > 0.0f)
    {
        return cfg.physical.rated_current;
    }
    return cfg.limit.max_phase_current_a;
}

/*
 * configuredSpeedPidOutputLimit — 获取速度 PI 的有效 Iq 输出上限
 *
 * control.speed.output_limit 是速度环局部上限, 不能越过正常运行 Iq 软上限。
 * 局部上限未配置(<=0)时回退到全局软上限, 保持浮点与 Q15 后端语义一致。
 */
float MotorTargetLimiter::configuredSpeedPidOutputLimit(const MotorConfig& cfg)
{
    const float iq_limit = configuredIqLimit(cfg);
    const float speed_limit = cfg.control.speed.output_limit;

    if (speed_limit <= 0.0f)
    {
        return iq_limit;
    }
    if (iq_limit <= 0.0f)
    {
        return speed_limit;
    }
    return (speed_limit < iq_limit) ? speed_limit : iq_limit;
}

/*
 * limitSpeedReference — 速度目标限幅: 绝对值钳位 + 斜率限制
 *
 * 步骤:
 *   1. 绝对值钳位: 速度限制在 [-max_speed_rpm, max_speed_rpm]
 *   2. 斜率限制: 每次 tick 速度变化量 ≤ slew_rate × dt
 *
 * 斜率限制用于柔和启停, 防止阶跃速度命令造成电流冲击过大。
 */
float MotorTargetLimiter::limitSpeedReference(const MotorConfig& cfg,
                                              float dt,
                                              float current_limited_rpm,
                                              float requested_rpm)
{
    float limited = requested_rpm;

    // [1] 绝对值钳位
    if (cfg.limit.max_speed_rpm > 0.0f)
    {
        limited = clampFloat(limited, -cfg.limit.max_speed_rpm, cfg.limit.max_speed_rpm);
    }

    // [2] 斜率限速
    if (cfg.motion.speed_slew_rate_rpm_s > 0.0f)
    {
        limited = slewFloat(current_limited_rpm,
                            limited,
                            cfg.motion.speed_slew_rate_rpm_s * dt);  // RPM/s × s = RPM 每 tick 允许最大变化量
    }

    return limited;
}

/*
 * limitIqReference — Iq 目标限幅: 三级保护
 *
 *   1. 绝对值钳位: clamp 到 [-iq_limit, iq_limit]
 *   2. 超速硬切除: 已到达限速且仍在同向加速 → Iq = 0
 *   3. 转矩保护带: 转矩模式下进 guard_band 后按线性比例削减 Iq (0→1)
 */
float MotorTargetLimiter::limitIqReference(const MotorConfig& cfg,
                                           float speed_rpm,
                                           float iq_ref)
{
    // [1] 绝对值钳位
    const float iq_limit = configuredIqLimit(cfg);
    if (iq_limit > 0.0f)
    {
        iq_ref = clampFloat(iq_ref, -iq_limit, iq_limit);
    }

    if (cfg.limit.max_speed_rpm <= 0.0f)
    {
        return iq_ref;                         // 无限速 → 跳过后续检查
    }

    const float speed_abs = fabsf(speed_rpm);
    const bool accelerates_current_direction = (iq_ref * speed_rpm) > 0.0f;  // Iq 与转速同向 = 仍在加速

    // [2] 超速硬切除
    if (speed_abs >= cfg.limit.max_speed_rpm && accelerates_current_direction)
    {
        return 0.0f;                           // 已达上限且同向 → 切断全部 Iq
    }

    // [3] 转矩保护带: 本轮已扩到所有模式 (取代仅TORQUE),
    // 速度环 PI 自身也能受益于 guard_band 减速进带时削减同向 Iq 命令
    if (cfg.motion.torque_speed_guard_band_rpm > 0.0f &&
        accelerates_current_direction)
    {
        const float guard_start = cfg.limit.max_speed_rpm - cfg.motion.torque_speed_guard_band_rpm;
        if (speed_abs > guard_start)            // 已进入保护带
        {
            const float scale =
                (cfg.limit.max_speed_rpm - speed_abs) / cfg.motion.torque_speed_guard_band_rpm;
            iq_ref *= clampFloat(scale, 0.0f, 1.0f);  // 线性削减: guard_start 处 scale=1, max_speed 处 scale=0
        }
    }

    return iq_ref;
}

/*
 * applyIqSlewRate — Iq 电流斜率限制: 双向非对称
 *
 * 作用: 抑制相邻 tick 之间的电流台阶, 减小响应过冲
 * 双向非对称 (本轮新增):
 *   驱动方向 (Iq 朝 request 同向变化, 即力矩朝目标加速) 用 iq_slew_rate_a_per_s;
 *   制动方向 (Iq 朝 request 反向变化, 即卸载救援/反向制动)
 *     用 iq_slew_rate_brake_a_per_s (> 0 时生效; =0 回退用 iq_slew_rate_a_per_s)。
 *
 * slew_rate = 0 时直通不限制 (保留旧行为)。
 */
float MotorTargetLimiter::applyIqSlewRate(const MotorConfig& cfg,
                                          float dt,
                                          float current_limited_iq,
                                          float requested_iq)
{
    const float drive_slew = cfg.motion.iq_slew_rate_a_per_s;
    // 制动斜率: 0 = 回退用驱动斜率 (兼容老 cfg); >0 优先用
    float brake_slew = cfg.motion.iq_slew_rate_brake_a_per_s > 0.0f
                       ? cfg.motion.iq_slew_rate_brake_a_per_s
                       : drive_slew;

    // 当前为 0 时, 把"驱动"理解为加速, "制动"为反向 Iq
    // 判定依据: 请求方向是否与当前 Iq 反号 = 制动 (卸载救援);否则驱动
    // current_iq ≈ 0 时按 requested 符号判定。
    bool is_braking = (current_limited_iq > 1e-6f && requested_iq < current_limited_iq - 1e-6f)
                   || (current_limited_iq < -1e-6f && requested_iq > current_limited_iq + 1e-6f);
    const float slew = is_braking ? brake_slew : drive_slew;

    if (slew <= 0.0f)
    {
        return requested_iq;                     // 不限制 → 直通
    }

    return slewFloat(current_limited_iq,
                     requested_iq,
                     slew * dt);  // A/s × s = A 每 tick 允许最大变化
}

/*
 * modulationKmod — 调制系数 (电压矢量长度相对 Vbus 的最大利用率)
 *
 * SPWM    无共模注入, 线性区矢量上限 = Vbus/2, K_mod = 0.5
 * SVPWM   min-max 共模注入, 线性区矢量上限 = Vbus/√3, K_mod = 1/√3 ≈ 0.5774
 * THIPWM  1/3 三次谐波注入, 线性区矢量上限 = Vbus/√3, 与 SVPWM 同
 * SIXSTEP 方波换相, 线电压 ≈ Vbus (理论), K_mod = 1.0 (占位, 实际有死区/压降损失)
 */
float MotorTargetLimiter::modulationKmod(ModulationMethod m)
{
    switch (m)
    {
        case ModulationMethod::SPWM:    return 0.5f;
        case ModulationMethod::SVPWM:   return 0.5773502691f;   // 1/√3
        case ModulationMethod::THIPWM:  return 0.5773502691f;
        case ModulationMethod::SIXSTEP: return 1.0f;
        default:                        return 0.5773502691f;
    }
}

/*
 * limitVoltageVector — d/q 轴联合电压矢量限幅 (圆限制)
 *
 * 原理: 真正的物理约束是 √(vd² + vq²) ≤ Vs_max, 不是各自 ±10V 方限制。
 * Vs_max = K_mod × Vbus, 随母线动态变化, 解决固定 10V 与宽 VBUS 失配问题。
 *
 * Ke 不在此处扣减: 当前控制链尚未把 ωKe 作为电压前馈加回 vq。
 * 若直接用 Vs_max - |ω|Ke 限制最终输出, 会把克服反电势所需的电压也压掉。
 * 后续接入反电势/交叉耦合前馈后, 再在前馈占用电压基础上做剩余电压预算。
 */
void MotorTargetLimiter::limitVoltageVector(const MotorConfig& cfg,
                                             float vbus,
                                             float omega_elec_rad_s,
                                             float& vd,
                                             float& vq)
{
    (void)omega_elec_rad_s;

    const float K_mod = modulationKmod(cfg.control.modulation);
    const float Vs_max = K_mod * (vbus > 0.0f ? vbus : 0.0f);
    if (Vs_max <= 0.0f)
    {
        vd = 0.0f; vq = 0.0f;
        return;
    }

    const float Vsq = vd * vd + vq * vq;
    if (Vsq <= Vs_max * Vs_max) return;  // 在圆内无需缩放

    const float Vmag = sqrtf(Vsq);
    if (Vmag < 1e-9f) { vd = 0.0f; vq = 0.0f; return; }

    const float scale = Vs_max / Vmag;
    vd *= scale;
    vq *= scale;
}

/*
 * isOverspeed — 超速故障判定
 *
 *   |speed_rpm| > max_speed_rpm × overspeed_factor → 触发 Fault::OVERSPEED
 *   仅作判定, 真正锁故障由 Motor_RuntimeMonitor 完成。
 */
bool MotorTargetLimiter::isOverspeed(const MotorConfig& cfg, float speed_rpm)
{
    if (cfg.limit.max_speed_rpm <= 0.0f || cfg.motion.overspeed_factor <= 0.0f) return false;
    const float threshold = cfg.limit.max_speed_rpm * cfg.motion.overspeed_factor;
    return fabsf(speed_rpm) > threshold;
}

/*
 * loadStepGuard — 负载突变飞车预兆监控
 *
 * 工作原理:
 *   每 tick 计算 dω/dt = (ω_now - ω_prev)/dt;
 *   若 |dω/dt| > spike_threshold (远超正常加速度)
 *     且 dω/dt 与当前 Iq 方向相反 (即负载撤除飞车征兆)
 *   → 主动反向 Iq 抢先制动 N ticks, 不等速度环反应。
 *
 * state_inout 由调用方持有 (0 初始化), > 0 时表示剩余 override tick 数。
 *
 * 返回:
 *   > 0 → 已 override (override_tick > 0), 调用方接管 Iq;
 *   0  → 不干预, 用原 iq_ref_current;
 */
float MotorTargetLimiter::loadStepGuard(const MotorConfig& cfg,
                                         float dt,
                                         float speed_rpm_now,
                                         float speed_rpm_prev,
                                         float iq_ref_current,
                                         int32_t& state_inout)
{
    // state 持续阶段直接 override
    if (state_inout > 0)
    {
        --state_inout;
        const float iq_limit = configuredIqLimit(cfg);
        const float brake_iq = -copysignf(iq_limit * 0.5f, speed_rpm_now);
        return brake_iq;
    }

    if (dt <= 1e-6f) return 0.0f;

    const float domega = (speed_rpm_now - speed_rpm_prev) / dt;  // RPM/s
    // spike_threshold: 默认 3 倍 speed_slew_rate_rpm_s, 至少 500 RPM/s
    float spike_threshold = 500.0f;
    if (cfg.motion.speed_slew_rate_rpm_s > 0.0f)
        spike_threshold = cfg.motion.speed_slew_rate_rpm_s * 3.0f;
    if (spike_threshold < 500.0f) spike_threshold = 500.0f;

    if (fabsf(domega) < spike_threshold) return 0.0f;

    // 飞车征兆: dω 与当前 Iq 反向 (Iq 已不能跟着加速)
    const bool flying = (iq_ref_current * domega) < 0.0f;
    if (!flying) return 0.0f;

    // 进入 override, 持续 ~5 tick (250 μs @ 20 kHz)
    state_inout = 5;
    const float iq_limit = configuredIqLimit(cfg);
    const float brake_iq = -copysignf(iq_limit * 0.5f, speed_rpm_now);
    return brake_iq;
}

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
/*
 * isStallCandidate — 堵转候选单 tick 判定
 *
 * 三条件同时满足:
 *   1. 低速:    |转速| < speed_abs_rpm              — 几乎不转
 *   2. 大误差: |速度误差| > speed_error_rpm          — 命令转但实际未跟上
 *   3. 饱和:   |Iq| ≥ iq_limit × iq_limit_ratio     — 已输出极限力矩
 *
 * 返回 true 仅表示"候选", 持续确认由 updateStallProtection() 完成。
 */
bool MotorTargetLimiter::isStallCandidate(const MotorConfig& cfg,
                                          float speed_rpm,
                                          float speed_error_rpm,
                                          float iq_ref,
                                          float iq_limit)
{
    const auto& stall = cfg.motion.stall;
    if (!stall.enable || iq_limit <= 0.0f)       // 堵转未启用或 Iq 上限无效 → 不判定
    {
        return false;
    }

    const bool low_speed = fabsf(speed_rpm) < stall.speed_abs_rpm;                      // 条件 1: 几乎静止
    const bool large_error = fabsf(speed_error_rpm) > stall.speed_error_rpm;            // 条件 2: 大速度误差
    const bool iq_saturated = fabsf(iq_ref) >= (iq_limit * stall.iq_limit_ratio);       // 条件 3: 力矩饱和
    return low_speed && large_error && iq_saturated;                                     // 三条件同时满足
}
#endif

} // namespace Lib_Motor
