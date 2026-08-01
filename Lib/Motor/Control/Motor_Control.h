/*
 * Motor_Control.h — PID 控制器持有者
 *
 * 当前持有 D/Q 轴电流环 PID 和速度环 PID
 * 位置环 PID 暂未接入执行链
 *
 * 扩展方向: 持有速度环 PID + 位置环 PID 实例
 */

#pragma once

#include "../Public/Motor_Config.h"
#include "../Common/Filter/PidController.h"
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../Core/Numeric/Motor_FixedNumeric.h"
#endif

namespace Lib_Motor
{

/*
 * MotorPIAutoTune — 电流/速度环 PI 增益自动推导
 *
 * 电流环 (本轮接入):
 *   电流环对象方程: L·dI/dt + R·I = V
 *   设期望闭环带宽 f_bw (Hz), 使用保守一阶带宽整定:
 *     ωc = 2πf_bw (rad/s)
 *     kp = L*ωc                 [单位 V/A]
 *     ki = R*ωc                 [单位 V/(A·s)] (与积分项 integral += ki*error*dt 一致)
 *
 * 速度环 (本轮仅声明, P5 落地):
 *   推导需要机械侧 J/torque_constant; 未实现 → ConfigCheck 拒绝 (auto_derive_speed_pid=true)。
 */
class MotorPIAutoTune
{
public:
    /* 电流环增益推导 (本轮完整实现) */
    static void deriveCurrentLoopGains(
        float bandwidth_hz, float zeta,
        float rs, float ls,
        float& out_kp, float& out_ki)
    {
        const float omega_c = 2.0f * 3.14159265f * bandwidth_hz;
        if (bandwidth_hz <= 0.0f || zeta <= 0.0f || rs <= 0.0f || ls <= 0.0f)
        {
            out_kp = 0.0f;
            out_ki = 0.0f;
            return;
        }
        out_kp = omega_c * ls;                    // [V/A]
        out_ki = omega_c * rs;                    // [V/(A·s)] 直接积分关系
    }

    /* 速度环增益推导 (本轮仅占位, P5 在 cfg 物理惯量数据完备后落地)
     * 需要 cfg.physical.load_inertia_kg_m2 > 0 与 torque_constant > 0 才有意义;
     * 当前 ConfigCheck 已拒绝 auto_derive_speed_pid=true, 走到此项应返错误。
     */
    static void deriveSpeedLoopGains(
        float bandwidth_hz, float zeta,
        float torque_constant_n_m_per_a,
        float inertia_kg_m2,
        float& out_kp, float& out_ki)
    {
        (void)bandwidth_hz; (void)zeta;
        (void)torque_constant_n_m_per_a; (void)inertia_kg_m2;
        out_kp = 0.0f; out_ki = 0.0f;
    }

    /* [P4] 位置环增益推导占位 (P5 落地)
     * 需要 cfg.physical.load_inertia > 0 与 torque_constant > 0;
     * 当前 ConfigCheck 已拒绝 auto_derive_position_pid=true。
     */
    static void derivePositionLoopGains(
        float bandwidth_hz, float zeta,
        float torque_constant_n_m_per_a,
        float inertia_kg_m2,
        float& out_kp, float& out_ki)
    {
        (void)bandwidth_hz; (void)zeta;
        (void)torque_constant_n_m_per_a; (void)inertia_kg_m2;
        out_kp = 0.0f; out_ki = 0.0f;
    }
};

class MotorControl
{
public:
    MotorControl() = default;

    /* 根据 MotorConfig 初始化 PID 参数 */
    void init(const MotorConfig& cfg)
    {
        /* === 电流环 PI 增益自动推导 (本轮启用) ===
         * auto_derive_current_pid=true 时:
         *   - 取 effective R 与 d/q 轴电感, Ld/Lq 未填时自动回退到标量 Ls;
         *   - 用 MotorPIAutoTune::deriveCurrentLoopGains(bw, zeta, R, Lx, kp, ki);
         *   - 用推导结果分别覆盖 current_d/current_q 的 PIDParam 副本;
         *   - 电流环 output_limit 由运行期 Vbus 动态覆盖;
         * 若 effective R/L 不足(eff==0): 直接退化用手填 kp/ki, 并等待 RL 辨识后再覆盖。
         */
        PIDParam pid_d_p = cfg.control.current_d;
        PIDParam pid_q_p = cfg.control.current_q;

        if (cfg.control.auto_derive_current_pid)
        {
            pid_d_p.output_limit = 0.0f;
            pid_q_p.output_limit = 0.0f;
            const float R = cfg.physical.rs_effective();
            const float Ld = cfg.physical.ld_effective_for_current_loop();
            const float Lq = cfg.physical.lq_effective_for_current_loop();
            if (R > 0.0f && Ld > 0.0f)
            {
                float kp = 0.0f, ki = 0.0f;
                MotorPIAutoTune::deriveCurrentLoopGains(
                    cfg.control.current_loop_bandwidth_hz,
                    cfg.control.current_loop_damping_ratio,
                    R, Ld, kp, ki);
                pid_d_p.kp = kp; pid_d_p.ki = ki;
            }
            if (R > 0.0f && Lq > 0.0f)
            {
                float kp = 0.0f, ki = 0.0f;
                MotorPIAutoTune::deriveCurrentLoopGains(
                    cfg.control.current_loop_bandwidth_hz,
                    cfg.control.current_loop_damping_ratio,
                    R, Lq, kp, ki);
                pid_q_p.kp = kp; pid_q_p.ki = ki;
            }
            /* R/L 不足时静默退化:
             * init 阶段保留手填 kp/ki, RL 辨识完成后由 applyAutoDeriveIfEnabled() 覆盖。
             */
        }

        pid_d.init(pid_d_p);     // D 轴电流环
        pid_q.init(pid_q_p);     // Q 轴电流环
        pid_speed.init(cfg.control.speed);   // 速度环: 输出 Iq 目标
        pid_position.init(cfg.control.position); // 位置环: 输出速度目标 (rad/s)

        /* 速度环自动推导 (本轮仅预留字段, ConfigCheck 已拒绝 auto_derive_speed_pid=true;
         * 走到此处保留手填 kp/ki 即可。P5 阶段在此处加 deriveSpeedLoopGains 调用。
         */
    }

    void reset()
    {
        pid_d.reset();
        pid_q.reset();
        pid_speed.reset();
        pid_position.reset();
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        fixed_pid_d_.reset();
        fixed_pid_q_.reset();
        fixed_pid_speed_.reset();
#endif
    }

    /* [P2] RL 辨识完成后回调: 若 auto_derive_current_pid=true,
     * 用新 effective R/Ld/Lq 推导 kp/ki, 分别同步到 pid_d/q;
     * 在 MotorRLIdentifyRoutine::step Phase::FINALIZE 中调用。
     */
    void applyAutoDeriveIfEnabled(const MotorConfig& cfg)
    {
        if (!cfg.control.auto_derive_current_pid) return;
        const float R = cfg.physical.rs_effective();
        const float Ld = cfg.physical.ld_effective_for_current_loop();
        const float Lq = cfg.physical.lq_effective_for_current_loop();
        if (R <= 0.0f) return;

        // 同步覆盖 kp/ki 需暴露 setter 或重建 PIDParam; 此处保守做法:
        // 通过 reset+init 装载新增益 (清零积分, 避免冷切换抖动)
        if (Ld > 0.0f)
        {
            float kp = 0.0f, ki = 0.0f;
            MotorPIAutoTune::deriveCurrentLoopGains(
                cfg.control.current_loop_bandwidth_hz,
                cfg.control.current_loop_damping_ratio,
                R, Ld, kp, ki);
            PIDParam p = cfg.control.current_d;
            p.kp = kp; p.ki = ki;
            p.output_limit = 0.0f;
            pid_d.init(p);
        }
        if (Lq > 0.0f)
        {
            float kp = 0.0f, ki = 0.0f;
            MotorPIAutoTune::deriveCurrentLoopGains(
                cfg.control.current_loop_bandwidth_hz,
                cfg.control.current_loop_damping_ratio,
                R, Lq, kp, ki);
            PIDParam p = cfg.control.current_q;
            p.kp = kp; p.ki = ki;
            p.output_limit = 0.0f;
            pid_q.init(p);
        }
    }

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    void syncFixedCurrentPid(const MotorConfig& cfg, float dt, float vbus)
    {
        const float current_base = currentBase(cfg);
        const float voltage_base = (vbus > 0.1f) ? vbus : 0.1f;
        fixed_pid_d_.configure(pid_d.kp(), pid_d.ki(), current_base, voltage_base,
                               dt, pid_d.outputLimit(), false);
        fixed_pid_q_.configure(pid_q.kp(), pid_q.ki(), current_base, voltage_base,
                               dt, pid_q.outputLimit(), false);
    }

    void syncFixedSpeedPid(const MotorConfig& cfg, float dt, float output_limit_a)
    {
        fixed_pid_speed_.configure(pid_speed.kp(), pid_speed.ki(), speedBase(cfg),
                                   currentBase(cfg), dt, output_limit_a, false);
    }

    FixedNumeric::q15_t updateFixedCurrentD(FixedNumeric::q15_t error)
    {
        return fixed_pid_d_.update(error);
    }

    FixedNumeric::q15_t updateFixedCurrentQ(FixedNumeric::q15_t error)
    {
        return fixed_pid_q_.update(error);
    }

    float updateFixedSpeed(const MotorConfig& cfg, float speed_error_rpm, bool hold_integral)
    {
        const FixedNumeric::q15_t error =
            FixedNumeric::fromPhysical(speed_error_rpm, speedBase(cfg));
        const FixedNumeric::q15_t output = fixed_pid_speed_.update(error, hold_integral);
        return FixedNumeric::toPhysical(output, currentBase(cfg));
    }

    void decayFixedSpeedIntegral(float factor)
    {
        fixed_pid_speed_.decayIntegral(factor);
    }

    static float currentBase(const MotorConfig& cfg)
    {
        if (cfg.physical.rated_current > 0.0f)
        {
            return cfg.physical.rated_current;
        }
        return (cfg.limit.max_current_a > 0.0f) ? cfg.limit.max_current_a : 1.0f;
    }

    static float speedBase(const MotorConfig& cfg)
    {
        return (cfg.limit.max_speed_rpm > 0.0f) ? cfg.limit.max_speed_rpm : 1.0f;
    }
#endif

    PidController pid_d;     // D 轴电流 PI 控制器
    PidController pid_q;     // Q 轴电流 PI 控制器
    PidController pid_speed; // 速度 PI 控制器, 输出 Q 轴电流目标
    PidController pid_position; // 位置 PI 控制器, 输出速度目标
    /*
     * 注 (Phase 3 预留): IMPEDANCE_CONTROL 将复用本实例的 Kp/Kd 表达
     * stiffness/torque_constant 与 damping/torque_constant。届时把
     * pid_position.init 换成"按 MotorImpedanceParam 折算 PID gain"的入口。
     */
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
private:
    FixedNumeric::FixedPiController fixed_pid_d_;
    FixedNumeric::FixedPiController fixed_pid_q_;
    FixedNumeric::FixedPiController fixed_pid_speed_;
#endif
};

} // namespace Lib_Motor
