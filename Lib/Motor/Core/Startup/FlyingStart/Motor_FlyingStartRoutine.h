#pragma once

#include <stdint.h>

/* 前向声明避免循环 include */
namespace Lib_Motor { class MotorManager; }

namespace Lib_Motor
{

/*
 * Motor_FlyingStartRoutine -- 顺逆风启动内部流程 (相电压反电势 PLL 重构转子角度) [P2]
 *
 * 触发条件:
 *   1. cfg.observer.flying_start_mode == PHASE_VOLTAGE
 *   2. cfg.sensor.has_phase_voltage == true 且 BuildCfg MOTOR_BUILD_HAS_PHASE_VOLTAGE
 *   3. start() 时检测 |speed_rpm| > 辨识阈值 (能产生足够反电势), 默认 100 RPM
 *
 * 工作流程:
 *   Phase IDLE → 启动 start() 在 handleAlignment 前检查 kinematics (有速度?)
 *      ─ ─ yes (|ω| > threshold) → set run_phase = RunPhase::FLYING_START (反向注入 RunPhase 枚举未新增, 简化用 FSM 状态机)
 *   Phase WAIT_BEMF: 维持 PWM coast 关闭功率管 (避免强迫 IF 拉回转子), 仅采样相电压:
 *      v_alpha = (v_u - v_w)/3 ... 经 Clarke 简化
 *      v_beta  = (v_u - 2v_v + v_w)/3 / sqrt(3)
 *      用 atan2(v_beta, -v_alpha) 提取相位滑差, 形成简单 PLL 跟踪 (复用 SMO PLL kp/ki)
 *   Phase SEED_SMO: 等 PLL locked_ticks (500 tick = 25 ms @20kHz) + speed 估计稳定,
 *      把 SMO 的 angle_est_ / speed_est_ 直接置为 PLL 输出 → 设 angle 当前控制角 = PLL 角
 *      → 调 RunPhaseFSM setRunPhase(SMO_ONLY), 跳过 ALIGNMENT/FORCE_DRAG
 *   Phase DONE: 置 event_.flying_start_done = 1, 之后正常 SMO_ONLY 控制流接管
 *
 * 失败兜底:
 *   - PLL 不收敛超时 (timeout_ticks = 100 ms) → 发 fault 走回 STOP (BRAKE_ON_LOSS 由 user cfg 决定)
 *   - 若 |ω| 不够阈值, handleAlignment 前已完成 handled: 不进入 FSM, 走标准 ALIGNMENT 流程
 *
 * 说明:
 *   本类是由 RunPhaseFSM 触发的内部分阶段流程, 不是 Core/FSM 架构层状态机。
 *
 * 编译开关: LIB_MOTOR_ENABLE_FLYING_START (BuildCfg 默认关)
 */

class MotorFlyingStartRoutine
{
public:
    enum class Phase : uint8_t
    {
        IDLE = 0,
        WAIT_BEMF,         // 等待相电压反电势 PLL 锁相
        SEED_SMO,           // PLL 已锁, 把角度/速度灌入 SMO 内部 state
        DONE,
    };

    void reset()
    {
        phase_           = Phase::IDLE;
        tick_counter_    = 0U;
        pll_kp_          = 2.0f;
        pll_ki_          = 50.0f;
        pll_integral_    = 0.0f;
        angle_est_       = 0.0f;
        speed_est_       = 0.0f;
        last_angle_obs_  = 0.0f;
        locked_ticks_    = 0U;
    }

    /* 一个控制 tick 调用一次, 由 MotorManager::tick 在 RUN + run_phase==FLYING_START 时调用 */
    void step(MotorManager& m);

    bool done() const { return phase_ == Phase::DONE; }

private:
    Phase   phase_             = Phase::IDLE;
    uint32_t tick_counter_    = 0U;
    float   pll_kp_           = 2.0f;     // PLL 比例 (取自 SMO 同档)
    float   pll_ki_           = 50.0f;    // PLL 积分
    float   pll_integral_    = 0.0f;
    float   angle_est_       = 0.0f;     // 跟踪到的转子电角度
    float   speed_est_       = 0.0f;     // 跟踪到的电角速度 rad/s
    float   last_angle_obs_  = 0.0f;
    uint16_t locked_ticks_   = 0U;
};

} // namespace Lib_Motor
