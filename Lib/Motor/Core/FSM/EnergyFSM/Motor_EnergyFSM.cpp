/*
 * Motor_EnergyFSM.cpp — 能量/制动回收 FSM 实现
 */

#include "Motor_EnergyFSM.h"

#include "../../../Public/Motor_Features.h"

#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM

#include "../../Manager/Motor_Manager.h"
#include "../../Limit/Motor_TargetLimiter.h"

#include <cmath>

namespace Lib_Motor
{

namespace
{
/*
 * clampMagnitude — 绝对值钳位: 限制 value 在 [-max_abs, max_abs] 区间
 * max_abs ≤ 0 表示不限制
 */
float clampMagnitude(float value, float max_abs)
{
    if (max_abs <= 0.0f)
    {
        return value;
    }

    if (value > max_abs) return max_abs;       // 正向超限 → 钳到上限
    if (value < -max_abs) return -max_abs;     // 负向超限 → 钳到下限
    return value;
}

float minPositive(float primary, float candidate)
{
    if (candidate <= 0.0f)
    {
        return primary;
    }
    if (primary <= 0.0f || candidate < primary)
    {
        return candidate;
    }
    return primary;
}

float staticPathLimit(float hard_limit, float path_limit)
{
    return minPositive(hard_limit, path_limit);
}

float dynamicPathLimit(float hard_limit, float path_limit)
{
    if (path_limit <= 0.0f)
    {
        return 0.0f;
    }
    return minPositive(hard_limit, path_limit);
}

} // namespace

/*
 * step — 非 RUN/STOPPING 状态下恢复能量状态为 IDLE
 * (制动仅在运行或正在停止时有效)
 */
void Motor_EnergyFSM::step(MotorManager& m)
{
    if (m.state() != State::RUN && m.state() != State::STOPPING)
    {
        m.energy_state_ = EnergyState::IDLE;                // 不在运行/停止态 → 空闲
    }
}

void Motor_EnergyFSM::clear(MotorManager& m)
{
    m.energy_state_ = EnergyState::IDLE;
}

/*
 * limitIqReference — 根据能量承接路径 + budget 限制制动 Iq 电流
 *
 * 决策顺序:
 *   1. Iq ≈ 0 → IDLE, return 0
 *   2. Iq × speed > 0 (驱动) → DRIVE, return requested
 *   3. Iq × speed < 0 (制动):
 *      闸门: allow_foc_negative_iq 必须 true, 否则 BLOCKED
 *      按 budget 注入状态决定动态判 / 静态判:
 *        budget_active  → 用 budget.regen_allowed/dump_allowed + 限额
 *        否则            → 用 cfg.brake.has_battery_regen_path / has_brake_resistor + 静态限
 *      路径可用性:
 *        budget_active: regen/dump 必须 allowed 且 max_*_iq_a > 0
 *        静态 cfg: has_*_path 可用, per-path Iq=0 时回退全局制动 Iq
 *        若两者均不可用 → BLOCKED, return 0
 *      电流上限: min(max_brake_iq_a, max_regen_iq_a/max_dump_iq_a)
 */
float Motor_EnergyFSM::limitIqReference(MotorManager& m,
                                        float requested_iq,
                                        float speed_rpm)
{
    // 1) 电流几乎为零 → IDLE
    if (fabsf(requested_iq) <= 1e-6f)
    {
        m.energy_state_ = EnergyState::IDLE;
        return 0.0f;
    }

    // 2) 驱动电流 (Iq 与速度同向) → 不限制
    if (!isBrakingCurrent(requested_iq, speed_rpm))
    {
        m.energy_state_ = EnergyState::DRIVE;
        return requested_iq;
    }

    // 3) 制动电流
    if (!m.config_.brake.allow_foc_negative_iq)
    {
        m.energy_state_ = EnergyState::BLOCKED;
        return 0.0f;
    }

    const bool use_budget = budgetValid(m);
    if (!use_budget && m.budget_loss_blocked_)
    {
        m.energy_state_ = EnergyState::BLOCKED;
        return 0.0f;
    }

    // 3a) 取两条硬件承接路径的当前可用性
    bool regen_path_ok = use_budget
        ? m.energy_budget_.regen_allowed && !m.energy_budget_.source_fault
        : m.config_.brake.has_battery_regen_path;

    bool dump_path_ok = use_budget
        ? m.energy_budget_.dump_allowed && !m.energy_budget_.source_fault
        : m.config_.brake.has_brake_resistor;

#if !LIB_MOTOR_ENABLE_BATTERY_REGEN_PATH
    regen_path_ok = false;
#endif
#if !LIB_MOTOR_ENABLE_BRAKE_RESISTOR
    dump_path_ok = false;
#endif

    // 3b) 取 Iq 电流上限: 动态 budget 下 0 表示该路径不可用
    const float hard_limit = configuredBrakeIqLimit(m);
    float regen_limit = use_budget
        ? dynamicPathLimit(hard_limit, m.energy_budget_.max_regen_iq_a)
        : staticPathLimit(hard_limit, m.config_.brake.max_regen_iq_a);
    float dump_limit = use_budget
        ? dynamicPathLimit(hard_limit, m.energy_budget_.max_dump_iq_a)
        : staticPathLimit(hard_limit, m.config_.brake.max_dump_iq_a);

    if (regen_limit <= 0.0f)
    {
        regen_path_ok = false;
    }
    if (dump_limit <= 0.0f)
    {
        dump_path_ok = false;
    }

    if (use_budget)
    {
        // 制动电阻温度保护 (仅 dump 路径)
        if (m.energy_budget_.resistor_temp_limit_c > 0.0f &&
            m.energy_budget_.resistor_temp_c >= m.energy_budget_.resistor_temp_limit_c)
        {
            dump_path_ok = false;
        }
    }

    // 3c) 按策略分流。DC 电流/功率字段只作为外部能量侧边界, 不在此处换算为 Iq。
    if (regen_path_ok && dump_path_ok)
    {
        BrakeEnergyPathPolicy policy = m.config_.brake.energy_path_policy;
        if (use_budget)
        {
            policy = m.energy_budget_.energy_path_policy;
        }

        bool choose_dump = (policy == BrakeEnergyPathPolicy::DUMP_FIRST);
        if (policy == BrakeEnergyPathPolicy::DUMP_ON_OVERVOLT)
        {
            const float soft_limit = (use_budget && m.energy_budget_.vbus_limit_v > 0.0f)
                ? m.energy_budget_.vbus_limit_v
                : m.config_.limit.over_voltage_v;
            choose_dump = (soft_limit > 0.0f && m.ctx_.v_bus >= soft_limit);
        }

        if (choose_dump)
        {
            m.energy_state_ = EnergyState::DUMP_ACTIVE;
            return clampMagnitude(requested_iq, dump_limit);
        }

        m.energy_state_ = EnergyState::REGEN_ACTIVE;
        return clampMagnitude(requested_iq, regen_limit);
    }
    if (regen_path_ok)
    {
        m.energy_state_ = EnergyState::REGEN_ACTIVE;
        return clampMagnitude(requested_iq, regen_limit);
    }
    if (dump_path_ok)
    {
        m.energy_state_ = EnergyState::DUMP_ACTIVE;
        return clampMagnitude(requested_iq, dump_limit);
    }

    // 无任何承接路径 → 防母线过压, 钳零
    m.energy_state_ = EnergyState::BLOCKED;
    return 0.0f;
}

bool Motor_EnergyFSM::isBrakingCurrent(float iq_ref, float speed_rpm)
{
    return (iq_ref * speed_rpm) < 0.0f;
}

float Motor_EnergyFSM::configuredBrakeIqLimit(const MotorManager& m)
{
    if (m.config_.brake.max_brake_iq_a > 0.0f)
    {
        return m.config_.brake.max_brake_iq_a;
    }
    return MotorTargetLimiter::configuredIqLimit(m.config_);
}

/*
 * budgetValid — budget 是否注入且未过期
 *   过期时按 cfg.brake.budget_loss_behavior 决定后续动作:
 *     BLOCK_ON_LOSS/BRAKE_ON_LOSS: 清 budget_active_, latch event, 阻断负 Iq
 *     FALLBACK_TO_STATIC/HOLD_ON_LOSS: 清 budget_active_, 回退静态 cfg
 *   未注入过则静默返回 false (不 latch)
 */
bool Motor_EnergyFSM::budgetValid(MotorManager& m)
{
    if (!m.budget_active_)
    {
        return false;
    }

    const uint32_t now = m.fsmTimerTicks();
    const bool expired = (now > m.budget_expire_ticks_);  // 简单 wrap 容忍: 32bit @10kHz ~ 4.3 天

    if (!expired)
    {
        return true;
    }

    // 已过期: 按 behavior 决定回退语义
    m.budget_active_ = false;
    m.budget_loss_blocked_ =
        (m.config_.brake.budget_loss_behavior == BudgetLossBehavior::BLOCK_ON_LOSS);
    if (m.budget_loss_blocked_)
    {
        m.event_.budget_expired = 1U;  // latched event, 上层可读 getMonitorData 看到
    }
    return false;
}

} // namespace Lib_Motor

#endif
