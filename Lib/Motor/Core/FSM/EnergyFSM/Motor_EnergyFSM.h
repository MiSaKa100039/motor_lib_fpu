/*
 * Motor_EnergyFSM.h — 能量/制动回收 FSM 声明
 *
 * 职责:
 *   管理制动过程中的能量流向, 决定 Iq 电流是否可以用于再生制动,
 *   并根据能源回收路径的存在性限制制动电流。
 *
 * 能量状态 (EnergyState):
 *   IDLE          — 空闲 (无能量流动)
 *   DRIVE         — 驱动状态 (Iq 与速度同向)
 *   BLOCKED       — 制动被阻止 (不允许再生, 无回收路径)
 *   REGEN_ACTIVE  — 再生制动进行中 (能量回充电池)
 *   DUMP_ACTIVE   — 能耗制动进行中 (能量经制动电阻耗散)
 *
 * 核心判断: isBrakingCurrent(iq, speed)
 *   speed × iq < 0 → 制动 (Iq 与速度方向相反, 磁场力阻碍转动)
 *   speed × iq > 0 → 驱动 (Iq 与速度方向相同, 推动转动)
 */

#pragma once

namespace Lib_Motor
{

class MotorManager;

class Motor_EnergyFSM
{
public:
    /*
     * step — 非 RUN/STOPPING 状态下将能量状态恢复为 IDLE
     */
    static void step(MotorManager& m);

    static void clear(MotorManager& m);

/*
 * limitIqReference — 根据能量承接路径 + budget 限制 Iq 制动电流
 *
 * 物理决策树:
 *   Iq ≈ 0                → IDLE, return 0
 *   Iq × speed > 0        → DRIVE, return requested (驱动方向不放限制)
 *   Iq × speed < 0 (制动):
 *     1. 闸门: allow_foc_negative_iq 必须 true 才允许任何反向 Iq
 *     2. 上层 budget 是否注入 (budget_active_)?
 *         true 且未过期 → 按 budget 动态判 regen/dump 可用性与 max_*_iq_a
 *         超时 BLOCK   → BLOCKED, 直到上层重新注入有效 budget
 *         否则          → 按 cfg.brake.has_*_path 静态走
 *     3. 能量承接路径:
 *         两条路径都可用 → 按 BrakeEnergyPathPolicy 分流
 *         单一路径可用  → 进入对应 EnergyState
 *         两者都不可用           → BLOCKED (钳零, 保护母线不充爆)
 *
 *  注: LOW_SIDE_BRAKE 不在本 FSM 调用, 它属于 StopFSM::ELECTRICAL_BRAKE 主动停机路径,
 *      与运行期负 Iq 的承接路径解耦。
 */
static float limitIqReference(MotorManager& m,
                              float requested_iq, float speed_rpm);

private:
    static bool isBrakingCurrent(float iq_ref, float speed_rpm);

    static float configuredBrakeIqLimit(const MotorManager& m);

    /*
     * budgetValid — budget 是否注入且未过期
     *   未注入或过期: 返回 false
     *   过期时按 cfg.brake.budget_loss_behavior 决定是否阻断负 Iq
     */
    static bool budgetValid(MotorManager& m);
};

} // namespace Lib_Motor
