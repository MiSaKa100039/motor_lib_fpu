#pragma once

#include <stdint.h>
#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor { class MotorManager; }

namespace Lib_Motor
{

enum class RLIdentifyPhase : uint8_t
{
    IDLE = 0,
    SEARCH_RAMP,       // 自动搜索 Vstep: 斜坡注入并观察电流响应
    MEASURE_PREPARE,   // 正式采样前短暂卸压, 避免搜索残余电流影响 di/dt
    INJECT_STEP,       // L 固定 10ms 物理窗口: 用阶跃后的 di/dt 估算电感
    STEADY_STATE,      // R 采样窗口: 跳过前段过渡后平均 V/I
    FINALIZE,
    DONE,
};

enum class RLIdentifyFailReason : uint8_t
{
    NONE = 0,
    CURRENT_NAN,
    SIGNAL_TOO_LOW_AT_VOLTAGE_LIMIT,
    L_DI_TOO_SMALL,
    R_CURRENT_TOO_SMALL,
    ESTIMATE_INVALID,
    OVERCURRENT,
};

enum RLIdentifyQualityWarning : uint16_t
{
    RL_QUALITY_NONE = 0U,
    RL_QUALITY_VOLTAGE_LIMIT_BELOW_TARGET = 1U << 0,
    RL_QUALITY_REPEAT_SPREAD_HIGH = 1U << 1,
    RL_QUALITY_CURRENT_NEAR_ABORT = 1U << 2,
    RL_QUALITY_R_SETTLE_TIMEOUT = 1U << 3,
};

struct RLIdentifyDebugSnapshot
{
    volatile RLIdentifyPhase phase = RLIdentifyPhase::IDLE;
    volatile RLIdentifyFailReason last_fail_reason = RLIdentifyFailReason::NONE;
    volatile uint16_t quality_warning_flags = RL_QUALITY_NONE;
    volatile float search_vstep_v = 0.0f;
    volatile float locked_vstep_v = 0.0f;
    volatile float measured_abs_id_a = 0.0f;
    volatile float min_signal_current_a = 0.0f;
    volatile float abort_current_a = 0.0f;
    volatile uint32_t ramp_ticks = 0U;
    volatile uint32_t l_sample_ticks = 0U;
    volatile uint32_t r_settle_ticks = 0U;
    volatile uint32_t r_average_ticks = 0U;
    volatile float rs_estimate_ohm = 0.0f;
    volatile float ls_estimate_h = 0.0f;
    volatile uint8_t repeat_index = 0U;
    volatile uint8_t repeat_count = 0U;
    volatile uint8_t valid_repeat_count = 0U;
    volatile float rs_average_ohm = 0.0f;
    volatile float ls_average_h = 0.0f;
};

/*
 * Motor_RLIdentifyRoutine - R/L 辨识内部流程。
 *
 * 调用入口: mode_ == CALIB_RL_IDENTIFY 时由 MotorManager 每个控制 tick 推进。
 *
 * 流程:
 *   1. 从很小的 d 轴注入电压开始试探, 按实测电流自动放大 Vstep。
 *   2. 电流达到目标区间后锁定当前 Vstep。
 *   3. 用固定 10ms 物理窗口线性拟合 di/dt 估算 L, 再等待电流稳定后平均 V/I 估算 R。
 *   4. 锁定同一个 Vstep 重复多轮采样, 全部通过后写入平均 R/L。
 *
 * 安全边界:
 *   - 用户只配置目标辨识电流和注入电压上限。
 *   - 斜坡、采样窗口、有效信号阈值、搜索倍率和硬终止电流由内部派生。
 *   - 严重过流或电压到上限仍信号不足时终止, 不写入伪辨识值。
 */
class MotorRLIdentifyRoutine
{
public:
    using Phase = RLIdentifyPhase;
    using FailReason = RLIdentifyFailReason;

    void reset()
    {
        phase_ = Phase::IDLE;
        tick_counter_ = 0U;
        ramp_ticks_ = 0U;
        prepare_ticks_ = 0U;
        l_sample_ticks_ = 0U;
        l_target_sample_ticks_ = 0U;
        r_min_settle_ticks_ = 0U;
        r_max_settle_ticks_ = 0U;
        r_stable_confirm_ticks_ = 0U;
        r_min_average_ticks_ = 0U;
        r_settle_ticks_ = 0U;
        r_average_ticks_ = 0U;
        r_stable_ticks_ = 0U;
        v_step_injected_ = 0.0f;
        locked_v_step_ = 0.0f;
        target_current_ = 0.0f;
        target_low_current_ = 0.0f;
        min_signal_current_ = 0.0f;
        abort_current_ = 0.0f;
        i_d_at_step_start_ = 0.0f;
        i_d_at_step_end_ = 0.0f;
        v_steady_avg_ = 0.0f;
        i_steady_avg_ = 0.0f;
        ls_estimate_ = 0.0f;
        rs_estimate_ = 0.0f;
        l_sum_x_ = 0.0f;
        l_sum_y_ = 0.0f;
        l_sum_xx_ = 0.0f;
        l_sum_xy_ = 0.0f;
        r_last_current_ = 0.0f;
        sample_count_ = 0U;
        repeat_count_ = 0U;
        repeat_index_ = 0U;
        valid_repeat_count_ = 0U;
        rs_accum_ = 0.0f;
        ls_accum_ = 0.0f;
        rs_min_ = 0.0f;
        rs_max_ = 0.0f;
        ls_min_ = 0.0f;
        ls_max_ = 0.0f;
        quality_warning_flags_ = RL_QUALITY_NONE;
        clearDebugSnapshot();
    }

    void step(MotorManager& m);

    bool done() const { return phase_ == Phase::DONE; }
    float rsEstimate() const { return rs_estimate_; }
    float lsEstimate() const { return ls_estimate_; }
    const RLIdentifyDebugSnapshot& debugSnapshot() const { return debug_snapshot_; }

private:
    bool hardCurrentOk(MotorManager& m);
    void fail(MotorManager& m, Fault fault, FailReason reason);
    void clearDebugSnapshot();
    void updateDebugSnapshot(float measured_abs_id);
    void setQualityWarning(uint16_t flag);

    Phase phase_ = Phase::IDLE;
    uint32_t tick_counter_ = 0U;
    uint32_t ramp_ticks_ = 0U;
    uint32_t prepare_ticks_ = 0U;
    uint32_t l_sample_ticks_ = 0U;
    uint32_t l_target_sample_ticks_ = 0U;
    uint32_t r_min_settle_ticks_ = 0U;
    uint32_t r_max_settle_ticks_ = 0U;
    uint32_t r_stable_confirm_ticks_ = 0U;
    uint32_t r_min_average_ticks_ = 0U;
    uint32_t r_settle_ticks_ = 0U;
    uint32_t r_average_ticks_ = 0U;
    uint32_t r_stable_ticks_ = 0U;
    float v_step_injected_ = 0.0f;
    float locked_v_step_ = 0.0f;
    float target_current_ = 0.0f;
    float target_low_current_ = 0.0f;
    float min_signal_current_ = 0.0f;
    float abort_current_ = 0.0f;
    float i_d_at_step_start_ = 0.0f;
    float i_d_at_step_end_ = 0.0f;
    float v_steady_avg_ = 0.0f;
    float i_steady_avg_ = 0.0f;
    float ls_estimate_ = 0.0f;
    float rs_estimate_ = 0.0f;
    float l_sum_x_ = 0.0f;
    float l_sum_y_ = 0.0f;
    float l_sum_xx_ = 0.0f;
    float l_sum_xy_ = 0.0f;
    float r_last_current_ = 0.0f;
    uint32_t sample_count_ = 0U;
    uint8_t repeat_count_ = 0U;
    uint8_t repeat_index_ = 0U;
    uint8_t valid_repeat_count_ = 0U;
    float rs_accum_ = 0.0f;
    float ls_accum_ = 0.0f;
    float rs_min_ = 0.0f;
    float rs_max_ = 0.0f;
    float ls_min_ = 0.0f;
    float ls_max_ = 0.0f;
    uint16_t quality_warning_flags_ = RL_QUALITY_NONE;
    RLIdentifyDebugSnapshot debug_snapshot_;
};

} // namespace Lib_Motor
