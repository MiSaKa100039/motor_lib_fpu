/*
 * Motor_RLIdentifyRoutine.cpp - 自动搜索注入电压的 R/L 辨识流程。
 *
 * 说明:
 *   - RL 辨识仍然使用开环 d 轴电压注入, 不依赖尚未整定的电流环。
 *   - 用户配置目标辨识电流和最大注入电压; 本流程从小 Vstep 试探并自动放大。
 *   - 搜索完成后锁定 Vstep, 再重复多轮自适应估算 L/R。
 */

#include "../../Public/Motor_Features.h"

#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY

#include "Motor_RLIdentifyRoutine.h"

#include "../Manager/Motor_Manager.h"

#include <cmath>

namespace Lib_Motor
{

namespace
{
    constexpr float DI_EPS = 1e-5f;
    constexpr float LS_MIN = 1e-6f;
    constexpr float LS_MAX = 0.1f;
    constexpr float RS_MIN = 1e-4f;
    constexpr float RS_MAX = 100.0f;

    constexpr float SEARCH_INITIAL_RATIO = 0.05f;
    constexpr float SEARCH_MIN_VOLTAGE_V = 0.01f;
    constexpr float SEARCH_SIGNAL_GROWTH = 1.5f;
    constexpr float SEARCH_MIN_RATIO_STEP = 1.2f;
    constexpr float SEARCH_MAX_RATIO_STEP = 2.0f;
    constexpr float TARGET_LOCK_RATIO = 0.70f;
    constexpr float MIN_SIGNAL_RATIO = 0.10f;
    constexpr float MIN_SIGNAL_CURRENT_A = 0.01f;
    constexpr float ABORT_CURRENT_RATIO = 1.6f;
    constexpr float HARD_LIMIT_FACTOR = 0.8f;
    constexpr float CURRENT_NEAR_ABORT_RATIO = 0.85f;
    constexpr float REPEAT_SPREAD_WARN_RATIO = 0.15f;

    constexpr float RAMP_TIME_S = 0.020f;
    constexpr float PREPARE_TIME_S = 0.020f;
    constexpr float L_SAMPLE_TIME_S = 0.010f;
    constexpr float R_MIN_SETTLE_TIME_S = 0.005f;
    constexpr float R_MAX_SETTLE_TIME_S = 0.100f;
    constexpr float R_STABLE_CONFIRM_TIME_S = 0.003f;
    constexpr float R_MIN_AVERAGE_TIME_S = 0.050f;
    constexpr float R_STABLE_DELTA_RATIO = 0.002f;
    constexpr float R_STABLE_DELTA_MIN_A = 0.002f;

    uint32_t secondsToTicks(float seconds, float control_freq_hz)
    {
        if (seconds <= 0.0f || control_freq_hz <= 0.0f) return 1U;
        const float ticks = seconds * control_freq_hz;
        if (ticks < 1.0f) return 1U;
        return static_cast<uint32_t>(ticks + 0.999f);
    }

    float clampFloat(float value, float low, float high)
    {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    float minFloat(float a, float b)
    {
        return (a < b) ? a : b;
    }

    float maxFloat(float a, float b)
    {
        return (a > b) ? a : b;
    }

    bool atVoltageLimit(float value, float limit)
    {
        return value >= (limit - 1.0e-6f);
    }

    bool relativeSpanTooLarge(float min_value, float max_value, float average_value)
    {
        if (!std::isfinite(min_value) ||
            !std::isfinite(max_value) ||
            !std::isfinite(average_value) ||
            average_value <= 0.0f)
        {
            return true;
        }
        const float span = max_value - min_value;
        return span > (average_value * REPEAT_SPREAD_WARN_RATIO);
    }

    float nextSearchVoltage(float current_v,
                            float measured_current,
                            float target_current,
                            float min_signal_current,
                            float voltage_limit)
    {
        float next_v = current_v;
        if (measured_current < min_signal_current)
        {
            next_v = current_v * SEARCH_SIGNAL_GROWTH;
        }
        else
        {
            const float ratio = clampFloat(target_current / measured_current,
                                           SEARCH_MIN_RATIO_STEP,
                                           SEARCH_MAX_RATIO_STEP);
            next_v = current_v * ratio;
        }
        return minFloat(next_v, voltage_limit);
    }
}

void MotorRLIdentifyRoutine::clearDebugSnapshot()
{
    debug_snapshot_.phase = Phase::IDLE;
    debug_snapshot_.last_fail_reason = FailReason::NONE;
    debug_snapshot_.quality_warning_flags = RL_QUALITY_NONE;
    debug_snapshot_.search_vstep_v = 0.0f;
    debug_snapshot_.locked_vstep_v = 0.0f;
    debug_snapshot_.measured_abs_id_a = 0.0f;
    debug_snapshot_.min_signal_current_a = 0.0f;
    debug_snapshot_.abort_current_a = 0.0f;
    debug_snapshot_.ramp_ticks = 0U;
    debug_snapshot_.l_sample_ticks = 0U;
    debug_snapshot_.r_settle_ticks = 0U;
    debug_snapshot_.r_average_ticks = 0U;
    debug_snapshot_.rs_estimate_ohm = 0.0f;
    debug_snapshot_.ls_estimate_h = 0.0f;
    debug_snapshot_.repeat_index = 0U;
    debug_snapshot_.repeat_count = 0U;
    debug_snapshot_.valid_repeat_count = 0U;
    debug_snapshot_.rs_average_ohm = 0.0f;
    debug_snapshot_.ls_average_h = 0.0f;
}

void MotorRLIdentifyRoutine::updateDebugSnapshot(float measured_abs_id)
{
    debug_snapshot_.phase = phase_;
    debug_snapshot_.quality_warning_flags = quality_warning_flags_;
    debug_snapshot_.search_vstep_v = v_step_injected_;
    debug_snapshot_.locked_vstep_v = locked_v_step_;
    debug_snapshot_.measured_abs_id_a = measured_abs_id;
    debug_snapshot_.min_signal_current_a = min_signal_current_;
    debug_snapshot_.abort_current_a = abort_current_;
    debug_snapshot_.ramp_ticks = ramp_ticks_;
    debug_snapshot_.l_sample_ticks = l_sample_ticks_;
    debug_snapshot_.r_settle_ticks = r_settle_ticks_;
    debug_snapshot_.r_average_ticks = r_average_ticks_;
    debug_snapshot_.rs_estimate_ohm = rs_estimate_;
    debug_snapshot_.ls_estimate_h = ls_estimate_;
    debug_snapshot_.repeat_index = repeat_index_;
    debug_snapshot_.repeat_count = repeat_count_;
    debug_snapshot_.valid_repeat_count = valid_repeat_count_;
    if (valid_repeat_count_ > 0U)
    {
        const float n = static_cast<float>(valid_repeat_count_);
        debug_snapshot_.rs_average_ohm = rs_accum_ / n;
        debug_snapshot_.ls_average_h = ls_accum_ / n;
    }
    else
    {
        debug_snapshot_.rs_average_ohm = 0.0f;
        debug_snapshot_.ls_average_h = 0.0f;
    }
}

void MotorRLIdentifyRoutine::setQualityWarning(uint16_t flag)
{
    quality_warning_flags_ = static_cast<uint16_t>(quality_warning_flags_ | flag);
    debug_snapshot_.quality_warning_flags = quality_warning_flags_;
}

void MotorRLIdentifyRoutine::fail(MotorManager& m, Fault fault, FailReason reason)
{
    auto& ctx = m.ctx_;
    ctx.v_d = 0.0f;
    ctx.v_q = 0.0f;
    m.event_.rl_identify_done = 0U;
    m.setConfigFaultDetail(MotorConfigFaultDetail::RL_IDENTIFY_FAILED);
    phase_ = Phase::DONE;
    debug_snapshot_.last_fail_reason = reason;
    updateDebugSnapshot(std::isfinite(ctx.i_d) ? fabsf(ctx.i_d) : 0.0f);
    m.stopForFault(fault);
}

bool MotorRLIdentifyRoutine::hardCurrentOk(MotorManager& m)
{
    const float abs_id = fabsf(m.ctx_.i_d);
    if (!std::isfinite(abs_id))
    {
        fail(m, Fault::PARAM_ERROR, FailReason::CURRENT_NAN);
        return false;
    }

    debug_snapshot_.measured_abs_id_a = abs_id;
    if (abort_current_ > 0.0f && abs_id > (abort_current_ * CURRENT_NEAR_ABORT_RATIO))
    {
        setQualityWarning(RL_QUALITY_CURRENT_NEAR_ABORT);
    }
    if (abort_current_ > 0.0f && abs_id > abort_current_)
    {
        fail(m, Fault::OVERCURRENT, FailReason::OVERCURRENT);
        return false;
    }

    return true;
}

void MotorRLIdentifyRoutine::step(MotorManager& m)
{
    auto& cfg = m.config_;
    auto& ctx = m.ctx_;
    const auto& identify = cfg.identify;

    switch (phase_)
    {
    case Phase::IDLE:
    {
        m.event_.rl_identify_done = 0U;
        quality_warning_flags_ = RL_QUALITY_NONE;
        clearDebugSnapshot();

        target_current_ = identify.rl_target_current_a;
        target_low_current_ = target_current_ * TARGET_LOCK_RATIO;
        min_signal_current_ = maxFloat(MIN_SIGNAL_CURRENT_A,
                                       target_current_ * MIN_SIGNAL_RATIO);
        abort_current_ = target_current_ * ABORT_CURRENT_RATIO;
        repeat_count_ = (identify.rl_repeat_count > 0U) ? identify.rl_repeat_count : 1U;
        if (repeat_count_ > 16U)
        {
            repeat_count_ = 16U;
        }
        repeat_index_ = 1U;
        valid_repeat_count_ = 0U;
        rs_accum_ = 0.0f;
        ls_accum_ = 0.0f;
        if (std::isfinite(cfg.limit.max_current_a) && cfg.limit.max_current_a > 0.0f)
        {
            abort_current_ = minFloat(abort_current_,
                                      cfg.limit.max_current_a * HARD_LIMIT_FACTOR);
        }

        const float voltage_limit = identify.rl_injection_voltage_limit_v;
        v_step_injected_ = minFloat(voltage_limit,
                                    maxFloat(SEARCH_MIN_VOLTAGE_V,
                                             voltage_limit * SEARCH_INITIAL_RATIO));
        if (!std::isfinite(v_step_injected_) || v_step_injected_ <= 0.0f)
        {
            fail(m, Fault::PARAM_ERROR, FailReason::ESTIMATE_INVALID);
            break;
        }

        ramp_ticks_ = secondsToTicks(RAMP_TIME_S, cfg.control.control_freq_hz);
        prepare_ticks_ = secondsToTicks(PREPARE_TIME_S, cfg.control.control_freq_hz);
        l_target_sample_ticks_ = secondsToTicks(L_SAMPLE_TIME_S, cfg.control.control_freq_hz);
        r_min_settle_ticks_ = secondsToTicks(R_MIN_SETTLE_TIME_S, cfg.control.control_freq_hz);
        r_max_settle_ticks_ = secondsToTicks(R_MAX_SETTLE_TIME_S, cfg.control.control_freq_hz);
        if (r_max_settle_ticks_ <= r_min_settle_ticks_)
        {
            r_max_settle_ticks_ = r_min_settle_ticks_ + 1U;
        }
        r_stable_confirm_ticks_ = secondsToTicks(R_STABLE_CONFIRM_TIME_S, cfg.control.control_freq_hz);
        r_min_average_ticks_ = secondsToTicks(R_MIN_AVERAGE_TIME_S, cfg.control.control_freq_hz);

        locked_v_step_ = 0.0f;
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
        r_settle_ticks_ = 0U;
        r_average_ticks_ = 0U;
        r_stable_ticks_ = 0U;
        sample_count_ = 0U;
        tick_counter_ = 0U;

        ctx.v_d = 0.0f;
        ctx.v_q = 0.0f;
        phase_ = Phase::SEARCH_RAMP;
        updateDebugSnapshot(std::isfinite(ctx.i_d) ? fabsf(ctx.i_d) : 0.0f);
        break;
    }

    case Phase::SEARCH_RAMP:
    {
        ++tick_counter_;
        const float scale = clampFloat(static_cast<float>(tick_counter_) /
                                       static_cast<float>(ramp_ticks_),
                                       0.0f,
                                       1.0f);
        ctx.v_d = v_step_injected_ * scale;
        ctx.v_q = 0.0f;

        if (!hardCurrentOk(m))
        {
            break;
        }

        if (tick_counter_ >= ramp_ticks_)
        {
            const float measured_current = fabsf(ctx.i_d);
            const float voltage_limit = identify.rl_injection_voltage_limit_v;
            if (!std::isfinite(measured_current))
            {
                fail(m, Fault::PARAM_ERROR, FailReason::CURRENT_NAN);
                break;
            }

            const bool voltage_limited = atVoltageLimit(v_step_injected_, voltage_limit);
            if (measured_current >= target_low_current_ || voltage_limited)
            {
                if (measured_current < min_signal_current_)
                {
                    fail(m,
                         Fault::PARAM_ERROR,
                         FailReason::SIGNAL_TOO_LOW_AT_VOLTAGE_LIMIT);
                    break;
                }

                if (voltage_limited && measured_current < target_low_current_)
                {
                    setQualityWarning(RL_QUALITY_VOLTAGE_LIMIT_BELOW_TARGET);
                }

                locked_v_step_ = v_step_injected_;
                tick_counter_ = 0U;
                ctx.v_d = 0.0f;
                ctx.v_q = 0.0f;
                phase_ = Phase::MEASURE_PREPARE;
                updateDebugSnapshot(measured_current);
                break;
            }

            const float next_v = nextSearchVoltage(v_step_injected_,
                                                   measured_current,
                                                   target_current_,
                                                   min_signal_current_,
                                                   voltage_limit);
            if (!std::isfinite(next_v) || next_v <= v_step_injected_)
            {
                fail(m, Fault::PARAM_ERROR, FailReason::ESTIMATE_INVALID);
                break;
            }

            v_step_injected_ = next_v;
            tick_counter_ = 0U;
        }
        updateDebugSnapshot(fabsf(ctx.i_d));
        break;
    }

    case Phase::MEASURE_PREPARE:
    {
        ctx.v_d = 0.0f;
        ctx.v_q = 0.0f;

        if (!hardCurrentOk(m))
        {
            break;
        }

        ++tick_counter_;
        if (tick_counter_ >= prepare_ticks_)
        {
            const float sample_current = ctx.i_d;
            if (!std::isfinite(sample_current))
            {
                fail(m, Fault::PARAM_ERROR, FailReason::CURRENT_NAN);
                break;
            }

            i_d_at_step_start_ = sample_current;
            i_d_at_step_end_ = sample_current;
            l_sample_ticks_ = 0U;
            l_sum_x_ = 0.0f;
            l_sum_y_ = 0.0f;
            l_sum_xx_ = 0.0f;
            l_sum_xy_ = 0.0f;
            sample_count_ = 0U;
            tick_counter_ = 0U;
            phase_ = Phase::INJECT_STEP;
        }
        updateDebugSnapshot(fabsf(ctx.i_d));
        break;
    }

    case Phase::INJECT_STEP:
    {
        ctx.v_d = locked_v_step_;
        ctx.v_q = 0.0f;

        if (!hardCurrentOk(m))
        {
            break;
        }

        const float sample_current = ctx.i_d;
        if (!std::isfinite(sample_current))
        {
            fail(m, Fault::PARAM_ERROR, FailReason::CURRENT_NAN);
            break;
        }

        const float x = static_cast<float>(l_sample_ticks_);
        l_sum_x_ += x;
        l_sum_y_ += sample_current;
        l_sum_xx_ += x * x;
        l_sum_xy_ += x * sample_current;
        ++l_sample_ticks_;
        ++sample_count_;
        i_d_at_step_end_ = sample_current;

        const float di = i_d_at_step_end_ - i_d_at_step_start_;
        if (l_sample_ticks_ >= l_target_sample_ticks_)
        {
            if (fabsf(di) < min_signal_current_)
            {
                fail(m, Fault::PARAM_ERROR, FailReason::L_DI_TOO_SMALL);
                break;
            }

            const float n = static_cast<float>(sample_count_);
            const float denom = n * l_sum_xx_ - l_sum_x_ * l_sum_x_;
            if (sample_count_ < 2U || !std::isfinite(denom) || fabsf(denom) <= DI_EPS)
            {
                fail(m, Fault::PARAM_ERROR, FailReason::L_DI_TOO_SMALL);
                break;
            }

            const float slope_per_tick = (n * l_sum_xy_ - l_sum_x_ * l_sum_y_) / denom;
            const float didt = slope_per_tick * cfg.control.control_freq_hz;
            const float r_known = cfg.physical.rs_effective();
            const float i_avg = l_sum_y_ / n;

            if (fabsf(didt) > DI_EPS)
            {
                float v_eff = locked_v_step_ - r_known * i_avg;
                if (!std::isfinite(v_eff) || v_eff <= 0.0f)
                {
                    v_eff = locked_v_step_;
                }

                ls_estimate_ = v_eff / didt;
                if (ls_estimate_ < 0.0f) ls_estimate_ = -ls_estimate_;
                if (!std::isfinite(ls_estimate_) || ls_estimate_ <= 0.0f)
                {
                    fail(m, Fault::PARAM_ERROR, FailReason::ESTIMATE_INVALID);
                    break;
                }
                ls_estimate_ = clampFloat(ls_estimate_, LS_MIN, LS_MAX);
            }
            else
            {
                fail(m, Fault::PARAM_ERROR, FailReason::L_DI_TOO_SMALL);
                break;
            }

            phase_ = Phase::STEADY_STATE;
            tick_counter_ = 0U;
            sample_count_ = 0U;
            v_steady_avg_ = 0.0f;
            i_steady_avg_ = 0.0f;
            r_settle_ticks_ = 0U;
            r_average_ticks_ = 0U;
            r_stable_ticks_ = 0U;
            r_last_current_ = fabsf(ctx.i_d);
        }
        updateDebugSnapshot(fabsf(ctx.i_d));
        break;
    }

    case Phase::STEADY_STATE:
    {
        ctx.v_d = locked_v_step_;
        ctx.v_q = 0.0f;

        if (!hardCurrentOk(m))
        {
            break;
        }

        const float current_abs = fabsf(ctx.i_d);
        const float stable_delta = maxFloat(R_STABLE_DELTA_MIN_A,
                                            target_current_ * R_STABLE_DELTA_RATIO);

        if (sample_count_ == 0U)
        {
            const float delta = fabsf(current_abs - r_last_current_);
            r_last_current_ = current_abs;
            ++r_settle_ticks_;

            if (r_settle_ticks_ >= r_min_settle_ticks_ && delta <= stable_delta)
            {
                ++r_stable_ticks_;
            }
            else
            {
                r_stable_ticks_ = 0U;
            }

            const bool settle_done = r_stable_ticks_ >= r_stable_confirm_ticks_;
            const bool settle_timeout = r_settle_ticks_ >= r_max_settle_ticks_;
            if (!settle_done && !settle_timeout)
            {
                updateDebugSnapshot(current_abs);
                break;
            }

            if (settle_timeout && !settle_done)
            {
                setQualityWarning(RL_QUALITY_R_SETTLE_TIMEOUT);
            }
        }

        v_steady_avg_ += ctx.v_d;
        i_steady_avg_ += current_abs;
        ++sample_count_;
        r_average_ticks_ = sample_count_;

        if (r_average_ticks_ >= r_min_average_ticks_)
        {
            if (sample_count_ == 0U)
            {
                fail(m, Fault::PARAM_ERROR, FailReason::R_CURRENT_TOO_SMALL);
                break;
            }

            v_steady_avg_ = v_steady_avg_ / static_cast<float>(sample_count_);
            i_steady_avg_ = i_steady_avg_ / static_cast<float>(sample_count_);

            if (!std::isfinite(i_steady_avg_) ||
                i_steady_avg_ < min_signal_current_)
            {
                fail(m, Fault::PARAM_ERROR, FailReason::R_CURRENT_TOO_SMALL);
                break;
            }

            rs_estimate_ = v_steady_avg_ / i_steady_avg_;
            if (rs_estimate_ < 0.0f) rs_estimate_ = -rs_estimate_;
            if (!std::isfinite(rs_estimate_) || rs_estimate_ <= 0.0f)
            {
                fail(m, Fault::PARAM_ERROR, FailReason::ESTIMATE_INVALID);
                break;
            }
            rs_estimate_ = clampFloat(rs_estimate_, RS_MIN, RS_MAX);

            if (valid_repeat_count_ == 0U)
            {
                rs_min_ = rs_estimate_;
                rs_max_ = rs_estimate_;
                ls_min_ = ls_estimate_;
                ls_max_ = ls_estimate_;
            }
            else
            {
                if (rs_estimate_ < rs_min_) rs_min_ = rs_estimate_;
                if (rs_estimate_ > rs_max_) rs_max_ = rs_estimate_;
                if (ls_estimate_ < ls_min_) ls_min_ = ls_estimate_;
                if (ls_estimate_ > ls_max_) ls_max_ = ls_estimate_;
            }

            rs_accum_ += rs_estimate_;
            ls_accum_ += ls_estimate_;
            ++valid_repeat_count_;

            if (valid_repeat_count_ >= repeat_count_)
            {
                const float n = static_cast<float>(valid_repeat_count_);
                rs_estimate_ = rs_accum_ / n;
                ls_estimate_ = ls_accum_ / n;
                if (relativeSpanTooLarge(rs_min_, rs_max_, rs_estimate_) ||
                    relativeSpanTooLarge(ls_min_, ls_max_, ls_estimate_))
                {
                    setQualityWarning(RL_QUALITY_REPEAT_SPREAD_HIGH);
                }
                phase_ = Phase::FINALIZE;
            }
            else
            {
                ++repeat_index_;
                tick_counter_ = 0U;
                sample_count_ = 0U;
                v_steady_avg_ = 0.0f;
                i_steady_avg_ = 0.0f;
                i_d_at_step_start_ = 0.0f;
                i_d_at_step_end_ = 0.0f;
                l_sample_ticks_ = 0U;
                r_settle_ticks_ = 0U;
                r_average_ticks_ = 0U;
                r_stable_ticks_ = 0U;
                l_sum_x_ = 0.0f;
                l_sum_y_ = 0.0f;
                l_sum_xx_ = 0.0f;
                l_sum_xy_ = 0.0f;
                r_last_current_ = 0.0f;
                rs_estimate_ = 0.0f;
                ls_estimate_ = 0.0f;
                ctx.v_d = 0.0f;
                ctx.v_q = 0.0f;
                phase_ = Phase::MEASURE_PREPARE;
            }
        }
        updateDebugSnapshot(fabsf(ctx.i_d));
        break;
    }

    case Phase::FINALIZE:
    {
        if (valid_repeat_count_ == 0U)
        {
            fail(m, Fault::PARAM_ERROR, FailReason::ESTIMATE_INVALID);
            break;
        }

        if (!std::isfinite(rs_estimate_) ||
            !std::isfinite(ls_estimate_) ||
            rs_estimate_ <= 0.0f ||
            ls_estimate_ <= 0.0f)
        {
            fail(m, Fault::PARAM_ERROR, FailReason::ESTIMATE_INVALID);
            break;
        }

        cfg.physical.rs_ohm_identified = rs_estimate_;
        cfg.physical.ls_h_identified = ls_estimate_;
        m.event_.rl_identify_done = 1U;
        m.setConfigFaultDetail(MotorConfigFaultDetail::NONE);

        m.controller_.applyAutoDeriveIfEnabled(cfg);

        ctx.v_d = 0.0f;
        ctx.v_q = 0.0f;
        phase_ = Phase::DONE;
        debug_snapshot_.last_fail_reason = FailReason::NONE;
        updateDebugSnapshot(std::isfinite(ctx.i_d) ? fabsf(ctx.i_d) : 0.0f);
        break;
    }

    case Phase::DONE:
        ctx.v_d = 0.0f;
        ctx.v_q = 0.0f;
        updateDebugSnapshot(std::isfinite(ctx.i_d) ? fabsf(ctx.i_d) : 0.0f);
        break;
    }
}

} // namespace Lib_Motor

#endif // LIB_MOTOR_ENABLE_AUTO_IDENTIFY
