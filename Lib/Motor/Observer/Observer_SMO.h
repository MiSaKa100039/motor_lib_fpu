#pragma once

#include "Observer_Base.h"
#include "../Public/Motor_Features.h"
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../Control/Utils/LowPassFilter.h"
#include "../Core/Numeric/Motor_FixedNumeric.h"
#else
#include "../Control/Utils/FocMath.h"
#endif
#include <cmath>
#include <limits>

namespace Lib_Motor
{

static constexpr float kDefaultSmoBemfLpfCutoffHz = 1000.0f;

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15

/*
 * SlidingModeObserverQ15 — fixed-q15 SMO shadow observer
 *
 * 本实现只服务 fixed-q15 tick 路径。初始化阶段允许把 MotorCfg 里的物理量
 * 转成定点系数；update() 内只保留整数乘加、移位和饱和。
 */
class SlidingModeObserverQ15
{
#if defined(MOTOR_LIB_HOST_TEST)
    friend struct SmoQ15TestAccess;
#endif
public:
    SlidingModeObserverQ15() = default;

    void init(float rs, float ls, float smo_gain, float smo_pll_kp, float smo_pll_ki)
    {
        init(rs, ls, smo_gain, smo_pll_kp, smo_pll_ki,
             1.0f, 1.0f, 1.0f, 0.0f, 1U);
    }

    void init(float rs, float ls, float smo_gain,
              float smo_pll_kp, float smo_pll_ki,
              float current_base_a, float voltage_base_v,
              float speed_base_rpm, float dt, uint8_t pole_pairs,
              float smo_bemf_lpf_cutoff_hz = kDefaultSmoBemfLpfCutoffHz)
    {
        reset();
        configured_ = false;
        pole_pairs_ = pole_pairs > 0U ? pole_pairs : 1U;
        speed_base_rpm_ = speed_base_rpm;
        voltage_base_v_ = voltage_base_v;

        if (!std::isfinite(rs) || !std::isfinite(ls) ||
            !std::isfinite(smo_gain) || !std::isfinite(smo_pll_kp) ||
            !std::isfinite(smo_pll_ki) || !std::isfinite(current_base_a) ||
            !std::isfinite(voltage_base_v) || !std::isfinite(speed_base_rpm) ||
            !std::isfinite(dt) || !std::isfinite(smo_bemf_lpf_cutoff_hz) ||
            !(rs > 0.0f) || !(ls > 0.0f) ||
            !(current_base_a > 0.0f) || !(voltage_base_v > 0.0f) ||
            !(speed_base_rpm > 0.0f) || !(dt > 0.0f) ||
            !(smo_bemf_lpf_cutoff_hz > 0.0f))
        {
            return;
        }

        const float gain_abs = (smo_gain >= 0.0f) ? smo_gain : -smo_gain;
        r_drop_q15_ = FixedNumeric::fromNormalized(
            (rs * current_base_a) / voltage_base_v);
        model_gain_q12_ = positiveToQ12(
            (voltage_base_v * dt) / (ls * current_base_a));
        sliding_gain_q12_ = positiveToQ12(
            (gain_abs * current_base_a) / voltage_base_v);
        z_limit_q15_ = FixedNumeric::fromPhysical(gain_abs, voltage_base_v);

        pll_kp_speed_q15_ = positiveToQ15(
            (smo_pll_kp * 60.0f) /
            (2.0f * static_cast<float>(pole_pairs_) * speed_base_rpm));
        pll_ki_speed_per_tick_q20_ = positiveToQ20(
            (smo_pll_ki * dt * 60.0f) /
            (2.0f * static_cast<float>(pole_pairs_) * speed_base_rpm));

        configurePhaseStepScale(speed_base_rpm, dt);

        constexpr float kTwoPi = 6.28318530717958647692f;
        bemf_alpha_lpf_.setSamplePeriod(dt);
        bemf_beta_lpf_.setSamplePeriod(dt);
        bemf_alpha_lpf_.setTf(1.0f / (kTwoPi * smo_bemf_lpf_cutoff_hz));
        bemf_beta_lpf_.setTf(1.0f / (kTwoPi * smo_bemf_lpf_cutoff_hz));

        configured_ =
            model_gain_q12_ > 0 &&
            sliding_gain_q12_ > 0 &&
            z_limit_q15_ > 0 &&
            phase_step_q16_per_speed_q15_ > 0U;
    }

    void setValidityCriteria(float min_elec_speed_rad_s,
                             float max_pll_error_rad,
                             uint16_t convergence_ticks,
                             float min_signal_level)
    {
        convergence_ticks_ = convergence_ticks > 0U ? convergence_ticks : 1U;
        if (std::isfinite(min_elec_speed_rad_s) &&
            min_elec_speed_rad_s > 0.0f &&
            speed_base_rpm_ > 0.0f)
        {
            constexpr float kTwoPi = 6.28318530717958647692f;
            const float min_mech_rpm =
                min_elec_speed_rad_s * 60.0f /
                (kTwoPi * static_cast<float>(pole_pairs_));
            min_valid_speed_q15_ =
                FixedNumeric::absoluteQ15(FixedNumeric::fromPhysical(
                    min_mech_rpm, speed_base_rpm_));
        }
        else
        {
            min_valid_speed_q15_ = 0;
        }

        max_valid_pll_error_q15_ =
            FixedNumeric::absoluteQ15(radiansToPiQ15(max_pll_error_rad));
        if (max_valid_pll_error_q15_ == 0)
        {
            max_valid_pll_error_q15_ = 1;
        }

        min_signal_level_q15_ =
            (std::isfinite(min_signal_level) && min_signal_level > 0.0f)
                ? FixedNumeric::fromPhysical(min_signal_level, voltage_base_v_)
                : 0;
    }

    const ObserverEstimateQ15& estimate() const
    {
        return last_estimate_;
    }

    void reset()
    {
        angle_est_phase_ = 0U;
        speed_est_q15_ = 0;
        pll_integral_speed_q35_ = 0;
        last_pll_error_q15_ = 0;
        valid_ticks_ = 0U;
        i_alpha_est_ = 0;
        i_beta_est_ = 0;
        bemf_alpha_lpf_.resetQ15(0);
        bemf_beta_lpf_.resetQ15(0);
        current_model_ready_ = false;
        last_estimate_ = ObserverEstimateQ15();
    }

    void seedAngle(float angle_rad)
    {
        reset();
        angle_est_phase_ = FixedNumeric::phaseFromRadians(angle_rad);
    }

    void seedAngle(FixedNumeric::phase_u32_t angle_phase)
    {
        reset();
        angle_est_phase_ = angle_phase;
    }

    void update(FixedNumeric::q15_t v_alpha, FixedNumeric::q15_t v_beta,
                FixedNumeric::q15_t i_alpha, FixedNumeric::q15_t i_beta,
                ObserverEstimateQ15* estimate_out)
    {
        if (!configured_)
        {
            fillEstimate(estimate_out, 0);
            return;
        }

        if (!current_model_ready_)
        {
            i_alpha_est_ = i_alpha;
            i_beta_est_ = i_beta;
            current_model_ready_ = true;
            fillEstimate(estimate_out, 0);
            return;
        }

        const FixedNumeric::q15_t z_alpha =
            slidingControl(FixedNumeric::subtractQ15(i_alpha, i_alpha_est_));
        const FixedNumeric::q15_t z_beta =
            slidingControl(FixedNumeric::subtractQ15(i_beta, i_beta_est_));

        i_alpha_est_ = FixedNumeric::saturateQ15(
            static_cast<std::int32_t>(i_alpha_est_) +
            modelStepQ15(v_alpha, i_alpha_est_, bemf_alpha_lpf_.getValueQ15(), z_alpha));
        i_beta_est_ = FixedNumeric::saturateQ15(
            static_cast<std::int32_t>(i_beta_est_) +
            modelStepQ15(v_beta, i_beta_est_, bemf_beta_lpf_.getValueQ15(), z_beta));

        const FixedNumeric::q15_t bemf_alpha = bemf_alpha_lpf_.updateQ15(z_alpha);
        const FixedNumeric::q15_t bemf_beta = bemf_beta_lpf_.updateQ15(z_beta);
        const FixedNumeric::q15_t signal_level =
            magnitudeApproxQ15(bemf_alpha, bemf_beta);

        FixedNumeric::q15_t pll_error = 0;
        if (signal_level > 0)
        {
            const FixedNumeric::phase_u32_t angle_obs =
                atan2PhaseQ15(negateQ15(bemf_alpha), bemf_beta);
            const std::uint32_t raw_error = angle_obs - angle_est_phase_;
            pll_error = phaseErrorToPiQ15(static_cast<std::int32_t>(raw_error));
        }
        last_pll_error_q15_ = pll_error;

        const FixedNumeric::q15_t integral_speed = updatePllIntegral(pll_error);
        const FixedNumeric::q15_t proportional_speed =
            scaleErrorByQ15(pll_error, pll_kp_speed_q15_);
        speed_est_q15_ = FixedNumeric::addQ15(proportional_speed,
                                              integral_speed);
        angle_est_phase_ += phaseStepFromSpeedQ15(speed_est_q15_);

        const FixedNumeric::q15_t speed_abs =
            FixedNumeric::absoluteQ15(speed_est_q15_);
        const FixedNumeric::q15_t error_abs =
            FixedNumeric::absoluteQ15(last_pll_error_q15_);
        const bool signal_ok =
            (min_signal_level_q15_ <= 0) || (signal_level >= min_signal_level_q15_);
        const bool candidate =
            speed_abs >= min_valid_speed_q15_ &&
            signal_ok &&
            error_abs <= max_valid_pll_error_q15_;
        if (candidate)
        {
            if (valid_ticks_ < 0xFFFFU)
            {
                ++valid_ticks_;
            }
        }
        else
        {
            valid_ticks_ = 0U;
        }

        fillEstimate(estimate_out, signal_level);
    }

    void update(FixedNumeric::q15_t v_alpha, FixedNumeric::q15_t v_beta,
                FixedNumeric::q15_t i_alpha, FixedNumeric::q15_t i_beta,
                FixedNumeric::phase_u32_t* angle_out,
                FixedNumeric::q15_t* speed_out)
    {
        ObserverEstimateQ15 estimate;
        update(v_alpha, v_beta, i_alpha, i_beta, &estimate);
        if (angle_out != nullptr)
        {
            *angle_out = estimate.angle_phase;
        }
        if (speed_out != nullptr)
        {
            *speed_out = estimate.speed_rpm_q15;
        }
    }

    /* 当前 Q15 SMO 不执行 Ke 辨识，保持 0 表示未产生有效估计。 */
    float getEstimatedKe() const { return 0.0f; }

private:
    static std::int32_t positiveToQ12(float value)
    {
        if (!std::isfinite(value) || value <= 0.0f)
        {
            return 0;
        }
        const float scaled = value * 4096.0f;
        if (scaled >= static_cast<float>((std::numeric_limits<std::int32_t>::max)()))
        {
            return (std::numeric_limits<std::int32_t>::max)();
        }
        return static_cast<std::int32_t>(scaled + 0.5f);
    }

    static std::int32_t positiveToQ15(float value)
    {
        if (!std::isfinite(value) || value <= 0.0f)
        {
            return 0;
        }
        const float scaled = value * 32768.0f;
        if (scaled >= 32767.0f)
        {
            return 32767;
        }
        return static_cast<std::int32_t>(scaled + 0.5f);
    }

    static std::int32_t positiveToQ20(float value)
    {
        if (!std::isfinite(value) || value <= 0.0f)
        {
            return 0;
        }
        const float scaled = value * 1048576.0f;
        if (scaled >= 32767.0f)
        {
            return 32767;
        }
        return static_cast<std::int32_t>(scaled + 0.5f);
    }

    void configurePhaseStepScale(float speed_base_rpm, float dt)
    {
        phase_step_q16_per_speed_q15_ = 0U;
        if (!std::isfinite(speed_base_rpm) || !std::isfinite(dt) ||
            !(speed_base_rpm > 0.0f) || !(dt > 0.0f))
        {
            return;
        }

        const float scaled =
            speed_base_rpm *
            static_cast<float>(pole_pairs_) *
            8589934592.0f *
            dt / 60.0f;
        if (scaled >= 4294967040.0f)
        {
            phase_step_q16_per_speed_q15_ = 0xFFFFFFFFUL;
        }
        else if (scaled > 0.0f)
        {
            phase_step_q16_per_speed_q15_ =
                static_cast<std::uint32_t>(scaled + 0.5f);
        }
    }

    FixedNumeric::q15_t slidingControl(FixedNumeric::q15_t current_error) const
    {
        const FixedNumeric::q15_t raw =
            scaleSignedByQ12(current_error, sliding_gain_q12_);
        return clampSymmetricQ15(raw, z_limit_q15_);
    }

    std::int32_t modelStepQ15(FixedNumeric::q15_t voltage,
                              FixedNumeric::q15_t current_est,
                              FixedNumeric::q15_t bemf,
                              FixedNumeric::q15_t sliding) const
    {
        const FixedNumeric::q15_t r_drop =
            FixedNumeric::multiplyQ15(r_drop_q15_, current_est);
        const FixedNumeric::q15_t term = FixedNumeric::saturateQ15(
            static_cast<std::int32_t>(voltage) -
            static_cast<std::int32_t>(r_drop) -
            static_cast<std::int32_t>(bemf) +
            static_cast<std::int32_t>(sliding));
        return scaleSignedByQ12(term, model_gain_q12_);
    }

    FixedNumeric::phase_u32_t phaseStepFromSpeedQ15(FixedNumeric::q15_t speed_q15) const
    {
        const std::int32_t speed = static_cast<std::int32_t>(speed_q15);
        const std::uint32_t speed_abs =
            (speed < 0) ? static_cast<std::uint32_t>(-speed)
                        : static_cast<std::uint32_t>(speed);
        const std::uint32_t step = static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(speed_abs) *
             static_cast<std::uint64_t>(phase_step_q16_per_speed_q15_)) >> 16U);
        return (speed < 0) ? static_cast<FixedNumeric::phase_u32_t>(0UL - step)
                           : step;
    }

    void fillEstimate(ObserverEstimateQ15* estimate_out,
                      FixedNumeric::q15_t signal_level)
    {
        const FixedNumeric::q15_t speed_abs =
            FixedNumeric::absoluteQ15(speed_est_q15_);
        const FixedNumeric::q15_t error_abs =
            FixedNumeric::absoluteQ15(last_pll_error_q15_);
        last_estimate_.angle_phase = angle_est_phase_;
        last_estimate_.speed_rpm_q15 = speed_est_q15_;
        last_estimate_.pll_error_q15 = last_pll_error_q15_;
        last_estimate_.signal_level_q15 = signal_level;
        last_estimate_.quality_q15 = computeQuality(speed_abs, error_abs, signal_level);
        last_estimate_.valid_ticks = valid_ticks_;
        last_estimate_.valid = valid_ticks_ >= convergence_ticks_;
        if (estimate_out != nullptr)
        {
            *estimate_out = last_estimate_;
        }
    }

    FixedNumeric::q15_t computeQuality(FixedNumeric::q15_t speed_abs,
                                       FixedNumeric::q15_t error_abs,
                                       FixedNumeric::q15_t signal_level) const
    {
        const FixedNumeric::q15_t speed_quality =
            ratioQuality(speed_abs, min_valid_speed_q15_);
        const FixedNumeric::q15_t error_quality =
            (error_abs >= max_valid_pll_error_q15_)
                ? 0
                : FixedNumeric::subtractQ15(FixedNumeric::kQ15One,
                                            ratioQuality(error_abs, max_valid_pll_error_q15_));
        const FixedNumeric::q15_t signal_quality =
            ratioQuality(signal_level, min_signal_level_q15_);
        const FixedNumeric::q15_t first =
            (speed_quality < error_quality) ? speed_quality : error_quality;
        return (first < signal_quality) ? first : signal_quality;
    }

    static FixedNumeric::q15_t scaleSignedByQ12(FixedNumeric::q15_t value,
                                                std::int32_t coeff_q12)
    {
        // Q12 系数使用完整 int32 范围，先用宽乘积缩放，再收窄到 Q15。
        const std::int64_t scaled =
            (static_cast<std::int64_t>(value) * coeff_q12) >> 12U;
        if (scaled > 32767) return 32767;
        if (scaled < -32768) return -32768;
        return static_cast<FixedNumeric::q15_t>(scaled);
    }

    static FixedNumeric::q15_t scaleErrorByQ15(FixedNumeric::q15_t value,
                                               std::int32_t coeff_q15)
    {
        const std::int32_t product =
            static_cast<std::int32_t>(value) * coeff_q15;
        return FixedNumeric::saturateQ15(product >> 15U);
    }

    FixedNumeric::q15_t updatePllIntegral(FixedNumeric::q15_t error)
    {
        // Q15 误差乘 Q20 系数得到 Q35，保留小数跨周期累积，避免积分死区。
        constexpr std::int64_t kLimit = static_cast<std::int64_t>(32767) << 20U;
        const std::int64_t next = pll_integral_speed_q35_ +
            static_cast<std::int64_t>(error) * pll_ki_speed_per_tick_q20_;
        pll_integral_speed_q35_ = (next > kLimit) ? kLimit
                                   : ((next < -kLimit) ? -kLimit : next);

        // 只在输出处按绝对值四舍五入，再恢复符号，保证正负小误差对称。
        const bool negative = pll_integral_speed_q35_ < 0;
        const std::int64_t magnitude = negative ? -pll_integral_speed_q35_
                                                : pll_integral_speed_q35_;
        const std::int32_t rounded = static_cast<std::int32_t>(
            (magnitude + (static_cast<std::int64_t>(1) << 19U)) >> 20U);
        return static_cast<FixedNumeric::q15_t>(negative ? -rounded : rounded);
    }

    static FixedNumeric::q15_t clampSymmetricQ15(FixedNumeric::q15_t value,
                                                 FixedNumeric::q15_t limit)
    {
        const FixedNumeric::q15_t abs_limit = FixedNumeric::absoluteQ15(limit);
        if (value > abs_limit)
        {
            return abs_limit;
        }
        if (value < static_cast<FixedNumeric::q15_t>(-abs_limit))
        {
            return static_cast<FixedNumeric::q15_t>(-abs_limit);
        }
        return value;
    }

    static FixedNumeric::q15_t magnitudeApproxQ15(FixedNumeric::q15_t alpha,
                                                  FixedNumeric::q15_t beta)
    {
        const FixedNumeric::q15_t abs_alpha = FixedNumeric::absoluteQ15(alpha);
        const FixedNumeric::q15_t abs_beta = FixedNumeric::absoluteQ15(beta);
        const FixedNumeric::q15_t maximum =
            (abs_alpha > abs_beta) ? abs_alpha : abs_beta;
        const FixedNumeric::q15_t minimum =
            (abs_alpha > abs_beta) ? abs_beta : abs_alpha;
        return FixedNumeric::saturateQ15(
            static_cast<std::int32_t>(maximum) +
            ((static_cast<std::int32_t>(minimum) * 3) >> 3U));
    }

    static FixedNumeric::q15_t ratioQuality(FixedNumeric::q15_t value,
                                            FixedNumeric::q15_t reference)
    {
        if (reference <= 0)
        {
            return FixedNumeric::kQ15One;
        }
        if (value >= reference)
        {
            return FixedNumeric::kQ15One;
        }
        const std::int32_t ratio =
            (static_cast<std::int32_t>(value) * FixedNumeric::kQ15One) /
            static_cast<std::int32_t>(reference);
        return FixedNumeric::saturateQ15(ratio);
    }

    static FixedNumeric::q15_t radiansToPiQ15(float radians)
    {
        constexpr float kPi = 3.14159265358979323846f;
        if (!std::isfinite(radians) || radians <= 0.0f)
        {
            return 0;
        }
        return FixedNumeric::fromNormalized(radians / kPi);
    }

    static FixedNumeric::q15_t phaseErrorToPiQ15(std::int32_t phase_error)
    {
        return static_cast<FixedNumeric::q15_t>(phase_error >> 16);
    }

    static FixedNumeric::q15_t negateQ15(FixedNumeric::q15_t value)
    {
        return FixedNumeric::saturateQ15(-static_cast<std::int32_t>(value));
    }

    static FixedNumeric::phase_u32_t atan2PhaseQ15(FixedNumeric::q15_t y,
                                                   FixedNumeric::q15_t x)
    {
        if (x == 0 && y == 0)
        {
            return 0U;
        }

        static constexpr std::uint32_t kAtanPhase[16] = {
            536870912UL, 316933406UL, 167458907UL, 85004756UL,
            42667331UL, 21354465UL, 10679838UL, 5340245UL,
            2670163UL, 1335087UL, 667544UL, 333772UL,
            166886UL, 83443UL, 41722UL, 20861UL};

        std::int32_t xi = x;
        std::int32_t yi = y;
        std::uint32_t angle = 0U;
        if (xi < 0)
        {
            xi = -xi;
            yi = -yi;
            angle = 0x80000000UL;
        }

        for (std::uint8_t i = 0U; i < 16U; ++i)
        {
            const std::int32_t x_shift = xi >> i;
            const std::int32_t y_shift = yi >> i;
            if (yi > 0)
            {
                xi += y_shift;
                yi -= x_shift;
                angle += kAtanPhase[i];
            }
            else
            {
                xi -= y_shift;
                yi += x_shift;
                angle -= kAtanPhase[i];
            }
        }
        return angle;
    }

    bool configured_ = false;
    bool current_model_ready_ = false;
    uint8_t pole_pairs_ = 1U;
    float speed_base_rpm_ = 1.0f;
    float voltage_base_v_ = 1.0f;

    FixedNumeric::q15_t r_drop_q15_ = 0;
    FixedNumeric::q15_t z_limit_q15_ = 0;
    std::int32_t model_gain_q12_ = 0;
    std::int32_t sliding_gain_q12_ = 0;
    std::int32_t pll_kp_speed_q15_ = 0;
    std::int32_t pll_ki_speed_per_tick_q20_ = 0;
    std::uint32_t phase_step_q16_per_speed_q15_ = 0U;

    FixedNumeric::q15_t min_valid_speed_q15_ = 0;
    FixedNumeric::q15_t max_valid_pll_error_q15_ = FixedNumeric::kQ15One;
    FixedNumeric::q15_t min_signal_level_q15_ = 0;
    uint16_t convergence_ticks_ = 1U;
    uint16_t valid_ticks_ = 0U;

    FixedNumeric::phase_u32_t angle_est_phase_ = 0U;
    FixedNumeric::q15_t speed_est_q15_ = 0;
    std::int64_t pll_integral_speed_q35_ = 0;
    FixedNumeric::q15_t last_pll_error_q15_ = 0;
    FixedNumeric::q15_t i_alpha_est_ = 0;
    FixedNumeric::q15_t i_beta_est_ = 0;
    LowPassFilter bemf_alpha_lpf_;
    LowPassFilter bemf_beta_lpf_;
    ObserverEstimateQ15 last_estimate_;
};

using SlidingModeObserver = SlidingModeObserverQ15;

#else

/*
 * SlidingModeObserver — 滑模观测器 (SMO)
 *
 * 用于中高速无感FOC，通过滑模控制律从定子电流中估算转子电角度和角速度。
 * SMO 适合中高速运行 (反电动势足够大)，低速时配合 HFI 或 IF 启动使用。
 *
 * 工作原理:
 *   [1] 电流模型预测: i_est[k+1] = i_est[k] + (V - R*i_est - BEMF + z) / L * dt
 *   [2] 滑模控制律: z = sign(i_meas - i_est) * gain → 强制电流误差收敛到滑模面
 *   [3] 反电动势提取: BEMF = LPF(z) → 滤波得到等效反电动势 (包含角度信息)
 *   [4] 角度计算: θ_bemf = atan2(-BEMF_α, BEMF_β) → 从反电动势矢量提取角度
 *   [5] PLL跟踪: error = wrap(θ_bemf - θ_est) → PI → ω_est → ∫ → θ_est
 *
 * 滑模控制数学表达:
 *   L * di_est/dt = v - R*i_est - BEMF + k*sign(i_meas - i_est)
 *   其中 sign() 是符号函数，k 是滑模增益
 *
 * 典型调用时序:
 *   每个控制周期调用 update(v_α, v_β, i_α, i_β, dt, estimate_out)
 *   → 返回估计角度 angle_rad 和角速度 speed_rad_s
 */
class SlidingModeObserver
{
public:
    SlidingModeObserver() = default;

    /*
     * init() — 初始化SMO参数
     *
     * rs:       单相定子电阻 (Ω)，影响电流模型精度
     * ls:       单相定子电感 (H)，电感越小电流响应越快
     * smo_gain: 滑模增益，越大收敛越快但抖振也越大
     * smo_pll_kp:   PLL 比例增益 (跟踪反电动势角度)
     * smo_pll_ki:   PLL 积分增益
     * smo_bemf_lpf_cutoff_hz: 滑模注入量提取反电势时的一阶低通截止频率
     */
    void init(float rs, float ls, float smo_gain, float smo_pll_kp, float smo_pll_ki,
              float smo_bemf_lpf_cutoff_hz = kDefaultSmoBemfLpfCutoffHz)
    {
        rs_       = rs;
        ls_       = ls;
        smo_gain_ = smo_gain;
        pll_kp_   = smo_pll_kp;
        pll_ki_   = smo_pll_ki;
        bemf_lpf_cutoff_hz_ =
            (std::isfinite(smo_bemf_lpf_cutoff_hz) &&
             smo_bemf_lpf_cutoff_hz > 0.0f)
                ? smo_bemf_lpf_cutoff_hz
                : kDefaultSmoBemfLpfCutoffHz;
        reset();
    }

    /*
     * setValidityCriteria() — 设置SMO有效性判断条件
     *
     * min_elec_speed_rad_s:  最小有效电角速度 (rad/s)，低于此速度认为不可信
     * max_pll_error_rad:     最大允许PLL角度误差 (rad)，超出认为不可信
     * convergence_ticks:     连续满足条件的tick数后才标记valid
     */
    void setValidityCriteria(float min_elec_speed_rad_s,
                             float max_pll_error_rad,
                             uint16_t convergence_ticks,
                             float min_signal_level)
    {
        min_valid_speed_rad_s_ = min_elec_speed_rad_s;
        max_valid_pll_error_rad_ = max_pll_error_rad;
        convergence_ticks_ = convergence_ticks;
        min_signal_level_ = min_signal_level > 0.0f ? min_signal_level : 0.0f;
    }

    const ObserverEstimate& estimate() const
    {
        return last_estimate_;
    }

    void reset()
    {
        angle_est_    = 0.0f;
        speed_est_    = 0.0f;
        pll_integral_ = 0.0f;
        valid_ticks_  = 0U;
        last_pll_error_rad_ = 0.0f;
        i_alpha_est_ = 0.0f;
        i_beta_est_ = 0.0f;
        bemf_alpha_lpf_ = 0.0f;
        bemf_beta_lpf_ = 0.0f;
        current_model_ready_ = false;
        last_estimate_ = ObserverEstimate();
    }

    /* 用外部角度种子初始化 SMO 内部状态 (IF强拖→SMO切换时使用) */
    void seedAngle(float angle_rad)
    {
        angle_est_    = angle_rad;
        speed_est_    = 0.0f;
        pll_integral_ = 0.0f;
        valid_ticks_  = 0U;
        last_pll_error_rad_ = 0.0f;
        i_alpha_est_ = 0.0f;
        i_beta_est_ = 0.0f;
        bemf_alpha_lpf_ = 0.0f;
        bemf_beta_lpf_ = 0.0f;
        current_model_ready_ = false;
        last_estimate_ = ObserverEstimate();
    }

    /*
     * update() — SMO 主更新函数 (每个控制周期调用一次)
     *
     * 参数:
     *   v_alpha, v_beta — αβ轴电压 (Clarke坐标系)
     *   i_alpha, i_beta — αβ轴电流测量值
     *   dt              — 控制周期 (秒)
     *   estimate_out    — 输出估计结果 (角度/速度/有效性)
     *
     * 处理流程:
     *   [1] 电流模型就绪检查 (首次运行用实测电流初始化)
     *   [2] 电流误差计算 + 滑模符号函数
     *   [3] 电流模型预测 (欧拉法积分)
     *   [4] 低通滤波提取反电动势 (Z → BEMF)
     *   [5] atan2计算角度 + PLL跟踪
     *   [6] 有效性判断 (速度+误差阈值)
     *   [7] 填充输出结构体
     */
    void update(float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float dt,
                ObserverEstimate* estimate_out)
    {
        if (dt <= 0.0f)
        {
            fillEstimate(estimate_out, 0.0f);
            return;
        }

        // [1] 电流模型就绪检查: 首次运行时用实测电流初始化估计值
        if (!current_model_ready_)
        {
            i_alpha_est_ = i_alpha;
            i_beta_est_ = i_beta;
            current_model_ready_ = true;
            fillEstimate(estimate_out, 0.0f);
            return;
        }

        const float ls_safe = ls_ > 1.0e-9f ? ls_ : 1.0e-9f;  // 防止除零
        const float gain_abs = fabsf(smo_gain_);

        // [2] 电流误差 + 滑模符号函数
        //     z = gain * sign(i_meas - i_est)
        //     clampSymmetric 实现饱和函数替代纯符号函数，减少抖振
        const float i_error_alpha = i_alpha - i_alpha_est_;
        const float i_error_beta = i_beta - i_beta_est_;
        const float z_alpha =
            clampSymmetric(i_error_alpha * gain_abs, gain_abs);
        const float z_beta =
            clampSymmetric(i_error_beta * gain_abs, gain_abs);

        // [3] 电流模型预测 (欧拉法)
        //     L * di_est/dt = v - R*i_est - BEMF + z
        //     → di_est = (v - R*i_est - BEMF + z) / L
        //     → i_est[k+1] = i_est[k] + di_est * dt
        const float di_alpha =
            (v_alpha - rs_ * i_alpha_est_ - bemf_alpha_lpf_ + z_alpha) /
            ls_safe;
        const float di_beta =
            (v_beta - rs_ * i_beta_est_ - bemf_beta_lpf_ + z_beta) /
            ls_safe;
        i_alpha_est_ += di_alpha * dt;
        i_beta_est_ += di_beta * dt;

        // [4] 低通滤波提取反电动势
        //     BEMF = LPF(z)，滤除滑模控制律产生的高频抖振
        //     z 等效于反电动势信号，LPF 后得到平滑的角度信息
        const float alpha = lowPassAlphaFromCutoffHz(bemf_lpf_cutoff_hz_, dt);
        bemf_alpha_lpf_ += alpha * (z_alpha - bemf_alpha_lpf_);
        bemf_beta_lpf_ += alpha * (z_beta - bemf_beta_lpf_);

        const float signal_level =
            foc_sqrt_f32(bemf_alpha_lpf_ * bemf_alpha_lpf_ +
                         bemf_beta_lpf_ * bemf_beta_lpf_);

        // [5] atan2 计算角度 + PLL 跟踪
        //     θ_bemf = atan2(-BEMF_α, BEMF_β)
        //     error = wrap(θ_bemf - θ_est) → PI → ω_est → ∫ → θ_est
        float pll_error = 0.0f;
        if (signal_level > 1.0e-6f)  // 信号太弱则跳过更新
        {
            const float angle_obs =
                foc_atan2_f32(-bemf_alpha_lpf_, bemf_beta_lpf_);
            pll_error = wrapSignedAngle(angle_obs - angle_est_);
        }
        last_pll_error_rad_ = pll_error;

        // PLL: PI控制器 + 积分得角度
        pll_integral_ += pll_error * dt * pll_ki_;   // 积分项累加
        speed_est_ = pll_error * pll_kp_ + pll_integral_;  // PI = P*err + I
        angle_est_ += speed_est_ * dt;                // 角度积分

        // 归一化到 [0, 2π)
        while (angle_est_ > TWO_PI_f) angle_est_ -= TWO_PI_f;
        while (angle_est_ < 0.0f)    angle_est_ += TWO_PI_f;

        // [6] 有效性判断
        //     有效条件: 角速度 > 最小阈值 AND PLL误差 < 最大允许值
        //     连续满足 convergence_ticks_ 次后才标记valid
        const float speed_abs = fabsf(speed_est_);
        const float error_abs = fabsf(last_pll_error_rad_);
        const bool signal_ok =
            (min_signal_level_ <= 0.0f) || (signal_level >= min_signal_level_);
        const bool candidate =
            speed_abs >= min_valid_speed_rad_s_ &&
            signal_ok &&
            error_abs <= max_valid_pll_error_rad_;
        if (candidate)
        {
            if (valid_ticks_ < 0xFFFFU)
            {
                ++valid_ticks_;  // 连续有效计数
            }
        }
        else
        {
            valid_ticks_ = 0U;    // 失效则清零
        }

// [7] 填充输出结构体
        if (estimate_out != nullptr)
        {
            estimate_out->angle_rad = angle_est_;
            estimate_out->speed_rad_s = speed_est_;
            estimate_out->pll_error_rad = last_pll_error_rad_;
            estimate_out->signal_level = signal_level;
            estimate_out->quality = computeQuality(speed_abs, error_abs, signal_level);
            estimate_out->valid_ticks = valid_ticks_;
            estimate_out->valid = valid_ticks_ >= convergence_ticks_;
            last_estimate_ = *estimate_out;
        }

        /*
         * Ke 自动估算暂不放在 SMO tick 内执行。
         * 后续若需要自动 Ke，单独放到 Identify 层显式触发，避免观测器承担参数辨识职责。
         */
    }

    /* 当前 SMO tick 未更新 Ke 缓存，返回 0 表示未产生有效估计。 */
    float getEstimatedKe() const { return ke_estimated_v_per_rad_s_; }

    void update(float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float dt,
                float* angle_out, float* speed_out)
    {
        ObserverEstimate estimate;
        update(v_alpha, v_beta, i_alpha, i_beta, dt, &estimate);

        if (angle_out != nullptr)
        {
            *angle_out = estimate.angle_rad;
        }
        if (speed_out != nullptr)
        {
            *speed_out = estimate.speed_rad_s;
        }
    }

private:
    void fillEstimate(ObserverEstimate* estimate_out, float signal_level)
    {
        if (estimate_out == nullptr)
        {
            return;
        }
        estimate_out->angle_rad = angle_est_;
        estimate_out->speed_rad_s = speed_est_;
        estimate_out->pll_error_rad = last_pll_error_rad_;
        estimate_out->signal_level = signal_level;
        estimate_out->quality = 0.0f;
        estimate_out->valid_ticks = valid_ticks_;
        estimate_out->valid = false;
        last_estimate_ = *estimate_out;
    }

    /*
     * computeQuality() — 计算SMO估计质量
     *
     * 综合考虑速度达标程度和误差大小:
     *   speed_quality = speed / min_valid_speed (越接近阈值越高)
     *   error_quality = 1 - error / max_error     (误差越小越高)
     *   取两者最小值作为综合质量
     *
     * quality ∈ [0, 1]，1表示最佳
     */
    float computeQuality(float speed_abs, float error_abs, float signal_level) const
    {
        float speed_quality = 1.0f;
        if (min_valid_speed_rad_s_ > 0.0f)
        {
            speed_quality = speed_abs / min_valid_speed_rad_s_;
            if (speed_quality > 1.0f) speed_quality = 1.0f;
        }

        float error_quality = 1.0f;
        if (max_valid_pll_error_rad_ > 0.0f)
        {
            error_quality = 1.0f - (error_abs / max_valid_pll_error_rad_);
            if (error_quality < 0.0f) error_quality = 0.0f;
        }

        float signal_quality = 1.0f;
        if (min_signal_level_ > 0.0f)
        {
            signal_quality = signal_level / min_signal_level_;
            if (signal_quality > 1.0f) signal_quality = 1.0f;
        }

        const float quality =
            speed_quality < error_quality ? speed_quality : error_quality;
        return quality < signal_quality ? quality : signal_quality;
    }

    static float lowPassAlphaFromCutoffHz(float cutoff_hz, float dt)
    {
        const float alpha = TWO_PI_f * cutoff_hz * dt;
        if (alpha <= 0.0f) return 0.0f;
        if (alpha >= 1.0f) return 1.0f;
        return alpha;
    }

    static float clampSymmetric(float value, float limit)
    {
        if (limit <= 0.0f)
        {
            return 0.0f;        // 限幅为0表示禁用
        }
        if (value > limit) return limit;
        if (value < -limit) return -limit;
        return value;
    }

    static float wrapSignedAngle(float angle)
    {
        while (angle >  PI_f) angle -= TWO_PI_f;
        while (angle < -PI_f) angle += TWO_PI_f;
        return angle;
    }

    /* ================ 成员变量 ================ */

    /* --- 电机参数 --- */
    float rs_       = 0.1f;      // 单相定子电阻 (Ω)
    float ls_       = 0.001f;    // 单相定子电感 (H)

    /* --- SMO 参数 --- */
    float smo_gain_ = 1.0f;      // 滑模增益 (越大收敛越快, 噪声越大)

    /* --- 反电动势提取 --- */
    float bemf_alpha_lpf_ = 0.0f;    // α轴反电动势 (LPF输出)
    float bemf_beta_lpf_  = 0.0f;    // β轴反电动势 (LPF输出)
    float bemf_lpf_cutoff_hz_ = kDefaultSmoBemfLpfCutoffHz;  // 反电动势LPF截止频率 (Hz)

    /* --- PLL 锁相环 --- */
    float pll_kp_   = 2.0f;      // PLL 比例增益
    float pll_ki_   = 50.0f;     // PLL 积分增益
    float angle_est_    = 0.0f;  // 估计电角度 (rad), 输出结果
    float speed_est_    = 0.0f;  // 估计电角速度 (rad/s)
    float pll_integral_ = 0.0f;  // PLL 积分项累加
    float last_pll_error_rad_ = 0.0f;  // 最近一次PLL角度误差 (rad)

    /* --- 电流模型状态 --- */
    float i_alpha_est_ = 0.0f;   // α轴估计电流
    float i_beta_est_  = 0.0f;   // β轴估计电流
    bool current_model_ready_ = false;  // 电流模型就绪标志 (首次运行需初始化)

    /* --- 有效性判断参数 --- */
    float min_valid_speed_rad_s_ = 0.0f;    // 最小有效角速度阈值 (rad/s)
    float max_valid_pll_error_rad_ = 3.14159265358979323846f; // 最大PLL角度误差 (rad)
    float min_signal_level_ = 0.0f;
    uint16_t convergence_ticks_ = 1U;       // 需要的连续有效tick数
    uint16_t valid_ticks_ = 0U;             // 当前连续有效tick计数

/* --- 输出缓存 --- */
    ObserverEstimate last_estimate_;  // 最近一次估计结果缓�?

    /* --- 反电势常数 Ke 估计预留 ---
     * 当前 SMO 不在 update() 内计算 Ke，本缓存保持 0。
     * 后续若接入显式 Ke 辨识，应放在 Identify 层触发，避免估角路径承担参数辨识职责。
     */
    float ke_estimated_v_per_rad_s_ = 0.0f;

    static constexpr float PI_f     = 3.14159265358979323846f;
    static constexpr float TWO_PI_f = 6.28318530717958647692f;
};

#endif

} // namespace Lib_Motor
