#pragma once

#include "Observer_Base.h"
#include "../Control/Utils/FocMath.h"
#include <cmath>

namespace Lib_Motor
{

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
     */
    void init(float rs, float ls, float smo_gain, float smo_pll_kp, float smo_pll_ki)
    {
        rs_       = rs;
        ls_       = ls;
        smo_gain_ = smo_gain;
        pll_kp_   = smo_pll_kp;
        pll_ki_   = smo_pll_ki;
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

        // [8] 反电势常数 Ke 滑窗估计 (本轮新增, 仅采集不消费)
        // SMO 收敛 & ω 足够大时, ke_inst = |bemf| / |ω| 作 Ke 瞬时估计;
        // 慢滤波确保不被噪声推高; |ω| 阈值用 min_valid_speed/2 防低速段信号弱时估错;
        if (valid_ticks_ >= convergence_ticks_ &&
            speed_abs > min_valid_speed_rad_s_ * 0.5f &&
            speed_abs > 1.0f)                     // 最低 1 rad/s 防除零
        {
            const float ke_inst = signal_level / speed_abs;     // V/(rad/s)
            // 慢滤波: 时间常数约 50 ms @ 20kHz 控制
            ke_estimated_v_per_rad_s_ = ke_estimated_v_per_rad_s_ * 0.99f + ke_inst * 0.01f;
        }
    }

    /* 取当前估计的 Ke, 供 MotorManager::ctx_ 与 API 拉取缓存到 cfg.physical.ke_*_learned */
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
    float bemf_lpf_cutoff_hz_ = 1000.0f;  // 反电动势LPF截止频率 (Hz)

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

    /* --- 反电势常数 Ke 估计 (SMO 收敛后副产物) ---
     * 滑窗平均: ke_inst = |bemf_lpf| / |ω_elec|
     * 慢滤波确保 Ke 不被瞬时噪声推高 (系数 0.99/0.01 约 50 ms 时间常数 @20kHz)
     * 仅 valid + |ω| > min_valid_speed/2 时写入; 防止低速段 bemf 信号弱时估错。
     */
    float ke_estimated_v_per_rad_s_ = 0.0f;

static constexpr float PI_f     = 3.14159265358979323846f;
    static constexpr float TWO_PI_f = 6.28318530717958647692f;
};

} // namespace Lib_Motor
