#pragma once

#include "Observer_Base.h"
#include "../Control/Utils/FocMath.h"
#include <cmath>
#include <cstdint>

namespace Lib_Motor
{

/*
 * HighFreqInjectionObserver — 高频注入观测器 (HFI)
 *
 * 用于低速/零速时获取转子电角度，解决无感FOC在低速时反电动势过小无法观测的问题。
 *
 * 支持两种注入模式:
 *   1. SQUARE_ALTERNATE — 方波交变注入: 在d轴交替施加±Vhf，利用凸极效应解调
 *   2. SINE_CARRIER     — 正弦载波注入: 叠加高频正弦信号，通过PLL跟踪角度
 *
 * 工作原理:
 *   凸极电机的d轴和q轴电感不同 (Ld ≠ Lq)
 *   → 高频电压注入后，电流响应包含角度信息
 *   → 解调提取误差信号 → PLL跟踪 → 得到估计角度
 *
 * 典型调用时序 (每个控制周期):
 *   1. getInjectionCommand() → 获取注入电压指令 v_d/v_q
 *   2. 将 v_d/v_q 叠加到 FOC 电压指令上 → SVPWM 输出
 *   3. ADC 采样得到 i_alpha/i_beta
 *   4. update() → 解调 + PLL → 输出估计角度
 */
class HighFreqInjectionObserver
{
public:
    HighFreqInjectionObserver() = default;

    /*
     * init 重载1 — 正弦模式便捷初始化 (默认参数)
     * 仅指定频率/电压/PLL 增益, 其余取默认: 1 tick 锁定、无信号阈值、无斜坡
     */
    void init(float freq_hz, float voltage, float pll_kp, float pll_ki)
    {
        init(HfiInjectionMode::SINE_CARRIER, freq_hz, voltage, pll_kp, pll_ki,
             1U, 0.0f, 0.0f, 0.25f, 0.25f);
    }

    /*
     * init 重载2 — 正弦模式 + 有效性/斜坡配置
     * 额外指定 lock_ticks(锁定 tick 数)、min_signal_level(信号阈值)、voltage_ramp_time_s(电压爬升时间)
     */
    void init(float freq_hz, float voltage, float pll_kp, float pll_ki,
              uint16_t lock_ticks, float min_signal_level, float voltage_ramp_time_s)
    {
        init(HfiInjectionMode::SINE_CARRIER, freq_hz, voltage, pll_kp, pll_ki,
             lock_ticks, min_signal_level, voltage_ramp_time_s, 0.25f, 0.25f);
    }

    /*
     * init 重载3 — 主初始化 (全参数, 可选注入模式)
     *
     * mode                  SQUARE_ALTERNATE(方波) 或 SINE_CARRIER(正弦)
     * sine_freq_hz          注入频率 (Hz); 正弦=载波频率, 方波=仅作记录
     * voltage               目标注入电压幅值 (V)
     * pll_kp / pll_ki       PLL PI 增益 (角度误差 → 角速度)
     * lock_ticks            认定有效所需连续 tick 数
     * min_signal_level      解调信号强度阈值 (≤0 表示不判断)
     * voltage_ramp_time_s   注入电压爬升时间 (s, ≤0 为立即生效)
     * square_current_scale  方波二阶差分电流比例系数
     * sine_lpf_cutoff_ratio 正弦解调 LPF 截止频率 / 载频
     * square_lpf_cutoff_ratio 方波解调 LPF 截止频率 / 方波载频
     * pll_integral_limit_rad_s PLL 积分项限幅 (≤0 不限)
     * pll_speed_limit_rad_s    PLL 速度输出限幅 (≤0 不限)
     *
     * 所有非法(≤0)参数会被夹到安全默认值, 随后调用 reset() 清状态
     */
    void init(HfiInjectionMode mode, float sine_freq_hz, float voltage,
              float pll_kp, float pll_ki, uint16_t lock_ticks,
              float min_signal_level, float voltage_ramp_time_s,
              float square_current_scale, float sine_lpf_cutoff_ratio,
              float square_lpf_cutoff_ratio = 0.025f,
              float pll_integral_limit_rad_s = 0.0f,
              float pll_speed_limit_rad_s = 0.0f)
    {
        injection_mode_ = mode;
        sine_freq_hz_ = sine_freq_hz;
        pll_kp_ = pll_kp;
        pll_ki_ = pll_ki;
        lock_ticks_ = lock_ticks > 0U ? lock_ticks : 1U;
        min_signal_level_ = min_signal_level > 0.0f ? min_signal_level : 0.0f;
        square_current_scale_ = square_current_scale > 0.0f ? square_current_scale : 0.25f;
        sine_lpf_cutoff_ratio_ = sine_lpf_cutoff_ratio > 0.0f
            ? sine_lpf_cutoff_ratio
            : 0.25f;
        square_lpf_cutoff_ratio_ = square_lpf_cutoff_ratio > 0.0f
            ? square_lpf_cutoff_ratio
            : 0.025f;
        pll_integral_limit_rad_s_ = pll_integral_limit_rad_s > 0.0f
            ? pll_integral_limit_rad_s
            : 0.0f;
        pll_speed_limit_rad_s_ = pll_speed_limit_rad_s > 0.0f
            ? pll_speed_limit_rad_s
            : 0.0f;
        voltage_ramp_time_s_ = voltage_ramp_time_s > 0.0f ? voltage_ramp_time_s : 0.0f;
        setInjectionVoltage(voltage);
        reset();
    }

    /*
     * reset — 重置观测器全部运行状态
     *
     * 清零 PLL(角度/速度/积分)、载波相位、当前注入电压、解调输出、
     * 方波调试量与有效计数, 并置 enabled_=false。
     * 注意: 不清基本参数(频率/增益/限幅等), 仅清状态; 内部调用 resetSquareState()。
     */
    void reset()
    {
        angle_est_ = 0.0f;
        speed_est_ = 0.0f;
        pll_integral_ = 0.0f;
        hf_phase_ = 0.0f;
        injection_voltage_now_ = 0.0f;
        demod_error_lpf_ = 0.0f;
        demod_signal_lpf_ = 0.0f;
        last_square_demod_alpha_ = 0.0f;
        last_square_demod_beta_ = 0.0f;
        last_square_demod_d_ = 0.0f;
        last_square_demod_q_ = 0.0f;
        last_square_raw_demod_alpha_ = 0.0f;
        last_square_raw_demod_beta_ = 0.0f;
        last_square_meas_2theta_ = 0.0f;
        valid_ticks_ = 0U;
        enabled_ = false;
        resetSquareState();
    }

    // 运行时使能/禁用 HFI。配合 updateInjectionAmplitude() 决定注入电压目标值 (enabled→Vhf, 否则→0)
    void setEnabled(bool enabled)
    {
        enabled_ = enabled;
    }

    /*
     * setInjectionVoltage — 设置目标注入电压幅值 (V)
     * 并据 voltage_ramp_time_s_ 重算爬升速率 voltage_ramp_rate_v_s_ = Vhf / ramp_time
     * (ramp_time≤0 时速率=0, 即立即生效)。负输入被夹到 0。
     */
    void setInjectionVoltage(float voltage)
    {
        hf_voltage_ = voltage > 0.0f ? voltage : 0.0f;
        voltage_ramp_rate_v_s_ = voltage_ramp_time_s_ > 0.0f
            ? hf_voltage_ / voltage_ramp_time_s_
            : 0.0f;
    }

    /*
     * setPllGains — 运行时调整 PLL PI 增益
     * 安全约定: ki=0 时清积分项; 若 kp 同时为 0, 再清 speed_est_ (防止残留速度驱动电机)
     */
    void setPllGains(float pll_kp, float pll_ki)
    {
        pll_kp_ = pll_kp;
        pll_ki_ = pll_ki;
        if (pll_ki_ == 0.0f)
        {
            pll_integral_ = 0.0f;
            if (pll_kp_ == 0.0f)
            {
                speed_est_ = 0.0f;
            }
        }
    }

    /*
     * setPllLimits — 设置 PLL 积分项与速度输出限幅 (rad/s, ≤0 表示不限)
     * 修改后立即对现有 pll_integral_ / speed_est_ 做对称限幅, 避免残留超限值
     */
    void setPllLimits(float integral_limit_rad_s, float speed_limit_rad_s)
    {
        pll_integral_limit_rad_s_ = integral_limit_rad_s > 0.0f
            ? integral_limit_rad_s
            : 0.0f;
        pll_speed_limit_rad_s_ = speed_limit_rad_s > 0.0f
            ? speed_limit_rad_s
            : 0.0f;
        pll_integral_ = clampSymmetric(pll_integral_,
                                       pll_integral_limit_rad_s_);
        speed_est_ = clampSymmetric(speed_est_, pll_speed_limit_rad_s_);
    }

    // 设置锁定所需连续有效 tick 数 (≤0 夹到 1)
    void setLockTicks(uint16_t lock_ticks)
    {
        lock_ticks_ = lock_ticks > 0U ? lock_ticks : 1U;
    }

    // 设置最小有效信号阈值 (≤0 表示不做信号强度判断)
    void setMinSignalLevel(float min_signal_level)
    {
        min_signal_level_ = min_signal_level > 0.0f ? min_signal_level : 0.0f;
    }

    // 设置方波二阶差分电流比例系数 (>0 才接受)
    void setSquareCurrentScale(float square_current_scale)
    {
        if (square_current_scale > 0.0f)
        {
            square_current_scale_ = square_current_scale;
        }
    }

    // 设置正弦解调 LPF 截止频率/载频比值 (>0 才接受)
    void setSineLpfCutoffRatio(float sine_lpf_cutoff_ratio)
    {
        if (sine_lpf_cutoff_ratio > 0.0f)
        {
            sine_lpf_cutoff_ratio_ = sine_lpf_cutoff_ratio;
        }
    }

    // 设置方波解调 LPF 截止频率/方波载频比值 (>0 才接受)
    void setSquareLpfCutoffRatio(float square_lpf_cutoff_ratio)
    {
        if (square_lpf_cutoff_ratio > 0.0f)
        {
            square_lpf_cutoff_ratio_ = square_lpf_cutoff_ratio;
        }
    }

    /*
     * update 重载 (4参) — 便捷接口, 仅传 dq 电流
     * 内部把 i_d/i_q 同时当作 αβ 传入 6 参版。
     * 注意: 仅正弦模式可用 (正弦解调用 dq); 方波模式需真实 αβ 电流, 请用 6 参重载。
     */
    void update(float i_d, float i_q, float dt, ObserverEstimate* estimate_out)
    {
        update(i_d, i_q, i_d, i_q, dt, estimate_out);
    }

    /*
     * update() — HFI 主更新函数 (每个控制周期调用一次)
     *
     * 参数:
     *   i_alpha, i_beta — αβ轴电流 (Clarke变换输出, 方波解调用)
     *   i_d, i_q        — dq轴电流 (Park变换输出, 正弦解调用)
     *   dt              — 控制周期 (秒)
     *   estimate_out    — 输出估计结果 (角度/速度/有效性)
     *
     * 流程:
     *   1. 更新注入电压幅值 (斜坡爬升)
     *   2. 根据注入模式选择解调算法
     *   3. 判断HFI有效性
     *   4. 填充输出结构体
     */
    void update(float i_alpha, float i_beta, float i_d, float i_q,
                float dt, ObserverEstimate* estimate_out)
    {
        if (dt <= 0.0f)
        {
            fillEstimate(estimate_out);
            return;
        }

        updateInjectionAmplitude(dt);  // 注入电压斜坡爬升

        if (injection_mode_ == HfiInjectionMode::SQUARE_ALTERNATE)
        {
            updateSquareDemod(i_alpha, i_beta, dt);  // 方波解调 (用αβ电流)
        }
        else
        {
            updateSineDemod(i_d, i_q, dt);  // 正弦解调 (用dq电流)
        }

        updateValidity();   // 判断HFI是否有效
        fillEstimate(estimate_out);  // 填充输出
    }

    /*
     * getInjectionSignal — 取当前注入瞬时电压值
     * 方波: injection_voltage_now_ × square_next_sign_ (±Vhf, 不翻转符号)
     * 正弦: injection_voltage_now_ × sin(hf_phase_)
     * 与 getInjectionCommand() 区别: 本函数只返回标量值, 不翻转方波符号也不填指令结构体
     */
    float getInjectionSignal() const
    {
        if (injection_mode_ == HfiInjectionMode::SQUARE_ALTERNATE)
        {
            return injection_voltage_now_ * static_cast<float>(square_next_sign_);
        }
        return injection_voltage_now_ * sinf(hf_phase_);
    }

    /*
     * getInjectionCommand() — 获取HFI注入电压指令
     *
     * 返回: HfiInjectionCommand {v_d, v_q, enabled}
     *   v_d     — d轴注入电压 (方波模式: ±Vhf, 正弦模式: Vhf*sin(ωt))
     *   v_q     — q轴注入电压 (始终为0, 只注入d轴)
     *   enabled — 注入是否有效
     *
     * 方波注入时序:
     *   每次调用时 v_d 符号翻转 (+V → -V → +V → ...)
     *   square_sample_sign_ 记录当前符号, 供解调时使用
     *   square_next_sign_   预存下次符号, 实现交替注入
     */
    HfiInjectionCommand getInjectionCommand()
    {
        HfiInjectionCommand command;
        command.enabled = injection_voltage_now_ > 0.0f;  // 注入电压有效才启用
        if (!command.enabled)
        {
            square_sample_sign_ = 0.0f;  // 未启用时清零
            square_next_sign_ = 1;
            return command;
        }

        if (injection_mode_ == HfiInjectionMode::SQUARE_ALTERNATE)
        {
            // 方波模式: d轴注入 ±Vhf, q轴为0
            command.v_d = injection_voltage_now_ * static_cast<float>(square_next_sign_);
            command.v_q = 0.0f;
            square_sample_sign_ = static_cast<float>(square_next_sign_);  // 记录当前符号供解调
            square_next_sign_ = -square_next_sign_;  // 翻转下次符号
        }
        else
        {
            // 正弦模式: d轴注入 Vhf*sin(ωt)
            command.v_d = getInjectionSignal();
            command.v_q = 0.0f;
        }
        return command;
    }

private:
    /*
     * updateSineDemod — 正弦 HFI 解调
     *
     * 流程:
     *   1. updateCarrier() 推进载波相位
     *   2. 误差采样 = i_q × sin(载波)  (凸极使 q 轴含角度误差, sin 同步整流)
     *   3. 信号强度 = |i_d×sin| + |i_q×cos|
     *   4. 一阶 LPF 平滑误差与信号
     *   5. updatePll() 跟踪角度
     *
     * 注意: 当前为可运行骨架, 凸极误差模型与电机/拓扑相关, 需在实际 bring-up 中继续标定。
     */
    void updateSineDemod(float i_d, float i_q, float dt)
    {
        updateCarrier(dt);

        float carrier_sin, carrier_cos;
        foc_sin_cos_f32(hf_phase_, carrier_sin, carrier_cos);

        /*
         * 正弦解调的首个可执行骨架；最终凸极误差模型与电机/拓扑相关，
         * 需要在实际 bring-up 中继续标定。
         */
        const float error_sample = i_q * carrier_sin;
        const float signal_sample = fabsf(i_d * carrier_sin) + fabsf(i_q * carrier_cos);
        const float alpha = lowPassAlpha(dt);
        demod_error_lpf_ += alpha * (error_sample - demod_error_lpf_);
        demod_signal_lpf_ += alpha * (signal_sample - demod_signal_lpf_);

        updatePll(demod_error_lpf_, dt);
    }

    /*
     * updateSquareDemod() — 方波HFI解调核心函数
     *
     * 原理:
     *   1. 二阶差分提取高频电流分量 (去除低频基波)
     *   2. 同步整流: 用注入符号解调，提取凸极误差
     *   3. Park变换: 将误差映射到估计坐标系
     *   4. PLL跟踪: 误差→角度估计
     *
     * 二阶差分公式:
     *   hf[n] = (i[n] - 2*i[n-1] + i[n-2]) * scale
     *   作用: 高通滤波，提取方波注入引起的高频电流响应
     *
     * 同步整流:
     *   因为getInjectionCommand()已提前翻转符号，
     *   所以用 -square_sample_sign_ 作为解调符号
     *
     * 参数:
     *   i_alpha, i_beta — αβ轴电流采样值
     *   dt              — 控制周期
     */
    void updateSquareDemod(float i_alpha, float i_beta, float dt)
    {
        // [1] 二阶差分提取高频分量 (高通滤波)
        //     hf = (i[n] - 2*i[n-1] + i[n-2]) * scale
        const float hf_alpha =
            (i_alpha - 2.0f * square_i_alpha_z1_ + square_i_alpha_z2_) *
            square_current_scale_;
        const float hf_beta =
            (i_beta - 2.0f * square_i_beta_z1_ + square_i_beta_z2_) *
            square_current_scale_;

        // [2] 就绪判断: 至少2个历史周期 + 高频分量已更新 + 注入符号有效
        const bool ready =
            square_history_count_ >= 2U &&
            square_hf_ready_ &&
            square_sample_sign_ != 0.0f;

        if (ready)
        {
            /*
             * [3] 同步整流: 用反号解调
             * 因为getInjectionCommand()已翻转符号，
             * 当前hf_alpha/beta是由上一拍注入产生的，所以用反号解调
            */
            const float sync_sign = -square_sample_sign_;
            const float raw_demod_alpha = (hf_alpha - square_hf_alpha_z1_) * sync_sign;
            const float raw_demod_beta = (hf_beta - square_hf_beta_z1_) * sync_sign;

            const float est_theta = angle_est_;
            float est_s, est_c;
            foc_sin_cos_f32(est_theta, est_s, est_c);
            const float demod_d =
                raw_demod_alpha * est_c + raw_demod_beta * est_s;
            const float demod_q =
                -raw_demod_alpha * est_s + raw_demod_beta * est_c;

            const float signal_sample =
                foc_sqrt_f32(raw_demod_alpha * raw_demod_alpha +
                             raw_demod_beta * raw_demod_beta);
            float error_sample = 0.0f;
            if (signal_sample > 1.0e-6f)
            {
                const float phase_error = -demod_q / signal_sample;
                error_sample = 0.5f * clampSymmetric(phase_error, 1.0f);
            }
            const float meas_2theta_error = foc_atan2_f32(-demod_q, demod_d);

            last_square_demod_alpha_ = raw_demod_alpha;
            last_square_demod_beta_ = raw_demod_beta;
            last_square_demod_d_ = demod_d;
            last_square_demod_q_ = demod_q;
            last_square_raw_demod_alpha_ = raw_demod_alpha;
            last_square_raw_demod_beta_ = raw_demod_beta;
            last_square_meas_2theta_ = meas_2theta_error;

            // [5] 更新PLL，跟踪角度
            const float alpha = squareLowPassAlpha(dt);
            demod_error_lpf_ += alpha * (error_sample - demod_error_lpf_);
            demod_signal_lpf_ += alpha * (signal_sample - demod_signal_lpf_);
            updatePll(demod_error_lpf_, dt);
        }
        else
        {
            // 未就绪时清零输出
            demod_error_lpf_ = 0.0f;
            demod_signal_lpf_ = 0.0f;
            last_square_demod_alpha_ = 0.0f;
            last_square_demod_beta_ = 0.0f;
            last_square_demod_d_ = 0.0f;
            last_square_demod_q_ = 0.0f;
            last_square_raw_demod_alpha_ = 0.0f;
            last_square_raw_demod_beta_ = 0.0f;
            last_square_meas_2theta_ = 0.0f;
        }

        // [6] 更新历史缓存 (移位寄存器，保存最近2拍数据)
        square_i_alpha_z2_ = square_i_alpha_z1_;  // i_alpha[n-2] = i_alpha[n-1]
        square_i_beta_z2_ = square_i_beta_z1_;
        square_i_alpha_z1_ = i_alpha;             // i_alpha[n-1] = i_alpha[n]
        square_i_beta_z1_ = i_beta;
        square_hf_alpha_z1_ = hf_alpha;           // 保存当前高频分量供下次差分
        square_hf_beta_z1_ = hf_beta;
        square_hf_ready_ = true;
        if (square_history_count_ < 2U)
        {
            ++square_history_count_;  // 累计历史计数
        }
    }

    /*
     * updatePll() — PLL(锁相环)角度跟踪
     *
     * PI控制器: 角度误差 → 角速度估计 → 积分得角度
     *   speed_est = Kp * error + Ki * ∫error·dt
     *   angle_est += speed_est * dt
     *
     * 当error→0时，angle_est跟踪真实电角度
     */
    void updatePll(float error_sample, float dt)
    {
        pll_integral_ += error_sample * pll_ki_ * dt;           // 积分项累加
        pll_integral_ = clampSymmetric(pll_integral_,
                                       pll_integral_limit_rad_s_);
        speed_est_ = pll_kp_ * error_sample + pll_integral_;   // PI输出 = 角速度
        speed_est_ = clampSymmetric(speed_est_, pll_speed_limit_rad_s_);
        angle_est_ += speed_est_ * dt;                          // 积分得角度
        wrapAngle();                                            // 归一化到[0, 2π)
    }

    /*
     * updateValidity() — 判断HFI是否有效
     *
     * 有效条件 (全部满足):
     *   1. enabled_ = true (HFI功能已启用)
     *   2. amplitude_ready: 注入电压已爬升到目标值的95%
     *   3. signal_ready: 解调信号强度超过最小阈值
     *
     * 连续满足条件 lock_ticks_ 个周期后，valid=true
     * 一旦失效，valid_ticks_ 立即清零
     */
    void updateValidity()
    {
        const bool amplitude_ready =
            hf_voltage_ <= 0.0f ||
            injection_voltage_now_ >= hf_voltage_ * 0.95f;  // 注入幅值已爬升到95%
        const bool signal_ready =
            min_signal_level_ > 0.0f &&
            demod_signal_lpf_ >= min_signal_level_;          // 解调信号强度足够

        if (enabled_ && amplitude_ready && signal_ready)
        {
            if (valid_ticks_ < lock_ticks_)
            {
                ++valid_ticks_;  // 连续有效计数
            }
        }
        else
        {
            valid_ticks_ = 0U;  // 失效则清零
        }
    }

    // 推进正弦载波相位: hf_phase_ += 2π·f·dt, 并归一化到 [0, 2π)。仅正弦模式使用
    void updateCarrier(float dt)
    {
        hf_phase_ += sine_freq_hz_ * TWO_PI * dt;
        while (hf_phase_ >= TWO_PI) hf_phase_ -= TWO_PI;
        while (hf_phase_ < 0.0f) hf_phase_ += TWO_PI;
    }

    /*
     * updateInjectionAmplitude — 注入电压幅值斜坡控制
     * target = enabled ? hf_voltage_ : 0; 朝 target 按 voltage_ramp_rate_v_s_·dt 步进并夹到目标。
     * ramp_rate≤0 时直接置 target (无斜坡, 立即生效)。
     */
    void updateInjectionAmplitude(float dt)
    {
        const float target = enabled_ ? hf_voltage_ : 0.0f;
        if (voltage_ramp_rate_v_s_ <= 0.0f)
        {
            injection_voltage_now_ = target;
            return;
        }

        const float step = voltage_ramp_rate_v_s_ * dt;
        if (injection_voltage_now_ < target)
        {
            injection_voltage_now_ += step;
            if (injection_voltage_now_ > target) injection_voltage_now_ = target;
        }
        else if (injection_voltage_now_ > target)
        {
            injection_voltage_now_ -= step;
            if (injection_voltage_now_ < target) injection_voltage_now_ = target;
        }
    }

    // 正弦解调 LPF 系数: 截止频率 = 注入载频 × sine_lpf_cutoff_ratio_
    float lowPassAlpha(float dt) const
    {
        const float cutoff_hz = sine_freq_hz_ * sine_lpf_cutoff_ratio_;
        return lowPassAlphaFromCutoffHz(cutoff_hz, dt);
    }

    /*
     * 方波解调 LPF 系数: 方波载频取 0.5/dt (每两拍翻转一次),
     * 截止频率 = 方波载频 × square_lpf_cutoff_ratio_。dt≤0 返回 0
     */
    float squareLowPassAlpha(float dt) const
    {
        if (dt <= 0.0f)
        {
            return 0.0f;
        }
        const float square_carrier_hz = 0.5f / dt;
        const float cutoff_hz = square_carrier_hz * square_lpf_cutoff_ratio_;
        return lowPassAlphaFromCutoffHz(cutoff_hz, dt);
    }

    /*
     * 由截止频率计算一阶 LPF 系数 α ≈ 2π·fc·dt (前向欧拉近似)
     * 限幅到 [0,1]: ≤0 返回 0 (不更新), ≥1 返回 1 (直通)
     */
    float lowPassAlphaFromCutoffHz(float cutoff_hz, float dt) const
    {
        const float alpha = TWO_PI * cutoff_hz * dt;
        if (alpha <= 0.0f) return 0.0f;
        if (alpha >= 1.0f) return 1.0f;
        return alpha;
    }

    // 对称限幅到 [-limit, +limit]; limit≤0 表示不限幅, 原值返回
    static float clampSymmetric(float value, float limit)
    {
        if (limit <= 0.0f)
        {
            return value;
        }
        if (value > limit) return limit;
        if (value < -limit) return -limit;
        return value;
    }

    // 有符号角度归一化到 (-π, π], 双角误差 (meas_2theta - est_2theta) 用
    static float wrapSignedAngle(float angle)
    {
        while (angle > PI) angle -= TWO_PI;
        while (angle < -PI) angle += TWO_PI;
        return angle;
    }

    // 无符号角度归一化到 [0, 2π), 估计电角度 angle_est_ 用
    void wrapAngle()
    {
        while (angle_est_ >= TWO_PI) angle_est_ -= TWO_PI;
        while (angle_est_ < 0.0f) angle_est_ += TWO_PI;
    }

    /*
     * fillEstimate — 将内部状态写入 ObserverEstimate 输出结构
     * 填入角度/速度/误差/信号/方波调试量, 计算 quality, 置 valid=(valid_ticks_>=lock_ticks_)
     * estimate_out 为 nullptr 时直接返回
     */
    void fillEstimate(ObserverEstimate* estimate_out) const
    {
        if (estimate_out == nullptr)
        {
            return;
        }

        estimate_out->angle_rad = angle_est_;
        estimate_out->speed_rad_s = speed_est_;
        estimate_out->pll_error_rad = demod_error_lpf_;
        estimate_out->signal_level = demod_signal_lpf_;
        estimate_out->hfi_demod_alpha = last_square_demod_alpha_;
        estimate_out->hfi_demod_beta = last_square_demod_beta_;
        estimate_out->hfi_demod_d = last_square_demod_d_;
        estimate_out->hfi_demod_q = last_square_demod_q_;
        estimate_out->hfi_raw_demod_alpha = last_square_raw_demod_alpha_;
        estimate_out->hfi_raw_demod_beta = last_square_raw_demod_beta_;
        estimate_out->hfi_meas_2theta = last_square_meas_2theta_;
        estimate_out->quality = computeQuality();
        estimate_out->valid_ticks = valid_ticks_;
        estimate_out->valid = valid_ticks_ >= lock_ticks_;
    }

    /*
     * computeQuality — 信号质量分 [0,1] = demod_signal_lpf_ / min_signal_level_
     * min_signal_level_≤0 (未设阈值) 时返回 0 (不评估质量)
     */
    float computeQuality() const
    {
        if (min_signal_level_ <= 0.0f)
        {
            return 0.0f;
        }

        float quality = demod_signal_lpf_ / min_signal_level_;
        if (quality > 1.0f) quality = 1.0f;
        if (quality < 0.0f) quality = 0.0f;
        return quality;
    }

    /*
     * resetSquareState() — 重置方波解调状态
     *
     * 清零所有历史缓存和状态标志，
     * 通常在HFI启用/禁用或模式切换时调用
     */
    void resetSquareState()
    {
        square_i_alpha_z1_ = 0.0f;      // α轴电流 z-1 拍
        square_i_alpha_z2_ = 0.0f;      // α轴电流 z-2 拍
        square_i_beta_z1_ = 0.0f;       // β轴电流 z-1 拍
        square_i_beta_z2_ = 0.0f;       // β轴电流 z-2 拍
        square_hf_alpha_z1_ = 0.0f;     // α轴高频分量 z-1 拍
        square_hf_beta_z1_ = 0.0f;      // β轴高频分量 z-1 拍
        square_history_count_ = 0U;     // 历史数据计数 (需≥2才能解调)
        square_hf_ready_ = false;       // 高频分量就绪标志
        square_sample_sign_ = 0.0f;     // 当前采样对应的注入符号
        square_next_sign_ = 1;          // 下一次注入符号 (+1/-1)
    }

    /* ================ 成员变量 ================ */

    /* --- 注入模式与基本参数 --- */
    HfiInjectionMode injection_mode_ = HfiInjectionMode::SINE_CARRIER;  // 注入模式
    float sine_freq_hz_ = 1000.0f;     // 注入频率 (Hz), 方波模式下为基频
    float hf_voltage_ = 10.0f;         // 目标注入电压幅值 (V)
    float pll_kp_ = 2.0f;              // PLL 比例增益
    float pll_ki_ = 50.0f;             // PLL 积分增益
    float pll_integral_limit_rad_s_ = 0.0f;
    float pll_speed_limit_rad_s_ = 0.0f;

    /* --- PLL 状态 --- */
    float angle_est_ = 0.0f;           // 估计电角度 (rad), 输出结果
    float speed_est_ = 0.0f;           // 估计电角速度 (rad/s)
    float pll_integral_ = 0.0f;        // PLL 积分项累加

    /* --- 正弦注入相关 --- */
    float hf_phase_ = 0.0f;            // 正弦载波相位 (rad), 仅正弦模式用

    /* --- 注入电压斜坡控制 --- */
    float injection_voltage_now_ = 0.0f;   // 当前注入电压 (斜坡爬升中)
    float voltage_ramp_time_s_ = 0.0f;     // 电压爬升时间常数 (s)
    float voltage_ramp_rate_v_s_ = 0.0f;   // 电压爬升速率 (V/s)

    /* --- 解调输出 --- */
    float demod_error_lpf_ = 0.0f;     // 解调误差 (角度误差信号)
    float demod_signal_lpf_ = 0.0f;    // 解调信号强度 (用于有效性判断)

    /* --- 有效性判断参数 --- */
    float min_signal_level_ = 0.0f;    // 最小有效信号阈值

    /* --- 方波解调参数 --- */
    float square_current_scale_ = 0.25f;   // 二阶差分电流比例系数
    float square_lpf_cutoff_ratio_ = 0.025f;

    /* --- 正弦解调参数 --- */
    float sine_lpf_cutoff_ratio_ = 0.25f;  // 解调LPF截止频率与载频比值

    /* --- 方波解调状态 (二阶差分历史缓存) --- */
    float square_i_alpha_z1_ = 0.0f;   // i_α[n-1]
    float square_i_alpha_z2_ = 0.0f;   // i_α[n-2]
    float square_i_beta_z1_ = 0.0f;    // i_β[n-1]
    float square_i_beta_z2_ = 0.0f;    // i_β[n-2]
    float square_hf_alpha_z1_ = 0.0f;  // hf_α[n-1] (高频分量历史)
    float square_hf_beta_z1_ = 0.0f;   // hf_β[n-1]

    /* --- 方波解调控制 --- */
    uint16_t lock_ticks_ = 1U;         // 锁定所需连续有效tick数
    uint16_t valid_ticks_ = 0U;        // 当前连续有效计数
    uint8_t square_history_count_ = 0U; // 历史数据计数 (≥2才能解调)
    bool square_hf_ready_ = false;     // 高频分量就绪标志
    bool enabled_ = false;             // HFI功能使能标志

    /* --- 方波注入符号控制 --- */
    float square_sample_sign_ = 0.0f;  // 当前采样对应的注入符号 (+1/-1)
    int8_t square_next_sign_ = 1;      // 下一次注入符号 (交替翻转)

    float last_square_demod_alpha_ = 0.0f;
    float last_square_demod_beta_ = 0.0f;
    float last_square_demod_d_ = 0.0f;
    float last_square_demod_q_ = 0.0f;
    float last_square_raw_demod_alpha_ = 0.0f;
    float last_square_raw_demod_beta_ = 0.0f;
    float last_square_meas_2theta_ = 0.0f;
    static constexpr float PI = 3.14159265358979323846f;
    static constexpr float TWO_PI = 6.28318530717958647692f;
};

} // namespace Lib_Motor
