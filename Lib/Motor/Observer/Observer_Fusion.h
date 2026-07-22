/*
 * Observer_Fusion.h — 观测器融合 (互补滤波)
 *
 * 原理: 根据不同转速区间, 将 HFI (零低速) 和 SMO (中高速) 的输出做加权混合
 *
 *   转速区间与权重:
 *
 *     weight_hfi │
 *         1.0    │████████████████
 *                │                ██
 *                │                  ██
 *                │                    ██
 *         0.0    │████████████████████████
 *                └──────────────────────────→ speed_rpm
 *                ↑ blend_start     ↑ blend_end
 *                (纯 HFI)          (纯 SMO)
 *
 *   融合公式 (每 tick):
 *     w_hfi = map(speed, blend_start, blend_end, 1.0f, 0.0f)
 *     w_hfi = clamp(w_hfi, 0.0f, 1.0f)
 *     w_smo = 1.0f - w_hfi
 *
 *     angle = w_hfi × angle_hfi + w_smo × angle_smo
 *     speed = w_hfi × speed_hfi + w_smo × speed_smo
 *
 * 依赖:
 *   - HFI 观测器 (HighFreqInjectionObserver) — 零低速角度估算
 *   - SMO 观测器 (SlidingModeObserver) — 中高速角度估算
 *   - 二者必须均已实现并初始化
 *
 * ★ 当前 HFI 未实现, FusionObserver 暂未启用
 *   runObserverLoop() 中暂用 SMO 占位
 */

#pragma once

#include "Observer_SMO.h"
#include "Observer_HFI.h"
#include <cmath>
#include <algorithm>

namespace Lib_Motor
{

class FusionObserver
{
public:
    FusionObserver() = default;

    /*
     * init — 初始化 SMO 和 HFI 观测器 (融合需要两者)
     * 参数分别传递给 SMO::init() 和 HFI::init()
     */
    void init(float rs, float ls, float smo_gain, float smo_pll_kp, float smo_pll_ki,
              float hfi_freq_hz, float hfi_voltage, float hfi_pll_kp, float hfi_pll_ki)
    {
        smo_.init(rs, ls, smo_gain, smo_pll_kp, smo_pll_ki);
        hfi_.init(hfi_freq_hz, hfi_voltage, hfi_pll_kp, hfi_pll_ki);
        reset();
    }

    void reset()
    {
        smo_.reset();
        hfi_.reset();
        angle_out_ = 0.0f;
        speed_out_ = 0.0f;
    }

    /*
     * update — 运行 HFI + SMO, 按 speed 权重融合输出
     *
     * @param v_alpha, v_beta — αβ 电压 (来自 InvPark)
     * @param i_alpha, i_beta — αβ 电流 (来自 Clarke)
     * @param vbus — 母线电压 (V)
     * @param dt  — 控制周期 (秒)
     * @param angle_out — 融合后电角度 (rad)
     * @param speed_out — 融合后电角速度 (rad/s)
     * @param blend_start, blend_end — 融合转速区间 (RPM)
     *
     * ★ TODO: 当前 HFI 未实现, 暂用 SMO 输出占位
     */
    void update(float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float vbus, float dt,
                float blend_start_rpm, float blend_end_rpm,
                float* angle_out, float* speed_out)
    {
        (void)vbus;
        (void)blend_start_rpm;
        (void)blend_end_rpm;

        // ★ 暂时只运行 SMO (HFI 未实现)
        smo_.update(v_alpha, v_beta, i_alpha, i_beta, dt, &angle_out_, &speed_out_);

        // ★ TODO: HFI 实现后取消注释以下代码
        // float angle_hfi, speed_hfi, angle_smo, speed_smo;
        // float w;
        //
        // // 同时运行两种观测器
        // smo_.update(v_alpha, v_beta, i_alpha, i_beta, dt, &angle_smo, &speed_smo);
        // hfi_.update(dt, &angle_hfi, &speed_hfi);
        //
        // // 根据速度计算融合权重
        // float current_speed_rpm = speed_smo * 60.0f / TWO_PI_F;  // rad/s → RPM
        // if (current_speed_rpm <= blend_start_rpm) {
        //     w = 1.0f;   // 纯 HFI
        // } else if (current_speed_rpm >= blend_end_rpm) {
        //     w = 0.0f;   // 纯 SMO
        // } else {
        //     w = 1.0f - (current_speed_rpm - blend_start_rpm) / (blend_end_rpm - blend_start_rpm);
        // }
        //
        // angle_out_ = w * angle_hfi + (1.0f - w) * angle_smo;
        // speed_out_ = w * speed_hfi + (1.0f - w) * speed_smo;

        *angle_out = angle_out_;
        *speed_out = speed_out_;
    }

private:
    SlidingModeObserver      smo_;
    HighFreqInjectionObserver hfi_;
    float angle_out_ = 0.0f;
    float speed_out_ = 0.0f;

    static constexpr float TWO_PI_F = 6.28318530717958647692f;
};

} // namespace Lib_Motor
