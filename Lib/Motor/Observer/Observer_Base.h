#pragma once

#include "../Public/Motor_Config.h"
#include <cstdint>

namespace Lib_Motor
{

// 观测器统一输出结构 (HFI/SMO/Sensor 共用)
struct ObserverEstimate
{
    float angle_rad = 0.0f;          // 估计电角度 (rad), 归一化到 [0, 2π)
    float speed_rad_s = 0.0f;        // 估计电角速度 (rad/s)
    float pll_error_rad = 0.0f;      // PLL 角度误差信号 (解调后经 LPF)
    float signal_level = 0.0f;       // 解调信号强度 (用于有效性/质量判断)
    // --- 以下为方波 HFI 专用调试量, 其他观测器保持 0 ---
    float hfi_demod_alpha = 0.0f;    // 同步整流后解调 α 分量
    float hfi_demod_beta = 0.0f;     // 同步整流后解调 β 分量
    float hfi_demod_d = 0.0f;        // 解调结果投影到估计 d 轴
    float hfi_demod_q = 0.0f;        // 解调结果投影到估计 q 轴 (误差主要来源)
    float hfi_raw_demod_alpha = 0.0f;// 未归一化解调 α
    float hfi_raw_demod_beta = 0.0f; // 未归一化解调 β
    float hfi_meas_2theta = 0.0f;    // atan2(-q, d) 双角测量值, 双角 PLL 用
    // --- 通用 ---
    float quality = 0.0f;            // 信号质量分 [0,1], = signal/min_signal_level
    uint16_t valid_ticks = 0U;       // 当前连续有效 tick 计数
    bool valid = false;              // valid_ticks >= lock_ticks_ 时认定为有效
};

// fixed-q15 后端的观测器内部输出；转换到 float 只允许在 monitor/API 读取点发生。
struct ObserverEstimateQ15
{
    uint32_t angle_phase = 0U;        // 电角度 phase_u32, 0..2^32 映射 0..2π
    int16_t speed_rpm_q15 = 0;        // 机械转速 / speed_base_rpm
    int16_t pll_error_q15 = 0;        // PLL 角度误差, ±32768 映射 ±π
    int16_t signal_level_q15 = 0;     // 反电势信号 / voltage_base
    int16_t quality_q15 = 0;          // 质量分 [0,1]
    uint16_t valid_ticks = 0U;
    bool valid = false;
};

// HFI 注入电压指令 (叠加到 FOC d/q 轴电压上)
struct HfiInjectionCommand
{
    float v_d = 0.0f;        // d 轴注入电压 (V): 方波=±Vhf, 正弦=Vhf·sin(ωt)
    float v_q = 0.0f;        // q 轴注入电压 (V), HFI 只注入 d 轴故恒为 0
    bool enabled = false;    // 注入是否生效 (仅当注入电压>0 为 true)
};

// 观测器虚函数表 (C 风格多态接口, 供未来切换/融合统一调度)
struct ObserverVTable
{
    void (*init)(void* self, const MotorConfig* cfg);   // 初始化: 传入电机配置
    void (*update)(void* self, float v_alpha, float v_beta,
                   float i_alpha, float i_beta,
                   float vbus, float dt,
                   ObserverEstimate* estimate_out);     // 每周期更新: 电压/电流/母线/dt → 估计输出
    void (*reset)(void* self);                          // 状态复位
};

} // namespace Lib_Motor
