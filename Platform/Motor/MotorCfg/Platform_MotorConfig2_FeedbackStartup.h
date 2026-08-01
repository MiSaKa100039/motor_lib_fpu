#pragma once

#include "Motor_Config.h"
#include "Platform_HALL3.h"
#include "Platform_ABZ.h"
#include "Platform_AS5600.h"

namespace Platform_MotorConfig
{

static volatile Lib_Motor::MotorIFStartupPhase Platform_NormalIFStartupPhases[] = {
    {0.50f,   0.0f, 0.80f, 0.00f},
    {1.00f,  80.0f, 0.20f, 0.40f},
    {2.00f, 200.0f, 0.00f, 0.70f},
    {2.00f, 400.0f, 0.00f, 1.00f},
    {1.50f, 600.0f, 0.00f, 1.20f},
};

static Lib_Motor::MotorIFStartupProfile Platform_NormalIFStartupProfile = {
    Platform_NormalIFStartupPhases,
    5U
};

/* ===================== [8] 位置反馈 ===================== */
/*
 * rotor_feedback 仅描述 steady_source=SENSOR 时的有感主反馈类别。
 * 当前工程 steady_source=SMO, HFI/SMO 的静止角度、最低有效速度和切换阈值
 * 由 cfg.observer 管理, 不在这里伪装成 SENSORLESS 主传感器。
 *
 * API 能否进入转矩/速度/位置模式由 BuildCfg 与 feedback_class 统一推导,
 * 不在平台 cfg 里手填 supports_* 或静止有效性能力位。
 */

inline void ApplyFeedbackAndStartupConfig(Lib_Motor::MotorConfig& cfg)
{
    cfg.position.rotor_feedback.feedback_class = Lib_Motor::AngleFeedbackClass::SENSORLESS;

    #ifdef BSP_HALL3_ENABLE
    cfg.position.rotor_sensor = Platform_Get_HALL3();
    Platform_HALL3_SetPolePairs(cfg.physical.pole_pairs);
    #endif


    /* ===================== [9] 可选反馈链 ===================== */
    /*
     * enable_redundant_rotor_sensor: 是否启用冗余转子传感器 (双编码器切换)
     * redundant_rotor_sensor:        冗余传感器接口指针, 需 BuildCfg 开启 REDUNDANT_SENSOR
     * enable_output_sensor:          是否启用输出侧传感器 (减速后/负载侧)
     * output_sensor:                 输出侧传感器接口指针, 需 BuildCfg 开启 OUTPUT_SENSOR
     * transmission_ratio:            传动比 (输出转速/电机转速), 仅输出侧传感器时使用
     */
    cfg.position.enable_redundant_rotor_sensor = false;
    cfg.position.redundant_rotor_sensor = nullptr;
    cfg.position.enable_output_sensor = false;
    cfg.position.output_sensor = nullptr;
    cfg.position.transmission_ratio = 1.0f;

    /* ===================== [10] 传感器健康检查 ===================== */
    /*
     * enable_angle_step_check: 是否启用角度跳变检测 (异常突变 → 标记故障)
     * max_mech_angle_step_rad: 最大允许机械角度步进 (rad), 超出视为异常
     * invalid_confirm_ticks:   角度无效持续 tick 数后确认故障 (防误触发)
     * enable_redundant_check:  是否启用冗余传感器交叉校验
     * max_redundant_error_rad: 冗余传感器最大允许角度偏差 (rad)
     * redundant_confirm_ticks: 冗余偏差持续 tick 数后确认故障
     */
    cfg.position.health.enable_angle_step_check = false;
    cfg.position.health.max_mech_angle_step_rad = 0.0f;
    cfg.position.health.invalid_confirm_ticks = 1;
    cfg.position.health.enable_redundant_check = false;
    cfg.position.health.max_redundant_error_rad = 0.35f;
    cfg.position.health.redundant_confirm_ticks = 3;

    /* ===================== [11] 默认运行策略 ===================== */
    /*
     * startup_source: 启动角度源
     *   SENSOR → 有感启动 (编码器/HALL)
     *   IF     → IF 开环拖动启动
     *   HFI    → 高频注入启动
     * steady_source: 稳态运行角度源 (需 BuildCfg 已编译对应算法)
     * sensor_fault_action: 传感器故障时的处置策略
     *   STOP → 立即停机
     *   FALLBACK_IF → 切换到 IF 开环运行
    */
    cfg.default_run_policy.startup_source = Lib_Motor::StartupSource::IF;
    cfg.default_run_policy.steady_source = Lib_Motor::SteadyAngleSource::SMO;
    cfg.observer.sensor_fault_action = Lib_Motor::SensorFaultAction::STOP;

    /* ===================== [12] 顺逆风启动与静止保证 ===================== */
    /*
     * 顺逆风启动只用于非 SENSOR 启动中"反电动势足够强"的旋转接管。
     * SENSOR 有感启动可直接读角度, 不应开启相电压顺逆风启动; ConfigCheck 会拒绝。
     *
     * 顺逆风启动按 |speed_rpm| 判断是否超过相电压反电动势重构阈值;
     * 顺风/逆风共用同一流程, 方向由 PLL/观测速度符号表达。
     *
     * 高于阈值: PHASE_VOLTAGE 可先 coast 采样相电压, 重构角度后 seed SMO。
     * 低于阈值: 反电动势太弱, 不应强行重构, 需要回到 startup_source:
     *   - HFI: 低速/静止仍可辨识角度, 后续 HFI 闭环可用后应优先尝试 HFI;
     *   - IF: 无绝对角度。若大惯量且转子被外力带动, 贸然预对齐/强拖
     *         可能丢步或冲击; 这类场景优先更换为 HFI/SENSOR 启动源,
     *         或补齐相电压顺逆风启动能力后再允许启动;
     *   - SENSOR: 有感场景可直接读角度, 通常不需要 FlyingStartMode。
     *         当前 RunPlan 会直接进 SENSOR_ONLY, 不触发 ALIGNMENT 内的相电压
     *         顺逆风启动检测。
     *
     * 本工程 startup_source=IF 且顺逆风启动 DISABLED, 因此仍需明确静止保证
     * 机制 (否则 ConfigCheck 拒绝)。用户确认 start() 前不会被外力拉动,
     * 所以选 EXPLICIT_BYPASS; 若后续存在外力拖动风险, 应改为更可靠的
     * 启动源/观测器, 或选择 PHASE_VOLTAGE_FLYING/LOW_SIDE_BRAKE 并补齐硬件能力。
     */
    cfg.observer.flying_start_mode = Lib_Motor::FlyingStartMode::DISABLED;
    cfg.observer.rotor_stationary_guarantee        = Lib_Motor::RotorStationaryGuarantee::EXPLICIT_BYPASS;
    cfg.observer.rotor_explicit_bypass_confirmed   = true;

    /* [顺逆风启动速度阈值]
     * 按实测反电动势可稳定重构角度的最低机械转速填写。
     * cfg 打开 PHASE_VOLTAGE 时, start() 内 |ω| > 该值才走相电压顺逆风启动分支;
     * 低于该值必须依赖 startup_source, 不能把弱反电动势当作可靠角度源。
     */
    cfg.observer.flying_start_speed_threshold_rpm = 100.0f;

    /* ===================== [13] IF 启动参数 ===================== */
    /*
     * align_current_a:     对齐电流 (A), 初始定位阶段的 d 轴电流
     * align_time_s:        对齐时间 (s), 转子磁链对齐到 d 轴的持续时间
     * drag_current_a:      拖动电流 (A), IF 开环阶段的电流幅值
     * drag_accel_rpm_s:    拖动加速度 (RPM/s), IF 开环阶段的加速斜率
     * force_drag_timeout_s: 强制拖动超时 (s), 超时未切换则报故障
     */
    cfg.observer.align_current_a = 1.0f;
    cfg.observer.align_time_s = 0.5f;
    cfg.observer.drag_current_a = 1.5f;
    cfg.observer.drag_accel_rpm_s = 1000.0f;
    cfg.observer.force_drag_timeout_s = 3.0f;
    cfg.observer.if_startup_profile = &Platform_NormalIFStartupProfile;

    /* ===================== [14] HFI / 低速无感参数 ===================== */
    /*
     * HFI 通用参数：
     *   hfi_injection_mode:       注入模式选择，支持方波交变或正弦载波注入。
     *   hfi_injection_voltage_ratio: 注入电压占实时母线电压的比例。
     *   hfi_pll_kp / hfi_pll_ki:  角度 PLL 增益。
     *   hfi_pll_integral_limit_rad_s: PLL 积分项限幅 (rad/s)，防止积分饱和。
     *   hfi_pll_speed_limit_rad_s:    PLL 输出角速度限幅 (rad/s)，限制最大跟踪速率。
     *   hfi_prepare_speed_rpm:    低于该速度时允许提前启动/预热 HFI。
     *   hfi_disable_speed_rpm:    SMO 有效且高于该速度后允许关闭 HFI。
     *   hfi_to_smo_rpm:           低速角度源加速后交接到高速观测器的阈值；当前高速源为 SMO，后续可替换为 EKF 等。
     *   hfi_speed_predict_time_s: 低速风险预测前视时间 (s)。
     *   hfi_voltage_ramp_time_s:  注入幅值斜坡时间 (s)。
     *   hfi_min_signal_level_a:   HFI 有效所需最小解调电流信号 (A)。
     *   hfi_lock_ticks:           HFI 连续有效确认 tick 数。
     *
     * 方波交变注入：
     *   hfi_square_current_scale:    二阶差分高频电流比例系数，0.25 对应旧实现的 /4 提取器。
     *   hfi_square_lpf_cutoff_ratio: 方波解调低通滤波截止频率比例。
     *
     * 正弦载波注入：
     *   hfi_sine_injection_freq_hz: 正弦载波频率，方波模式下忽略。
     *   hfi_sine_lpf_cutoff_ratio:  解调低通截止频率相对载波频率的比例。
     */
    cfg.observer.hfi_injection_mode = Lib_Motor::HfiInjectionMode::SQUARE_ALTERNATE;
    cfg.observer.hfi_injection_voltage_ratio = 0.05f;
    cfg.observer.hfi_pll_kp = 5.0f;
    cfg.observer.hfi_pll_ki = 100.0f;
    cfg.observer.hfi_pll_integral_limit_rad_s = 5.0f;
    cfg.observer.hfi_pll_speed_limit_rad_s = 50.0f;
    cfg.observer.hfi_prepare_speed_rpm = 650.0f;
    cfg.observer.hfi_disable_speed_rpm = 800.0f;
    cfg.observer.hfi_to_smo_rpm        = 800.0f;
    cfg.observer.hfi_speed_predict_time_s = 0.02f;
    cfg.observer.hfi_voltage_ramp_time_s = 0.02f;
    cfg.observer.hfi_min_signal_level_a = 0.0f;
    cfg.observer.hfi_lock_ticks = 100;

    cfg.observer.hfi_square_current_scale = 0.25f;
    cfg.observer.hfi_square_lpf_cutoff_ratio = 0.05f;

    cfg.observer.hfi_sine_injection_freq_hz = 1000.0f;
    cfg.observer.hfi_sine_lpf_cutoff_ratio = 0.25f;

    /* ===================== [15] SMO / 无感参数 ===================== */
    /*
     * smo_gain:               滑模增益, 影响观测器收敛速度和抖振
     * smo_pll_kp:                 PLL 锁相环比例增益, 跟踪反电势角度
     * smo_pll_ki:                 PLL 锁相环积分增益
     * smo_min_signal_level:       SMO 有效所需最小反电势信号, 0 表示关闭
     * switch_speed_rpm:       无感/有感切换速度 (RPM), 高于此值切换到 SMO
     * hysteresis_rpm:         切换滞环 (RPM), 防止在切换点反复跳变
     * max_handover_error_rad: 切换时最大允许角度误差 (rad), 超出则延迟切换
     * convergence_ticks:      SMO 收敛所需 tick 数, 收敛前不切换
     * smo_to_hfi_rpm:         SMO 低速有效性不足时主动降级到低速角度源的阈值；当前低速源为 HFI/IF 组合策略。
     * switch_hysteresis_rpm:  观测器切换阈值迟滞 (RPM), 防边界抖动。
     * switch_grace_time_s:    切回失败的超时窗口 (s), 超时置 Fault::OBSERVER_LOSS。
     * enable_auto_swap:       关闭时维持 valid=false 即 Fault 老语义；打开后先尝试预防性降级。
     */
    cfg.observer.smo_gain = 8.0f;
    cfg.observer.smo_pll_kp = 80.0f;
    cfg.observer.smo_pll_ki = 1200.0f;
    cfg.observer.smo_min_signal_level = 0.0f;
    /* [P4] 启动期 IF→SMO 单向加速切换 (旧 switch_speed_rpm/hysteresis_rpm)
     * 仅 handleForceDrag 消费; 稳态加速交接阈值使用 HFI 段的 hfi_to_smo_rpm。 */
    cfg.observer.hfi_to_smo_startup_rpm            = 500.0f;
    cfg.observer.hfi_to_smo_startup_hysteresis_rpm = 50.0f;
    cfg.observer.max_handover_error_rad = 0.35f;
    cfg.observer.convergence_ticks = 100;
    cfg.observer.allow_smo_closed_loop = false;
    cfg.observer.smo_to_hfi_rpm        = 500.0f;
    cfg.observer.switch_hysteresis_rpm = 80.0f;
    cfg.observer.switch_grace_time_s   = 0.2f;
    cfg.observer.enable_auto_swap      = false;
}

} // namespace Platform_MotorConfig
