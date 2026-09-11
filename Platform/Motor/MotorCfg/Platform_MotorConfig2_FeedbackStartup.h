#pragma once

#include "Motor_Config.h"
#include "Platform_HALL3.h"
#include "Platform_ABZ.h"
#include "Platform_AS5600.h"

namespace Platform_MotorConfig
{

/* ===================== [13] IF 启动 profile ===================== */
/* 正式 IF 启动曲线；调试 IF 使用 TestCfg 中的独立 profile。
 * phase_count 决定实际执行的阶段；无目标启动完成交接后，速度环保持最后一个有效 Ramp 的 final_speed_rpm。
 * 因此最后一个有效 Ramp 必须不低于 smo_handover.min_speed_rpm，且不得超过 max_speed_rpm。
 */
static volatile Lib_Motor::MotorIFStartupPhase Platform_NormalIFStartupPhases[] = {
    Lib_Motor::MotorIFStartupPhase::Alignment(0.50f, 0.80f, 15.00f),
    Lib_Motor::MotorIFStartupPhase::Ramp(1.50f,  400.0f, 0.00f, 10.0f),
    Lib_Motor::MotorIFStartupPhase::Ramp(2.00f, 200.0f, 0.00f, 0.70f),
    Lib_Motor::MotorIFStartupPhase::Ramp(2.00f, 400.0f, 0.00f, 1.00f),
    Lib_Motor::MotorIFStartupPhase::Ramp(1.50f, 600.0f, 0.00f, 1.20f),
};

static Lib_Motor::MotorIFStartupProfile Platform_NormalIFStartupProfile = {
    Platform_NormalIFStartupPhases,
    2U
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
     * startup_source: 启动阶段建立电角度的来源。
     *   SENSOR -> 由编码器/HALL 等转子传感器直接启动。
     *   IF     -> 依次执行 if_startup_profile 的对齐与开环拖动阶段。
     *   HFI    -> 高频注入低速启动；当前算法尚未实现且 BuildCfg 未编译，选择后会被配置校验拒绝。
     *
     * steady_source: 启动完成后真正参与 FOC 的角度与速度来源，需 BuildCfg 已编译对应算法。
     *   SENSOR -> 始终使用外部转子传感器。
     *   SMO    -> 当前仅支持 IF 启动后，经有效性和交接判据确认，再融合到 SMO。
     *   其余无感源仍为预留项，配置校验不会允许进入运行。
     *
     * startup_auto_restart: IF profile 结束仍未完成 SMO 交接时，是否先 COAST 停机再自动重试。
     *   false -> 立即报告 OBSERVER_LOSS，不执行自动重试。
     *   true  -> 按下面的次数和等待时间重新执行完整 IF profile。
     * startup_max_retry_count: 首次启动之外允许的额外重试次数；设为 3 表示最多共尝试 4 次。
     *   startup_auto_restart=false 时该值不参与运行。
     * startup_restart_interval_s: 每次失败进入 COAST/STOP 后，到重新启动 IF 的等待时间。
     *
     * sensor_fault_action: 仅处理 steady_source=SENSOR 时的外部转子传感器故障。
     *   STOP                    -> 立即停机，当前唯一可执行的有感故障策略。
     *   SWITCH_TO_CONVERGED_SMO -> 预留；当前有感配置校验会拒绝。
     *   SWITCH_TO_HFI           -> 预留；当前有感配置校验会拒绝。
     *   此字段不处理 SMO 失锁；SMO 失锁由 smo_validity 和 enable_auto_swap 决定。
    */
    cfg.default_run_policy.startup_source = Lib_Motor::StartupSource::IF;
    cfg.default_run_policy.steady_source = Lib_Motor::SteadyAngleSource::SMO;
    cfg.default_run_policy.startup_auto_restart = false;
    cfg.default_run_policy.startup_max_retry_count = 3U;
    cfg.default_run_policy.startup_restart_interval_s = 1.0f;
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

    /* ===================== [13] IF 启动参数 ======================== */
    /* 首段负责 Id 对齐，后续段负责开环转速与 dq 电流连续爬升。 */
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
     *   hfi_to_smo_rpm:           运行期自动切换预留阈值；首次 IF/HFI→SMO 交接不使用此项。
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
     * [SMO 动态参数]
     * smo_gain: 滑模反馈增益。增大可增强电流误差校正，过小可能无法建立稳定反电势，
     *   过大则更容易进入注入限幅并放大抖振与噪声。
     * smo_pll_kp: PLL 比例增益，决定相位误差的即时修正强度。
     *   增大可加快跟踪，但会放大反电势纹波并增加观测速度抖动。
     * smo_pll_ki: PLL 积分增益，负责消除稳态相位/速度偏差。
     *   过小会留下稳态误差，过大可能引起低频振荡或积分饱和。
     * smo_bemf_lpf_cutoff_hz: 从滑模注入量提取反电势的一阶低通截止频率。
     *   降低截止频率可抑制抖振，但会增加反电势相位滞后；提高则相反。
     *
     * [SMO 首次有效判定]
     * acquire_min_speed_rpm: 观测器估算机械速度绝对值必须达到的下限；同时作为当前
     *   IF->SMO 运行中速度目标的最低允许值。STOP 下低于该值表示按完整 IF profile 无目标启动。
     * acquire_min_signal_level: ObserverSignalLevel 必须达到的反电势幅值下限；0 表示关闭幅值门槛。
     * acquire_max_pll_error_rad: ObserverPllError 绝对值允许的上限，用于判断 PLL 自身是否锁定。
     * acquire_ticks: 上述三项必须连续成立的控制周期数；任一项失败会将 ValidTicks 清零。
     *   当前 600 tick 在 12 kHz 下为 50 ms，达到后锁存 observer_converged。
     *
     * [SMO 有效状态释放]
     * release_min_speed_rpm / release_min_signal_level / release_max_pll_error_rad:
     *   observer_converged 已锁存后使用的宽松保持门槛，构成相对 acquire 条件的迟滞。
     * release_ticks: 宽松条件连续失败达到该周期数才撤销有效状态；条件恢复会清零 InvalidTicks。
     *   当前 120 tick 在 12 kHz 下为 10 ms，避免单次噪声导致立即失锁。
     *
     * [IF -> SMO 交接与融合]
     * smo_handover.min_speed_rpm: 允许交接的最低 SMO 估算机械转速，不替代 acquire 速度门槛；
     *   IF profile 最后一个有效 Ramp 的 final_speed_rpm 必须达到该值。
     * smo_handover.max_angle_error_rad: 方向和 π 分支修正后的 SMO 角度与当前 IF 控制角之间，
     *   允许的最短圆周角差；它不同于 SMO 内部的 ObserverPllError。
     * smo_handover.max_speed_error_rpm: SMO 估算速度与当前 IF 速度之间允许的最大差值。
     * smo_handover.confirm_ticks: observer_converged、同方向、速度及角度条件连续成立的确认周期数；
     *   任一交接条件失败即清零。当前 120 tick 在 12 kHz 下为 10 ms。
     * smo_handover.blend_time_s: 交接确认后，将捕获的角度偏移衰减到零，并把速度从 IF 值
     *   平滑插值到 SMO 值的持续时间；当前 0.05 s 对应约 600 tick。
     * allow_smo_closed_loop: SMO 进入闭环的安全总开关。steady_source=SMO 时必须为 true，
     *   否则在配置校验阶段拒绝启动，而不是只禁止最后一次角度切换。
     *
     * [稳态 SMO 失锁后的自动降级]
     * smo_to_hfi_rpm: SMO 已失锁且打开自动切换后，判断是否允许回退低速源的基准速度。
     * switch_hysteresis_rpm: 当前从上述基准值中减去，形成实际回退门槛；本配置为 420 rpm。
     * switch_grace_time_s: 当前仅供 HFI 运行阶段的失效宽限窗口使用；HFI 未实现且未编译时无效。
     * enable_auto_swap: false 时 SMO 撤销有效状态后直接报告 OBSERVER_LOSS；true 时仅在低于
     *   回退门槛后尝试切到已编译的低速源，当前无 HFI 时回到 IF 并重新走 smo_handover。
     * 当前真正可执行的首次交接只有 IF -> SMO；HFI -> SMO 虽复用交接入口，仍会被配置校验拒绝。
     */
    cfg.observer.smo_gain = 8.0f;
    cfg.observer.smo_pll_kp = 60.0f;
    cfg.observer.smo_pll_ki = 1200.0f;
    cfg.observer.smo_bemf_lpf_cutoff_hz = 300.0f;
    cfg.observer.smo_validity.acquire_min_speed_rpm = 300.0f;
    cfg.observer.smo_validity.acquire_min_signal_level = 5.0f;
    cfg.observer.smo_validity.acquire_max_pll_error_rad = 0.35f;
    cfg.observer.smo_validity.acquire_ticks = 600U; // 12 kHz 下 50 ms，覆盖 300 rpm 时一个电周期。
    cfg.observer.smo_validity.release_min_speed_rpm = 250.0f;
    cfg.observer.smo_validity.release_min_signal_level = 4.0f;
    cfg.observer.smo_validity.release_max_pll_error_rad = 0.60f;
    cfg.observer.smo_validity.release_ticks = 120U;

    cfg.observer.smo_handover.min_speed_rpm = 350.0f;
    cfg.observer.smo_handover.max_angle_error_rad = 0.35f;
    cfg.observer.smo_handover.max_speed_error_rpm = 80.0f;
    cfg.observer.smo_handover.confirm_ticks = 120U;
    cfg.observer.smo_handover.blend_time_s = 0.05f;
    cfg.observer.allow_smo_closed_loop = true;
    cfg.observer.smo_to_hfi_rpm        = 500.0f;
    cfg.observer.switch_hysteresis_rpm = 80.0f;
    cfg.observer.switch_grace_time_s   = 0.2f;
    cfg.observer.enable_auto_swap      = false;
}

} // namespace Platform_MotorConfig
