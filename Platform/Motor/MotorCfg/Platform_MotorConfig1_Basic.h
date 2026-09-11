#pragma once

#include "Motor_Config.h"

namespace Platform_MotorConfig
{

inline void ApplyBasicConfig(Lib_Motor::MotorConfig& cfg)
{
    /* ===================== [1] ADC 采样基准 ===================== */
    /*
     * adc_v_ref:           ADC 参考电压 (V), 需实测 VDDA 电压而非直接写 3.30
     * adc_resolution:      ADC 量化级数 (12-bit = 4096)
     * adc_calibration_samples: 上电自校准采样次数, 用于计算 ADC 零漂
     * auto_calib_interval_s:   周期性自动校准间隔 (s),
     *                          设为 0 则仅上电校准一次
     */
    cfg.sensor.adc_v_ref = 5.0f;
    cfg.sensor.adc_resolution = 4096.0f;
    cfg.sensor.adc_calibration_samples = 1000;
    cfg.sensor.auto_calib_interval_s = 30.0f;

    /* ===================== [2] 采样低通滤波 ===================== */
    /*
     * 一阶低通滤波器时间常数 (s), 滤除 PWM 开关噪声。
     * 不能设太大, 否则相位滞后导致电流环失稳。
     *   phase_current: 相电流 float 监控/反馈路径的一阶低通; 当前 50us 为轻滤波。
     *   bus_current/bus_voltage: 分压器+运放 → 噪声大, 常数稍大
     */
    cfg.sensor.phase_current_lpf_tf_s = 0.00005f;
    cfg.sensor.bus_current_lpf_tf_s = 0.02f;
    cfg.sensor.phase_voltage_lpf_tf_s = 0.02f;
    cfg.sensor.bus_voltage_lpf_tf_s = 0.02f;

    /* ===================== [3] 相电流采样 ===================== */
    /*
     * current_sensor_type: SHUNT_RESISTOR (分流电阻)
     *                      HALL_SENSOR (霍尔电流传感器)
     * current_sense_mode:   DUAL_SENSOR → 两个相电流传感器 (Iu, Iv), Iw 计算
     *                       TRIPLE_SENSOR → 三个相电流传感器
     * has_bus_current:      是否有独立母线电流采样电阻
     *
     * invert_phase_current_{u,v,w}: 运放极性是否反相所致符号翻转,
     *                               可通过真值判断: U 相电流流入电机时,
     *                               采样值为正才不需 invert
     *
     * hall_sensitivity_mV_per_A: 霍尔灵敏度 (mV/A), 零点由上电 ADC offset 校准
     *
     * phase_shunt_resistor/phase_amp_gain: 仅 SHUNT_RESISTOR 模式使用
     * bus_shunt_resistor/bus_amp_gain: 仅 has_bus_current=true 时使用
    */
    cfg.sensor.current_sensor_type = Lib_Motor::CurrentSensorType::SHUNT_RESISTOR;
    cfg.sensor.current_sense_mode = Lib_Motor::CurrentSenseMode::DUAL_SENSOR;
    cfg.sensor.has_bus_current = true;

    cfg.sensor.invert_phase_current_u = true;
    cfg.sensor.invert_phase_current_v = true;
    cfg.sensor.invert_phase_current_w = true;

    cfg.sensor.hall_sensitivity_mV_per_A = 4.4f;

    cfg.sensor.phase_shunt_resistor = 0.001f;
    cfg.sensor.phase_amp_gain = 20.0f;

    // bus_shunt_* 用于 OPA2/ADC14 母线电流采样和母线软件过流判断。
    cfg.sensor.bus_shunt_resistor = 0.0001f;
    cfg.sensor.bus_amp_gain = 20.0f;

    /* ===================== [4] 母线电压采样 ===================== */
    /*
     * 分压比: V_adc = VBUS × R_down / (R_up + R_down)
     * has_bus_voltage: false 时用额定电压代替, 影响调制比和过压保护
     */
    cfg.sensor.has_bus_voltage = true;

    cfg.sensor.vbus_r_up = 220000.0f;
    cfg.sensor.vbus_r_down = 10000.0f;

    cfg.sensor.has_phase_voltage = false;
    cfg.sensor.vphase_r_up = 300000.0f;
    cfg.sensor.vphase_r_down = 10000.0f;

    /* ===================== [5] NTC 温度采样 ===================== */
    /*
     * NTC 通道数量由 MOTOR_BUILD_NTC_SLOTS 决定。
     * ntc[index]:
     *   enabled:     是否使能该路
     *   topology:     LOW_SIDE_NTC → NTC 下端接 GND, 上端串联分压到 VDDA
     *                 HIGH_SIDE_NTC → NTC 上端接 VDDA
     *   r_series:     串联分压电阻 (Ω)
     *   r_25:         25°C 时 NTC 标称阻值 (Ω)
     *   beta:         NTC 热敏指数 (B 值, K)
     *   over_temp_c:  过热保护阈值 (°C)
     */
    cfg.sensor.ntc[0].enabled = true;
    cfg.sensor.ntc[0].topology = Lib_Motor::NtcTopology::LOW_SIDE_NTC;
    cfg.sensor.ntc[0].r_series = 10000.0f;
    cfg.sensor.ntc[0].r_25 = 10000.0f;
    cfg.sensor.ntc[0].beta = 3380.0f;
    cfg.sensor.ntc[0].over_temp_c = 85.0f;


    // cfg.sensor.ntc[1].enabled = true;
    // cfg.sensor.ntc[1].topology = Lib_Motor::NtcTopology::LOW_SIDE_NTC;
    // cfg.sensor.ntc[1].r_series = 100000.0f;
    // cfg.sensor.ntc[1].r_25 = 10000.0f;
    // cfg.sensor.ntc[1].beta = 3950.0f;
    // cfg.sensor.ntc[1].over_temp_c = 85.0f;


    /* ===================== [6] 电机物理参数 ===================== */
    /*
     * pole_pairs:    极对数, angle_elec = angle_mech × pole_pairs
     * rated_current: 电机连续额定相电流 (A), 不是调试限流值
     *                优先级说明:
     *                - Iq 软上限取自 configuredIqLimit() 三级回退:
     *                  (1) cfg.motion.max_iq_ref_a > 0 → 优先用 max_iq_ref_a
     *                  (2) max_iq_ref_a = 0 时回退用 rated_current
     *                  (3) rated_current = 0 时回退用 limit.max_phase_current_a (相线软件保护)
     *                - 本工程在运动软限幅配置中填写 max_iq_ref_a, rated_current
     *                  仅在该值为 0 时作为兜底, 保留字段不删除。
     * rated_voltage: 电机额定电压 (V), 不是实时母线电压
     *                优先级说明:
     *                - 仅在 RL 辨识时 vbus 采样=0 兜底用 (Motor_RLIdentifyRoutine.cpp);
     *                - 日常运行用 ctx.v_bus 实时采样, 不读 rated_voltage;
     *                - 保留字段作为辨识场景的安全兜底。
     * rs/ls:         单相定子电阻 (Ω) / 等效电感 (H), SMO/Kalman 等无感算法使用
     *                有感 FOC 运行时 rs/ls 不决定角度来源, 但影响电流环前馈和 PI 整定
     * ld/lq:         强凸极/高性能电机可手动填写 d/q 轴电感; 普通场景保持 0,
     *                电流环推导会按 Ld/Lq 无效时回退到 Ls。
     *                若 LCR 多个转子位置测得 L 差异明显, 通用 Ls 建议填较小值作为保守整定;
     *                需要更高性能时再手动区分 Ld/Lq。本轮自动 RL 不做转子角度扫描。
     */
    cfg.physical.pole_pairs = 4;
    cfg.physical.rated_current = 30.0f;
    cfg.physical.rated_voltage = 24.0f;
    /* === 电机参数固化配置值 (=0 表示未知) ===
     * *_measured:
     *   仪器、datasheet、反拖台架等外部方式确认后的手动测量固化值。
     *
     * *_learned_cfg:
     *   上次调用 startRLIdentification() 或 Ke 辨识得到 *_identified 后,
     *   用户人工确认并回填到 cfg 的自动辨识固化值。它与 measured 分开保存,
     *   方便看清当前工程到底在用"仪器测量值"还是"自动辨识回填值"。
     *
     * electrical_param_source:
     *   AUTO:        运行期 identified > measured > learned_cfg > 0, 保持调试期自动生效语义。
     *                未接入 flash 时, 冷启动 identified=0; 若已人工回填 learned_cfg 且 measured=0,
     *                AUTO 会使用 learned_cfg。若 measured 与 learned_cfg 都有效, AUTO 优先 measured。
     *   MEASURED:    只使用 measured, 为 0 时 effective 也为 0。
     *   LEARNED_CFG: 只使用 learned_cfg, 为 0 时 effective 也为 0; 可用于强制跳过 measured。
     */
    cfg.physical.rs_ohm_measured         = 0.0275f;
    cfg.physical.ls_h_measured           = 0.000038f;
    cfg.physical.ld_h_measured           = 0.0f; // D 轴电感手动测量值, 普通场景保持 0 并使用 Ls
    cfg.physical.lq_h_measured           = 0.0f; // Q 轴电感手动测量值, 普通场景保持 0 并使用 Ls
    cfg.physical.ke_v_per_rad_s_measured = 0.0f; // Ke 可由反拖台架或 Ke 辨识确认后回填

    cfg.physical.rs_ohm_learned_cfg         = 0.0275f; // 单位 Ω
    cfg.physical.ls_h_learned_cfg           = 0.000038f;      // 单位 H
    cfg.physical.ke_v_per_rad_s_learned_cfg = 0.0f;
    cfg.physical.electrical_param_source = Lib_Motor::MotorPhysicalParamSource::MEASURED;   //仅使用仪器测量值

    /* ===================== [7] RL 辨识约束参数 ===================== */
    /*
     * 这些参数只约束 startRLIdentification() 的静态 R/L 辨识流程,
     * 不代表电机最终参数。辨识成功后仍需读取 *_identified, 确认后人工回填到
     * 上面的 *_learned_cfg 或 *_measured 字段, 下次冷启动才作为固化值使用。
     *
     * RL 辨识仍然是电压注入: 用户给目标电流和最大注入电压, 库内部从很小的 Vstep
     * 开始试探, 按测得电流自动放大, 达到目标区间后锁定 Vstep 再采样 R/L。
     * 斜坡、L 采样窗口和 R 稳态平均窗口由库内部按 control_freq_hz 换算为 tick:
     *   L 使用固定 10ms 物理窗口, 用于保持与旧版辨识口径接近;
     *   R 先等待电流稳定, 再执行固定平均窗口。
     * 用户不需要根据 ADC 采样频率或电机参数手动填写窗口时间。
     * 限流电源首次接新电机时, 建议保持 0.5A 目标电流和 0.2V 电压上限;
     * 若电流信号不足再逐步提高电压上限, 不要直接按母线电压比例放大。
     *
     * 默认值含义:
     *   0.5A 目标电流 / 0.2V 最大注入 / 3 轮重复采样平均。
     *   搜索序列约为 0.01V -> 0.015V -> ... -> 0.2V; 每次试探都先走斜坡。
     *   rl_repeat_count=3 时会在锁定 Vstep 后重复 3 轮采样并取平均, 耗时约按轮数线性增加。
     *   RL 辨识不需要 VF profile、速度爬坡或 IF 拖动参数, 只依赖静态开环电压注入和相电流采样。
     *   自动 RL 只输出标量 Ls; 多数场景按 Ld=Lq=Ls 足够, 强凸极/高性能场景再手填 ld/lq。
     *
     * Ozone 调试:
     *   若 fault=PARAM_ERROR 且 detail=RL_IDENTIFY_FAILED, 先展开 rl_identify_routine_.debug_snapshot_。
     *   debug_snapshot_ 会显示自动推导出的 ramp_ticks/l_sample_ticks/r_settle_ticks/r_average_ticks。
     *   quality_warning_flags 仅表示辨识质量提示, 不等同于硬故障; 建议扫多档电压, 取 R/L 稳定平台。
     *   last_fail_reason=SIGNAL_TOO_LOW_AT_VOLTAGE_LIMIT: 0.2V 下信号不足, 优先查相电流采样/缩放, 再逐步加电压上限。
     *   若母线电流明显变化但 measured_abs_id_a 仍为 0, 优先查 RL 电流投影/相电流采样链路, 不要继续提高电压上限。
     *   last_fail_reason=L_DI_TOO_SMALL: 自动 L 窗口内电流变化太小, 优先提高电压上限或检查采样链路。
     *   last_fail_reason=CURRENT_NAN: 优先查 ADC 读数、电流换算和滤波初始化。
     *   last_fail_reason=OVERCURRENT: 降低目标电流或电压上限。
     */
    cfg.identify.rl_target_current_a = 1.0f;           // 目标辨识电流 (A), 内部自动搜索注入电压
    cfg.identify.rl_injection_voltage_limit_v = 0.8f;  // 注入电压幅值上限 (V), 低阻电机先用低值
    cfg.identify.rl_repeat_count = 5;                  // 锁定 Vstep 后重复采样轮数, 多轮平均提高稳定性

    /* 参数持久化策略 (本工程默认不使用 flash)
     * persistent_enabled: 是否启用 learned 参数持久化框架。
     * persistent_load_on_init: init 时是否从 flash 载入 learned 参数。
     * persistent_flash_addr: flash 记录起始地址, 需由平台避开程序区。
     * persistent_flash_signature: 判断 flash 记录是否属于当前电机/配置。
     *
     * 当前 BSP flash_* 回调为 nullptr, 因此保持 false/0, 不做自动保存/加载。
     * 若没有仪器测量, 调试时用 startRLIdentification() 辨识, 再把确认值人工回填到上面的
     * *_learned_cfg 字段, 并把 electrical_param_source 设为 AUTO 或 LEARNED_CFG。
     */
    cfg.persistent_enabled            = false;
    cfg.persistent_load_on_init       = false;
    cfg.persistent_flash_addr         = 0;
    cfg.persistent_flash_signature    = 0;

    /* ===================== [8] 硬保护限值 ===================== */
    /*
     * 以下是驱动运行保护值, 不应直接照抄电机额定值。
     * max_phase_current_a: 相线软件过流立即锁故障, 首次验证采样方向时须显著降低
     * max_bus_current_a:   母线软件过流立即锁故障, 独立于相线阈值
     * max_speed_rpm:   控制目标 / 保护的机械速度上限
     * over_voltage_v:  母线过压阈值, 高于正常母线但低于功率器件耐压
     * under_voltage_v: 初始化采样完成后母线低于此值立即锁欠压故障
     * max_duty_cycle:  调制比上限, 留出 ADC 采样和死区余量
     */
    cfg.limit.max_phase_current_a = 100.0f;
    cfg.limit.max_bus_current_a = 100.0f;
    cfg.limit.max_speed_rpm = 3000.0f;
    cfg.limit.over_voltage_v = 24.0f;
    cfg.limit.under_voltage_v = 8.0f;
    cfg.limit.max_duty_cycle = 0.90f;
}

} // namespace Platform_MotorConfig
