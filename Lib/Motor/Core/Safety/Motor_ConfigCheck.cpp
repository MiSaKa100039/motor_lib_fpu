/*
 * Motor_ConfigCheck.cpp -- 安全检测: 配置检查
 *
 * 文件层级: Safety/ConfigCheck
 *
 * 职责: 上电前验证 MotorConfig 参数完整性, 防止硬件/参数配置错误
 *   1. validateBuildConfig: BuildCfg 宏合法性检查 (采样模式/NTC/占位能力)
 *   2. validateBase: 基础硬件配置检查 (HAL 接口/极对数/控制频率/传感器/LPF)
 *   3. validateFeedback: 反馈路径配置检查 (观测器/传感器/冗余参数)
 *   4. validateStartupTargetMode: 启动目标模式检查 (速度环/力矩环/位置环)
 *   5. validateInitConfig: 总入口, 依次调用上述全部校验
 *   6. isSteadySourceCompiled: 检查稳态角度源是否已编译
 *   7. validSensor: 检查传感器接口有效性
 */

#include "Motor_ConfigCheck.h"
#include "../Feature/Motor_FeatureRegistry.h"
#include "BuildCfg/Platform_MotorBuildConfig.h"

#include <cmath>

namespace Lib_Motor
{

namespace
{

Fault configPass(MotorConfigFaultDetail* detail)
{
    if (detail != nullptr)
    {
        *detail = MotorConfigFaultDetail::NONE;
    }
    return Fault::NONE;
}

Fault configFail(MotorConfigFaultDetail* detail,
                 Fault fault,
                 MotorConfigFaultDetail fault_detail)
{
    if (detail != nullptr)
    {
        *detail = fault_detail;
    }
    return fault;
}

bool validHfiParams(const MotorObserverParam& observer)
{
    const bool common_valid =
        std::isfinite(observer.hfi_injection_voltage_ratio) &&
        std::isfinite(observer.hfi_pll_kp) &&
        std::isfinite(observer.hfi_pll_ki) &&
        std::isfinite(observer.hfi_pll_integral_limit_rad_s) &&
        std::isfinite(observer.hfi_pll_speed_limit_rad_s) &&
        std::isfinite(observer.hfi_prepare_speed_rpm) &&
        std::isfinite(observer.hfi_disable_speed_rpm) &&
        std::isfinite(observer.hfi_speed_predict_time_s) &&
        std::isfinite(observer.hfi_voltage_ramp_time_s) &&
        std::isfinite(observer.hfi_min_signal_level_a) &&
        observer.hfi_injection_voltage_ratio > 0.0f &&
        observer.hfi_injection_voltage_ratio <= 1.0f &&
        observer.hfi_pll_kp >= 0.0f &&
        observer.hfi_pll_ki >= 0.0f &&
        observer.hfi_pll_integral_limit_rad_s >= 0.0f &&
        observer.hfi_pll_speed_limit_rad_s >= 0.0f &&
        observer.hfi_prepare_speed_rpm >= 0.0f &&
        observer.hfi_disable_speed_rpm > observer.hfi_prepare_speed_rpm &&
        observer.hfi_speed_predict_time_s >= 0.0f &&
        observer.hfi_voltage_ramp_time_s >= 0.0f &&
        observer.hfi_min_signal_level_a >= 0.0f &&
        observer.hfi_lock_ticks > 0U;

    if (!common_valid)
    {
        return false;
    }

    switch (observer.hfi_injection_mode)
    {
        case HfiInjectionMode::SQUARE_ALTERNATE:
            return std::isfinite(observer.hfi_square_current_scale) &&
                   std::isfinite(observer.hfi_square_lpf_cutoff_ratio) &&
                   observer.hfi_square_current_scale > 0.0f &&
                   observer.hfi_square_lpf_cutoff_ratio > 0.0f;

        case HfiInjectionMode::SINE_CARRIER:
            return std::isfinite(observer.hfi_sine_injection_freq_hz) &&
                   std::isfinite(observer.hfi_sine_lpf_cutoff_ratio) &&
                   observer.hfi_sine_injection_freq_hz > 0.0f &&
                   observer.hfi_sine_lpf_cutoff_ratio > 0.0f;

        default:
            return false;
    }
}

bool validIFStartupProfile(const MotorIFStartupProfile* profile)
{
    if (profile == nullptr)
    {
        return true;
    }
    if (profile->phases == nullptr || profile->phase_count == 0U)
    {
        return false;
    }

    const uint8_t phase_count = profile->phase_count;
    for (uint8_t i = 0U; i < phase_count; ++i)
    {
        const volatile MotorIFStartupPhase& phase = profile->phases[i];
        if (!std::isfinite(phase.duration_s) ||
            !std::isfinite(phase.final_speed_rpm) ||
            !std::isfinite(phase.final_id_a) ||
            !std::isfinite(phase.final_iq_a) ||
            phase.duration_s < 0.0f)
        {
            return false;
        }
    }

    return true;
}

bool validPhysicalParamSource(MotorPhysicalParamSource source)
{
    switch (source)
    {
        case MotorPhysicalParamSource::AUTO:
        case MotorPhysicalParamSource::MEASURED:
        case MotorPhysicalParamSource::LEARNED_CFG:
            return true;
        default:
            return false;
    }
}

bool validPhysicalElectricalParams(const MotorPhysicalParam& physical)
{
    return validPhysicalParamSource(physical.electrical_param_source) &&
           std::isfinite(physical.rs_ohm_measured) &&
           std::isfinite(physical.ls_h_measured) &&
           std::isfinite(physical.ld_h_measured) &&
           std::isfinite(physical.lq_h_measured) &&
           std::isfinite(physical.ke_v_per_rad_s_measured) &&
           std::isfinite(physical.rs_ohm_learned_cfg) &&
           std::isfinite(physical.ls_h_learned_cfg) &&
           std::isfinite(physical.ke_v_per_rad_s_learned_cfg) &&
           std::isfinite(physical.rs_ohm_identified) &&
           std::isfinite(physical.ls_h_identified) &&
           std::isfinite(physical.ke_v_per_rad_s_identified) &&
           physical.rs_ohm_measured >= 0.0f &&
           physical.ls_h_measured >= 0.0f &&
           physical.ld_h_measured >= 0.0f &&
           physical.lq_h_measured >= 0.0f &&
           physical.ke_v_per_rad_s_measured >= 0.0f &&
           physical.rs_ohm_learned_cfg >= 0.0f &&
           physical.ls_h_learned_cfg >= 0.0f &&
           physical.ke_v_per_rad_s_learned_cfg >= 0.0f &&
           physical.rs_ohm_identified >= 0.0f &&
           physical.ls_h_identified >= 0.0f &&
           physical.ke_v_per_rad_s_identified >= 0.0f;
}

}

Fault MotorConfigCheck::validateBuildConfig(MotorConfigFaultDetail* detail)
{
    if (MOTOR_BUILD_CURRENT_SENSE_MODE != 1 &&
        MOTOR_BUILD_CURRENT_SENSE_MODE != 2 &&
        MOTOR_BUILD_CURRENT_SENSE_MODE != 3)
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::BUILD_CURRENT_SENSE_MODE_INVALID);
    }

    if (MOTOR_BUILD_NTC_SLOTS < 0 || MOTOR_BUILD_NTC_SLOTS > 4)
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::BUILD_NTC_SLOTS_INVALID);
    }

#if defined(MOTOR_BUILD_ENABLE_TICK_PROFILING) && \
    (!defined(__CORTEX_M) || (__CORTEX_M == 0x00U))
    return configFail(detail,
                      Fault::PARAM_ERROR,
                      MotorConfigFaultDetail::BUILD_TICK_PROFILING_UNSUPPORTED);
#endif

#if defined(MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR) && !defined(MOTOR_BUILD_ENABLE_SENSOR)
    /*
     * 冗余传感器比较建立在主转子传感器之上, 不能脱离 SENSOR 主链单独存在。
     */
    return configFail(detail,
                      Fault::PARAM_ERROR,
                      MotorConfigFaultDetail::BUILD_REDUNDANT_REQUIRES_SENSOR);
#endif

#if defined(MOTOR_BUILD_ENABLE_OUTPUT_SENSOR) && !defined(MOTOR_BUILD_ENABLE_SENSOR)
    /*
     * 输出轴传感器当前用于主反馈链的附加监测, 不能脱离 SENSOR 主链单独存在。
     */
return configFail(detail,
                      Fault::PARAM_ERROR,
                      MotorConfigFaultDetail::BUILD_OUTPUT_REQUIRES_SENSOR);
#endif

    /* === [本轮新增] 控制拓扑: FOC / TRAP 至少一个开关 === */
#if !defined(MOTOR_BUILD_ENABLE_FOC_CONTROL) && !defined(MOTOR_BUILD_ENABLE_TRAP_CONTROL)
    return configFail(detail,
                      Fault::PARAM_ERROR,
                      MotorConfigFaultDetail::CONTROL_TOPOLOGY_NONE_ENABLED);
#endif

    return configPass(detail);
}

/*
 * isSteadySourceCompiled -- 检查稳态角度源是否已编译进固件
 * 返回值: true=对应宏已启用, false=未编译/不可用
 */
bool MotorConfigCheck::isSteadySourceCompiled(SteadyAngleSource source)
{
    return MotorFeatureRegistry::supportsSteadySource(source);
}

/*
 * validateBase -- 基础硬件配置验证 (MotorAPI::init 时调用)
 *
 * 检查项:
 *   - hal 接口非空
 *   - pole_pairs > 0
 *   - control_freq_hz > 0 且为有限值
 *   - current_sense_mode 为 DUAL_SENSOR 或 TRIPLE_SENSOR
 *   - phase_current_valid_mask 合法 (DUAL: 2 通道有效, TRIPLE: 3 通道全有效)
 *   - LPF 时间常数为有限非负数
 *   - 母线电压采集: HAL 接口和分压电阻参数有效
 *   - 相电压采集: HAL 接口和分压电阻参数有效
 *
 * 返回 Fault::NONE 或 Fault::PARAM_ERROR
 */
Fault MotorConfigCheck::validateBase(const MotorConfig& cfg,
                                     MotorConfigFaultDetail* detail)
{
    const Fault build_fault = validateBuildConfig(detail);
    if (build_fault != Fault::NONE)
    {
        return build_fault;
    }

    if (cfg.hal == nullptr)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::HAL_NULL);
    }
    if (cfg.physical.pole_pairs == 0U)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::POLE_PAIRS_INVALID);
    }
    if (!validPhysicalElectricalParams(cfg.physical))
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::PHYSICAL_PARAM_INVALID);
    }
    if (!std::isfinite(cfg.control.control_freq_hz) || cfg.control.control_freq_hz <= 0.0f)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CONTROL_FREQ_INVALID);
    }

    if (cfg.sensor.current_sense_mode != CurrentSenseMode::SINGLE_SHUNT &&
        cfg.sensor.current_sense_mode != CurrentSenseMode::DUAL_SENSOR &&
        cfg.sensor.current_sense_mode != CurrentSenseMode::TRIPLE_SENSOR)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CURRENT_SENSE_MODE_INVALID);
    }

    if (static_cast<int>(cfg.sensor.current_sense_mode) != MOTOR_BUILD_CURRENT_SENSE_MODE)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CURRENT_SENSE_MODE_BUILD_MISMATCH);
    }

    if (!std::isfinite(cfg.sensor.adc_v_ref) ||
        !std::isfinite(cfg.sensor.adc_resolution) ||
        cfg.sensor.adc_v_ref <= 0.0f ||
        cfg.sensor.adc_resolution <= 0.0f)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CURRENT_SENSOR_CONFIG_INVALID);
    }

    if (cfg.sensor.current_sense_mode == CurrentSenseMode::SINGLE_SHUNT &&
        cfg.sensor.current_sensor_type != CurrentSensorType::SHUNT_RESISTOR)
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::SINGLE_SHUNT_REQUIRES_SHUNT_SENSOR);
    }

    if (cfg.sensor.current_sensor_type == CurrentSensorType::SHUNT_RESISTOR)
    {
        if (!std::isfinite(cfg.sensor.phase_shunt_resistor) ||
            !std::isfinite(cfg.sensor.phase_amp_gain) ||
            cfg.sensor.phase_shunt_resistor <= 0.0f ||
            cfg.sensor.phase_amp_gain <= 0.0f)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CURRENT_SENSOR_CONFIG_INVALID);
        }
    }
    else if (cfg.sensor.current_sensor_type == CurrentSensorType::HALL_SENSOR)
    {
        if (!std::isfinite(cfg.sensor.hall_sensitivity_mV_per_A) ||
            cfg.sensor.hall_sensitivity_mV_per_A <= 0.0f)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CURRENT_SENSOR_CONFIG_INVALID);
        }
    }
    else
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CURRENT_SENSOR_CONFIG_INVALID);
    }

    if (cfg.sensor.has_bus_current)
    {
        if (!std::isfinite(cfg.sensor.bus_shunt_resistor) ||
            !std::isfinite(cfg.sensor.bus_amp_gain) ||
            cfg.sensor.bus_shunt_resistor <= 0.0f ||
            cfg.sensor.bus_amp_gain <= 0.0f)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::CURRENT_SENSOR_CONFIG_INVALID);
        }
    }

    if (cfg.sensor.has_bus_current && !Platform_MotorBuildConfig::supportsBusCurrent())
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BUS_CURRENT_NOT_BUILT);
    }

    if (cfg.sensor.has_phase_voltage && !Platform_MotorBuildConfig::supportsPhaseVoltage())
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::PHASE_VOLTAGE_NOT_BUILT);
    }

    const uint8_t phase_mask = cfg.hal->phase_current_valid_mask;
    if ((phase_mask & static_cast<uint8_t>(~MOTOR_PHASE_CURRENT_ALL_VALID)) != 0U)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::PHASE_CURRENT_MASK_INVALID);
    }

    const bool dual_mask_valid =
        phase_mask == (MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_V_VALID) ||
        phase_mask == (MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_W_VALID) ||
        phase_mask == (MOTOR_PHASE_CURRENT_V_VALID | MOTOR_PHASE_CURRENT_W_VALID);

    if (cfg.sensor.current_sense_mode == CurrentSenseMode::SINGLE_SHUNT)
    {
        if (phase_mask != 0U)
        {
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::PHASE_CURRENT_SINGLE_SHUNT_MASK_INVALID);
        }
        if (cfg.hal->read_single_shunt_pair_raw == nullptr ||
            cfg.hal->apply_current_sample_schedule == nullptr)
        {
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::SINGLE_SHUNT_HAL_INVALID);
        }
        if (!std::isfinite(cfg.sensor.single_shunt_min_sample_window_s) ||
            cfg.sensor.single_shunt_min_sample_window_s <= 0.0f ||
            cfg.sensor.single_shunt_min_sample_window_s *
                cfg.control.control_freq_hz >= 0.5f)
        {
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::SINGLE_SHUNT_WINDOW_INVALID);
        }
        if (cfg.control.modulation != ModulationMethod::SVPWM)
        {
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::SINGLE_SHUNT_REQUIRES_SVPWM);
        }
    }

    if (cfg.sensor.current_sense_mode == CurrentSenseMode::DUAL_SENSOR && !dual_mask_valid)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::PHASE_CURRENT_DUAL_MASK_INVALID);
    }

    if (cfg.sensor.current_sense_mode == CurrentSenseMode::TRIPLE_SENSOR &&
        phase_mask != MOTOR_PHASE_CURRENT_ALL_VALID)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::PHASE_CURRENT_TRIPLE_MASK_INVALID);
    }

    if (!std::isfinite(cfg.sensor.phase_current_lpf_tf_s) ||
        !std::isfinite(cfg.sensor.bus_current_lpf_tf_s) ||
        !std::isfinite(cfg.sensor.bus_voltage_lpf_tf_s) ||
        !std::isfinite(cfg.sensor.phase_voltage_lpf_tf_s))
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::SENSOR_LPF_INVALID);
    }

    if (cfg.sensor.phase_current_lpf_tf_s < 0.0f ||
        cfg.sensor.bus_current_lpf_tf_s < 0.0f ||
        cfg.sensor.bus_voltage_lpf_tf_s < 0.0f ||
        cfg.sensor.phase_voltage_lpf_tf_s < 0.0f)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::SENSOR_LPF_NEGATIVE);
    }

    if (cfg.sensor.has_bus_voltage)
    {
        if (cfg.hal->read_vbus_raw == nullptr ||
            !std::isfinite(cfg.sensor.vbus_r_up) ||
            !std::isfinite(cfg.sensor.vbus_r_down) ||
            cfg.sensor.vbus_r_up <= 0.0f ||
            cfg.sensor.vbus_r_down <= 0.0f)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::VBUS_CONFIG_INVALID);
        }
    }

    if (cfg.sensor.has_phase_voltage)
    {
        if (cfg.hal->read_phase_voltages_raw == nullptr ||
            !std::isfinite(cfg.sensor.vphase_r_up) ||
            !std::isfinite(cfg.sensor.vphase_r_down) ||
            cfg.sensor.vphase_r_up <= 0.0f ||
            cfg.sensor.vphase_r_down <= 0.0f)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::PHASE_VOLTAGE_CONFIG_INVALID);
        }
    }

if (cfg.brake.allow_mechanical_brake)
    {
        if (!Platform_MotorBuildConfig::supportsMechanicalBrake())
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::MECHANICAL_BRAKE_NOT_BUILT);
        }
        if (cfg.hal->mechanical_brake_engage == nullptr)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::MECHANICAL_BRAKE_HAL_INVALID);
        }
    }

const bool uses_brake_energy_fsm =
        cfg.brake.allow_foc_negative_iq ||
        cfg.brake.has_brake_resistor ||
        cfg.brake.has_battery_regen_path;
    if (uses_brake_energy_fsm &&
        !Platform_MotorBuildConfig::supportsBrakeEnergyFsm())
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BRAKE_ENERGY_FSM_NOT_BUILT);
    }

    // 反向 Iq 输出后, 能量必然经 H 桥体二极管灌到母线电容 → 母线电压快速上升。
    // 若既没有回充路径, 也没有泄放电阻, 也没有 low_side 短路 (后者属 StopFSM 主动停机),
    // 运行期 EnergyFSM 也只能 BLOCKED → 物理上等价于负 Iq 一出母线过压 → 硬件 OVP 炸管。
    // init 阶段显式拒绝而非运行期被动等过压。
    if (cfg.brake.allow_foc_negative_iq &&
        !cfg.brake.has_battery_regen_path &&
        !cfg.brake.has_brake_resistor)
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BRAKE_NO_SINK_PATH);
    }

    if (cfg.brake.has_battery_regen_path &&
        !Platform_MotorBuildConfig::supportsBatteryRegenPath())
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BATTERY_REGEN_NOT_BUILT);
    }

if (cfg.brake.has_brake_resistor &&
        !Platform_MotorBuildConfig::supportsBrakeResistor())
    {
        return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BRAKE_RESISTOR_NOT_BUILT);
    }

    if (cfg.brake.control_brake_resistor_chopper)
    {
        if (!cfg.brake.has_brake_resistor)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BRAKE_CHOPPER_REQUIRES_RESISTOR);
        }
        if (!Platform_MotorBuildConfig::supportsBrakeChopperControl())
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BRAKE_CHOPPER_NOT_BUILT);
        }
        if (cfg.hal->brake_resistor_set_duty == nullptr ||
            cfg.hal->brake_resistor_disable == nullptr)
        {
            return configFail(detail, Fault::PARAM_ERROR, MotorConfigFaultDetail::BRAKE_CHOPPER_HAL_INVALID);
        }
    }

    /* === [本轮新增] SIXSTEP 调制要求 BuildCfg 开 TRAP_CONTROL === */
    if (cfg.control.modulation == ModulationMethod::SIXSTEP &&
        !Platform_MotorBuildConfig::supportsTrapControl())
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::MODULATION_SIXSTEP_REQUIRES_TRAP);
    }

#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    {
        const auto& identify = cfg.identify;
        const bool rl_identify_param_invalid =
            !std::isfinite(identify.rl_target_current_a) ||
            !std::isfinite(identify.rl_injection_voltage_limit_v) ||
            identify.rl_target_current_a <= 0.0f ||
            identify.rl_injection_voltage_limit_v <= 0.0f ||
            identify.rl_repeat_count == 0U ||
            identify.rl_repeat_count > 16U;

        if (rl_identify_param_invalid)
        {
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::RL_IDENTIFY_PARAM_INVALID);
        }
    }
#endif

    /* === [本轮新增] 电流环带宽校验 ===
     * - auto_derive_current_pid=true 时: bw 必须 < control_freq_hz / 10, 否则离散化失效;
     * - R/L 未知不阻断 init: 先使用手填 fallback PI, RL 辨识完成后再自动推导覆盖;
     * - auto_derive_speed_pid=true: 当前未实现, 直接拒绝 (P5 阶段开放)
     */
    if (cfg.control.auto_derive_current_pid)
    {
        const float f_bw = cfg.control.current_loop_bandwidth_hz;
        const float f_ctrl = cfg.control.control_freq_hz;
        if (f_ctrl > 0.0f && f_bw >= f_ctrl / 10.0f)
        {
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::CURRENT_LOOP_BW_TOO_HIGH);
        }
    }

    if (cfg.control.auto_derive_speed_pid)
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::SPEED_LOOP_AUTOTUNE_NOT_IMPLEMENTED);
    }

    /* [P4] 位置环自动推导预留: 当前未实现, 直接拒绝 (P5 阶段开放) */
    if (cfg.control.auto_derive_position_pid)
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::POSITION_LOOP_AUTOTUNE_NOT_IMPLEMENTED);
    }

    /* === [P2] Ke 校验: 纯值判定 (measured + identified 都 0 时视为未知, 不锁错) ===
     * 当前电压圆限幅仅按 Kmod*Vbus 约束, 不扣减 Ke。
     * Ke 有效值保留给后续反电势前馈、弱磁或速度裕量预算使用。
     * 撤掉 KE_OUT_OF_RANGE 校验, 由用户填错时通过运行时观测自行察觉。
     */

    return configPass(detail);
}

/*
 * validateFeedback -- 反馈路径配置验证
 *
 * 检查项:
 *   - 稳态角度源 (SENSOR/SMO/HFI 等) 已编译
 *   - 启动角度源已编译
 *   - 角度跳变检查参数合法
 *   - 冗余传感器配置完整性 (传感器接口 + 冗余检查参数)
 *   - 输出传感器配置完整性
 *   - 各角度源的具体参数完整性:
 *       SENSOR: 传感器接口/反馈能力/AngleFeedback 参数
 *       SMO:    滞环速度/切换速度/对齐参数/拖拽参数/切换误差/收敛计数
 *       其他:   暂不支持
 *
 * 返回 Fault::NONE 或对应的故障码
 */
Fault MotorConfigCheck::validateFeedback(const MotorConfig& cfg,
                                         MotorConfigFaultDetail* detail)
{
    return validateFeedback(cfg, cfg.default_run_policy, detail);
}

Fault MotorConfigCheck::validateFeedback(const MotorConfig& cfg,
                                         const MotorRunPolicy& policy,
                                         MotorConfigFaultDetail* detail)
{
    return validateFeedback(MakeMotorHardwareConfig(cfg),
                            MakeMotorAlgorithmParam(cfg),
                            policy,
                            detail);
}

/*
 * validateStartupTargetMode -- 校验启动目标模式是否合法
 *
 * 检查项:
 *   - NONE/DEBUG/CALIB 模式: 直接放行
 *   - TORQUE_CONTROL: 需模式已编译; SENSOR 模式下还需有感反馈支持转矩环
 *   - VELOCITY_CONTROL: 需模式已编译; SENSOR 模式下还需有感反馈支持速度环
 *   - POSITION_CONTROL: 需模式已编译; SENSOR 模式下还需高分辨率反馈
 *
 * 返回 Fault::NONE 或对应的故障码
 */
Fault MotorConfigCheck::validateStartupTargetMode(const MotorConfig& cfg,
                                                  MotorConfigFaultDetail* detail)
{
    const Mode mode = cfg.control.startup_target_mode;

    if (!MotorFeatureRegistry::supportsMode(mode))
    {
        switch (mode)
        {
        case Mode::DEBUG_PWM_MANUAL:
        case Mode::DEBUG_CURRENT_LOCK:
        case Mode::DEBUG_VF_DRAG:
        case Mode::DEBUG_HFI_OBSERVER:
        case Mode::DEBUG_IF_DRAG:
        case Mode::DEBUG_IF_SMO_OBSERVER:
        case Mode::DEBUG_IF_HFI_OBSERVER:
            if (!MotorFeatureRegistry::supportsDangerousTestApi())
            {
                return configFail(detail,
                                  Fault::PARAM_ERROR,
                                  MotorConfigFaultDetail::STARTUP_DEBUG_API_DISABLED);
            }
            if (mode == Mode::DEBUG_IF_DRAG ||
                mode == Mode::DEBUG_IF_SMO_OBSERVER ||
                mode == Mode::DEBUG_IF_HFI_OBSERVER)
            {
                return configFail(detail,
                                  Fault::PARAM_ERROR,
                                  MotorConfigFaultDetail::STARTUP_IF_DEBUG_DISABLED);
            }
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::STARTUP_MODE_INVALID);

        case Mode::POSITION_CONTROL:
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::STARTUP_POSITION_UNSUPPORTED);

        default:
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::STARTUP_MODE_INVALID);
        }
    }

    switch (mode)
    {
        case Mode::NONE:
        case Mode::DEBUG_PWM_MANUAL:
        case Mode::DEBUG_CURRENT_LOCK:
        case Mode::DEBUG_VF_DRAG:
        case Mode::DEBUG_IF_DRAG:
        case Mode::DEBUG_IF_SMO_OBSERVER:
            return configPass(detail);

        case Mode::DEBUG_HFI_OBSERVER:
        case Mode::DEBUG_IF_HFI_OBSERVER:
            if (cfg.sensor.current_sense_mode == CurrentSenseMode::SINGLE_SHUNT)
            {
                return configFail(detail,
                                  Fault::OBSERVER_UNAVAILABLE,
                                  MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED);
            }
            return configPass(detail);

        case Mode::CALIB_RL_IDENTIFY:
            /* [P4] RL 辨识必须 BuildCfg 开 AUTO_IDENTIFY, 否则拒绝 */
            if (!Platform_MotorBuildConfig::supportsAutoIdentify())
            {
                return configFail(detail, Fault::OBSERVER_UNAVAILABLE,
                                  MotorConfigFaultDetail::RL_IDENTIFY_NOT_BUILT);
            }
            return configPass(detail);

        case Mode::TORQUE_CONTROL:
            if (cfg.default_run_policy.steady_source == SteadyAngleSource::SENSOR)
            {
                const AngleFeedbackClass feedback_class = cfg.position.rotor_feedback.feedback_class;
                if (!FeedbackClassSupportsTorqueControl(feedback_class))
                {
                    return configFail(detail,
                                      Fault::SENSOR_CONFIG_ERROR,
                                      MotorConfigFaultDetail::STARTUP_SENSOR_TORQUE_UNSUPPORTED);
                }
            }
            return configPass(detail);

        case Mode::VELOCITY_CONTROL:
            if (cfg.default_run_policy.steady_source == SteadyAngleSource::SENSOR)
            {
                const AngleFeedbackClass feedback_class = cfg.position.rotor_feedback.feedback_class;
                if (!FeedbackClassSupportsTorqueControl(feedback_class) ||
                    !FeedbackClassSupportsSpeedControl(feedback_class))
                {
                    return configFail(detail,
                                      Fault::SENSOR_CONFIG_ERROR,
                                      MotorConfigFaultDetail::STARTUP_SENSOR_SPEED_UNSUPPORTED);
                }
            }
            return configPass(detail);

        case Mode::POSITION_CONTROL:
            if (cfg.default_run_policy.steady_source != SteadyAngleSource::SENSOR)
            {
                return configFail(detail,
                                  Fault::SENSOR_CONFIG_ERROR,
                                  MotorConfigFaultDetail::STARTUP_POSITION_UNSUPPORTED);
            }
            else
            {
                const AngleFeedbackClass feedback_class = cfg.position.rotor_feedback.feedback_class;
                if (!FeedbackClassSupportsPositionControl(feedback_class))
                {
                    return configFail(detail,
                                      Fault::SENSOR_CONFIG_ERROR,
                                      MotorConfigFaultDetail::STARTUP_POSITION_UNSUPPORTED);
                }
            }
            return configPass(detail);

        default:
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::STARTUP_MODE_INVALID);
    }
}

/*
 * validateInitConfig -- 初始化配置总校验入口
 *
 * 依次调用:
 *   1. validateBuildConfig  -- BuildCfg 宏合法性
 *   2. validateBase         -- 基础硬件配置
 *   3. validateFeedback     -- 反馈路径配置
 *   4. validateStartupTargetMode -- 启动目标模式
 *
 * 任一环节失败立即返回对应故障码, 全部通过返回 Fault::NONE
 */
Fault MotorConfigCheck::validateInitConfig(const MotorConfig& cfg,
                                           MotorConfigFaultDetail* detail)
{
    Fault fault = validateBuildConfig(detail);
    if (fault != Fault::NONE)
    {
        return fault;
    }

    fault = validateBase(cfg, detail);
    if (fault != Fault::NONE)
    {
        return fault;
    }

    fault = validateFeedback(cfg, detail);
    if (fault != Fault::NONE)
    {
        return fault;
    }

    return validateStartupTargetMode(cfg, detail);
}

Fault MotorConfigCheck::validateFeedback(const MotorHardwareConfig& hardware,
                                         const MotorAlgorithmParam& algorithm,
                                         const MotorRunPolicy& policy,
                                         MotorConfigFaultDetail* detail)
{
    if (hardware.position.enable_redundant_rotor_sensor &&
        !Platform_MotorBuildConfig::supportsRedundantSensor())
    {
        return configFail(detail,
                          Fault::SENSOR_CONFIG_ERROR,
                          MotorConfigFaultDetail::FEEDBACK_REDUNDANT_NOT_BUILT);
    }

    if (hardware.position.enable_output_sensor &&
        !Platform_MotorBuildConfig::supportsOutputSensor())
    {
        return configFail(detail,
                          Fault::SENSOR_CONFIG_ERROR,
                          MotorConfigFaultDetail::FEEDBACK_OUTPUT_NOT_BUILT);
    }
    /* === [P3.B] 顺逆风与静止保证二维校验 (与启动源解耦) ===
     * 决策矩阵:
     *   flying_start_mode=PHASE_VOLTAGE -> 仅允许非 SENSOR 启动; 必须硬件 has_phase_voltage + BuildCfg PHASE_VOLTAGE + FLYING_START;
     *   flying_start_mode=DISABLED:
     *     - startup_source=SENSOR: 编码器直接给角度, 不需静止保证 -> 通过;
     *     - 其他 startup_source (IF/HFI): 必须有 rotor_stationary_guarantee != NONE, 否则报错;
     *     - 选 LOW_SIDE_BRAKE 校验 HAL pwm_brake_lowside;
     *     - 选 PHASE_VOLTAGE_FLYING 校验硬件能力 (相当于不变相电压)
     *     - 选 EXPLICIT_BYPASS 校验 rotor_explicit_bypass_confirmed=true
     *   flying_start_mode=AUTO_DETECT: P3 占位拒绝
     */
    switch (algorithm.observer.flying_start_mode)
    {
        case FlyingStartMode::DISABLED:
        {
            if (policy.startup_source == StartupSource::SENSOR) break;
            const RotorStationaryGuarantee g = algorithm.observer.rotor_stationary_guarantee;
            switch (g)
            {
                case RotorStationaryGuarantee::NONE:
                    return configFail(detail, Fault::PARAM_ERROR,
                                      MotorConfigFaultDetail::IF_STATIONARY_GUARANTEE_REQUIRED);
                case RotorStationaryGuarantee::PHASE_VOLTAGE_FLYING:
                    if (!Platform_MotorBuildConfig::supportsPhaseVoltage() ||
                        !hardware.sensor.has_phase_voltage)
                    {
                        return configFail(detail, Fault::PARAM_ERROR,
                                          MotorConfigFaultDetail::IF_STATIONARY_NEEDS_PHASE_VOLTAGE_BUILD);
                    }
                    break;
                case RotorStationaryGuarantee::LOW_SIDE_BRAKE:
                    if (hardware.hal == nullptr ||
                        hardware.hal->pwm_brake_lowside == nullptr)
                    {
                        return configFail(detail, Fault::PARAM_ERROR,
                                          MotorConfigFaultDetail::IF_STATIONARY_NEEDS_LOW_SIDE_BRAKE);
                    }
                    break;
                case RotorStationaryGuarantee::EXPLICIT_BYPASS:
                    if (!algorithm.observer.rotor_explicit_bypass_confirmed)
                    {
                        return configFail(detail, Fault::PARAM_ERROR,
                                          MotorConfigFaultDetail::IF_STATIONARY_EXPLICIT_BYPASS_UNCONFIRMED);
                    }
                    break;
                default:
                    return configFail(detail, Fault::PARAM_ERROR,
                                      MotorConfigFaultDetail::IF_STATIONARY_GUARANTEE_REQUIRED);
            }
            break;
        }

        case FlyingStartMode::PHASE_VOLTAGE:
            if (policy.startup_source == StartupSource::SENSOR)
            {
                return configFail(detail, Fault::PARAM_ERROR,
                                  MotorConfigFaultDetail::FEEDBACK_FLYING_START_SENSOR_STARTUP_UNSUPPORTED);
            }
            if (!hardware.sensor.has_phase_voltage ||
                !Platform_MotorBuildConfig::supportsPhaseVoltage())
            {
                return configFail(detail, Fault::PARAM_ERROR,
                                  MotorConfigFaultDetail::FEEDBACK_FLYING_START_NEEDS_PHASE_VOLTAGE);
            }
            if (!Platform_MotorBuildConfig::supportsFlyingStart())
            {
                return configFail(detail, Fault::OBSERVER_UNAVAILABLE,
                                  MotorConfigFaultDetail::FEEDBACK_FLYING_START_NOT_BUILT);
            }
            break;

        case FlyingStartMode::AUTO_DETECT:
            // P3 占位, 实际实现留 P4
            return configFail(detail, Fault::OBSERVER_UNAVAILABLE,
                              MotorConfigFaultDetail::FEEDBACK_FLYING_START_UNAVAILABLE);

        default:
            return configFail(detail, Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_INVALID);
    }

    if (!MotorFeatureRegistry::supportsSteadySource(policy.steady_source))
    {
        return configFail(detail,
                          Fault::OBSERVER_UNAVAILABLE,
                          MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_NOT_BUILT);
    }
    if (!MotorFeatureRegistry::supportsStartupSource(policy.startup_source))
    {
        return configFail(detail,
                          Fault::OBSERVER_UNAVAILABLE,
                          MotorConfigFaultDetail::FEEDBACK_STARTUP_SOURCE_NOT_BUILT);
    }

    if ((hardware.position.health.enable_angle_step_check &&
         hardware.position.health.max_mech_angle_step_rad <= 0.0f) ||
        (hardware.position.enable_output_sensor &&
         hardware.position.transmission_ratio <= 0.0f))
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::FEEDBACK_HEALTH_PARAM_INVALID);
    }

    if (hardware.position.enable_redundant_rotor_sensor &&
        !validSensor(hardware.position.redundant_rotor_sensor))
    {
        return configFail(detail,
                          Fault::SENSOR_CONFIG_ERROR,
                          MotorConfigFaultDetail::FEEDBACK_REDUNDANT_SENSOR_INVALID);
    }
    if (hardware.position.enable_output_sensor &&
        !validSensor(hardware.position.output_sensor))
    {
        return configFail(detail,
                          Fault::SENSOR_CONFIG_ERROR,
                          MotorConfigFaultDetail::FEEDBACK_OUTPUT_SENSOR_INVALID);
    }
    if (hardware.position.health.enable_redundant_check &&
        (!hardware.position.enable_redundant_rotor_sensor ||
         hardware.position.health.max_redundant_error_rad <= 0.0f ||
         hardware.position.health.redundant_confirm_ticks == 0U))
    {
        return configFail(detail,
                          Fault::SENSOR_CONFIG_ERROR,
                          MotorConfigFaultDetail::FEEDBACK_REDUNDANT_CHECK_INVALID);
    }

    const bool uses_hfi =
        policy.startup_source == StartupSource::HFI ||
        policy.steady_source == SteadyAngleSource::HFI;
    if (hardware.sensor.current_sense_mode == CurrentSenseMode::SINGLE_SHUNT &&
        uses_hfi)
    {
        return configFail(detail,
                          Fault::OBSERVER_UNAVAILABLE,
                          MotorConfigFaultDetail::FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED);
    }
    if (uses_hfi && !validHfiParams(algorithm.observer))
    {
        return configFail(detail,
                          Fault::PARAM_ERROR,
                          MotorConfigFaultDetail::FEEDBACK_HFI_PARAM_INVALID);
    }

    switch (policy.steady_source)
    {
        case SteadyAngleSource::SENSOR:
            if (!validSensor(hardware.position.rotor_sensor))
            {
                return configFail(detail,
                                  Fault::SENSOR_CONFIG_ERROR,
                                  MotorConfigFaultDetail::FEEDBACK_ROTOR_SENSOR_INVALID);
            }
            if (policy.startup_source != StartupSource::SENSOR)
            {
                return configFail(detail,
                                  Fault::SENSOR_CONFIG_ERROR,
                                  MotorConfigFaultDetail::FEEDBACK_SENSOR_STARTUP_MISMATCH);
            }
            if (hardware.position.rotor_feedback.feedback_class == AngleFeedbackClass::UNSPECIFIED ||
                !FeedbackClassSupportsTorqueControl(hardware.position.rotor_feedback.feedback_class) ||
                !FeedbackClassSupportsSpeedControl(hardware.position.rotor_feedback.feedback_class) ||
                !FeedbackClassAngleValidAtStandstill(hardware.position.rotor_feedback.feedback_class))
            {
                return configFail(detail,
                                  Fault::SENSOR_CONFIG_ERROR,
                                  MotorConfigFaultDetail::FEEDBACK_ROTOR_CAPABILITY_INVALID);
            }
            if (hardware.position.health.invalid_confirm_ticks == 0U)
            {
                return configFail(detail,
                                  Fault::PARAM_ERROR,
                                  MotorConfigFaultDetail::FEEDBACK_SENSOR_INVALID_TICKS);
            }
            if (algorithm.observer.sensor_fault_action != SensorFaultAction::STOP)
            {
                return configFail(detail,
                                  Fault::OBSERVER_UNAVAILABLE,
                                  MotorConfigFaultDetail::FEEDBACK_SENSOR_FAULT_ACTION_UNSUPPORTED);
            }
            return configPass(detail);

        case SteadyAngleSource::SMO:
            if (!algorithm.observer.allow_smo_closed_loop)
            {
                return configFail(detail,
                                  Fault::OBSERVER_UNAVAILABLE,
                                  MotorConfigFaultDetail::FEEDBACK_SMO_CLOSED_LOOP_DISABLED);
            }
            if (algorithm.observer.hfi_to_smo_startup_hysteresis_rpm < 0.0f ||
                algorithm.observer.hfi_to_smo_startup_rpm <= algorithm.observer.hfi_to_smo_startup_hysteresis_rpm ||
                !std::isfinite(algorithm.observer.smo_min_signal_level) ||
                algorithm.observer.smo_min_signal_level < 0.0f ||
                algorithm.observer.max_handover_error_rad <= 0.0f ||
                algorithm.observer.convergence_ticks == 0U)
            {
                return configFail(detail,
                                  Fault::PARAM_ERROR,
                                  MotorConfigFaultDetail::FEEDBACK_SMO_PARAM_INVALID);
            }

switch (policy.startup_source)
            {
                case StartupSource::IF:
                    if (algorithm.observer.align_time_s < 0.0f ||
                        algorithm.observer.drag_current_a <= 0.0f ||
                        algorithm.observer.drag_accel_rpm_s <= 0.0f ||
                        algorithm.observer.force_drag_timeout_s <= 0.0f ||
                        !validIFStartupProfile(algorithm.observer.if_startup_profile))
                    {
                        return configFail(detail,
                                          Fault::PARAM_ERROR,
                                          MotorConfigFaultDetail::FEEDBACK_IF_PARAM_INVALID);
                    }

                    return configPass(detail);

                case StartupSource::HFI:
                    /*
                     * HFI 观测器目前只有占位类，尚未实现注入、解调和角度输出。
                     * RunPhaseFSM 同样按不可用处理，避免配置检查通过后才在运行中失败。
                     */
                    return configFail(detail,
                                      Fault::OBSERVER_UNAVAILABLE,
                                      MotorConfigFaultDetail::FEEDBACK_HFI_UNAVAILABLE);

                case StartupSource::SENSOR:
                    /*
                     * SENSOR -> SMO 的影子观测与无扰交接尚未接入 AngleFSM，
                     * 当前不能把它声明为可执行启动链。
                     */
                    return configFail(detail,
                                      Fault::OBSERVER_UNAVAILABLE,
                                      MotorConfigFaultDetail::FEEDBACK_SENSOR_TO_SMO_UNAVAILABLE);

                default:
                    return configFail(detail,
                                      Fault::PARAM_ERROR,
                                      MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_INVALID);
            }

        case SteadyAngleSource::HFI:
        case SteadyAngleSource::NONLINEAR_FLUX:
        case SteadyAngleSource::LUENBERGER:
        case SteadyAngleSource::EKF:
            return configFail(detail,
                              Fault::OBSERVER_UNAVAILABLE,
                              MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_UNAVAILABLE);

        default:
            return configFail(detail,
                              Fault::PARAM_ERROR,
                              MotorConfigFaultDetail::FEEDBACK_STEADY_SOURCE_INVALID);
    }
}

/*
 * validSensor -- 检查传感器接口有效性
 * 要求: 接口指针非空, read_angle 非空, is_ready 非空
 */
bool MotorConfigCheck::validSensor(const SensorInterface_t* sensor)
{
    return sensor != nullptr &&
           sensor->read_angle != nullptr &&
           sensor->is_ready != nullptr;
}

} // namespace Lib_Motor
