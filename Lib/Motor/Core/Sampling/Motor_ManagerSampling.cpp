#include "../Manager/Motor_Manager.h"

#include "../Safety/Motor_SignalHealth.h"

#include <cmath>

namespace Lib_Motor
{


/*
 * updateSensorMeasurements() -- ADC 采样 -> 物理量换算 + 低通滤波
 *
 * 每个 tick 调用, 步骤:
 *
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │ [1] 读取 ADC 原始值 (BSP 层: 触发 ADC 转换)                       │
 *   │     raw_u, raw_v, raw_w, raw_ibus                                │
 *   │                                                                  │
 *   │ [2] ADC count -> 电流 (A)                                        │
 *   │     current = (raw - offset) × scale                             │
 *   │     BSP 输出的 ADC 原始值顺序为 U/V/W; 三相电流符号由配置决定       │
 *   │     DUAL_SENSOR: 由 BSP 可编程运放通道有效性 + KCL 推导缺失相      │
 *   │     TRIPLE_SENSOR: 三电阻独立采样                                 │
 *   │                                                                  │
 *   │ [3] 低通滤波: 一阶 IIR 滤波以抑制噪声 (ADC 噪声/EMI 耦合)          │
 *   │     i_a = lpf_ia_.update(ia)  (一阶 IIR)                         │
 *   │                                                                  │
 *   │ [4] 母线电压 (V)                                                  │
 *   │     v_bus = raw_adc × voltage_scale + 低通滤波                    │
 *   │     不带传感器的母线电压默认为 50% over_voltage_v                  │
 *   │                                                                  │
 *   │ [5] NTC 温度 (低通: ~20Hz)                                        │
 *   │     slow_loop_counter 每 512 tick (约 25.6ms) 采样一次            │
 *   │     raw_temp -> Steinhart-Hart 公式 -> 温度 °C                    │
 *   └──────────────────────────────────────────────────────────────────┘
 */
void MotorManager::updateSensorMeasurements()
{
    const auto* hal = config_.hal;
    if (hal == nullptr) return;

    /* ================================================================
     * [1] 读取 ADC 原始值
     *     BSP 层触发 ADC 转换并返回 4 路 (三相反馈电流 + 母线反馈电流)
     *     read_currents_raw 返回三相电流 + 母线电流原始值
     * ================================================================ */
    uint16_t raw_u = 0, raw_v = 0, raw_w = 0, raw_ibus = 0;
    uint16_t raw_single_first = 0, raw_single_second = 0;
    bool single_pair_valid = false;
    if (config_.sensor.current_sense_mode == CurrentSenseMode::SINGLE_SHUNT)
    {
        if (hal->read_single_shunt_pair_raw != nullptr)
        {
            single_pair_valid =
                hal->read_single_shunt_pair_raw(&raw_single_first, &raw_single_second);
        }
    }
    else
    {
        if (hal->read_currents_raw != nullptr)
        {
            hal->read_currents_raw(&raw_u, &raw_v, &raw_w, &raw_ibus);
        }
    }
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    ctx_.adc_raw_phase_u = raw_u;
    ctx_.adc_raw_phase_v = raw_v;
    ctx_.adc_raw_phase_w = raw_w;
    ctx_.adc_raw_single_shunt_first = raw_single_first;
    ctx_.adc_raw_single_shunt_second = raw_single_second;
    ctx_.adc_raw_bus_current = raw_ibus;
#endif

    float scale_phase = ctx_.current_scale_Phase_per_count;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    float scale_bus   = ctx_.current_scale_Bus_per_count;
#endif

    /* ================================================================
     * [2] ADC count -> 电流 (A)
     *     current = (raw_adc - calib_offset) * scale_A_per_count
     *
     *     offset 由 runAdcCalibration() 校准获得 (PWM 关闭时的 ADC 零位)
     *
     *     BSP 输出的 ADC 原始值顺序为 U/V/W
     *     三相电流符号由配置决定
     *     DUAL_SENSOR: 由 BSP 可编程运放通道有效性 + KCL 推导缺失相
     *     TRIPLE_SENSOR: 三电阻独立采样
     * ================================================================ */
    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;
    if (config_.sensor.current_sense_mode == CurrentSenseMode::SINGLE_SHUNT)
    {
        if (single_pair_valid)
        {
            const float sample_first =
                (raw_single_first - ctx_.offset_single_shunt_first) * scale_phase;
            const float sample_second =
                (raw_single_second - ctx_.offset_single_shunt_second) * scale_phase;
            MotorSingleShuntCurrents reconstructed;
            if (MotorSingleShunt::reconstruct(ctx_.single_shunt_sample_plan,
                                              sample_first,
                                              sample_second,
                                              reconstructed))
            {
                ia = reconstructed.ia;
                ib = reconstructed.ib;
                ic = reconstructed.ic;
            }
        }
    }
    else
    {
        ia = (raw_u - ctx_.offset_ia) * scale_phase;
        ib = (raw_v - ctx_.offset_ib) * scale_phase;
        ic = (raw_w - ctx_.offset_ic) * scale_phase;

        switch (config_.sensor.current_sense_mode)
        {
            case CurrentSenseMode::TRIPLE_SENSOR:
                break;
            case CurrentSenseMode::DUAL_SENSOR:
            default:
                if ((hal->phase_current_valid_mask & MOTOR_PHASE_CURRENT_U_VALID) == 0U)
                    ia = -(ib + ic);
                else if ((hal->phase_current_valid_mask & MOTOR_PHASE_CURRENT_V_VALID) == 0U)
                    ib = -(ia + ic);
                else
                    ic = -(ia + ib);
                break;
        }
    }

    if (config_.sensor.invert_phase_current_u) ia = -ia;
    if (config_.sensor.invert_phase_current_v) ib = -ib;
    if (config_.sensor.invert_phase_current_w) ic = -ic;

    ctx_.i_alpha_raw = ia;
    ctx_.i_beta_raw  = 0.577350269f * (ia + 2.0f * ib);

    /* ================================================================
     * [3] 低通滤波 (一阶 IIR)
     *
     *     y[n] = alpha * x[n] + (1-alpha) * y[n-1]
     *     alpha = dt_ / (Tf + dt_)
     *
     *     FOC 需要 ia/ib/ic 滤波后的平滑信号反馈
     *     Tf=0 则跳过滤波 (直通)
     *     Tf 由配置决定, 典型值为 0.5~2 个控制周期
     * ================================================================ */
    ctx_.i_a = lpf_ia_.update(ia);
    ctx_.i_b = lpf_ib_.update(ib);
    ctx_.i_c = lpf_ic_.update(ic);

    /* ================================================================
     * [3续] 母线电流滤波
     *
     *       有母线电流传感器 (has_bus_current=true) 时对 ibus 进行滤波
     *       无传感器时设为 0 (缺失值)
     * ================================================================ */
    if (config_.sensor.has_bus_current)
    {
#if MOTOR_BUILD_HAS_BUS_CURRENT
        float ibus = (raw_ibus - ctx_.offset_ibus) * scale_bus;
        ctx_.i_bus = lpf_ibus_.update(ibus);
#endif
    }
#if MOTOR_BUILD_HAS_BUS_CURRENT
    else
    {
        ctx_.i_bus = 0.0f;
    }
#endif

    /* ================================================================
     * [4] 母线电压 (V)
     *
     * 有电压传感器: raw_adc * voltage_scale_V_per_count -> V
     * 无电压传感器: 默认为 50% over_voltage_v
     *   (简单估算, 在无传感器场景供电电压不变)
     * ================================================================ */
    if (config_.sensor.has_bus_voltage && hal->read_vbus_raw != nullptr)
    {
        uint16_t raw_vbus = hal->read_vbus_raw();
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
        ctx_.adc_raw_vbus = raw_vbus;
#endif
        ctx_.v_bus = lpf_vbus_.update(raw_vbus * ctx_.voltage_scale_V_per_count);
    }
    else
    {
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
        ctx_.adc_raw_vbus = 0U;
#endif
        ctx_.v_bus = config_.limit.over_voltage_v * 0.5f;
    }

#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    if (config_.sensor.has_phase_voltage && hal->read_phase_voltages_raw != nullptr)
    {
        uint16_t raw_vu = 0, raw_vv = 0, raw_vw = 0;
        hal->read_phase_voltages_raw(&raw_vu, &raw_vv, &raw_vw);
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
        ctx_.adc_raw_phase_voltage_u = raw_vu;
        ctx_.adc_raw_phase_voltage_v = raw_vv;
        ctx_.adc_raw_phase_voltage_w = raw_vw;
#endif
        ctx_.v_phase_u = lpf_vphase_u_.update(raw_vu * ctx_.phase_voltage_scale_V_per_count);
        ctx_.v_phase_v = lpf_vphase_v_.update(raw_vv * ctx_.phase_voltage_scale_V_per_count);
        ctx_.v_phase_w = lpf_vphase_w_.update(raw_vw * ctx_.phase_voltage_scale_V_per_count);
    }
    else
    {
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
        ctx_.adc_raw_phase_voltage_u = 0U;
        ctx_.adc_raw_phase_voltage_v = 0U;
        ctx_.adc_raw_phase_voltage_w = 0U;
#endif
        ctx_.v_phase_u = 0.0f;
        ctx_.v_phase_v = 0.0f;
        ctx_.v_phase_w = 0.0f;
    }
#endif

    /* ================================================================
     * [5] NTC 温度采样 (低频率, 约 39Hz)
     *
     * slow_loop_counter 每 tick 递增
     * 0x1FF = 511 -> 每 512 tick (50us * 512 = 25.6ms) 采样一次
     *
     * 降频原因: NTC 温度变化缓慢, 无需每 20kHz 采样
     *
     * NTC 温度计算流程:
     *   raw_temp -> calculateNTC() -> Steinhart-Hart 公式 -> 温度 °C
     * ================================================================ */
    ctx_.slow_loop_counter++;
#if MOTOR_BUILD_NTC_SLOTS > 0
    if ((ctx_.slow_loop_counter & 0x1FF) == 0)
    {
        for (uint8_t i = 0; i < MOTOR_BUILD_NTC_SLOTS; i++)
        {
            if (!config_.sensor.ntc[i].enabled) continue;
            if (hal->read_temp_raw == nullptr) continue;
            uint16_t raw_temp = hal->read_temp_raw(i);
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
            ctx_.adc_raw_temp[i] = raw_temp;
#endif
            ctx_.temperature_c[i] = calculateNTC(raw_temp, i);
        }
    }
#endif

    /* ================================================================
     * [6] Clarke 变换: ABC (三相电流) -> alpha/beta (两相正交)
     *
     *   i_alpha = ia                    -- 与 a 相同
     *   i_beta  = (ia + 2*ib) / sqrt(3) -- 正交分量
     *     由 ia + ib + ic = 0 推导
     *
     * alpha/beta 供 runObserverLoop (SMO 观测器) 和 runControlLoop (Park) 使用
     * ================================================================ */
    ctx_.i_alpha = ctx_.i_a;
    ctx_.i_beta  = 0.577350269f * (ctx_.i_a + 2.0f * ctx_.i_b);
}

/*
 * updatePositionSensorMeasurement() - 更新所有位置传感器读数
 *
 * 每 tick 调用一次, 更新所有配置的传感器 (主/冗余/输出) 读数。
 * 无传感器反馈时跳过 (VF/IF 开环模式)。
 *
 * 三个独立传感器通道:
 *   rotor_sensor            -- 主传感器, 用于 FOC 闭环控制
 *   redundant_rotor_sensor  -- 冗余传感器, 用于角度比对 (安全冗余)
 *   output_sensor           -- 输出端传感器, 用于传动偏差检测
 *
 * 注意:
 *   - sensor->get_health(): BSP 层传感器健康检测 (信号/通信/配置错误)
 *   - 主传感器: 角度跳变检测每 tick 运行一次 (安全冗余)
 *   - runSafetyCheck(): 冗余传感器角度超差合并到 fault_ 寄存器
 *   - runObserverLoop(): 传感器数据融合 + 速度计算 (PLL/极性等)
 */
void MotorManager::updatePositionSensorMeasurement()
{
    MotorSignalHealth::updatePositionSensors(*this);
}


/*
 * runAdcCalibration -- ADC 偏移校准
 *
 * 在 ADC_CAL 状态下每个 tick 调用, 累加 ADC 原始值计算偏移量。
 *
 * 流程:
 *   init() 清零 ctx_ (calib_accum_*, calib_counter = 0)
 *    |
 *   FSM: INIT -> ADC_CAL
 *    |
 *   每个 tick:
 *     read_currents_raw() -> 4 通道 ADC 值
 *     calib_accum_* += raw -> calib_counter++
 *    |
 *   当 calib_counter >= adc_calibration_samples:
 *     offset_* = calib_accum_* / calib_counter (算术平均)
 *     校准完成, FSM -> STOP
 *
 * 相电流 = (raw_adc - offset) * scale
 * 校准需在 PWM 关闭时进行以避免电流干扰。
 */
void MotorManager::runAdcCalibration()
{
    if (config_.hal == nullptr)
    {
        return;
    }

    uint16_t raw_u = 0, raw_v = 0, raw_w = 0, raw_ibus = 0;
    uint16_t raw_single_first = 0, raw_single_second = 0;
    if (config_.sensor.current_sense_mode == CurrentSenseMode::SINGLE_SHUNT)
    {
        if (config_.hal->read_single_shunt_pair_raw != nullptr)
        {
            config_.hal->read_single_shunt_pair_raw(&raw_single_first, &raw_single_second);
        }
    }
    else
    {
        if (config_.hal->read_currents_raw != nullptr)
        {
            config_.hal->read_currents_raw(&raw_u, &raw_v, &raw_w, &raw_ibus);
        }
    }

    ctx_.calib_accum_ia   += raw_u;
    ctx_.calib_accum_ib   += raw_v;
    ctx_.calib_accum_ic   += raw_w;
    ctx_.calib_accum_single_shunt_first += raw_single_first;
    ctx_.calib_accum_single_shunt_second += raw_single_second;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    ctx_.calib_accum_ibus += raw_ibus;
#endif
    ctx_.calib_counter++;

    if (ctx_.calib_counter >= config_.sensor.adc_calibration_samples)
    {
        float count = (float)ctx_.calib_counter;
        ctx_.offset_ia   = ctx_.calib_accum_ia   / count;
        ctx_.offset_ib   = ctx_.calib_accum_ib   / count;
        ctx_.offset_ic   = ctx_.calib_accum_ic   / count;
        ctx_.offset_single_shunt_first =
            ctx_.calib_accum_single_shunt_first / count;
        ctx_.offset_single_shunt_second =
            ctx_.calib_accum_single_shunt_second / count;
#if MOTOR_BUILD_HAS_BUS_CURRENT
        ctx_.offset_ibus = ctx_.calib_accum_ibus / count;
#endif

        lpf_ia_.reset();
        lpf_ib_.reset();
        lpf_ic_.reset();
#if MOTOR_BUILD_HAS_BUS_CURRENT
        lpf_ibus_.reset();
#endif
        ctx_.i_a = 0.0f;
        ctx_.i_b = 0.0f;
        ctx_.i_c = 0.0f;
#if MOTOR_BUILD_HAS_BUS_CURRENT
        ctx_.i_bus = 0.0f;
#endif
        ctx_.i_alpha_raw = 0.0f;
        ctx_.i_beta_raw = 0.0f;
        ctx_.i_alpha = 0.0f;
        ctx_.i_beta = 0.0f;
    }


}

/*
 * calculateNTC() -- Steinhart-Hart 温度计算
 *
 * 将 ADC 原始值转换为 NTC 温度值 (°C)
 *
 * 算法:
 *   [1] ADC -> 电压 (V)
 *       LOW_SIDE_NTC:  v_ntc = raw * Vref / adc_res
 *       HIGH_SIDE_NTC: v_ntc = Vref - raw * Vref / adc_res
 *       (取决于分压电阻在上臂还是下臂)
 *
 *   [2] 分压比 -> 电阻 (Ohm)
 *       r_ntc = R_series * v_ntc / (Vref - v_ntc)
 *
 *   [3] Beta 公式 -> 温度 (K)
 *       1/T = 1/T0 + ln(R/R25) / beta
 *       where T0 = 298.15K (25°C)
 *       -> T_K = 1 / (1/298.15 + ln(r_ntc/R25) / beta)
 *
 *   [4] K -> °C
 *       temp_C = T_K - 273.15
 *
 * NTC 参数 (MotorSensorParam::NtcConfig 结构体):
 *   topology    : LOW_SIDE_NTC / HIGH_SIDE_NTC
 *   r_series    : 分压电阻 (Ohm)
 *   r_25        : 25°C 时 NTC 电阻 (Ohm, 典型 10k)
 *   beta        : Beta 值 (K, 典型 3435)
 */
#if MOTOR_BUILD_NTC_SLOTS > 0
float MotorManager::calculateNTC(uint16_t raw_adc, uint8_t ntc_index)
{
    const auto& ntc = config_.sensor.ntc[ntc_index];
    float adc_res = config_.sensor.adc_resolution;
    float v_ref   = config_.sensor.adc_v_ref;

    /* [1] ADC count -> NTC 电压 (V) */
    float v_ntc;
    if (ntc.topology == NtcTopology::LOW_SIDE_NTC)
    {
        // 下臂 NTC: NTC 接地, 分压电阻接 Vref
        // NTC 电压 = ADC 读数对应的电压
        v_ntc = raw_adc * v_ref / adc_res;
    }
    else
    {
        // 上臂 NTC: NTC 接 Vref, 分压电阻接地
        // NTC 电压 = Vref - ADC 读数对应的电压 (分压电阻上的电压)
        v_ntc = v_ref - raw_adc * v_ref / adc_res;
    }

    /* [2] 分压比 -> NTC 电阻
     *     r_ntc / r_series = v_ntc / (v_ref - v_ntc)
     */
    float r_series = ntc.r_series;
    float r_ntc = r_series * v_ntc / (v_ref - v_ntc);

    /* [3] Beta 公式 -> 温度 (K)
     *     1/T = 1/T0 + ln(R/R25) / beta
     *     T0 = 25°C = 298.15K
     */
    float r25 = ntc.r_25;
    float beta = ntc.beta;

    float temp_k = 1.0f / (1.0f / 298.15f + logf(r_ntc / r25) / beta);

    /* [4] 开尔文转摄氏度 */
    return temp_k - 273.15f;
}
#endif

} // namespace Lib_Motor
