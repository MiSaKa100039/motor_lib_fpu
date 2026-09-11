/*
 * Motor_Config.h -- 电机库配置结构体定义
 *
 * 用户唯一需要填写的配置入口: MotorConfig
 * 包含多个子结构体, 覆盖硬件绑定、物理参数、保护、传感器、观测器、控制参数和策略预留
 *
 * 使用流程:
 *   Platform_Motor.cpp 中创建 MotorConfig cfg;
 *   依次填写各子结构体 -> cfg.hal = BSP_Get_HAL_Impl();
 *   -> motor.init(cfg) -> motor.start()
 */

#pragma once

#include <stdint.h>
#include "Motor_HAL_Interface.h"
#include "Motor_Definitions.h"
#include "Motor_Sensor_Interface.h"
#include "Motor_Features.h"

namespace Lib_Motor
{

enum class MotorPhysicalParamSource : uint8_t
{
    AUTO        = 0,  // 自动选择: 运行期 identified > measured > learned_cfg > 0
    MEASURED    = 1,  // 只使用仪器/datasheet 等手动测量固化值
    LEARNED_CFG = 2,  // 只使用上次自动辨识后人工回填的固化值
};

/* =========================================================
 * [1] 电机物理参数 -- 从铭牌或数据手册获取
 *     pole_pairs=0 会被 checkConfigValid() 拒绝
 * ========================================================= */
struct MotorPhysicalParam
{
    /* === 铭牌/装机参数 (永远手填, 无辨识通道) === */
    uint8_t pole_pairs      = 7;
    float   rated_current   = 1.0f;
    float   rated_voltage   = 24.0f;

    /* === 手动测量固化值 (=0 表示未知) ===
     * *_measured 表示用户确认后写进 cfg 的外部测量参数, 来源可以是仪器测量、
     * datasheet 或反拖台架。自动辨识后的人工回填值放到 *_learned_cfg。
     * rs/ls/ke 用值=0 作为"未知"哨兵; 这与 PID kp/ki=0 仍可能合法不同。
     */
    float   rs_ohm_measured            = 0.1f;    // 单相定子电阻 (Ω), 用户固化值
    float   ls_h_measured              = 0.001f;  // 单相等效电感 (H), 普通场景按 Ld=Lq=Ls 使用
    float   ld_h_measured              = 0.0f;    // D 轴电感 (H), 仅强凸极/高性能场景手动填写
    float   lq_h_measured              = 0.0f;    // Q 轴电感 (H), 仅强凸极/高性能场景手动填写
    float   ke_v_per_rad_s_measured    = 0.0f;    // 反电势常数 V/(rad/s), 用户固化值
    float   kv_rating                  = 100.0f;  // KV 值 (RPM/V) 历史占位, 暂未消费

    /* === 自动辨识后人工固化值 (=0 表示未回填) ===
     * *_learned_cfg 表示用户把上次 *_identified 结果确认后手动写回 cfg 的值。
     * 它与仪器测量值分开保存, 便于在 Ozone/代码评审时看清参数来源。
     */
    float   rs_ohm_learned_cfg            = 0.0f;
    float   ls_h_learned_cfg              = 0.0f;
    float   ke_v_per_rad_s_learned_cfg    = 0.0f;
    MotorPhysicalParamSource electrical_param_source = MotorPhysicalParamSource::AUTO;

    /* === 运行期辨识值 (RAM, 冷启动丢失) ===
     * *_identified 由库内 RL 辨识或后续显式 Ke 辨识流程写入, 平台 cfg 不手填。
     * 默认 0 = 未辨识; 若未接入 flash 持久化, 需要人工把确认值回填到 *_learned_cfg 或 *_measured。
     */
    float   rs_ohm_identified            = 0.0f;
    float   ls_h_identified            = 0.0f;
    float   ke_v_per_rad_s_identified    = 0.0f;

    /* === 取值优先级 helper (运行期只读, 上层永远用 effective 不直接读 measured/identified) ===
     * AUTO: identified>0 优先, 否则 measured>0, 否则 learned_cfg>0, 否则 0;
     * 未接入 flash 时冷启动 identified=0, 人工回填 learned_cfg 后可在 measured=0 时被 AUTO 使用;
     * 若 measured 与 learned_cfg 都有效且希望强制使用回填值, 选择 LEARNED_CFG。
     * MEASURED / LEARNED_CFG: 只使用指定来源, 避免调试时来源混淆。
     * Ld/Lq 只作为手动测量补充项参与电流环推导, 自动 RL 辨识仍只写标量 Ls;
     * 若只填 Ld/Lq 而 Ls=0, ls_effective() 使用较小有效值作为保守单标量;
     * 当前 SMO 估角与电压圆限幅不消费 Ke; Ke 字段保留给后续反电势前馈、
     * 弱磁、速度裕量预算或显式 Ke 辨识。
     */
    float rs_effective()  const
    {
        return selectElectricalParam(rs_ohm_identified,
                                     rs_ohm_measured,
                                     rs_ohm_learned_cfg);
    }
    float ls_effective()  const
    {
        const float scalar_l = selectElectricalParam(ls_h_identified,
                                                     ls_h_measured,
                                                     ls_h_learned_cfg);
        if (scalar_l > 0.0f)
        {
            return scalar_l;
        }

        if (electrical_param_source != MotorPhysicalParamSource::LEARNED_CFG)
        {
            return measured_saliency_l_min();
        }
        return 0.0f;
    }
    float ld_effective_for_current_loop() const
    {
        if (electrical_param_source == MotorPhysicalParamSource::AUTO &&
            ls_h_identified > 0.0f)
        {
            return ls_h_identified;
        }
        if (electrical_param_source != MotorPhysicalParamSource::LEARNED_CFG &&
            ld_h_measured > 0.0f)
        {
            return ld_h_measured;
        }
        return ls_effective();
    }
    float lq_effective_for_current_loop() const
    {
        if (electrical_param_source == MotorPhysicalParamSource::AUTO &&
            ls_h_identified > 0.0f)
        {
            return ls_h_identified;
        }
        if (electrical_param_source != MotorPhysicalParamSource::LEARNED_CFG &&
            lq_h_measured > 0.0f)
        {
            return lq_h_measured;
        }
        return ls_effective();
    }
    float ke_effective_for_limit() const
    {
        const float k = selectElectricalParam(ke_v_per_rad_s_identified,
                                              ke_v_per_rad_s_measured,
                                              ke_v_per_rad_s_learned_cfg);
        return k;   // 兼容旧名; 当前电压圆限幅不扣减 Ke
    }
    float ke_effective_for_feedforward() const
    {
        return ke_effective_for_limit();
    }

    /* === Phase 2: 负载动力学参数 (机器狗/机械臂预留, 打印机可填 0/默认) ===
     * load_inertia 在 P3 同时被速度环 auto_derive_speed_pid 消费;
     * 当前 =0 → auto_derive_speed_pid=true 会被 ConfigCheck 拒绝。
     */
    float   load_inertia    = 0.0f;
    float   gear_ratio      = 1.0f;
    float   torque_constant = 0.0f;

    float measured_saliency_l_min() const
    {
        const bool ld_valid = ld_h_measured > 0.0f;
        const bool lq_valid = lq_h_measured > 0.0f;
        if (ld_valid && lq_valid)
        {
            return (ld_h_measured < lq_h_measured) ? ld_h_measured : lq_h_measured;
        }
        if (ld_valid) return ld_h_measured;
        if (lq_valid) return lq_h_measured;
        return 0.0f;
    }

    float selectElectricalParam(float runtime_identified,
                                float measured,
                                float learned_cfg) const
    {
        switch (electrical_param_source)
        {
            case MotorPhysicalParamSource::MEASURED:
                return measured > 0.0f ? measured : 0.0f;
            case MotorPhysicalParamSource::LEARNED_CFG:
                return learned_cfg > 0.0f ? learned_cfg : 0.0f;
            case MotorPhysicalParamSource::AUTO:
            default:
                if (runtime_identified > 0.0f) return runtime_identified;
                if (measured > 0.0f) return measured;
                if (learned_cfg > 0.0f) return learned_cfg;
                return 0.0f;
        }
    }
};

/* =========================================================
 * [2] 保护限制 -- 超限触发 Fault, FSM 自动关断 PWM
 *     所有阈值在 Platform_Motor.cpp 中配置 (用户层)
 * ========================================================= */
struct MotorLimitParam
{
    float max_phase_current_a = 5.0f;    // 相线软件过流阈值 (A), 超过即 OVCURRENT
    float max_bus_current_a   = 5.0f;    // 母线软件过流阈值 (A), has_bus_current=true 时生效
    float max_speed_rpm       = 3000.0f; // 最大转速 (RPM)
    float max_duty_cycle      = 0.95f;   // 最大占空比限制 [0~1], 留出 ADC 采样窗口
    float over_voltage_v      = 30.0f;   // 过压阈值 (V)
    float under_voltage_v     = 10.0f;   // 欠压阈值 (V), 初始化采样完成后持续检查
};

/* =========================================================
 * [3] 传感器与采样配置 -- 适配不同硬件方案
 *
 *     两个正交维度, 自由组合:
 *       current_sensor_type: SHUNT_RESISTOR / HALL_SENSOR  (传感器类型)
 *       current_sense_mode:  DUAL_SENSOR / TRIPLE_SENSOR    (传感器数量)
 *
 *     has_bus_current: 是否硬件具备母线电流采样
 *     has_bus_voltage: 是否硬件具备母线电压采样
 *     ntc[]: 最多 4 个 NTC, 每个独立启用 + 独立阈值
 * ========================================================= */

enum class NtcTopology : uint8_t
{
    LOW_SIDE_NTC,   // NTC 接地, 分压电阻接 VCC -> V_out 随温升降低
    HIGH_SIDE_NTC   // NTC 接 VCC, 分压电阻接地 -> V_out 随温升升高
};

struct NtcConfig
{
    bool        enabled     = true;    // 是否启用此 NTC
    float       r_series    = 10000.0f;// 分压电阻 (Ω)
    float       r_25        = 10000.0f;// 25°C 时的 NTC 阻值 (Ω)
    float       beta        = 3950.0f; // B 值 (K)
    NtcTopology topology    = NtcTopology::LOW_SIDE_NTC;
    float       over_temp_c = 85.0f;   // 此 NTC 的过温保护阈值 (°C)
};

enum class CurrentSensorType : uint8_t
{
    SHUNT_RESISTOR = 0,  // 分流电阻 + 运放
    HALL_SENSOR    = 1,  // 霍尔电流传感器, 零点由 ADC offset 校准, 电流由 mV/A 灵敏度换算
    //预留数字采样信号
};

enum class CurrentSenseMode : uint8_t
{
    DUAL_SENSOR   = 2,  // 双传感器: BSP 声明实测的两相, 缺失相由 KCL 计算
    TRIPLE_SENSOR = 3,  // 三传感器: 三相独立采样
};

struct MotorSensorParam
{
    uint16_t adc_calibration_samples = 1000; // ADC 零点校准样本数

    float adc_v_ref      = 3.3f;     // ADC 参考电压 (V)
    float adc_resolution = 4096.0f;  // ADC 分辨率 (12-bit -> 4096)

    CurrentSensorType current_sensor_type = CurrentSensorType::SHUNT_RESISTOR; // 电流传感器类型
    CurrentSenseMode  current_sense_mode = CurrentSenseMode::DUAL_SENSOR;    // 传感器数量拓扑

    /*
     * read_currents_raw() 返回值已由 BSP 固定对应物理 U/V/W 相。
     * 此处仅修正各物理相在去除 ADC 零点后的采样方向, 不负责 ADC 通道映射。
     */
    bool invert_phase_current_u = false;
    bool invert_phase_current_v = false;
    bool invert_phase_current_w = false;
    bool  has_phase_voltage = false;    // 是否具备三相相电压采样硬件
    float vphase_r_up       = 100000.0f;// 相电压分压上臂电阻 (Ω)
    float vphase_r_down     = 6200.0f;  // 相电压分压下臂电阻 (Ω)

    bool  has_bus_voltage = true;     // 是否具备母线电压采样硬件
    float vbus_r_up       = 100000.0f;// 分压上臂电阻 (Ω)
    float vbus_r_down     = 6200.0f;  // 分压下臂电阻 (Ω)

    float phase_shunt_resistor = 0.001f; // 相电流采样电阻 (Ω), 分流电阻模式用
    float phase_amp_gain       = 50.0f;  // 相电流运放增益 (V/V), 分流电阻模式用

    float hall_sensitivity_mV_per_A = 4.4f; // 霍尔灵敏度 (mV/A), 零点由上电 ADC offset 校准

    bool  has_bus_current      = false;  // 是否具备母线电流采样硬件
    float bus_shunt_resistor   = 0.001f; // 母线电流采样电阻 (Ω)
    float bus_amp_gain         = 20.0f;  // 母线电流运放增益 (V/V)

    /*
     * 采样软件低通时间常数 (秒), 0 = 直通, 关闭软件滤波。
     * phase_current_lpf_tf_s 位于 Id/Iq 电流闭环反馈路径中:
     *   - 默认 50us 对应 fc ≈ 3.18kHz, 仅提供轻量去噪
     *   - PWM 同步采样足够干净时可设为 0
     *   - 不应设置为会明显限制目标电流环带宽的较大值
     */
    float phase_current_lpf_tf_s = 0.00005f;
    float phase_voltage_lpf_tf_s = 0.02f; // 相电压采样滤波时间常数
    float bus_current_lpf_tf_s   = 0.02f; // 母线电流监测滤波; has_bus_current=true 时生效
    float bus_voltage_lpf_tf_s   = 0.02f; // 母线电压监测/保护滤波

    NtcConfig ntc[4];           // 每个 NTC 独立配置

    float auto_calib_interval_s = 30.0f;  // STOP 状态自动校准间隔 (秒), 0=禁用静默校准
};

/* =========================================================
 * [4] 外部位置传感器硬件绑定
 *
 *     rotor_sensor 表示参与 FOC 的转子位置传感器。
 *     redundant_rotor_sensor 表示同轴冗余传感器, 仅用于诊断/未来接管。
 *     output_sensor 表示关节减速器输出轴传感器，用于形变监测，不可直接
 *     替代电机转子电角度。
 *     它与 observer.steady_source 的控制反馈选择解耦。
 *       - VF/IF 调试时可仅监测外部传感器
 *       - steady_source=SENSOR 闭环时必须配置并保持 ready
 * ========================================================= */
struct MotorPositionHealthParam
{
    bool enable_angle_step_check = false;       // 主转子角每 tick 跳变检查；仅启用后生效
    float max_mech_angle_step_rad = 0.0f;       // 允许的最大机械角增量 (rad/tick)
    uint16_t invalid_confirm_ticks = 1;         // 主传感器作为反馈源时连续无效确认数
    bool enable_redundant_check = false;        // true = 冗余转子角超差后锁故障
    float max_redundant_error_rad = 0.35f;      // 同轴两个传感器允许的电角度差
    uint16_t redundant_confirm_ticks = 3;       // 连续超差多少 tick 后报 SENSOR_MISMATCH
};

struct MotorFeedbackCapabilityParam
{
    AngleFeedbackClass feedback_class = AngleFeedbackClass::UNSPECIFIED;

    /*
     * 有感反馈能力由 feedback_class 统一推导。
     * 无感 SMO/HFI 不在这里伪装成 SENSOR 主反馈。
     */
    float position_resolution_mech_deg = 0.0f; // 有感机械角分辨率, 0=未知/不声明
    float min_valid_speed_rpm          = 0.0f; // 有感边沿估算最低有效速度, 0=无下限
};

inline bool FeedbackClassSupportsTorqueControl(AngleFeedbackClass feedback_class)
{
    return feedback_class == AngleFeedbackClass::HIGH_RES_ENCODER ||
           feedback_class == AngleFeedbackClass::HALL_SECTOR;
}

inline bool FeedbackClassSupportsSpeedControl(AngleFeedbackClass feedback_class)
{
    return feedback_class == AngleFeedbackClass::HIGH_RES_ENCODER ||
           feedback_class == AngleFeedbackClass::HALL_SECTOR;
}

inline bool FeedbackClassSupportsPositionControl(AngleFeedbackClass feedback_class)
{
    return feedback_class == AngleFeedbackClass::HIGH_RES_ENCODER;
}

inline bool FeedbackClassAngleValidAtStandstill(AngleFeedbackClass feedback_class)
{
    return feedback_class == AngleFeedbackClass::HIGH_RES_ENCODER ||
           feedback_class == AngleFeedbackClass::HALL_SECTOR;
}

struct MotorPositionParam
{
    SensorInterface_t* rotor_sensor = nullptr;            // 电机转子反馈/监测
    MotorFeedbackCapabilityParam rotor_feedback;          // 有感 SENSOR 主反馈链能力
    bool enable_redundant_rotor_sensor = false;
    SensorInterface_t* redundant_rotor_sensor = nullptr;  // 同轴冗余反馈源
    bool enable_output_sensor = false;
    SensorInterface_t* output_sensor = nullptr;           // 关节输出轴 (负载端)
    /*
     * 电机机械转角 / 负载输出轴机械转角。
     * 例如 50:1 减速器填 50.0f; 未启用 output_sensor 时本值不参与计算。
     * 缺省 1.0f 只是无减速的中性占位, 不代表实际关节传动比。
     */
    float transmission_ratio = 1.0f;
    MotorPositionHealthParam health;
};

/* =========================================================
 * [5] PID 参数 -- 电流环/速度环/位置环
 *     output_limit: 输出电压/电流钳制
 *     ramp_rate: 目标值变化率限制 (暂未实现)
 * ========================================================= */
struct PIDParam
{
    float kp           = 0.0f;  // 比例增益
    float ki           = 0.0f;  // 积分增益
    float kd           = 0.0f;  // 微分增益
    float output_limit = 0.0f;  // 输出限幅; 电流环运行期按 Vbus 动态覆盖
    float ramp_rate    = 0.0f;  // 斜坡率 (保留)
};

/*
 * 弱磁配置占位:
 *   仅提供参数边界与平台配置入口, 当前版本尚未接入执行链。
 *
 * 预期执行位置:
 *   速度/转矩目标 -> Iq/Id 参考生成 -> 弱磁调节 -> d/q 电流环 -> 电压矢量限幅 -> SVPWM
 */
struct MotorFieldWeakeningParam
{
    bool  enable = false;                 // 总开关, false=完全关闭弱磁逻辑
    float enter_speed_rpm = 0.0f;         // 进入弱磁的转速阈值 (RPM)
    float exit_speed_rpm  = 0.0f;         // 退出弱磁的回差阈值 (RPM)
    float target_voltage_ratio = 0.95f;   // 目标电压利用率, 相对线性调制上限
    float max_negative_id_a = 0.0f;       // 允许施加的最大负 Id 幅值 (A)
    float current_reserve_ratio = 0.10f;  // 给 Iq 保留的电流占比, 防止全部电流都被负 Id 吃掉
    float kp = 0.0f;                      // 弱磁电压调节比例项 (预留)
    float ki = 0.0f;                      // 弱磁电压调节积分项 (预留)
};

/*
 * MotorFeedforwardParam -- Phase 2 前馈通道运行期开关
 *
 * BuildCfg MOTOR_BUILD_ENABLE_VELOCITY_FEEDFORWARD /
 *          MOTOR_BUILD_ENABLE_TORQUE_FEEDFORWARD 与本结构双层闸控:
 *   - BuildCfg 关 -> 前馈消费代码根本不编进固件 (ctx_ 字段也不占 RAM)
 *   - BuildCfg 开 + 本结构 enable=false -> 代码编译进固件, 运行期忽略 MotionSetpoint 中的 ff 字段
 *   - BuildCfg 开 + enable=true -> 实际叠加 ff 到控制环
 *
 * 多实例固件可为每个电机实例独立决定是否启用 ff。
 */
struct MotorFeedforwardParam
{
    bool  enable_vel_ff    = false;  // 速度前馈运行期开关
    bool  enable_torque_ff = false;  // 力矩前馈运行期开关 (Phase 3)
    float vel_ff_gain      = 1.0f;   // 速度前馈增益
    float torque_ff_gain   = 1.0f;   // 力矩前馈增益 (Phase 3)
};

#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
/*
 * MotorImpedanceParam -- Phase 3 阻抗控制默认值 (Phase 2 仅占位)
 *
 * MotionSetpoint 每次 writeSetpoint 可覆盖这些值; 此处仅给默认基线,
 * 用于 setRunPolicy 切到 IMPEDANCE_CONTROL 时的初始化。
 */
struct MotorImpedanceParam
{
    float default_stiffness   = 0.0f;  // 弹簧刚度默认 N*m/rad
    float default_damping     = 0.0f;  // 阻尼默认 N*m/(rad/s)
    float torque_bias_default = 0.0f;  // 偏置力矩默认 (A), 用于重力补偿等稳态前馈
    float max_torque_a        = 0.0f;  // 阻抗输出 Iq 钳位 (A), 0 = 不额外限幅
};
#endif

struct MotorControlParam
{
    float control_freq_hz = 10000.0f;  // FOC 控制频率 (Hz), 通常 = PWM 频率
    int8_t command_direction = 1;      // +1=API 正方向按内部方向; -1=API 正方向反向

    PIDParam current_d;       // D 轴电流环 PID
    PIDParam current_q;       // Q 轴电流环 PID
    PIDParam speed;           // 速度环 PID (输出为 Iq 目标)
    PIDParam position;        // 位置环 PID (输出为速度目标)
    MotorFieldWeakeningParam field_weakening; // 弱磁参数占位, 当前未接入执行链
    MotorFeedforwardParam   feedforward;     // 前馈通道运行期开关 (Phase 2)

    /* === 电流环 PI 增益自动推导 (本轮接入) ===
     * auto_derive_current_pid=true 且 R/L 已知时, init() 用以下两字段覆盖 current_d/current_q 的 kp/ki:
     *   kp = L * (2πf_bw)
     *   ki = R * (2πf_bw)
     * 公式见 MotorPIAutoTune::deriveCurrentLoopGains
     *
     * ConfigCheck 校验 f_bw < control_freq_hz/10 (避免离散化失稳)。
     * R/L 未知时 init 使用 current_d/current_q 的手填 fallback, RL 辨识完成后再覆盖。
     * current_d/current_q.output_limit 仅作初始化兜底,
     * 运行期统一按实时 Vbus 计算电流环可用电压。
    */
    bool  auto_derive_current_pid  = true;
    float current_loop_bandwidth_hz  = 100.0f;     // 期望电流环带宽 (Hz), 首次自动推导用保守值
    float current_loop_damping_ratio = 0.707f;     // 保留字段: 二阶阻尼比占位, 当前一阶整定不参与 kp/ki 计算

    /* === 速度环 PI 增益自动推导 (本轮仅预留字段, ConfigCheck 拒绝 auto_derive_speed_pid=true) ===
     * P5 阶段实现 deriveSpeedLoopGains 时启用
     * 推导需要 cfg.physical.load_inertia > 0 且 torque_constant > 0;
     * 当前 =0 -> ConfigCheck 拒绝 auto_derive_speed_pid=true.
     */
    bool  auto_derive_speed_pid   = false;
    float speed_loop_bandwidth_hz  = 100.0f;
    float speed_loop_damping_ratio = 0.707f;

    /* === [P4] 位置环 PI 增益自动推导 (本轮仅预留字段, ConfigCheck 拒 auto_derive_position_pid=true) ===
     * P5 阶段实现 derivePositionLoopGains 时启用;
     * 推导需 cfg.physical.load_inertia > 0 与 torque_constant > 0;
     * 当前 =0 → ConfigCheck 拒绝 auto_derive_position_pid=true.
     */
    bool  auto_derive_position_pid   = false;
    float position_loop_bandwidth_hz  = 100.0f;
    float position_loop_damping_ratio = 0.707f;

    ControlMethod    default_control_method = ControlMethod::FOC_SPEED;
    ModulationMethod modulation            = ModulationMethod::SVPWM;
    /*
     * start() 默认进入的闭环模式。
     * 由 Platform 层明确填写, 以后只在调试模式切换时再做检查。
     *
     * 可选取值:
     *   NONE              上电初始化中, 不允许直接 start()
     *   TORQUE_CONTROL    默认转矩/力矩模式
     *   VELOCITY_CONTROL  默认速度模式
     *   POSITION_CONTROL  默认位置模式 (Phase 2 起接通)
     *   IMPEDANCE_CONTROL 默认阻抗模式 (Phase 3 起接通, BuildCfg 启用时有效)
     */
    Mode             startup_target_mode   = Mode::NONE;
};

/* =========================================================
 * [6] 运动目标与闭环限幅
 *
 *     这里是控制目标的软限制, 不等同于硬故障阈值
 *       - limit.max_phase_current_a / max_bus_current_a 是软件过流保护
 *       - motion.max_iq_ref_a 是正常控制允许给电流环的 Iq 上限
 * ========================================================= */
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
struct MotorStallProtectionParam
{
    bool enable = false;

    float speed_abs_rpm      = 30.0f;  // 低于该速度且长期满 Iq 可认为堵转候选
    float speed_error_rpm    = 100.0f; // 速度误差超过该值才检查堵转
    float speed_error_hysteresis_rpm = 20.0f; // 候选退出比进入更宽松 (RPM), 减少边界抖动
    float iq_limit_ratio     = 0.90f;  // |Iq_ref| 超过 Iq 限幅多少比例才认为电流打满
    float confirm_time_s     = 0.5f;   // 候选持续多久后置 motor_stalled

    /*
     * 堵转已确认后, 允许速度环先冻结积分并逐步衰减, 避免负载突然消失时暴冲。
     * hold_integral 由 Manager 在运行时控制, 这里仅配置衰减系数。
     */
    float speed_integral_decay = 0.98f; // 堵转时速度环积分衰减系数 [0,1]
    bool  latch_fault          = false; // true=确认堵转后锁 Fault::STALL, 需 clearFault()

    /*
     * 堵转恢复判据:
     *   release_speed_rpm       转速回到该值以上后, 才允许清除 motor_stalled 标志
     *   release_confirm_time_s  恢复状态需持续多久, 防止在边界速度附近抖动
     */
    float release_speed_rpm       = 80.0f;
    float release_confirm_time_s  = 0.10f;

    /*
     * 堵转累计计数:
     *   true  = 每次确认堵转时累计一次, 由 init/reset 清零
     *   false = 不累计, 只保留当前 motor_stalled 事件位
     */
    bool count_total_events = false;

    /*
     * 自动重试策略:
     *   auto_restart=false 时:
     *     - latch_fault=true  -> 进 ERROR, 用户 clearFault() 后再启动
     *     - latch_fault=false -> 仅置堵转事件位, 便于上层决定是否停机
     *
     *   auto_restart=true 时:
     *     - restart_count         最大自动重试次数
     *     - restart_interval_s    两次重试之间的等待时间
     *     - restart_stop_mode     堵转后先用哪种方式停住再重试
     *     - clear_fault_before_restart  若未来引入可恢复故障, 是否在重试前清掉故障位
     *     - require_manual_clear_after_retry_exhausted  重试耗尽后是否锁人工恢复故障
     */
    bool  auto_restart         = false;
    uint8_t restart_count      = 0;
    float restart_interval_s   = 1.0f;
    StopMode restart_stop_mode = StopMode::COAST;
    bool  clear_fault_before_restart = true;
    bool  require_manual_clear_after_retry_exhausted = true;
};
#endif

struct MotorMotionParam
{
    float max_iq_ref_a = 0.0f;              // 0=使用 physical.rated_current 作为正常 Iq 上限
    float iq_slew_rate_a_per_s      = 0.0f;  // 驱动方向 Iq 斜率 (A/s), 0=关闭
    float iq_slew_rate_brake_a_per_s = 0.0f; // 制动方向 (反向) Iq 斜率 (A/s), 0=回退到 iq_slew_rate_a_per_s
    float speed_slew_rate_rpm_s = 0.0f;     // 0=关闭速度目标斜率限制
    float torque_speed_guard_band_rpm = 100.0f; // 接近限速时削减同向 Iq 的过渡带 (本轮扩到所有模式)
    float overspeed_factor = 1.10f;         // 超速故障系数, |ω| > max_speed_rpm × 该值 -> Fault::OVERSPEED
    float reverse_zero_band_rpm = 50.0f;    // 反转跨零软减速门限 (RPM), |ω| 高于该值来反向指令时先减速到 0

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    MotorStallProtectionParam stall;
#endif

#if LIB_MOTOR_ENABLE_STREAM_HOLD_WHEN_IDLE
    /* hold_timeout_ticks: 上层断流超过该 tick 数后 ACTIVE -> HELD (位置冻结保持)。
     *   0 = 永不切到 HELD (即关闭流持仓功能)
     *   机器狗无线丢包典型值, 5~20 ms 折算成 tick 数 (如 1kHz tick 设为 20)
     *   打印机断流安全值, 100~500 tick
     */
    uint32_t hold_timeout_ticks = 0;
#endif
};

/* =========================================================
 * [7] 制动/能量策略预留
 *
 *     Platform/BSP 只声明硬件动作能力, 这里表达用户策略偏好。
 *     第一版只保留配置边界, 完整 BrakeManager/EnergyManager 后续实现。
 * ========================================================= */
struct MotorBrakeParam
{
    bool allow_foc_negative_iq = false;
    bool allow_low_side_brake  = true;
    bool allow_mechanical_brake = false;
    bool allow_coast           = true;
    bool has_brake_resistor    = false;
    bool has_battery_regen_path = false;
    bool control_brake_resistor_chopper = false;

    float max_brake_iq_a = 0.0f;       // 0=跟随 motion.max_iq_ref_a
    float max_regen_iq_a = 0.0f;       // 0=跟随 max_brake_iq_a, 仅约束电池回充路径
    float max_dump_iq_a = 0.0f;        // 0=跟随 max_brake_iq_a, 仅约束电阻泄放路径
    float max_regen_power_w = 0.0f;    // 0=未声明静态功率边界, 由上层 budget 或硬件自限
    float max_dump_power_w = 0.0f;     // 0=未声明静态功率边界, 由上层 budget 或硬件自限
    float normal_decel_rpm_s = 0.0f;   // 0=暂不启用库内减速规划
    float fast_decel_rpm_s   = 0.0f;
    BrakeEnergyPathPolicy energy_path_policy = BrakeEnergyPathPolicy::REGEN_FIRST;

    // 上层 setEnergyBudget 租约超时后的回退策略:
    //   BRAKE_ON_LOSS/BLOCK_ON_LOSS: 禁止运行期负 Iq, 等待新 budget
    //   HOLD_ON_LOSS/FALLBACK_TO_STATIC: 回退到静态 cfg.has_*_path 与静态限额
    BudgetLossBehavior budget_loss_behavior = BudgetLossBehavior::BRAKE_ON_LOSS;
};

/*
 * MotorEnergyBudget -- 上层 (BMS/VCU/运动算法) 周期性注入的能量预算
 *
 * 由 MotorAPI::setEnergyBudget(b) 写入:
 *   - lib 内部按当前 fsm_timer_ticks 把 valid_for_us 转为 tick 计时租约 (与上层 loop 漂移无关)
 *   - 仅在 EnergyFSM::limitIqReference 中产生反向 Iq 检查时被消费
 *   - 驱动 Iq 不读 budget; budget 只约束反向 Iq 的能量承接路径档位
 *   - 超时后按 cfg.brake.budget_loss_behavior 决定阻断负 Iq 或回退静态 cfg
 */
struct MotorEnergyBudget
{
    uint32_t timestamp_us = 0U;
    uint32_t valid_for_us = 0U;       // 租约时长 (us); 内部转 ticks 与 ISR 时钟节拍一致

    bool regen_allowed = false;      // 当前允许回充电池
    bool dump_allowed = false;       // 当前允许泄放电阻接手
    bool brake_derated = false;      // 当前为减速预算被优先衰减 (BMS 过充预热阶段等)
    bool source_fault = false;       // BMS/外源侧主动声明不可用

    float vbus_v = 0.0f;             // 上层观测到的母线电压 (参考; lib 仍以自身采样 v_bus 为准)
    float vbus_limit_v = 0.0f;       // 上层期望的母线电压上限 (0 = 用 cfg.limit.over_voltage_v)
    BrakeEnergyPathPolicy energy_path_policy = BrakeEnergyPathPolicy::REGEN_FIRST;

    float max_regen_iq_a = 0.0f;            // 动态回充 Iq 上限; 0=不允许 regen 路径
    float max_dump_iq_a = 0.0f;             // 动态泄放 Iq 上限; 0=不允许 dump 路径
    float max_regen_dc_current_a = 0.0f;    // BMS 允许的 DC 回充电流边界, 不直接换算为 Iq
    float max_dump_current_a = 0.0f;        // 制动电阻/斩波支路电流边界, 不直接换算为 Iq
    float max_regen_power_w = 0.0f;         // 能量侧功率边界, 供上层/HAL 预留, EnergyFSM 不直接换算
    float max_dump_power_w = 0.0f;          // 能量侧功率边界, 供上层/HAL 预留, EnergyFSM 不直接换算

    float regen_dc_current_limit_a = 0.0f;  // Deprecated: use max_regen_dc_current_a
    float dump_current_limit_a = 0.0f;      // Deprecated: use max_dump_current_a
    float max_brake_power_w = 0.0f;         // Deprecated: use max_regen_power_w/max_dump_power_w
    float resistor_temp_c = 0.0f;           // 制动电阻当前温度 (°C)
    float resistor_temp_limit_c = 0.0f;     // 制动电阻温度上限 (°C)
    // 动态 budget 的 max_*_iq_a 是控制环直接钳位量; DC/功率字段不自动换算为 Iq。
};

/* =========================================================
 * [9] 观测器参数
 *     启动源 / 稳态源的默认选择不再放在此结构体里重复配置。
 *     运行策略由 MotorRunPolicy 表达:
 *       - 编译期默认值来自 Platform/Motor/BuildCfg
 *       - 未来运行时切换也通过 MotorRunPolicy API 进入
 *
 *     这里仅保留各观测路径本身需要的参数边界。
 * ========================================================= */
struct SmoValidityParam
{
    float acquire_min_speed_rpm = 500.0f;       // 首次确认有效的最低机械转速
    float acquire_min_signal_level = 0.0f;      // 首次确认有效的最小反电势幅值
    float acquire_max_pll_error_rad = 0.35f;    // 首次确认有效的最大 PLL 误差
    uint16_t acquire_ticks = 100U;               // 严格条件连续成立的 tick 数

    float release_min_speed_rpm = 400.0f;        // 已有效后允许保持的最低机械转速
    float release_min_signal_level = 0.0f;       // 已有效后允许保持的最小反电势幅值
    float release_max_pll_error_rad = 0.60f;     // 已有效后允许保持的最大 PLL 误差
    uint16_t release_ticks = 100U;               // 宽松条件连续失败后才撤销有效
};

struct SmoHandoverParam
{
    float min_speed_rpm = 550.0f;                // 启动源交给 SMO 的最低机械转速
    float max_angle_error_rad = 0.35f;           // 两角度源交接前的最大相位差
    float max_speed_error_rpm = 100.0f;          // 两角度源交接前的最大速度差
    uint16_t confirm_ticks = 100U;                // 交接条件连续成立的 tick 数
    float blend_time_s = 0.05f;                  // 角度与速度平滑融合时间
};

struct MotorObserverParam
{
    /* ==================== [A1] IF 启动参数 ==================== */
    const MotorIFStartupProfile* if_startup_profile = nullptr; // IF 对齐与拖动的必选分段启动曲线

    HfiInjectionMode hfi_injection_mode = HfiInjectionMode::SINE_CARRIER;

    /* ==================== [A2] HFI 启动/低速参数 (暂未实现) ==================== */
    float hfi_injection_voltage_ratio = 0.10f; // 高频注入电压占实时母线电压的比例
    float hfi_pll_kp               = 5.0f;    // HFI 角度跟踪比例项
    float hfi_pll_ki               = 100.0f;  // HFI 角度跟踪积分项
    float hfi_pll_integral_limit_rad_s = 0.0f; // HFI PLL integral clamp, 0 disables.
    float hfi_pll_speed_limit_rad_s    = 0.0f; // HFI PLL speed output clamp, 0 disables.
    float hfi_prepare_speed_rpm     = 650.0f;  // HFI 提前启动/预热速度阈值 (RPM)
    float hfi_disable_speed_rpm     = 800.0f;  // SMO 有效后允许关闭 HFI 的速度阈值 (RPM)
    float hfi_speed_predict_time_s  = 0.02f;   // 低速风险预测前视时间 (s)
    float hfi_voltage_ramp_time_s   = 0.02f;   // HFI 注入幅值斜坡时间 (s)
    float hfi_min_signal_level_a    = 0.0f;    // HFI 有效所需最小解调电流幅值 (A)
    uint16_t hfi_lock_ticks         = 100;     // HFI 连续有效确认 tick 数

    /* ==================== [B1] HFI 方波/正弦注入参数 ==================== */
    /* 方波交变注入参数。*/
    float hfi_square_current_scale    = 0.25f;   // Square-wave demod current extraction scale.
    float hfi_square_lpf_cutoff_ratio = 0.025f;  // Square demod LPF cutoff ratio.

    /* 正弦载波注入参数。*/
    float hfi_sine_injection_freq_hz = 1000.0f; // 正弦载波频率 (Hz)
    float hfi_sine_lpf_cutoff_ratio  = 0.25f;   // 解调低通截止频率相对载波频率的比例

    SensorFaultAction sensor_fault_action = SensorFaultAction::STOP;

    /* ==================== [B2] SMO 稳态参数 ==================== */
    float smo_gain = 1.0f;   // 滑模增益 (越大收敛越快, 但噪声越大)
    float smo_pll_kp   = 2.0f;   // PLL 比例增益 (跟踪反电动势角度)
    float smo_pll_ki   = 50.0f;  // PLL 积分增益
    float smo_bemf_lpf_cutoff_hz = 1000.0f; // SMO 从滑模注入量提取反电势的一阶低通截止频率 (Hz)
    SmoValidityParam smo_validity;         // SMO 自身锁定/失锁判据，与启动源无关
    SmoHandoverParam smo_handover;         // 任意低速启动源交给 SMO 的统一判据
    bool allow_smo_closed_loop = false;   // 允许 SMO 角度真正参与闭环前必须显式打开。

    /* ==================== [C1] 运行中角度源自动降级参数 ====================
     * 首次启动及降级后的再次交接均统一使用 smo_handover；本组只负责稳态低速降级。
     * enable_auto_swap=false 时维持旧的 valid=false 即 Fault 老语义。
     * hfi_to_smo_rpm 仅保留给运行期策略扩展和 Ke 高速确认，不参与当前交接门控。
     */
    float smo_to_hfi_rpm        = 500.0f;  // 高速观测器减速到该值以下主动降级到低速角度源
    float hfi_to_smo_rpm        = 800.0f;  // 运行期策略扩展与 Ke 高速确认的预留阈值
    float switch_hysteresis_rpm = 80.0f;   // 切换阈值迟滞 (RPM), 防边界抖动
    float switch_grace_time_s   = 0.2f;    // 切回失败的超时窗口 (s), 超时置 Fault::OBSERVER_LOSS
    bool  enable_auto_swap      = false;    // 默认关闭, 观测器切换策略调试稳定后再开

    /* ==================== [C2] 启动前转子静止保证机制 ====================
     * 任意非 SENSOR 启动且未启用 PHASE_VOLTAGE 顺逆风启动时, ConfigCheck 要求选择一种静止保证机制。
     * NONE 以外的机制由 BuildCfg/HAL/显式承诺决定是否可用。
     * 失败时返回 IF_STATIONARY_GUARANTEE_REQUIRED (见 Motor_Definitions.h)。
     * [P3.B] 启动前转子静止保证 (与启动源解耦, 不是 IF 专属)
     * 任何非 SENSOR 启动 + flying_start_mode != PHASE_VOLTAGE 必须配 != NONE,
     * 否则 ConfigCheck 返回 IF_STATIONARY_GUARANTEE_REQUIRED (detail 码名暂保留).
     */
    RotorStationaryGuarantee rotor_stationary_guarantee        = RotorStationaryGuarantee::NONE;
    bool                    rotor_explicit_bypass_confirmed = false; // 选 EXPLICIT_BYPASS 时必须置 true

    FlyingStartMode flying_start_mode = FlyingStartMode::DISABLED; // 顺逆风启动策略, 与默认运行角度源解耦

    /* [P3.B] 顺逆风启动 start() 自动检测速度阈值 (机械 RPM)
     * 仅 flying_start_mode=PHASE_VOLTAGE 时 start() 第一个 tick 检查 |w| > 该值才走顺逆风启动分支;
     * 低于该值走标准 ALIGNMENT 流程 (反电势太弱 PLL 锁不上)。
     */
    float flying_start_speed_threshold_rpm = 100.0f;

};
/* =========================================================
 * [10] 运行策略
 *
 *     与参数配置分离:
 *       - MotorObserverParam 只描述算法参数
 *       - MotorRunPolicy 描述本次运行使用哪条反馈链
 *
 *     当前版本默认策略由 Platform BuildCfg 固定;
 *     未来通用测试固件可在 STOP 状态切换 active policy。
 * ========================================================= */
struct MotorRunPolicy
{
    StartupSource     startup_source = StartupSource::IF;
    SteadyAngleSource steady_source  = SteadyAngleSource::SMO;
    bool    startup_auto_restart = false;       // 启动交接失败后是否自动重试
    uint8_t startup_max_retry_count = 0U;       // 首次启动之外允许的额外尝试次数
    float   startup_restart_interval_s = 0.0f;  // COAST 停机后的重试等待时间
};

/* =========================================================
 * [11] 参数辨识约束
 *
 *     这些字段只约束 RL/后续辨识流程的目标电流、注入上限和重复采样次数。
 *     它们不是电机参数结果; 辨识结果仍写入 physical.*_identified。
 * ========================================================= */
struct MotorIdentifyParam
{
    float rl_target_current_a = 0.50f;        // RL 辨识目标电流 (A), 库内部按实测电流自动搜索注入电压
    float rl_injection_voltage_limit_v = 0.20f; // RL 注入电压幅值上限 (V), 首次接新电机建议用保守低值
    uint8_t rl_repeat_count = 3U;              // 锁定 Vstep 后重复采样轮数, 多轮平均提升稳定性
};

struct MotorHardwareConfig
{
    const MotorHAL_t* hal = nullptr;

    MotorPhysicalParam physical;
    MotorLimitParam    limit;
    MotorSensorParam   sensor;
    MotorPositionParam position;
};

struct MotorAlgorithmParam
{
    MotorObserverParam observer;
    MotorControlParam  control;
    MotorMotionParam   motion;
    MotorBrakeParam    brake;
    MotorIdentifyParam identify;
};

/* =========================================================
 * [Total] 总配置入口 -- Platform 层唯一需要填写的结构体
 *     hal: BSP 层返回的硬件接口指针 (必须非空)
 * ========================================================= */
struct MotorConfig
{
    const MotorHAL_t*  hal = nullptr;

    MotorPhysicalParam physical;   // [1] 电机物理参数
    MotorLimitParam    limit;      // [2] 保护限制
    MotorSensorParam   sensor;     // [3] 传感器与采样
    MotorPositionParam position;   // [4] 外部位置传感器硬件
    MotorObserverParam observer;   // [5] 控制反馈估计源
    MotorControlParam  control;    // [6] PID + 控制模式
    MotorMotionParam   motion;     // [7] 目标限幅/斜率/堵转策略
    MotorBrakeParam    brake;      // [8] 制动策略偏好 (预留)
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
    MotorImpedanceParam impedance;  // [9] 阻抗控制默认值 (Phase 3, BuildCfg 启用时占位)
#endif
    MotorRunPolicy     default_run_policy; // [10] 默认运行策略
    MotorIdentifyParam  identify;           // [11] 参数辨识约束

    /* [12] 运行期学习参数的持久化策略
     * persistent_enabled: 是否启用 learned 参数持久化框架; false 时完全不调用 HAL flash 接口。
     * persistent_load_on_init: init 时是否尝试从 flash 载入 learned 参数到 *_identified。
     * persistent_flash_addr: 项目指定的 flash 记录起始地址, 必须由平台避开程序区。
     * persistent_flash_signature: 用于确认 flash 记录属于当前电机/固件配置。
     *
     * 当前若 HAL flash_read/write/erase 为 nullptr, 即使打开开关也不能自动保存/加载。
     * 未接入 flash 前, RL/Ke 辨识结果需要用户确认后人工回填到 *_learned_cfg 或 *_measured。
     */
    bool    persistent_enabled            = false;
    bool    persistent_load_on_init       = false;
    uint32_t persistent_flash_addr        = 0;
    uint32_t persistent_flash_signature   = 0;
};

inline MotorHardwareConfig MakeMotorHardwareConfig(const MotorConfig& cfg)
{
    MotorHardwareConfig out;
    out.hal      = cfg.hal;
    out.physical = cfg.physical;
    out.limit    = cfg.limit;
    out.sensor   = cfg.sensor;
    out.position = cfg.position;
    return out;
}

inline MotorAlgorithmParam MakeMotorAlgorithmParam(const MotorConfig& cfg)
{
    MotorAlgorithmParam out;
    out.observer = cfg.observer;
    out.control  = cfg.control;
    out.motion   = cfg.motion;
    out.brake    = cfg.brake;
    out.identify = cfg.identify;
    return out;
}

} // namespace Lib_Motor
