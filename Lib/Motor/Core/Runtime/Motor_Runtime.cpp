#include "../Manager/Motor_Manager.h"

#include "../FSM/Motor_FSM.h"

#include "../Limit/Motor_TargetLimiter.h"

#include "../../Control/Utils/FocMath.h"

#include "../../Control/Modulation_SVPWM.h"
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
#include "../Numeric/Motor_FixedNumeric.h"
#endif

#include <cmath>

namespace Lib_Motor
{

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
namespace
{

std::int32_t positiveFloatToInt32(float value)
{
    if (!std::isfinite(value) || value <= 0.0f)
    {
        return 0;
    }
    if (value > 2147483520.0f)
    {
        return 2147483647;
    }
    return static_cast<std::int32_t>(value + 0.5f);
}

std::int32_t positiveFloatToQ15PerCount(float value)
{
    if (!std::isfinite(value) || value <= 0.0f)
    {
        return 0;
    }
    /*
     * per-count 系数用于 ISR 内的 delta * q15_per_count。
     * 正数至少保留 1LSB, 上限钳到 32767，保证 16bit ADC delta 乘法不溢出 int32。
     */
    if (value < 1.0f)
    {
        return 1;
    }
    if (value >= 32767.0f)
    {
        return 32767;
    }
    return static_cast<std::int32_t>(value + 0.5f);
}

std::uint32_t positiveFloatToUInt32(float value, std::uint32_t fallback)
{
    if (!std::isfinite(value) || value <= 0.0f)
    {
        return fallback;
    }
    if (value > 4294967040.0f)
    {
        return 4294967295UL;
    }
    return static_cast<std::uint32_t>(value + 0.5f);
}

float phaseCurrentScalePerCount(const MotorConfig& cfg)
{
    const float adc_res = cfg.sensor.adc_resolution;
    const float v_ref = cfg.sensor.adc_v_ref;
    if (cfg.sensor.current_sensor_type == CurrentSensorType::HALL_SENSOR)
    {
        return 1000.0f * v_ref /
               (adc_res * cfg.sensor.hall_sensitivity_mV_per_A);
    }
    return v_ref /
           (adc_res * cfg.sensor.phase_shunt_resistor * cfg.sensor.phase_amp_gain);
}

float busCurrentScalePerCount(const MotorConfig& cfg)
{
    const float adc_res = cfg.sensor.adc_resolution;
    const float v_ref = cfg.sensor.adc_v_ref;
    return v_ref /
           (adc_res * cfg.sensor.bus_shunt_resistor * cfg.sensor.bus_amp_gain);
}

float busVoltageScalePerCount(const MotorConfig& cfg)
{
    const float adc_res = cfg.sensor.adc_resolution;
    const float v_ref = cfg.sensor.adc_v_ref;
    const float divider_ratio =
        (cfg.sensor.vbus_r_up + cfg.sensor.vbus_r_down) /
        cfg.sensor.vbus_r_down;
    return v_ref / adc_res * divider_ratio;
}

#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
float phaseVoltageScalePerCount(const MotorConfig& cfg)
{
    const float adc_res = cfg.sensor.adc_resolution;
    const float v_ref = cfg.sensor.adc_v_ref;
    const float divider_ratio =
        (cfg.sensor.vphase_r_up + cfg.sensor.vphase_r_down) /
        cfg.sensor.vphase_r_down;
    return v_ref / adc_res * divider_ratio;
}
#endif

FixedNumeric::q15_t modulationKmodQ15(ModulationMethod method)
{
    switch (method)
    {
        case ModulationMethod::SPWM:
            return FixedNumeric::kQ15Half;
        case ModulationMethod::SVPWM:
        case ModulationMethod::THIPWM:
            return 18919; // 1 / sqrt(3)
        case ModulationMethod::SIXSTEP:
            return FixedNumeric::kQ15One;
        default:
            return 18919;
    }
}

} // namespace
#endif

void MotorManager::updateCurrentPidVoltageLimit(float omega_elec_rad_s)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    (void)omega_elec_rad_s;
    refreshFixedCurrentPidOutputLimit();
#else
    (void)omega_elec_rad_s;

    const float k_mod = MotorTargetLimiter::modulationKmod(config_.control.modulation);
    const float vbus = runtimeBusVoltageToPhysical(ctx_, ctx_.v_bus);
    float v_available = k_mod * vbus;

    if (!std::isfinite(v_available) || v_available < 0.0f)
    {
        v_available = 0.0f;
    }

    const float pid_limit = (v_available > 1.0e-6f) ? v_available : 1.0e-6f;
    controller_.setCurrentOutputLimit(pid_limit);
#endif
}

#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
void MotorManager::configureFixedRuntimeScales()
{
    const float current_base = MotorControl::currentBase(config_);
    const float speed_base = MotorControl::speedBase(config_);
    const float bus_voltage_base =
        (config_.limit.over_voltage_v > 1.0f) ? config_.limit.over_voltage_v : 1.0f;
    const float phase_current_scale = phaseCurrentScalePerCount(config_);
    const float vbus_scale = busVoltageScalePerCount(config_);

    ctx_.current_base_mA = positiveFloatToUInt32(current_base * 1000.0f, 1000U);
    ctx_.bus_voltage_base_mV =
        positiveFloatToUInt32(bus_voltage_base * 1000.0f, 1000U);
    ctx_.voltage_base_mV = ctx_.bus_voltage_base_mV;
    ctx_.speed_base_rpm_x10 =
        positiveFloatToUInt32(speed_base * 10.0f, 10U);
    ctx_.phase_current_q15_per_count =
        positiveFloatToQ15PerCount((phase_current_scale / current_base) * 32768.0f);
    ctx_.vbus_q15_per_count =
        positiveFloatToQ15PerCount((vbus_scale / bus_voltage_base) * 32768.0f);
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    const float phase_voltage_scale = phaseVoltageScalePerCount(config_);
    ctx_.phase_voltage_q15_per_count =
        positiveFloatToQ15PerCount((phase_voltage_scale / bus_voltage_base) * 32768.0f);
#endif
#if MOTOR_BUILD_HAS_BUS_CURRENT
    const float bus_current_scale = busCurrentScalePerCount(config_);
    ctx_.bus_current_q15_per_count =
        positiveFloatToQ15PerCount((bus_current_scale / current_base) * 32768.0f);
    ctx_.bus_overcurrent_delta_counts =
        positiveFloatToInt32(config_.limit.max_bus_current_a / bus_current_scale);
#else
    ctx_.bus_current_q15_per_count = 0;
    ctx_.bus_overcurrent_delta_counts = 0;
#endif
    ctx_.phase_overcurrent_delta_counts =
        positiveFloatToInt32(config_.limit.max_phase_current_a / phase_current_scale);
    ctx_.vbus_overvoltage_counts =
        positiveFloatToInt32(config_.limit.over_voltage_v / vbus_scale);
    ctx_.vbus_undervoltage_counts =
        positiveFloatToInt32(config_.limit.under_voltage_v / vbus_scale);
    ctx_.phase_overcurrent_q15 = FixedNumeric::absoluteQ15(
        runtimeCurrentFromPhysical(ctx_, config_.limit.max_phase_current_a));
#if MOTOR_BUILD_HAS_BUS_CURRENT
    ctx_.bus_overcurrent_q15 = FixedNumeric::absoluteQ15(
        runtimeCurrentFromPhysical(ctx_, config_.limit.max_bus_current_a));
#else
    ctx_.bus_overcurrent_q15 = 0;
#endif
    ctx_.current_ref_limit_q15 = FixedNumeric::absoluteQ15(
        runtimeCurrentFromPhysical(ctx_, MotorTargetLimiter::configuredIqLimit(config_)));
    ctx_.vbus_overvoltage_q15 =
        runtimeBusVoltageFromPhysical(ctx_, config_.limit.over_voltage_v);
    ctx_.vbus_undervoltage_q15 =
        runtimeBusVoltageFromPhysical(ctx_, config_.limit.under_voltage_v);
    ctx_.vbus_default_q15 =
        runtimeBusVoltageFromPhysical(ctx_, config_.limit.over_voltage_v * 0.5f);
    ctx_.max_duty = runtimeClampDuty(
        FixedNumeric::fromNormalized(config_.limit.max_duty_cycle),
        FixedNumeric::kQ15One);
    ctx_.v_bus = ctx_.vbus_default_q15;
    ctx_.current_pid_modulation_limit_q15 =
        modulationKmodQ15(config_.control.modulation);
    ctx_.current_pid_output_limit_q15 =
        FixedNumeric::multiplyQ15(ctx_.v_bus,
                                  ctx_.current_pid_modulation_limit_q15);
#if LIB_MOTOR_ENABLE_IF_STARTUP
    ctx_.if_switch_up_speed_q15 = FixedNumeric::absoluteQ15(
        runtimeSpeedFromPhysical(
            ctx_,
            config_.observer.hfi_to_smo_startup_rpm +
            config_.observer.hfi_to_smo_startup_hysteresis_rpm));
#endif
}

void MotorManager::refreshFixedRuntimeOffsetCounts()
{
    /* q15 后端 offset 直接以 ADC counts 保存; 本函数保留用于旧调用点兼容。 */
}

void MotorManager::configureFixedCurrentPidRuntime(bool reset_integrator)
{
    controller_.configureFixedCurrentPid(runtimeCurrentBaseA(ctx_),
                                         runtimeControlVoltageBaseV(ctx_),
                                         dt_,
                                         reset_integrator);
    refreshFixedCurrentPidOutputLimit();
}

void MotorManager::refreshFixedCurrentPidOutputLimit()
{
    RuntimeVoltage vbus_q15 = ctx_.v_bus;
    if (vbus_q15 < 0)
    {
        vbus_q15 = 0;
    }

    RuntimeVoltage limit_q15 = FixedNumeric::multiplyQ15(
        vbus_q15,
        ctx_.current_pid_modulation_limit_q15);
    if (limit_q15 <= 0)
    {
        limit_q15 = 1;
    }

    ctx_.current_pid_output_limit_q15 = limit_q15;
    controller_.setFixedCurrentOutputLimitQ15(limit_q15);
}

RuntimeCurrent MotorManager::clampCurrentTargetQ15(RuntimeCurrent value) const
{
    const RuntimeCurrent limit = ctx_.current_ref_limit_q15;
    if (limit <= 0)
    {
        return 0;
    }

    const RuntimeCurrent negative_limit =
        FixedNumeric::saturateQ15(-static_cast<std::int32_t>(limit));
    if (value > limit) return limit;
    if (value < negative_limit) return negative_limit;
    return value;
}

void MotorManager::setPwmDutyQ15(const FixedNumeric::DutyAbc& duty)
{
    ctx_.duty_a = runtimeClampDuty(duty.a, ctx_.max_duty);
    ctx_.duty_b = runtimeClampDuty(duty.b, ctx_.max_duty);
    ctx_.duty_c = runtimeClampDuty(duty.c, ctx_.max_duty);
}
#endif

void MotorManager::runObserverLoop()
{
    Motor_FSM::runObserverStep(*this);
}

void MotorManager::syncRuntimeTargets()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    /* q15 基准在 init/config 阶段预计算, tick 内不再刷新 float base。 */
#else
    /* float 后端直接读写 ctx_ 顶层字段, 无需镜像同步。 */
#endif
}

void MotorManager::syncRuntimeInputs(float id_ref, float iq_ref)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    ctx_.target_id = runtimeCurrentFromPhysical(ctx_, id_ref);
    ctx_.target_iq = runtimeCurrentFromPhysical(ctx_, iq_ref);
    ctx_.angle_sin_cos = FixedNumeric::sinCos(ctx_.angle_elec);
#else
    (void)id_ref;
    (void)iq_ref;
    foc_sin_cos_f32(ctx_.angle_elec,
                    ctx_.sin_angle_elec,
                    ctx_.cos_angle_elec);
#endif
}

void MotorManager::publishRuntimeOutputs()
{
    /* 两种后端都已直接写 ctx_ 顶层字段, 这里保留为空兼容调用点。 */
}

#if LIB_MOTOR_ENABLE_SMO_OBSERVER
void MotorManager::resetSmoAngleDirectionLatch()
{
    smo_angle_direction_ = 0;
}

int8_t MotorManager::updateSmoAngleDirectionLatch(RuntimeSpeed direction_hint)
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    RuntimeSpeed candidate = direction_hint;
    RuntimeSpeed deadband =
        runtimeSpeedFromPhysical(ctx_, config_.motion.reverse_zero_band_rpm);
    if (deadband < 0)
    {
        deadband = static_cast<RuntimeSpeed>(-deadband);
    }
    if (deadband == 0)
    {
        deadband = 1;
    }

    if (FixedNumeric::absoluteQ15(candidate) < deadband)
    {
        candidate = ctx_.speed_rpm_observer;
    }
    if (FixedNumeric::absoluteQ15(candidate) < deadband)
    {
        candidate = ctx_.speed_rpm;
    }
    if (FixedNumeric::absoluteQ15(candidate) < deadband)
    {
        candidate = ctx_.target_rpm;
    }
    if (FixedNumeric::absoluteQ15(candidate) >= deadband)
    {
        smo_angle_direction_ =
            (candidate > 0) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    }
#else
    float candidate = runtimeSpeedToPhysical(ctx_, direction_hint);
    float deadband = config_.motion.reverse_zero_band_rpm;
    if (!std::isfinite(deadband) || deadband < 1.0f)
    {
        deadband = 1.0f;
    }

    if (std::fabs(candidate) < deadband)
    {
        candidate = runtimeSpeedToPhysical(ctx_, ctx_.speed_rpm_observer);
    }
    if (std::fabs(candidate) < deadband)
    {
        candidate = runtimeSpeedToPhysical(ctx_, ctx_.speed_rpm);
    }
    if (std::fabs(candidate) < deadband)
    {
        candidate = runtimeSpeedToPhysical(ctx_, ctx_.target_rpm);
    }
    if (std::fabs(candidate) >= deadband)
    {
        smo_angle_direction_ =
            (candidate > 0.0f) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    }
#endif

    return (smo_angle_direction_ != 0) ? smo_angle_direction_ : static_cast<int8_t>(1);
}

RuntimeAngle MotorManager::correctSmoAngleForControl(RuntimeAngle raw_angle,
                                                     RuntimeSpeed direction_hint)
{
    return runtimeCorrectSmoAngleForDirection(
        raw_angle,
        updateSmoAngleDirectionLatch(direction_hint));
}
#endif

void MotorManager::clearControlTargets()
{
    /*
     * 清除所有控制目标值: 清零 target_rpm/target_iq/target_id, 类似 stop 时的复位
     * 通常在 clearFaultAndStop 或 start 后重新进入时调用
     */
    ctx_.target_rpm = 0;
    ctx_.target_iq = 0;
    ctx_.target_id = 0;
    ctx_.pending_reverse_rpm = 0;
#if LIB_MOTOR_ENABLE_SMO_OBSERVER
    resetSmoAngleDirectionLatch();
#endif
    clearDerivedTargets();
    syncRuntimeTargets();
}

void MotorManager::clearDerivedTargets()
{
    /*
     * 清除衍生控制目标 (速度环 PID 中间量, Iq 斜坡限制器状态)
     * 通常在模式切换或停止时调用, 清零 intermediate 值
     */
    ctx_.speed_ref_limited = 0;
    ctx_.speed_pid_iq = 0;
ctx_.iq_ref_command = 0;
    ctx_.iq_ref_limited = 0;
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
    ctx_.target_pos_rad = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
    ctx_.vel_ff_rad_s = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_TORQUE_FEEDFORWARD
    ctx_.torque_ff_a = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
    ctx_.imp_pos_ref_rad    = 0.0f;
    ctx_.imp_stiffness_n_m  = 0.0f;
    ctx_.imp_damping_n_m_s  = 0.0f;
    ctx_.imp_torque_bias_a  = 0.0f;
    ctx_.imp_torque_limit_a = 0.0f;
#endif
#if LIB_MOTOR_ENABLE_HFI
    ctx_.hfi_injection = HfiInjectionCommand();
#endif
    syncRuntimeTargets();
}


/*
 * computeIqReference -- 计算 Iq 电流参考值 (三层级联)
 *
 * 三层级联:
 *   1. 模式计算:
 *      VELOCITY_CONTROL -> 速度 PID 输出 Iq (用户设置目标转速)
 *      TORQUE_CONTROL   -> 直接使用 ctx_.target_iq (由 API 设置)
 *      其他              -> iq_ref = 0 (安全关闭)
 *
 *   2. 限幅:
 *      MotorTargetLimiter::limitIqReference()
 *        (电流硬限 + 速度弱磁 + 功率限制)
 *
 *   3. 斜坡:
 *      MotorTargetLimiter::applyIqSlewRate()
 *        (每 tick 斜率限制, 防止电流突变)
 *
 *   4. 堵转保护 (速度环模式下)
 *
 * 输出: ctx_.iq_ref_command / ctx_.iq_ref_limited 用于调试监控
 */
RuntimeCurrent MotorManager::computeIqReference()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    /*
     * fixed-q15 固件当前只放开 debug PWM/VF/current-lock/plain IF。
     * 速度环、位置环、能量策略和堵转保护尚未完成 q15 版本，BuildCfg 已拦截。
     * 本函数仅保留误入闭环时的安全清零路径，避免把 float 限幅器带入 tick。
     */
    ctx_.speed_ref_limited = 0;
    ctx_.speed_pid_iq = 0;
    ctx_.iq_ref_command = 0;
    ctx_.iq_ref_limited = 0;
    return 0;
#else
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    const float iq_limit = MotorTargetLimiter::configuredIqLimit(config_);  // 最大 Iq 限幅 (从 motion/motor/hard_limit 读取)
#endif

    if (mode_ == Mode::VELOCITY_CONTROL)
    {
        // [A] 速度参考值限幅: 对目标转速施加加速度斜坡限制

        // [A.0] 跨零软减速: 若 pending_reverse_rpm 待执行 & 实测速度已落到 zero_band 内, 提交目标
        if (ctx_.pending_reverse_rpm != 0.0f &&
            fabsf(ctx_.speed_rpm) <= config_.motion.reverse_zero_band_rpm)
        {
            ctx_.target_rpm = ctx_.pending_reverse_rpm;
            ctx_.pending_reverse_rpm = 0.0f;
        }

        ctx_.speed_ref_limited = MotorTargetLimiter::limitSpeedReference(
            config_, dt_, ctx_.speed_ref_limited, ctx_.target_rpm);
        const float speed_error = ctx_.speed_ref_limited - ctx_.speed_rpm;

        // [B] 堵转保持: 堵转状态主动衰减积分项, 防止积分饱和导致恢复时电流冲击
        const bool stall_hold = (event_.motor_stalled != 0U);
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        if (stall_hold)
        {
            controller_.decaySpeedIntegral(config_.motion.stall.speed_integral_decay);
        }
#endif
        // [C] 速度 PID 输出 Iq 参考值
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        controller_.syncFixedSpeedPid(config_, dt_, speed_pid_output_limit);
        ctx_.speed_pid_iq = controller_.updateFixedSpeed(config_, speed_error, stall_hold);
#else
        ctx_.speed_pid_iq = controller_.pid_speed.update(speed_error, dt_, stall_hold);
#endif
        ctx_.iq_ref_command = ctx_.speed_pid_iq;
    }
#if LIB_MOTOR_ENABLE_POSITION_CONTROL
    else if (mode_ == Mode::POSITION_CONTROL)
    {
        // [P1] 位置环: pos_err PID 输出 speed_ref, 叠加速度前馈 vel_ff
        /* TODO HostTest: 位置环方向校正 (pos_ref > θ 时 iq_ref 应为正)、vel_ff 叠加方向、
         *                位置阶跃收敛方向均未编单测, 当前依赖 VOFA 真硬件验证。
         *                调试 PID 前先确认 pos_err 与 iq_ref 极性一致, 方向反了电机会跑飞。 */
        const float pos_err = ctx_.target_pos_rad - ctx_.angle_mech_sensor;
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        controller_.syncFixedPositionPid(config_, dt_, MotorControl::speedBase(config_));
        float speed_ref = controller_.updateFixedPosition(config_, pos_err);
#else
        float speed_ref = controller_.pid_position.update(pos_err, dt_);
#endif
#if LIB_MOTOR_ENABLE_VELOCITY_FEEDFORWARD
        if (config_.control.feedforward.enable_vel_ff)
        {
            speed_ref += config_.control.feedforward.vel_ff_gain * ctx_.vel_ff_rad_s;
        }
#endif
        // [P2] 速度参考限幅 (沿用速度环限幅器, 防 Iq 突变)
        ctx_.speed_ref_limited = MotorTargetLimiter::limitSpeedReference(
            config_, dt_, ctx_.speed_ref_limited, speed_ref);
        const float speed_error = ctx_.speed_ref_limited - ctx_.speed_rpm;

        // [P3] 堵转保持 (与速度环同语义)
        const bool stall_hold = (event_.motor_stalled != 0U);
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        if (stall_hold)
        {
            controller_.decaySpeedIntegral(config_.motion.stall.speed_integral_decay);
        }
#endif
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
        controller_.syncFixedSpeedPid(config_, dt_, speed_pid_output_limit);
        ctx_.speed_pid_iq = controller_.updateFixedSpeed(config_, speed_error, stall_hold);
#else
        ctx_.speed_pid_iq = controller_.pid_speed.update(speed_error, dt_, stall_hold);
#endif
        ctx_.iq_ref_command = ctx_.speed_pid_iq;

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        // 位置环下也清堵转候选计数, 防止位置静止时被误判为堵转
        event_.motor_stalled = 0U;
        stall_candidate_ticks_ = 0U;
        stall_release_ticks_ = 0U;
#endif
    }
#endif
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
    else if (mode_ == Mode::IMPEDANCE_CONTROL)
    {
        /* Phase 3 接通点: 阻抗律
         *   iq_ref = K*(pos_ref-θ) - D*(-ω) + torque_bias + torque_ff
         *   iq_ref = clamp(iq_ref, ±torque_limit)
         * Phase 2 仅占位: BuildCfg 启用 IMPEDANCE 时本段被编译,
         * 启动入口被 supportsMode 拒, 实际不会执行到此处。 */
        ctx_.speed_ref_limited = 0.0f;
        ctx_.speed_pid_iq = 0.0f;
        ctx_.iq_ref_command = 0.0f;
    }
#endif
    else
    {
        // 扭矩/其他模式: Iq 参考值直接使用用户设置的 target_iq
        ctx_.speed_ref_limited = 0.0f;
        ctx_.speed_pid_iq = 0.0f;
        ctx_.iq_ref_command = ctx_.target_iq;
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
        // 非速度环模式下清除堵转状态 (堵转保护仅在速度环下生效)
        if (mode_ != Mode::VELOCITY_CONTROL)
        {
            event_.motor_stalled = 0U;
            stall_candidate_ticks_ = 0U;
            stall_release_ticks_ = 0U;
        }
#endif
    }

    // [D] Iq 限幅: 电流硬限 + 速度弱磁 + 功率限制
    float iq_ref = MotorTargetLimiter::limitIqReference(
        config_, ctx_.speed_rpm, ctx_.iq_ref_command);
    iq_ref = Motor_FSM::limitIqReferenceByEnergy(*this, iq_ref, ctx_.speed_rpm);
    // [E] Iq 斜率限制: 每 tick 限制变化幅度, 防止电流突变
    iq_ref = MotorTargetLimiter::applyIqSlewRate(
        config_, dt_, ctx_.iq_ref_limited, iq_ref);
    ctx_.iq_ref_limited = iq_ref;
    syncRuntimeTargets();

#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    /* 位置环下也跑速度环, 故 stall 检测也生效; IMPEDANCE 不跑速度环不参与 */
    if (mode_ == Mode::VELOCITY_CONTROL ||
        mode_ == Mode::POSITION_CONTROL)
    {
        updateStallProtection(ctx_.speed_ref_limited - ctx_.speed_rpm,
                              ctx_.iq_ref_limited,
                              iq_limit);
    }
#endif

    return ctx_.iq_ref_limited;
#endif
}

void MotorManager::runControlLoop()
{
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    /*
     * fixed-q15 的普通 FOC 闭环尚未完成全路径定点化。
     * 若配置误入这里，保持输出为零并锁定故障，防止软浮点路径混入 ISR。
     */
    controller_.resetCurrentPid();
    ctx_.i_d = 0;
    ctx_.i_q = 0;
    ctx_.v_d = 0;
    ctx_.v_q = 0;
    ctx_.v_alpha = 0;
    ctx_.v_beta = 0;
    setPwmDutyQ15(FixedNumeric::DutyAbc{0, 0, 0});
    setFault(Fault::OBSERVER_UNAVAILABLE);
    publishRuntimeOutputs();
    return;
#else
    float id_ref = runtimeCurrentToPhysical(ctx_, ctx_.target_id);
    float iq_ref = computeIqReference();
    syncRuntimeInputs(id_ref, iq_ref);

    /* ================================================================
     * [1] 预算 sin/cos: Park 与 InvPark 共用同一电角度 angle_elec,
     *     只调用一次联合接口, 消除原先 6 次三角调用 → 2 次。
     *     (中间 [3] 段 PID/HFI 注入只改 v_d/v_q, 不改角度, 故可复用)
     * ================================================================ */
    const float sin_ae = ctx_.sin_angle_elec;
    const float cos_ae = ctx_.cos_angle_elec;

    /* ================================================================
     * [2] Park 变换: alpha/beta (两相) -> dq (旋转)
     *
     *   i_d =  i_alpha * cos(theta) + i_beta * sin(theta)
     *   i_q = -i_alpha * sin(theta) + i_beta * cos(theta)
     *
     * theta = ctx_.angle_elec (电角度, 来自观测器/SMO/传感器)
     * i_alpha/beta = ctx_.i_alpha/beta (来自 updateSensorMeasurements Clarke 变换)
     * ================================================================ */
    park_transform_sc(ctx_.i_alpha, ctx_.i_beta, sin_ae, cos_ae,
                      ctx_.i_d, ctx_.i_q);
    const float omega_elec = ctx_.speed_rpm * config_.physical.pole_pairs * (2.0f * 3.14159265f / 60.0f);
    updateCurrentPidVoltageLimit(omega_elec);

    /* ================================================================
     * [3] d/q 电流 PID 控制
     *
     *   v_d = PID_d(id_ref - i_d)  -- D 轴, id_ref = 0 (Id=0 控制)
     *   v_q = PID_q(iq_ref - i_q)  -- Q 轴, iq_ref 来自速度环/扭矩命令
     *
     * PID 控制器内部已含积分限幅, output_limit 防止饱和
     * ================================================================ */
    ctx_.v_d = controller_.pid_d.update(id_ref - ctx_.i_d, dt_);
    ctx_.v_q = controller_.pid_q.update(iq_ref - ctx_.i_q, dt_);
#if LIB_MOTOR_ENABLE_HFI
    if (ctx_.hfi_injection.enabled)
    {
        ctx_.v_d += ctx_.hfi_injection.v_d;
        ctx_.v_q += ctx_.hfi_injection.v_q;
    }
#endif

    /* [3.5] 联合电压矢量圆限幅: √(vd²+vq²) ≤ K_mod × Vbus
     * 替代旧的固定 ±10V 方限制, 解决宽 VBUS 失配问题;
     * 饱和前留一份 v_q_pre_clamp 用于饱和回算;
     * Ke 不在此处扣减; 当前未接入 ωKe 电压前馈, 仅做实时 Vbus 圆限幅。
     */
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL || LIB_MOTOR_ENABLE_POSITION_CONTROL
    const float v_q_pre_clamp = ctx_.v_q;
#endif
    MotorTargetLimiter::limitVoltageVector(config_, ctx_.v_bus, omega_elec,
                                            ctx_.v_d, ctx_.v_q);

    /* [3.6] 电流环饱和回算给速度环 (anti-windup 一致性):
     * 当 vq 被 limitVoltageVector 显著缩放 (电压饱和) 或电流环 PID 自身 isSaturated() 时,
     * 把速度环积分衰减一次, 防止速度环继续推 Iq 而物理上无法实现 → 漂移。
     */
#if LIB_MOTOR_ENABLE_VELOCITY_CONTROL || LIB_MOTOR_ENABLE_POSITION_CONTROL
    {
        const float vq_mag_pre = fabsf(v_q_pre_clamp);
        const float vq_mag_post = fabsf(ctx_.v_q);
        const bool voltage_saturated =
            (vq_mag_pre > 1e-3f) &&
            (vq_mag_post < vq_mag_pre * 0.95f);
        const bool pid_saturated = controller_.pid_q.isSaturated();
        if (voltage_saturated || pid_saturated)
        {
            // 电压饱和: 衰减速度环积分 5% 每 tick, 不一次性清零;
            // 用经验值 0.95 (本轮回算系数固定, P2 阶段可暴露为 cfg)
            controller_.decaySpeedIntegral(0.95f);
        }
    }
#endif

    /* ================================================================
     * [4] InvPark 变换: dq (旋转) -> alpha/beta (两相)
     *
     *   v_alpha = v_d * cos(theta) - v_q * sin(theta)
     *   v_beta  = v_d * sin(theta) + v_q * cos(theta)
     *
     * 复用 [1] 预算的 sin_ae/cos_ae, 不再重复调用三角函数
     * ================================================================ */
    inv_park_transform_sc(ctx_.v_d, ctx_.v_q, sin_ae, cos_ae,
                          ctx_.v_alpha, ctx_.v_beta);

    /* ================================================================
     * [5] 调制: alpha/beta 电压 -> 三相占空比
     *
     * SPWM: 正弦调制, 电压利用率 1.0
     *   公式: duty = 0.5 + V_phase / Vbus
     *
     * SVPWM: 空间矢量调制, 电压利用率 2/sqrt(3) ~ 1.1547
     *   将 V_alpha/beta 映射为 8 种开关状态 (000~111), 计算占空比
     *   比 SPWM 提高 15.47% 电压利用率
     * ================================================================ */
    switch (config_.control.modulation)
    {
        case ModulationMethod::SPWM:
        {
            float duty[3];
            spwm_modulate(ctx_.v_alpha, ctx_.v_beta, ctx_.v_bus, duty);
            ctx_.duty_a = duty[0];
            ctx_.duty_b = duty[1];
            ctx_.duty_c = duty[2];
        }
        break;

        case ModulationMethod::SVPWM:
        default:
        {
            float duty[3];
            svpwm_modulate(ctx_.v_alpha, ctx_.v_beta, ctx_.v_bus, duty);
            ctx_.duty_a = duty[0];
            ctx_.duty_b = duty[1];
            ctx_.duty_c = duty[2];
        }
        break;
    }
    publishRuntimeOutputs();
#endif
}

} // namespace Lib_Motor
