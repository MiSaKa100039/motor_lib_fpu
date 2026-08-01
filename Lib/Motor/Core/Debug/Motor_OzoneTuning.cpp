#include "Motor_OzoneTuning.h"

namespace Lib_Motor
{

#if LIB_MOTOR_ENABLE_OZONE_TUNING

MotorOzoneTuningFacade g_motor_ozone_tuning[LIB_MOTOR_MAX_INSTANCES];

namespace
{
constexpr uint32_t kGroupCurrentLoop = 1UL << 0U;
constexpr uint32_t kGroupSpeedLoop = 1UL << 1U;
constexpr uint32_t kGroupLimits = 1UL << 2U;
constexpr uint32_t kGroupObserver = 1UL << 3U;

MotorOzonePidTuning toTuningPid(const PIDParam& src)
{
    MotorOzonePidTuning out;
    out.kp = src.kp;
    out.ki = src.ki;
    out.kd = src.kd;
    out.output_limit = src.output_limit;
    return out;
}
} // namespace

void Motor_OzoneTuning_LoadFromConfig(uint8_t motor_id, const MotorConfig& cfg)
{
    if (motor_id >= LIB_MOTOR_MAX_INSTANCES)
    {
        return;
    }

    MotorOzoneTuningFacade& dst = g_motor_ozone_tuning[motor_id];
    dst.magic = 0x4D54554EUL;
    dst.version = 1U;
    dst.enabled_groups = 0U;
    dst.dirty_flags = 0U;

#if LIB_MOTOR_TUNING_ENABLE_CURRENT_LOOP
    dst.enabled_groups |= kGroupCurrentLoop;
    dst.target_id_a = 0.0f;
    dst.target_iq_a = 0.0f;
    dst.current_d = toTuningPid(cfg.control.current_d);
    dst.current_q = toTuningPid(cfg.control.current_q);
#endif

#if LIB_MOTOR_TUNING_ENABLE_SPEED_LOOP
    dst.enabled_groups |= kGroupSpeedLoop;
    dst.target_speed_rpm = 0.0f;
    dst.speed = toTuningPid(cfg.control.speed);
#endif

#if LIB_MOTOR_TUNING_ENABLE_LIMITS
    dst.enabled_groups |= kGroupLimits;
    dst.max_current_a = cfg.limit.max_current_a;
    dst.max_speed_rpm = cfg.limit.max_speed_rpm;
    dst.max_duty_cycle = cfg.limit.max_duty_cycle;
    dst.over_voltage_v = cfg.limit.over_voltage_v;
    dst.under_voltage_v = cfg.limit.under_voltage_v;
#endif

#if LIB_MOTOR_TUNING_ENABLE_OBSERVER
    dst.enabled_groups |= kGroupObserver;
    dst.align_current_a = cfg.observer.align_current_a;
    dst.drag_current_a = cfg.observer.drag_current_a;
    dst.drag_accel_rpm_s = cfg.observer.drag_accel_rpm_s;
    dst.smo_gain = cfg.observer.smo_gain;
    dst.smo_pll_kp = cfg.observer.smo_pll_kp;
    dst.smo_pll_ki = cfg.observer.smo_pll_ki;
#endif
}

void Motor_OzoneTuning_ClearDirty(uint8_t motor_id)
{
    if (motor_id >= LIB_MOTOR_MAX_INSTANCES)
    {
        return;
    }

    g_motor_ozone_tuning[motor_id].dirty_flags = 0U;
}

#endif

} // namespace Lib_Motor
