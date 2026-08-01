#pragma once

#include "../../Public/Motor_Config.h"
#include "../../Public/Motor_Features.h"

namespace Lib_Motor
{

struct MotorOzonePidTuning
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float output_limit = 0.0f;
};

struct MotorOzoneTuningFacade
{
    uint32_t magic = 0x4D54554EUL;
    uint32_t version = 1U;
    uint32_t enabled_groups = 0U;
    uint32_t dirty_flags = 0U;

#if LIB_MOTOR_TUNING_ENABLE_CURRENT_LOOP
    float target_id_a = 0.0f;
    float target_iq_a = 0.0f;
    MotorOzonePidTuning current_d;
    MotorOzonePidTuning current_q;
#endif

#if LIB_MOTOR_TUNING_ENABLE_SPEED_LOOP
    float target_speed_rpm = 0.0f;
    MotorOzonePidTuning speed;
#endif

#if LIB_MOTOR_TUNING_ENABLE_LIMITS
    float max_current_a = 0.0f;
    float max_speed_rpm = 0.0f;
    float max_duty_cycle = 0.0f;
    float over_voltage_v = 0.0f;
    float under_voltage_v = 0.0f;
#endif

#if LIB_MOTOR_TUNING_ENABLE_OBSERVER
    float align_current_a = 0.0f;
    float drag_current_a = 0.0f;
    float drag_accel_rpm_s = 0.0f;
    float smo_gain = 0.0f;
    float smo_pll_kp = 0.0f;
    float smo_pll_ki = 0.0f;
#endif
};

#if LIB_MOTOR_ENABLE_OZONE_TUNING
extern MotorOzoneTuningFacade g_motor_ozone_tuning[LIB_MOTOR_MAX_INSTANCES];

void Motor_OzoneTuning_LoadFromConfig(uint8_t motor_id, const MotorConfig& cfg);
void Motor_OzoneTuning_ClearDirty(uint8_t motor_id);
#endif

} // namespace Lib_Motor
