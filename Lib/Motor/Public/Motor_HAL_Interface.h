#pragma once

#include <stdint.h>

namespace Lib_Motor
{

// PWM control: inputs are normalized duties in [0.0, 1.0].
using Motor_HAL_SetPWMCallback = void (*)(float u, float v, float w);

// BSP declares which physical phase-current channels are really populated.
// read_currents_raw() must always return physical U/V/W in fixed positions.
constexpr uint8_t MOTOR_PHASE_CURRENT_U_VALID = (1U << 0);
constexpr uint8_t MOTOR_PHASE_CURRENT_V_VALID = (1U << 1);
constexpr uint8_t MOTOR_PHASE_CURRENT_W_VALID = (1U << 2);
constexpr uint8_t MOTOR_PHASE_CURRENT_ALL_VALID =
    MOTOR_PHASE_CURRENT_U_VALID | MOTOR_PHASE_CURRENT_V_VALID | MOTOR_PHASE_CURRENT_W_VALID;

constexpr uint8_t MOTOR_CURRENT_SAMPLE_SCHEDULE_MAX_POINTS = 2U;

// Hardware-only ADC trigger schedule. Phase/sign reconstruction stays inside Sampling.
struct MotorCurrentSampleSchedule
{
    uint8_t enabled = 0U;
    uint8_t sample_count = 0U;
    float trigger_duty[MOTOR_CURRENT_SAMPLE_SCHEDULE_MAX_POINTS] = {0.5f, 0.5f};
};

// Current ADC samples. The bus-current value is optional and ignored when has_bus_current=false.
using Motor_HAL_ReadCurrentCallback =
    void (*)(uint16_t* raw_u, uint16_t* raw_v, uint16_t* raw_w, uint16_t* raw_bus);
using Motor_HAL_ReadSingleShuntPairCallback =
    bool (*)(uint16_t* raw_first, uint16_t* raw_second);
using Motor_HAL_ApplyCurrentSampleScheduleCallback =
    void (*)(const MotorCurrentSampleSchedule* schedule);

// Voltage/temperature ADC samples.
using Motor_HAL_ReadVbusCallback = uint16_t (*)(void);
using Motor_HAL_ReadPhaseVoltagesCallback =
    void (*)(uint16_t* raw_u, uint16_t* raw_v, uint16_t* raw_w);
using Motor_HAL_ReadTempCallback = uint16_t (*)(uint8_t index);

using Motor_HAL_VoidCallback = void (*)(void);
using Motor_HAL_SetBrakeChopperDutyCallback = void (*)(float duty);

/* [本轮新增] Flash 持久化接口 (本轮仅 HAL 接口预占位, 库不调用)
 * 由项目侧 BSP 实现 boot/flash_read/write/erase; nullptr 表示无 flash 持久化能力,
 * AutoTune learned 字段仅在 RAM 中维护, 关机丢失 (P2 阶段再接通存储执行链)。
 *
 * 全部按字节单位: addr 由项目指定绝对或相对扇区起始; len 是字节数。
 */
enum class MotorHAL_Result : uint8_t { Ok = 0, Error, Busy };
using Motor_HAL_FlashReadCallback  = MotorHAL_Result (*)(uint32_t addr, void* dst, uint32_t len);
using Motor_HAL_FlashWriteCallback = MotorHAL_Result (*)(uint32_t addr, const void* src, uint32_t len);
using Motor_HAL_FlashEraseCallback = MotorHAL_Result (*)(uint32_t addr, uint32_t len_bytes);

struct MotorHAL_t
{
    /* --- PWM control --- */
    Motor_HAL_SetPWMCallback set_duty;
    Motor_HAL_VoidCallback   pwm_enable;
    Motor_HAL_VoidCallback   pwm_disable;

    /* --- stop/brake behavior --- */
    Motor_HAL_VoidCallback   pwm_coast;
    Motor_HAL_VoidCallback   pwm_brake_lowside;
    Motor_HAL_SetBrakeChopperDutyCallback brake_resistor_set_duty = nullptr;
    Motor_HAL_VoidCallback   brake_resistor_disable = nullptr;
    Motor_HAL_VoidCallback   mechanical_brake_engage;
    Motor_HAL_VoidCallback   mechanical_brake_release;

    /* --- sampling --- */
    Motor_HAL_ReadCurrentCallback       read_currents_raw;
    uint8_t                             phase_current_valid_mask;
    Motor_HAL_ReadPhaseVoltagesCallback read_phase_voltages_raw;
    Motor_HAL_ReadVbusCallback          read_vbus_raw;
    Motor_HAL_ReadTempCallback          read_temp_raw;

    /* --- critical section --- */
    Motor_HAL_VoidCallback enter_critical;
    Motor_HAL_VoidCallback exit_critical;

    /* --- flash persistence (本轮新增, 可空) ---
     * 全部 nullptr 时库不调用; 项目按需实现写 learned 字段;
     * flash_sector_size 用于库侧判断是否可整扇区擦除。
     */
    Motor_HAL_FlashReadCallback  flash_read       = nullptr;
    Motor_HAL_FlashWriteCallback flash_write      = nullptr;
    Motor_HAL_FlashEraseCallback flash_erase      = nullptr;
    uint32_t                     flash_sector_size = 0;

    Motor_HAL_ReadSingleShuntPairCallback read_single_shunt_pair_raw = nullptr;
    Motor_HAL_ApplyCurrentSampleScheduleCallback apply_current_sample_schedule = nullptr;
};

} // namespace Lib_Motor
