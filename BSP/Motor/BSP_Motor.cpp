/*
 * BSP_Motor.cpp - LC32 电机驱动硬件适配层
 *
 * 本文件把当前板级 PWM/ADC 能力绑定到 Lib_Motor::MotorHAL_t。
 * 与 STM32G4 参考 BSP 不同，LC32 工程的 TIM/ADC 寄存器操作集中在
 * Core/Src/main.c，这里只通过 Core_* wrapper 转发硬件请求。
 *
 * 当前 LC32 低成本板硬件映射：
 *   - TIM1 CH1/2/3：U/V/W PWM 比较输出，由 Core_TIM1_SetPwmCompare() 写入。
 *   - TIM1 CH4：ADC 触发比较点，用于驱动 PA0/PA1 regular 扫描。
 *   - PA1：Vbus raw；PA0：外部 NTC raw。电流 raw pair 当前未接入。
 *   - phase_current_valid_mask：0，表示没有独立安装 U/V/W 相电流传感器。
 *
 * 分层约束：
 *   - BSP 只声明真实硬件能力，不根据 MotorCfg/BuildCfg 的采样模式伪造通道。
 *   - 当前采样算法由 MotorCfg/BuildCfg 选择，能力不匹配由 ConfigCheck 拦截。
 *   - Core_Motor_HardwareBindingEnabled() 打开前禁止真实桥臂输出。
 */

#include "BSP_Motor.h"

#include "main.h"
#include "lcm32f039.h"

namespace
{

constexpr uint16_t kVofaTelemetryDividerFromFastIrq = 10U;

volatile uint8_t s_vofa_telemetry_enabled = 0U;
volatile uint8_t s_vofa_telemetry_pending = 0U;
uint16_t s_vofa_telemetry_divider_count = 0U;

uint32_t BSP_SaveAndDisableIrq()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void BSP_RestoreIrq(uint32_t primask)
{
    if ((primask & 1UL) == 0UL)
    {
        __enable_irq();
    }
}

void BSP_ResetVofaTelemetryGate(bool enabled)
{
    const uint32_t primask = BSP_SaveAndDisableIrq();
    s_vofa_telemetry_enabled = enabled ? 1U : 0U;
    s_vofa_telemetry_pending = 0U;
    s_vofa_telemetry_divider_count = 0U;
    BSP_RestoreIrq(primask);
}

float BSP_ClampDuty_Impl(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

uint16_t BSP_DutyToCompare_Impl(float duty)
{
    const uint32_t period = Core_TIM1_PwmPeriodCounts();
    const float clamped = BSP_ClampDuty_Impl(duty);
    float compare = clamped * static_cast<float>(period);
    if (compare < 0.0f) compare = 0.0f;
    if (compare > static_cast<float>(period)) compare = static_cast<float>(period);
    return static_cast<uint16_t>(compare);
}

void BSP_PWM_DisablePowerChannels_Impl()
{
    Core_TIM1_DisablePwmOutputs();
}

/*
 * 写入三相 PWM 占空比。
 * LC32 Core 层负责真实 TIM1 CCR 写入；BSP 只做归一化 duty 到计数值的换算，
 * 并在硬件绑定确认前强制关闭桥臂输出。
 */
void BSP_SetPWM_Impl(float u, float v, float w)
{
    const uint16_t compare_u = BSP_DutyToCompare_Impl(u);
    const uint16_t compare_v = BSP_DutyToCompare_Impl(v);
    const uint16_t compare_w = BSP_DutyToCompare_Impl(w);

    if (!Core_Motor_HardwareBindingEnabled())
    {
        BSP_PWM_DisablePowerChannels_Impl();
        return;
    }

    Core_TIM1_SetPwmCompare(compare_u, compare_v, compare_w);
}

void BSP_PWM_Enable_Impl()
{
    if (!Core_Motor_HardwareBindingEnabled())
    {
        BSP_PWM_DisablePowerChannels_Impl();
    }
}

void BSP_PWM_Disable_Impl()
{
    BSP_PWM_DisablePowerChannels_Impl();
}

void BSP_PWM_Coast_Impl()
{
    BSP_PWM_DisablePowerChannels_Impl();
}

/*
 * 下管短路制动占位。
 * 当前 LC32 上电安全路径仍在写 0% duty 后关闭功率通道；待桥臂绑定关系确认后，
 * 若硬件支持低边短路制动，可在这里改为保持低边导通。
 */
void BSP_PWM_BrakeLow_Impl()
{
    Core_TIM1_SetPwmCompare(0U, 0U, 0U);
    BSP_PWM_DisablePowerChannels_Impl();
}

/*
 * 双相/三相相电流接口兼容路径。
 * 当前 Core ADC 仅接入 PA0/PA1 辅助采样，phase_current_valid_mask 固定为 0。
 * 这里返回 0 只保持 HAL 契约完整，不表示真实安装了相电流传感器。
 */
void BSP_ReadCurrent_Raw_Impl(uint16_t* raw_u,uint16_t* raw_v,uint16_t* raw_w,uint16_t* raw_bus)
{
    if ((raw_u == nullptr) || (raw_v == nullptr) ||
        (raw_w == nullptr) || (raw_bus == nullptr))
    {
        return;
    }

    *raw_u = 0U;
    *raw_v = 0U;
    *raw_w = 0U;
    *raw_bus = 0U;
}

/*
 * 单电阻两点采样接口暂未接入真实电流放大器。
 * PA0/PA1 当前保留给 NTC 与 Vbus，不能作为 single-shunt raw pair 上报。
 */
bool BSP_ReadSingleShuntPair_Raw_Impl(uint16_t* raw_first, uint16_t* raw_second)
{
    if ((raw_first == nullptr) || (raw_second == nullptr))
    {
        return false;
    }

    *raw_first = 0U;
    *raw_second = 0U;
    return false;
}

/*
 * 下发下一 PWM 周期的 ADC 采样硬件时刻。
 * 当前未接入 single-shunt 电流 raw pair，保持 ADC 为 PA0/PA1 regular 扫描。
 */
void BSP_ApplyCurrentSampleSchedule_Impl(const Lib_Motor::MotorCurrentSampleSchedule* schedule)
{
    (void)schedule;
    Core_ADC_EnableTriggerPairMode(false);
}

uint16_t BSP_ReadVbus_Raw_Impl()
{
    return Core_Motor_HardwareInitialized() ? Core_ADC_GetVbusRaw() : 0U;
}

uint16_t BSP_ReadTemp_Raw_Impl(uint8_t index)
{
    if (!Core_Motor_HardwareInitialized() || (index != 0U)) return 0U;
    return Core_ADC_GetTempRaw(0U);
}

void BSP_EnterCritical_Impl()
{
    __disable_irq();
}

void BSP_ExitCritical_Impl()
{
    __enable_irq();
}

Lib_Motor::MotorHAL_t bsp_hal_impl{
    BSP_SetPWM_Impl,
    BSP_PWM_Enable_Impl,
    BSP_PWM_Disable_Impl,
    BSP_PWM_Coast_Impl,
    BSP_PWM_BrakeLow_Impl,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    BSP_ReadCurrent_Raw_Impl,
    0U, /* 当前单电阻板没有独立相电流采样通道。 */
    nullptr,
    BSP_ReadVbus_Raw_Impl,
    BSP_ReadTemp_Raw_Impl,
    BSP_EnterCritical_Impl,
    BSP_ExitCritical_Impl,
    nullptr,
    nullptr,
    nullptr,
    0U,
    BSP_ReadSingleShuntPair_Raw_Impl,
    BSP_ApplyCurrentSampleSchedule_Impl};

} // namespace

volatile uint32_t BSP_Motor_HardwareStartStage = 0U;
volatile uint32_t BSP_Motor_HardwareStartFailure = 0U;
volatile uint32_t BSP_Motor_InjectedAdc1CallbackCount = 0U;
volatile uint32_t BSP_Motor_TelemetryProducedCount = 0U;
volatile uint32_t BSP_Motor_TelemetryConsumedCount = 0U;
volatile uint32_t BSP_Motor_TelemetryDroppedCount = 0U;

#ifdef MOTOR_BUILD_ENABLE_TICK_PROFILING
volatile uint32_t BSP_Motor_InjectedAdc1CallbackRateHz = 0U;
volatile uint32_t BSP_Motor_TickBudgetCycles = 0U;
volatile uint32_t BSP_Motor_TickCoreCyclesLast = 0U;
volatile uint32_t BSP_Motor_TickCoreCyclesMax = 0U;
volatile uint32_t BSP_Motor_TickIsrCyclesLast = 0U;
volatile uint32_t BSP_Motor_TickIsrCyclesMax = 0U;
volatile uint32_t BSP_Motor_TickOverBudgetCount = 0U;
#endif

const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void)
{
    return &bsp_hal_impl;
}

void BSP_Motor_SetControlFrequencyHz(float control_freq_hz)
{
#ifdef MOTOR_BUILD_ENABLE_TICK_PROFILING
    BSP_Motor_TickBudgetCycles =
        (control_freq_hz > 0.0f)
            ? static_cast<uint32_t>(SystemCoreClock / control_freq_hz)
            : 0U;
#else
    (void)control_freq_hz;
#endif
}

void BSP_Motor_Hardware_Start(void)
{
    BSP_Motor_HardwareStartStage = 1U;
    BSP_Motor_HardwareStartFailure = 0U;
    BSP_Motor_TelemetryProducedCount = 0U;
    BSP_Motor_TelemetryConsumedCount = 0U;
    BSP_Motor_TelemetryDroppedCount = 0U;
    BSP_ResetVofaTelemetryGate(true);

    if (Core_Motor_HardwareStart())
    {
        BSP_Motor_HardwareStartStage = 2U;
        return;
    }

    BSP_ResetVofaTelemetryGate(false);
    BSP_Motor_HardwareStartFailure = 1U;
    BSP_PWM_DisablePowerChannels_Impl();
}

void BSP_Motor_RequestVofaTelemetryFromFastIrq(void)
{
    if (s_vofa_telemetry_enabled == 0U)
    {
        return;
    }

    if (++s_vofa_telemetry_divider_count < kVofaTelemetryDividerFromFastIrq)
    {
        return;
    }

    s_vofa_telemetry_divider_count = 0U;
    ++BSP_Motor_TelemetryProducedCount;
    if (s_vofa_telemetry_pending != 0U)
    {
        ++BSP_Motor_TelemetryDroppedCount;
        return;
    }

    s_vofa_telemetry_pending = 1U;
}

bool BSP_Motor_TakeTelemetryPending(void)
{
    const uint32_t primask = BSP_SaveAndDisableIrq();
    if (s_vofa_telemetry_pending == 0U)
    {
        BSP_RestoreIrq(primask);
        return false;
    }

    s_vofa_telemetry_pending = 0U;
    BSP_RestoreIrq(primask);
    ++BSP_Motor_TelemetryConsumedCount;
    return true;
}
