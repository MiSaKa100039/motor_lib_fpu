/*
 * BSP_Motor.cpp - LC32 电机驱动硬件适配层
 *
 * 本文件把当前板级 PWM/ADC 能力绑定到 Lib_Motor::MotorHAL_t。
 * 与 STM32G4 参考 BSP 不同，LC32 工程的 TIM/ADC 寄存器操作集中在
 * Core/Src/main.c，这里只通过 Core_* wrapper 转发硬件请求。
 *
 * 当前 LC32 板级硬件映射：
 *   - TIM1 CH1/2/3：U/V/W PWM 比较输出，由 Core_TIM1_SetPwmCompare() 写入。
 *   - TIM1 CH4：ADC 单次触发比较点，用于驱动 U/V/Bus/Vbus/NTC regular 扫描。
 *   - OPA0 OP0OEX->ADCIN[11]：U 相；OPA1 OP1OEX->ADCIN[12]：V 相。
 *   - OPA2 OP2O/PC12->ADCIN[14]：母线电流，同时给 ACMP1 做硬件过流。
 *   - phase_current_valid_mask：U|V，W 由库按 KCL 重构。
 *
 * 分层约束：
 *   - BSP 只声明真实硬件能力，不根据 MotorCfg/BuildCfg 的采样模式伪造通道。
 *   - 当前采样算法由 MotorCfg/BuildCfg 选择，能力不匹配由 ConfigCheck 拦截。
 *   - Core_PowerBridgeBindingEnabled() 打开前禁止真实桥臂输出。
 */

#include "BSP_Motor.h"

#include "main.h"
#include "lcm32f039.h"

volatile uint32_t BSP_Motor_HardwareStartStage = 0U;
volatile uint32_t BSP_Motor_HardwareStartFailure = 0U;
volatile uint32_t BSP_Motor_InjectedAdc1CallbackCount = 0U;
volatile uint32_t BSP_Motor_TelemetryProducedCount = 0U;
volatile uint32_t BSP_Motor_TelemetryConsumedCount = 0U;
volatile uint32_t BSP_Motor_TelemetryDroppedCount = 0U;
volatile uint32_t BSP_Motor_HardwareFaultLatch = 0U;

#ifdef MOTOR_BUILD_ENABLE_TICK_PROFILING
volatile uint32_t BSP_Motor_InjectedAdc1CallbackRateHz = 0U;
volatile uint32_t BSP_Motor_TickBudgetCycles = 0U;
volatile uint32_t BSP_Motor_TickCoreCyclesLast = 0U;
volatile uint32_t BSP_Motor_TickCoreCyclesMax = 0U;
volatile uint32_t BSP_Motor_TickIsrCyclesLast = 0U;
volatile uint32_t BSP_Motor_TickIsrCyclesMax = 0U;
volatile uint32_t BSP_Motor_TickOverBudgetCount = 0U;
#endif

namespace
{

constexpr uint16_t kVofaTelemetryDividerFromFastIrq = 10U;
constexpr bool kPhaseUOutputInverted = false;
constexpr bool kPhaseVOutputInverted = false;
constexpr bool kPhaseWOutputInverted = false;
constexpr uint32_t kQ15DutyFullScale = 32767UL;

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

#if !LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
float BSP_ClampDuty_Impl(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

uint16_t BSP_DutyToCompare_Impl(float duty, bool inverted)
{
    const uint32_t period = Core_TIM1_PwmPeriodCounts();
    const float clamped = BSP_ClampDuty_Impl(duty);
    const float effective_duty = inverted ? (1.0f - clamped) : clamped;
    float compare = effective_duty * static_cast<float>(period);
    if (compare < 0.0f) compare = 0.0f;
    if (compare > static_cast<float>(period)) compare = static_cast<float>(period);
    return static_cast<uint16_t>(compare);
}
#endif

static int16_t BSP_ClampDutyQ15_Impl(int16_t value)
{
    if (value < 0) return 0;
    if (value > static_cast<int16_t>(kQ15DutyFullScale))
    {
        return static_cast<int16_t>(kQ15DutyFullScale);
    }
    return value;
}

uint16_t BSP_DutyQ15ToCompare_Impl(int16_t duty, bool inverted, uint32_t period)
{
    const uint32_t clamped = static_cast<uint32_t>(BSP_ClampDutyQ15_Impl(duty));
    const uint32_t effective_duty = inverted ? (kQ15DutyFullScale - clamped) : clamped;
    if (effective_duty >= kQ15DutyFullScale)
    {
        return static_cast<uint16_t>(period);
    }
    const uint32_t compare = (effective_duty * period + 16384UL) >> 15U;
    return static_cast<uint16_t>((compare > period) ? period : compare);
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
#if !LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
void BSP_SetPWM_Impl(float u, float v, float w)
{
    const uint16_t compare_u = BSP_DutyToCompare_Impl(u, kPhaseUOutputInverted);
    const uint16_t compare_v = BSP_DutyToCompare_Impl(v, kPhaseVOutputInverted);
    const uint16_t compare_w = BSP_DutyToCompare_Impl(w, kPhaseWOutputInverted);

    if (!Core_PowerBridgeBindingEnabled())
    {
        BSP_PWM_DisablePowerChannels_Impl();
        return;
    }

    Core_TIM1_SetPwmCompare(compare_u, compare_v, compare_w);
}
#endif

void BSP_SetPWM_Q15_Impl(int16_t u, int16_t v, int16_t w)
{
    const uint32_t period = Core_TIM1_PwmPeriodCounts();
    const uint16_t compare_u = BSP_DutyQ15ToCompare_Impl(u, kPhaseUOutputInverted, period);
    const uint16_t compare_v = BSP_DutyQ15ToCompare_Impl(v, kPhaseVOutputInverted, period);
    const uint16_t compare_w = BSP_DutyQ15ToCompare_Impl(w, kPhaseWOutputInverted, period);

    if (!Core_PowerBridgeBindingEnabled())
    {
        BSP_PWM_DisablePowerChannels_Impl();
        return;
    }

    Core_TIM1_SetPwmCompare(compare_u, compare_v, compare_w);
}

void BSP_PWM_Enable_Impl()
{
    const uint32_t primask = BSP_SaveAndDisableIrq();
    if (!Core_PowerBridgeBindingEnabled() || !Core_PeripheralsReady() ||
        BSP_Motor_ReadHardwareFaultLatch() != BSP_MOTOR_HW_FAULT_NONE)
    {
        BSP_PWM_DisablePowerChannels_Impl();
        BSP_RestoreIrq(primask);
        return;
    }

    Core_TIM1_EnablePwmOutputs();
    BSP_RestoreIrq(primask);
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
    // 以上注释保留原占位背景；当前通过 Core 实现低边保持，故障锁存优先关断。
    const uint32_t primask = BSP_SaveAndDisableIrq();
    if (!Core_PowerBridgeBindingEnabled() || !Core_PeripheralsReady() ||
        BSP_Motor_ReadHardwareFaultLatch() != BSP_MOTOR_HW_FAULT_NONE)
    {
        BSP_PWM_DisablePowerChannels_Impl();
    }
    else
    {
        Core_TIM1_BrakeLowOutputs();
    }
    BSP_RestoreIrq(primask);
}

/* 双电阻相电流 + 母线电流 raw 读取。W 相没有独立 ADC 槽，由库按 U/V 做 KCL 重构。 */
void BSP_ReadCurrent_Raw_Impl(uint16_t* raw_u,uint16_t* raw_v,uint16_t* raw_w,uint16_t* raw_bus)
{
    if ((raw_u == nullptr) || (raw_v == nullptr) ||
        (raw_w == nullptr) || (raw_bus == nullptr))
    {
        return;
    }

    if (!Core_PeripheralsReady())
    {
        *raw_u = 0U;
        *raw_v = 0U;
        *raw_w = 0U;
        *raw_bus = 0U;
        return;
    }

    *raw_u = Core_ADC_GetPhaseURaw();
    *raw_v = Core_ADC_GetPhaseVRaw();
    *raw_w = 0U;
    *raw_bus = Core_ADC_GetBusCurrentRaw();
}

uint32_t BSP_ReadHardwareFaultLatch_Impl()
{
    return BSP_Motor_ReadHardwareFaultLatch();
}

void BSP_ClearHardwareFaultLatch_Impl(uint32_t flags)
{
    BSP_Motor_ClearHardwareFaultLatch(flags);
}

uint16_t BSP_ReadVbus_Raw_Impl()
{
    /*
     * Core ADC binding:
     *   CORE_ADC_SCAN_SLOT_VBUS -> ADCIN_1_A -> board Vbus divider raw.
     * Voltage scaling is done by Lib from MotorCfg divider parameters.
     */
    return Core_PeripheralsReady() ? Core_ADC_GetVbusRaw() : 0U;
}

uint16_t BSP_ReadTemp_Raw_Impl(uint8_t index)
{
    /*
     * Core ADC binding:
     *   index 0 -> CORE_ADC_SCAN_SLOT_TEMP0 -> ADCIN_0_A -> board NTC0 raw.
     * Additional NTC channels must first be added to the Core ADC sequence.
     */
    if (!Core_PeripheralsReady() || (index != 0U)) return 0U;
    return Core_ADC_GetTemp0Raw();
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
#if LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15
    nullptr,
#else
    BSP_SetPWM_Impl,
#endif
    BSP_PWM_Enable_Impl,
    BSP_PWM_Disable_Impl,
    BSP_PWM_Coast_Impl,
    BSP_PWM_BrakeLow_Impl,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    BSP_ReadCurrent_Raw_Impl,
    static_cast<uint8_t>(Lib_Motor::MOTOR_PHASE_CURRENT_U_VALID |
                         Lib_Motor::MOTOR_PHASE_CURRENT_V_VALID),
    nullptr,
    BSP_ReadVbus_Raw_Impl,
    BSP_ReadTemp_Raw_Impl,
    BSP_EnterCritical_Impl,
    BSP_ExitCritical_Impl,
    nullptr,
    nullptr,
    nullptr,
    0U,
    BSP_ReadHardwareFaultLatch_Impl,
    BSP_ClearHardwareFaultLatch_Impl,
    BSP_SetPWM_Q15_Impl};

} // namespace

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
    BSP_Motor_ClearHardwareFaultLatch(BSP_MOTOR_HW_FAULT_ALL);
    BSP_ResetVofaTelemetryGate(true);

    if (Core_TIM1_StartAdcTriggerPath())
    {
        BSP_Motor_HardwareStartStage = 2U;
        return;
    }

    BSP_ResetVofaTelemetryGate(false);
    BSP_Motor_HardwareStartFailure = 1U;
    BSP_PWM_DisablePowerChannels_Impl();
}

uint8_t BSP_Motor_ResyncSamplingPath(void)
{
    BSP_Motor_HardwareStartStage = 3U;
    BSP_Motor_HardwareStartFailure = 0U;
    BSP_PWM_DisablePowerChannels_Impl();
    BSP_Motor_ClearHardwareFaultLatch(BSP_MOTOR_HW_FAULT_ALL);
    BSP_ResetVofaTelemetryGate(true);

    if (Core_TIM1_ResyncAdcTriggerPath())
    {
        BSP_Motor_HardwareStartStage = 2U;
        return 1U;
    }

    BSP_ResetVofaTelemetryGate(false);
    BSP_Motor_HardwareStartFailure = 1U;
    BSP_PWM_DisablePowerChannels_Impl();
    return 0U;
}

void BSP_Motor_LatchHardwareFaultFromISR(uint32_t flags)
{
    if (flags == BSP_MOTOR_HW_FAULT_NONE)
    {
        flags = BSP_MOTOR_HW_FAULT_UNKNOWN;
    }

    BSP_Motor_HardwareFaultLatch |= flags;
    BSP_PWM_DisablePowerChannels_Impl();
}

uint32_t BSP_Motor_ReadHardwareFaultLatch(void)
{
    return BSP_Motor_HardwareFaultLatch;
}

void BSP_Motor_ClearHardwareFaultLatch(uint32_t flags)
{
    const uint32_t primask = BSP_SaveAndDisableIrq();
    if (flags == BSP_MOTOR_HW_FAULT_NONE || flags == BSP_MOTOR_HW_FAULT_ALL)
    {
        BSP_Motor_HardwareFaultLatch = BSP_MOTOR_HW_FAULT_NONE;
    }
    else
    {
        BSP_Motor_HardwareFaultLatch &= ~flags;
    }
    BSP_RestoreIrq(primask);
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
