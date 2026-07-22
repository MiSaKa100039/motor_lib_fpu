/*
 * Motor_RuntimeMonitor.cpp -- 安全检测: 运行时监控
 *
 * 文件层级: Safety/RuntimeMonitor
 *
 * 职责: 每个控制 tick 检查运行时物理量是否超出安全阈值
 *   1. 过流检测: 三相电流中最大值超过 max_current_a -> Fault::OVERCURRENT
 *   2. 过压检测: 母线电压超过 over_voltage_v -> Fault::OVERVOLT
 *   3. 欠压检测: 初始化采样完成后母线电压低于 under_voltage_v -> Fault::UNDERVOLT
 *   4. 过温检测: 任一使能的 NTC 超过 over_temp_c -> Fault::OVERTEMP
 *
 * 所有故障通过 OR 位运算合并到 fault_ 寄存器, 由 Motor_FSM 处理
 */

#include "Motor_RuntimeMonitor.h"

#include <cmath>

namespace Lib_Motor
{

/*
 * check -- 运行时安全阈值检查 (每 tick 调用)
 *
 * @param limits  -- 限制参数 (max_current_a/over_voltage_v/under_voltage_v)
 * @param sensor  -- 传感器配置 (NTC 数量/阈值)
 * @param ia,ib,ic -- 三相电流 (已滤波)
 * @param v_bus   -- 母线电压
 * @param temp    -- NTC 温度数组
 * @param state   -- 当前 FSM 状态 (INIT/ADC_CAL 由调用方跳过)
 * @param fault   -- 输出: 合并故障位
 */
void MotorRuntimeMonitor::check(const MotorLimitParam& limits,
                                const MotorSensorParam& sensor,
                                float ia, float ib, float ic,
                                float v_bus, const float temp[4],
                                State state, Fault* fault)
{
    if (fault == nullptr) return;

    Fault current_fault = Fault::NONE;

    // [A] 过流检测: 取三相电流绝对值的最大值
    float max_current = fabsf(ia);
    if (fabsf(ib) > max_current) max_current = fabsf(ib);
    if (fabsf(ic) > max_current) max_current = fabsf(ic);

    if (max_current > limits.max_current_a)
    {
        current_fault = static_cast<Fault>(
            static_cast<uint16_t>(current_fault) |
            static_cast<uint16_t>(Fault::OVERCURRENT));
    }

    // [B] 过压检测
    if (v_bus > limits.over_voltage_v)
    {
        current_fault = static_cast<Fault>(
            static_cast<uint16_t>(current_fault) |
            static_cast<uint16_t>(Fault::OVERVOLT));
    }

    // [C] 欠压在 STOP/STOPPING/RUN 均锁故障, 防止无母线上电后静默等待到 start() 才报错。
    if (v_bus < limits.under_voltage_v)
    {
        current_fault = static_cast<Fault>(
            static_cast<uint16_t>(current_fault) |
            static_cast<uint16_t>(Fault::UNDERVOLT));
    }

    // [D] 过温检测: 遍历所有使能的 NTC 通道
    for (uint8_t i = 0; i < sensor.ntc_count; ++i)
    {
        if (!sensor.ntc[i].enabled) continue;
        if (temp[i] > sensor.ntc[i].over_temp_c)
        {
            current_fault = static_cast<Fault>(
                static_cast<uint16_t>(current_fault) |
                static_cast<uint16_t>(Fault::OVERTEMP));
        }
    }

    // 将本轮检测到的故障合并到外部 fault_ 寄存器
    *fault = static_cast<Fault>(
        static_cast<uint16_t>(*fault) |
        static_cast<uint16_t>(current_fault));
}

/*
 * checkOverspeed — 超速故障检查 (本轮新增)
 *
 * |speed_rpm_now| > max_speed_rpm × overspeed_factor -> Fault::OVERSPEED
 * 该故障由上层 FSM 视为故障级锁停机 (latch=true 由 SafetyFSM 决定)。
 * 与 limitIqReference 的超速硬切除不同: 硬切除只是切同向 Iq, 不锁故障;
 * OVERSPEED 是机械保护类故障, 一旦触发应停机由人工/上位机复位。
 */
void MotorRuntimeMonitor::checkOverspeed(float max_speed_rpm,
                                          float overspeed_factor,
                                          float speed_rpm_now,
                                          Fault* fault)
{
    if (fault == nullptr) return;
    if (max_speed_rpm <= 0.0f || overspeed_factor <= 0.0f) return;

    const float threshold = max_speed_rpm * overspeed_factor;
    if (fabsf(speed_rpm_now) > threshold)
    {
        *fault = static_cast<Fault>(
            static_cast<uint16_t>(*fault) |
            static_cast<uint16_t>(Fault::OVERSPEED));
    }
}

} // namespace Lib_Motor
