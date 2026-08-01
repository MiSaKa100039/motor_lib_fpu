#pragma once

#include "../../Public/Motor_Config.h"
#include "../../Public/Motor_Definitions.h"

namespace Lib_Motor
{

class MotorRuntimeMonitor
{
public:
    void check(const MotorLimitParam& limits,
               const MotorSensorParam& sensor,
               float ia, float ib, float ic,
               float v_bus, const float temp[4],
               State state, Fault* fault);

    /* 超速故障检查 (本轮新增)
     * @param max_speed_rpm  额定速度上限 (RPM)
     * @param overspeed_factor  超速系数 (1.10 表示超出 10% 触发)
     * @param speed_rpm_now  当前实测转速 (RPM)
     * @param fault  输出: 合并 Fault::OVERSPEED
     */
    void checkOverspeed(float max_speed_rpm,
                        float overspeed_factor,
                        float speed_rpm_now,
                        Fault* fault);
};

} // namespace Lib_Motor
