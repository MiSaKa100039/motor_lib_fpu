#include "Platform_MotorTestConfig.h"
#include "User_MotorTestConfig.h"

/* ===================== 调试/测试变量实例化 ===================== */

#if defined(USER_MOTOR_TEST_PWM_MANUAL)
/*
 * PWM 手动测试默认占空比 (5%) — 低压安全测试起点。
 * 调大前须确保: 功率级已上电 + 电流探头已就绪 + 限流阈值已降低。
 */
volatile float Platform_TestPWM_U = 0.0f;
volatile float Platform_TestPWM_V = 0.0f;
volatile float Platform_TestPWM_W = 0.0f;
#endif

#if defined(USER_MOTOR_TEST_CURRENT_LOCK)
/*
 * 电流锁定默认 Id=0, Iq=0 — 无转矩输出。
 * 增加到小值 (如 Id=1A, Iq=0) 可实现锁轴测试验证角度校准。
 */
volatile float Platform_TestCurrentLockId = 0.0f;
volatile float Platform_TestCurrentLockIq = 0.0f;
#endif

#if defined(USER_MOTOR_TEST_VF_CONTROL)
/*
 * VF 开环拖动阶段表:
 *   {持续时间(s), 目标转速(RPM), 最终占空比}
 *
 * 若电机不转: 逐步增加 duty (但不超过母线电压/额定电压)。
 * 若电机振动: 减小 duty 或延长每段时间, 可能是爬升太快。
 * 48V 测试母线额定参考 — 实际根据供电调整每个阶段的 duty。
 * Platform_TestVFStartupPhases 用于调试中修改阶段参数
 * Platform_TestVFStartupProfile 用于设置启用段数
 */
volatile Lib_Motor::MotorVFStartupPhase Platform_TestVFStartupPhases[] = {
    {1.00f,   100.0f, 0.04f},
    {1.00f, 400.0f, 0.04f},
    {1.00f, 600.0f, 0.045f},
    {1.00f, 800.0f, 0.05f},
    {1.00f, 1000.0f, 0.055f},
};
#endif

#if defined(USER_MOTOR_TEST_IF_CONTROL) || \
    defined(USER_MOTOR_TEST_IF_SMO_OBSERVER) || \
    defined(USER_MOTOR_TEST_IF_HFI_OBSERVER)
/*
 * IF 开环拖动阶段表:
 *   {持续时间(s), 目标转速(RPM), 最终 Id(A), 最终 Iq(A)}
 *
 * Id 提供励磁, Iq 提供转矩来拖动转子。
 * 阶段 1: 纯 Id 对齐 (转速=0), 保持 align_time_s 秒让转子吸合到位
 * 阶段 2~N: Id 逐渐过渡到 0, Iq 增大, 转速爬升
 * 若失步: 减小 accel 斜率或增大 Iq
 * Platform_TestIFStartupPhases
 * Platform_TestIFStartupProfile
 */
volatile Lib_Motor::MotorIFStartupPhase Platform_TestIFStartupPhases[] = {
    {1.00f,   100.0f, 0.00f, 11.00f},
    {1.00f,  400.0f, 0.00f, 7.00f},
    {1.50f, 1200.0f, 0.00f, 8.00f},
    {2.00f, 3000.0f, 0.00f, 6.00f},
    {1.50f, 600.0f, 0.00f, 1.20f},
};
#endif

#if defined(USER_MOTOR_TEST_VF_CONTROL)
/*
 * VF 起动曲线 — 将阶段表打包为起动定义。
 * phase_count: 使用的阶段数 (≤ 数组长度)。
 * 仅第一个阶段启动 VF, 若阶段数 > 1 则按序列自动衔接。
 */
Lib_Motor::MotorVFStartupProfile Platform_TestVFStartupProfile = {
    Platform_TestVFStartupPhases,
    1U
};
#endif

#if defined(USER_MOTOR_TEST_IF_CONTROL) || \
    defined(USER_MOTOR_TEST_IF_SMO_OBSERVER) || \
    defined(USER_MOTOR_TEST_IF_HFI_OBSERVER)
/*
 * IF 起动曲线 — 同上, 将阶段表打包。
 */
Lib_Motor::MotorIFStartupProfile Platform_TestIFStartupProfile = {
    Platform_TestIFStartupPhases,
#if defined(USER_MOTOR_TEST_IF_SMO_OBSERVER) || \
    defined(USER_MOTOR_TEST_IF_HFI_OBSERVER)
    static_cast<uint8_t>(
        sizeof(Platform_TestIFStartupPhases) /
        sizeof(Platform_TestIFStartupPhases[0]))
#else
    1U
#endif
};
#endif
