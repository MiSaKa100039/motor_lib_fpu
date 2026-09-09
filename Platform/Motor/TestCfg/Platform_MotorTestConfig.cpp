#include "Platform_MotorTestConfig.h"

/* ===================== 调试/测试变量实例化 ===================== */

#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
/*
 * PWM 手动测试默认占空比 (5%) — 低压安全测试起点。
 * 调大前须确保: 功率级已上电 + 电流探头已就绪 + 限流阈值已降低。
 */
volatile float Platform_TestPWM_U = 0.0f;
volatile float Platform_TestPWM_V = 0.0f;
volatile float Platform_TestPWM_W = 0.0f;
#endif

#if LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
/*
 * 电流锁定默认 Id=0, Iq=0 — 无转矩输出。
 * 增加到小值 (如 Id=1A, Iq=0) 可实现锁轴测试验证角度校准。
 */
volatile float Platform_TestCurrentLockId = 0.0f;
volatile float Platform_TestCurrentLockIq = 0.0f;
#endif

#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
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
    {1.00f,   0.0f, 0.03f},
    {1.00f, 400.0f, 0.035f},
    {1.00f, 600.0f, 0.055f},
    {1.00f, 1000.0f, 0.08f},
    {1.00f, 1000.0f, 0.055f},
};
#endif

#if LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
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

