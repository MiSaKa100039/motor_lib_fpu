#include "Usermain.h"

#include <cmath>

#include "Platform_Motor.h"
#include "Platform_VOFA.h"
#include "User_MotorTestConfig.h"
#include "TestCfg/Platform_MotorTestConfig.h"

#include <stdint.h>

#include "main.h"

// TODO: 外部传感器错误判断
// TODO: ISR 中断时间计算
// TODO: 观测器 / 霍尔冗余
// TODO: 未包含的文件
// TODO: 库裁剪
// TODO: 电流方向诊断
// TODO: 多编码器数据处理
// TODO: 弱磁 park/clark 等幅值/等功率下电流环限流区别
// TODO: 母线泄放 / 动能回收 / 刹车
// TODO: 相电压采集预留接口
// TODO: 母线相线电流对应关系
// TODO: sensor health: not ready / not configured

/*
 * 开发路线图:
 *
 * 共同基础:
 *   电流静态检查 → PWM → 低能量 VF → 锁轴电流 → IF
 *
 * 有感:
 *   被动读传感器 → VF 同步观察方向/极对数 → 编码器偏置 → SENSOR 闭环 → 环路整定
 *
 * 无感:
 *   VF/IF 正常 → 影子运行 SMO/HFI 对比角度 → 切换无感闭环 → 环路整定
 */

volatile uint8_t CMD = 0;  // Ozone Watch: CMD

static void User_SendVofaTelemetryIfPending(void)
{
    if (!Core_Motor_TakeTelemetryPending())
    {
        return;
    }

    Lib_Motor::MotorMonitorData monitor{};
    Global_Motor_0.getMonitorData(monitor);

    float data[12];
    data[0]  = monitor.angle_elec_command;
    data[1]  = monitor.angle_elec_observer;
    data[2]  = monitor.observer_error;
    data[3]  = monitor.speed_rpm;
    data[4]  = monitor.speed_rpm_observer;
    data[5]  = monitor.observer_pll_error;
    data[6]  = monitor.observer_signal_level;
    data[7]  = monitor.observer_quality;
    data[8]  = static_cast<float>(monitor.observer_valid_ticks);
    data[9]  = monitor.event.observer_converged ? 1.0f : 0.0f;
    data[10] = monitor.i_d;
    data[11] = monitor.i_q;

    Platform_VOFA_SendFloat(data, static_cast<uint8_t>(sizeof(data) / sizeof(data[0])));
}

static Lib_Motor::Result User_StartMotor(void)
{
#if defined(USER_MOTOR_TEST_PWM_MANUAL)
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugPWMManual(
        Platform_TestPWM_U,
        Platform_TestPWM_V,
        Platform_TestPWM_W);
    Global_Motor_0.lockDebug();
    return result;
#elif defined(USER_MOTOR_TEST_CURRENT_LOCK)
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugCurrentLock(
        Platform_TestCurrentLockId,
        Platform_TestCurrentLockIq);
    Global_Motor_0.lockDebug();
    return result;
#elif defined(USER_MOTOR_TEST_VF_CONTROL)
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugVFControl(
        Platform_TestVFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif defined(USER_MOTOR_TEST_IF_CONTROL)
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugIFControl(
        Platform_TestIFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif defined(USER_MOTOR_TEST_IF_SMO_OBSERVER)
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugIFSMOObserver(
        Platform_TestIFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif defined(USER_MOTOR_TEST_IF_HFI_OBSERVER)
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugIFHFIObserver(
        Platform_TestIFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif defined(USER_MOTOR_TEST_HFI_OBSERVER)
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugHFIObserver();
    Global_Motor_0.lockDebug();
    return result;
#elif defined(USER_MOTOR_TEST_RL_IDENTIFY)
    return Global_Motor_0.startRLIdentification();
#else
    return Global_Motor_0.start();
#endif
}

void User_Init(void)
{
    Platform_VOFA_Init();

    const Lib_Motor::Result motor_init_result = Platform_Motor_Init();
    if (motor_init_result != Lib_Motor::Result::Ok)
    {
        return;
    }
}

void User_Loop(void)
{
    User_SendVofaTelemetryIfPending();

    uint8_t cmd_snapshot = CMD;
    if (cmd_snapshot == 0)
    {
        return;
    }

    CMD = 0;

    // CMD 调度: 读后即清零, 每条命令仅执行一次
    switch (cmd_snapshot)
    {
        case 1: (void)User_StartMotor();                                      break;
        case 2: (void)Global_Motor_0.stop(Lib_Motor::StopMode::COAST);        break;
        case 3:
            (void)Global_Motor_0.stop(Lib_Motor::StopMode::ELECTRICAL_BRAKE);
            break;
        case 4: (void)Global_Motor_0.clearFault();                            break;
        case 5: (void)Global_Motor_0.emergencyStop();                         break;
        default: break;
    }

    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
