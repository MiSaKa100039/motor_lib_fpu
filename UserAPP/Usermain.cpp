#include "Usermain.h"

#include <cmath>
#include "BSP_HALL3.h"
#include "BSP_Motor.h"
#include "Platform_Motor.h"
#include "Platform_VOFA.h"
#include "TestCfg/Platform_MotorTestConfig.h"

#include <stdint.h>

#include "main.h"

// TODO: 外部传感器错误判断
// TODO: ISR 中断时间计算
// TODO: 观测器 / 霍尔冗余
// TODO: 多编码器数据处理
// TODO: 弱磁 park/clark 等幅值/等功率下电流环限流区别
// TODO: 母线泄放 / 动能回收 / 刹车
// TODO: 运行时正反切换
// TODO: 堵转保护以及检测
// TODO: 带载启动？
// TODO: ramp之间的平滑过度
// TODO: debug用例的影子字段调试
// TODO: motorcfg中if启动参数的多余
// TODO: IF对齐爬坡和持续时间
// TODO: 堵转 观测器失效情况下自动重启 对于当前静止的判定？ 是否需要开启三相电压采集进行反电动势检测 配合下管制动？
// TODO: 根据反馈电流来控制最大制动力矩 以及什么时候关闭制动

volatile uint8_t CMD = 0;  // Ozone Watch: CMD
volatile Lib_Motor::Result LastCmdResult = Lib_Motor::Result::Ok; // Ozone Watch: last command result
volatile int32_t MotorTargetSpeedRpm = 400; // Ozone Watch: 用户机械转速目标 (真实 RPM)

#if !LIB_MOTOR_ENABLE_DEBUG_ANY
static int32_t UserLastProcessedTargetSpeedRpm = 0;
static bool UserTargetSpeedWatchInitialized = false;

static Lib_Motor::Result User_ApplyTargetSpeedFromWatch(void)
{
    const int32_t target_rpm = MotorTargetSpeedRpm;
    UserLastProcessedTargetSpeedRpm = target_rpm;
    UserTargetSpeedWatchInitialized = true;
    return Global_Motor_0.setTargetSpeed(static_cast<float>(target_rpm));
}
#endif


static Lib_Motor::Result User_StartMotor(void)
{
#if LIB_MOTOR_ENABLE_DEBUG_PWM_MANUAL
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugPWMManual(
        Platform_TestPWM_U,
        Platform_TestPWM_V,
        Platform_TestPWM_W);
    Global_Motor_0.lockDebug();
    return result;
#elif LIB_MOTOR_ENABLE_DEBUG_CURRENT_LOCK
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugCurrentLock(
        Platform_TestCurrentLockId,
        Platform_TestCurrentLockIq);
    Global_Motor_0.lockDebug();
    return result;
#elif LIB_MOTOR_ENABLE_DEBUG_VF_CONTROL
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugVFControl(
        Platform_TestVFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugIFControl(
        Platform_TestIFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugIFSMOObserver(
        Platform_TestIFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugIFHFIObserver(
        Platform_TestIFStartupProfile);
    Global_Motor_0.lockDebug();
    return result;
#elif LIB_MOTOR_ENABLE_DEBUG_HFI_OBSERVER
    Global_Motor_0.unlockDebug();
    const Lib_Motor::Result result = Global_Motor_0.debugHFIObserver();
    Global_Motor_0.lockDebug();
    return result;
#else
    const Lib_Motor::Result target_result = User_ApplyTargetSpeedFromWatch();
    if (target_result != Lib_Motor::Result::Ok)
    {
        return target_result;
    }
    return Global_Motor_0.start();
#endif
}

static void User_SendVofaTelemetryIfPending(void)
{
    if (!BSP_Motor_TakeTelemetryPending())
    {
        return;
    }
    static constexpr Lib_Motor::MotorTelemetryChannel kVofaChannels[] = {
        // Lib_Motor::MotorTelemetryChannel::SpeedRpm,
        // Lib_Motor::MotorTelemetryChannel::ObserverSignalLevel,
        // Lib_Motor::MotorTelemetryChannel::ObserverPllError,
        // Lib_Motor::MotorTelemetryChannel::ObserverValidTicks,

        // 正式交接测试：RunPhaseState 应依次经过 FORCE_DRAG(3)、FUSION(9)、SMO_ONLY(4)。
        Lib_Motor::MotorTelemetryChannel::SpeedRpmObserver,
        Lib_Motor::MotorTelemetryChannel::PhaseCurrentA,
        Lib_Motor::MotorTelemetryChannel::PhaseCurrentB,
        Lib_Motor::MotorTelemetryChannel::BusVoltage,
        Lib_Motor::MotorTelemetryChannel::Temperature0,


        // Lib_Motor::MotorTelemetryChannel::SpeedRpmObserver,
        // Lib_Motor::MotorTelemetryChannel::SpeedRpm,
        // Lib_Motor::MotorTelemetryChannel::AngleCommand,
        // Lib_Motor::MotorTelemetryChannel::AngleElecObserver,
        // Lib_Motor::MotorTelemetryChannel::RunPhaseState,
    };
    static constexpr uint8_t kVofaChannelCount =
        static_cast<uint8_t>(sizeof(kVofaChannels) / sizeof(kVofaChannels[0]));

    float data[kVofaChannelCount] = {};
    const uint8_t count =
        Global_Motor_0.getTelemetryFloats(kVofaChannels, kVofaChannelCount, data);
    if (count > 0U)
    {
        Platform_VOFA_SendFloat(data, count);
    }
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
    if (cmd_snapshot != 0)
    {
        CMD = 0;

        // CMD 调度: 读后即清零, 每条命令仅执行一次
        switch (cmd_snapshot)
        {
            case 1: LastCmdResult = User_StartMotor();                            break;
            case 2: LastCmdResult = Global_Motor_0.stop(Lib_Motor::StopMode::COAST); break;
            case 3: LastCmdResult = Global_Motor_0.stop(Lib_Motor::StopMode::ELECTRICAL_BRAKE);break;
            case 4: LastCmdResult = Platform_Motor_ResetRuntimeAndSampling();      break;
            default: LastCmdResult = Lib_Motor::Result::InvalidParam;             break;
        }
        return;
    }

#if !LIB_MOTOR_ENABLE_DEBUG_ANY
    /* Watch 变量只在变化后下发一次；Q15 换算位于慢速主循环，不进入控制 ISR。 */
    if (Global_Motor_0.getState() == Lib_Motor::State::RUN)
    {
        const int32_t target_rpm = MotorTargetSpeedRpm;
        if (!UserTargetSpeedWatchInitialized ||
            target_rpm != UserLastProcessedTargetSpeedRpm)
        {
            LastCmdResult = User_ApplyTargetSpeedFromWatch();
        }
    }
#endif

    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
