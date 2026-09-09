/*
 * Motor_LifecycleFSM.h — 电机生命周期 FSM 声明
 *
 * 职责:
 *   管理电机从初始化到故障的完整生命周期状态转换。
 *
 * 状态转换图:
 *
 *   ┌─────────┐
 *   │  INIT   │──→ ADC_CAL (清零校准计数器)
 *   └────┬────┘
 *        ▼
 *   ┌──────────┐
 *   │ ADC_CAL  │──→ STOP (校准采样达标后自动进入)
 *   └────┬─────┘
 *        ▼
 *   ┌─────────┐   run_requested && classifyStartRequest
 *   │  STOP   │───────────────────────────────────→ RUN
 *   └────▲────┘  ←───────────────────────────────────┘
 *        │        shouldStopRun (停止条件满足) 或 故障
 *        │
 *   ┌────┴─────┐
 *   │ STOPPING │  (StopFSM 执行制动, 完成后回到 STOP)
 *   └────┬─────┘
 *        ▼
 *   ┌──────────┐
 *   │  ERROR   │  (等待 clearFault, 回到 STOP)
 *   └──────────┘
 *
 * 内部枚举:
 *   StartRequestKind — 将目标 Mode 分类为启动请求类型:
 *     None:       Mode::NONE (不启动, 取消 run_requested_)
 *     Debug:      DEBUG_PWM / CURRENT_LOCK / IF_DRAG / VF_DRAG
 *     Control:    TORQUE_CONTROL / VELOCITY_CONTROL / POSITION_CONTROL
 *     Calibration: CALIB_RL_IDENTIFY
 *     Invalid:    未实现的模式 → stopForFault(PARAM_ERROR)
 */

#pragma once

namespace Lib_Motor
{

class MotorManager;

class Motor_LifecycleFSM
{
public:
    static void step(MotorManager& m);

private:
    // 启动请求分类: 将目标 Mode 映射为启动类型
    enum class StartRequestKind : unsigned char
    {
        None,          // 不启动
        Debug,         // 调试模式 (直接运行, 跳过闭环检查)
        Control,       // 正常控制模式 (需通过 validateClosedLoopEntry)
        Calibration,   // 校准模式 (RL 辨识)
        Invalid,       // 非法模式
    };

    static StartRequestKind classifyStartRequest(MotorManager& m);
    static void enterRun(MotorManager& m, int initial_phase_value);
    static bool shouldStopRun(const MotorManager& m);

    // 各状态处理函数
    static void handleInit(MotorManager& m);
    static void handleAdcCal(MotorManager& m);
    static void handleStop(MotorManager& m);
    static void handleRun(MotorManager& m);
    static void handleStopping(MotorManager& m);
    static void handleError(MotorManager& m);
};

} // namespace Lib_Motor
