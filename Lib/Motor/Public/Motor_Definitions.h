/*
 * Motor_Definitions.h — 电机库类型词典
 *
 * 定义所有枚举、状态标志、数据结构
 * 本文档被 Config / API / FSM / Manager 共同引用
 * 位于依赖链最底层, 避免循环引用
 *
 * 核心概念:
 *   State (5 种主状态)  — 电机生命周期: INIT→CAL→STOP↔RUN→ERROR
 *   Mode  (15 种模式)   — 运行模式下具体做什么: FOC / VF / IF / Debug
 *   RunPhase (13种子态) — RUN 内部阶段: ALIGN→DRAG→SMO / SENSOR
 *   Fault (8 种故障)   — 位掩码, 可同时存在多个故障
 *
 * 数据出口:
 *   MotorMonitorData (15 字段) — Tier1 生产监控 (仪表盘)
 *   MotorDebugData   (11 字段) — Tier2 调试诊断 (示波器)
 *   RuntimeCtx       (私有)    — Tier3 Lib 内部, 不对外开放
 */

#ifndef LIB_MOTOR_PUBLIC_MOTOR_DEFINITIONS_H
#define LIB_MOTOR_PUBLIC_MOTOR_DEFINITIONS_H

#pragma once

#include <stdint.h>
#include "Motor_Features.h"

namespace Lib_Motor
{

/* ============================================================
 * State — 电机主状态 (FSM 的 5 个节点)
 *
 * 流转:
 *   INIT ──→ ADC_CAL ──→ STOP ↔ RUN
 *                        ↓     ↑
 *                        ↓     ↑(故障)
 *                        └───→ ERROR → clearFault()
 * ============================================================ */
enum class State : uint8_t
{
    UNINITIALIZED = 0, // 尚未完成 init()，或 init() 失败
    INIT          = 1,
    ADC_CAL       = 2,
    STOP          = 3,
    RUN           = 4,
    STOPPING      = 5,
    ERROR         = 6,
    ISR_DIAGNOSTIC = 7, // ISR 负载诊断状态: 不启动电机, 仅用于确认快中断是否饿住主循环
};

/* ============================================================
 * 子 FSM 状态枚举 — 由 Safety/Command/Task/Angle/Stop/Energy 六个子 FSM 使用
 *
 * 每个子 FSM 独立管理一个维度, Motor_FSM::step() 按序编排:
 *   SafetyFSM → CommandFSM → TaskFSM → AngleFSM → StopFSM → EnergyFSM
 * ============================================================ */

/*
 * SafetyState — 安全 FSM (SafetyFSM) 状态
 *
 * CLEAR         : 无故障, 所有保护正常工作
 * FAULT_LATCHED : 出现可恢复故障, 故障被锁存, 需 clearFault() 清除后恢复
 * ESTOP_LATCHED : 急停锁存, 最高优先级, 需手动确认后才能恢复
 */
enum class SafetyState : uint8_t
{
    CLEAR = 0,
    FAULT_LATCHED,
    ESTOP_LATCHED,
};

/*
 * CommandState — 命令 FSM (CommandFSM) 状态
 *
 * IDLE            : 空闲, 等待用户命令
 * START_REQUESTED : 用户已调用 start(), 等待 FSM 条件满足后进入 ACTIVE
 * ACTIVE          : 正常运行, 监听 stop 命令
 * STOP_REQUESTED  : 用户已调用 stop(), 等待 FSM 停止流程完成后回到 IDLE
 * FAULTED         : 故障触发, 锁定命令直到故障被清除
 */
enum class CommandState : uint8_t
{
    IDLE = 0,
    START_REQUESTED,
    ACTIVE,
    STOP_REQUESTED,
    FAULTED,
};

/*
 * StreamState — setpoint 流临停状态 (原 TaskState, 语义改为流管理)
 *
 * lib 不再持有"任务"概念; writeSetpoint 每拍覆盖, 无完成判据。
 * StreamState 用于描述"setpoint 流"在 fault/stop 等异常下的临停状态,
 * 便于上层故障恢复后查询"流被冻结在何种情形"。
 *
 * NONE                : 上层从未写入过 setpoint
 * ACTIVE              : 上层正在每拍 writeSetpoint 跟随中
 * SUSPENDED_BY_FAULT  : 故障打断, 上一拍 reference 已冻结
 * SUSPENDED_BY_USER   : stop()/emergencyStop 主动冻结流
 * HELD               : 上层断流超时, 自动退到位置保持 (hold_when_idle)
 * FAULTED_LATCHED     : 旧任务语义残留转义的终态保留值, 与 SUSPENDED_BY_FAULT 等价
 */
enum class StreamState : uint8_t
{
    NONE = 0,
    ACTIVE,
    SUSPENDED_BY_FAULT,
    SUSPENDED_BY_USER,
    HELD,
    FAULTED_LATCHED,   // 兼容旧 SafetyFSM 锁存路径, 实质同 SUSPENDED_BY_FAULT
};

/*
 * AngleState — 角度 FSM (AngleFSM) 状态
 *
 * NONE        : 角度未初始化或不可用
 * SENSOR      : 使用外部传感器角度 (编码器/霍尔)
 * SMO         : 使用滑模观测器估算角度
 * HFI         : 使用高频注入估算角度 (零/低速)
 * FUSION      : 多源角度融合 (传感器 + 观测器平滑切换)
 * UNAVAILABLE : 所有角度源均失效
 */
enum class AngleState : uint8_t
{
    NONE = 0,
    SENSOR,
    SMO,
    HFI,
    FUSION,
    UNAVAILABLE,
};

/*
 * StopState — 停止 FSM (StopFSM) 状态
 *
 * IDLE                       : 未执行停止流程
 * COAST                      : 惯性滑行中 (所有 FET 关断)
 * ELECTRICAL_BRAKE           : 电气制动中 (短路制动或再生制动)
 * MECHANICAL_BRAKE_WAIT      : 等待机械抱闸响应时间
 * MECHANICAL_BRAKE_ENGAGED   : 机械抱闸已啮合, 可安全关闭扭矩
 */
enum class StopState : uint8_t
{
    IDLE = 0,
    COAST,
    ELECTRICAL_BRAKE,
    MECHANICAL_BRAKE_WAIT,
    MECHANICAL_BRAKE_ENGAGED,
};

/*
 * EnergyState — 能量 FSM (EnergyFSM) 状态
 *
 * IDLE         : 空闲, 无能量管理动作
 * DRIVE        : 电动模式, 正常供电
 * REGEN_ACTIVE : 再生制动中, 能量回馈母线
 * DUMP_ACTIVE  : 泄放中, 多余能量通过泄放电阻消耗
 * BLOCKED      : 能量阻断, 禁止电流流动 (急停/故障)
 */
enum class EnergyState : uint8_t
{
    IDLE = 0,
    DRIVE,
    REGEN_ACTIVE,
    DUMP_ACTIVE,
    BLOCKED,
};

/* ============================================================
 * Mode — 运行模式 (决定 RUN 状态下执行什么控制策略)
 *
 * 三大类:
 *   [Debug]  DEBUG_xxx  — Debug 构建可用, Release 物理不存在
 *   [Calib]  CALIB_xxx  — 生产校准, Release 也可用
 *   [Closed] TORQUE/VELOCITY/POSITION — 全闭环运行
 * ============================================================ */
enum class Mode : uint8_t
{
    NONE = 0,                // 空闲 (对应 STOP 状态)

    /* --- Debug Only (由 Platform_MotorFeatures 控制 API 是否可调用) --- */
    DEBUG_PWM_MANUAL,        // 手动 PWM: 直接设 CCR, 跳过 FOC
    DEBUG_CURRENT_LOCK,      // 电流闭环: 角度锁 0°, Id/Iq PI 控制
    DEBUG_IF_DRAG,           // I/F 强拖: 角度开环, 电流闭环
    DEBUG_IF_SMO_OBSERVER,
    DEBUG_IF_HFI_OBSERVER,
    DEBUG_HFI_OBSERVER,      // HFI bring-up: injection and demodulation only
    DEBUG_VF_DRAG,           // V/F 强拖: 角度开环, 电压开环

    /* --- 校准 (Release Available) --- */
    CALIB_RL_IDENTIFY,       // RL 参数辨识: 测相电阻/电感
    CALIB_ENCODER_OFFSET,    // 编码器零点搜索

    /* --- 全闭环 --- */
    TORQUE_CONTROL,          // 转矩控制: Iq 闭环, 角度来自观测器
    VELOCITY_CONTROL,        // 速度控制: 速度环 + 电流环双环
    POSITION_CONTROL,        // 位置控制: 位置环 + 速度环 + 电流环
    IMPEDANCE_CONTROL,       // 阻抗控制: 位置作弹簧中心 + 力矩前馈, 用于机器狗/柔顺关节

    /* --- 制动 --- */
    BRAKE_PASSIVE,           // 短路制动: 低侧 MOSFET 全导通
    BRAKE_ACTIVE,            // 主动制动: 反向电流注入

    RESERVED                 // 扩展预留
};

/* ============================================================
 * MotionSetpoint — 单拍运动设定值流
 *
 * lib 唯一的运动目标入参, 由上层每控制周期调用 writeSetpoint() 写入。
 * 语义: 上拍覆盖、无任务、无完成判据; "完成"与否由上层协调器判断。
 *
 * 字段含义随 mode 不同而不同:
 *   TORQUE_CONTROL   : 仅 torque_ff 生效 (作为 Iq 直接给定)
 *   VELOCITY_CONTROL : vel_ff 生效 (作为速度目标 RPM), torque_ff 作为前馈
 *   POSITION_CONTROL : pos_ref 作为位置目标 (rad), vel_ff/torque_ff 作为前馈
 *   IMPEDANCE_CONTROL: pos_ref 作弹簧中心, 力矩 = stiffness*(pos_ref-θ) - damping*ω
 *                      + torque_bias + torque_ff; 输出被 torque_limit 钳位
 *
 * hold_when_idle: 上层断流(若干 tick 无 writeSetpoint)时, lib 退到位置保持
 *                 true  = 用上一拍 pos_ref 冻结为位置目标
 *                 false = 自由滑行 (退到 StopMode::COAST 语义)
 * ============================================================ */
struct MotionSetpoint
{
    Mode  mode             = Mode::NONE;
    float pos_ref          = 0.0f;   // 位置参考 (rad)
    float vel_ff           = 0.0f;   // 速度前馈/目标 (rad/s 或 RPM, 按 mode 解读)
    float torque_ff        = 0.0f;   // 力矩前馈 (A, 等效 Iq)
    float stiffness        = 0.0f;   // IMPEDANCE: 弹簧刚度 N·m/rad
    float damping          = 0.0f;   // IMPEDANCE: 阻尼 N·m/(rad/s)
    float torque_bias      = 0.0f;   // IMPEDANCE: 偏置力矩 (A, 用于重力补偿等)
    float torque_limit     = 0.0f;   // IMPEDANCE: 输出力矩钳位 (A); 0 = 不限
    bool  hold_when_idle   = true;   // 上层停喂时的安全行为
};

/* ============================================================
 * PlaybackTrigger — setpoint 段回放触发时机
 *
 * IMMEDIATE  : 立即开始 tick-by-tick 消费 FIFO (单 MCU 即刻同步)
 * AWAIT_SYNC : 预加载后等通信层 SYNC 信号触发 (多 MCU 分布式同步)
 *
 * BuildCfg 关闭 SETPOINT_FIFO_PLAYBACK 时, append/trigger/abort 返回 NotSupported。
 * BuildCfg 开启后, Motor 只负责本地 FIFO 回放; SYNC 广播和时钟同步由外层实现。
 * ============================================================ */
enum class PlaybackTrigger : uint8_t
{
    IMMEDIATE  = 0,
    AWAIT_SYNC = 1,
};

/* ============================================================
 * StopMode — 电机停止方式
 *
 * COAST              : 惯性滑行, 所有 FET 关断 (Hi-Z), 电机自由转动
 * ELECTRICAL_BRAKE   : 电气制动, 通过短路绕组或反向电流产生制动转矩
 * MECHANICAL_BRAKE    : 机械抱闸, 由 StopFSM 控制抱闸时序 (等待→啮合)
 *
 * 三选一, 由 User 层调用 stop(mode) 指定。
 * EnergyFSM 根据 StopMode 决定能量的泄放/回收/阻断策略。
 * ============================================================ */
enum class StopMode : uint8_t
{
    COAST = 0,
    ELECTRICAL_BRAKE = 1,
    MECHANICAL_BRAKE = 2,
};

/* ============================================================
 * RunPhase — RUN 状态的子阶段
 *
 * 无感启动流程:
 *   ALIGNMENT → FORCE_DRAG → SMO_ONLY (速度过阈值)
 *   SMO_ONLY → FORCE_DRAG               (速度回落, 有迟滞)
 *
 * 有感流程:
 *   ALIGNMENT → SENSOR_ONLY
 *
 * 调试流程:
 *   RUN_DIRECT (跳过所有启动阶段)
 * ============================================================ */
enum class RunPhase : uint8_t
{
    NONE = 0,

    RUN_DIRECT,         // 调试: 跳过启动, 直接运行

    /* --- 无感启动链 --- */
    ALIGNMENT,          // 预对准: 注入固定角度电流, 锁转子到已知位置
    FORCE_DRAG,         // I/F 或 V/F 开环强拖, 角度发生器加速
    SMO_ONLY,           // 滑模观测器闭环 (中高速)
    FLYING_START,       // [P2] 顺逆风启动: phase voltage PLL 重构角度 (LIB_MOTOR_ENABLE_FLYING_START)

    /* --- 有感 --- */
    SENSOR_ONLY,        // 纯编码器/霍尔传感器

    /* --- 无感扩展 --- */
    HFI_ONLY,           // 纯高频注入 (零/低速)
    HFI_TRANS,          // HFI→SMO 渐变切换
    FUSION,             // 多源角度平滑交接阶段 (预留)

    /* --- 高级观测器 (预留) --- */
    BEMF_BASIC,         // 反电动势线性观测
    BEMF_ADVANCED,      // Luenberger / 自适应
    EKF,                // 扩展卡尔曼
    HYBRID,             // 多模型混合

    RESERVED
};

/* ============================================================
 * Fault — 故障掩码 (可同时存在多个故障)
 *
 * 位掩码运算:
 *   fault |= OVCURRENT           // 置位
 *   fault &  OVCURRENT           // 检测
 *   fault = NONE                 // 清除
 * ============================================================ */
enum class Fault : uint16_t
{
    NONE         = 0x0000,  // 无故障
    OVERCURRENT  = 0x0001,  // 过流 (Bit0): max(Ia,Ib,Ic) > limit.max_current_a
    OVERVOLT     = 0x0002,  // 过压 (Bit1): v_bus > limit.over_voltage_v
    STALL        = 0x0004,  // 堵转 (Bit2): 反电动势过低或电流长期饱和
    SENSOR_LOSS  = 0x0008,  // 传感器丢失 (Bit3): 编码器/I2C 通信异常
    UNDERVOLT    = 0x0010,  // 欠压 (Bit4): v_bus < limit.under_voltage_v
    OVERTEMP     = 0x0020,  // 过温 (Bit5): 任一 NTC 超阈值
    PHASE_LOSS   = 0x0040,  // 缺相 (Bit6): 某相电流持续为零
    PARAM_ERROR  = 0x0080,  // 参数错误 (Bit7): 配置非法
    SENSOR_CONFIG_ERROR = 0x0100, // 位置传感器反馈被选择但未正确配置
    OBSERVER_UNAVAILABLE = 0x0200,// 选择的观测器算法未编译或尚未可用
    SENSOR_SIGNAL_ERROR  = 0x0400,// 传感器数据已到达但健康检查失败
    SENSOR_MISMATCH      = 0x0800,// 同轴冗余传感器角度差超限
    OBSERVER_LOSS        = 0x1000,// 当前观测器离开有效工作区且无安全回退源
    STALL_RETRY_EXHAUSTED = 0x2000,// 堵转自动重试次数耗尽, 需人工干预
    EMERGENCY_STOP       = 0x4000,// 用户/上层系统触发急停, 需人工确认后恢复
    OVERSPEED            = 0x8000,// 超速 (Bit15): |ω| > max_speed_rpm × overspeed_factor
};

/* ============================================================
 * 通用返回结果 — API 调用返回值
 * ============================================================ */
enum class MotorConfigFaultDetail : uint16_t
{
    NONE = 0x0000, // 无配置错误

    BUILD_CURRENT_SENSE_MODE_INVALID = 0x0101, // BuildCfg 电流采样模式不是 1/2/3
    BUILD_NTC_SLOTS_INVALID = 0x0102, // BuildCfg NTC 通道数超出 0~4 范围
    BUILD_REDUNDANT_REQUIRES_SENSOR = 0x0103, // BuildCfg 启用冗余传感器但未启用 SENSOR 能力
    BUILD_OUTPUT_REQUIRES_SENSOR = 0x0104, // BuildCfg 启用输出侧传感器但未启用 SENSOR 能力
    BUILD_REGEN_REQUIRES_ENERGY_FSM = 0x0105, // BuildCfg 启用电池回充但未编译制动能量 FSM
    BUILD_RESISTOR_REQUIRES_ENERGY_FSM = 0x0106, // BuildCfg 启用制动电阻但未编译制动能量 FSM
    BUILD_TICK_PROFILING_UNSUPPORTED = 0x0107, // 当前平台不支持内部 tick profiling

    HAL_NULL = 0x0201, // MotorCfg 未绑定 HAL 接口
    POLE_PAIRS_INVALID = 0x0202, // 极对数为 0
    CONTROL_FREQ_INVALID = 0x0203, // 控制频率非有限值或小于等于 0
    CURRENT_SENSE_MODE_INVALID = 0x0204, // MotorCfg 电流采样模式不是有效枚举值
    CURRENT_SENSE_MODE_BUILD_MISMATCH = 0x0205, // MotorCfg 与实际编译命令中的 MOTOR_BUILD_CURRENT_SENSE_MODE 不一致；命令行 -D 会覆盖头文件默认值
    BUS_CURRENT_NOT_BUILT = 0x0206, // MotorCfg 启用母线电流采样但 BuildCfg 未编译
    PHASE_VOLTAGE_NOT_BUILT = 0x0207, // MotorCfg 启用相电压采样但 BuildCfg 未编译
    PHASE_CURRENT_MASK_INVALID = 0x0209, // BSP 上报的相电流有效通道 mask 含非法位
    PHASE_CURRENT_DUAL_MASK_INVALID = 0x020A, // 双电阻模式下 BSP 未声明恰好两个有效相电流通道
    PHASE_CURRENT_TRIPLE_MASK_INVALID = 0x020B, // 三电阻模式下 BSP 未声明 U/V/W 三相全部有效
    PHASE_CURRENT_SINGLE_SHUNT_MASK_INVALID = 0x021B, // 单电阻模式下 BSP 不应声明独立相电流通道
    SINGLE_SHUNT_HAL_INVALID = 0x021C, // 单电阻模式缺少 raw pair 读取或采样 schedule 下发回调
    SINGLE_SHUNT_WINDOW_INVALID = 0x021D, // 单电阻最小采样窗口参数非法或超过 PWM 周期可用范围
    SINGLE_SHUNT_REQUIRES_SHUNT_SENSOR = 0x021E, // 单电阻模式要求电流传感器类型为分流电阻
    SINGLE_SHUNT_REQUIRES_SVPWM = 0x021F, // 单电阻重构当前只支持 SVPWM 调制
    SENSOR_LPF_INVALID = 0x020C, // 采样低通滤波时间常数不是有限值
    SENSOR_LPF_NEGATIVE = 0x020D, // 采样低通滤波时间常数为负数
    VBUS_CONFIG_INVALID = 0x020E, // 母线电压分压或采样参数非法
    PHASE_VOLTAGE_CONFIG_INVALID = 0x020F, // 相电压分压或采样参数非法
    BRAKE_NO_SINK_PATH = 0x0210, // 允许反向 Iq 但没有配置回充、制动电阻或低边制动能量去路
    BRAKE_ENERGY_FSM_NOT_BUILT = 0x0211, // 制动能量 FSM 未编译
    BATTERY_REGEN_NOT_BUILT = 0x0212, // 电池回充路径未编译
    BRAKE_RESISTOR_NOT_BUILT = 0x0213, // 制动电阻路径未编译
    MECHANICAL_BRAKE_NOT_BUILT = 0x0214, // 机械制动能力未编译
    MECHANICAL_BRAKE_HAL_INVALID = 0x0215, // 机械制动 HAL 回调缺失
    BRAKE_CHOPPER_NOT_BUILT = 0x0216, // 制动斩波控制未编译
    BRAKE_CHOPPER_HAL_INVALID = 0x0217, // 制动斩波 HAL 回调缺失
    BRAKE_CHOPPER_REQUIRES_RESISTOR = 0x0218, // 启用制动斩波但未配置制动电阻
    PHYSICAL_PARAM_INVALID = 0x0219, // 电机物理参数来源或固化辨识值非法
    CURRENT_SENSOR_CONFIG_INVALID = 0x021A, // 电流传感器类型、灵敏度、电阻或放大倍数非法

    STARTUP_DEBUG_API_DISABLED = 0x0301, // 启动目标为调试模式但危险测试 API 未编译
    STARTUP_IF_DEBUG_DISABLED = 0x0302, // 启动目标为 IF 调试模式但 IF 启动能力未编译
    STARTUP_SPEED_LOOP_DISABLED = 0x0303, // 启动目标需要速度环但速度控制能力未编译
    STARTUP_SENSOR_TORQUE_UNSUPPORTED = 0x0304, // 有感反馈类型不支持转矩模式
    STARTUP_SENSOR_SPEED_UNSUPPORTED = 0x0305, // 有感反馈类型不支持速度模式
    STARTUP_POSITION_UNSUPPORTED = 0x0306, // 位置模式未编译或反馈源不支持位置控制
    STARTUP_MODE_INVALID = 0x0307, // 启动目标模式非法或未被当前 BuildCfg 支持

    FEEDBACK_REDUNDANT_NOT_BUILT = 0x0401, // 启用冗余传感器但 BuildCfg 未编译
    FEEDBACK_OUTPUT_NOT_BUILT = 0x0402, // 启用输出侧传感器但 BuildCfg 未编译
    FEEDBACK_STEADY_SOURCE_NOT_BUILT = 0x0403, // 稳态角度源对应能力未编译
    FEEDBACK_STARTUP_SOURCE_NOT_BUILT = 0x0404, // 启动角度源对应能力未编译
    FEEDBACK_HEALTH_PARAM_INVALID = 0x0405, // 传感器健康检查参数非法
    FEEDBACK_REDUNDANT_SENSOR_INVALID = 0x0406, // 冗余传感器接口或配置非法
    FEEDBACK_OUTPUT_SENSOR_INVALID = 0x0407, // 输出侧传感器接口或配置非法
    FEEDBACK_REDUNDANT_CHECK_INVALID = 0x0408, // 冗余角度一致性检查参数非法
    FEEDBACK_ROTOR_SENSOR_INVALID = 0x0409, // 主转子传感器接口或配置非法
    FEEDBACK_SENSOR_STARTUP_MISMATCH = 0x040A, // 启动角度源选择与传感器配置不匹配
    FEEDBACK_ROTOR_CAPABILITY_INVALID = 0x040B, // 转子反馈能力声明与控制模式需求不匹配
    FEEDBACK_SENSOR_INVALID_TICKS = 0x040C, // 传感器无效计数阈值非法
    FEEDBACK_SENSOR_FAULT_ACTION_UNSUPPORTED = 0x040D, // 所选传感器故障处理动作当前不支持
    FEEDBACK_SMO_PARAM_INVALID = 0x040E, // SMO 观测器参数非法
    FEEDBACK_IF_PARAM_INVALID = 0x040F, // IF 启动参数非法
    FEEDBACK_HFI_UNAVAILABLE = 0x0410, // HFI 观测器能力未编译或不可用
    FEEDBACK_SENSOR_TO_SMO_UNAVAILABLE = 0x0411, // SENSOR 到 SMO 接管链路未实现或不可用
    FEEDBACK_STEADY_SOURCE_UNAVAILABLE = 0x0412, // 所选稳态角度源当前不可用
    FEEDBACK_STEADY_SOURCE_INVALID = 0x0413, // 稳态角度源枚举非法
    FEEDBACK_HFI_PARAM_INVALID = 0x0414, // HFI 参数非法
    FEEDBACK_SMO_CLOSED_LOOP_DISABLED = 0x0415, // 闭环使用 SMO 但未显式允许 SMO 闭环
    FEEDBACK_FLYING_START_NOT_BUILT = 0x0416, // 顺逆风启动能力未编译
    FEEDBACK_FLYING_START_NEEDS_PHASE_VOLTAGE = 0x0417, // 顺逆风启动需要相电压采样能力
    FEEDBACK_FLYING_START_UNAVAILABLE = 0x0418, // 顺逆风启动执行链路当前不可用
    FEEDBACK_SINGLE_SHUNT_HFI_UNSUPPORTED = 0x041E, // 单电阻采样不支持 HFI 启动、稳态源或调试入口

    /* 0x04xx: IF 启动前转子静止保证机制 */
    IF_STATIONARY_GUARANTEE_REQUIRED        = 0x0419, // IF 启动未配置任何静止保证机制
    IF_STATIONARY_NEEDS_PHASE_VOLTAGE_BUILD = 0x041A, // 选择相电压顺逆风判断但 BuildCfg 或 MotorCfg 未启用相电压
    IF_STATIONARY_NEEDS_LOW_SIDE_BRAKE       = 0x041B, // 选择低边短路制动但制动配置或 HAL 回调不完整
    IF_STATIONARY_EXPLICIT_BYPASS_UNCONFIRMED = 0x041C, // 选择显式绕过静止保证但未设置确认标志
    FEEDBACK_FLYING_START_SENSOR_STARTUP_UNSUPPORTED = 0x041D, // 有感启动不应同时启用相电压顺逆风启动

    /* 0x05xx: 控制环带宽、自动推导和辨识 */
    CURRENT_LOOP_BW_TOO_HIGH          = 0x0501, // 电流环带宽不应高于控制频率的十分之一
    CURRENT_LOOP_BW_REQUIRES_L_KNOWN = 0x0502, // 自动推导电流环 PI 但缺少有效电感
    SPEED_LOOP_AUTOTUNE_NOT_IMPLEMENTED   = 0x0503, // 速度环自动推导尚未实现
    /* Ke 缺省使用 0 表示未知，当前电压限幅不因 Ke 缺失而报错。 */
    RL_IDENTIFY_NOT_BUILT              = 0x0505, // 启动 RL 辨识但 AUTO_IDENTIFY 未编译
    RL_IDENTIFY_FAILED                 = 0x0506, // RL 辨识运行失败
    KE_IDENTIFY_FAILED                 = 0x0507, // Ke 辨识运行失败
    POSITION_LOOP_AUTOTUNE_NOT_IMPLEMENTED = 0x0508, // 位置环自动推导尚未实现
    RL_IDENTIFY_PARAM_INVALID          = 0x0509, // RL 辨识约束参数非法

    /* 0x06xx: 调制方式和控制拓扑 */
    MODULATION_SIXSTEP_REQUIRES_TRAP = 0x0601, // SIXSTEP 调制要求 BuildCfg 启用 TRAP_CONTROL
    CONTROL_TOPOLOGY_NONE_ENABLED    = 0x0602, // FOC 和 TRAP 控制拓扑至少需要启用一个
};

/*
 * MotorConfigFaultDetailToString — 配置错误码转可读字符串
 *
 * 默认开启 (LIB_MOTOR_ENABLE_HUMAN_READABLE_FAULT_DETAIL), 量产可关省 ~1.3KB ROM
 * 关闭时仅返回空串, 上层用裸 detail 值自行查表
 */
const char* MotorConfigFaultDetailToString(MotorConfigFaultDetail detail);

enum class Result : uint8_t
{
    Ok = 0,            // 成功
    Error,             // 通用错误
    InvalidHandle,     // 电机句柄无效 (未初始化或超出范围)
    InvalidState,      // 当前状态不允此操作 (如 RUN 状态调用 init)
    InvalidParam,      // 参数超出范围
    Busy,              // 资源忙 / 当前任务尚未清除，暂不接受新任务型命令
    Timeout,           // 超时 (如校准超时)
    NotSupported       // 功能已声明但当前尚未实现或当前能力不支持
};

/* ============================================================
 * 控制/调制/观测器选择枚举
 * ============================================================ */

enum class ControlMethod : uint8_t
{
    OPENLOOP_VF   = 0,  // V/F 开环电压拖动
    OPENLOOP_IF   = 1,  // I/F 开环电流拖动
    FOC_TORQUE    = 2,  // FOC 转矩控制
    FOC_SPEED     = 3,  // FOC 速度控制
    FOC_POSITION  = 4,  // FOC 位置控制
};

enum class ModulationMethod : uint8_t
{
    SPWM    = 0,    // 正弦 PWM — 电压利用率 1.0 (相对 vbus/2)
    SVPWM   = 1,    // 七段式空间矢量 — 电压利用率 1.1547 (相对 vbus/2)
    THIPWM  = 2,    // 三次谐波注入 — 电压利用率 1.1547 (相对 vbus/2)
    SIXSTEP = 3,    // 六步方波换相 — 线电压 ≈ Vbus (P6 仅占位, BuildCfg 需开 TRAP_CONTROL)
};

/*
 * RotorStationaryGuarantee — 启动前转子静止保证机制 (与启动源解耦, P3.B 重命名)
 *
 * 用途: 防止转子被外力转动时贸然启动导致预对齐失败或拉扯炸管;
 *       不仅适用于 IF 启动, 也适用于 HFI/SMO 任何非 SENSOR 启动
 *       (SENSOR 启动可直接由编码器读到角度, 不需静止保证)
 *
 *   NONE                 : 未配置 (ConfigCheck 拒绝非 SENSOR 启动)
 *   PHASE_VOLTAGE_FLYING : 强制相电压采样, 顺逆风启动 PLL 重构转子位置
 *   LOW_SIDE_BRAKE       : 启动前低边短路制动强制转子静止
 *   EXPLICIT_BYPASS      : 用户明确承诺启动前转子必然静止
 *                          (需 rotor_explicit_bypass_confirmed=true 双确认防误填)
 */
enum class RotorStationaryGuarantee : uint8_t
{
    NONE                 = 0, // 未提供静止保证; 非 SENSOR 启动由 ConfigCheck 拒绝
    PHASE_VOLTAGE_FLYING = 1, // 用相电压反电动势重构旋转中的转子角度
    LOW_SIDE_BRAKE       = 2, // 启动前低边短路制动, 先把转子拉到近似静止
    EXPLICIT_BYPASS      = 3, // 用户明确确认启动前不会被外力拖动
};

/*
 * StartupSource 表示启动/低速建立可用电角度的来源。
 * 若稳态观测器随后失效，FSM 自动回到此来源；IF 没有绝对角度，故只能停机。
 */
enum class StartupSource : uint8_t
{
    SENSOR = 0, // 由转子编码器/霍尔直接启动，也可作为低速回退源
    IF     = 1, // 开环 I/F 强拖启动；稳态观测器失效后停机
    HFI    = 2, // 高频注入低速启动/回退源 (暂未实现)
};

/*
 * SteadyAngleSource 表示电机平稳运行后参与控制的角度来源。
 * 尚未实现的算法可作为配置占位，但 Validator 会拒绝进入运行。
 */
enum class FlyingStartMode : uint8_t
{
    DISABLED       = 0,   // 关闭顺逆风启动: 用于 startup_source==SENSOR 启动, 或用户明确承诺不会被外力转动
    PHASE_VOLTAGE  = 1,   // 相电压反电势 PLL 重构转子角度后接 SMO; 启动 start() 自动检测 ω 是否足够大
    AUTO_DETECT    = 2,   // (P3 占位) 检测 ω 跨阈值后自动决定; P2 仅占位返未实现
};

/*
 * BrakeEnergyPathPolicy -- regen/dump 两条能量承接路径同时可用时的选择策略
 */
enum class BrakeEnergyPathPolicy : uint8_t
{
    REGEN_FIRST       = 0, // 优先回充电池, dump 作为回退路径
    DUMP_FIRST        = 1, // 优先制动电阻耗散, regen 作为回退路径
    DUMP_ON_OVERVOLT  = 2, // 母线达到软上限后优先 dump, 否则优先 regen
};

/*
 * BudgetLossBehavior -- 运行期能量预算 (MotorEnergyBudget) 失效时的回退策略
 *
 *   BLOCK_ON_LOSS     : 失联 → 禁止运行期负 Iq 制动, 直到上层重新注入有效 budget
 *   FALLBACK_TO_STATIC: 失联 → 回退到静态 cfg.has_*_path 与静态限额
 *
 * 旧名字保留为别名, 避免已有平台配置立即失配。
 */
enum class BudgetLossBehavior : uint8_t
{
    BLOCK_ON_LOSS      = 0,
    BRAKE_ON_LOSS      = BLOCK_ON_LOSS,
    FALLBACK_TO_STATIC = 1,
    HOLD_ON_LOSS       = FALLBACK_TO_STATIC,
};

enum class SteadyAngleSource : uint8_t
{
    SENSOR         = 0, // 外部转子传感器
    SMO            = 1, // 滑模/反电动势观测器
    HFI            = 2, // 高频注入持续反馈 (暂未实现)
    NONLINEAR_FLUX = 3, // 非线性磁链观测器 (预留)
    LUENBERGER     = 4, // Luenberger 观测器 (预留)
    EKF            = 5, // 扩展卡尔曼观测器 (预留)
};

enum class HfiInjectionMode : uint8_t
{
    SQUARE_ALTERNATE = 0, // 方波交变注入: 在 d/q 轴交替注入高频方波电压
    SINE_CARRIER     = 1, // 正弦载波注入: 叠加高频正弦载波信号
};

enum class SensorFaultAction : uint8_t
{
    STOP                    = 0, // 当前唯一已接通执行链的策略: 立即停机
    SWITCH_TO_CONVERGED_SMO = 1, // 预留: 主传感器失效后切到已收敛的 SMO
    SWITCH_TO_HFI           = 2, // 预留: 中低速失效后切到 HFI
};

enum class AngleFeedbackClass : uint8_t
{
    UNSPECIFIED       = 0,
    HIGH_RES_ENCODER  = 1, // ABZ/磁编码器/旋变等可用于精细位置闭环的反馈
    HALL_SECTOR       = 2, // 三霍尔扇区级反馈, 可用于粗略有感 FOC/速度环
    SENSORLESS        = 3, // SMO/PLL 等无感观测器
    OPEN_LOOP         = 4, // 开环角度, 仅用于调试/启动/对准
};

/* ============================================================
 * SensorHealth — BSP 层传感器健康状态 (位掩码)
 *
 * 由 BSP 驱动层上报, 库层可在此基础上叠加更多 plausibility 故障。
 *
 * 位定义:
 *   OK                 = 正常工作, 无异常
 *   NOT_CONFIGURED     = 传感器未配置 (配置中未启用)
 *   NOT_READY          = 传感器已配置但尚未准备好 (如: 首次上电未收到有效数据)
 *   BUS_TIMEOUT        = 通信超时 (I2C/SPI 无应答)
 *   CRC_ERROR          = 数据校验错误 (如编码器 CRC)
 *   MAGNET_ERROR       = 磁编码器磁场异常 (磁铁距离/偏移)
 *   ILLEGAL_STATE      = 传感器内部状态机异常
 *   TIMER_NOT_RUNNING  = 定时器未运行 (如 ABZ 定时器)
 *   ANGLE_JUMP         = 角度跳变: 前后两帧角度差超过阈值
 *   STALE_WHILE_MOVING = 数据停滞: 电机转动但角度不更新
 *   REDUNDANT_MISMATCH = 冗余传感器角度偏差超限
 * ============================================================ */
enum class SensorHealth : uint16_t
{
    OK                   = 0x0000, // 正常工作
    NOT_CONFIGURED       = 0x0001, // 未配置
    NOT_READY            = 0x0002, // 未就绪
    BUS_TIMEOUT          = 0x0004, // 总线超时
    CRC_ERROR            = 0x0008, // CRC 校验错误
    MAGNET_ERROR         = 0x0010, // 磁编码器磁场异常
    ILLEGAL_STATE        = 0x0020, // 传感器内部状态异常
    TIMER_NOT_RUNNING    = 0x0040, // 定时器未运行
    ANGLE_JUMP           = 0x0080, // 角度跳变
    STALE_WHILE_MOVING   = 0x0100, // 数据停滞
    REDUNDANT_MISMATCH   = 0x0200, // 冗余传感器失配
};

enum class PidGroup : uint8_t
{
    CurrentD = 0,   // D 轴电流环 PID
    CurrentQ = 1,   // Q 轴电流环 PID
    Speed    = 2,   // 速度环 PID
    Position = 3,   // 位置环 PID
};

/* ============================================================
 * 调试启动分段表
 *
 * 阶段数组由 Platform/User 层持有。调试运行期间数组必须保持有效。
 * phases 指向 volatile 数组，便于在 Ozone 等调试器中修改表值后启动。
 * phase_count 也保留为 volatile，便于逐段放开测试。
 * ============================================================ */
struct MotorVFStartupPhase
{
    float duration_s;       // 阶段持续时间，=0 时按一个控制周期处理
    float final_speed_rpm;  // 本阶段结束时的机械转速目标
    float final_duty;       // 本阶段结束时的相电压占空比幅值，以 0.5 为中心摆动
};

struct MotorVFStartupProfile
{
    const volatile MotorVFStartupPhase* phases; // VF 阶段数组
    volatile uint8_t phase_count;               // 启用的阶段数量，可在调试器中临时修改
};

struct MotorIFStartupPhase
{
    float duration_s;       // 阶段持续时间，=0 时按一个控制周期处理
    float final_speed_rpm;  // 本阶段结束时的机械转速目标
    float final_id_a;       // 本阶段结束时的 d 轴电流目标
    float final_iq_a;       // 本阶段结束时的 q 轴电流目标
};

struct MotorIFStartupProfile
{
    const volatile MotorIFStartupPhase* phases; // IF 阶段数组
    volatile uint8_t phase_count;               // 启用的阶段数量，可在调试器中临时修改
};

/* ============================================================
 * MotorEvent — FSM 运行时状态标志 (位段, 省内存)
 * ============================================================ */
/*
 * MotorEvent — 运行时事件标志 (位段结构体, 共 1 字节)
 *
 * 用于在 FSM 和控制算法之间传递异步事件信号, 避免引入额外状态变量。
 *
 * 位段字段 (各占 1 bit):
 *   observer_converged  — SMO/HFI 观测器收敛, 角度可信
 *   speed_valid         — PLL 速度估算稳定, 可用于速度环
 *   adc_ready           — ADC 注入组本轮转换完成, 触发 FOC 计算
 *   motor_stalled       — 反电动势过低, 电机堵转
 *
 * MotorEvent() 默认构造: 全部清零 (上电/复位时的安全初始态)
 */
struct MotorEvent
{
    uint8_t observer_converged : 1; // 候选: 当前观测器角度可用，经由 MonitorData 查看
    uint8_t speed_valid        : 1; // 速度反馈信号有效 (PLL 稳定)
    uint8_t adc_ready          : 1; // ADC 注入组完成转换 (驱动 FOC)
    uint8_t motor_stalled      : 1; // 堵转标志 (过流电机关断)
    uint8_t budget_expired    : 1; // 上层 setEnergyBudget 已超时 (BRAKE_ON_LOSS 模式触发故障)
    uint8_t rl_identify_done  : 1; // [P2] RL 辨识完成 (rs_ohm_identified/ls_h_identified 已写入)
    uint8_t ke_identify_done  : 1; // [P2] Ke 辨识完成 (ke_v_per_rad_s_identified 已写入, 仅 startKEstimation API)
    uint8_t flying_start_done : 1; // [P2] 顺逆风 PHASE_VOLTAGE 启动完成 (转子角度已重构 seedAngle SMO)

    MotorEvent()
        : observer_converged(0), speed_valid(0)
        , adc_ready(0), motor_stalled(0), budget_expired(0)
        , rl_identify_done(0), ke_identify_done(0), flying_start_done(0)
    {}
};

/* ============================================================
 * MotorMonitorData — Tier1 生产监控 (用户仪表盘)
 *
 * 15 字段, 覆盖最常见的运行指标
 * 用于: 串口日志、仪表盘、VOFA 低速波形
 *
 * 获取方式: Global_Motor_0.getMonitorData(data)
 * ============================================================ */
struct MotorMonitorData
{
    MotorConfigFaultDetail config_fault_detail;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    float v_phase_u;
    float v_phase_v;
    float v_phase_w;
#endif

    /* Group 1: 仪表盘 */
    State   state;             // 当前主状态 (INIT/STOP/RUN/ERROR)
    Fault   fault_code;        // 故障码 (运行检查)
    float   v_bus;             // 母线电压 (V)
    float   speed_rpm;         // 当前转速 (RPM)
    float   speed_rpm_observer; // Sensorless observer RPM
    float   speed_rpm_sensor;   // Primary position sensor RPM
#if MOTOR_BUILD_NTC_SLOTS > 0
    float   temperature[MOTOR_BUILD_NTC_SLOTS];   // NTC temperatures, index 0..MOTOR_BUILD_NTC_SLOTS-1
#endif

    /* Group 2: FSM 内部 */
    Mode       mode;           // 当前工作模式
    Mode       target_mode;    // 用户请求的目标模式
    RunPhase   run_phase;      // 启动子阶段
    MotorEvent event;          // 事件标志
    EnergyState energy_state;  // 能量 FSM 当前状态 (IDLE/DRIVE/REGEN_ACTIVE/DUMP_ACTIVE/BLOCKED)
#if LIB_MOTOR_ENABLE_STALL_PROTECTION
    uint32_t   stall_total_count; // 上电/复位后累计确认堵转次数
#endif

    /* Group 3: 示波器 */
    float i_a, i_b, i_c;       // 三相电流 (A)
#if MOTOR_BUILD_HAS_BUS_CURRENT
    float i_bus;               // 母线电流 (A)
#endif
    float i_d, i_q;            // DQ 轴电流 (A)
    float angle_elec;          // 实际参与控制的电角度 (rad)
    float angle_elec_command;  // 开环/对准产生的命令电角度 (rad)
    float angle_elec_observer; // 内部稳态观测器输出电角度 (rad)
    float angle_elec_sensor;   // 外部位置传感器换算后的电角度 (rad)
    float angle_mech_sensor;   // 外部位置传感器机械角度 (rad)
    int32_t sensor_raw;        // 外部位置传感器原始计数
    bool sensor_ready;         // 外部位置传感器当前是否有效
    SensorHealth sensor_health;// 主转子传感器健康结果
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    float angle_elec_redundant_sensor; // 同轴冗余传感器电角度 (rad)
    float angle_mech_redundant_sensor; // 同轴冗余传感器机械角度 (rad)
    int32_t redundant_sensor_raw;      // 同轴冗余传感器原始计数
    bool redundant_sensor_ready;       // 同轴冗余传感器是否有效
    SensorHealth redundant_sensor_health;
    float rotor_redundancy_error;     // 两个转子角度源电角度差 (rad)
#endif
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    float angle_mech_output_sensor;   // 关节输出轴传感器机械角度 (rad)
    int32_t output_sensor_raw;        // 输出轴传感器原始计数
    bool output_sensor_ready;         // 输出轴传感器是否有效
    SensorHealth output_sensor_health;
    float transmission_deflection_rad;// 电机侧折算角与输出轴角之差 (rad)
#endif
    float duty_a, duty_b, duty_c; // 占空比 [0,1]
    float observer_error;      // 观测器误差
    float observer_pll_error;
    float observer_signal_level;
    float observer_hfi_demod_alpha;
    float observer_hfi_demod_beta;
    float observer_hfi_demod_d;
    float observer_hfi_demod_q;
    float observer_hfi_raw_demod_alpha;
    float observer_hfi_raw_demod_beta;
    float observer_hfi_meas_2theta;
    float observer_quality;
    uint16_t observer_valid_ticks;
};

/* ============================================================
 * MotorDebugData — Tier2 调试诊断 (开发者专用)
 *
 * 11 字段, 包含 FOC 内部中间量
 * 用于: PID 调参、观测器验证、VOFA 高频波形
 *
 * 获取方式: Global_Motor_0.getDebugData(data)
 * ============================================================ */
struct MotorDebugData
{
    float i_alpha, i_beta;     // Clarke 变换后静止坐标系电流
    float v_d, v_q;            // DQ 轴参考电压 (PID 输出)
    float v_alpha, v_beta;     // InvPark 变换后静止坐标系电压
    float target_id, target_iq;// 用户目标 DQ 电流
    float target_rpm;          // 用户目标转速
    float speed_ref_limited;   // 限幅/斜率后的速度目标
    float speed_pid_iq;        // 速度环输出的 Iq
    float iq_ref_command;      // 模式层生成的 Iq 命令
    float iq_ref_limited;      // 限幅/斜率后的最Iq 目标
    float target_pos_rad;
    float vel_ff_rad_s;
    float torque_ff_a;
    StreamState stream_state;
    uint8_t playback_state;
    uint16_t playback_count;
#if MOTOR_BUILD_NTC_SLOTS > 0
    float temperature_c[MOTOR_BUILD_NTC_SLOTS];    // NTC temperature in Celsius
#endif
#if LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    uint16_t adc_raw_phase_u;
    uint16_t adc_raw_phase_v;
    uint16_t adc_raw_phase_w;
    uint16_t adc_raw_bus_current;
    uint16_t adc_raw_vbus;
#if MOTOR_BUILD_HAS_PHASE_VOLTAGE
    uint16_t adc_raw_phase_voltage_u;
    uint16_t adc_raw_phase_voltage_v;
    uint16_t adc_raw_phase_voltage_w;
#endif
#if MOTOR_BUILD_NTC_SLOTS > 0
    uint16_t adc_raw_temp[MOTOR_BUILD_NTC_SLOTS];
#endif
#endif
};

} // namespace Lib_Motor

#endif // LIB_MOTOR_PUBLIC_MOTOR_DEFINITIONS_H
