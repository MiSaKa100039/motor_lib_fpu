#pragma once

/*
 * BuildCfg -- 固件编译能力声明
 *
 * 本文件只描述“这份固件编译进了什么能力 / 硬件边界”。
 * MotorCfg (Platform_MotorConfig*.h) 继续描述“当前电机实际怎么配置”。
 *
 * 规则:
 *   1. BuildCfg 负责裁代码、裁成员、限定能力边界。
 *   2. MotorCfg 负责选择本台电机实际使用哪条链路。
 *   3. 若 MotorCfg 选用了 BuildCfg 未启用的能力, 由库内检查拒绝初始化。
 *
 * 启用能力:  #define MOTOR_BUILD_ENABLE_XXX
 * 禁用能力:  // #define MOTOR_BUILD_ENABLE_XXX
 *
 * 能力原子 ↔ 产品组合对照:
 *
 *   产品形态          | Motor 原子能力组合建议
 *   ------------------|------------------------------------------------------------
 *   风扇 / EV 主轴    | TORQUE_CONTROL 或 VELOCITY_CONTROL
 *   3D 打印机 / 贴片机| POSITION_CONTROL + VELOCITY_FEEDFORWARD + SETPOINT_FIFO
 *   CNC               | POSITION_CONTROL + VELOCITY_FEEDFORWARD + SETPOINT_FIFO
 *   机械臂关节        | POSITION_CONTROL + VELOCITY/TORQUE_FEEDFORWARD + HELD
 *   机器狗关节        | POSITION/IMPEDANCE + TORQUE_FEEDFORWARD + HELD
 *   力反馈旋钮        | SENSOR + TORQUE_CONTROL, 或 IMPEDANCE + TORQUE_FEEDFORWARD
 *
 * 上表只是能力组合建议, 不是 Motor 内部产品分支。不要在 Lib/Motor 内增加
 * 面向具体产品形态的裁切宏。
 *
 * 高级应用 API 放在上层独立模块:
 *   - Lib/Motion: G-code、直线/圆弧/S 曲线、lookahead, 输出 MotionSetpoint[]。
 *   - Lib/Haptics: 旋钮卡点、虚拟弹簧、端点、摩擦模型, 输出 torque/impedance setpoint。
 *   - UserAPP/App/Robot: 步态、MPC、腿部运动学, 输出每个关节的 MotionSetpoint。
 *
 * Lib/Motor 只做单轴控制、setpoint 落地、本地 FIFO 回放和监控快照。
 * G-code、轨迹规划、通信协议、ACK、水位、主从同步策略不属于 Lib/Motor。
 */

/* ==================== [1] 实例容量 / 采样硬件 ==================== */
// 固件内最多可创建的电机实例数量; 单电机产品建议固定为 1
#define MOTOR_BUILD_MAX_INSTANCES 1

// 电流采样模式: 2 = 双电阻采样(Iu+Iv, Iw 由 Kirchhoff 计算), 3 = 三电阻采样
// Current sampling mode: 1=single DC-link shunt, 2=dual phase sensors, 3=triple phase sensors.
#ifndef MOTOR_BUILD_CURRENT_SENSE_MODE
#define MOTOR_BUILD_CURRENT_SENSE_MODE 1
#endif

// NTC 温度通道数: 0~4
#define MOTOR_BUILD_NTC_SLOTS 1

// #define MOTOR_BUILD_HAS_BUS_CURRENT  // BUS_CURRENT: 母线电流采样, 用于过流保护/功率估算
// #define MOTOR_BUILD_HAS_PHASE_VOLTAGE  // PHASE_VOLTAGE: 相电压采样, 用于顺逆风启动/无感观测器/调制校正

/* ==================== [2] 数学加速后端 ==================== */
/* 启用 CORDIC 硬件协处理器加速 sin/cos (STM32G4/H5/U5/WL55 等自带 CORDIC 外设)。
 * 未定义时回退到 libm (sinf/cosf), 适用于无 CORDIC 的 MCU (F4/F7/L4/G0/H7 等)。
 * atan2/sqrt 暂统一走 libm, 后续按需扩展。
 * 启用前提: CubeMX 已启用 CORDIC 外设 (MX_CORDIC_Init 会在 HAL_CORDIC_MspInit 中开时钟)。
 * sin/cos 输入范围 [-π, π], 调用方角度需先归一化。 */
// #define MOTOR_BUILD_ENABLE_CORDIC_ACCEL

/* 数值后端选择：目标固件只选择一个后端，避免 fast loop 里出现运行时分支。 */
/* 数值后端选择：默认使用 float32；打开下方宏后使用 fixed Q15。 */
#define MOTOR_BUILD_NUMERIC_BACKEND_FIXED_Q15

/* Ozone 调参门面按场景裁剪。未开启时不编入影子字段，节省 RAM/符号表噪声。 */
// #define MOTOR_BUILD_ENABLE_OZONE_TUNING
// #define MOTOR_BUILD_TUNING_ENABLE_CURRENT_LOOP
// #define MOTOR_BUILD_TUNING_ENABLE_SPEED_LOOP
// #define MOTOR_BUILD_TUNING_ENABLE_LIMITS
// #define MOTOR_BUILD_TUNING_ENABLE_OBSERVER

/* ==================== [3] 控制拓扑能力 (FOC vs 方波/六步换相) ====================
 * 二选一/可共存, 编译期裁切代码:
 *   FOC_CONTROL  默认 ON  — 编译 Park/InvPark/d 轴 PID/SMO/HFI/电压矢量圆限幅/弱磁等 FOC 链路;
 *   TRAP_CONTROL 默认 OFF — 编译六步换相/BEMF 过零检测等方波链路; 启用时裁掉 FOC-only 模块边界最强。
 *
 * 启用条件:
 *   - 至少一个开关必须 ON, 否则 ConfigCheck 报 CONTROL_TOPOLOGY_NONE_ENABLED = 0x0602;
 *   - cfg.control.modulation = SIXSTEP 时必须有 TRAP_CONTROL = ON, 否则报 MODULATION_SIXSTEP_REQUIRES_TRAP = 0x0601;
 *   - 启动期方波切换 (IF + TRAP) 未单独建开关, 全程 FOC 工程仍开 FOC_CONTROL 即可。
 */
#define MOTOR_BUILD_ENABLE_FOC_CONTROL
// #define MOTOR_BUILD_ENABLE_TRAP_CONTROL

/* ==================== [4] 角度源 / 观测器 / 启动 ==================== */
// #define MOTOR_BUILD_ENABLE_SENSOR  // SENSOR: 有感角度源(编码器/HALL/ABZ 等), 关闭后不编译有感反馈链
#define MOTOR_BUILD_ENABLE_IF_STARTUP  // IF_STARTUP: I/F 对齐与开环强拖启动, 可与 SMO 组合, HFI/SENSOR 启动固件可关闭
// #define MOTOR_BUILD_ENABLE_SMO  // SMO: 滑模观测器(Sliding Mode Observer), 无感 FOC 核心算法
// #define MOTOR_BUILD_ENABLE_HFI  // HFI: 高频注入法, 当前仅占位; 注入/解调完成前 MotorCfg 选择 HFI 会被拒绝
// #define MOTOR_BUILD_ENABLE_NONLINEAR_FLUX  // NONLINEAR_FLUX: 非线性磁链观测器, 中高速无感方案
// #define MOTOR_BUILD_ENABLE_LUENBERGER  // LUENBERGER: 龙伯格观测器, 线性状态观测器方案
// #define MOTOR_BUILD_ENABLE_EKF  // EKF: 扩展卡尔曼滤波器, 高精度状态估计

/* 参数自动辨识: 启用 CALIB_RL_IDENTIFY 真辨识实现 + MotorAPI::startKEstimation 接口。
 * 量产可关闭裁切阶跃电压测 R/L 算法体 + 状态机分支。
 * 关闭后 startRLIdentification()/startKEstimation() 返 NotSupported。 */
// #define MOTOR_BUILD_ENABLE_AUTO_IDENTIFY

/* 顺逆风启动: 启用相电压反电势 PLL 角度重构 + MotorFlyingStartRoutine。
 * 启动 start() 时若 |ω|>阈值 + observer.flying_start_mode=PHASE_VOLTAGE 自动走顺逆风启动分支。
 * 关闭后 start() 永远走 ALIGNMENT 正常启动, observer.flying_start_mode=PHASE_VOLTAGE 会被 ConfigCheck 拒绝。
 * 启用前提: MOTOR_BUILD_HAS_PHASE_VOLTAGE 已编入。 */
// #define MOTOR_BUILD_ENABLE_FLYING_START

/* ==================== [5] 反馈链扩展 ==================== */
// #define MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR  // REDUNDANT_SENSOR: 冗余转子传感器支持(双编码器切换), 需 SENSOR 主链开启
// #define MOTOR_BUILD_ENABLE_OUTPUT_SENSOR  // OUTPUT_SENSOR: 输出侧传感器 (减速后/负载侧), 需 SENSOR 主链开启

/* ==================== [6] 控制模式能力 ==================== */
/* 决定 Mode::*_CONTROL 是否被 validateModeSelection / MotorFeatureRegistry::supportsMode 接受。
 * 关闭后, selectMode 选此模式会被 NotSupported 拒绝, 对应控制环分支不编译。 */
// #define MOTOR_BUILD_ENABLE_TORQUE_CONTROL            // 转矩模式: 风扇/EV/打印机/机器狗都常备
// #define MOTOR_BUILD_ENABLE_VELOCITY_CONTROL          // 速度模式: 风扇/EV
// #define MOTOR_BUILD_ENABLE_POSITION_CONTROL          // 位置模式: 打印机XY/机械臂/机器狗
// #define MOTOR_BUILD_ENABLE_IMPEDANCE_CONTROL      // 阻抗模式: 机器狗/力反馈旋钮等柔顺执行器

/* ==================== [7] 安全策略 ==================== */
// #define MOTOR_BUILD_ENABLE_STALL_PROTECTION  // 堵转保护与自动重试状态

/* ==================== [8] 制动 / 能量路径 ==================== */
// #define MOTOR_BUILD_ENABLE_BATTERY_REGEN_PATH  // 电池回充路径: 需要外部电源/BMS 提供允许回充能力边界
// #define MOTOR_BUILD_ENABLE_BRAKE_RESISTOR  // 外部制动电阻泄放路径: 需要温度/电流能力边界
// #define MOTOR_BUILD_ENABLE_BRAKE_CHOPPER_CONTROL  // MCU 直接控制制动电阻斩波 MOSFET
// #define MOTOR_BUILD_ENABLE_MECHANICAL_BRAKE  // 外部机械制动执行机构

/* ==================== [9] setpoint / 前馈 / 高级能力 ==================== */
/* 关闭时: MotionSetpoint.vel_ff / torque_ff 字段仍存在但被 lib 忽略
 * (省一次加法 + 一次 if 判定, 也省 RAM 中相应 ctx_ 字段)。 */
// #define MOTOR_BUILD_ENABLE_VELOCITY_FEEDFORWARD      // 速度前馈: 预规划轨迹/机械臂/机器狗可用
// #define MOTOR_BUILD_ENABLE_TORQUE_FEEDFORWARD     // 力矩前馈: 机械臂重力补偿 / 机器狗 MPC / 力反馈旋钮可用

/* 实时 setpoint 流断流安全。
 * 适用: 机器狗/机械臂/MPC/力反馈等实时反应式应用。
 * 启用后: 上层若干 tick 未 writeSetpoint, lib 自动切到 STREAM::HELD 状态,
 *         按上一拍 pos_ref 锁存位置保持 (无线丢包 / 主控短暂抖动场景)。
 * 关闭时: HELD 路径不编译, hold_when_idle 字段被忽略。
 * 注: writeSetpoint 作为核心 API 始终存在; 本开关只控制断流后的安全持仓路径。 */
// #define MOTOR_BUILD_ENABLE_STREAM_HOLD_WHEN_IDLE

/* 预加载 FIFO 同步回放。
 * 适用: 贴片机/CNC/打印机等可预计轨迹。上层 Lib/Motion 或 App 已完成插补后,
 *       将每个控制 tick 的 MotionSetpoint[] 预装进 Motor 本地 FIFO。
 * 开关关闭: appendSetpointSeg / triggerSegPlayback / abortSegPlayback API 仍存在,
 *           但返回 Result::NotSupported, 且 FIFO 成员与消费逻辑不编译。
 * 开关开启: 编译固定容量本地 FIFO v1, trigger 后由 MotorManager::tick() 每拍最多消费一个 setpoint。
 * 注意: MOTOR_BUILD_SETPOINT_FIFO_CAPACITY 只决定本地 FIFO 深度, 不代表功能开启。
 *       G-code、轨迹规划、通信协议、ACK、水位、SYNC 广播和多 MCU 时钟同步策略均在 lib 外层。 */
// #define MOTOR_BUILD_ENABLE_SETPOINT_FIFO_PLAYBACK
#ifndef MOTOR_BUILD_SETPOINT_FIFO_CAPACITY
#define MOTOR_BUILD_SETPOINT_FIFO_CAPACITY 16
#endif

/* ==================== [10] 调试接口 / 诊断 ==================== */
// #define MOTOR_BUILD_ENABLE_DANGEROUS_TEST_API  // 危险测试接口: 强制 PWM 输出、电流注入等, 仅调试阶段使用, 量产固件应关闭
// #define MOTOR_BUILD_ENABLE_TICK_PROFILING  // Tick profiling: 统计每个控制 tick 的核心/ISR 耗时与超预算次数
// #define MOTOR_BUILD_ENABLE_ISR_LOAD_DIAGNOSTIC  // ISR 负载诊断: 进入独立诊断状态, 仅用于确认快中断是否饿住主循环

/* 控制 MotorConfigFaultDetailToString 是否编译到固件:
 *   默认 ON (开发期) — 错误码可一眼翻译成可读字符串;
 *   量产关闭省 ~1.3KB ROM, 应用层用裸 16 位 detail 值自行查表。 */
// #define MOTOR_BUILD_ENABLE_HUMAN_READABLE_FAULT_DETAIL
