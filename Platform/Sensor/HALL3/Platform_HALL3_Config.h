/*
 * Platform_HALL3_Config.h — HALL3 霍尔传感器平台参数配置
 *
 * 硬件引脚接线定义在 BSP/Sensor/BSP_HALL3/BSP_HALL3.cpp 顶部。
 * 本文件配置三路数字霍尔的电平极性、扇区映射、角度/速度估算参数。
 *
 * 参数说明:
 *   POLE_PAIRS           — 电机极对数
 *   OFFSET_ELEC_RAD      — 电角度零位偏移（rad）
 *   ACTIVE_LOW_MASK      — 低有效位掩码: bit0=H1, bit1=H2, bit2=H3
 *   USE_INTERPOLATION     — 是否启用边沿间角度插值
 *   STALE_TIMEOUT_US      — 信号过期超时（无新边沿时将速度归零）
 *   MIN_EDGE_INTERVAL_US  — 最小边沿间隔（滤除高频噪声）
 *   VELOCITY_FILTER_ALPHA — 速度 IIR 低通滤波系数 [0, 1]
 *   SECTOR_xxx            — 标准化霍尔状态 → 扇区号 [0..5] 的查找表
 */

#pragma once

#ifdef BSP_HALL3_ENABLE

#ifndef PLATFORM_HALL3_ENABLE
#define PLATFORM_HALL3_ENABLE BSP_HALL3_ENABLE
#endif

/* ===================== 电机参数 ===================== */

/*
 * 极对数 — 决定 HALL3 内部每机械转的扇区边沿数 (CPR = 6×P)。
 *
 *   此值仅用于 HALL3 驱动层内部计算机械角度和速度；
 *   控制环的电角度换算(Elec=mech×P+offset)由 MotorManager 传入 SensorObserver，
 *   使用 cfg.physical.pole_pairs (Platform_Motor.cpp)。
 *
 *   初始化流程中 Platform_Motor_Init() 会调用 Platform_HALL3_SetPolePairs()
 *   将 cfg.physical.pole_pairs 同步注入，使两者完全一致。
 *   这里的宏仅作为 HALL3 模块独立上电瞬间的默认初始值。
 *
 *   设错后果: 机械角=0~2π 的刻度错误 → 速度估算成倍偏差 →
 *             FOC 电角度与转子实际位置错位，无法闭环。
 *
 *   确定方法: 手转转子一圈，观察 BSP_HALL3_Debug_Sector(0→5) 的完整循环次数；
 *             完整循环 1 次 = 6 个扇区 → 循环 N 次 = N×6 个扇区边沿 = N 极对数。
 *             或用 VF 开环拖动，比较 angle_elec_command 圈数与机械圈数的比值。
 */
#ifndef PLATFORM_HALL3_POLE_PAIRS_DEFAULT
#define PLATFORM_HALL3_POLE_PAIRS_DEFAULT 8U
#endif

/*
 * 电角度零位偏移 (rad) — 矫正霍尔传感器扇区 0 与 A 相绕组磁场零点的安装偏差。
 *
 *   此值最终注入 SensorInterface_t::spec.offset_elec，
 *   在 SensorObserver::update() 中参与电角度计算:
 *       angle_elec = mech_angle × pole_pairs × dir + offset_elec
 *
 *   校准方法: VF 开环拖动，Id=0、Iq=恒小电流(让电机转几圈的电流)，
 *             调节此值(0~2π)，观察 VOFA 上角度曲线是否连续无跳变。
 *             锁轴测试: Id=小电流、Iq=0，若电机抖动或自行转动则偏移不正确。
 *
 *   设错后果: FOC 电流矢量与转子 d/q 轴错角 → 转矩不足、抖动甚至反转。
 */
#ifndef PLATFORM_HALL3_OFFSET_ELEC_RAD
#define PLATFORM_HALL3_OFFSET_ELEC_RAD 0.0f
#endif

/*
 * 传感器旋转正方向 — 定义机械角度递增方向与转子物理旋转方向的关系。
 *
 *   此值注入 SensorInterface_t::spec.direction，在 SensorObserver 中:
 *       angle_elec = mech_angle × pole_pairs × direction + offset_elec
 *
 *    1 (默认): 机械角度递增方向 = 转子物理正转方向
 *   -1:       机械角度递增方向与转子物理正转相反，需取反
 *
 *   验证方法: VF 开环拖动，转子旋转时观察 VOFA angle_mech_sensor 的增减方向。
 *            转子正转(通常 CCW 从轴端看) → angle_mech 应递增(0→2π)。
 *            若 angle_mech 递减，将此值改为 -1。
 *
 *   设错后果: FOC 电角度方向反转 → 电机反方向转动/震荡/无法锁定。
 */
#ifndef PLATFORM_HALL3_DIRECTION
#define PLATFORM_HALL3_DIRECTION 1
#endif

/* ===================== 霍尔信号极性 ===================== */

/*
 * 低有效位掩码 — 哪些霍尔通道在触发时输出低电平。
 *
 *   霍尔为 OC/OD 输出 + 外部上拉时，触发 → 引脚拉到 GND → 低有效。
 *   GPIO 读回的原生值中 bit0=H1, bit1=H2, bit2=H3。
 *   HW XOR 此掩码将原始电平统一归一化为"高有效"后再查扇区表。
 *
 *   例: H1(bit0) + H3(bit2) 低有效 → 0x05U (binary: 0b101)
 *        H2 低有效               → 0x02U
 *        H1+H2+H3 全部低有效    → 0x07U
 *        H1+H2+H3 全部高有效    → 0x00U
 *
 *   设错后果: 归一化后序列与扇区映射表不匹配 → 机械角跳变/反转/速度突变。
 */
#ifndef PLATFORM_HALL3_ACTIVE_LOW_MASK
#define PLATFORM_HALL3_ACTIVE_LOW_MASK 0x05U
#endif

/* ===================== 角度与速度估算 ===================== */

/*
 * 角度插值开关 — 是否在两个霍尔边沿之间线性插值机械角度。
 *
 *   1 (启用): 低速时角度更平滑，VOFA 上 angle_mech 连续不台阶；
 *             额外消耗极少量 CPU (每 tick 一次乘法)。
 *   0 (禁用): 角度仅在扇区边沿跳变 — 低速走台阶，高速(<1000RPM)无感。
 *
 *   极高速场景(>15000RPM)可关闭以省 CPU，但 20kHz 控制环下通常无必要。
 */
#ifndef PLATFORM_HALL3_USE_INTERPOLATION
#define PLATFORM_HALL3_USE_INTERPOLATION 1
#endif

/*
 * 信号过期超时 (us) — 超过此时间未收到霍尔边沿，速度强制归零。
 *
 *   用于检测堵转/断线/传感器失效，避免积分的角度无限漂移。
 *   HALL3 最低速时每机械圈有 6×P 个边沿，设太长则堵转检测迟钝，
 *   设太短则极低速时正常转动也会被判定为过期。
 *
 *   一般设 > 完成一个电周期所需时间:
 *     最低转速(如 60RPM) → 1圈=1s, 1电周期=1/P s → 边沿间隔=1/(6P) s ≈ 20ms(P=8)
 *     建议 100ms 及以上。
 */
#ifndef PLATFORM_HALL3_STALE_TIMEOUT_US
#define PLATFORM_HALL3_STALE_TIMEOUT_US 100000U
#endif

/*
 * 最小边沿间隔 (us) — 小于此间隔的连续边沿视为噪声并丢弃。
 *
 *   滤除触点抖动或 EMI 导致的假换相脉冲。
 *   基于最高期望转速计算: 最高边沿频率 = RPM × P / 60 × 6。
 *   设太大 → 高速时真边沿被丢弃，角度丢失。
 *   设太小 → 噪声边沿不滤除，角度/速度跳变。
 *
 *   当前 20us = 50MHz 等效边沿频率，对 <10000RPM 足够滤除伪脉冲。
 */
#ifndef PLATFORM_HALL3_MIN_EDGE_INTERVAL_US
#define PLATFORM_HALL3_MIN_EDGE_INTERVAL_US 20U
#endif

/*
 * 速度 IIR 低通滤波系数 — 取值范围 (0, 1]。
 *
 *   speed_filtered = alpha × speed_raw + (1 - alpha) × speed_prev
 *
 *   越大: 越信任当前实时速度，响应快但噪声大 (高速时可加大)。
 *   越小: 越平滑但滞后严重，速度变化时响应慢 (低速时可用小值)。
 *
 *   通常从 0.2~0.5 开始调试，如果 VOFA 速度曲线抖动就减小，
 *   如果速度响应明显滞后于实际转速变化就增大。
 */
#ifndef PLATFORM_HALL3_VELOCITY_FILTER_ALPHA
#define PLATFORM_HALL3_VELOCITY_FILTER_ALPHA 0.35f
#endif

/* ===================== 扇区映射表 ===================== */
/*
 * 标准 120° 霍尔序列（高有效归一化后）:
 *   001 → 101 → 100 → 110 → 010 → 011 → 001 ...
 *
 * 宏名格式: PLATFORM_HALL3_SECTOR_<H3H2H1>
 *   值 = 电周期内扇区索引 [0..5], 非法状态填 PLATFORM_HALL3_SECTOR_INVALID
 *
 * 实际硬件电平极性不同时，只改 ACTIVE_LOW_MASK 即可，
 * 表本身无需改动。
 */
#ifndef PLATFORM_HALL3_SECTOR_INVALID
#define PLATFORM_HALL3_SECTOR_INVALID 0xFFU
#endif
#ifndef PLATFORM_HALL3_SECTOR_000
#define PLATFORM_HALL3_SECTOR_000 PLATFORM_HALL3_SECTOR_INVALID
#endif
#ifndef PLATFORM_HALL3_SECTOR_001
#define PLATFORM_HALL3_SECTOR_001 0U
#endif
#ifndef PLATFORM_HALL3_SECTOR_010
#define PLATFORM_HALL3_SECTOR_010 4U
#endif
#ifndef PLATFORM_HALL3_SECTOR_011
#define PLATFORM_HALL3_SECTOR_011 5U
#endif
#ifndef PLATFORM_HALL3_SECTOR_100
#define PLATFORM_HALL3_SECTOR_100 2U
#endif
#ifndef PLATFORM_HALL3_SECTOR_101
#define PLATFORM_HALL3_SECTOR_101 1U
#endif
#ifndef PLATFORM_HALL3_SECTOR_110
#define PLATFORM_HALL3_SECTOR_110 3U
#endif
#ifndef PLATFORM_HALL3_SECTOR_111
#define PLATFORM_HALL3_SECTOR_111 PLATFORM_HALL3_SECTOR_INVALID
#endif

#endif /* BSP_HALL3_ENABLE */
