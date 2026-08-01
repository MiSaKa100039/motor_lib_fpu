/*
 * HALL3_Driver.h - 三路数字霍尔传感器算法层
 *
 * 纯逻辑实现，零硬件依赖: 不涉及GPIO、定时器、HAL或MCU。
 * 由Platform/BSP层传入原始3bit状态值和时间戳。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Motor_Definitions.h"
#include "Sensor_Config.h"

#ifdef BSP_HALL3_ENABLE

namespace Lib_Sensor
{

constexpr uint8_t HALL3_INVALID_SECTOR = 0xFFU;  // 无效扇区标志(如000或111)
constexpr uint8_t HALL3_STATE_COUNT = 8U;         // 霍尔状态总数(3bit = 8种)

// Driver可配置参数
struct HALL3_Driver_Config
{
    uint8_t  pole_pairs;                              // 电机极对数
    uint8_t  active_low_mask;                         // 低有效掩码(bit0=H1, bit1=H2, bit2=H3)
    uint8_t  state_to_sector[HALL3_STATE_COUNT];      // 8种状态到6扇区的映射表
    uint8_t  use_interpolation;                       // 是否启用角度插值
    uint32_t stale_timeout_us;                        // 信号过期超时(us)
    uint32_t min_edge_interval_us;                    // 最小边沿间隔(us)滤除噪声
    float    velocity_filter_alpha;                   // 速度低通滤波系数(0~1)
};

// Driver运行时上下文
struct HALL3_Driver_Ctx
{
    HALL3_Driver_Config config;         // 当前配置
    uint8_t  raw_state;                // 原始霍尔状态(未应用低有效掩码)
    uint8_t  state;                    // 当前有效状态(已应用掩码)
    uint8_t  sector;                   // 当前扇区(0~5)
    uint8_t  valid;                    // 有效标志: 0=未就绪, 1=已识别
    int8_t   direction;                // 转动方向: 1=正向, -1=反向, 0=未知
    int32_t  transition_count;         // 扇区跳变累计(含方向)
    uint32_t edge_count;               // 有效边沿总计数
    uint32_t illegal_state_count;      // 非法状态计数
    uint32_t invalid_transition_count; // 非法跳变计数
    uint32_t last_edge_tick;           // 上次边沿时刻
    uint32_t last_interval_ticks;      // 上次正常跳变的间隔节拍数
    float    velocity;                 // 估算速度(rad/s 机械角速度)
    float    last_angle;               // 上一次计算的机械角度
    Lib_Motor::SensorHealth health;    // 传感器健康状态
};

// 算法层对外接口
void HALL3_Driver_DefaultConfig(HALL3_Driver_Config* cfg, uint8_t pole_pairs);  // 生成默认配置
void HALL3_Driver_SetDefault120Map(uint8_t state_to_sector[HALL3_STATE_COUNT]); // 设置120度标准映射
void HALL3_Driver_Init(HALL3_Driver_Ctx* drv, const HALL3_Driver_Config* cfg);  // 初始化
void HALL3_Driver_SetConfig(HALL3_Driver_Ctx* drv, const HALL3_Driver_Config* cfg); // 更新配置
void HALL3_Driver_Update(HALL3_Driver_Ctx* drv,                           // 状态更新(由轮询/中断调用)
                         uint8_t raw_state,
                         uint32_t now_tick,
                         uint32_t ticks_per_second);
float HALL3_Driver_ReadAngle(HALL3_Driver_Ctx* drv,                       // 读取机械角度
                             uint32_t now_tick,
                             uint32_t ticks_per_second);
float HALL3_Driver_ReadVelocity(const HALL3_Driver_Ctx* drv);             // 读取速度
int32_t HALL3_Driver_GetRaw(const HALL3_Driver_Ctx* drv);                 // 获取原始跳变计数
bool HALL3_Driver_IsReady(const HALL3_Driver_Ctx* drv);                    // 传感器是否就绪
Lib_Motor::SensorHealth HALL3_Driver_GetHealth(const HALL3_Driver_Ctx* drv); // 获取健康状态
void HALL3_Driver_Calibrate(HALL3_Driver_Ctx* drv, uint32_t now_tick);     // 校准(以当前位置为零点)

} // namespace Lib_Sensor

#endif /* BSP_HALL3_ENABLE */
