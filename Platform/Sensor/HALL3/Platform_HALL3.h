/*
 * Platform_HALL3.h - 三路霍尔传感器平台层
 *
 * 组装: BSP_HALL3(硬件引脚) + Lib_Sensor HALL3(算法) → Motor SensorInterface_t
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "BSP_HALL3.h"
#include "Platform_HALL3_Config.h"
#include "Motor_Sensor_Interface.h"

#ifdef BSP_HALL3_ENABLE

#ifdef __cplusplus
extern "C" {
#endif

// 平台层可配置参数(运行时可通过ApplyConfig动态修改)
typedef struct {
    int8_t   direction;                // 传感器旋转正方向 (1 或 -1)
    uint8_t  pole_pairs;              // 电机极对数
    float    offset_elec_rad;         // 电气角度偏移量(rad)
    uint8_t  active_low_mask;         // 霍尔低有效掩码(bit0=H1, bit1=H2, bit2=H3)
    uint8_t  state_to_sector[8];      // 8种状态到扇区的映射表
    uint8_t  use_interpolation;       // 是否启用角度插值
    uint32_t stale_timeout_us;        // 信号过期超时(us)
    uint32_t min_edge_interval_us;    // 最小边沿间隔(us)滤除噪声
    float    velocity_filter_alpha;   // 速度低通滤波系数(0~1)
} Platform_HALL3_Config_t;

Lib_Motor::SensorInterface_t* Platform_Get_HALL3(void);  // 获取传感器接口指针
bool Platform_HALL3_IsEnabled(void);       // 模块是否启用

void Platform_HALL3_LoadDefaultConfig(Platform_HALL3_Config_t* cfg);  // 加载默认配置
void Platform_HALL3_ApplyConfig(const Platform_HALL3_Config_t* cfg);  // 应用新配置
void Platform_HALL3_GetConfig(Platform_HALL3_Config_t* cfg);          // 获取当前配置

// 便捷设置单参数接口
void Platform_HALL3_SetDirection(int8_t direction);                    // 设置旋转正方向 (1 或 -1)
void Platform_HALL3_SetPolePairs(uint8_t pole_pairs);                // 设置极对数
void Platform_HALL3_SetElectricalOffset(float offset_elec_rad);      // 设置电气偏移
void Platform_HALL3_SetActiveLowMask(uint8_t active_low_mask);       // 设置低有效掩码
void Platform_HALL3_SetStateToSector(const uint8_t state_to_sector[8]); // 设置扇区映射表

#ifdef __cplusplus
}
#endif

#endif /* BSP_HALL3_ENABLE */
