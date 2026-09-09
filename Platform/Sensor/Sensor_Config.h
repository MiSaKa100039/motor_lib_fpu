/*
 * Sensor_Config.h — 传感器模块编译期开关
 *
 * 取消注释对应的宏以启用该传感器。所有 BSP / Lib / Platform 层通过 include 此文件
 * 读取同一份宏定义，确保各层 #ifdef 守卫的一致性，而不产生层间依赖。
 *
 * 新增传感器类型时在此追加对应的 BSP_xxx_ENABLE 即可。
 */

#pragma once

// ==================== 霍尔传感器 ====================
// #define BSP_HALL3_ENABLE

// ==================== 编码器 ====================
// #define BSP_ABZ_ENABLE

// ==================== 磁编码器 ====================
// #define BSP_AS5600_ENABLE

// ==================== 预留扩展 ====================
// #define BSP_GYRO_ENABLE
// #define BSP_MAGNETOMETER_ENABLE
// #define BSP_LINEAR_HALL_ENABLE
