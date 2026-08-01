/*
 * Motor_Sensor_Interface.h — 传感器统一接口契约
 *
 * 所有位置传感器 (AS5600/MT6866/霍尔/ABZ/旋变) 都实现此 7 个函数指针
 * Lib 的 Observer_Sensor 通过 SensorInterface_t* 调用, 不感知传感器型号
 *
 * 更换传感器流程:
 *   1. 在 BSP/Sensor/ 下写新驱动 (如 BSP_MT6866/)
 *   2. 在 Platform/Sensor/ 下创建绑定 (如 Platform/MT6866/)
 *   3. 在 Platform_Motor.cpp 中配置: cfg.position.rotor_sensor = Platform_Get_MT6866()
 *   Lib 无需任何改动
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Motor_Definitions.h"

namespace Lib_Motor
{

/* 传感器规格参数 */
struct SensorSpec
{
    int8_t   direction;    // +1: CCW 为正, -1: CW 为正
    uint32_t cpr;          // 每转计数值 (Counts Per Revolution)
    float    offset_elec;  // 电角度零位偏置 (rad)
};

/*
 * SensorInterface_t — 传感器抽象接口
 *
 * 8 个函数指针的分级:
 *   [必须] read_angle, is_ready — 没有这两个不能闭环
 *   [建议] read_velocity, get_raw, get_health — 速度估算与诊断
 *   [可选] init, calibrate — BSP 外部完成的传 NULL
 */
struct SensorInterface_t
{
    /* --- 必须实现 --- */
    float (*read_angle)(void* ctx);      // 机械角度 (rad), [0, 2π)
    bool  (*is_ready)(void* ctx);        // 数据是否就绪 (传感器通信正常)

    /* --- 建议实现 (NULL = 不支持) --- */
    float   (*read_velocity)(void* ctx); // 机械角速度 (rad/s), 传感器硬件计算
    int32_t (*get_raw)(void* ctx);       // 原始数值 (计数/编码值, 诊断校准用)
    /*
     * 设备自身诊断由 BSP 实现并通过此回调报告:
     * I2C/SPI 超时、CRC、磁场状态、霍尔非法码、定时器未启动等。
     * 角度跳变、冗余不一致等跨源合理性检查由 Lib 层处理。
     */
    SensorHealth (*get_health)(void* ctx);// NULL = 库仅依据 is_ready 判断

    /* --- 可选实现 (NULL = 无需操作) --- */
    void (*init)(void* ctx);             // 初始化传感器 (I2C/SPI 配置)
    void (*calibrate)(void* ctx);        // 零位校准 (对齐 Z 脉冲)

    /* --- 私有上下文 + 规格 --- */
    void*      ctx;       // 指向传感器的私有数据结构 (如 AS5600_Ctx_t)
    SensorSpec spec;      // 传感器规格: 方向/CPR/偏置
};

} // namespace Lib_Motor
