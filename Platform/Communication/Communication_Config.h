/*
 * Communication_Config.h — 通信模块编译期开关
 *
 * 取消注释对应宏以启用该通信模块。BSP / Lib / Platform 层通过 include
 * 本文件读取同一份宏定义，保证各层 #ifdef 守卫一致。
 */

#pragma once

// ==================== VOFA 调试串口 ====================
#define BSP_VOFA_ENABLE

// ==================== 预留扩展 ====================
// #define BSP_CAN_ENABLE
// #define BSP_LIN_ENABLE
// #define BSP_UART_COMMAND_ENABLE
