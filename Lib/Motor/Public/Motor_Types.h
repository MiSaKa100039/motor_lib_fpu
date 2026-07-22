/*
 * Motor_Types.h — 公共类型定义
 *
 * MotorHandle: 不透明句柄, 用户通过此句柄操作电机
 *   - 底层实现: 静态实例槽位 + MotorManager* 数组索引
 *   - 用户不关心: 只需把 handle 传给 API 函数
 *
 * MOTOR_MAX_INSTANCES: 最大支持电机数
 *   - 静态实例存储在 Motor_API.cpp 的实例池中分配
 *   - 增大此值 → 更多内存 → 可驱动更多电机
 */

#pragma once

#include <stdint.h>
#include "Motor_Features.h"

namespace Lib_Motor
{

typedef void* MotorHandle;          // 不透明句柄 (索引 + 1, 防 NULL)

#define MOTOR_MAX_INSTANCES  LIB_MOTOR_MAX_INSTANCES

#if MOTOR_MAX_INSTANCES < 1
#error "MOTOR_MAX_INSTANCES must be at least 1"
#endif

} // namespace Lib_Motor
