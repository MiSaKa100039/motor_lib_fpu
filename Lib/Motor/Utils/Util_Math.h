/*
 * Util_Math.h — 数学工具函数
 *
 * clamp:      值域限制
 * wrap_angle: 角度归一化到 [0, 2π)
 * angle_diff: 两角最小差 (考虑圆周回绕)
 * deadband:   死区 (|value|<threshold → 0)
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace Lib_Motor
{

inline float clamp(float value, float min_val, float max_val)
{
    return std::max(min_val, std::min(max_val, value));
}

inline float wrap_angle(float angle)
{
    constexpr float TWO_PI = 6.28318530717958647692f;
    while (angle > TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f)   angle += TWO_PI;
    return angle;
}

inline float angle_diff(float a, float b)
{
    constexpr float PI  = 3.14159265358979323846f;
    constexpr float TWO_PI = 6.28318530717958647692f;
    float diff = a - b;
    while (diff > PI)  diff -= TWO_PI;
    while (diff < -PI) diff += TWO_PI;
    return diff;
}

inline float deadband(float value, float threshold)
{
    if (fabsf(value) < threshold) return 0.0f;
    return value;
}

} // namespace Lib_Motor
